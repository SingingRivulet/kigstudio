#include "test_common.h"

#include <cJSON.h>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <set>
#include <vector>

#include "kigstudio/sdf/sdf_chunked.h"
#include "kigstudio/sdf/sdf_shape.h"
#include "kigstudio/voxel/voxel2mesh.h"

using namespace sinriv::kigstudio;
using namespace sinriv::kigstudio::sdf;
using namespace sinriv::kigstudio::voxel;

namespace {

using ChunkMeshes =
    std::unordered_map<uint64_t, std::vector<std::tuple<Triangle, vec3f>>>;

void fill_box(VoxelGrid& grid, int min_v, int max_v) {
    for (int x = min_v; x <= max_v; ++x) {
        for (int y = min_v; y <= max_v; ++y) {
            for (int z = min_v; z <= max_v; ++z) {
                grid.insert(Vec3i(x, y, z));
            }
        }
    }
}

ChunkMeshes generate_all_chunked(VoxelGrid& grid, const SDFBase* sdf,
                                 int& num_triangles) {
    ChunkMeshes result;
    generateSmoothMeshChunked(
        grid, num_triangles, {}, true, 1, sdf,
        [&](uint64_t key, const std::vector<std::tuple<Triangle, vec3f>>& tris) {
            result[key] = tris;
        });
    return result;
}

}  // namespace

int main() {
    setup_test_environment();

    // ============ 用例 1：fromSDF 烘焙与解析式 SDF 采样一致 ============
    {
        // 球心 (20,20,20) 半径 12，voxel_size=1，global_position=0
        SDF_Sphere sphere(Vec3f(20.f, 20.f, 20.f), 12.f);
        SDFChunkedGrid grid = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));

        // 体素中心处应与解析值（经同样的窄带截断）完全一致
        auto clamped = [](float v) {
            if (v > 8.0f) return kSDFChunkedFar;
            if (v < -8.0f) return -kSDFChunkedFar;
            return v;
        };
        for (int z = -4; z <= 43; z += 7) {
            for (int y = -4; y <= 43; y += 7) {
                for (int x = -4; x <= 43; x += 7) {
                    Vec3f wp = grid.voxelCenterToWorld({x, y, z});
                    float expected = clamped(sphere.get(wp));
                    float actual = grid.getVoxelValue(x, y, z);
                    assert(std::fabs(actual - expected) < 1e-4f &&
                           "voxel center sample should match analytic SDF");
                }
            }
        }

        // 三线性插值点：符号应与解析值一致（半径足够大，插值误差不会翻号，
        // 避开表面附近 0.5 范围）
        for (int z = 0; z <= 40; z += 5) {
            for (int y = 0; y <= 40; y += 5) {
                for (int x = 0; x <= 40; x += 5) {
                    Vec3f wp(x + 0.37f, y + 0.61f, z + 0.23f);
                    float analytic = sphere.get(wp);
                    if (std::fabs(analytic) < 1.0f) continue;
                    float interp = grid.get(wp);
                    assert((analytic < 0) == (interp < 0) &&
                           "interpolated sign should match analytic SDF");
                }
            }
        }

        // 烘焙范围外的远场
        assert(grid.getVoxelValue(1000, 1000, 1000) == kSDFChunkedFar);
        assert(grid.get(Vec3f(1000.f, 1000.f, 1000.f)) == kSDFChunkedFar);

        // 远场 chunk 应被压缩为 Uniform 或不占用 Dense 内存
        size_t dense_count = 0;
        for (const auto& [key, chunk] : grid.chunks) {
            if (chunk.type == SDFChunk::Type::Dense) dense_count++;
        }
        assert(dense_count < grid.chunks.size() &&
               "far-field chunks should stay Uniform");
    }

    // ============ 用例 2：updateRegion 局部更新只影响目标 chunk ============
    {
        SDF_Sphere sphere(Vec3f(20.f, 20.f, 20.f), 12.f);
        SDFChunkedGrid grid = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));

        // 快照一个远离的 chunk 的值
        auto snapshot = grid.getVoxelValue(40, 40, 40);

        // 在 chunk (0,0,0) 内挖掉 [0,5]^3（置为远场 = 移除材料）
        std::vector<uint64_t> affected = grid.updateRegion(
            Vec3i(0, 0, 0), Vec3i(5, 5, 5),
            [](const Vec3f&, float) { return kSDFChunkedFar; });

        assert(affected.size() == 1 &&
               "single-chunk region should affect exactly one chunk");
        assert(affected[0] == packChunkKey(0, 0, 0));

        // 区域内已被修改
        assert(grid.getVoxelValue(2, 2, 2) == kSDFChunkedFar);
        // 区域外同 chunk 未变
        {
            float expected =
                sphere.get(grid.voxelCenterToWorld({10, 10, 10}));
            assert(std::fabs(grid.getVoxelValue(10, 10, 10) - expected) <
                   1e-4f);
        }
        // 远离的 chunk 未变
        assert(grid.getVoxelValue(40, 40, 40) == snapshot);

        // 恒等 func 作用于缺失的远场区域：不产生新 chunk，affected 为空
        size_t chunks_before = grid.chunks.size();
        std::vector<uint64_t> noop = grid.updateRegion(
            Vec3i(500, 500, 500), Vec3i(510, 510, 510),
            [](const Vec3f&, float old) { return old; });
        assert(noop.empty());
        assert(grid.chunks.size() == chunks_before &&
               "identity update on far field must not allocate chunks");
    }

    // ============ 用例 3：Uniform/Dense 提升与 compress 回收 ============
    {
        SDFChunkedGrid grid;

        // 写入一个体素：Uniform -> Dense 提升
        grid.setVoxelValue(3, 4, 5, -2.0f);
        const uint64_t key = packChunkKey(0, 0, 0);
        auto it = grid.chunks.find(key);
        assert(it != grid.chunks.end());
        assert(it->second.type == SDFChunk::Type::Dense);
        assert(it->second.get(3, 4, 5) == -2.0f);
        assert(it->second.get(0, 0, 0) == kSDFChunkedFar);

        // 整个 chunk 写成同值后 compress 回收为 Uniform
        for (int z = 0; z < 32; ++z)
            for (int y = 0; y < 32; ++y)
                for (int x = 0; x < 32; ++x)
                    grid.setVoxelValue(x, y, z, 7.5f);
        assert(it->second.compress());
        assert(it->second.type == SDFChunk::Type::Uniform);
        assert(it->second.uniform_value == 7.5f);
        assert(grid.getVoxelValue(9, 9, 9) == 7.5f);

        // 非同质 chunk compress 不应回收
        grid.setVoxelValue(1, 1, 1, -1.0f);
        assert(!it->second.compress());
    }

    // ============ 用例 4：序列化 round-trip（含类型注册） ============
    {
        SDF_Sphere sphere(Vec3f(20.f, 20.f, 20.f), 12.f);
        SDFChunkedGrid grid = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));

        cJSON* json = grid.toJSON();
        char* text = cJSON_Print(json);
        cJSON_Delete(json);
        cJSON* parsed = cJSON_Parse(text);
        cJSON_free(text);

        auto restored_ptr = sdf_from_json(parsed);
        cJSON_Delete(parsed);
        assert(restored_ptr && "chunked_grid type should be registered");
        auto* restored = dynamic_cast<SDFChunkedGrid*>(restored_ptr.get());
        assert(restored);
        assert(restored->chunks.size() == grid.chunks.size());

        for (int z = -4; z <= 43; z += 9) {
            for (int y = -4; y <= 43; y += 9) {
                for (int x = -4; x <= 43; x += 9) {
                    assert(restored->getVoxelValue(x, y, z) ==
                               grid.getVoxelValue(x, y, z) &&
                           "round-trip should preserve values exactly");
                }
            }
        }
    }

    // ============ 用例 4b：.sdfchk 二进制文件 round-trip ============
    {
        SDF_Sphere sphere(Vec3f(20.f, 20.f, 20.f), 12.f);
        SDFChunkedGrid grid = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));
        grid.global_position = Vec3f(1.5f, -2.0f, 3.25f);
        grid.voxel_size = Vec3f(0.125f, 0.125f, 0.125f);
        // 手动加一个 Uniform chunk，覆盖两种存储状态
        grid.chunks[packChunkKey(7, 7, 7)].type = SDFChunk::Type::Uniform;
        grid.chunks[packChunkKey(7, 7, 7)].uniform_value = -3.5f;

        const auto path =
            std::filesystem::temp_directory_path() / "test_sdfchk_roundtrip.sdfchk";
        std::string error;
        assert(save_chunked_file(path, grid, &error) && error.c_str());

        SDFChunkedGrid restored;
        assert(load_chunked_file(path, restored, &error) && error.c_str());
        std::filesystem::remove(path);

        assert(restored.chunks.size() == grid.chunks.size());
        assert(restored.global_position.x == 1.5f &&
               restored.global_position.y == -2.0f &&
               restored.global_position.z == 3.25f);
        assert(restored.voxel_size.x == 0.125f);
        auto uit = restored.chunks.find(packChunkKey(7, 7, 7));
        assert(uit != restored.chunks.end());
        assert(uit->second.type == SDFChunk::Type::Uniform);
        assert(uit->second.uniform_value == -3.5f);
        for (int z = -4; z <= 43; z += 7) {
            for (int y = -4; y <= 43; y += 7) {
                for (int x = -4; x <= 43; x += 7) {
                    assert(restored.getVoxelValue(x, y, z) ==
                               grid.getVoxelValue(x, y, z) &&
                           "sdfchk round-trip should preserve values exactly");
                }
            }
        }

        // 损坏文件不应崩溃，且不应覆盖调用方数据
        SDFChunkedGrid guard;
        guard.setVoxelValue(0, 0, 0, -1.0f);
        assert(!load_chunked_file(path, guard, &error));  // 文件已删除
        assert(guard.getVoxelValue(0, 0, 0) == -1.0f);
    }

    // ============ 用例 5：与 mesher 联调，局部重 meshing 等价于全量 ============
    {
        VoxelGrid voxels;
        fill_box(voxels, 0, 40);  // 占据 chunk (0,0,0)..(1,1,1)

        SDF_Sphere sphere(Vec3f(20.f, 20.f, 20.f), 15.f);
        SDFChunkedGrid sdf_grid = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-2, -2, -2), Vec3i(42, 42, 42));

        int baseline_tri = 0;
        ChunkMeshes baseline =
            generate_all_chunked(voxels, &sdf_grid, baseline_tri);
        assert(baseline_tri > 0 && "baseline mesh should contain triangles");

        // 挖掉穿过球面的区域 [3,8]×[15,25]×[15,25]（完全位于 chunk (0,0,0) 内，
        // 球心 (20,20,20) 半径 15，点 (5,20,20) 正在表面上）
        sdf_grid.updateRegion(Vec3i(3, 15, 15), Vec3i(8, 25, 25),
                              [](const Vec3f&, float) {
                                  return kSDFChunkedFar;
                              });

        int region_tri = 0;
        ChunkMeshes region = generateSmoothMeshForRegion(
            voxels, Vec3i(3, 15, 15), Vec3i(8, 25, 25), region_tri, 1,
            &sdf_grid, true);

        // 受影响 chunk：膨胀 1 体素后仍在 [0..31] 内 => 仅 chunk (0,0,0)
        assert(region.size() == 1);
        assert(region.find(packChunkKey(0, 0, 0)) != region.end());

        // 局部重建结果 == 修改后全量重建的对应 chunk
        int full_tri = 0;
        ChunkMeshes full = generate_all_chunked(voxels, &sdf_grid, full_tri);
        for (const auto& [key, tris] : region) {
            auto it = full.find(key);
            size_t expected = (it == full.end()) ? 0 : it->second.size();
            assert(tris.size() == expected &&
                   "region mesh should match full rebuild for the chunk");
        }

        // 修改确实反映到 mesh：chunk (0,0,0) 的三角形数应变化
        auto base_it = baseline.find(packChunkKey(0, 0, 0));
        auto full_it = full.find(packChunkKey(0, 0, 0));
        size_t base_n = base_it == baseline.end() ? 0 : base_it->second.size();
        size_t full_n = full_it == full.end() ? 0 : full_it->second.size();
        assert(base_n != full_n && "carving should change the chunk mesh");
    }

    std::cout << "test_chunked_sdf: all cases passed" << std::endl;
    return 0;
}
