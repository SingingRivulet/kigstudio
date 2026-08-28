#include "test_common.h"

#include <cassert>
#include <iostream>
#include <set>
#include <unordered_map>
#include <vector>

#include "kigstudio/voxel/voxel2mesh.h"

using namespace sinriv::kigstudio;
using namespace sinriv::kigstudio::voxel;

namespace {

using ChunkMeshes =
    std::unordered_map<uint64_t, std::vector<std::tuple<Triangle, vec3f>>>;

// 全量按 chunk 生成（与 RenderVoxel::loadSDFChunked 同一入口）
// subdivisions=1 控制测试开销
ChunkMeshes generate_all_chunked(VoxelGrid& grid, int& num_triangles) {
    ChunkMeshes result;
    generateSmoothMeshChunked(
        grid, num_triangles, {}, true, 1, nullptr,
        [&](uint64_t key, const std::vector<std::tuple<Triangle, vec3f>>& tris) {
            result[key] = tris;
        });
    return result;
}

// 构造实心方块 [min_v, max_v]^3（体素坐标，闭区间）
void fill_box(VoxelGrid& grid, int min_v, int max_v) {
    for (int x = min_v; x <= max_v; ++x) {
        for (int y = min_v; y <= max_v; ++y) {
            for (int z = min_v; z <= max_v; ++z) {
                grid.insert(Vec3i(x, y, z));
            }
        }
    }
}

void remove_box(VoxelGrid& grid, int min_v, int max_v) {
    for (int x = min_v; x <= max_v; ++x) {
        for (int y = min_v; y <= max_v; ++y) {
            for (int z = min_v; z <= max_v; ++z) {
                grid.remove(Vec3i(x, y, z));
            }
        }
    }
}

}  // namespace

int main() {
    setup_test_environment();

    // ============ 用例 1：跨 chunk 方块，局部==全量等价 ============
    {
        VoxelGrid grid;
        fill_box(grid, 0, 40);  // 占据 chunk (0,0,0)..(1,1,1)

        int baseline_tri = 0;
        ChunkMeshes baseline = generate_all_chunked(grid, baseline_tri);
        assert(baseline_tri > 0 && "baseline mesh should contain triangles");
        assert(!baseline.empty());

        // 删除角落 (0..5)^3，只影响 chunk (0,0,0) 及其负方向邻居
        remove_box(grid, 0, 5);

        int region_tri = 0;
        ChunkMeshes region = generateSmoothMeshForRegion(
            grid, Vec3i(0, 0, 0), Vec3i(5, 5, 5), region_tri, 1, nullptr, true);

        // 受影响 chunk：膨胀后 [-1..6] >> 5 => 每轴 {-1, 0}，共 8 个
        std::set<uint64_t> expected_keys;
        for (int cx = -1; cx <= 0; ++cx) {
            for (int cy = -1; cy <= 0; ++cy) {
                for (int cz = -1; cz <= 0; ++cz) {
                    expected_keys.insert(packChunkKey(cx, cy, cz));
                }
            }
        }
        assert(region.size() == expected_keys.size() &&
               "region should cover exactly the 8 affected chunks");
        for (const auto& [key, tris] : region) {
            assert(expected_keys.count(key) && "unexpected chunk in region");
        }
        // 远离的 chunk 不受影响
        assert(region.find(packChunkKey(1, 0, 0)) == region.end());
        assert(region.find(packChunkKey(1, 1, 1)) == region.end());

        // 局部重建结果 == 修改后全量重建的对应 chunk
        int full_tri = 0;
        ChunkMeshes full = generate_all_chunked(grid, full_tri);
        for (const auto& [key, tris] : region) {
            auto it = full.find(key);
            size_t expected = (it == full.end()) ? 0 : it->second.size();
            assert(tris.size() == expected &&
                   "region mesh should match full rebuild for the chunk");
        }

        // 未受影响的 chunk 的 mesh 应与基准一致（抽查 (1,0,0)）
        {
            uint64_t key = packChunkKey(1, 0, 0);
            auto b = baseline.find(key);
            auto f = full.find(key);
            size_t bn = (b == baseline.end()) ? 0 : b->second.size();
            size_t fn = (f == full.end()) ? 0 : f->second.size();
            assert(bn == fn && "untouched chunk mesh should be unchanged");
        }

        std::cout << "case 1 passed: region chunks=" << region.size()
                  << ", region_tri=" << region_tri << std::endl;
    }

    // ============ 用例 2：全实心 / 全空 chunk 产出空 mesh ============
    {
        VoxelGrid grid;
        // chunk (2,2,2) 完全被方块包围（方块覆盖其 SDF 采样域 63..97）
        fill_box(grid, 32, 127);

        // 全实心内部区域（chunk (2,2,2) 采样域内无符号变化）
        int solid_tri = 0;
        ChunkMeshes solid = generateSmoothMeshForRegion(
            grid, Vec3i(70, 70, 70), Vec3i(80, 80, 80), solid_tri, 1, nullptr,
            true);
        assert(solid.size() == 1);
        assert(solid.begin()->second.empty() &&
               "fully solid chunk should produce empty mesh");
        assert(solid_tri == 0);

        // 全空区域（方块外的空气中，相关 chunk 采样域不触及方块）
        int air_tri = 0;
        ChunkMeshes air = generateSmoothMeshForRegion(
            grid, Vec3i(0, 0, 0), Vec3i(5, 5, 5), air_tri, 1, nullptr, true);
        for (const auto& [key, tris] : air) {
            (void)key;
            assert(tris.empty() && "air chunk should produce empty mesh");
        }
        assert(air_tri == 0);

        std::cout << "case 2 passed" << std::endl;
    }

    std::cout << "Test Passed!" << std::endl;
    return 0;
}
