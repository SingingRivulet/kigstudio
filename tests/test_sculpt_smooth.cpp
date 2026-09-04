#include "test_common.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "kigstudio/sdf/sdf_chunked.h"
#include "kigstudio/sdf/sdf_shape.h"
#include "kigstudio/voxel/voxel.h"

using namespace sinriv::kigstudio;
using namespace sinriv::kigstudio::sdf;

namespace {

// 带高频扰动的球面 SDF（模拟粗糙表面）
struct NoisySphere : public SDFBase {
    Vec3f center;
    float radius;
    NoisySphere(const Vec3f& c, float r) : center(c), radius(r) {}
    float get(const Vec3f& p) const override {
        float d = (p - center).length() - radius;
        // 低频扰动，幅度 0.5，波长 ~8 体素（远大于 27 邻域窗口）
        d += 0.5f * std::sin(p.x * 0.8f) * std::sin(p.y * 0.8f) *
             std::sin(p.z * 0.8f);
        return d;
    }
    std::string getInfo(int indent = 0) const override { return "noisy"; }
    cJSON* toJSON() const override { return nullptr; }
    void fromJSON(const cJSON*) override {}
};

// 区域内表面附近（|sdf| < 2）采样点的波动度量：与 27 邻域均值的平均绝对偏差
float surface_roughness(SDFChunkedGrid& grid, const Vec3i& bmin,
                        const Vec3i& bmax) {
    float sum = 0.f;
    int count = 0;
    for (int z = bmin.z; z <= bmax.z; ++z) {
        for (int y = bmin.y; y <= bmax.y; ++y) {
            for (int x = bmin.x; x <= bmax.x; ++x) {
                float v = grid.getVoxelValue(x, y, z);
                if (std::fabs(v) >= 2.0f) continue;
                float nsum = 0.f;
                for (int dz = -1; dz <= 1; ++dz)
                    for (int dy = -1; dy <= 1; ++dy)
                        for (int dx = -1; dx <= 1; ++dx)
                            nsum += grid.getVoxelValue(x + dx, y + dy, z + dz);
                sum += std::fabs(v - nsum / 27.f);
                ++count;
            }
        }
    }
    return count > 0 ? sum / count : 0.f;
}

}  // namespace

int main() {
    setup_test_environment();

    // ============ 用例 1：平滑降低表面粗糙度，区域外不受影响 ============
    {
        NoisySphere sphere(Vec3f(20.f, 20.f, 20.f), 12.f);
        SDFChunkedGrid grid = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));

        // 测量区域完全落在笔刷范围内（笔刷 x[1,14] y/z[13,26]）
        const Vec3i bmin(4, 16, 16), bmax(12, 24, 24);
        float before = surface_roughness(grid, bmin, bmax);
        assert(before > 0.01f && "noisy sphere should be rough");

        Vec3i rmin, rmax;
        std::vector<uint64_t> affected;
        // 笔刷中心在球面上 (8,20,20)，半径 6 体素（voxel_size=1）
        bool changed = grid.smoothRegion(Vec3f(8.f, 20.f, 20.f), 6.0f, 1.0f,
                                         rmin, rmax, &affected);
        assert(changed);
        assert(!affected.empty());
        assert(rmin.x <= 2 && rmax.x >= 14 && "region should cover brush");

        float after = surface_roughness(grid, bmin, bmax);
        assert(after < before && "smoothing should reduce roughness");

        // 区域外数据不变（如 (40,40,40) 远场、对侧表面 (38,20,20) 附近）
        assert(grid.getVoxelValue(40, 40, 40) >= kSDFChunkedFar);
        float opposite = grid.getVoxelValue(38, 20, 20);
        float expected_opposite = sphere.get(Vec3f(38.5f, 20.5f, 20.5f));
        assert(std::fabs(opposite - expected_opposite) < 1e-4f &&
               "outside brush region should be untouched");

        // 被触碰的 chunk 都在笔刷范围内（chunk (0,0,0)，体素 2..14）
        for (uint64_t key : affected) {
            int cx, cy, cz;
            voxel::unpackChunkKey(key, cx, cy, cz);
            assert(cx == 0 && cy == 0 && cz == 0);
        }
    }

    // ============ 用例 2：强度 0 不变，强度 1 多次迭代单调收敛 ============
    {
        NoisySphere sphere(Vec3f(20.f, 20.f, 20.f), 12.f);
        SDFChunkedGrid grid = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));

        // strength = 0：完全不变
        SDFChunkedGrid copy = grid;
        Vec3i rmin, rmax;
        assert(!grid.smoothRegion(Vec3f(8.f, 20.f, 20.f), 6.0f, 0.0f, rmin,
                                  rmax));

        // 多次 strength=1 迭代：粗糙度单调下降并收敛到球面自身曲率残差附近
        // 测量 box 收缩到笔刷内部（角点距中心 3.46 < 半径 6，falloff 仍有 ~0.38），
        // 避免边缘 falloff→0 的样本拉低收敛速度
        const Vec3i bmin(6, 18, 18), bmax(10, 22, 22);
        // 纯净球面的残差地板（27 邻域均值对曲面本身有系统性偏差）
        SDF_Sphere clean(Vec3f(20.f, 20.f, 20.f), 12.f);
        SDFChunkedGrid clean_grid = SDFChunkedGrid::fromSDF(
            clean, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));
        const float floor_roughness = surface_roughness(clean_grid, bmin, bmax);

        float initial = surface_roughness(copy, bmin, bmax);
        float prev = initial;
        for (int i = 0; i < 8; ++i) {
            copy.smoothRegion(Vec3f(8.f, 20.f, 20.f), 6.0f, 1.0f, rmin, rmax);
            float cur = surface_roughness(copy, bmin, bmax);
            assert(cur <= prev + 1e-6f && "roughness should not increase");
            prev = cur;
        }
        assert(prev < initial * 0.5f && "repeated smoothing should converge");
        assert(prev < floor_roughness * 2.0f + 0.02f &&
               "should approach the clean-sphere roughness floor");
    }

    // ============ 用例 3：chunk 快照恢复（模拟笔画撤销） ============
    {
        NoisySphere sphere(Vec3f(20.f, 20.f, 20.f), 12.f);
        SDFChunkedGrid grid = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));

        // 快照受影响 chunk
        const uint64_t key = voxel::packChunkKey(0, 0, 0);
        SDFChunk snapshot = grid.chunks.at(key);

        Vec3i rmin, rmax;
        grid.smoothRegion(Vec3f(8.f, 20.f, 20.f), 6.0f, 1.0f, rmin, rmax);

        // 恢复快照
        grid.chunks[key] = snapshot;
        for (int z = 0; z < 32; ++z) {
            for (int y = 0; y < 32; ++y) {
                for (int x = 0; x < 32; ++x) {
                    assert(grid.chunks.at(key).get(x, y, z) ==
                               snapshot.get(x, y, z) &&
                           "restored chunk should equal snapshot");
                }
            }
        }
    }

    // ============ 用例 4：纯远场区域不做任何事 ============
    {
        SDF_Sphere sphere(Vec3f(20.f, 20.f, 20.f), 12.f);
        SDFChunkedGrid grid = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));
        size_t chunks_before = grid.chunks.size();

        Vec3i rmin, rmax;
        bool changed = grid.smoothRegion(Vec3f(200.f, 200.f, 200.f), 5.0f,
                                         1.0f, rmin, rmax);
        assert(!changed && "far field smoothing should be a no-op");
        assert(grid.chunks.size() == chunks_before);
    }

    // ============ 用例 5：铲平把表面拉向锁定平面 ============
    {
        NoisySphere sphere(Vec3f(20.f, 20.f, 20.f), 12.f);
        SDFChunkedGrid grid = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));

        // 锁定平面：过笔刷中心 (8,20,20)，法线 +X（d_plane = w.x - 8）
        const Vec3f plane_o(8.f, 20.f, 20.f), plane_n(1.f, 0.f, 0.f);
        // 笔刷核心区（角点距中心 3.46 < 半径 6，falloff >= 0.38）
        const Vec3i bmin(6, 18, 18), bmax(10, 22, 22);
        auto plane_dev = [&]() {
            float sum = 0.f;
            int count = 0;
            for (int z = bmin.z; z <= bmax.z; ++z)
                for (int y = bmin.y; y <= bmax.y; ++y)
                    for (int x = bmin.x; x <= bmax.x; ++x) {
                        float v = grid.getVoxelValue(x, y, z);
                        if (std::fabs(v) >= 2.0f) continue;
                        sum += std::fabs(v - ((x + 0.5f) - 8.f));
                        ++count;
                    }
            return count > 0 ? sum / count : 0.f;
        };
        const float before = plane_dev();
        assert(before > 0.05f && "curved noisy surface should deviate from plane");

        Vec3i rmin, rmax;
        for (int i = 0; i < 8; ++i) {
            bool changed =
                grid.flattenRegion(Vec3f(8.f, 20.f, 20.f), 6.0f, 1.0f, plane_o,
                                   plane_n, rmin, rmax);
            assert(changed);
        }
        const float after = plane_dev();
        assert(after < before * 0.5f && "flatten should converge to the plane");

        // 区域外不受影响
        assert(grid.getVoxelValue(40, 40, 40) >= kSDFChunkedFar);
        const float opposite = grid.getVoxelValue(38, 20, 20);
        const float expected_opposite = sphere.get(Vec3f(38.5f, 20.5f, 20.5f));
        assert(std::fabs(opposite - expected_opposite) < 1e-4f);
    }

    // ============ 用例 6：增减料笔刷（增料外扩 / 减料刻槽 / 空中起料） ============
    {
        SDF_Sphere sphere(Vec3f(20.f, 20.f, 20.f), 12.f);
        SDFChunkedGrid grid = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));

        Vec3i rmin, rmax;
        // 增料：球面外一点 (6,20,20)（体素中心世界 (6.5,20.5,20.5)，
        // 原 SDF≈+1.5 > 0）被推成内部
        assert(grid.getVoxelValue(6, 20, 20) > 0.f);
        bool changed = grid.drawRegion(Vec3f(8.f, 20.f, 20.f), 6.0f, 1.0f,
                                       2.0f, 1.0f, rmin, rmax);
        assert(changed);
        assert(grid.getVoxelValue(6, 20, 20) < 0.f && "add should expand");

        // 减料：在未增料的副本上刻槽，笔刷中心体素 (8,20,20) 变正
        SDFChunkedGrid grid2 = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));
        assert(grid2.getVoxelValue(8, 20, 20) < 0.f);
        changed = grid2.drawRegion(Vec3f(8.f, 20.f, 20.f), 6.0f, 1.0f, 2.0f,
                                   -1.0f, rmin, rmax);
        assert(changed);
        assert(grid2.getVoxelValue(8, 20, 20) > 0.f && "carve should dig");

        // 空中起料：纯远场 dab（cap=radius 兜底）生成新 chunk
        const size_t chunks_before = grid.chunks.size();
        changed = grid.drawRegion(Vec3f(200.f, 200.f, 200.f), 5.0f, 1.0f,
                                  2.0f, 1.0f, rmin, rmax);
        assert(changed);
        assert(grid.chunks.size() > chunks_before);
        assert(grid.getVoxelValue(200, 200, 200) < kSDFChunkedFar);
        // 空中减料仍是 no-op
        const size_t chunks_after_air = grid.chunks.size();
        changed = grid.drawRegion(Vec3f(260.f, 260.f, 260.f), 5.0f, 1.0f,
                                  2.0f, -1.0f, rmin, rmax);
        assert(!changed && "carving empty air should be a no-op");
        assert(grid.chunks.size() == chunks_after_air);
    }

    // ============ 用例 7：膨胀/收缩（平台型 falloff） ============
    {
        SDF_Sphere sphere(Vec3f(20.f, 20.f, 20.f), 12.f);
        SDFChunkedGrid grid = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));

        Vec3i rmin, rmax;
        // 膨胀：核心区内（t<=0.7，falloff=1）均匀外扩 amount
        // 体素 (8,20,20)：连续中心 7.5，dist=0.866，t=0.144 < 0.7 → 全额
        // 球面外点 (6,20,20)（t=0.25 < 0.7 → 平台区）也精确减少 amount
        const float w0 = grid.getVoxelValue(6, 20, 20);
        assert(w0 > 0.f);
        const float v0 = grid.getVoxelValue(8, 20, 20);
        bool changed = grid.inflateRegion(Vec3f(8.f, 20.f, 20.f), 6.0f, 1.0f,
                                          1.0f, 1.0f, rmin, rmax);
        assert(changed);
        const float v1 = grid.getVoxelValue(8, 20, 20);
        assert(std::fabs(v1 - (v0 - 1.0f)) < 1e-4f &&
               "inflate core should shift by exactly amount");
        assert(std::fabs(grid.getVoxelValue(6, 20, 20) - (w0 - 1.0f)) <
                   1e-4f &&
               "plateau falloff should be 1 inside t<=0.7");

        // 收缩：副本上 dir=-1，表面点变正
        SDFChunkedGrid grid2 = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));
        changed = grid2.inflateRegion(Vec3f(8.f, 20.f, 20.f), 6.0f, 1.0f,
                                      1.0f, -1.0f, rmin, rmax);
        assert(changed);
        assert(grid2.getVoxelValue(8, 20, 20) > 0.f && "deflate should dig");
        // 区域外不变
        assert(grid2.getVoxelValue(40, 40, 40) >= kSDFChunkedFar);
        const float opposite = grid2.getVoxelValue(38, 20, 20);
        const float expected_opposite = sphere.get(Vec3f(38.5f, 20.5f, 20.5f));
        assert(std::fabs(opposite - expected_opposite) < 1e-4f);
    }

    // ============ 用例 8：变形（拖动平流） ============
    {
        SDF_Sphere sphere(Vec3f(20.f, 20.f, 20.f), 12.f);
        SDFChunkedGrid grid = SDFChunkedGrid::fromSDF(
            sphere, Vec3i(-4, -4, -4), Vec3i(43, 43, 43));

        Vec3i rmin, rmax;
        // delta≈0：no-op
        assert(!grid.moveRegion(Vec3f(8.f, 20.f, 20.f), 6.0f, 1.0f,
                                Vec3f(0.f, 0.f, 0.f), rmin, rmax));

        // 表面点 (8,20,20)（原 SDF≈-0.48 < 0）沿 +X 拖 2 个体素后变正
        // （表面被推离该点）
        assert(grid.getVoxelValue(8, 20, 20) < 0.f);
        bool changed =
            grid.moveRegion(Vec3f(8.f, 20.f, 20.f), 6.0f, 1.0f,
                            Vec3f(2.f, 0.f, 0.f), rmin, rmax);
        assert(changed);
        assert(grid.getVoxelValue(8, 20, 20) > 0.f &&
               "surface should move away from the dab point");
        // 反侧不受影响
        const float opposite = grid.getVoxelValue(38, 20, 20);
        const float expected_opposite = sphere.get(Vec3f(38.5f, 20.5f, 20.5f));
        assert(std::fabs(opposite - expected_opposite) < 1e-4f);
    }

    // ============ 用例 9：场修复（局部重距离化） ============
    {
        // 平面场 v = (x+0.5) - 8（完美距离场，零交叉在 x=8）
        struct Plane : public SDFBase {
            float get(const Vec3f& p) const override { return p.x - 8.f; }
            std::string getInfo(int indent = 0) const override {
                return "plane";
            }
            cJSON* toJSON() const override { return nullptr; }
            void fromJSON(const cJSON*) override {}
        } plane;
        SDFChunkedGrid grid = SDFChunkedGrid::fromSDF(plane, Vec3i(0, 0, 0),
                                                      Vec3i(31, 31, 31),
                                                      Vec3f(0.f, 0.f, 0.f),
                                                      Vec3f(1.f, 1.f, 1.f),
                                                      20.0f);

        // 把笔刷区域内的值 ×1.5（零交叉不变，距离失真）
        Vec3i rmin, rmax;
        grid.brushRegionAABB(Vec3f(8.f, 16.f, 16.f), 8.0f, rmin, rmax);
        for (int z = rmin.z; z <= rmax.z; ++z)
            for (int y = rmin.y; y <= rmax.y; ++y)
                for (int x = rmin.x; x <= rmax.x; ++x) {
                    const float v = grid.getVoxelValue(x, y, z);
                    if (std::fabs(v) < kSDFChunkedFar) {
                        grid.setVoxelValue(x, y, z, v * 1.5f);
                    }
                }
        // 在中心线 y=z=16, x∈[5,11] 上测最大偏差（该处 falloff >= 0.59，
        // 修复确实写入；区域边缘 falloff→0 不纳入测量）
        auto max_dev = [&]() {
            float dev = 0.f;
            for (int x = 5; x <= 11; ++x) {
                const float v = grid.getVoxelValue(x, 16, 16);
                dev = std::max(dev, std::fabs(v - ((x + 0.5f) - 8.f)));
            }
            return dev;
        };
        const float dev_before = max_dev();
        assert(dev_before > 1.0f && "distorted field should deviate");

        Vec3i omin, omax;
        const bool changed = grid.repairRegion(Vec3f(8.f, 16.f, 16.f), 8.0f,
                                               1.0f, omin, omax);
        assert(changed);
        const float dev_after = max_dev();
        assert(dev_after < dev_before * 0.6f &&
               "repair should reduce distance error");
        // 符号不变
        assert(grid.getVoxelValue(7, 16, 16) < 0.f);
        assert(grid.getVoxelValue(8, 16, 16) > 0.f);
    }

    std::cout << "test_sculpt_smooth: all cases passed" << std::endl;
    return 0;
}
