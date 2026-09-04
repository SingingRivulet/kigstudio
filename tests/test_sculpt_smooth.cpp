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

    std::cout << "test_sculpt_smooth: all cases passed" << std::endl;
    return 0;
}
