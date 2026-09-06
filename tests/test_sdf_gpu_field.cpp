// SDF GPU 渲染数据路径的 CPU 仿真测试：
// 复刻 RenderSdfGpu 的 chunk map 编码 + 34^3 ghost brick 组装逻辑，
// 以及 fs_sdf_raymarch 的 sampleSDF 三线性采样与球体追踪，
// 验证编码后的"GPU 采样场"与 SDFChunkedGrid 直接采样场一致、
// 且 marching 命中位置与解析球面吻合（跨 chunk 边界无系统偏差）。
#include "test_common.h"

#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <unordered_map>
#include <vector>

#include "kigstudio/sdf/sdf_chunked.h"
#include "kigstudio/sdf/sdf_shape.h"
#include "kigstudio/voxel/voxel.h"

using namespace sinriv::kigstudio;
using namespace sinriv::kigstudio::sdf;
using namespace sinriv::kigstudio::voxel;

namespace {

constexpr int S = SDFChunk::SIZE;  // 32
constexpr int BS = S + 2;          // 34（含 ghost layer）

// 复刻 RenderSdfGpu 的编码（不含 bgfx 纹理，纯内存）
struct GpuFieldEmu {
    Vec3i cmin{0, 0, 0};
    Vec3i cdim{0, 0, 0};
    Vec3f origin{0.f, 0.f, 0.f};
    float vs = 1.0f;
    std::unordered_map<uint64_t, float> cmap;        // chunk key -> 编码
    std::unordered_map<uint64_t, int> brick_of;      // chunk key -> brick
    std::vector<std::array<float, BS * BS * BS>> bricks;

    static bool chunkNeedsBrick(const SDFChunk& chunk) {
        if (chunk.type == SDFChunk::Type::Dense) return true;
        return chunk.uniform_value >= 0.0f &&
               chunk.uniform_value < kSDFChunkedFar;
    }

    void build(const SDFChunkedGrid& grid) {
        origin = grid.global_position;
        vs = grid.voxel_size.x;
        bool first = true;
        Vec3i cmax;
        for (const auto& [key, chunk] : grid.chunks) {
            int cx, cy, cz;
            unpackChunkKey(key, cx, cy, cz);
            if (first) {
                cmin = cmax = {cx, cy, cz};
                first = false;
            } else {
                cmin.x = std::min(cmin.x, cx);
                cmin.y = std::min(cmin.y, cy);
                cmin.z = std::min(cmin.z, cz);
                cmax.x = std::max(cmax.x, cx);
                cmax.y = std::max(cmax.y, cy);
                cmax.z = std::max(cmax.z, cz);
            }
        }
        cdim = {cmax.x - cmin.x + 1, cmax.y - cmin.y + 1, cmax.z - cmin.z + 1};
        for (const auto& [key, chunk] : grid.chunks) {
            int cx, cy, cz;
            unpackChunkKey(key, cx, cy, cz);
            if (chunk.type == SDFChunk::Type::Uniform) {
                if (chunk.uniform_value >= kSDFChunkedFar) {
                    cmap[key] = 0.0f;
                    continue;
                }
                if (chunk.uniform_value < 0.0f) {
                    cmap[key] = chunk.uniform_value;
                    continue;
                }
            }
            const int brick = static_cast<int>(bricks.size());
            bricks.emplace_back();
            brick_of[key] = brick;
            cmap[key] = static_cast<float>(brick) + 1.0f;
            // 组装 34^3（含 ghost layer + ±band 限幅），与
            // RenderSdfGpu::uploadBrick 一致
            const float cap = 8.0f * vs;
            auto& buf = bricks.back();
            for (int z = -1; z <= S; ++z) {
                for (int y = -1; y <= S; ++y) {
                    for (int x = -1; x <= S; ++x) {
                        float v;
                        if (x >= 0 && x < S && y >= 0 && y < S && z >= 0 &&
                            z < S) {
                            v = chunk.get(x, y, z);
                        } else {
                            const float nv = grid.getVoxelValue(
                                cx * S + x, cy * S + y, cz * S + z);
                            if (std::fabs(nv) >= kSDFChunkedFar) {
                                v = chunk.get(std::min(std::max(x, 0), S - 1),
                                              std::min(std::max(y, 0), S - 1),
                                              std::min(std::max(z, 0), S - 1));
                            } else {
                                v = nv;
                            }
                        }
                        v = std::min(std::max(v, -cap), cap);
                        buf[(static_cast<size_t>(z + 1) * BS + (y + 1)) * BS +
                            (x + 1)] = v;
                    }
                }
            }
        }
    }

    // 复刻 fs_sdf_raymarch 的 sampleSDF（vox = (p-origin)/vs - 0.5）
    float sample(Vec3f p) const {
        const float vv[3] = {(p.x - origin.x) / vs, (p.y - origin.y) / vs,
                             (p.z - origin.z) / vs};
        int cf[3];
        int cm[3];
        for (int a = 0; a < 3; ++a) {
            cf[a] = static_cast<int>(std::floor(vv[a] / 32.0f));
        }
        cm[0] = cf[0] - cmin.x;
        cm[1] = cf[1] - cmin.y;
        cm[2] = cf[2] - cmin.z;
        if (cm[0] < 0 || cm[1] < 0 || cm[2] < 0 || cm[0] > cdim.x - 1 ||
            cm[1] > cdim.y - 1 || cm[2] > cdim.z - 1) {
            return 1.0e6f;
        }
        auto it = cmap.find(packChunkKey(cf[0], cf[1], cf[2]));
        const float code = it == cmap.end() ? 0.0f : it->second;
        if (code == 0.0f) return 1.0e6f;
        if (code < 0.0f) return code;
        const int brick = static_cast<int>(code) - 1;
        const auto& buf = bricks[brick];
        float local[3];
        float tu[3];
        for (int a = 0; a < 3; ++a) {
            local[a] = vv[a] - cf[a] * 32.0f;   // [0,32)
            tu[a] = local[a] + 1.0f;            // texel 坐标
        }
        // 三线性（texel 中心在整数+0.5）
        float result = 0.0f;
        int idx0[3];
        float frac[3];
        for (int a = 0; a < 3; ++a) {
            const float x = tu[a] - 0.5f;
            idx0[a] = static_cast<int>(std::floor(x));
            frac[a] = x - idx0[a];
        }
        for (int dz = 0; dz <= 1; ++dz)
            for (int dy = 0; dy <= 1; ++dy)
                for (int dx = 0; dx <= 1; ++dx) {
                    const float w = (dx ? frac[0] : 1 - frac[0]) *
                                    (dy ? frac[1] : 1 - frac[1]) *
                                    (dz ? frac[2] : 1 - frac[2]);
                    result += w * buf[(static_cast<size_t>(idx0[2] + dz) * BS +
                                       (idx0[1] + dy)) *
                                          BS +
                                      (idx0[0] + dx)];
                }
        return result;
    }
};

// 参考场：直接对 SDFChunkedGrid 做三线性（体素中心采样约定一致）
float sampleRef(const SDFChunkedGrid& grid, Vec3f p) {
    const float vv[3] = {(p.x - grid.global_position.x) / grid.voxel_size.x,
                         (p.y - grid.global_position.y) / grid.voxel_size.y,
                         (p.z - grid.global_position.z) / grid.voxel_size.z};
    int idx0[3];
    float frac[3];
    for (int a = 0; a < 3; ++a) {
        const float x = vv[a] - 0.5f;
        idx0[a] = static_cast<int>(std::floor(x));
        frac[a] = x - idx0[a];
    }
    float result = 0.0f;
    for (int dz = 0; dz <= 1; ++dz)
        for (int dy = 0; dy <= 1; ++dy)
            for (int dx = 0; dx <= 1; ++dx) {
                const float w = (dx ? frac[0] : 1 - frac[0]) *
                                (dy ? frac[1] : 1 - frac[1]) *
                                (dz ? frac[2] : 1 - frac[2]);
                result += w * grid.getVoxelValue(idx0[0] + dx, idx0[1] + dy,
                                                 idx0[2] + dz);
            }
    return result;
}

}  // namespace

int main() {
    setup_test_environment();

    // 球心刻意放在非整数位置并横跨 chunk 边界（x=32 平面）
    SDF_Sphere sphere(Vec3f(31.7f, 19.3f, 33.4f), 12.0f);
    SDFChunkedGrid grid =
        SDFChunkedGrid::fromSDF(sphere, Vec3i(-4, -4, -4), Vec3i(75, 75, 75));

    GpuFieldEmu emu;
    emu.build(grid);

    // ============ 1. 采样一致性：密集 chunk 覆盖区 +1 体素内应精确一致 ====
    {
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> uf(-2.0f, 73.0f);
        float max_diff = 0.0f;
        int tested = 0;
        for (int i = 0; i < 200000; ++i) {
            Vec3f p(uf(rng), uf(rng), uf(rng));
            // 只比较三线性 8 角点都不是远场的点（ghost 钳制区除外）
            bool has_far = false;
            {
                const float vv[3] = {p.x - grid.global_position.x,
                                     p.y - grid.global_position.y,
                                     p.z - grid.global_position.z};
                int idx0[3];
                for (int a = 0; a < 3; ++a)
                    idx0[a] = static_cast<int>(std::floor(vv[a] - 0.5f));
                for (int dz = 0; dz <= 1 && !has_far; ++dz)
                    for (int dy = 0; dy <= 1 && !has_far; ++dy)
                        for (int dx = 0; dx <= 1; ++dx) {
                            if (std::fabs(grid.getVoxelValue(
                                    idx0[0] + dx, idx0[1] + dy,
                                    idx0[2] + dz)) >= kSDFChunkedFar) {
                                has_far = true;
                                break;
                            }
                        }
            }
            if (has_far) continue;
            const float ref = sampleRef(grid, p);
            const float gpu = emu.sample(p);
            if (gpu >= 1.0e6f) continue;  // chunk map 外的点不比较
            const float diff = std::fabs(gpu - ref);
            if (diff > max_diff) max_diff = diff;
            ++tested;
        }
        std::cout << "[sample] tested=" << tested << " max_diff=" << max_diff
                  << std::endl;
        if (tested == 0 || max_diff > 1e-3f) {
            std::cerr << "FAIL: gpu field mismatch, max_diff=" << max_diff
                      << std::endl;
            return 1;
        }
    }

    // ============ 2. 跨 chunk 边界连续性：边界两侧对踩 ===================
    {
        float max_step = 0.0f;
        Vec3f worst_p, worst_q;
        float worst_a = 0, worst_b = 0;
        std::mt19937 rng(7);
        std::uniform_real_distribution<float> uf(4.0f, 60.0f);
        for (int i = 0; i < 50000; ++i) {
            const int axis = i % 3;
            Vec3f p(uf(rng), uf(rng), uf(rng));
            // 恰在 32/64 边界面 ±0.001 处对踩
            const float plane = 32.0f * (1 + (i / 3) % 2);
            auto setAxis = [&](Vec3f& v, float val) {
                if (axis == 0) v.x = val;
                if (axis == 1) v.y = val;
                if (axis == 2) v.z = val;
            };
            setAxis(p, plane + 0.001f);
            Vec3f q = p;
            setAxis(q, plane - 0.001f);
            const float a = emu.sample(p);
            const float b = emu.sample(q);
            // 只考察窄带内部（等值面附近）的连续性；窄带边缘限幅过渡区
            // （|d|≈8 处，原始数据本身存在 7.x→1e6 硬跳变）不参与比较
            if (std::fabs(a) >= 7.0f || std::fabs(b) >= 7.0f) continue;
            const float step = std::fabs(a - b);
            if (step > max_step) {
                max_step = step;
                worst_p = p;
                worst_q = q;
                worst_a = a;
                worst_b = b;
            }
        }
        std::cout << "[boundary] max_step=" << max_step << std::endl;
        if (max_step > 0.05f) {
            std::cerr << "FAIL: discontinuity across chunk boundary "
                      << max_step << std::endl;
            std::cerr << "  p=(" << worst_p.x << "," << worst_p.y << ","
                      << worst_p.z << ") a=" << worst_a << "\n"
                      << "  q=(" << worst_q.x << "," << worst_q.y << ","
                      << worst_q.z << ") b=" << worst_b << std::endl;
            // 打印 q 三线性 8 角点的原始网格值
            const float vv[3] = {worst_q.x, worst_q.y, worst_q.z};
            int idx0[3];
            for (int ax = 0; ax < 3; ++ax)
                idx0[ax] = static_cast<int>(std::floor(vv[ax] - 0.5f));
            for (int dz = 0; dz <= 1; ++dz)
                for (int dy = 0; dy <= 1; ++dy)
                    for (int dx = 0; dx <= 1; ++dx)
                        std::cerr << "  corner(" << idx0[0] + dx << ","
                                  << idx0[1] + dy << "," << idx0[2] + dz
                                  << ")="
                                  << grid.getVoxelValue(idx0[0] + dx,
                                                        idx0[1] + dy,
                                                        idx0[2] + dz)
                                  << std::endl;
            return 1;
        }
    }

    // ============ 3. marching：命中半径应≈12（两个采样场都测） ===========
    {
        const Vec3f center(31.7f, 19.3f, 33.4f);
        std::mt19937 rng(123);
        std::uniform_real_distribution<float> uf(-1.0f, 1.0f);
        float max_err_emu = 0.0f, max_err_ref = 0.0f;
        int miss_emu = 0;
        for (int i = 0; i < 2000; ++i) {
            Vec3f dir(uf(rng), uf(rng), uf(rng));
            const float len =
                std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
            if (len < 1e-3f) continue;
            dir = Vec3f(dir.x / len, dir.y / len, dir.z / len);
            const Vec3f ro = center + dir * 25.0f;
            const Vec3f rd = dir * -1.0f;

            auto march = [&](auto&& sampler) -> float {
                float t = 0.0f, prevT = 0.0f;
                for (int it = 0; it < 256; ++it) {
                    const Vec3f p = ro + rd * t;
                    const float d = sampler(p);
                    if (d < 0.0f) {
                        float lo = prevT, hi = t;
                        for (int k = 0; k < 8; ++k) {
                            const float mid = (lo + hi) * 0.5f;
                            if (sampler(ro + rd * mid) < 0.0f)
                                hi = mid;
                            else
                                lo = mid;
                        }
                        return (lo + hi) * 0.5f;
                    }
                    float step;
                    if (d >= 1.0e5f) {
                        step = 32.0f;  // 简化：测试场无跳 chunk 风险
                    } else {
                        step = std::max(d * 0.9f, 0.25f);
                    }
                    prevT = t;
                    t += step;
                    if (t > 60.0f) break;
                }
                return -1.0f;
            };

            const float t_emu = march(
                [&](Vec3f p) { return emu.sample(p); });
            const float t_ref = march(
                [&](Vec3f p) { return sampleRef(grid, p); });
            if (t_emu < 0.0f) {
                ++miss_emu;
                continue;
            }
            const Vec3f hit_emu = ro + rd * t_emu;
            const float r_emu = (hit_emu - center).length();
            const float err_emu = std::fabs(r_emu - 12.0f);
            if (err_emu > max_err_emu) max_err_emu = err_emu;
            if (t_ref > 0.0f) {
                const Vec3f hit_ref = ro + rd * t_ref;
                const float err_ref =
                    std::fabs((hit_ref - center).length() - 12.0f);
                if (err_ref > max_err_ref) max_err_ref = err_ref;
            }
        }
        std::cout << "[march] miss_emu=" << miss_emu
                  << " max_err_emu=" << max_err_emu
                  << " max_err_ref=" << max_err_ref << std::endl;
        if (miss_emu > 0 || max_err_emu > 0.1f) {
            std::cerr << "FAIL: marching err=" << max_err_emu
                      << " miss=" << miss_emu << std::endl;
            return 1;
        }
    }

    std::cout << "test_sdf_gpu_field OK" << std::endl;
    return 0;
}
