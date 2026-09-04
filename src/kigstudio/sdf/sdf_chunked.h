#pragma once

#include <cJSON.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "kigstudio/sdf/sdf.h"
#include "kigstudio/voxel/voxel.h"

namespace sinriv::kigstudio::sdf {

// 缺失 chunk / 空场的远场值（外部为正），与 SDFGrid 空网格的返回值一致
inline constexpr float kSDFChunkedFar = 1e6f;

// ================= SDFChunk =================
// 32³ 体素分辨率的 SDF chunk，采样点位于体素中心。
// 两种存储状态（对应体素 Chunk 的压缩思路）：
//   Uniform —— 整块为常量（远场/内部），只占一个 float
//   Dense   —— 32³ float
struct SDFChunk {
    static constexpr int SIZE = 32;
    static constexpr int VOXEL_COUNT = SIZE * SIZE * SIZE;

    enum class Type : uint8_t { Uniform, Dense };

    Type type = Type::Uniform;
    float uniform_value = kSDFChunkedFar;
    std::unique_ptr<float[]> dense;

    SDFChunk() = default;
    SDFChunk(const SDFChunk& o) : type(o.type), uniform_value(o.uniform_value) {
        if (o.dense) {
            dense = std::make_unique<float[]>(VOXEL_COUNT);
            std::memcpy(dense.get(), o.dense.get(),
                        sizeof(float) * VOXEL_COUNT);
        }
    }
    SDFChunk& operator=(const SDFChunk& o) {
        if (this == &o) {
            return *this;
        }
        type = o.type;
        uniform_value = o.uniform_value;
        if (o.dense) {
            dense = std::make_unique<float[]>(VOXEL_COUNT);
            std::memcpy(dense.get(), o.dense.get(),
                        sizeof(float) * VOXEL_COUNT);
        } else {
            dense.reset();
        }
        return *this;
    }
    SDFChunk(SDFChunk&&) = default;
    SDFChunk& operator=(SDFChunk&&) = default;

    static inline int index(int x, int y, int z) {
        return (z * SIZE + y) * SIZE + x;
    }

    inline float get(int x, int y, int z) const {
        if (type == Type::Uniform) {
            return uniform_value;
        }
        return dense[index(x, y, z)];
    }

    inline void set(int x, int y, int z, float v) {
        if (type == Type::Uniform) {
            if (v == uniform_value) {
                return;
            }
            promote();
        }
        dense[index(x, y, z)] = v;
    }

    // Uniform -> Dense 提升（对应体素 AllOne clear 时的细分）
    inline void promote() {
        if (type == Type::Dense) {
            return;
        }
        dense = std::make_unique<float[]>(VOXEL_COUNT);
        std::fill_n(dense.get(), VOXEL_COUNT, uniform_value);
        type = Type::Dense;
    }

    // 全部值相等时回收为 Uniform，返回是否发生压缩
    inline bool compress() {
        if (type == Type::Uniform) {
            return false;
        }
        const float first = dense[0];
        for (int i = 1; i < VOXEL_COUNT; ++i) {
            if (dense[i] != first) {
                return false;
            }
        }
        uniform_value = first;
        dense.reset();
        type = Type::Uniform;
        return true;
    }
};

// ================= SDFChunkedGrid =================
// 稀疏分块 SDF 场：unordered_map<chunkKey, SDFChunk>，key 与 VoxelGrid
// 共用 packChunkKey 约定。采样点位于体素中心：
//   world(v) = global_position + (v + 0.5) * voxel_size
// 缺失 chunk 视为 +kSDFChunkedFar（外部）。实现 SDFBase 接口，
// mesher（generateSmoothMeshChunked / generateSmoothMeshForRegion）与
// 渲染层（loadSDFChunked / updateSDFRegion）无需改动即可使用。
class SDFChunkedGrid : public SDFBase {
   public:
    std::unordered_map<uint64_t, SDFChunk> chunks;
    Vec3f global_position{0.f, 0.f, 0.f};
    Vec3f voxel_size{1.f, 1.f, 1.f};

    // ============ 体素级读写（整数体素坐标） ============

    inline float getVoxelValue(int x, int y, int z) const {
        auto it = chunks.find(voxel::packChunkKey(x >> 5, y >> 5, z >> 5));
        if (it == chunks.end()) {
            return kSDFChunkedFar;
        }
        return it->second.get(x & 31, y & 31, z & 31);
    }

    inline float getVoxelValue(const Vec3i& p) const {
        return getVoxelValue(p.x, p.y, p.z);
    }

    inline void setVoxelValue(int x, int y, int z, float v) {
        chunks[voxel::packChunkKey(x >> 5, y >> 5, z >> 5)].set(x & 31, y & 31,
                                                                z & 31, v);
    }

    inline Vec3f voxelCenterToWorld(const Vec3i& v) const {
        return Vec3f(global_position.x + (v.x + 0.5f) * voxel_size.x,
                     global_position.y + (v.y + 0.5f) * voxel_size.y,
                     global_position.z + (v.z + 0.5f) * voxel_size.z);
    }

    // ============ 局部更新 ============
    // 对闭区间 [voxel_min, voxel_max] 内的体素应用
    // func(world_pos, old_value) -> new_value，只分配/触碰与区域相交的
    // chunk。返回受影响的 chunk key 列表（可直接交给渲染层
    // updateSDFRegion 做局部重 meshing）。
    template <typename F>
    std::vector<uint64_t> updateRegion(const Vec3i& voxel_min,
                                       const Vec3i& voxel_max, F&& func) {
        std::vector<uint64_t> affected;
        const int min_cx = voxel_min.x >> 5, min_cy = voxel_min.y >> 5,
                  min_cz = voxel_min.z >> 5;
        const int max_cx = voxel_max.x >> 5, max_cy = voxel_max.y >> 5,
                  max_cz = voxel_max.z >> 5;

        for (int cz = min_cz; cz <= max_cz; ++cz) {
            for (int cy = min_cy; cy <= max_cy; ++cy) {
                for (int cx = min_cx; cx <= max_cx; ++cx) {
                    const int bx0 = std::max(voxel_min.x, cx << 5);
                    const int by0 = std::max(voxel_min.y, cy << 5);
                    const int bz0 = std::max(voxel_min.z, cz << 5);
                    const int bx1 = std::min(voxel_max.x, (cx << 5) + 31);
                    const int by1 = std::min(voxel_max.y, (cy << 5) + 31);
                    const int bz1 = std::min(voxel_max.z, (cz << 5) + 31);

                    const uint64_t key = voxel::packChunkKey(cx, cy, cz);
                    const bool existed = chunks.find(key) != chunks.end();
                    SDFChunk& chunk = chunks[key];
                    for (int z = bz0; z <= bz1; ++z) {
                        for (int y = by0; y <= by1; ++y) {
                            for (int x = bx0; x <= bx1; ++x) {
                                const float old_v =
                                    chunk.get(x & 31, y & 31, z & 31);
                                const float new_v =
                                    func(voxelCenterToWorld({x, y, z}), old_v);
                                chunk.set(x & 31, y & 31, z & 31, new_v);
                            }
                        }
                    }
                    // 新建且未被实际修改的远场 chunk 不留存，避免污染 map
                    if (!existed && chunk.type == SDFChunk::Type::Uniform &&
                        chunk.uniform_value == kSDFChunkedFar) {
                        chunks.erase(key);
                        continue;
                    }
                    affected.push_back(key);
                }
            }
        }
        return affected;
    }

    // ============ 笔刷共用 ============
    // 球形笔刷的 SDF 体素 AABB（采样点在体素中心：v = (p-gp)/vs - 0.5）
    inline void brushRegionAABB(const Vec3f& center_world, float radius,
                                Vec3i& out_min, Vec3i& out_max) const {
        const float cx =
            (center_world.x - global_position.x) / voxel_size.x - 0.5f;
        const float cy =
            (center_world.y - global_position.y) / voxel_size.y - 0.5f;
        const float cz =
            (center_world.z - global_position.z) / voxel_size.z - 0.5f;
        const float rv = radius / voxel_size.x;  // 分块网格总是各向同性
        out_min = Vec3i(static_cast<int>(std::floor(cx - rv)),
                        static_cast<int>(std::floor(cy - rv)),
                        static_cast<int>(std::floor(cz - rv)));
        out_max = Vec3i(static_cast<int>(std::ceil(cx + rv)),
                        static_cast<int>(std::ceil(cy + rv)),
                        static_cast<int>(std::ceil(cz + rv)));
    }

    // ============ 平滑笔刷 ============
    // 球形区域内的 27 邻域均值平滑：
    //   new = lerp(old, avg27, strength * falloff(dist/radius))
    // falloff 为 smoothstep(1→0)，中心强、边缘无感。
    // 两遍法（先快照区域+1 halo 再写回），跨 chunk 采样走 getVoxelValue。
    // 窄带外的 ±kSDFChunkedFar 参与平均前会被钳制到局部窄带宽度 cap，
    // 防止远场值把表面拉飞；纯远场区域直接返回 false。
    // 返回是否有写入；out_min/out_max 为受影响区域（SDF 体素坐标闭区间），
    // affected_chunks 非空时收集被触碰的 chunk key。
    bool smoothRegion(const Vec3f& center_world, float radius, float strength,
                      Vec3i& out_min, Vec3i& out_max,
                      std::vector<uint64_t>* affected_chunks = nullptr) {
        if (radius <= 0.f || strength <= 0.f) {
            return false;
        }
        // 世界 -> SDF 体素连续坐标（采样点在体素中心：v = (p-gp)/vs - 0.5）
        const float cx =
            (center_world.x - global_position.x) / voxel_size.x - 0.5f;
        const float cy =
            (center_world.y - global_position.y) / voxel_size.y - 0.5f;
        const float cz =
            (center_world.z - global_position.z) / voxel_size.z - 0.5f;
        const float rv = radius / voxel_size.x;  // 分块网格总是各向同性

        brushRegionAABB(center_world, radius, out_min, out_max);

        const int nx = out_max.x - out_min.x + 1;
        const int ny = out_max.y - out_min.y + 1;
        const int nz = out_max.z - out_min.z + 1;
        // 快照含 1 体素 halo
        const int hx = nx + 2, hy = ny + 2, hz = nz + 2;
        std::vector<float> snap(static_cast<size_t>(hx) * hy * hz);
        auto snap_idx = [&](int x, int y, int z) {
            return (z * hy + y) * hx + x;
        };
        for (int z = 0; z < hz; ++z) {
            for (int y = 0; y < hy; ++y) {
                for (int x = 0; x < hx; ++x) {
                    snap[snap_idx(x, y, z)] =
                        getVoxelValue(out_min.x + x - 1, out_min.y + y - 1,
                                      out_min.z + z - 1);
                }
            }
        }

        // 局部窄带宽度：快照内非远场值的最大绝对值；全远场则无事可做
        float cap = 0.f;
        for (float v : snap) {
            if (std::fabs(v) < kSDFChunkedFar) {
                cap = std::max(cap, std::fabs(v));
            }
        }
        if (cap <= 0.f) {
            return false;
        }
        auto clamp_far = [&](float v) {
            if (v >= kSDFChunkedFar) return cap;
            if (v <= -kSDFChunkedFar) return -cap;
            return v;
        };

        std::vector<uint64_t> touched;
        bool changed = false;
        for (int z = 0; z < nz; ++z) {
            for (int y = 0; y < ny; ++y) {
                for (int x = 0; x < nx; ++x) {
                    // 27 邻域均值（快照坐标 +1 偏移 halo）
                    float sum = 0.f;
                    for (int dz = -1; dz <= 1; ++dz) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            for (int dx = -1; dx <= 1; ++dx) {
                                sum += clamp_far(snap[snap_idx(
                                    x + 1 + dx, y + 1 + dy, z + 1 + dz)]);
                            }
                        }
                    }
                    const float avg = sum * (1.f / 27.f);

                    const float wx = out_min.x + x - cx;
                    const float wy = out_min.y + y - cy;
                    const float wz = out_min.z + z - cz;
                    const float dist = std::sqrt(wx * wx + wy * wy + wz * wz);
                    float t = dist / rv;  // 0 中心 → 1+ 边缘
                    t = std::clamp(t, 0.f, 1.f);
                    const float falloff = 1.f - t * t * (3.f - 2.f * t);
                    if (falloff <= 0.f) {
                        continue;
                    }

                    const int vx = out_min.x + x;
                    const int vy = out_min.y + y;
                    const int vz = out_min.z + z;
                    const float old_v = snap[snap_idx(x + 1, y + 1, z + 1)];
                    const float new_v =
                        old_v + (clamp_far(avg) - clamp_far(old_v)) *
                                    strength * falloff;
                    if (new_v == old_v) {
                        continue;
                    }
                    // 远场（缺失/Uniform ±kFar）且平滑后仍在窄带外：跳过，
                    // 避免把远场写成无意义的大数值稠密 chunk
                    if (std::fabs(old_v) >= kSDFChunkedFar &&
                        std::fabs(new_v) > cap) {
                        continue;
                    }
                    setVoxelValue(vx, vy, vz, new_v);
                    touched.push_back(
                        voxel::packChunkKey(vx >> 5, vy >> 5, vz >> 5));
                    changed = true;
                }
            }
        }
        if (changed) {
            // 去重 + 压缩回收
            std::sort(touched.begin(), touched.end());
            touched.erase(std::unique(touched.begin(), touched.end()),
                          touched.end());
            for (uint64_t key : touched) {
                auto it = chunks.find(key);
                if (it != chunks.end()) {
                    it->second.compress();
                }
            }
            if (affected_chunks) {
                *affected_chunks = std::move(touched);
            }
        }
        return changed;
    }

    // ============ 铲平笔刷 ============
    // 把笔刷球内的 SDF 值双向拉向目标平面（世界单位距离）：
    //   new = lerp(old, dot(w - plane_origin, plane_normal), strength * falloff)
    // 逐点操作，单遍写回；falloff、远场 clamp/跳过、chunk 簿记与
    // smoothRegion 一致。plane_normal 需已归一化。纯远场区域返回 false。
    bool flattenRegion(const Vec3f& center_world, float radius, float strength,
                       const Vec3f& plane_origin, const Vec3f& plane_normal,
                       Vec3i& out_min, Vec3i& out_max,
                       std::vector<uint64_t>* affected_chunks = nullptr) {
        if (radius <= 0.f || strength <= 0.f) {
            return false;
        }
        const float cx =
            (center_world.x - global_position.x) / voxel_size.x - 0.5f;
        const float cy =
            (center_world.y - global_position.y) / voxel_size.y - 0.5f;
        const float cz =
            (center_world.z - global_position.z) / voxel_size.z - 0.5f;
        const float rv = radius / voxel_size.x;
        brushRegionAABB(center_world, radius, out_min, out_max);

        // 局部窄带宽度：区域内非远场值的最大绝对值；全远场则无事可做
        float cap = 0.f;
        for (int z = out_min.z; z <= out_max.z; ++z) {
            for (int y = out_min.y; y <= out_max.y; ++y) {
                for (int x = out_min.x; x <= out_max.x; ++x) {
                    const float v = getVoxelValue(x, y, z);
                    if (std::fabs(v) < kSDFChunkedFar) {
                        cap = std::max(cap, std::fabs(v));
                    }
                }
            }
        }
        if (cap <= 0.f) {
            return false;
        }
        auto clamp_far = [&](float v) {
            if (v >= kSDFChunkedFar) return cap;
            if (v <= -kSDFChunkedFar) return -cap;
            return v;
        };

        std::vector<uint64_t> touched;
        bool changed = false;
        for (int z = out_min.z; z <= out_max.z; ++z) {
            for (int y = out_min.y; y <= out_max.y; ++y) {
                for (int x = out_min.x; x <= out_max.x; ++x) {
                    const float wx = x - cx, wy = y - cy, wz = z - cz;
                    const float dist = std::sqrt(wx * wx + wy * wy + wz * wz);
                    float t = dist / rv;
                    t = std::clamp(t, 0.f, 1.f);
                    const float falloff = 1.f - t * t * (3.f - 2.f * t);
                    if (falloff <= 0.f) {
                        continue;
                    }

                    const float old_v = getVoxelValue(x, y, z);
                    const Vec3f w = voxelCenterToWorld({x, y, z});
                    const float d_plane =
                        (w.x - plane_origin.x) * plane_normal.x +
                        (w.y - plane_origin.y) * plane_normal.y +
                        (w.z - plane_origin.z) * plane_normal.z;
                    const float new_v =
                        old_v + (clamp_far(d_plane) - clamp_far(old_v)) *
                                    strength * falloff;
                    if (new_v == old_v) {
                        continue;
                    }
                    if (std::fabs(old_v) >= kSDFChunkedFar &&
                        std::fabs(new_v) > cap) {
                        continue;
                    }
                    setVoxelValue(x, y, z, new_v);
                    touched.push_back(voxel::packChunkKey(x >> 5, y >> 5,
                                                          z >> 5));
                    changed = true;
                }
            }
        }
        if (changed) {
            std::sort(touched.begin(), touched.end());
            touched.erase(std::unique(touched.begin(), touched.end()),
                          touched.end());
            for (uint64_t key : touched) {
                auto it = chunks.find(key);
                if (it != chunks.end()) {
                    it->second.compress();
                }
            }
            if (affected_chunks) {
                *affected_chunks = std::move(touched);
            }
        }
        return changed;
    }

    // ============ 增减料笔刷 ============
    // 把等值面沿法线外推/内收：
    //   new = old - dir * amount * strength * falloff
    // dir=+1 增料（表面外扩），dir=-1 减料/刻槽。amount 为世界单位。
    // 与平滑/铲平不同：cap 取 max(局部窄带 cap, radius)，允许在空处
    // 起一团料。逐点操作，单遍写回；其余簿记与 smoothRegion 一致。
    bool drawRegion(const Vec3f& center_world, float radius, float strength,
                    float amount, float dir, Vec3i& out_min, Vec3i& out_max,
                    std::vector<uint64_t>* affected_chunks = nullptr) {
        if (radius <= 0.f || strength <= 0.f || amount <= 0.f || dir == 0.f) {
            return false;
        }
        const float cx =
            (center_world.x - global_position.x) / voxel_size.x - 0.5f;
        const float cy =
            (center_world.y - global_position.y) / voxel_size.y - 0.5f;
        const float cz =
            (center_world.z - global_position.z) / voxel_size.z - 0.5f;
        const float rv = radius / voxel_size.x;
        brushRegionAABB(center_world, radius, out_min, out_max);

        float cap = 0.f;
        for (int z = out_min.z; z <= out_max.z; ++z) {
            for (int y = out_min.y; y <= out_max.y; ++y) {
                for (int x = out_min.x; x <= out_max.x; ++x) {
                    const float v = getVoxelValue(x, y, z);
                    if (std::fabs(v) < kSDFChunkedFar) {
                        cap = std::max(cap, std::fabs(v));
                    }
                }
            }
        }
        cap = std::max(cap, radius);  // 空中起料的兜底窄带
        auto clamp_far = [&](float v) {
            if (v >= kSDFChunkedFar) return cap;
            if (v <= -kSDFChunkedFar) return -cap;
            return v;
        };

        std::vector<uint64_t> touched;
        bool changed = false;
        for (int z = out_min.z; z <= out_max.z; ++z) {
            for (int y = out_min.y; y <= out_max.y; ++y) {
                for (int x = out_min.x; x <= out_max.x; ++x) {
                    const float wx = x - cx, wy = y - cy, wz = z - cz;
                    const float dist = std::sqrt(wx * wx + wy * wy + wz * wz);
                    float t = dist / rv;
                    t = std::clamp(t, 0.f, 1.f);
                    const float falloff = 1.f - t * t * (3.f - 2.f * t);
                    if (falloff <= 0.f) {
                        continue;
                    }

                    const float old_v = getVoxelValue(x, y, z);
                    const float new_v = clamp_far(old_v) - dir * amount *
                                                              strength *
                                                              falloff;
                    if (new_v == old_v) {
                        continue;
                    }
                    // 远场且结果仍在窄带外：跳过（减料进空气/增料进深内部）
                    if (std::fabs(old_v) >= kSDFChunkedFar &&
                        std::fabs(new_v) > cap) {
                        continue;
                    }
                    setVoxelValue(x, y, z, new_v);
                    touched.push_back(voxel::packChunkKey(x >> 5, y >> 5,
                                                          z >> 5));
                    changed = true;
                }
            }
        }
        if (changed) {
            std::sort(touched.begin(), touched.end());
            touched.erase(std::unique(touched.begin(), touched.end()),
                          touched.end());
            for (uint64_t key : touched) {
                auto it = chunks.find(key);
                if (it != chunks.end()) {
                    it->second.compress();
                }
            }
            if (affected_chunks) {
                *affected_chunks = std::move(touched);
            }
        }
        return changed;
    }

    // ============ 膨胀/收缩笔刷 ============
    // 与 drawRegion 相同，但 falloff 为平台型：t<=0.7 恒为 1，[0.7,1] 内
    // smoothstep 降到 0。核心区内等值面沿法线均匀外扩/内缩（整体肿胀），
    // 区别于 draw 的球面凸起轮廓。dir=+1 膨胀，dir=-1 收缩。
    bool inflateRegion(const Vec3f& center_world, float radius, float strength,
                       float amount, float dir, Vec3i& out_min, Vec3i& out_max,
                       std::vector<uint64_t>* affected_chunks = nullptr) {
        if (radius <= 0.f || strength <= 0.f || amount <= 0.f || dir == 0.f) {
            return false;
        }
        const float cx =
            (center_world.x - global_position.x) / voxel_size.x - 0.5f;
        const float cy =
            (center_world.y - global_position.y) / voxel_size.y - 0.5f;
        const float cz =
            (center_world.z - global_position.z) / voxel_size.z - 0.5f;
        const float rv = radius / voxel_size.x;
        brushRegionAABB(center_world, radius, out_min, out_max);

        float cap = 0.f;
        for (int z = out_min.z; z <= out_max.z; ++z) {
            for (int y = out_min.y; y <= out_max.y; ++y) {
                for (int x = out_min.x; x <= out_max.x; ++x) {
                    const float v = getVoxelValue(x, y, z);
                    if (std::fabs(v) < kSDFChunkedFar) {
                        cap = std::max(cap, std::fabs(v));
                    }
                }
            }
        }
        cap = std::max(cap, radius);  // 空中起料的兜底窄带
        auto clamp_far = [&](float v) {
            if (v >= kSDFChunkedFar) return cap;
            if (v <= -kSDFChunkedFar) return -cap;
            return v;
        };

        std::vector<uint64_t> touched;
        bool changed = false;
        for (int z = out_min.z; z <= out_max.z; ++z) {
            for (int y = out_min.y; y <= out_max.y; ++y) {
                for (int x = out_min.x; x <= out_max.x; ++x) {
                    const float wx = x - cx, wy = y - cy, wz = z - cz;
                    const float dist = std::sqrt(wx * wx + wy * wy + wz * wz);
                    float t = dist / rv;
                    t = std::clamp(t, 0.f, 1.f);
                    // 平台型 falloff
                    float falloff = 1.f;
                    if (t > 0.7f) {
                        const float t2 = (t - 0.7f) / 0.3f;
                        falloff = 1.f - t2 * t2 * (3.f - 2.f * t2);
                    }
                    if (falloff <= 0.f) {
                        continue;
                    }

                    const float old_v = getVoxelValue(x, y, z);
                    const float new_v = clamp_far(old_v) - dir * amount *
                                                              strength *
                                                              falloff;
                    if (new_v == old_v) {
                        continue;
                    }
                    if (std::fabs(old_v) >= kSDFChunkedFar &&
                        std::fabs(new_v) > cap) {
                        continue;
                    }
                    setVoxelValue(x, y, z, new_v);
                    touched.push_back(voxel::packChunkKey(x >> 5, y >> 5,
                                                          z >> 5));
                    changed = true;
                }
            }
        }
        if (changed) {
            std::sort(touched.begin(), touched.end());
            touched.erase(std::unique(touched.begin(), touched.end()),
                          touched.end());
            for (uint64_t key : touched) {
                auto it = chunks.find(key);
                if (it != chunks.end()) {
                    it->second.compress();
                }
            }
            if (affected_chunks) {
                *affected_chunks = std::move(touched);
            }
        }
        return changed;
    }

    // ============ 变形笔刷（拖动平流） ============
    // 把笔刷球内的场沿 drag 方向平流：new(p) = old(p - delta·strength·falloff)。
    // 两遍法：快照 AABB + halo（覆盖最大位移），从快照三线性采样后写回。
    // delta 为世界单位位移（通常取相邻 dab 的鼠标世界位移）。
    bool moveRegion(const Vec3f& center_world, float radius, float strength,
                    const Vec3f& delta_world, Vec3i& out_min, Vec3i& out_max,
                    std::vector<uint64_t>* affected_chunks = nullptr) {
        if (radius <= 0.f || strength <= 0.f) {
            return false;
        }
        const float dlen = delta_world.length();
        if (dlen * strength < 1e-6f) {
            return false;
        }
        const float cx =
            (center_world.x - global_position.x) / voxel_size.x - 0.5f;
        const float cy =
            (center_world.y - global_position.y) / voxel_size.y - 0.5f;
        const float cz =
            (center_world.z - global_position.z) / voxel_size.z - 0.5f;
        const float rv = radius / voxel_size.x;
        brushRegionAABB(center_world, radius, out_min, out_max);

        // halo 覆盖最大位移（体素单位）
        const int halo =
            static_cast<int>(std::ceil(dlen * strength / voxel_size.x)) + 1;
        const int nx = out_max.x - out_min.x + 1;
        const int ny = out_max.y - out_min.y + 1;
        const int nz = out_max.z - out_min.z + 1;
        const int hx = nx + 2 * halo, hy = ny + 2 * halo, hz = nz + 2 * halo;
        const int ox = out_min.x - halo, oy = out_min.y - halo,
                  oz = out_min.z - halo;
        std::vector<float> snap(static_cast<size_t>(hx) * hy * hz);
        auto snap_idx = [&](int x, int y, int z) {
            return (z * hy + y) * hx + x;
        };
        for (int z = 0; z < hz; ++z) {
            for (int y = 0; y < hy; ++y) {
                for (int x = 0; x < hx; ++x) {
                    snap[snap_idx(x, y, z)] =
                        getVoxelValue(ox + x, oy + y, oz + z);
                }
            }
        }

        float cap = 0.f;
        for (float v : snap) {
            if (std::fabs(v) < kSDFChunkedFar) {
                cap = std::max(cap, std::fabs(v));
            }
        }
        if (cap <= 0.f) {
            return false;
        }
        auto clamp_far = [&](float v) {
            if (v >= kSDFChunkedFar) return cap;
            if (v <= -kSDFChunkedFar) return -cap;
            return v;
        };

        // 快照内三线性采样（输入为体素连续坐标）
        auto sample = [&](float fx, float fy, float fz) {
            const float sx = fx - static_cast<float>(ox);
            const float sy = fy - static_cast<float>(oy);
            const float sz = fz - static_cast<float>(oz);
            const int x0 = std::clamp(static_cast<int>(std::floor(sx)), 0,
                                      hx - 2);
            const int y0 = std::clamp(static_cast<int>(std::floor(sy)), 0,
                                      hy - 2);
            const int z0 = std::clamp(static_cast<int>(std::floor(sz)), 0,
                                      hz - 2);
            const float tx = std::clamp(sx - static_cast<float>(x0), 0.f, 1.f);
            const float ty = std::clamp(sy - static_cast<float>(y0), 0.f, 1.f);
            const float tz = std::clamp(sz - static_cast<float>(z0), 0.f, 1.f);
            const float c00 =
                snap[snap_idx(x0, y0, z0)] * (1.f - tx) +
                snap[snap_idx(x0 + 1, y0, z0)] * tx;
            const float c10 =
                snap[snap_idx(x0, y0 + 1, z0)] * (1.f - tx) +
                snap[snap_idx(x0 + 1, y0 + 1, z0)] * tx;
            const float c01 =
                snap[snap_idx(x0, y0, z0 + 1)] * (1.f - tx) +
                snap[snap_idx(x0 + 1, y0, z0 + 1)] * tx;
            const float c11 =
                snap[snap_idx(x0, y0 + 1, z0 + 1)] * (1.f - tx) +
                snap[snap_idx(x0 + 1, y0 + 1, z0 + 1)] * tx;
            const float c0 = c00 * (1.f - ty) + c10 * ty;
            const float c1 = c01 * (1.f - ty) + c11 * ty;
            return c0 * (1.f - tz) + c1 * tz;
        };

        // 位移（体素单位）
        const float dvx = delta_world.x * strength / voxel_size.x;
        const float dvy = delta_world.y * strength / voxel_size.y;
        const float dvz = delta_world.z * strength / voxel_size.z;

        std::vector<uint64_t> touched;
        bool changed = false;
        for (int z = out_min.z; z <= out_max.z; ++z) {
            for (int y = out_min.y; y <= out_max.y; ++y) {
                for (int x = out_min.x; x <= out_max.x; ++x) {
                    const float wx = x - cx, wy = y - cy, wz = z - cz;
                    const float dist = std::sqrt(wx * wx + wy * wy + wz * wz);
                    float t = dist / rv;
                    t = std::clamp(t, 0.f, 1.f);
                    const float falloff = 1.f - t * t * (3.f - 2.f * t);
                    if (falloff <= 0.f) {
                        continue;
                    }

                    const float old_v = getVoxelValue(x, y, z);
                    const float new_v = clamp_far(
                        sample(x - dvx * falloff, y - dvy * falloff,
                               z - dvz * falloff));
                    if (new_v == old_v) {
                        continue;
                    }
                    if (std::fabs(old_v) >= kSDFChunkedFar &&
                        std::fabs(new_v) > cap) {
                        continue;
                    }
                    setVoxelValue(x, y, z, new_v);
                    touched.push_back(voxel::packChunkKey(x >> 5, y >> 5,
                                                          z >> 5));
                    changed = true;
                }
            }
        }
        if (changed) {
            std::sort(touched.begin(), touched.end());
            touched.erase(std::unique(touched.begin(), touched.end()),
                          touched.end());
            for (uint64_t key : touched) {
                auto it = chunks.find(key);
                if (it != chunks.end()) {
                    it->second.compress();
                }
            }
            if (affected_chunks) {
                *affected_chunks = std::move(touched);
            }
        }
        return changed;
    }

    // ============ 场修复笔刷（局部重距离化） ============
    // 雕刻多笔后 SDF 偏离真实距离场（|∇f|≠1），meshing 出现棱刺。
    // 在笔刷区域内做 Godunov 上风重距离化：符号固定不变，符号变化的
    // 6 邻接体素为锚点（保值），其余按 |∇u|=1（体素单位）迭代求解，
    // 写回 new = lerp(old, sign·repaired, strength·falloff)。
    bool repairRegion(const Vec3f& center_world, float radius, float strength,
                      Vec3i& out_min, Vec3i& out_max,
                      std::vector<uint64_t>* affected_chunks = nullptr) {
        if (radius <= 0.f || strength <= 0.f) {
            return false;
        }
        const float cx =
            (center_world.x - global_position.x) / voxel_size.x - 0.5f;
        const float cy =
            (center_world.y - global_position.y) / voxel_size.y - 0.5f;
        const float cz =
            (center_world.z - global_position.z) / voxel_size.z - 0.5f;
        const float rv = radius / voxel_size.x;
        brushRegionAABB(center_world, radius, out_min, out_max);

        const int nx = out_max.x - out_min.x + 1;
        const int ny = out_max.y - out_min.y + 1;
        const int nz = out_max.z - out_min.z + 1;
        const int hx = nx + 2, hy = ny + 2, hz = nz + 2;
        std::vector<float> snap(static_cast<size_t>(hx) * hy * hz);
        auto idx = [&](int x, int y, int z) { return (z * hy + y) * hx + x; };
        for (int z = 0; z < hz; ++z) {
            for (int y = 0; y < hy; ++y) {
                for (int x = 0; x < hx; ++x) {
                    snap[idx(x, y, z)] = getVoxelValue(out_min.x + x - 1,
                                                       out_min.y + y - 1,
                                                       out_min.z + z - 1);
                }
            }
        }
        float cap = 0.f;
        for (float v : snap) {
            if (std::fabs(v) < kSDFChunkedFar) {
                cap = std::max(cap, std::fabs(v));
            }
        }
        if (cap <= 0.f) {
            return false;
        }
        auto clamp_far = [&](float v) {
            if (v >= kSDFChunkedFar) return cap;
            if (v <= -kSDFChunkedFar) return -cap;
            return v;
        };

        // 工作场：幅度（体素单位）+ 符号 + 锚点标记；halo 层固定为边界
        const float inv_vs = 1.f / voxel_size.x;
        std::vector<float> u(snap.size());
        std::vector<int8_t> sgn(snap.size());
        std::vector<char> fixed(snap.size(), 0);
        for (int z = 0; z < hz; ++z) {
            for (int y = 0; y < hy; ++y) {
                for (int x = 0; x < hx; ++x) {
                    const float v = clamp_far(snap[idx(x, y, z)]);
                    const int i = idx(x, y, z);
                    sgn[i] = v >= 0.f ? 1 : -1;
                    u[i] = std::fabs(v) * inv_vs;
                    // halo 层固定
                    if (x == 0 || y == 0 || z == 0 || x == hx - 1 ||
                        y == hy - 1 || z == hz - 1) {
                        fixed[i] = 1;
                    }
                }
            }
        }
        // 锚点：内部体素且 6 邻接有异号
        for (int z = 1; z < hz - 1; ++z) {
            for (int y = 1; y < hy - 1; ++y) {
                for (int x = 1; x < hx - 1; ++x) {
                    const int i = idx(x, y, z);
                    const int8_t s = sgn[i];
                    if (sgn[idx(x - 1, y, z)] != s ||
                        sgn[idx(x + 1, y, z)] != s ||
                        sgn[idx(x, y - 1, z)] != s ||
                        sgn[idx(x, y + 1, z)] != s ||
                        sgn[idx(x, y, z - 1)] != s ||
                        sgn[idx(x, y, z + 1)] != s) {
                        fixed[i] = 1;
                    }
                }
            }
        }
        // 非固定点初始化为大范围（体素单位）
        const float big = cap * inv_vs;
        for (size_t i = 0; i < u.size(); ++i) {
            if (!fixed[i]) {
                u[i] = big;
            }
        }

        // Godunov 上风 Gauss-Seidel 迭代
        const int sweeps = nx + ny + nz;
        for (int sweep = 0; sweep < sweeps; ++sweep) {
            for (int z = 1; z < hz - 1; ++z) {
                for (int y = 1; y < hy - 1; ++y) {
                    for (int x = 1; x < hx - 1; ++x) {
                        const int i = idx(x, y, z);
                        if (fixed[i]) {
                            continue;
                        }
                        float a = std::min(u[idx(x - 1, y, z)],
                                           u[idx(x + 1, y, z)]);
                        float b = std::min(u[idx(x, y - 1, z)],
                                           u[idx(x, y + 1, z)]);
                        float c = std::min(u[idx(x, y, z - 1)],
                                           u[idx(x, y, z + 1)]);
                        if (a > b) std::swap(a, b);
                        if (b > c) std::swap(b, c);
                        if (a > b) std::swap(a, b);
                        double un = a + 1.0;
                        if (un > b) {
                            const double disc2 =
                                2.0 - (a - b) * (a - b);
                            un = (a + b + std::sqrt(std::max(0.0, disc2))) *
                                 0.5;
                            if (un > c) {
                                const double s = a + b + c;
                                const double disc3 =
                                    s * s -
                                    3.0 * (a * a + b * b + c * c - 1.0);
                                un = (s + std::sqrt(std::max(0.0, disc3))) /
                                     3.0;
                            }
                        }
                        if (un < u[i]) {
                            u[i] = static_cast<float>(un);
                        }
                    }
                }
            }
        }

        std::vector<uint64_t> touched;
        bool changed = false;
        for (int z = 1; z < hz - 1; ++z) {
            for (int y = 1; y < hy - 1; ++y) {
                for (int x = 1; x < hx - 1; ++x) {
                    const float wx = out_min.x + x - 1 - cx;
                    const float wy = out_min.y + y - 1 - cy;
                    const float wz = out_min.z + z - 1 - cz;
                    const float dist = std::sqrt(wx * wx + wy * wy + wz * wz);
                    float t = dist / rv;
                    t = std::clamp(t, 0.f, 1.f);
                    const float falloff = 1.f - t * t * (3.f - 2.f * t);
                    if (falloff <= 0.f) {
                        continue;
                    }

                    const int vx = out_min.x + x - 1;
                    const int vy = out_min.y + y - 1;
                    const int vz = out_min.z + z - 1;
                    const float old_v = snap[idx(x, y, z)];
                    const int i = idx(x, y, z);
                    const float repaired =
                        static_cast<float>(sgn[i]) * u[i] * voxel_size.x;
                    const float new_v =
                        old_v + (repaired - clamp_far(old_v)) * strength *
                                    falloff;
                    if (new_v == old_v) {
                        continue;
                    }
                    if (std::fabs(old_v) >= kSDFChunkedFar &&
                        std::fabs(new_v) > cap) {
                        continue;
                    }
                    setVoxelValue(vx, vy, vz, new_v);
                    touched.push_back(voxel::packChunkKey(vx >> 5, vy >> 5,
                                                          vz >> 5));
                    changed = true;
                }
            }
        }
        if (changed) {
            std::sort(touched.begin(), touched.end());
            touched.erase(std::unique(touched.begin(), touched.end()),
                          touched.end());
            for (uint64_t key : touched) {
                auto it = chunks.find(key);
                if (it != chunks.end()) {
                    it->second.compress();
                }
            }
            if (affected_chunks) {
                *affected_chunks = std::move(touched);
            }
        }
        return changed;
    }

    // ============ 从解析式 SDF 烘焙 ============
    // 在闭区间 [voxel_min, voxel_max] 内按体素中心采样 src，逐 chunk 填充
    // 并压缩（整块同值的 chunk 回收为 Uniform）。
    static SDFChunkedGrid fromSDF(
        const SDFBase& src, const Vec3i& voxel_min, const Vec3i& voxel_max,
        const Vec3f& global_position = Vec3f(0.f, 0.f, 0.f),
        const Vec3f& voxel_size = Vec3f(1.f, 1.f, 1.f),
        float truncation = 8.0f) {
        SDFChunkedGrid grid;
        grid.global_position = global_position;
        grid.voxel_size = voxel_size;

        // 窄带截断：超出 ±truncation（世界单位）的值钳制为 ±kSDFChunkedFar，
        // 使远场/深内部 chunk 能压缩为 Uniform。meshing 只依赖符号与表面
        // 附近的梯度，截断不影响结果。
        auto clamp_value = [&](float v) {
            if (v > truncation) return kSDFChunkedFar;
            if (v < -truncation) return -kSDFChunkedFar;
            return v;
        };

        const int min_cx = voxel_min.x >> 5, min_cy = voxel_min.y >> 5,
                  min_cz = voxel_min.z >> 5;
        const int max_cx = voxel_max.x >> 5, max_cy = voxel_max.y >> 5,
                  max_cz = voxel_max.z >> 5;

        for (int cz = min_cz; cz <= max_cz; ++cz) {
            for (int cy = min_cy; cy <= max_cy; ++cy) {
                for (int cx = min_cx; cx <= max_cx; ++cx) {
                    const int bx0 = std::max(voxel_min.x, cx << 5);
                    const int by0 = std::max(voxel_min.y, cy << 5);
                    const int bz0 = std::max(voxel_min.z, cz << 5);
                    const int bx1 = std::min(voxel_max.x, (cx << 5) + 31);
                    const int by1 = std::min(voxel_max.y, (cy << 5) + 31);
                    const int bz1 = std::min(voxel_max.z, (cz << 5) + 31);
                    const int nx = bx1 - bx0 + 1;
                    const int ny = by1 - by0 + 1;
                    const int nz = bz1 - bz0 + 1;

                    // 批量采样该 chunk 与区域的交集（体素中心）
                    const Vec3f begin =
                        grid.voxelCenterToWorld({bx0, by0, bz0});
                    std::vector<float> block;
                    src.get(begin, voxel_size, Vec3i(nx, ny, nz), block);

                    const uint64_t key = voxel::packChunkKey(cx, cy, cz);
                    SDFChunk& chunk = grid.chunks[key];
                    for (int z = 0; z < nz; ++z) {
                        for (int y = 0; y < ny; ++y) {
                            for (int x = 0; x < nx; ++x) {
                                chunk.set(bx0 + x - (cx << 5),
                                          by0 + y - (cy << 5),
                                          bz0 + z - (cz << 5),
                                          clamp_value(
                                              block[(z * ny + y) * nx + x]));
                            }
                        }
                    }
                    chunk.compress();
                }
            }
        }
        return grid;
    }

    // ============ SDFBase 接口 ============

    // 世界坐标三线性插值采样；采样晶格位于体素中心
    float get(const Vec3f& p) const override {
        const float ux =
            (p.x - global_position.x) / voxel_size.x - 0.5f;
        const float uy =
            (p.y - global_position.y) / voxel_size.y - 0.5f;
        const float uz =
            (p.z - global_position.z) / voxel_size.z - 0.5f;

        const int x0 = static_cast<int>(std::floor(ux));
        const int y0 = static_cast<int>(std::floor(uy));
        const int z0 = static_cast<int>(std::floor(uz));
        const float tx = ux - static_cast<float>(x0);
        const float ty = uy - static_cast<float>(y0);
        const float tz = uz - static_cast<float>(z0);

        const float c000 = getVoxelValue(x0, y0, z0);
        const float c100 = getVoxelValue(x0 + 1, y0, z0);
        const float c010 = getVoxelValue(x0, y0 + 1, z0);
        const float c110 = getVoxelValue(x0 + 1, y0 + 1, z0);
        const float c001 = getVoxelValue(x0, y0, z0 + 1);
        const float c101 = getVoxelValue(x0 + 1, y0, z0 + 1);
        const float c011 = getVoxelValue(x0, y0 + 1, z0 + 1);
        const float c111 = getVoxelValue(x0 + 1, y0 + 1, z0 + 1);

        auto lerp = [](float a, float b, float t) { return a + (b - a) * t; };
        const float c00 = lerp(c000, c100, tx);
        const float c10 = lerp(c010, c110, tx);
        const float c01 = lerp(c001, c101, tx);
        const float c11 = lerp(c011, c111, tx);
        const float c0 = lerp(c00, c10, ty);
        const float c1 = lerp(c01, c11, ty);
        return lerp(c0, c1, tz);
    }

    // 批量采样（mesher 快路径）。与基类语义一致，但对连续点缓存
    // chunk 指针，避免每点 8 次 map 查找。
    void get(const Vec3f& begin, const Vec3f& sample_step,
             const Vec3i& sample_count, std::vector<float>& out) const override {
        const size_t total = static_cast<size_t>(sample_count.x) *
                             static_cast<size_t>(sample_count.y) *
                             static_cast<size_t>(sample_count.z);
        out.resize(total);

        uint64_t cached_key = 0;
        const SDFChunk* cached = nullptr;
        bool cached_valid = false;

        // 单点采样，复用 chunk 缓存
        auto sample = [&](const Vec3f& p) -> float {
            const float ux =
                (p.x - global_position.x) / voxel_size.x - 0.5f;
            const float uy =
                (p.y - global_position.y) / voxel_size.y - 0.5f;
            const float uz =
                (p.z - global_position.z) / voxel_size.z - 0.5f;

            const int x0 = static_cast<int>(std::floor(ux));
            const int y0 = static_cast<int>(std::floor(uy));
            const int z0 = static_cast<int>(std::floor(uz));
            const float tx = ux - static_cast<float>(x0);
            const float ty = uy - static_cast<float>(y0);
            const float tz = uz - static_cast<float>(z0);

            float corners[2][2][2];
            for (int dz = 0; dz <= 1; ++dz) {
                for (int dy = 0; dy <= 1; ++dy) {
                    for (int dx = 0; dx <= 1; ++dx) {
                        const int vx = x0 + dx, vy = y0 + dy, vz = z0 + dz;
                        const uint64_t key = voxel::packChunkKey(
                            vx >> 5, vy >> 5, vz >> 5);
                        if (!cached_valid || key != cached_key) {
                            auto it = chunks.find(key);
                            cached = (it == chunks.end()) ? nullptr
                                                          : &it->second;
                            cached_key = key;
                            cached_valid = true;
                        }
                        corners[dz][dy][dx] =
                            cached ? cached->get(vx & 31, vy & 31, vz & 31)
                                   : kSDFChunkedFar;
                    }
                }
            }

            auto lerp = [](float a, float b, float t) {
                return a + (b - a) * t;
            };
            const float c00 = lerp(corners[0][0][0], corners[0][0][1], tx);
            const float c10 = lerp(corners[0][1][0], corners[0][1][1], tx);
            const float c01 = lerp(corners[1][0][0], corners[1][0][1], tx);
            const float c11 = lerp(corners[1][1][0], corners[1][1][1], tx);
            const float c0 = lerp(c00, c10, ty);
            const float c1 = lerp(c01, c11, ty);
            return lerp(c0, c1, tz);
        };

        size_t i = 0;
        for (int z = 0; z < sample_count.z; ++z) {
            const float wz = begin.z + static_cast<float>(z) * sample_step.z;
            for (int y = 0; y < sample_count.y; ++y) {
                const float wy =
                    begin.y + static_cast<float>(y) * sample_step.y;
                for (int x = 0; x < sample_count.x; ++x) {
                    const float wx =
                        begin.x + static_cast<float>(x) * sample_step.x;
                    out[i++] = sample(Vec3f(wx, wy, wz));
                }
            }
        }
    }

    std::string getInfo(int indent = 0) const override {
        std::string prefix(indent * 2, ' ');
        return prefix + "SDFChunkedGrid(chunks=" + std::to_string(chunks.size()) +
               ")";
    }

    // ============ 序列化 ============
    // {"type":"chunked_grid", global_position, voxel_size,
    //  "chunks":[{"chunk":[cx,cy,cz], "value":v} |
    //            {"chunk":[cx,cy,cz], "data":[float...]}]}
    cJSON* toJSON() const override {
        cJSON* obj = cJSON_CreateObject();
        cJSON_AddStringToObject(obj, "type", "chunked_grid");
        cJSON_AddItemToObject(obj, "global_position",
                              sinriv::kigstudio::to_json(global_position));
        cJSON_AddItemToObject(obj, "voxel_size",
                              sinriv::kigstudio::to_json(voxel_size));

        cJSON* arr = cJSON_CreateArray();
        for (const auto& [key, chunk] : chunks) {
            int cx, cy, cz;
            voxel::unpackChunkKey(key, cx, cy, cz);
            cJSON* item = cJSON_CreateObject();
            cJSON* coord = cJSON_CreateIntArray(
                std::vector<int>{cx, cy, cz}.data(), 3);
            cJSON_AddItemToObject(item, "chunk", coord);
            if (chunk.type == SDFChunk::Type::Uniform) {
                cJSON_AddNumberToObject(item, "value", chunk.uniform_value);
            } else {
                cJSON_AddItemToObject(
                    item, "data",
                    cJSON_CreateFloatArray(chunk.dense.get(),
                                           SDFChunk::VOXEL_COUNT));
            }
            cJSON_AddItemToArray(arr, item);
        }
        cJSON_AddItemToObject(obj, "chunks", arr);
        return obj;
    }

    void fromJSON(const cJSON* json) override {
        if (!json) {
            return;
        }
        chunks.clear();

        const cJSON* gp = cJSON_GetObjectItem(json, "global_position");
        if (gp) {
            global_position = sinriv::kigstudio::vec3_from_json<Vec3f>(gp);
        }
        const cJSON* vs = cJSON_GetObjectItem(json, "voxel_size");
        if (vs) {
            voxel_size = sinriv::kigstudio::vec3_from_json<Vec3f>(vs);
        }

        const cJSON* arr = cJSON_GetObjectItem(json, "chunks");
        const cJSON* item = nullptr;
        cJSON_ArrayForEach(item, arr) {
            const cJSON* coord = cJSON_GetObjectItem(item, "chunk");
            if (!coord || cJSON_GetArraySize(coord) != 3) {
                continue;
            }
            const int cx = cJSON_GetArrayItem(coord, 0)->valueint;
            const int cy = cJSON_GetArrayItem(coord, 1)->valueint;
            const int cz = cJSON_GetArrayItem(coord, 2)->valueint;
            SDFChunk& chunk =
                chunks[voxel::packChunkKey(cx, cy, cz)];

            const cJSON* value = cJSON_GetObjectItem(item, "value");
            if (value && cJSON_IsNumber(value)) {
                chunk.type = SDFChunk::Type::Uniform;
                chunk.uniform_value =
                    static_cast<float>(cJSON_GetNumberValue(value));
                continue;
            }
            const cJSON* data = cJSON_GetObjectItem(item, "data");
            if (data && cJSON_IsArray(data) &&
                cJSON_GetArraySize(data) == SDFChunk::VOXEL_COUNT) {
                chunk.type = SDFChunk::Type::Dense;
                chunk.dense = std::make_unique<float[]>(SDFChunk::VOXEL_COUNT);
                int i = 0;
                const cJSON* v = nullptr;
                cJSON_ArrayForEach(v, data) {
                    chunk.dense[i++] =
                        static_cast<float>(cJSON_GetNumberValue(v));
                }
            }
        }
    }
};

}  // namespace sinriv::kigstudio::sdf
