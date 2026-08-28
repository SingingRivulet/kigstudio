#pragma once

#include <cJSON.h>
#include <cmath>
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
