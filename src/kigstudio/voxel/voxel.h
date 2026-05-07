#pragma once
#include "kigstudio/utils/plane.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/utils/compress.h"
#include "kigstudio/voxel/collision.h"
#include "kigstudio/voxel/concave.h"

#include <bit>
#include <bitset>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stack>
#include <unordered_map>
#include <vector>
#include <filesystem>
#include <fstream>
#include <cstdio>
#include <cstring>
#include <cJSON.h>

namespace sinriv::kigstudio {
namespace voxel {
// 定义三维向量结构体
struct Vec3i {
    int32_t x, y, z;

    inline Vec3i(int x = 0, int y = 0, int z = 0) : x(x), y(y), z(z) {}

    // 比较两个向量是否相等
    inline bool operator==(const Vec3i& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    inline bool operator!=(const Vec3i& other) const {
        return !(*this == other);
    }

    inline bool operator<(const Vec3i& other) const {
        return x < other.x || (x == other.x && y < other.y) ||
               (x == other.x && y == other.y && z < other.z);
    }
    inline bool operator<=(const Vec3i& other) const {
        return *this < other || *this == other;
    }
    inline bool operator>(const Vec3i& other) const {
        return !(*this <= other);
    }
    inline bool operator>=(const Vec3i& other) const {
        return !(*this < other);
    }
    inline Vec3i operator+(const Vec3i& other) const {
        return Vec3i(x + other.x, y + other.y, z + other.z);
    }
    inline Vec3i operator-(const Vec3i& other) const {
        return Vec3i(x - other.x, y - other.y, z - other.z);
    }
    inline Vec3i operator*(int scalar) const {
        return Vec3i(x * scalar, y * scalar, z * scalar);
    }
    inline Vec3i operator/(int scalar) const {
        return Vec3i(x / scalar, y / scalar, z / scalar);
    }
    inline Vec3i& operator+=(const Vec3i& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    inline Vec3i& operator-=(const Vec3i& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }
    inline Vec3i& operator*=(int scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }
    inline Vec3i& operator/=(int scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }
    inline friend std::ostream& operator<<(std::ostream& os, const Vec3i& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
};

// ================= Chunk =================

struct Chunk {
    static constexpr int SIZE = 32;
    static constexpr int VOXEL_COUNT = SIZE * SIZE * SIZE;  // 32768
    static constexpr int WORD_COUNT = VOXEL_COUNT / 64;     // 512

    uint64_t data[WORD_COUNT] = {};

    inline int index(int x, int y, int z) const {
        return (z * SIZE + y) * SIZE + x;
    }

    inline void set(int x, int y, int z) {
        int i = index(x, y, z);
        data[i >> 6] |= (1ULL << (i & 63));
    }

    inline bool get(int x, int y, int z) const {
        int i = index(x, y, z);
        return (data[i >> 6] >> (i & 63)) & 1ULL;
    }

    inline void clear(int x, int y, int z) {
        int i = index(x, y, z);
        data[i >> 6] &= ~(1ULL << (i & 63));
    }

    inline bool empty() const {
        for (int i = 0; i < WORD_COUNT; i++)
            if (data[i])
                return false;
        return true;
    }
};

// ================= Hash =================

inline uint64_t packChunkKey(int x, int y, int z) {
    return (uint64_t(uint32_t(x)) << 42) | (uint64_t(uint32_t(y)) << 21) |
           (uint64_t(uint32_t(z)));
}

// ================= Hybrid Segment =================

struct HybridSegment {
    kigstudio::Plane<float> plane;
    collision::CollisionGroup collision_group;
    bool enable_plane = true;
    bool enable_collision = true;
    bool use_plane_positive = true;
    bool use_collision_inside = true;
};

// ================= Grid =================

class VoxelGrid {
   public:
    std::unordered_map<uint64_t, Chunk> chunks;
    vec3<float> global_position = {0.f, 0.f, 0.f};  // 空间的全局位置
    vec3<float> voxel_size = {1.f, 1.f, 1.f};       // 每个小立方体的边长
    // ============ 插入 ============
    inline void insert(int x, int y, int z) {
        int cx = x >> 5;
        int cy = y >> 5;
        int cz = z >> 5;

        int lx = x & 31;
        int ly = y & 31;
        int lz = z & 31;

        uint64_t key = packChunkKey(cx, cy, cz);
        chunks[key].set(lx, ly, lz);
    }

    inline void insert(const Vec3i& p) { insert(p.x, p.y, p.z); }

    // ============ 查询 ============
    inline bool contains(int x, int y, int z) const {
        int cx = x >> 5;
        int cy = y >> 5;
        int cz = z >> 5;

        int lx = x & 31;
        int ly = y & 31;
        int lz = z & 31;

        uint64_t key = packChunkKey(cx, cy, cz);
        auto it = chunks.find(key);
        if (it == chunks.end())
            return false;

        return it->second.get(lx, ly, lz);
    }
    inline bool find(const Vec3i& point) const {
        return contains(point.x, point.y, point.z);
    }
    inline int num_chunk() const { return static_cast<int>(chunks.size()); }
    inline vec3<float> voxelCenterToWorld(const Vec3i& voxel) const {
        return vec3<float>((voxel.x + 0.5f) * voxel_size.x + global_position.x,
                           (voxel.y + 0.5f) * voxel_size.y + global_position.y,
                           (voxel.z + 0.5f) * voxel_size.z + global_position.z);
    }
    inline Vec3i worldToVoxel(const vec3<float>& world) const {
        return Vec3i(
            static_cast<int32_t>(
                std::floor((world.x - global_position.x) / voxel_size.x)),
            static_cast<int32_t>(
                std::floor((world.y - global_position.y) / voxel_size.y)),
            static_cast<int32_t>(
                std::floor((world.z - global_position.z) / voxel_size.z)));
    }
    inline bool containsWorldPoint(const vec3<float>& world) const {
        const Vec3i voxel = worldToVoxel(world);
        return contains(voxel.x, voxel.y, voxel.z);
    }

    // ============ 删除 ============
    inline void remove(int x, int y, int z) {
        int cx = x >> 5;
        int cy = y >> 5;
        int cz = z >> 5;

        int lx = x & 31;
        int ly = y & 31;
        int lz = z & 31;

        uint64_t key = packChunkKey(cx, cy, cz);
        auto it = chunks.find(key);
        if (it == chunks.end())
            return;

        it->second.clear(lx, ly, lz);

        if (it->second.empty())
            chunks.erase(it);
    }

    // ================= ITERATOR =================

    class Iterator {
        using ChunkIter = std::unordered_map<uint64_t, Chunk>::const_iterator;

        ChunkIter it, end;
        int word = 0;
        uint64_t bits = 0;
        int bit = 0;

        Vec3i current;
        bool valid = false;

        int cx, cy, cz;

        inline static void unpackKey(uint64_t key, int& x, int& y, int& z) {
            x = int(key >> 42);
            y = int((key >> 21) & ((1 << 21) - 1));
            z = int(key & ((1 << 21) - 1));
        }

        inline void advance() {
            valid = false;

            while (true) {
                // ===== 1. 结束条件 =====
                if (it == end) {
                    return;
                }

                // ===== 2. word 扫完，进入下一个 chunk =====
                if (word >= Chunk::WORD_COUNT) {
                    ++it;
                    word = 0;
                    bits = 0;

                    if (it == end) {
                        return;
                    }
                    continue;
                }

                // ===== 3. 如果当前没有 bits，加载当前 word =====
                if (bits == 0) {
                    bits = it->second.data[word];

                    if (bits == 0) {
                        // 当前 word 为空 → 跳下一个
                        word++;
                        continue;
                    }
                }

                // ===== 4. 提取最低位 bit =====
                int current_word = word;  // ⚠️ 必须保存（因为 word 可能会递增）

                bit = std::countr_zero(bits);
                bits &= (bits - 1);  // 清掉最低位的 1

                // ===== 5. 如果这个 word 用完了，推进到下一个 =====
                if (bits == 0) {
                    word++;
                }

                // ===== 6. 计算 voxel index =====
                int idx = (current_word << 6) + bit;

                int lx = idx & 31;
                int ly = (idx >> 5) & 31;
                int lz = (idx >> 10) & 31;

                // ===== 7. 解 chunk 坐标 =====
                unpackKey(it->first, cx, cy, cz);

                current = {(cx << 5) + lx, (cy << 5) + ly, (cz << 5) + lz};

                valid = true;
                return;
            }
        }

       public:
        inline Iterator(ChunkIter begin, ChunkIter end) : it(begin), end(end) {
            if (it != end)
                advance();
        }

        inline Vec3i operator*() const { return current; }

        inline Iterator& operator++() {
            advance();
            return *this;
        }

        inline bool operator!=(const Iterator& o) const { return it != o.it; }
    };

    inline Iterator begin() const {
        return Iterator(chunks.begin(), chunks.end());
    }

    inline Iterator end() const { return Iterator(chunks.end(), chunks.end()); }

    // ================= 集合操作 =================

    inline VoxelGrid unionWith_local(const VoxelGrid& other) const {
        VoxelGrid r = *this;

        for (auto& [key, chunk] : other.chunks) {
            auto& dst = r.chunks[key];
            for (int i = 0; i < Chunk::WORD_COUNT; i++)
                dst.data[i] |= chunk.data[i];
        }
        return r;
    }

    inline VoxelGrid intersection_local(const VoxelGrid& other) const {
        VoxelGrid r;

        for (auto& [key, chunk] : chunks) {
            auto it = other.chunks.find(key);
            if (it == other.chunks.end())
                continue;

            Chunk out;
            for (int i = 0; i < Chunk::WORD_COUNT; i++)
                out.data[i] = chunk.data[i] & it->second.data[i];

            if (!out.empty())
                r.chunks[key] = out;
        }
        return r;
    }

    inline VoxelGrid difference_local(const VoxelGrid& other) const {
        VoxelGrid r;

        for (auto& [key, chunk] : chunks) {
            Chunk out = chunk;

            auto it = other.chunks.find(key);
            if (it != other.chunks.end()) {
                for (int i = 0; i < Chunk::WORD_COUNT; i++)
                    out.data[i] &= ~it->second.data[i];
            }

            if (!out.empty())
                r.chunks[key] = out;
        }
        return r;
    }

    inline VoxelGrid unionWith(const VoxelGrid& other) const {
        VoxelGrid r = *this;

        for (const auto& voxel : other) {
            const vec3<float> world = other.voxelCenterToWorld(voxel);
            r.insert(r.worldToVoxel(world));
        }
        return r;
    }

    inline VoxelGrid intersection(const VoxelGrid& other) const {
        VoxelGrid r;
        r.global_position = global_position;
        r.voxel_size = voxel_size;

        for (const auto& voxel : *this) {
            const vec3<float> world = voxelCenterToWorld(voxel);
            if (other.containsWorldPoint(world)) {
                r.insert(voxel);
            }
        }
        return r;
    }
    inline VoxelGrid intersection(
        const collision::CollisionGroup& other) const {
        VoxelGrid r;
        r.global_position = global_position;
        r.voxel_size = voxel_size;

        for (const auto& voxel : *this) {
            const vec3<float> world = voxelCenterToWorld(voxel);
            if (other.contains(world)) {
                r.insert(voxel);
            }
        }
        return r;
    }

    inline VoxelGrid difference(const VoxelGrid& other) const {
        VoxelGrid r;
        r.global_position = global_position;
        r.voxel_size = voxel_size;

        for (const auto& voxel : *this) {
            const vec3<float> world = voxelCenterToWorld(voxel);
            if (!other.containsWorldPoint(world)) {
                r.insert(voxel);
            }
        }
        return r;
    }
    inline VoxelGrid difference(const collision::CollisionGroup& other) const {
        VoxelGrid r;
        r.global_position = global_position;
        r.voxel_size = voxel_size;

        for (const auto& voxel : *this) {
            const vec3<float> world = voxelCenterToWorld(voxel);
            if (!other.contains(world)) {
                r.insert(voxel);
            }
        }
        return r;
    }

    inline VoxelGrid dilate(int radius = 1, bool use_26_neighbors = true) const {
        if (radius <= 0)
            return *this;

        VoxelGrid current = *this;
        for (int step = 0; step < radius; ++step) {
            VoxelGrid next;
            next.global_position = global_position;
            next.voxel_size = voxel_size;

            for (const auto& voxel : current) {
                next.insert(voxel);

                for (int dz = -1; dz <= 1; ++dz) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            if (dx == 0 && dy == 0 && dz == 0)
                                continue;
                            if (!use_26_neighbors &&
                                std::abs(dx) + std::abs(dy) + std::abs(dz) != 1)
                                continue;

                            next.insert(voxel.x + dx,
                                        voxel.y + dy,
                                        voxel.z + dz);
                        }
                    }
                }
            }

            current = std::move(next);
        }

        return current;
    }

    inline VoxelGrid erode(int radius = 1, bool use_26_neighbors = true) const {
        if (radius <= 0)
            return *this;

        VoxelGrid current = *this;
        for (int step = 0; step < radius; ++step) {
            VoxelGrid next;
            next.global_position = global_position;
            next.voxel_size = voxel_size;

            for (const auto& voxel : current) {
                bool keep = true;

                for (int dz = -1; dz <= 1 && keep; ++dz) {
                    for (int dy = -1; dy <= 1 && keep; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            if (dx == 0 && dy == 0 && dz == 0)
                                continue;
                            if (!use_26_neighbors &&
                                std::abs(dx) + std::abs(dy) + std::abs(dz) != 1)
                                continue;

                            if (!current.contains(voxel.x + dx,
                                                  voxel.y + dy,
                                                  voxel.z + dz)) {
                                keep = false;
                                break;
                            }
                        }
                    }
                }

                if (keep)
                    next.insert(voxel);
            }

            current = std::move(next);
        }

        return current;
    }

    inline VoxelGrid getSurfaceVoxels(bool use_26_neighbors = true) const {
        VoxelGrid surface;
        surface.global_position = global_position;
        surface.voxel_size = voxel_size;

        for (const auto& voxel : *this) {
            bool touches_outside = false;

            for (int dz = -1; dz <= 1 && !touches_outside; ++dz) {
                for (int dy = -1; dy <= 1 && !touches_outside; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0 && dz == 0)
                            continue;
                        if (!use_26_neighbors &&
                            std::abs(dx) + std::abs(dy) + std::abs(dz) != 1)
                            continue;

                        if (!contains(voxel.x + dx,
                                      voxel.y + dy,
                                      voxel.z + dz)) {
                            touches_outside = true;
                            break;
                        }
                    }
                }
            }

            if (touches_outside)
                surface.insert(voxel);
        }

        return surface;
    }

    inline VoxelGrid getOuterAirSurfaceVoxels(
        bool use_26_neighbors = true) const {
        VoxelGrid air;
        air.global_position = global_position;
        air.voxel_size = voxel_size;

        for (const auto& voxel : *this) {
            for (int dz = -1; dz <= 1; ++dz) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0 && dz == 0)
                            continue;
                        if (!use_26_neighbors &&
                            std::abs(dx) + std::abs(dy) + std::abs(dz) != 1)
                            continue;

                        const Vec3i neighbor(voxel.x + dx,
                                             voxel.y + dy,
                                             voxel.z + dz);
                        if (!contains(neighbor.x, neighbor.y, neighbor.z))
                            air.insert(neighbor);
                    }
                }
            }
        }

        return air;
    }

    inline VoxelGrid detectWeakVoxels(int radius = 1,
                                      bool use_26_neighbors = true) const {
        if (radius <= 0) {
            VoxelGrid r;
            r.global_position = global_position;
            r.voxel_size = voxel_size;
            return r;
        }

        const VoxelGrid opened =
            erode(radius, use_26_neighbors).dilate(radius, use_26_neighbors);
        VoxelGrid weak = difference_local(opened);
        weak.global_position = global_position;
        weak.voxel_size = voxel_size;
        return weak;
    }

    inline VoxelGrid extractSkeletonByMaximalBalls(
        int min_radius = 1,
        bool use_26_neighbors = true) const {
        VoxelGrid skeleton;
        skeleton.global_position = global_position;
        skeleton.voxel_size = voxel_size;

        if (chunks.empty())
            return skeleton;

        constexpr int CHAMFER_AXIS = 3;
        constexpr int CHAMFER_EDGE = 4;
        constexpr int CHAMFER_CORNER = 5;
        const int min_radius_clamped =
            (min_radius < 1 ? 1 : min_radius) * CHAMFER_AXIS;
        constexpr int INF = std::numeric_limits<int>::max() / 4;
        struct Vec3iHash {
            inline std::size_t operator()(const Vec3i& p) const {
                std::size_t h = 1469598103934665603ull;
                auto mix = [&h](int32_t v) {
                    h ^= static_cast<uint32_t>(v);
                    h *= 1099511628211ull;
                };
                mix(p.x);
                mix(p.y);
                mix(p.z);
                return h;
            }
        };

        std::unordered_map<Vec3i, int, Vec3iHash> distance;
        struct QueueNode {
            Vec3i voxel;
            int distance = 0;
        };
        struct QueueNodeGreater {
            inline bool operator()(const QueueNode& a,
                                   const QueueNode& b) const {
                return a.distance > b.distance;
            }
        };
        std::priority_queue<QueueNode,
                            std::vector<QueueNode>,
                            QueueNodeGreater>
            queue;

        auto isNeighborEnabled = [use_26_neighbors](int dx, int dy, int dz) {
            if (dx == 0 && dy == 0 && dz == 0)
                return false;
            if (use_26_neighbors)
                return true;
            return std::abs(dx) + std::abs(dy) + std::abs(dz) == 1;
        };
        auto chamferWeight = [](int dx, int dy, int dz) {
            const int axis_count =
                (dx != 0 ? 1 : 0) + (dy != 0 ? 1 : 0) + (dz != 0 ? 1 : 0);
            if (axis_count == 1)
                return CHAMFER_AXIS;
            if (axis_count == 2)
                return CHAMFER_EDGE;
            return CHAMFER_CORNER;
        };

        for (const auto& voxel : *this) {
            distance[voxel] = INF;

            int boundary_distance = INF;
            for (int dz = -1; dz <= 1; ++dz) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (!isNeighborEnabled(dx, dy, dz))
                            continue;

                        if (!contains(voxel.x + dx,
                                      voxel.y + dy,
                                      voxel.z + dz)) {
                            const int neighbor_distance =
                                chamferWeight(dx, dy, dz);
                            if (neighbor_distance < boundary_distance)
                                boundary_distance = neighbor_distance;
                        }
                    }
                }
            }

            if (boundary_distance != INF) {
                distance[voxel] = boundary_distance;
                queue.push({voxel, boundary_distance});
            }
        }

        while (!queue.empty()) {
            const QueueNode node = queue.top();
            queue.pop();

            const Vec3i voxel = node.voxel;
            const int base_distance = distance[voxel];
            if (node.distance != base_distance)
                continue;

            for (int dz = -1; dz <= 1; ++dz) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (!isNeighborEnabled(dx, dy, dz))
                            continue;

                        const Vec3i neighbor(voxel.x + dx,
                                             voxel.y + dy,
                                             voxel.z + dz);
                        auto it = distance.find(neighbor);
                        if (it == distance.end())
                            continue;

                        const int next_distance =
                            base_distance + chamferWeight(dx, dy, dz);
                        if (next_distance < it->second) {
                            it->second = next_distance;
                            queue.push({neighbor, next_distance});
                        }
                    }
                }
            }
        }

        for (const auto& voxel : *this) {
            const int radius = distance[voxel];
            if (radius < min_radius_clamped)
                continue;

            bool covered_by_larger_ball = false;
            for (int dz = -1; dz <= 1 && !covered_by_larger_ball; ++dz) {
                for (int dy = -1; dy <= 1 && !covered_by_larger_ball; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (!isNeighborEnabled(dx, dy, dz))
                            continue;

                        const Vec3i neighbor(voxel.x + dx,
                                             voxel.y + dy,
                                             voxel.z + dz);
                        const auto it = distance.find(neighbor);
                        if (it == distance.end())
                            continue;

                        const int step_distance = chamferWeight(dx, dy, dz);
                        if (it->second >= radius + step_distance) {
                            covered_by_larger_ball = true;
                            break;
                        }
                    }
                }
            }

            if (!covered_by_larger_ball)
                skeleton.insert(voxel);
        }

        return skeleton;
    }

    inline std::tuple<VoxelGrid, VoxelGrid> segment(
        const collision::CollisionGroup& other) const {
        VoxelGrid positive_side;
        VoxelGrid negative_side;

        positive_side.global_position = global_position;
        positive_side.voxel_size = voxel_size;
        negative_side.global_position = global_position;
        negative_side.voxel_size = voxel_size;

        for (const auto& voxel : *this) {
            const vec3<float> world_position(
                voxel.x * voxel_size.x + global_position.x,
                voxel.y * voxel_size.y + global_position.y,
                voxel.z * voxel_size.z + global_position.z);

            if (other.contains(world_position)) {
                positive_side.insert(voxel);
            } else {
                negative_side.insert(voxel);
            }
        }

        return {positive_side, negative_side};
    }

    inline std::tuple<VoxelGrid, VoxelGrid> segment(
        const Plane<float>& other) const {
        VoxelGrid positive_side;
        VoxelGrid negative_side;

        positive_side.global_position = global_position;
        positive_side.voxel_size = voxel_size;
        negative_side.global_position = global_position;
        negative_side.voxel_size = voxel_size;

        for (const auto& voxel : *this) {
            const vec3<float> world_position(
                voxel.x * voxel_size.x + global_position.x,
                voxel.y * voxel_size.y + global_position.y,
                voxel.z * voxel_size.z + global_position.z);

            if (other.getSide(world_position)) {
                positive_side.insert(voxel);
            } else {
                negative_side.insert(voxel);
            }
        }

        return {positive_side, negative_side};
    }

    inline std::tuple<VoxelGrid, VoxelGrid> segment(
        const concave::Base& other) const {
        VoxelGrid positive_side;
        VoxelGrid negative_side;

        positive_side.global_position = global_position;
        positive_side.voxel_size = voxel_size;
        negative_side.global_position = global_position;
        negative_side.voxel_size = voxel_size;

        std::string err;
        if (!other.check(err)) {
            std::cout << "Concave shape check failed: " << err << std::endl;
            return {positive_side, negative_side};
        }

        for (const auto& voxel : *this) {
            const vec3<float> world_position(
                voxel.x * voxel_size.x + global_position.x,
                voxel.y * voxel_size.y + global_position.y,
                voxel.z * voxel_size.z + global_position.z);

            if (other.contains(world_position)) {
                positive_side.insert(voxel);
            } else {
                negative_side.insert(voxel);
            }
        }

        return {positive_side, negative_side};
    }

    inline std::tuple<VoxelGrid, VoxelGrid> segment(
        const HybridSegment& other) const {
        VoxelGrid positive_side;
        VoxelGrid negative_side;

        positive_side.global_position = global_position;
        positive_side.voxel_size = voxel_size;
        negative_side.global_position = global_position;
        negative_side.voxel_size = voxel_size;

        for (const auto& voxel : *this) {
            const vec3<float> world_position(
                voxel.x * voxel_size.x + global_position.x,
                voxel.y * voxel_size.y + global_position.y,
                voxel.z * voxel_size.z + global_position.z);

            bool keep = true;
            if (other.enable_plane) {
                if (other.plane.getSide(world_position) !=
                    other.use_plane_positive) {
                    keep = false;
                }
            }
            if (other.enable_collision) {
                if (other.collision_group.contains(world_position) !=
                    other.use_collision_inside) {
                    keep = false;
                }
            }

            if (keep) {
                positive_side.insert(voxel);
            } else {
                negative_side.insert(voxel);
            }
        }

        return {positive_side, negative_side};
    }
};
}  // namespace voxel

inline bool save(const std::filesystem::path& path, const voxel::VoxelGrid& grid, std::string* error = nullptr) {
#ifdef _WIN32
    FILE* fp = _wfopen(path.wstring().c_str(), L"wb");
    if (!fp) {
        if (error) *error = "open file failed";
        return false;
    }
    auto write = [&](const void* data, size_t size) -> bool {
        return fwrite(data, 1, size, fp) == size;
    };
    bool ok = true;
    const char magic[8] = {'V', 'X', 'G', 'R', 'I', 'D', '1', '\0'};
    uint32_t version = 1;
    ok = ok && write(magic, 8);
    ok = ok && write(&version, sizeof(version));
    ok = ok && write(&grid.global_position, sizeof(grid.global_position));
    ok = ok && write(&grid.voxel_size, sizeof(grid.voxel_size));
    uint32_t chunk_count = (uint32_t)grid.chunks.size();
    ok = ok && write(&chunk_count, sizeof(chunk_count));
    std::vector<uint8_t> raw;
    raw.reserve(chunk_count * (sizeof(uint64_t) + sizeof(voxel::Chunk)));
    for (const auto& [key, chunk] : grid.chunks) {
        size_t offset = raw.size();
        raw.resize(offset + sizeof(uint64_t) + sizeof(voxel::Chunk));
        std::memcpy(raw.data() + offset, &key, sizeof(uint64_t));
        std::memcpy(raw.data() + offset + sizeof(uint64_t), chunk.data,
                    sizeof(chunk.data));
    }
    std::vector<uint8_t> compressed;
    if (!zlibCompress(raw, compressed)) {
        if (error) *error = "zlib compress failed";
        fclose(fp);
        return false;
    }
    uint32_t comp_size = (uint32_t)compressed.size();
    uint32_t raw_size = (uint32_t)raw.size();
    ok = ok && write(&comp_size, sizeof(comp_size));
    ok = ok && write(&raw_size, sizeof(raw_size));
    ok = ok && write(compressed.data(), comp_size);
    fclose(fp);
    if (!ok && error) *error = "write file failed";
    return ok;
#else
    std::ofstream ofs(path.c_str(), std::ios::binary);
    if (!ofs) {
        if (error) *error = "open file failed";
        return false;
    }
    const char magic[8] = {'V', 'X', 'G', 'R', 'I', 'D', '1', '\0'};
    uint32_t version = 1;
    ofs.write(magic, 8);
    ofs.write(reinterpret_cast<char*>(&version), sizeof(version));
    ofs.write(reinterpret_cast<const char*>(&grid.global_position),
              sizeof(grid.global_position));
    ofs.write(reinterpret_cast<const char*>(&grid.voxel_size),
              sizeof(grid.voxel_size));
    uint32_t chunk_count = (uint32_t)grid.chunks.size();
    ofs.write(reinterpret_cast<char*>(&chunk_count), sizeof(chunk_count));
    std::vector<uint8_t> raw;
    raw.reserve(chunk_count * (sizeof(uint64_t) + sizeof(voxel::Chunk)));
    for (const auto& [key, chunk] : grid.chunks) {
        size_t offset = raw.size();
        raw.resize(offset + sizeof(uint64_t) + sizeof(voxel::Chunk));
        std::memcpy(raw.data() + offset, &key, sizeof(uint64_t));
        std::memcpy(raw.data() + offset + sizeof(uint64_t), chunk.data,
                    sizeof(chunk.data));
    }
    std::vector<uint8_t> compressed;
    if (!zlibCompress(raw, compressed)) {
        if (error) *error = "zlib compress failed";
        return false;
    }
    uint32_t comp_size = (uint32_t)compressed.size();
    uint32_t raw_size = (uint32_t)raw.size();
    ofs.write(reinterpret_cast<char*>(&comp_size), sizeof(comp_size));
    ofs.write(reinterpret_cast<char*>(&raw_size), sizeof(raw_size));
    ofs.write(reinterpret_cast<char*>(compressed.data()), comp_size);
    return true;
#endif
}

inline bool load(const std::filesystem::path& path, voxel::VoxelGrid& grid) {
#ifdef _WIN32
    FILE* fp = _wfopen(path.wstring().c_str(), L"rb");
    if (!fp) return false;
    auto read = [&](void* data, size_t size) -> bool {
        return fread(data, 1, size, fp) == size;
    };
    char magic[8];
    uint32_t version;
    if (!read(magic, 8) || !read(&version, sizeof(version))) {
        fclose(fp);
        return false;
    }
    if (std::strncmp(magic, "VXGRID1", 7) != 0 || version != 1) {
        fclose(fp);
        return false;
    }
    if (!read(&grid.global_position, sizeof(grid.global_position)) ||
        !read(&grid.voxel_size, sizeof(grid.voxel_size))) {
        fclose(fp);
        return false;
    }
    uint32_t chunk_count;
    if (!read(&chunk_count, sizeof(chunk_count))) {
        fclose(fp);
        return false;
    }
    uint32_t comp_size, raw_size;
    if (!read(&comp_size, sizeof(comp_size)) || !read(&raw_size, sizeof(raw_size))) {
        fclose(fp);
        return false;
    }
    std::vector<uint8_t> compressed(comp_size);
    if (!read(compressed.data(), comp_size)) {
        fclose(fp);
        return false;
    }
    fclose(fp);
    std::vector<uint8_t> raw;
    if (!zlibDecompress(compressed, raw, raw_size)) return false;
    grid.chunks.clear();
    size_t offset = 0;
    for (uint32_t i = 0; i < chunk_count; i++) {
        uint64_t key;
        std::memcpy(&key, raw.data() + offset, sizeof(uint64_t));
        offset += sizeof(uint64_t);
        voxel::Chunk chunk;
        std::memcpy(chunk.data, raw.data() + offset, sizeof(chunk.data));
        offset += sizeof(chunk.data);
        if (!chunk.empty()) grid.chunks[key] = chunk;
    }
    return true;
#else
    std::ifstream ifs(path.c_str(), std::ios::binary);
    if (!ifs) return false;
    char magic[8];
    uint32_t version;
    ifs.read(magic, 8);
    ifs.read(reinterpret_cast<char*>(&version), sizeof(version));
    if (std::strncmp(magic, "VXGRID1", 7) != 0 || version != 1)
        return false;
    ifs.read(reinterpret_cast<char*>(&grid.global_position),
             sizeof(grid.global_position));
    ifs.read(reinterpret_cast<char*>(&grid.voxel_size),
             sizeof(grid.voxel_size));
    uint32_t chunk_count;
    ifs.read(reinterpret_cast<char*>(&chunk_count), sizeof(chunk_count));
    uint32_t comp_size, raw_size;
    ifs.read(reinterpret_cast<char*>(&comp_size), sizeof(comp_size));
    ifs.read(reinterpret_cast<char*>(&raw_size), sizeof(raw_size));
    std::vector<uint8_t> compressed(comp_size);
    ifs.read(reinterpret_cast<char*>(compressed.data()), comp_size);
    std::vector<uint8_t> raw;
    if (!zlibDecompress(compressed, raw, raw_size))
        return false;
    grid.chunks.clear();
    size_t offset = 0;
    for (uint32_t i = 0; i < chunk_count; i++) {
        uint64_t key;
        std::memcpy(&key, raw.data() + offset, sizeof(uint64_t));
        offset += sizeof(uint64_t);
        voxel::Chunk chunk;
        std::memcpy(chunk.data, raw.data() + offset, sizeof(chunk.data));
        offset += sizeof(chunk.data);
        if (!chunk.empty()) grid.chunks[key] = chunk;
    }
    return true;
#endif
}
}  // namespace sinriv::kigstudio
