#pragma once
#include "kigstudio/utils/plane.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/utils/KDTree.h"
#include "kigstudio/voxel/collision.h"
#include "kigstudio/voxel/concave.h"

#include <bit>
#include <bitset>
#include <cmath>
#include <cstdint>
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

    inline void insert(const std::vector<Vec3i>& points) {
        std::unordered_map<uint64_t, std::vector<Vec3i>> grouped;
        grouped.reserve(points.size() / 8 + 1);
        for (const auto& p : points) {
            int cx = p.x >> 5, cy = p.y >> 5, cz = p.z >> 5;
            grouped[packChunkKey(cx, cy, cz)].push_back(p);
        }
        for (auto& [key, pts] : grouped) {
            auto& chunk = chunks[key];
            for (const auto& p : pts) {
                chunk.set(p.x & 31, p.y & 31, p.z & 31);
            }
        }
    }

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
    inline void remove(const Vec3i& point) {
        remove(point.x, point.y, point.z);
    }
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

    inline void remove(const std::vector<Vec3i>& points) {
        std::unordered_map<uint64_t, std::vector<Vec3i>> grouped;
        grouped.reserve(points.size() / 8 + 1);
        for (const auto& p : points) {
            int cx = p.x >> 5, cy = p.y >> 5, cz = p.z >> 5;
            grouped[packChunkKey(cx, cy, cz)].push_back(p);
        }
        for (auto& [key, pts] : grouped) {
            auto it = chunks.find(key);
            if (it == chunks.end()) continue;
            for (const auto& p : pts) {
                it->second.clear(p.x & 31, p.y & 31, p.z & 31);
            }
            if (it->second.empty()) {
                chunks.erase(it);
            }
        }
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
    VoxelGrid unionWith_local(const VoxelGrid& other) const;
    VoxelGrid intersection_local(const VoxelGrid& other) const;
    VoxelGrid difference_local(const VoxelGrid& other) const;
    VoxelGrid unionWith(const VoxelGrid& other) const;
    VoxelGrid intersection(const VoxelGrid& other) const;
    VoxelGrid intersection(
        const collision::CollisionGroup& other) const;
    VoxelGrid difference(const VoxelGrid& other) const;
    VoxelGrid difference(const collision::CollisionGroup& other) const;

    // ================= 骨架提取 =================
    VoxelGrid dilate(int radius = 1, bool use_26_neighbors = true) const;
    VoxelGrid erode(int radius = 1, bool use_26_neighbors = true) const;
    VoxelGrid getSurfaceVoxels(bool use_26_neighbors = true) const;
    VoxelGrid getOuterAirSurfaceVoxels(
        bool use_26_neighbors = true) const;
    VoxelGrid detectWeakVoxels(int radius = 1,
                                      bool use_26_neighbors = true) const;
    VoxelGrid extractSkeletonByMaximalBalls(
        int min_radius = 1,
        bool use_26_neighbors = true) const;

    std::tuple<VoxelGrid, VoxelGrid> segment(
        const Plane<float>& other) const;
    std::tuple<VoxelGrid, VoxelGrid> segment(
        const collision::CollisionGroup& other) const;
    std::tuple<VoxelGrid, VoxelGrid> segment(
        const concave::Base& other) const;
    std::tuple<VoxelGrid, VoxelGrid> segment(
        const HybridSegment& other) const;
    bool rayOccluded(const vec3<float>& origin,
                     const vec3<float>& target) const;
    VoxelGrid extractLitVoxels(const vec3<float>& lightPos) const;
    std::tuple<VoxelGrid, VoxelGrid> bfsSplit(
        const std::vector<Vec3i>& seeds,
        int max_distance,
        bool use_26_neighbors = false) const;
    std::vector<VoxelGrid> splitDisconnected(
        bool use_26_neighbors = false) const;
};
}  // namespace voxel

bool save(const std::filesystem::path& path, const voxel::VoxelGrid& grid, std::string* error = nullptr);
bool load(const std::filesystem::path& path, voxel::VoxelGrid& grid);

}  // namespace sinriv::kigstudio
