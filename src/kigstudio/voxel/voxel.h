#pragma once
#include "kigstudio/utils/vec3.h"
#include "kigstudio/utils/plane.h"

#include <bit>
#include <bitset>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <stack>

namespace sinriv::kigstudio::voxel {
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

// ================= Grid =================

class VoxelGrid {

   public:
    std::unordered_map<uint64_t, Chunk> chunks;
    vec3<float> global_position = {0.f, 0.f, 0.f}; // 空间的全局位置
    vec3<float> voxel_size = {1.f, 1.f, 1.f}; // 每个小立方体的边长
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
    inline int num_chunk() const { return chunks.size(); }
    inline vec3<float> voxelCenterToWorld(const Vec3i& voxel) const {
        return vec3<float>((voxel.x + 0.5f) * voxel_size.x + global_position.x,
                           (voxel.y + 0.5f) * voxel_size.y + global_position.y,
                           (voxel.z + 0.5f) * voxel_size.z + global_position.z);
    }
    inline Vec3i worldToVoxel(const vec3<float>& world) const {
        return Vec3i(
            static_cast<int32_t>(std::floor((world.x - global_position.x) / voxel_size.x)),
            static_cast<int32_t>(std::floor((world.y - global_position.y) / voxel_size.y)),
            static_cast<int32_t>(std::floor((world.z - global_position.z) / voxel_size.z)));
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
                if (it == end)
                    return;

                if (bits == 0) {
                    while (word < Chunk::WORD_COUNT) {
                        bits = it->second.data[word];
                        if (bits)
                            break;
                        word++;
                    }

                    if (word == Chunk::WORD_COUNT) {
                        ++it;
                        word = 0;
                        continue;
                    }
                }

                bit = std::countr_zero(bits);
                bits &= bits - 1;

                int idx = (word << 6) + bit;

                int lx = idx & 31;
                int ly = (idx >> 5) & 31;
                int lz = (idx >> 10) & 31;

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

        inline bool operator!=(const Iterator& o) const {
            return valid != o.valid || it != o.it;
        }
    };

    inline Iterator begin() const { return Iterator(chunks.begin(), chunks.end()); }

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
    inline std::tuple<VoxelGrid, VoxelGrid> segment(const Plane<float>& other) const {
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
};

}  // namespace sinriv::kigstudio::voxel
