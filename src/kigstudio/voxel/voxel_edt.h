#pragma once
#include "voxel.h"

namespace sinriv::kigstudio::voxel {

struct DenseGrid {
    Vec3i min_bound;
    Vec3i max_bound;

    int sx = 0;
    int sy = 0;
    int sz = 0;

    // occupancy
    std::vector<uint8_t> solid;

    // squared EDT distance
    std::vector<float> dist2;

    static constexpr float INF = 1e20f;

    inline int index(int x, int y, int z) const {
        return (z * sy + y) * sx + x;
    }

    inline bool inBounds(int x, int y, int z) const {
        return x >= 0 && y >= 0 && z >= 0 && x < sx && y < sy && z < sz;
    }

    inline bool getSolid(int x, int y, int z) const {
        return solid[index(x, y, z)] != 0;
    }

    inline void setSolid(int x, int y, int z, bool v) {
        solid[index(x, y, z)] = v ? 1 : 0;
    }

    inline float getDist2(int x, int y, int z) const {
        return dist2[index(x, y, z)];
    }

    inline void setDist2(int x, int y, int z, float v) {
        dist2[index(x, y, z)] = v;
    }

    inline Vec3i denseToWorldVoxel(int x, int y, int z) const {
        return Vec3i(x + min_bound.x, y + min_bound.y, z + min_bound.z);
    }

    inline Vec3i worldVoxelToDense(const Vec3i& p) const {
        return Vec3i(p.x - min_bound.x, p.y - min_bound.y, p.z - min_bound.z);
    }

    inline bool containsWorldVoxel(const Vec3i& p) const {
        return inBounds(p.x - min_bound.x, p.y - min_bound.y,
                        p.z - min_bound.z);
    }
};
struct SkeletonNode {
    Vec3i pos;
    float radius = 0.f;

    std::vector<int> neighbors;
};
struct SkeletonGraph {
    std::vector<SkeletonNode> nodes;

    std::unordered_map<int64_t, int> voxel_to_node;

    static inline int64_t pack(const Vec3i& p) {
        return (int64_t(uint32_t(p.x)) << 42) | (int64_t(uint32_t(p.y)) << 21) |
               int64_t(uint32_t(p.z));
    }

    inline bool contains(const Vec3i& p) const {
        return voxel_to_node.find(pack(p)) != voxel_to_node.end();
    }

    inline int getIndex(const Vec3i& p) const {
        auto it = voxel_to_node.find(pack(p));
        if (it == voxel_to_node.end())
            return -1;
        return it->second;
    }
};
struct DijkstraResult {
    std::vector<float> dist;
    std::vector<int> prev;
    int farthest = -1;
};
DenseGrid buildDenseGrid(const VoxelGrid& grid, int padding = 2);
void edt_1d(const float* f, float* d, int n);
void computeEDT(DenseGrid& grid);
void finalizeEDT(DenseGrid& grid);
bool isRidgeVoxel(const DenseGrid& dense, int x, int y, int z);
SkeletonGraph buildWeightedSkeleton(const DenseGrid& dense);
float edgeCost(const SkeletonNode& a, const SkeletonNode& b);
DijkstraResult runDijkstra(const SkeletonGraph& graph, int start);
std::vector<Vec3i> extractMainPath(const SkeletonGraph& graph);
std::vector<Vec3i> extractWeightedCenterline(const DenseGrid& dense);

// DenseGrid dense = buildDenseGrid(grid);
// computeEDT(dense);
// finalizeEDT(dense);
// std::vector<Vec3i> centerline =
//     extractWeightedCenterline(dense);

}  // namespace sinriv::kigstudio::voxel