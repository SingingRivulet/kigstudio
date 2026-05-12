#include "voxel_edt.h"
#include <queue>

namespace sinriv::kigstudio::voxel {

static inline const Vec3i kDirs26[26] = {
    {-1, -1, -1}, {0, -1, -1}, {1, -1, -1}, {-1, 0, -1}, {0, 0, -1},
    {1, 0, -1},   {-1, 1, -1}, {0, 1, -1},  {1, 1, -1},

    {-1, -1, 0},  {0, -1, 0},  {1, -1, 0},  {-1, 0, 0},  {1, 0, 0},
    {-1, 1, 0},   {0, 1, 0},   {1, 1, 0},

    {-1, -1, 1},  {0, -1, 1},  {1, -1, 1},  {-1, 0, 1},  {0, 0, 1},
    {1, 0, 1},    {-1, 1, 1},  {0, 1, 1},   {1, 1, 1},
};

DenseGrid buildDenseGrid(const VoxelGrid& grid, int padding) {
    DenseGrid dense;

    if (grid.empty())
        return dense;

    // =========================
    // Compute AABB
    // =========================

    Vec3i minv(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(),
               std::numeric_limits<int>::max());

    Vec3i maxv(std::numeric_limits<int>::min(), std::numeric_limits<int>::min(),
               std::numeric_limits<int>::min());

    for (const auto& p : grid) {
        minv.x = std::min(minv.x, p.x);
        minv.y = std::min(minv.y, p.y);
        minv.z = std::min(minv.z, p.z);

        maxv.x = std::max(maxv.x, p.x);
        maxv.y = std::max(maxv.y, p.y);
        maxv.z = std::max(maxv.z, p.z);
    }

    minv -= Vec3i(padding, padding, padding);
    maxv += Vec3i(padding, padding, padding);

    dense.min_bound = minv;
    dense.max_bound = maxv;

    dense.sx = maxv.x - minv.x + 1;
    dense.sy = maxv.y - minv.y + 1;
    dense.sz = maxv.z - minv.z + 1;

    const int total_size = dense.sx * dense.sy * dense.sz;

    dense.solid.resize(total_size, 0);
    dense.dist2.resize(total_size, DenseGrid::INF);

    // =========================
    // Fill occupancy
    // =========================

    for (const auto& p : grid) {
        const Vec3i d = dense.worldVoxelToDense(p);

        dense.setSolid(d.x, d.y, d.z, true);
    }

    // =========================
    // Detect boundary voxels
    // =========================

    static const Vec3i dirs6[6] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
    };

    for (int z = 0; z < dense.sz; z++) {
        for (int y = 0; y < dense.sy; y++) {
            for (int x = 0; x < dense.sx; x++) {
                if (!dense.getSolid(x, y, z))
                    continue;

                bool boundary = false;

                for (const auto& d : dirs6) {
                    const int nx = x + d.x;
                    const int ny = y + d.y;
                    const int nz = z + d.z;

                    if (!dense.inBounds(nx, ny, nz) ||
                        !dense.getSolid(nx, ny, nz)) {
                        boundary = true;
                        break;
                    }
                }

                dense.setDist2(x, y, z, boundary ? 0.f : DenseGrid::INF);
            }
        }
    }

    return dense;
}

// ============================================================
// Felzenszwalb 1D EDT
// ============================================================
void edt_1d(const float* f, float* d, int n) {
    std::vector<int> v(n);
    std::vector<float> z(n + 1);

    constexpr float INF = DenseGrid::INF;

    int k = 0;

    v[0] = 0;
    z[0] = -INF;
    z[1] = INF;

    auto sep = [&](int i, int j) -> float {
        return ((f[j] + float(j * j)) - (f[i] + float(i * i))) /
               (2.f * float(j - i));
    };

    for (int q = 1; q < n; q++) {
        float s = sep(v[k], q);

        while (s <= z[k]) {
            k--;

            if (k < 0) {
                k = 0;
                break;
            }

            s = sep(v[k], q);
        }

        k++;

        v[k] = q;
        z[k] = s;
        z[k + 1] = INF;
    }

    k = 0;

    for (int q = 0; q < n; q++) {
        while (z[k + 1] < q)
            k++;

        const float dx = float(q - v[k]);

        d[q] = dx * dx + f[v[k]];
    }
}

// ============================================================
// 3-pass EDT
// ============================================================
void computeEDT(DenseGrid& grid) {
    constexpr float INF = DenseGrid::INF;

    const int sx = grid.sx;
    const int sy = grid.sy;
    const int sz = grid.sz;

    // ========================================================
    // PASS X
    // ========================================================

    {
        std::vector<float> f(sx);
        std::vector<float> d(sx);

        for (int z = 0; z < sz; z++) {
            for (int y = 0; y < sy; y++) {
                // gather
                for (int x = 0; x < sx; x++) {
                    f[x] = grid.getDist2(x, y, z);
                }

                // edt
                edt_1d(f.data(), d.data(), sx);

                // scatter
                for (int x = 0; x < sx; x++) {
                    grid.setDist2(x, y, z, d[x]);
                }
            }
        }
    }

    // ========================================================
    // PASS Y
    // ========================================================

    {
        std::vector<float> f(sy);
        std::vector<float> d(sy);

        for (int z = 0; z < sz; z++) {
            for (int x = 0; x < sx; x++) {
                // gather
                for (int y = 0; y < sy; y++) {
                    f[y] = grid.getDist2(x, y, z);
                }

                // edt
                edt_1d(f.data(), d.data(), sy);

                // scatter
                for (int y = 0; y < sy; y++) {
                    grid.setDist2(x, y, z, d[y]);
                }
            }
        }
    }

    // ========================================================
    // PASS Z
    // ========================================================

    {
        std::vector<float> f(sz);
        std::vector<float> d(sz);

        for (int y = 0; y < sy; y++) {
            for (int x = 0; x < sx; x++) {
                // gather
                for (int z = 0; z < sz; z++) {
                    f[z] = grid.getDist2(x, y, z);
                }

                // edt
                edt_1d(f.data(), d.data(), sz);

                // scatter
                for (int z = 0; z < sz; z++) {
                    grid.setDist2(x, y, z, d[z]);
                }
            }
        }
    }
}

// ============================================================
// Optional sqrt pass
// ============================================================
void finalizeEDT(DenseGrid& grid) {
    for (float& v : grid.dist2) {
        if (v >= DenseGrid::INF * 0.5f)
            continue;

        v = std::sqrt(v);
    }
}

bool isRidgeVoxel(const DenseGrid& dense, int x, int y, int z) {
    if (!dense.getSolid(x, y, z))
        return false;

    const float r = dense.getDist2(x, y, z);

    if (r <= 1.f)
        return false;

    int larger_count = 0;

    for (const auto& d : kDirs26) {
        int nx = x + d.x;
        int ny = y + d.y;
        int nz = z + d.z;

        if (!dense.inBounds(nx, ny, nz))
            continue;

        if (!dense.getSolid(nx, ny, nz))
            continue;

        float nr = dense.getDist2(nx, ny, nz);

        if (nr > r)
            larger_count++;
    }

    // ridge / plateau
    return larger_count <= 2;
}

SkeletonGraph buildWeightedSkeleton(const DenseGrid& dense) {
    SkeletonGraph graph;

    // =====================================
    // Add nodes
    // =====================================

    for (int z = 0; z < dense.sz; z++) {
        for (int y = 0; y < dense.sy; y++) {
            for (int x = 0; x < dense.sx; x++) {
                if (!isRidgeVoxel(dense, x, y, z))
                    continue;

                SkeletonNode node;

                node.pos = dense.denseToWorldVoxel(x, y, z);
                node.radius = dense.getDist2(x, y, z);

                int index = (int)graph.nodes.size();

                graph.nodes.push_back(node);
                graph.voxel_to_node[SkeletonGraph::pack(node.pos)] = index;
            }
        }
    }

    // =====================================
    // Build edges
    // =====================================

    for (int i = 0; i < (int)graph.nodes.size(); i++) {
        const Vec3i p = graph.nodes[i].pos;

        for (const auto& d : kDirs26) {
            Vec3i np = p + d;

            int ni = graph.getIndex(np);

            if (ni < 0)
                continue;

            if (ni == i)
                continue;

            graph.nodes[i].neighbors.push_back(ni);
        }
    }

    return graph;
}

float edgeCost(const SkeletonNode& a, const SkeletonNode& b) {
    Vec3i d = b.pos - a.pos;

    float dist = std::sqrt(float(d.x * d.x + d.y * d.y + d.z * d.z));

    float radius = std::min(a.radius, b.radius);

    return dist / (radius + 1e-4f);
}

DijkstraResult runDijkstra(const SkeletonGraph& graph, int start) {
    struct QueueNode {
        int index;
        float dist;

        bool operator<(const QueueNode& r) const { return dist > r.dist; }
    };

    DijkstraResult result;

    const int n = (int)graph.nodes.size();

    result.dist.resize(n, 1e20f);
    result.prev.resize(n, -1);

    std::priority_queue<QueueNode> pq;

    result.dist[start] = 0.f;

    pq.push({start, 0.f});

    while (!pq.empty()) {
        QueueNode q = pq.top();
        pq.pop();

        if (q.dist != result.dist[q.index])
            continue;

        const auto& node = graph.nodes[q.index];

        for (int ni : node.neighbors) {
            float nd = q.dist + edgeCost(node, graph.nodes[ni]);

            if (nd >= result.dist[ni])
                continue;

            result.dist[ni] = nd;
            result.prev[ni] = q.index;

            pq.push({ni, nd});
        }
    }

    // farthest

    float best = -1.f;

    for (int i = 0; i < n; i++) {
        if (result.dist[i] >= 1e19f)
            continue;

        if (result.dist[i] > best) {
            best = result.dist[i];
            result.farthest = i;
        }
    }

    return result;
}

std::vector<Vec3i> extractMainPath(const SkeletonGraph& graph) {
    std::vector<Vec3i> empty;

    if (graph.nodes.empty())
        return empty;

    // =====================================
    // Pass 1
    // =====================================

    DijkstraResult d0 = runDijkstra(graph, 0);

    int A = d0.farthest;

    // =====================================
    // Pass 2
    // =====================================

    DijkstraResult d1 = runDijkstra(graph, A);

    int B = d1.farthest;

    // =====================================
    // Reconstruct path
    // =====================================

    std::vector<Vec3i> path;

    int cur = B;

    while (cur >= 0) {
        path.push_back(graph.nodes[cur].pos);

        cur = d1.prev[cur];
    }

    std::reverse(path.begin(), path.end());

    return path;
}

std::vector<Vec3i> extractWeightedCenterline(const DenseGrid& dense) {
    SkeletonGraph graph = buildWeightedSkeleton(dense);

    return extractMainPath(graph);
}

}  // namespace sinriv::kigstudio::voxel
