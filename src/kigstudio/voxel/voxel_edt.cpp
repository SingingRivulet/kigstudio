#include "voxel_EDT.h"
#include <algorithm>
#include <queue>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

namespace sinriv::kigstudio::voxel {

static const Vec3i kDirs26[26] = {
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

namespace {
struct LocalRouteNode {
    int index = -1;
    float cost = 0.f;

    bool operator<(const LocalRouteNode& r) const { return cost > r.cost; }
};

int localIndex(const DenseGrid& dense, const Vec3i& p) {
    return dense.index(p.x, p.y, p.z);
}

bool isSurfaceVoxel(const DenseGrid& dense, int x, int y, int z) {
    if (!dense.getSolid(x, y, z))
        return false;

    static const Vec3i dirs6[6] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
    };

    for (const auto& d : dirs6) {
        const int nx = x + d.x;
        const int ny = y + d.y;
        const int nz = z + d.z;
        if (!dense.inBounds(nx, ny, nz) || !dense.getSolid(nx, ny, nz))
            return true;
    }

    return false;
}

Vec3i traceGradientToCenter(const DenseGrid& dense, Vec3i p) {
    const int max_steps = dense.sx + dense.sy + dense.sz + 32;

    for (int step = 0; step < max_steps; ++step) {
        Vec3i best = p;
        float best_dist = dense.getDist2(p.x, p.y, p.z);

        for (const auto& d : kDirs26) {
            const Vec3i n = p + d;
            if (!dense.inBounds(n.x, n.y, n.z) || !dense.getSolid(n.x, n.y, n.z))
                continue;

            const float nd = dense.getDist2(n.x, n.y, n.z);
            if (nd > best_dist + 1e-4f) {
                best_dist = nd;
                best = n;
            }
        }

        if (best == p)
            break;

        p = best;
    }

    return p;
}

std::vector<Vec3i> extractFlowCenters(const DenseGrid& dense) {
    std::unordered_map<int, int> endpoint_count;

    for (int z = 0; z < dense.sz; ++z) {
        for (int y = 0; y < dense.sy; ++y) {
            for (int x = 0; x < dense.sx; ++x) {
                if (!isSurfaceVoxel(dense, x, y, z))
                    continue;

                const Vec3i endpoint = traceGradientToCenter(dense, {x, y, z});
                endpoint_count[localIndex(dense, endpoint)]++;
            }
        }
    }

    int max_count = 0;
    for (const auto& [index, count] : endpoint_count) {
        (void)index;
        max_count = std::max(max_count, count);
    }

    const int min_count = max_count >= 8 ? 2 : 1;
    std::vector<Vec3i> centers;
    centers.reserve(endpoint_count.size());

    for (const auto& [index, count] : endpoint_count) {
        if (count < min_count)
            continue;

        const int x = index % dense.sx;
        const int y = (index / dense.sx) % dense.sy;
        const int z = index / (dense.sx * dense.sy);
        centers.emplace_back(x, y, z);
    }

    std::sort(centers.begin(), centers.end(), [](const Vec3i& a, const Vec3i& b) {
        if (a.z != b.z)
            return a.z < b.z;
        if (a.y != b.y)
            return a.y < b.y;
        return a.x < b.x;
    });

    return centers;
}

float localDistance(const Vec3i& a, const Vec3i& b) {
    const float dx = static_cast<float>(a.x - b.x);
    const float dy = static_cast<float>(a.y - b.y);
    const float dz = static_cast<float>(a.z - b.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<Vec3i> findMaxDistanceRoute(const DenseGrid& dense,
                                        const Vec3i& start,
                                        const Vec3i& target) {
    if (start == target)
        return {start};

    const float direct_distance = localDistance(start, target);
    const int margin = std::max(4, static_cast<int>(std::ceil(direct_distance)));
    const Vec3i min_bound(std::max(0, std::min(start.x, target.x) - margin),
                          std::max(0, std::min(start.y, target.y) - margin),
                          std::max(0, std::min(start.z, target.z) - margin));
    const Vec3i max_bound(std::min(dense.sx - 1, std::max(start.x, target.x) + margin),
                          std::min(dense.sy - 1, std::max(start.y, target.y) + margin),
                          std::min(dense.sz - 1, std::max(start.z, target.z) + margin));

    const int total = dense.sx * dense.sy * dense.sz;
    std::vector<float> costs(total, DenseGrid::INF);
    std::vector<int> prev(total, -1);
    std::priority_queue<LocalRouteNode> pq;

    const int start_index = localIndex(dense, start);
    const int target_index = localIndex(dense, target);
    costs[start_index] = 0.f;
    pq.push({start_index, 0.f});

    auto inSearchBounds = [&](const Vec3i& p) {
        return p.x >= min_bound.x && p.y >= min_bound.y && p.z >= min_bound.z &&
               p.x <= max_bound.x && p.y <= max_bound.y && p.z <= max_bound.z;
    };

    while (!pq.empty()) {
        const LocalRouteNode cur = pq.top();
        pq.pop();

        if (cur.cost != costs[cur.index])
            continue;
        if (cur.index == target_index)
            break;

        const int x = cur.index % dense.sx;
        const int y = (cur.index / dense.sx) % dense.sy;
        const int z = cur.index / (dense.sx * dense.sy);
        const Vec3i p(x, y, z);

        for (const auto& d : kDirs26) {
            const Vec3i n = p + d;
            if (!dense.inBounds(n.x, n.y, n.z) || !inSearchBounds(n) ||
                !dense.getSolid(n.x, n.y, n.z)) {
                continue;
            }

            const float step_len =
                std::sqrt(static_cast<float>(d.x * d.x + d.y * d.y + d.z * d.z));
            const float field = std::max(0.25f, dense.getDist2(n.x, n.y, n.z));
            const float next_cost = cur.cost + step_len / field;
            const int next_index = localIndex(dense, n);

            if (next_cost >= costs[next_index])
                continue;

            costs[next_index] = next_cost;
            prev[next_index] = cur.index;
            pq.push({next_index, next_cost});
        }
    }

    if (prev[target_index] < 0)
        return {start, target};

    std::vector<Vec3i> route;
    for (int cur = target_index; cur >= 0; cur = prev[cur]) {
        const int x = cur % dense.sx;
        const int y = (cur / dense.sx) % dense.sy;
        const int z = cur / (dense.sx * dense.sy);
        route.emplace_back(x, y, z);
        if (cur == start_index)
            break;
    }

    std::reverse(route.begin(), route.end());
    return route;
}

void addRouteLines(const DenseGrid& dense,
                   const std::vector<Vec3i>& route,
                   std::vector<std::pair<Vec3i, Vec3i>>& lines,
                   std::unordered_set<uint64_t>& line_keys) {
    if (route.size() < 2)
        return;

    for (size_t i = 0; i + 1 < route.size(); ++i) {
        const Vec3i a = dense.denseToWorldVoxel(route[i].x, route[i].y,
                                                route[i].z);
        const Vec3i b = dense.denseToWorldVoxel(route[i + 1].x,
                                                route[i + 1].y,
                                                route[i + 1].z);
        if (a == b)
            continue;

        uint64_t ia = static_cast<uint64_t>(localIndex(dense, route[i]));
        uint64_t ib = static_cast<uint64_t>(localIndex(dense, route[i + 1]));
        if (ia > ib)
            std::swap(ia, ib);
        const uint64_t key = (ia << 32) ^ ib;
        if (!line_keys.insert(key).second)
            continue;

        lines.push_back({a, b});
    }
}
}  // namespace

std::vector<std::pair<Vec3i, Vec3i>> extractGradientFlowSkeletonLines(
    const DenseGrid& dense) {
    std::vector<std::pair<Vec3i, Vec3i>> lines;

    if (dense.sx <= 0 || dense.sy <= 0 || dense.sz <= 0)
        return lines;

    const std::vector<Vec3i> centers = extractFlowCenters(dense);
    if (centers.size() < 2)
        return lines;

    std::unordered_set<uint64_t> center_edges;
    std::unordered_set<uint64_t> line_keys;
    std::vector<size_t> parent(centers.size());
    for (size_t i = 0; i < parent.size(); ++i) {
        parent[i] = i;
    }

    auto findRoot = [&](size_t value) {
        size_t root = value;
        while (parent[root] != root) {
            root = parent[root];
        }
        while (parent[value] != value) {
            const size_t next = parent[value];
            parent[value] = root;
            value = next;
        }
        return root;
    };

    auto uniteCenters = [&](size_t a, size_t b) {
        const size_t ra = findRoot(a);
        const size_t rb = findRoot(b);
        if (ra != rb) {
            parent[rb] = ra;
        }
    };

    std::unordered_map<int, size_t> center_lookup;
    center_lookup.reserve(centers.size() * 2);
    for (size_t i = 0; i < centers.size(); ++i) {
        center_lookup[localIndex(dense, centers[i])] = i;
    }

    std::vector<int> degree(centers.size(), 0);
    auto connectCenters = [&](size_t i, size_t j, bool search_route) {
        if (i == j)
            return;

        uint64_t a = static_cast<uint64_t>(std::min(i, j));
        uint64_t b = static_cast<uint64_t>(std::max(i, j));
        const uint64_t edge_key = (a << 32) ^ b;
        if (!center_edges.insert(edge_key).second)
            return;

        degree[i]++;
        degree[j]++;
        uniteCenters(i, j);

        if (search_route) {
            const auto route = findMaxDistanceRoute(dense, centers[i], centers[j]);
            addRouteLines(dense, route, lines, line_keys);
        } else {
            addRouteLines(dense, {centers[i], centers[j]}, lines, line_keys);
        }
    };

    for (size_t i = 0; i < centers.size(); ++i) {
        for (const auto& d : kDirs26) {
            const Vec3i n = centers[i] + d;
            if (!dense.inBounds(n.x, n.y, n.z))
                continue;

            const auto found = center_lookup.find(localIndex(dense, n));
            if (found == center_lookup.end())
                continue;

            connectCenters(i, found->second, false);
        }
    }

    for (size_t i = 0; i < centers.size(); ++i) {
        if (degree[i] > 0)
            continue;

        float best_dist = DenseGrid::INF;
        size_t best = i;
        for (size_t j = 0; j < centers.size(); ++j) {
            if (i == j)
                continue;

            const float dist = localDistance(centers[i], centers[j]);
            if (dist < best_dist) {
                best_dist = dist;
                best = j;
            }
        }

        if (best != i)
            connectCenters(i, best, true);
    }

    while (true) {
        float best_dist = DenseGrid::INF;
        size_t best_a = centers.size();
        size_t best_b = centers.size();

        for (size_t i = 0; i < centers.size(); ++i) {
            const size_t root_i = findRoot(i);
            for (size_t j = i + 1; j < centers.size(); ++j) {
                if (root_i == findRoot(j))
                    continue;

                const float dist = localDistance(centers[i], centers[j]);
                if (dist < best_dist) {
                    best_dist = dist;
                    best_a = i;
                    best_b = j;
                }
            }
        }

        if (best_a == centers.size() || best_b == centers.size())
            break;

        connectCenters(best_a, best_b, true);
    }

    return lines;
}

std::vector<std::pair<Vec3i, Vec3i>> mapSurfaceVoxelsToSkeleton(
    const DenseGrid& dense,
    const std::vector<std::pair<Vec3i, Vec3i>>& skeleton_lines) {
    std::vector<std::pair<Vec3i, Vec3i>> mapping;
    if (dense.sx <= 0 || dense.sy <= 0 || dense.sz <= 0 ||
        skeleton_lines.empty()) {
        return mapping;
    }

    std::unordered_set<int> skeleton_set;
    std::vector<Vec3i> skeleton_points;
    skeleton_set.reserve(skeleton_lines.size() * 2);
    skeleton_points.reserve(skeleton_lines.size() * 2);

    auto addSkeletonPoint = [&](const Vec3i& world_voxel) {
        const Vec3i dense_voxel = dense.worldVoxelToDense(world_voxel);
        if (!dense.inBounds(dense_voxel.x, dense_voxel.y, dense_voxel.z))
            return;

        const int index = localIndex(dense, dense_voxel);
        if (skeleton_set.insert(index).second) {
            skeleton_points.push_back(dense_voxel);
        }
    };

    for (const auto& line : skeleton_lines) {
        addSkeletonPoint(line.first);
        addSkeletonPoint(line.second);
    }

    if (skeleton_points.empty())
        return mapping;

    auto nearestSkeletonPoint = [&](const Vec3i& p) {
        float best_dist = DenseGrid::INF;
        Vec3i best = skeleton_points.front();
        for (const Vec3i& candidate : skeleton_points) {
            const float dist = localDistance(p, candidate);
            if (dist < best_dist) {
                best_dist = dist;
                best = candidate;
            }
        }
        return best;
    };

    const int max_steps = dense.sx + dense.sy + dense.sz + 32;
    for (int z = 0; z < dense.sz; ++z) {
        for (int y = 0; y < dense.sy; ++y) {
            for (int x = 0; x < dense.sx; ++x) {
                if (!isSurfaceVoxel(dense, x, y, z))
                    continue;

                Vec3i p(x, y, z);
                Vec3i hit = nearestSkeletonPoint(p);

                for (int step = 0; step < max_steps; ++step) {
                    const int current_index = localIndex(dense, p);
                    if (skeleton_set.find(current_index) != skeleton_set.end()) {
                        hit = p;
                        break;
                    }

                    Vec3i best = p;
                    float best_dist = dense.getDist2(p.x, p.y, p.z);
                    for (const auto& d : kDirs26) {
                        const Vec3i n = p + d;
                        if (!dense.inBounds(n.x, n.y, n.z) ||
                            !dense.getSolid(n.x, n.y, n.z)) {
                            continue;
                        }

                        const float nd = dense.getDist2(n.x, n.y, n.z);
                        if (nd > best_dist + 1e-4f) {
                            best_dist = nd;
                            best = n;
                        }
                    }

                    if (best == p) {
                        hit = nearestSkeletonPoint(p);
                        break;
                    }
                    p = best;
                }

                mapping.push_back(
                    {dense.denseToWorldVoxel(x, y, z),
                     dense.denseToWorldVoxel(hit.x, hit.y, hit.z)});
            }
        }
    }

    return mapping;
}

// ============================================================
// Build EDT Init Field
// ============================================================

DenseGrid buildInsideEDTField(const DenseGrid& src) {
    DenseGrid g = src;

    for (int z = 0; z < g.sz; z++) {
        for (int y = 0; y < g.sy; y++) {
            for (int x = 0; x < g.sx; x++) {
                if (g.getSolid(x, y, z)) {
                    g.setDist2(x, y, z, DenseGrid::INF);
                } else {
                    g.setDist2(x, y, z, 0.f);
                }
            }
        }
    }

    return g;
}

// ============================================================
// Build Outside EDT Field
// ============================================================

DenseGrid buildOutsideEDTField(const DenseGrid& src) {
    DenseGrid g = src;

    for (int z = 0; z < g.sz; z++) {
        for (int y = 0; y < g.sy; y++) {
            for (int x = 0; x < g.sx; x++) {
                if (g.getSolid(x, y, z)) {
                    g.setDist2(x, y, z, 0.f);
                } else {
                    g.setDist2(x, y, z, DenseGrid::INF);
                }
            }
        }
    }

    return g;
}

// ============================================================
// Build Signed Distance Field
// ============================================================

SDFGrid buildSDF(const DenseGrid& src) {
    // ========================================================
    // Build inside EDT
    // ========================================================

    DenseGrid inside = buildInsideEDTField(src);

    computeEDT(inside);

    // ========================================================
    // Build outside EDT
    // ========================================================

    DenseGrid outside = buildOutsideEDTField(src);

    computeEDT(outside);

    // ========================================================
    // Combine
    // ========================================================

    SDFGrid sdf;

    sdf.min_bound = src.min_bound;
    sdf.max_bound = src.max_bound;

    sdf.sx = src.sx;
    sdf.sy = src.sy;
    sdf.sz = src.sz;

    sdf.sdf.resize(sdf.sx * sdf.sy * sdf.sz);

    for (int z = 0; z < sdf.sz; z++) {
        for (int y = 0; y < sdf.sy; y++) {
            for (int x = 0; x < sdf.sx; x++) {
                const float din = std::sqrt(inside.getDist2(x, y, z));

                const float dout = std::sqrt(outside.getDist2(x, y, z));

                float value;

                // ============================================
                // Standard sign convention:
                //
                // inside  = negative
                // outside = positive
                // ============================================

                if (src.getSolid(x, y, z)) {
                    value = -din;
                } else {
                    value = dout;
                }

                sdf.set(x, y, z, value);
            }
        }
    }

    return sdf;
}

}  // namespace sinriv::kigstudio::voxel
