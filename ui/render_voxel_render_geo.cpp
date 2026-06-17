#include "render_voxel_list.h"
#include "kigstudio/cgal/convex_hull.h"
#include "kigstudio/cgal/mesh_simplification.h"
#include "kigstudio/cgal/skeleton_extraction.h"
#include "kigstudio/mesh/conebox.h"
#include "kigstudio/sdf/sdf_mesh.h"
#include "kigstudio/utils/generator.h"
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <set>

namespace sinriv::ui::render {
namespace {
using SkeletonPoint = sinriv::kigstudio::voxel::vec3f;
using SkeletonLine = std::pair<SkeletonPoint, SkeletonPoint>;
using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;
using Vec3i = sinriv::kigstudio::Vec3i;

// Insert candidate voxels into the grid. If use_precise_voxelization is true,
// tri-face votes are counted and only voxels seen by a single face are verified
// with CGAL Side_of_triangle_mesh to reduce false positives.
static void insert_voxels_with_optional_verify(
    sinriv::kigstudio::voxel::VoxelGrid& voxel_data,
    const std::vector<Vec3i>& candidates_x,
    const std::vector<Vec3i>& candidates_y,
    const std::vector<Vec3i>& candidates_z,
    const std::vector<Triangle>& source_triangles,
    float voxel_size,
    bool use_precise_voxelization) {
    size_t total_candidates = candidates_x.size() + candidates_y.size() +
                              candidates_z.size();
    if (total_candidates == 0)
        return;

    std::vector<Vec3i> all_candidates;
    all_candidates.reserve(total_candidates);
    all_candidates.insert(all_candidates.end(), candidates_x.begin(),
                          candidates_x.end());
    all_candidates.insert(all_candidates.end(), candidates_y.begin(),
                          candidates_y.end());
    all_candidates.insert(all_candidates.end(), candidates_z.begin(),
                          candidates_z.end());

    std::sort(all_candidates.begin(), all_candidates.end(),
              [](const Vec3i& a, const Vec3i& b) {
                  if (a.x != b.x)
                      return a.x < b.x;
                  if (a.y != b.y)
                      return a.y < b.y;
                  return a.z < b.z;
              });

    std::vector<Vec3i> confirmed;
    std::vector<Vec3i> to_verify;
    confirmed.reserve(all_candidates.size() / 2 + 1);
    to_verify.reserve(all_candidates.size() / 10 + 1);

    for (size_t i = 0; i < all_candidates.size();) {
        size_t j = i + 1;
        while (j < all_candidates.size() &&
               all_candidates[j].x == all_candidates[i].x &&
               all_candidates[j].y == all_candidates[i].y &&
               all_candidates[j].z == all_candidates[i].z) {
            ++j;
        }
        int vote_count = static_cast<int>(j - i);
        if (vote_count >= 2) {
            confirmed.push_back(all_candidates[i]);
        } else {
            to_verify.push_back(all_candidates[i]);
        }
        i = j;
    }

    // 非精确模式下，只有至少两个轴都判定为有体素的候选才会被插入；
    // 精确模式下，对单轴候选再用 CGAL Side_of_triangle_mesh 做二次验证。
    if (use_precise_voxelization && !to_verify.empty()) {
        sinriv::kigstudio::sdf::SDF_Mesh cgal_tester;
        bool has_tester = cgal_tester.loadTriangles(source_triangles) &&
                          cgal_tester.hasInsideTester();
        if (has_tester) {
#pragma omp parallel
            {
                std::vector<Vec3i> local_confirmed;
#pragma omp for nowait
                for (int64_t idx = 0;
                     idx < static_cast<int64_t>(to_verify.size());
                     ++idx) {
                    const auto& v = to_verify[idx];
                    float wx = voxel_data.global_position.x +
                               (v.x + 0.5f) * voxel_size;
                    float wy = voxel_data.global_position.y +
                               (v.y + 0.5f) * voxel_size;
                    float wz = voxel_data.global_position.z +
                               (v.z + 0.5f) * voxel_size;
                    if (cgal_tester.isInside({wx, wy, wz})) {
                        local_confirmed.push_back(v);
                    }
                }
#pragma omp critical
                confirmed.insert(confirmed.end(), local_confirmed.begin(),
                                 local_confirmed.end());
            }
        } else {
            confirmed.insert(confirmed.end(), to_verify.begin(),
                             to_verify.end());
        }
    }

    voxel_data.insertMany(confirmed);
}

// 在 CPU 侧预生成所有 chunk 的体素网格数据（不创建 GPU 资源）。
// 耗时部分在 generateMeshForChunk，放到 lock 外执行可避免阻塞 UI。
static std::unordered_map<
    uint64_t,
    std::vector<std::tuple<Triangle, sinriv::kigstudio::voxel::vec3f>>>
generateChunkMeshData(sinriv::kigstudio::voxel::VoxelGrid& voxel_data,
                      double isolevel,
                      bool smooth_normals,
                      float expand = 0.0f) {
    std::unordered_map<
        uint64_t,
        std::vector<std::tuple<Triangle, sinriv::kigstudio::voxel::vec3f>>>
        result;
    for (const auto& [key, chunk] : voxel_data.chunks) {
        (void)chunk;
        int num_triangles = 0;
        auto generator = sinriv::kigstudio::voxel::generateMeshForChunk(
            voxel_data, key, isolevel, num_triangles, smooth_normals, expand);
        std::vector<std::tuple<Triangle, sinriv::kigstudio::voxel::vec3f>> mesh;
        for (auto [tri, n] : generator) {
            mesh.push_back({tri, n});
        }
        if (!mesh.empty()) {
            result[key] = std::move(mesh);
        }
    }
    return result;
}

static std::vector<sinriv::kigstudio::vec3<float>> clip_polygon_by_plane(
    const std::vector<sinriv::kigstudio::vec3<float>>& poly,
    const sinriv::kigstudio::vec3<float>& point,
    const sinriv::kigstudio::vec3<float>& normal,
    float threshold) {
    std::vector<sinriv::kigstudio::vec3<float>> out;
    if (poly.empty()) return out;
    const float EPS = 1e-5f;
    for (size_t i = 0; i < poly.size(); ++i) {
        const auto& curr = poly[i];
        const auto& next = poly[(i + 1) % poly.size()];
        float dc = normal.dot(curr - point) - threshold;
        float dn = normal.dot(next - point) - threshold;
        bool curr_in = dc >= -EPS;
        bool next_in = dn >= -EPS;
        if (curr_in && next_in) {
            out.push_back(next);
        } else if (curr_in && !next_in) {
            float t = dc / (dc - dn);
            out.push_back(curr + (next - curr) * t);
        } else if (!curr_in && next_in) {
            float t = dc / (dc - dn);
            out.push_back(curr + (next - curr) * t);
            out.push_back(next);
        }
    }
    return out;
}

static std::pair<std::vector<Triangle>, std::vector<Triangle>>
split_triangles_by_plane(
    const std::vector<Triangle>& triangles,
    const sinriv::kigstudio::Plane<float>& plane) {
    std::vector<Triangle> positive;
    std::vector<Triangle> negative;
    sinriv::kigstudio::vec3<float> normal(plane.A, plane.B, plane.C);
    float n_len = normal.length();
    if (n_len < 1e-12f) {
        return {triangles, {}};
    }
    sinriv::kigstudio::vec3<float> n_unit = normal * (1.0f / n_len);
    float d = plane.D / n_len;
    for (const auto& tri : triangles) {
        std::vector<sinriv::kigstudio::vec3<float>> poly = {
            std::get<0>(tri), std::get<1>(tri), std::get<2>(tri)};
        std::vector<sinriv::kigstudio::vec3<float>> pos_poly =
            clip_polygon_by_plane(poly, sinriv::kigstudio::vec3<float>{0, 0, 0},
                                  n_unit, -d);
        std::vector<sinriv::kigstudio::vec3<float>> neg_poly =
            clip_polygon_by_plane(poly, sinriv::kigstudio::vec3<float>{0, 0, 0},
                                  -n_unit, d);
        for (size_t i = 1; i + 1 < pos_poly.size(); ++i) {
            positive.emplace_back(
                std::make_tuple(pos_poly[0], pos_poly[i], pos_poly[i + 1]));
        }
        for (size_t i = 1; i + 1 < neg_poly.size(); ++i) {
            negative.emplace_back(
                std::make_tuple(neg_poly[0], neg_poly[i], neg_poly[i + 1]));
        }
    }
    return {std::move(positive), std::move(negative)};
}

struct SkeletonPointKey {
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;

    bool operator==(const SkeletonPointKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct SkeletonPointKeyHash {
    std::size_t operator()(const SkeletonPointKey& key) const {
        std::size_t h = 1469598103934665603ull;
        auto mix = [&h](int64_t v) {
            h ^= static_cast<std::size_t>(v);
            h *= 1099511628211ull;
        };
        mix(key.x);
        mix(key.y);
        mix(key.z);
        return h;
    }
};

SkeletonPointKey makeSkeletonPointKey(const SkeletonPoint& p) {
    constexpr double kScale = 10000.0;
    return {static_cast<int64_t>(std::llround(static_cast<double>(p.x) * kScale)),
            static_cast<int64_t>(std::llround(static_cast<double>(p.y) * kScale)),
            static_cast<int64_t>(std::llround(static_cast<double>(p.z) * kScale))};
}

uint64_t makeSkeletonEdgeKey(size_t a, size_t b) {
    if (a > b)
        std::swap(a, b);
    return (static_cast<uint64_t>(a) << 32) ^ static_cast<uint64_t>(b);
}

std::vector<SkeletonPoint> chaikinSmooth(const std::vector<SkeletonPoint>& points,
                                         bool closed,
                                         int iterations) {
    if (points.size() < 3 || iterations <= 0)
        return points;

    std::vector<SkeletonPoint> current = points;
    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<SkeletonPoint> next;
        next.reserve(current.size() * 2 + 2);

        if (!closed)
            next.push_back(current.front());

        const size_t segment_count = closed ? current.size() : current.size() - 1;
        for (size_t i = 0; i < segment_count; ++i) {
            const auto& a = current[i];
            const auto& b = current[(i + 1) % current.size()];
            next.emplace_back(a * 0.75f + b * 0.25f);
            next.emplace_back(a * 0.25f + b * 0.75f);
        }

        if (!closed)
            next.push_back(current.back());

        current = std::move(next);
    }

    return current;
}

void appendSmoothedChain(const std::vector<size_t>& chain,
                         bool closed,
                         const std::vector<SkeletonPoint>& nodes,
                         std::vector<SkeletonLine>& result) {
    if (chain.size() < 2)
        return;

    std::vector<SkeletonPoint> points;
    points.reserve(chain.size());
    for (size_t index : chain) {
        points.push_back(nodes[index]);
    }

    constexpr int kSkeletonSmoothIterations = 3;
    points = chaikinSmooth(points, closed, kSkeletonSmoothIterations);

    for (size_t i = 0; i + 1 < points.size(); ++i) {
        result.push_back({points[i], points[i + 1]});
    }
    if (closed && points.size() > 2) {
        result.push_back({points.back(), points.front()});
    }
}

std::vector<SkeletonLine> smoothSkeletonLines(const std::vector<SkeletonLine>& lines) {
    if (lines.size() < 3)
        return lines;

    std::vector<SkeletonPoint> nodes;
    std::unordered_map<SkeletonPointKey, size_t, SkeletonPointKeyHash> node_ids;
    std::vector<std::vector<size_t>> adjacency;

    auto getNode = [&](const SkeletonPoint& p) {
        const SkeletonPointKey key = makeSkeletonPointKey(p);
        const auto found = node_ids.find(key);
        if (found != node_ids.end())
            return found->second;

        const size_t index = nodes.size();
        node_ids[key] = index;
        nodes.push_back(p);
        adjacency.emplace_back();
        return index;
    };

    for (const auto& line : lines) {
        const size_t a = getNode(line.first);
        const size_t b = getNode(line.second);
        if (a == b)
            continue;

        adjacency[a].push_back(b);
        adjacency[b].push_back(a);
    }

    std::unordered_set<uint64_t> visited_edges;
    std::vector<SkeletonLine> result;
    result.reserve(lines.size() * 4);

    auto edgeVisited = [&](size_t a, size_t b) {
        return visited_edges.find(makeSkeletonEdgeKey(a, b)) != visited_edges.end();
    };
    auto markEdge = [&](size_t a, size_t b) {
        visited_edges.insert(makeSkeletonEdgeKey(a, b));
    };

    auto walkOpenChain = [&](size_t start, size_t next) {
        std::vector<size_t> chain = {start, next};
        markEdge(start, next);

        size_t prev = start;
        size_t cur = next;
        while (adjacency[cur].size() == 2) {
            size_t candidate = adjacency[cur][0] == prev ? adjacency[cur][1]
                                                         : adjacency[cur][0];
            if (edgeVisited(cur, candidate))
                break;

            markEdge(cur, candidate);
            chain.push_back(candidate);
            prev = cur;
            cur = candidate;
        }

        appendSmoothedChain(chain, false, nodes, result);
    };

    for (size_t i = 0; i < nodes.size(); ++i) {
        if (adjacency[i].size() == 2)
            continue;

        for (size_t next : adjacency[i]) {
            if (!edgeVisited(i, next))
                walkOpenChain(i, next);
        }
    }

    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t next : adjacency[i]) {
            if (edgeVisited(i, next))
                continue;

            std::vector<size_t> chain = {i, next};
            markEdge(i, next);

            size_t start = i;
            size_t prev = i;
            size_t cur = next;
            bool closed = false;

            while (adjacency[cur].size() == 2) {
                size_t candidate = adjacency[cur][0] == prev ? adjacency[cur][1]
                                                             : adjacency[cur][0];
                if (candidate == start) {
                    markEdge(cur, candidate);
                    closed = true;
                    break;
                }
                if (edgeVisited(cur, candidate))
                    break;

                markEdge(cur, candidate);
                chain.push_back(candidate);
                prev = cur;
                cur = candidate;
            }

            appendSmoothedChain(chain, closed, nodes, result);
        }
    }

    return result.empty() ? lines : result;
}

struct VoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const {
        std::size_t h = 1469598103934665603ull;
        auto mix = [&h](int v) {
            h ^= static_cast<uint32_t>(v);
            h *= 1099511628211ull;
        };
        mix(key.x);
        mix(key.y);
        mix(key.z);
        return h;
    }
};

VoxelKey makeVoxelKey(const sinriv::kigstudio::Vec3i& p) {
    return {p.x, p.y, p.z};
}

bool voxelLess(const sinriv::kigstudio::Vec3i& a,
               const sinriv::kigstudio::Vec3i& b) {
    if (a.x != b.x)
        return a.x < b.x;
    if (a.y != b.y)
        return a.y < b.y;
    return a.z < b.z;
}

std::vector<SkeletonPointPick>
buildSkeletonOrderCache(
    const std::vector<std::pair<sinriv::kigstudio::Vec3i,
                                sinriv::kigstudio::Vec3i>>& lines,
    const sinriv::kigstudio::voxel::VoxelGrid& voxel_grid,
    std::unordered_map<VoxelKey, int, VoxelKeyHash>& order_by_voxel) {
    using Vec3i = sinriv::kigstudio::Vec3i;
    using Pick = SkeletonPointPick;

    std::vector<Vec3i> nodes;
    std::unordered_map<VoxelKey, size_t, VoxelKeyHash> node_ids;
    std::vector<std::vector<size_t>> adjacency;

    auto getNode = [&](const Vec3i& p) {
        const VoxelKey key = makeVoxelKey(p);
        const auto found = node_ids.find(key);
        if (found != node_ids.end())
            return found->second;

        const size_t index = nodes.size();
        node_ids[key] = index;
        nodes.push_back(p);
        adjacency.emplace_back();
        return index;
    };

    for (const auto& line : lines) {
        const size_t a = getNode(line.first);
        const size_t b = getNode(line.second);
        if (a == b)
            continue;

        adjacency[a].push_back(b);
        adjacency[b].push_back(a);
    }

    if (nodes.empty())
        return {};

    for (auto& neighbors : adjacency) {
        std::sort(neighbors.begin(), neighbors.end(),
                  [&](size_t a, size_t b) {
                      return voxelLess(nodes[a], nodes[b]);
                  });
        neighbors.erase(std::unique(neighbors.begin(), neighbors.end()),
                        neighbors.end());
    }

    size_t start = 0;
    bool has_endpoint = false;
    for (size_t i = 0; i < nodes.size(); ++i) {
        const bool is_endpoint = adjacency[i].size() <= 1;
        if ((is_endpoint && !has_endpoint) ||
            (is_endpoint == has_endpoint && voxelLess(nodes[i], nodes[start]))) {
            start = i;
            has_endpoint = is_endpoint;
        }
    }

    std::vector<Pick> ordered;
    ordered.reserve(nodes.size());
    std::vector<uint8_t> visited(nodes.size(), 0);
    std::vector<size_t> stack = {start};

    auto pushNode = [&](size_t index) {
        visited[index] = 1;
        const auto world = voxel_grid.voxelCenterToWorld(nodes[index]);
        const int order = static_cast<int>(ordered.size());
        order_by_voxel[makeVoxelKey(nodes[index])] = order;
        ordered.push_back({sinriv::kigstudio::voxel::vec3f(
                               world.x, world.y, world.z),
                           order});
    };

    while (!stack.empty()) {
        const size_t cur = stack.back();
        stack.pop_back();
        if (visited[cur])
            continue;

        pushNode(cur);
        for (auto it = adjacency[cur].rbegin(); it != adjacency[cur].rend();
             ++it) {
            if (!visited[*it])
                stack.push_back(*it);
        }
    }

    for (size_t i = 0; i < nodes.size(); ++i) {
        if (!visited[i])
            pushNode(i);
    }

    return ordered;
}

sinriv::kigstudio::Generator<
    std::tuple<std::tuple<sinriv::kigstudio::vec3<float>,
                          sinriv::kigstudio::vec3<float>,
                          sinriv::kigstudio::vec3<float>>,
               sinriv::kigstudio::vec3<float>>>
triangle_generator_with_normals(
    const std::vector<std::tuple<sinriv::kigstudio::vec3<float>,
                                 sinriv::kigstudio::vec3<float>,
                                 sinriv::kigstudio::vec3<float>>>& triangles) {
    using Triangle =
        std::tuple<sinriv::kigstudio::vec3<float>,
                   sinriv::kigstudio::vec3<float>,
                   sinriv::kigstudio::vec3<float>>;
    using vec3f = sinriv::kigstudio::vec3<float>;
    for (const auto& tri : triangles) {
        const auto& v0 = std::get<0>(tri);
        const auto& v1 = std::get<1>(tri);
        const auto& v2 = std::get<2>(tri);
        vec3f normal = (v1 - v0).cross(v2 - v0);
        const float len = normal.length();
        if (len > 1e-8f)
            normal = normal / len;
        else
            normal = vec3f{0.0f, 0.0f, 0.0f};
        co_yield std::make_tuple(tri, normal);
    }
}
}  // namespace

std::vector<RenderVoxelList::RenderVoxelItem*> RenderVoxelList::do_segment(
    int index) {
    locker.lock();
    auto it = items.find(index);
    if (it == items.end()) {
        locker.unlock();
        return {};
    }
    if (it->second->write_count != 0) {
        locker.unlock();
        return {};
    }
    it->second->ref_count++;
    it->second->write_count++;
    locker.unlock();

    setQueueStatus(get_locale_string("status.segmenting"));
    queue_progress = 0.0f;
    std::vector<std::tuple<sinriv::kigstudio::voxel::VoxelGrid,
                           std::shared_ptr<sinriv::kigstudio::sdf::SDFBase>>>
        results;
    std::vector<std::vector<Triangle>> mesh_split_results;
    std::cout << "[do_segment] start item=" << index
              << " mode=" << it->second->segment_mode
              << " write_count=" << it->second->write_count << std::endl;
    try {
        if (it->second->mesh_only) {
            auto split = split_triangles_by_plane(it->second->source_triangles,
                                                  it->second->plane);
            mesh_split_results.push_back(std::move(split.first));
            mesh_split_results.push_back(std::move(split.second));
            for (size_t i = 0; i < mesh_split_results.size(); ++i) {
                sinriv::kigstudio::voxel::VoxelGrid dummy;
                results.emplace_back(std::move(dummy), nullptr);
            }
        } else {
            results = it->second->do_segment();
        }
        queue_progress = 0.7f;
    } catch (const std::exception& e) {
        std::cerr << "[do_segment] exception: " << e.what() << std::endl;
        {
            std::lock_guard<std::mutex> lock(locker);
            it->second->ref_count--;
            it->second->write_count--;
        }
        throw;
    }
    std::cout << "[do_segment] done item=" << index << std::endl;

    std::vector<int> new_ids;
    std::vector<std::vector<int>> new_children;
    std::vector<bool> new_auto_segment_update;
    std::vector<std::optional<CollisionEditorSnapshot>> child_snapshots;
    {
        std::lock_guard<std::mutex> lock(locker);
        it->second->ref_count--;
        it->second->write_count--;
        std::cout << "[do_segment] write_count decremented to="
                  << it->second->write_count << std::endl;
        size_t num_results = results.size();
        size_t num_existing = it->second->children.size();

        for (size_t i = 0; i < num_results; ++i) {
            int child_id;
            std::vector<int> child_children = {-1, -1};
            bool child_auto_update = true;
            std::optional<CollisionEditorSnapshot> snapshot;
            if (i < num_existing) {
                auto it_child = items.find(it->second->children[i]);
                if (it_child == items.end()) {
                    child_id = current_id++;
                } else {
                    child_id = it_child->second->id;
                    child_children = it_child->second->children;
                    child_auto_update = it_child->second->auto_segment_update;
                    snapshot = this->capture_snapshot(*it_child->second);
                    {
                        std::lock_guard<std::mutex> lock(
                            pending_deletion_mutex);
                        pending_deletion.push_back(std::move(it_child->second));
                    }
                    items.erase(it_child);
                    update_nav_node_status = true;
                    if (child_auto_update) {
                        for (int gc : child_children) {
                            if (items.find(gc) != items.end()) {
                                queue_do_segment(child_id);
                                break;
                            }
                        }
                    }
                }
            } else {
                child_id = current_id++;
            }
            new_ids.push_back(child_id);
            new_children.push_back(child_children);
            new_auto_segment_update.push_back(child_auto_update);
            child_snapshots.push_back(snapshot);
        }

        // 标记多余的旧子节点释放
        for (size_t i = num_results; i < num_existing; ++i) {
            auto it_child = items.find(it->second->children[i]);
            if (it_child != items.end()) {
                it_child->second->queue_release = true;
            }
        }

        it->second->children.clear();
        for (int id : new_ids) {
            it->second->children.push_back(id);
        }
    }

    std::vector<RenderVoxelItem*> result_ptrs;
    for (size_t i = 0; i < results.size(); ++i) {
        auto new_item = std::make_unique<RenderVoxelItem>();
        new_item->manager = this;
        new_item->id = new_ids[i];
        new_item->children = new_children[i];
        new_item->auto_segment_update = new_auto_segment_update[i];
        new_item->voxel_grid_data = std::move(std::get<0>(results[i]));
        new_item->sdf_data = std::get<1>(results[i]);
        new_item->thumbnail_dirty = true;
        if (child_snapshots[i].has_value()) {
            this->apply_snapshot(*new_item, child_snapshots[i].value());
        } else {
            new_item->segment_mode = it->second->segment_mode;
            if (it->second->segment_mode == RenderVoxelItem::CONCAVE_CONE) {
                new_item->concave_cone.apex = it->second->concave_cone.apex;
            }
        }
        if (it->second->segment_mode == RenderVoxelItem::NEIGHBOR) {
            new_item->neighbor_max_distance = it->second->neighbor_max_distance;
        }
        if (it->second->mesh_only && i < mesh_split_results.size()) {
            new_item->mesh_only = true;
            new_item->source_triangles = std::move(mesh_split_results[i]);
            if (!new_item->source_triangles.empty()) {
                new_item->mesh_renderer.loadGeometry(
                    triangle_generator_with_normals(new_item->source_triangles));
            }
        }
        auto ptr = new_item.get();
        result_ptrs.push_back(ptr);
        {
            std::lock_guard<std::mutex> lock(locker);
            items[new_item->id] = std::move(new_item);
            update_nav_node_status = true;
        }
    }
    return result_ptrs;
}

void RenderVoxelList::extract_skeleton(int index) {
    locker.lock();
    auto it = items.find(index);
    if (it == items.end()) {
        locker.unlock();
        return;
    }
    if (it->second->write_count != 0) {
        locker.unlock();
        return;
    }
    it->second->ref_count++;
    it->second->write_count++;
    locker.unlock();

    setQueueStatus(get_locale_string("status.extracting_skeleton"));
    queue_progress = 0.0f;
    std::vector<std::pair<sinriv::kigstudio::voxel::vec3f,
                          sinriv::kigstudio::voxel::vec3f>>
        chain_lines;
    std::vector<RenderVoxelItem::SurfaceSkeletonCacheEntry>
        surface_skeleton_world_cache;
    std::vector<SkeletonPointPick> skeleton_order_cache;
    bool cgal_succeeded = false;

    // --- CGAL mesh-based skeleton path ---
    if (it->second->use_cgal_skeleton && !it->second->stl_path.empty()) {
        try {
            setQueueStatus(get_locale_string("status.extracting_skeleton_cgal"));
            std::vector<sinriv::kigstudio::voxel::Triangle> cgal_triangles;
            if (!it->second->source_triangles.empty()) {
                cgal_triangles = it->second->source_triangles;
            } else {
                // Read STL directly
                for (auto [tri, n] : sinriv::kigstudio::voxel::readSTL(it->second->stl_path)) {
                    (void)n;
                    cgal_triangles.push_back(tri);
                }
            }
            if (!cgal_triangles.empty()) {
                auto cgal_lines = sinriv::kigstudio::cgal::extractSkeletonFromMesh(
                    cgal_triangles);
                if (!cgal_lines.empty()) {
                    chain_lines = std::move(cgal_lines);
                    chain_lines = smoothSkeletonLines(chain_lines);

                    // Build skeleton_order_cache from unique endpoints
                    std::set<std::tuple<float,float,float>> seen;
                    int order = 0;
                    for (const auto& line : chain_lines) {
                        for (const auto& p : {line.first, line.second}) {
                            auto key = std::make_tuple(p.x, p.y, p.z);
                            if (seen.insert(key).second) {
                                SkeletonPointPick pick;
                                pick.position = p;
                                pick.order = order++;
                                skeleton_order_cache.push_back(pick);
                            }
                        }
                    }
                    cgal_succeeded = true;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[CGAL Skeleton] failed: " << e.what()
                      << ", falling back to voxel method.\n";
        }
    }

    // --- Voxel-based skeleton path (fallback) ---
    if (!cgal_succeeded) try {
        char res_buf[512];
        setQueueStatus(get_locale_cstr("progress.extract_skeleton.buildDenseGrid"));
        kigstudio::voxel::DenseGrid dense =
            kigstudio::voxel::buildDenseGrid(it->second->voxel_grid_data);
        snprintf(res_buf, sizeof(res_buf),
                 get_locale_cstr("log.extract_skeleton.buildDenseGrid.result"),
                 dense.min_bound.x, dense.min_bound.y, dense.min_bound.z,
                 dense.max_bound.x, dense.max_bound.y, dense.max_bound.z,
                 it->second->voxel_grid_data.num_chunk());
        append_queue_logf(res_buf);
        std::cout << res_buf << std::endl;
        queue_progress = 0.25f;
        setQueueStatus(get_locale_cstr("progress.extract_skeleton.computeEDT"));
        kigstudio::voxel::computeEDT(dense);
        queue_progress = 0.5f;
        setQueueStatus(get_locale_cstr("progress.extract_skeleton.finalizeEDT"));
        kigstudio::voxel::finalizeEDT(dense);
        queue_progress = 0.75f;
        setQueueStatus(get_locale_cstr("progress.extract_skeleton.extractCenterline"));
        auto skeleton_lines =
            kigstudio::voxel::extractGradientFlowSkeletonLines(dense);
        auto surface_skeleton_cache =
            kigstudio::voxel::mapSurfaceVoxelsToSkeleton(dense, skeleton_lines);
        queue_progress = 1.0f;
        snprintf(res_buf, sizeof(res_buf),
                 get_locale_cstr("log.extract_skeleton.result"),
                 skeleton_lines.size() * 2);
        append_queue_logf(res_buf);
        std::cout << res_buf << std::endl;

        const auto& voxel_grid = it->second->voxel_grid_data;
        std::unordered_map<VoxelKey, int, VoxelKeyHash> order_by_voxel;
        skeleton_order_cache =
            buildSkeletonOrderCache(skeleton_lines, voxel_grid, order_by_voxel);

        auto to_world = [&voxel_grid](const kigstudio::Vec3i& voxel) {
            return voxel_grid.voxelCenterToWorld(voxel);
        };

        for (const auto& line : skeleton_lines) {
            const auto a = to_world(line.first);
            const auto b = to_world(line.second);
            chain_lines.push_back(
                {sinriv::kigstudio::voxel::vec3f(a.x, a.y, a.z),
                 sinriv::kigstudio::voxel::vec3f(b.x, b.y, b.z)});
        }
        chain_lines = smoothSkeletonLines(chain_lines);
        surface_skeleton_world_cache.reserve(surface_skeleton_cache.size());
        for (const auto& entry : surface_skeleton_cache) {
            const auto skeleton_world = to_world(entry.second);
            int order = 0;
            const auto order_it = order_by_voxel.find(makeVoxelKey(entry.second));
            if (order_it != order_by_voxel.end()) {
                order = order_it->second;
            }
            surface_skeleton_world_cache.push_back(
                {entry.first,
                 {sinriv::kigstudio::voxel::vec3f(
                      skeleton_world.x, skeleton_world.y, skeleton_world.z),
                  order}});
        }

    } catch (const std::exception& e) {
        std::cerr << "[extract_skeleton] exception: " << e.what() << std::endl;
        {
            std::lock_guard<std::mutex> lock(locker);
            it->second->ref_count--;
            it->second->write_count--;
        }
        throw;
    }

    {
        std::lock_guard<std::mutex> lock(locker);
        it->second->ref_count--;
        it->second->write_count--;
        it->second->skeleton_lines = std::move(chain_lines);
        it->second->surface_skeleton_cache =
            std::move(surface_skeleton_world_cache);
        it->second->skeleton_order_cache = std::move(skeleton_order_cache);
        it->second->sort_picked_skeleton_points();
        it->second->joint_wireframe_dirty = true;
    }
}

void RenderVoxelList::load_stl(std::string filename,
                               float voxel_size,
                               double isolevel,
                               bool smooth_normals,
                               int target_item_id,
                               int load_mode,
                               bool load_as_sdf,
                               bool use_precise_voxelization) {
    using namespace sinriv::kigstudio::voxel;
    using MeshData = mesh_detail::AsyncVoxelMeshData;
    using Triangle = triangle_bvh<float>::triangle;

    // Phase 1: Read STL
    queue_progress = 0.05f;
    triangle_bvh<float> bvh;
    size_t tri_count = 0;
    kdtree::pointVec kdtree_points;
    std::vector<Triangle> raw_triangles;
    raw_triangles.reserve(1024);
    for (auto [tri, n] : sinriv::kigstudio::voxel::readSTL(filename)) {
        (void)n;
        raw_triangles.push_back(tri);
        ++tri_count;
        if (tri_count % 1000 == 0) {
            queue_progress =
                (0.05f + 0.08f * std::min(1.0f, static_cast<float>(tri_count) /
                                                    50000.0f));
            setQueueStatus(get_locale_string("status.reading_stl"));
        }
    }

    // Load origin mesh (raw, before load mode processing)
    if (target_item_id >= 0) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(target_item_id);
        if (it != items.end()) {
            it->second->origin_mesh_renderer.clear();
            it->second->origin_mesh_renderer.setBaseColor(0.0f, 0.0f, 1.0f,
                                                          1.0f);
            it->second->origin_mesh_renderer.loadGeometry(
                triangle_generator_with_normals(raw_triangles));
        }
    }

    // Phase 1b: Optional preprocessing based on load mode
    std::vector<Triangle> source_triangles;
    if (load_mode == static_cast<int>(StlLoadMode::SILHOUETTE)) {
        vec3f cb_center{0.0f, 0.0f, 0.0f};
        if (target_item_id >= 0) {
            std::lock_guard<std::mutex> lock(locker);
            auto it = items.find(target_item_id);
            if (it != items.end()) {
                cb_center = it->second->silhouette_center;
            }
        }
        setQueueStatus(get_locale_string("status.generating_silhouette_mesh"));
        source_triangles = sinriv::kigstudio::mesh::conebox::
            build_closed_mesh_from_triangles_silhouette(
                raw_triangles, cb_center,
                [&]() { return queue_should_continue.load(); },
                [&](float t, const std::string& step) {
                    queue_progress = 0.13f + t * 0.02f;
                    setQueueStatus(step);
                });
    } else if (load_mode ==
               static_cast<int>(StlLoadMode::CONVEX_HULL)) {
        source_triangles =
            sinriv::kigstudio::cgal::convexHull3(raw_triangles);
    } else {
        source_triangles = std::move(raw_triangles);
    }

    // Surface-only and mesh-only modes do not support SDF
    if (load_mode == static_cast<int>(StlLoadMode::SURFACE_ONLY) ||
        load_mode == static_cast<int>(StlLoadMode::MESH_ONLY)) {
        load_as_sdf = false;
    }

    {
        size_t spatial_idx = 0;
        const size_t spatial_total = source_triangles.size();
        for (const auto& tri : source_triangles) {
            bvh.insert(tri);
            kdtree_points.push_back({static_cast<double>(std::get<0>(tri).x),
                                     static_cast<double>(std::get<0>(tri).y),
                                     static_cast<double>(std::get<0>(tri).z)});
            kdtree_points.push_back({static_cast<double>(std::get<1>(tri).x),
                                     static_cast<double>(std::get<1>(tri).y),
                                     static_cast<double>(std::get<1>(tri).z)});
            kdtree_points.push_back({static_cast<double>(std::get<2>(tri).x),
                                     static_cast<double>(std::get<2>(tri).y),
                                     static_cast<double>(std::get<2>(tri).z)});
            if (++spatial_idx % 100 == 0) {
                queue_progress =
                    0.13f +
                    0.02f * (static_cast<float>(spatial_idx) /
                             static_cast<float>(std::max(size_t(1), spatial_total)));
                setQueueStatus(
                    get_locale_string("status.building_spatial_index"));
            }
        }
    }

    // Phase 2: Voxelize
    setQueueStatus(get_locale_string("status.voxelizing"));
    queue_progress = 0.15f;

    VoxelGrid voxel_data;
    voxel_data.voxel_size.x = voxel_size;
    voxel_data.voxel_size.y = voxel_size;
    voxel_data.voxel_size.z = voxel_size;

    if (load_mode == static_cast<int>(StlLoadMode::SURFACE_ONLY)) {
        voxel_data.global_position.x = -0.5f * voxel_size;
        voxel_data.global_position.y = -0.5f * voxel_size;
        voxel_data.global_position.z = -0.5f * voxel_size;
        float precision = voxel_size / 10.0f;
        size_t surface_idx = 0;
        const size_t surface_total = source_triangles.size();
        for (const auto& tri : source_triangles) {
            for (auto point : sinriv::kigstudio::voxel::draw_triangle(
                     tri, voxel_size, voxel_size, voxel_size, precision)) {
                voxel_data.insert(voxel_data.worldToVoxel(point));
            }
            if (++surface_idx % 100 == 0) {
                queue_progress =
                    0.15f +
                    0.35f * (static_cast<float>(surface_idx) /
                             static_cast<float>(std::max(size_t(1), surface_total)));
                setQueueStatus(
                    get_locale_string("status.rasterizing_surface"));
            }
        }
        queue_progress = 0.50f;
    } else if (load_mode == static_cast<int>(StlLoadMode::MESH_ONLY)) {
        // Mesh-only mode: skip voxelization entirely
        queue_progress = 0.50f;
    } else {
        voxel_data.global_position.x = bvh.global_boundBox_min.x;
        voxel_data.global_position.y = bvh.global_boundBox_min.y;
        voxel_data.global_position.z = bvh.global_boundBox_min.z;

        float minx = floor(bvh.global_boundBox_min.x / voxel_size) * voxel_size;
        float miny = floor(bvh.global_boundBox_min.y / voxel_size) * voxel_size;
        float minz = floor(bvh.global_boundBox_min.z / voxel_size) * voxel_size;
        float maxx = ceil(bvh.global_boundBox_max.x / voxel_size) * voxel_size;
        float maxy = ceil(bvh.global_boundBox_max.y / voxel_size) * voxel_size;
        float maxz = ceil(bvh.global_boundBox_max.z / voxel_size) * voxel_size;
        int num_block_x = static_cast<int>(floor((maxx - minx) / voxel_size)) + 1;
        int num_block_y = static_cast<int>(floor((maxy - miny) / voxel_size)) + 1;
        int num_block_z = static_cast<int>(floor((maxz - minz) / voxel_size)) + 1;
        size_t total_rays = static_cast<size_t>(num_block_y) * num_block_z +
                            static_cast<size_t>(num_block_x) * num_block_z +
                            static_cast<size_t>(num_block_x) * num_block_y;
        if (total_rays == 0)
            total_rays = 1;

        std::mutex candidate_locker;
        std::vector<sinriv::kigstudio::Vec3i> candidates_x;
        std::vector<sinriv::kigstudio::Vec3i> candidates_y;
        std::vector<sinriv::kigstudio::Vec3i> candidates_z;
        candidates_x.reserve(1024);
        candidates_y.reserve(1024);
        candidates_z.reserve(1024);
        std::atomic<size_t> callback_count{0};

        auto make_callback = [&](std::vector<sinriv::kigstudio::Vec3i>& out) {
            return [&](auto start, auto end) {
                int start_x = static_cast<int>(std::round(
                    (start.x - voxel_data.global_position.x) / voxel_size));
                int start_y = static_cast<int>(std::round(
                    (start.y - voxel_data.global_position.y) / voxel_size));
                int start_z = static_cast<int>(std::round(
                    (start.z - voxel_data.global_position.z) / voxel_size));
                int end_x = static_cast<int>(std::round(
                    (end.x - voxel_data.global_position.x) / voxel_size));
                int end_y = static_cast<int>(std::round(
                    (end.y - voxel_data.global_position.y) / voxel_size));
                int end_z = static_cast<int>(std::round(
                    (end.z - voxel_data.global_position.z) / voxel_size));

                std::vector<sinriv::kigstudio::Vec3i> local;
                local.reserve(std::max(0, end_x - start_x + 1) *
                              std::max(0, end_y - start_y + 1) *
                              std::max(0, end_z - start_z + 1));
                for (int i = start_x; i <= end_x; ++i) {
                    for (int j = start_y; j <= end_y; ++j) {
                        for (int k = start_z; k <= end_z; ++k) {
                            if (i >= 0 && j >= 0 && k >= 0) {
                                local.push_back({i, j, k});
                            }
                        }
                    }
                }
                if (!local.empty()) {
                    std::lock_guard<std::mutex> lock(candidate_locker);
                    out.insert(out.end(), local.begin(), local.end());
                }

                size_t cnt =
                    callback_count.fetch_add(1, std::memory_order_relaxed) + 1;
                if (cnt % 100 == 0) {
                    float p =
                        0.15f +
                        0.35f * std::min(1.0f, static_cast<float>(cnt) /
                                                   static_cast<float>(total_rays *
                                                                      2));
                    queue_progress = p;
                    setQueueStatus(get_locale_string("status.voxelizing"));
                }
            };
        };

        bvh.getSolidByFace(voxel_size, voxel_size, voxel_size,
                           triangle_bvh<float>::voxel_face_X,
                           make_callback(candidates_x));
        bvh.getSolidByFace(voxel_size, voxel_size, voxel_size,
                           triangle_bvh<float>::voxel_face_Y,
                           make_callback(candidates_y));
        bvh.getSolidByFace(voxel_size, voxel_size, voxel_size,
                           triangle_bvh<float>::voxel_face_Z,
                           make_callback(candidates_z));

        insert_voxels_with_optional_verify(voxel_data, candidates_x,
                                            candidates_y, candidates_z,
                                            source_triangles, voxel_size,
                                            use_precise_voxelization);

        queue_progress = 0.50f;
    }

    // Phase 3: Generate chunked mesh
    setQueueStatus(get_locale_string("status.generating_mesh"));
    queue_progress = 0.75f;

    // 在拿 locker 之前在 CPU 侧生成 chunk mesh，避免阻塞 UI 线程。
    std::unordered_map<
        uint64_t,
        std::vector<std::tuple<Triangle, sinriv::kigstudio::voxel::vec3f>>>
        chunk_meshes;
    if (load_mode != static_cast<int>(StlLoadMode::MESH_ONLY)) {
        chunk_meshes = generateChunkMeshData(voxel_data, isolevel, smooth_normals);
    }

    if (target_item_id >= 0) {
        // 更新现有 item
        {
            std::lock_guard<std::mutex> lock(locker);
            auto it = items.find(target_item_id);
            if (it != items.end()) {
                auto& item = *it->second;
                item.mesh_renderer.clear();
                if (load_mode == static_cast<int>(StlLoadMode::SILHOUETTE) ||
                    load_mode ==
                        static_cast<int>(StlLoadMode::CONVEX_HULL)) {
                    item.mesh_renderer.loadGeometry(
                        triangle_generator_with_normals(source_triangles));
                } else {
                    item.mesh_renderer.loadGeometry(
                        sinriv::kigstudio::voxel::readSTL(filename));
                }
                item.exported_mesh_renderer.clear();
                item.cached_mesh.clear();
                item.cached_mesh_dirty = true;
                if (load_mode == static_cast<int>(StlLoadMode::MESH_ONLY)) {
                    item.voxel_renderer.clear();
                    item.voxel_grid_data.chunks.clear();
                    item.sdf_data = nullptr;
                    item.mesh_only = true;
                } else {
                    // DEFAULT / SILHOUETTE / SURFACE_ONLY / CONVEX_HULL
                    item.voxel_renderer.clear();
                    item.voxel_renderer.loadChunkMeshes(chunk_meshes);
                    item.voxel_grid_data = std::move(voxel_data);
                    if (load_as_sdf) {
                        auto mesh_sdf = std::make_shared<sinriv::kigstudio::sdf::SDF_Mesh>();
                        if (load_mode == static_cast<int>(StlLoadMode::SILHOUETTE) ||
                            load_mode == static_cast<int>(StlLoadMode::CONVEX_HULL)) {
                            mesh_sdf->loadTriangles(source_triangles);
                        } else {
                            mesh_sdf->loadSTL(filename);
                        }
                        item.sdf_data = std::move(mesh_sdf);
                    } else {
                        item.sdf_data = nullptr;
                    }
                }
                item.source_triangles = std::move(source_triangles);
                item.stl_path = filename;
                item.stl_voxel_size = voxel_size;
                item.stl_load_mode = load_mode;
                item.load_as_sdf = load_as_sdf;
                item.use_precise_voxelization = use_precise_voxelization;
                item.thumbnail_dirty = true;
                item.dirty = true;
                // 重新分割 children
                bool has_children = false;
                for (int cid : item.children) {
                    if (cid >= 0) {
                        has_children = true;
                        break;
                    }
                }
                if (has_children) {
                    queue_do_segment(target_item_id);
                }
            }
        }
    } else {
        auto item = std::make_unique<RenderVoxelItem>();
        item->manager = this;
        item->id = current_id++;
        std::cout << "[load_stl] new item id=" << item->id
                  << " write_count=" << item->write_count.load()
                  << " ref_count=" << item->ref_count.load() << std::endl;
        item->origin_mesh_renderer.setBaseColor(0.0f, 0.0f, 1.0f, 1.0f);
        item->origin_mesh_renderer.loadGeometry(
            triangle_generator_with_normals(raw_triangles));
        if (load_mode == static_cast<int>(StlLoadMode::SILHOUETTE) ||
            load_mode == static_cast<int>(StlLoadMode::CONVEX_HULL)) {
            item->mesh_renderer.loadGeometry(
                triangle_generator_with_normals(source_triangles));
        } else {
            item->mesh_renderer.loadGeometry(
                sinriv::kigstudio::voxel::readSTL(filename));
        }
        item->exported_mesh_renderer.clear();
        item->cached_mesh.clear();
        item->cached_mesh_dirty = true;
        if (load_mode == static_cast<int>(StlLoadMode::MESH_ONLY)) {
            item->mesh_only = true;
            item->voxel_grid_data.chunks.clear();
            item->sdf_data = nullptr;
        } else {
            // DEFAULT / SILHOUETTE / SURFACE_ONLY / CONVEX_HULL
            item->voxel_renderer.loadChunkMeshes(chunk_meshes);
            item->voxel_grid_data = std::move(voxel_data);
            if (load_as_sdf) {
                auto mesh_sdf = std::make_shared<sinriv::kigstudio::sdf::SDF_Mesh>();
                if (load_mode == static_cast<int>(StlLoadMode::SILHOUETTE) ||
                    load_mode == static_cast<int>(StlLoadMode::CONVEX_HULL)) {
                    mesh_sdf->loadTriangles(source_triangles);
                } else {
                    mesh_sdf->loadSTL(filename);
                }
                item->sdf_data = std::move(mesh_sdf);
            } else {
                item->sdf_data = nullptr;
            }
        }
        item->source_triangles = std::move(source_triangles);
        item->thumbnail_dirty = true;
        item->dirty = true;
        item->stl_path = filename;
        item->stl_voxel_size = voxel_size;
        item->stl_load_mode = load_mode;
        item->load_as_sdf = load_as_sdf;
        item->use_precise_voxelization = use_precise_voxelization;
        item->mesh_kd_tree = kdtree::KDTree(kdtree_points);
        {
            std::lock_guard<std::mutex> lock(locker);
            items[item->id] = std::move(item);
            update_nav_node_status = true;
        }
    }

    setQueueStatus(get_locale_string("status.uploading"));
    queue_progress = 0.95f;

    setQueueStatus(get_locale_string("status.done"));
    queue_progress = 1.0f;
}

std::filesystem::path RenderVoxelList::get_cache_dir(
    const std::string& subdir) const {
    std::filesystem::path base;
    if (!project_path.empty()) {
        base = utf8_path(project_path) / "cache" / subdir;
    } else {
        base = std::filesystem::temp_directory_path() / "kigstudio_cache" /
               subdir;
    }
    std::filesystem::create_directories(base);
    return base;
}

std::filesystem::path RenderVoxelList::get_mesh_cache_path(int node_id) const {
    return get_cache_dir("mesh") / (std::to_string(node_id) + ".stl");
}

std::filesystem::path RenderVoxelList::get_sdf_cache_path(int node_id) const {
    return get_cache_dir("sdf") / (std::to_string(node_id) + ".stl");
}

std::filesystem::path RenderVoxelList::get_voxel_cache_path(int node_id) const {
    return get_cache_dir("voxel") / (std::to_string(node_id) + ".vxgrid");
}

void RenderVoxelList::load_from_node(int target_item_id,
                                     int source_node_id,
                                     int node_source_data_type,
                                     int node_source_sdf_subdivisions,
                                     bool node_source_sdf_simplify,
                                     float node_source_sdf_simplify_ratio,
                                     int load_mode,
                                     bool load_as_sdf,
                                     bool use_precise_voxelization) {
    using namespace sinriv::kigstudio::voxel;
    using Triangle = triangle_bvh<float>::triangle;
    using vec3f = sinriv::kigstudio::vec3<float>;

    // Snapshot source data and mark target as writing under lock,
    // then perform heavy processing without holding locker so the UI stays
    // responsive.
    float voxel_size = 0.0f;
    vec3f silhouette_center{0.0f, 0.0f, 0.0f};
    std::vector<Triangle> source_triangles;
    VoxelGrid source_voxel_grid;
    std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> source_sdf;
    bool source_found = false;
    RenderVoxelItem* target_ptr = nullptr;
    int data_type = node_source_data_type;
    {
        std::lock_guard<std::mutex> lock(locker);
        auto source_it = items.find(source_node_id);
        auto target_it = items.find(target_item_id);
        if (source_it == items.end() || target_it == items.end()) {
            return;
        }
        auto& source = *source_it->second;
        target_ptr = target_it->second.get();
        target_ptr->ref_count++;
        target_ptr->write_count++;
        source_found = true;

        voxel_size = target_ptr->stl_voxel_size;
        silhouette_center = target_ptr->silhouette_center;

        source_triangles = source.source_triangles;
        source_voxel_grid = source.voxel_grid_data;
        source_sdf = source.sdf_data;
    }
    if (!source_found || !target_ptr) {
        return;
    }

    // Helper: voxelize a triangle set using the given voxel size
    auto voxelize_triangles = [&](const std::vector<Triangle>& triangles,
                                  VoxelGrid& out_voxel) {
        if (triangles.empty())
            return;
        triangle_bvh<float> bvh;
        size_t tri_idx = 0;
        const size_t tri_total = triangles.size();
        for (const auto& tri : triangles) {
            bvh.insert(tri);
            if (++tri_idx % 100 == 0) {
                queue_progress =
                    0.25f +
                    0.05f * (static_cast<float>(tri_idx) /
                             static_cast<float>(std::max(size_t(1), tri_total)));
                setQueueStatus(
                    get_locale_string("status.building_spatial_index"));
            }
        }

        out_voxel.voxel_size = {voxel_size, voxel_size, voxel_size};
        out_voxel.global_position = bvh.global_boundBox_min;

        float minx = floor(bvh.global_boundBox_min.x / voxel_size) * voxel_size;
        float miny = floor(bvh.global_boundBox_min.y / voxel_size) * voxel_size;
        float minz = floor(bvh.global_boundBox_min.z / voxel_size) * voxel_size;
        float maxx = ceil(bvh.global_boundBox_max.x / voxel_size) * voxel_size;
        float maxy = ceil(bvh.global_boundBox_max.y / voxel_size) * voxel_size;
        float maxz = ceil(bvh.global_boundBox_max.z / voxel_size) * voxel_size;
        int num_block_x =
            static_cast<int>(floor((maxx - minx) / voxel_size)) + 1;
        int num_block_y =
            static_cast<int>(floor((maxy - miny) / voxel_size)) + 1;
        int num_block_z =
            static_cast<int>(floor((maxz - minz) / voxel_size)) + 1;

        std::mutex candidate_locker;
        std::vector<Vec3i> candidates_x;
        std::vector<Vec3i> candidates_y;
        std::vector<Vec3i> candidates_z;
        candidates_x.reserve(1024);
        candidates_y.reserve(1024);
        candidates_z.reserve(1024);
        size_t total_rays = static_cast<size_t>(num_block_y) * num_block_z +
                            static_cast<size_t>(num_block_x) * num_block_z +
                            static_cast<size_t>(num_block_x) * num_block_y;
        if (total_rays == 0)
            total_rays = 1;
        std::atomic<size_t> callback_count{0};

        auto make_callback = [&](std::vector<Vec3i>& out) {
            return [&](auto start, auto end) {
                int start_x = static_cast<int>(std::round(
                    (start.x - out_voxel.global_position.x) / voxel_size));
                int start_y = static_cast<int>(std::round(
                    (start.y - out_voxel.global_position.y) / voxel_size));
                int start_z = static_cast<int>(std::round(
                    (start.z - out_voxel.global_position.z) / voxel_size));
                int end_x = static_cast<int>(std::round(
                    (end.x - out_voxel.global_position.x) / voxel_size));
                int end_y = static_cast<int>(std::round(
                    (end.y - out_voxel.global_position.y) / voxel_size));
                int end_z = static_cast<int>(std::round(
                    (end.z - out_voxel.global_position.z) / voxel_size));

                std::vector<Vec3i> local;
                local.reserve(std::max(0, end_x - start_x + 1) *
                              std::max(0, end_y - start_y + 1) *
                              std::max(0, end_z - start_z + 1));
                for (int i = start_x; i <= end_x; ++i) {
                    for (int j = start_y; j <= end_y; ++j) {
                        for (int k = start_z; k <= end_z; ++k) {
                            if (i >= 0 && j >= 0 && k >= 0) {
                                local.push_back({i, j, k});
                            }
                        }
                    }
                }
                if (!local.empty()) {
                    std::lock_guard<std::mutex> lock(candidate_locker);
                    out.insert(out.end(), local.begin(), local.end());
                }

                size_t cnt =
                    callback_count.fetch_add(1, std::memory_order_relaxed) + 1;
                if (cnt % 100 == 0) {
                    float p =
                        0.30f +
                        0.35f * std::min(1.0f, static_cast<float>(cnt) /
                                                   static_cast<float>(total_rays *
                                                                      2));
                    queue_progress = p;
                    setQueueStatus(get_locale_string("status.voxelizing"));
                }
            };
        };

        bvh.getSolidByFace(voxel_size, voxel_size, voxel_size,
                           triangle_bvh<float>::voxel_face_X,
                           make_callback(candidates_x));
        bvh.getSolidByFace(voxel_size, voxel_size, voxel_size,
                           triangle_bvh<float>::voxel_face_Y,
                           make_callback(candidates_y));
        bvh.getSolidByFace(voxel_size, voxel_size, voxel_size,
                           triangle_bvh<float>::voxel_face_Z,
                           make_callback(candidates_z));

        insert_voxels_with_optional_verify(out_voxel, candidates_x,
                                            candidates_y, candidates_z,
                                            triangles, voxel_size,
                                            use_precise_voxelization);
    };

    // Gather source mesh triangles according to node_source_data_type
    std::vector<Triangle> source_mesh;
    if (data_type == 0) {
        // Mesh
        source_mesh = std::move(source_triangles);
    } else if (data_type == 1) {
        // SDF
        if (source_sdf && !source_voxel_grid.chunks.empty()) {
            std::vector<std::tuple<Triangle, vec3f>> mesh;
            int numTriangles = 0;
            int sdf_mesh_progress = 0;
            for (auto tri : generateSmoothMeshFromSDF(
                     source_voxel_grid, numTriangles,
                     [&](const std::string& status) {
                         setQueueStatus(get_locale_string("status.sdf_mesh_prefix") + status);
                         return queue_should_continue.load() &&
                                queue_running.load();
                     },
                     true, node_source_sdf_subdivisions, source_sdf.get())) {
                mesh.push_back(tri);
                if (++sdf_mesh_progress % 100 == 0) {
                    queue_progress =
                        0.05f +
                        0.10f * std::min(1.0f, static_cast<float>(sdf_mesh_progress) / 50000.0f);
                }
            }
            if (!mesh.empty()) {
                mesh = cleanMesh(mesh);
            }
            if (node_source_sdf_simplify && !mesh.empty()) {
                mesh = sinriv::kigstudio::cgal::simplifyMesh(
                    mesh,
                    static_cast<double>(node_source_sdf_simplify_ratio));
            }
            source_mesh.reserve(mesh.size());
            for (size_t i = 0; i < mesh.size(); ++i) {
                source_mesh.push_back(std::get<0>(mesh[i]));
                if (i % 100 == 0) {
                    queue_progress =
                        0.15f +
                        0.05f * (static_cast<float>(i) /
                                 static_cast<float>(std::max(size_t(1), mesh.size())));
                    setQueueStatus(get_locale_string("status.preparing_mesh"));
                }
            }
        } else {
            source_mesh = std::move(source_triangles);
        }
    } else if (data_type == 2) {
        // Voxel
        if (!source_voxel_grid.chunks.empty()) {
            std::vector<std::tuple<Triangle, vec3f>> mesh;
            int numTriangles = 0;
            int voxel_mesh_progress = 0;
            for (auto tri : generateMesh(source_voxel_grid, 0.5, numTriangles,
                                          true)) {
                mesh.push_back(tri);
                if (++voxel_mesh_progress % 100 == 0) {
                    queue_progress =
                        0.05f +
                        0.10f * std::min(1.0f, static_cast<float>(voxel_mesh_progress) / 50000.0f);
                    setQueueStatus(
                        get_locale_string("status.generating_voxel_mesh"));
                }
            }
            source_mesh.reserve(mesh.size());
            for (size_t i = 0; i < mesh.size(); ++i) {
                source_mesh.push_back(std::get<0>(mesh[i]));
                if (i % 100 == 0) {
                    queue_progress =
                        0.15f +
                        0.05f * (static_cast<float>(i) /
                                 static_cast<float>(std::max(size_t(1), mesh.size())));
                    setQueueStatus(get_locale_string("status.preparing_mesh"));
                }
            }
        } else {
            source_mesh = std::move(source_triangles);
        }
    }

    // Keep a copy of the raw source mesh for origin_mesh_renderer
    std::vector<Triangle> origin_mesh_triangles = source_mesh;

    // Compute target data according to load_mode
    std::vector<Triangle> target_triangles;
    VoxelGrid target_voxel;
    std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> target_sdf;
    bool target_mesh_only = false;

    if (load_mode == static_cast<int>(StlLoadMode::SILHOUETTE)) {
        if (!source_mesh.empty()) {
            setQueueStatus(get_locale_string("status.generating_silhouette_mesh"));
            target_triangles =
                sinriv::kigstudio::mesh::conebox::
                    build_closed_mesh_from_triangles_silhouette(
                        source_mesh, silhouette_center,
                        [&]() { return queue_should_continue.load(); },
                        [&](float t, const std::string& step) {
                            queue_progress = t * 0.1f;
                            setQueueStatus(step);
                        });
            target_mesh_only = true;
        }
    } else if (load_mode == static_cast<int>(StlLoadMode::SURFACE_ONLY)) {
        if (!source_mesh.empty()) {
            target_triangles = source_mesh;
            VoxelGrid voxel_data;
            voxel_data.voxel_size = {voxel_size, voxel_size, voxel_size};
            voxel_data.global_position = {-0.5f * voxel_size,
                                          -0.5f * voxel_size,
                                          -0.5f * voxel_size};
            float precision = voxel_size / 10.0f;
            size_t surface_idx = 0;
            const size_t surface_total = target_triangles.size();
            for (const auto& tri : target_triangles) {
                for (auto point : sinriv::kigstudio::voxel::draw_triangle(
                         tri, voxel_size, voxel_size, voxel_size,
                         precision)) {
                    voxel_data.insert(voxel_data.worldToVoxel(point));
                }
                if (++surface_idx % 100 == 0) {
                    queue_progress =
                        0.30f +
                        0.20f * (static_cast<float>(surface_idx) /
                                 static_cast<float>(std::max(size_t(1), surface_total)));
                    setQueueStatus(
                        get_locale_string("status.rasterizing_surface"));
                }
            }
            target_voxel = std::move(voxel_data);
        }
    } else if (load_mode == static_cast<int>(StlLoadMode::MESH_ONLY)) {
        target_triangles = std::move(source_mesh);
        target_mesh_only = true;
    } else if (load_mode == static_cast<int>(StlLoadMode::CONVEX_HULL)) {
        if (!source_mesh.empty()) {
            target_triangles =
                sinriv::kigstudio::cgal::convexHull3(source_mesh);
            if (!target_triangles.empty()) {
                VoxelGrid voxel_data;
                voxelize_triangles(target_triangles, voxel_data);
                target_voxel = std::move(voxel_data);
            }
        }
    } else {
        // Default: follow node_source_data_type
        if (data_type == 0) {
            // Mesh: voxelize source triangles like file mode DEFAULT
            target_triangles = std::move(source_mesh);
            if (!target_triangles.empty()) {
                VoxelGrid voxel_data;
                voxelize_triangles(target_triangles, voxel_data);
                target_voxel = std::move(voxel_data);
            }
        } else if (data_type == 1) {
            // SDF: copy SDF and voxel grid from source
            target_voxel = source_voxel_grid;
            target_sdf = source_sdf;
            target_triangles = std::move(source_triangles);
        } else if (data_type == 2) {
            // Voxel
            target_voxel = source_voxel_grid;
            target_triangles = std::move(source_triangles);
        }
    }

    // Generate SDF from the loaded/reconstructed triangle mesh if requested.
    // For Silhouette / Convex Hull the processed mesh is used;
    // for Default the raw loaded mesh (or SDF-reconstructed mesh) is used.
    if (load_as_sdf && (data_type == 0 || data_type == 1) &&
        load_mode != static_cast<int>(StlLoadMode::SURFACE_ONLY) &&
        load_mode != static_cast<int>(StlLoadMode::MESH_ONLY)) {
        auto mesh_sdf = std::make_shared<sinriv::kigstudio::sdf::SDF_Mesh>();
        if (load_mode == static_cast<int>(StlLoadMode::SILHOUETTE) ||
            load_mode == static_cast<int>(StlLoadMode::CONVEX_HULL)) {
            if (!target_triangles.empty()) {
                mesh_sdf->loadTriangles(target_triangles);
                target_sdf = std::move(mesh_sdf);
            }
        } else {
            if (!source_mesh.empty()) {
                mesh_sdf->loadTriangles(source_mesh);
                target_sdf = std::move(mesh_sdf);
            }
        }
    }

    // 在拿 locker 之前在 CPU 侧生成 chunk mesh，避免阻塞 UI 线程。
    std::unordered_map<
        uint64_t,
        std::vector<std::tuple<Triangle, sinriv::kigstudio::voxel::vec3f>>>
        target_chunk_meshes;
    if (!target_voxel.chunks.empty()) {
        target_chunk_meshes = generateChunkMeshData(target_voxel, 0.5, true);
    }

    // Apply results to target under lock
    {
        std::lock_guard<std::mutex> lock(locker);
        auto target_it = items.find(target_item_id);
        if (target_it != items.end()) {
            auto& target = *target_it->second;
            target.mesh_renderer.clear();
            target.exported_mesh_renderer.clear();
            target.cached_mesh.clear();
            target.cached_mesh_dirty = true;
            target.voxel_renderer.clear();
            target.sdf_data = nullptr;
            target.source_triangles.clear();
            target.voxel_grid_data.chunks.clear();
            target.mesh_only = false;
            target.stl_path.clear();

            target.origin_mesh_renderer.clear();
            target.origin_mesh_renderer.setBaseColor(0.0f, 0.0f, 1.0f, 1.0f);
            if (!origin_mesh_triangles.empty()) {
                target.origin_mesh_renderer.loadGeometry(
                    triangle_generator_with_normals(origin_mesh_triangles));
            }

            if (!target_triangles.empty()) {
                target.source_triangles = std::move(target_triangles);
                target.mesh_renderer.loadGeometry(
                    triangle_generator_with_normals(target.source_triangles));
            }
            if (!target_voxel.chunks.empty()) {
                target.voxel_grid_data = std::move(target_voxel);
                target.voxel_renderer.loadChunkMeshes(target_chunk_meshes);
            }
            if (target_sdf) {
                target.sdf_data = std::move(target_sdf);
            }
            target.mesh_only = target_mesh_only;
            target.use_precise_voxelization = use_precise_voxelization;
            target.thumbnail_dirty = true;
            target.dirty = true;
        }
        target_ptr->ref_count--;
        target_ptr->write_count--;
    }

    // Write cache for source node data
    try {
        if (data_type == 0) {
            auto path = get_mesh_cache_path(source_node_id);
            std::vector<std::tuple<Triangle, vec3f>> mesh;
            mesh.reserve(source_triangles.size());
            for (size_t i = 0; i < source_triangles.size(); ++i) {
                const auto& tri = source_triangles[i];
                mesh.push_back(
                    {tri, sinriv::kigstudio::voxel::calcTriangleNormal(tri)});
                if (i % 100 == 0) {
                    queue_progress =
                        0.90f +
                        0.05f * (static_cast<float>(i) /
                                 static_cast<float>(std::max(
                                     size_t(1), source_triangles.size())));
                    setQueueStatus(get_locale_string("status.saving_cache"));
                }
            }
            sinriv::kigstudio::voxel::saveMeshToASCIISTL(mesh, path.string());
        } else if (data_type == 1) {
            auto path = get_sdf_cache_path(source_node_id);
            std::vector<std::tuple<Triangle, vec3f>> mesh;
            mesh.reserve(target_triangles.size());
            for (size_t i = 0; i < target_triangles.size(); ++i) {
                const auto& tri = target_triangles[i];
                mesh.push_back(
                    {tri, sinriv::kigstudio::voxel::calcTriangleNormal(tri)});
                if (i % 100 == 0) {
                    queue_progress =
                        0.90f +
                        0.05f * (static_cast<float>(i) /
                                 static_cast<float>(std::max(
                                     size_t(1), target_triangles.size())));
                    setQueueStatus(get_locale_string("status.saving_cache"));
                }
            }
            sinriv::kigstudio::voxel::saveMeshToASCIISTL(mesh, path.string());
        } else if (data_type == 2) {
            auto path = get_voxel_cache_path(source_node_id);
            std::string error;
            sinriv::kigstudio::save(path.string(), source_voxel_grid, &error);
        }
    } catch (const std::exception& e) {
        std::cerr << "[load_from_node] cache write failed: " << e.what()
                  << std::endl;
    }
}
}  // namespace sinriv::ui::render
