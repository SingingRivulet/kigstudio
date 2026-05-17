#include "render_voxel_list.h"
#include "kigstudio/cgal/skeleton_extraction.h"
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <set>

namespace sinriv::ui::render {
namespace {
using SkeletonPoint = sinriv::kigstudio::voxel::vec3f;
using SkeletonLine = std::pair<SkeletonPoint, SkeletonPoint>;

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

VoxelKey makeVoxelKey(const sinriv::kigstudio::voxel::Vec3i& p) {
    return {p.x, p.y, p.z};
}

bool voxelLess(const sinriv::kigstudio::voxel::Vec3i& a,
               const sinriv::kigstudio::voxel::Vec3i& b) {
    if (a.x != b.x)
        return a.x < b.x;
    if (a.y != b.y)
        return a.y < b.y;
    return a.z < b.z;
}

std::vector<RenderVoxelList::RenderVoxelItem::SkeletonPointPick>
buildSkeletonOrderCache(
    const std::vector<std::pair<sinriv::kigstudio::voxel::Vec3i,
                                sinriv::kigstudio::voxel::Vec3i>>& lines,
    const sinriv::kigstudio::voxel::VoxelGrid& voxel_grid,
    std::unordered_map<VoxelKey, int, VoxelKeyHash>& order_by_voxel) {
    using Vec3i = sinriv::kigstudio::voxel::Vec3i;
    using Pick = RenderVoxelList::RenderVoxelItem::SkeletonPointPick;

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

    queue_status = "Segmenting...";
    queue_progress = 0.0f;
    std::vector<sinriv::kigstudio::voxel::VoxelGrid> grids;
    std::cout << "[do_segment] start item=" << index
              << " mode=" << it->second->segment_mode
              << " write_count=" << it->second->write_count << std::endl;
    try {
        grids = it->second->do_segment();
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
        size_t num_results = grids.size();
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
    for (size_t i = 0; i < grids.size(); ++i) {
        auto new_item = std::make_unique<RenderVoxelItem>();
        new_item->manager = this;
        new_item->id = new_ids[i];
        new_item->children = new_children[i];
        new_item->auto_segment_update = new_auto_segment_update[i];
        new_item->voxel_grid_data = std::move(grids[i]);
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

    queue_status = "Extracting skeleton...";
    queue_progress = 0.0f;
    std::vector<std::pair<sinriv::kigstudio::voxel::vec3f,
                          sinriv::kigstudio::voxel::vec3f>>
        chain_lines;
    std::vector<RenderVoxelItem::SurfaceSkeletonCacheEntry>
        surface_skeleton_world_cache;
    std::vector<RenderVoxelItem::SkeletonPointPick> skeleton_order_cache;
    bool cgal_succeeded = false;

    // --- CGAL mesh-based skeleton path ---
    if (it->second->use_cgal_skeleton && !it->second->source_triangles.empty()) {
        try {
            queue_status = "CGAL skeleton extraction...";
            auto cgal_lines = sinriv::kigstudio::cgal::extractSkeletonFromMesh(
                it->second->source_triangles);
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
                            RenderVoxelItem::SkeletonPointPick pick;
                            pick.position = p;
                            pick.order = order++;
                            skeleton_order_cache.push_back(pick);
                        }
                    }
                }
                cgal_succeeded = true;
            }
        } catch (const std::exception& e) {
            std::cerr << "[CGAL Skeleton] failed: " << e.what()
                      << ", falling back to voxel method.\n";
        }
    }

    // --- Voxel-based skeleton path (fallback) ---
    if (!cgal_succeeded) try {
        char res_buf[512];
        queue_status =
            get_locale_cstr("progress.extract_skeleton.buildDenseGrid");
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
        queue_status = get_locale_cstr("progress.extract_skeleton.computeEDT");
        kigstudio::voxel::computeEDT(dense);
        queue_progress = 0.5f;
        queue_status = get_locale_cstr("progress.extract_skeleton.finalizeEDT");
        kigstudio::voxel::finalizeEDT(dense);
        queue_progress = 0.75f;
        queue_status =
            get_locale_cstr("progress.extract_skeleton.extractCenterline");
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

        auto to_world = [&voxel_grid](const kigstudio::voxel::Vec3i& voxel) {
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
                               int target_item_id) {
    using namespace sinriv::kigstudio::voxel;
    using MeshData = mesh_detail::AsyncVoxelMeshData;
    using Triangle = triangle_bvh<float>::triangle;

    // Phase 1: Read STL
    queue_progress = 0.05f;
    triangle_bvh<float> bvh;
    size_t tri_count = 0;
    kdtree::pointVec kdtree_points;
    std::vector<Triangle> source_triangles;
    source_triangles.reserve(1024);
    for (auto [tri, n] : sinriv::kigstudio::voxel::readSTL(filename)) {
        (void)n;
        bvh.insert(tri);
        source_triangles.push_back(tri);
        kdtree_points.push_back({static_cast<double>(std::get<0>(tri).x),
                                 static_cast<double>(std::get<0>(tri).y),
                                 static_cast<double>(std::get<0>(tri).z)});
        kdtree_points.push_back({static_cast<double>(std::get<1>(tri).x),
                                 static_cast<double>(std::get<1>(tri).y),
                                 static_cast<double>(std::get<1>(tri).z)});
        kdtree_points.push_back({static_cast<double>(std::get<2>(tri).x),
                                 static_cast<double>(std::get<2>(tri).y),
                                 static_cast<double>(std::get<2>(tri).z)});
        ++tri_count;
        if (tri_count % 1000 == 0) {
            queue_progress =
                (0.05f + 0.08f * std::min(1.0f, static_cast<float>(tri_count) /
                                                    50000.0f));
        }
    }

    // Phase 2: Voxelize
    queue_status = "Voxelizing...";
    queue_progress = 0.15f;

    VoxelGrid voxel_data;
    voxel_data.voxel_size.x = voxel_size;
    voxel_data.voxel_size.y = voxel_size;
    voxel_data.voxel_size.z = voxel_size;
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

    std::mutex bvh_locker;
    std::atomic<size_t> callback_count{0};

    auto voxel_callback = [&](auto start, auto end) {
        int start_x = static_cast<int>(
            std::round((start.x - voxel_data.global_position.x) / voxel_size));
        int start_y = static_cast<int>(
            std::round((start.y - voxel_data.global_position.y) / voxel_size));
        int start_z = static_cast<int>(
            std::round((start.z - voxel_data.global_position.z) / voxel_size));
        int end_x = static_cast<int>(
            std::round((end.x - voxel_data.global_position.x) / voxel_size));
        int end_y = static_cast<int>(
            std::round((end.y - voxel_data.global_position.y) / voxel_size));
        int end_z = static_cast<int>(
            std::round((end.z - voxel_data.global_position.z) / voxel_size));

        bvh_locker.lock();
        for (int i = start_x; i <= end_x; ++i) {
            for (int j = start_y; j <= end_y; ++j) {
                for (int k = start_z; k <= end_z; ++k) {
                    if (i >= 0 && j >= 0 && k >= 0) {
                        voxel_data.insert(i, j, k);
                    }
                }
            }
        }
        bvh_locker.unlock();

        size_t cnt = callback_count.fetch_add(1, std::memory_order_relaxed) + 1;
        if (cnt % 100 == 0) {
            float p =
                0.15f +
                0.35f * std::min(1.0f, static_cast<float>(cnt) /
                                           static_cast<float>(total_rays * 2));
            queue_progress = p;
        }
    };

    for (auto face :
         {triangle_bvh<float>::voxel_face_X, triangle_bvh<float>::voxel_face_Y,
          triangle_bvh<float>::voxel_face_Z}) {
        bvh.getSolidByFace(voxel_size, voxel_size, voxel_size, face,
                           voxel_callback);
    }

    queue_progress = 0.50f;

    // Phase 3: Generate chunked mesh
    queue_status = "Generating mesh...";
    queue_progress = 0.75f;

    if (target_item_id >= 0) {
        // 更新现有 item
        {
            std::lock_guard<std::mutex> lock(locker);
            auto it = items.find(target_item_id);
            if (it != items.end()) {
                auto& item = *it->second;
                item.mesh_renderer.clear();
                item.mesh_renderer.loadGeometry(
                    sinriv::kigstudio::voxel::readSTL(filename));
                item.voxel_renderer.clear();
                item.voxel_renderer.loadVoxelGridChunked(voxel_data, isolevel,
                                                         smooth_normals);
                item.voxel_grid_data = std::move(voxel_data);
                item.source_triangles = std::move(source_triangles);
                item.stl_voxel_size = voxel_size;
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
        item->mesh_renderer.loadGeometry(
            sinriv::kigstudio::voxel::readSTL(filename));
        item->voxel_renderer.loadVoxelGridChunked(voxel_data, isolevel,
                                                  smooth_normals);
        item->voxel_grid_data = std::move(voxel_data);
        item->source_triangles = std::move(source_triangles);
        item->thumbnail_dirty = true;
        item->stl_path = filename;
        item->stl_voxel_size = voxel_size;
        item->mesh_kd_tree = kdtree::KDTree(kdtree_points);
        {
            std::lock_guard<std::mutex> lock(locker);
            items[item->id] = std::move(item);
            update_nav_node_status = true;
        }
    }

    queue_status = "Uploading...";
    queue_progress = 0.95f;

    queue_status = "Done";
    queue_progress = 1.0f;
}
}  // namespace sinriv::ui::render
