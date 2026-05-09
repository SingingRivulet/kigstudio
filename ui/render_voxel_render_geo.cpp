#include "render_voxel_list.h"
namespace sinriv::ui::render {
std::vector<RenderVoxelList::RenderVoxelItem*>
RenderVoxelList::do_segment(int index) {
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
    auto grids = it->second->do_segment();
    queue_progress = 0.7f;

    std::vector<int> new_ids;
    std::vector<std::vector<int>> new_children;
    std::vector<bool> new_auto_segment_update;
    std::vector<std::optional<CollisionEditorSnapshot>> child_snapshots;
    {
        std::lock_guard<std::mutex> lock(locker);
        it->second->ref_count--;
        it->second->write_count--;
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
                        pending_deletion.push_back(
                            std::move(it_child->second));
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
    for (auto [tri, n] : sinriv::kigstudio::voxel::readSTL(filename)) {
        (void)n;
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
    int num_block_y = static_cast<int>(ceil((maxy - miny) / voxel_size));
    int num_block_z = static_cast<int>(ceil((maxz - minz) / voxel_size));
    size_t total_rays = static_cast<size_t>(num_block_y) * num_block_z;
    if (total_rays == 0)
        total_rays = 1;

    std::mutex bvh_locker;
    std::atomic<size_t> callback_count{0};

    bvh.getSolidByFace(
        voxel_size, voxel_size, voxel_size, triangle_bvh<float>::voxel_face_X,
        [&](auto start, auto end) {
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

            size_t cnt =
                callback_count.fetch_add(1, std::memory_order_relaxed) + 1;
            if (cnt % 100 == 0) {
                float p =
                    0.15f + 0.35f * std::min(1.0f, static_cast<float>(cnt) /
                                                       static_cast<float>(
                                                           total_rays * 2));
                queue_progress = p;
            }
        });

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
                item.voxel_renderer.loadVoxelGridChunked(
                    voxel_data, isolevel, smooth_normals);
                item.voxel_grid_data = std::move(voxel_data);
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
        item->mesh_renderer.loadGeometry(
            sinriv::kigstudio::voxel::readSTL(filename));
        item->voxel_renderer.loadVoxelGridChunked(
            voxel_data, isolevel, smooth_normals);
        item->voxel_grid_data = std::move(voxel_data);
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