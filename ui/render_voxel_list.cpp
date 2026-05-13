#include "render_voxel_list.h"
#include <cstring>
#include <limits>
namespace sinriv::ui::render {

void RenderVoxelList::setViewportSize(int width, int height) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->mesh_renderer.setViewportSize(width, height);
        it->second->voxel_renderer.setViewportSize(width, height);
    }
}

void RenderVoxelList::setViewProjection(const float* view, const float* proj) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->mesh_renderer.setViewProjection(view, proj);
        it->second->voxel_renderer.setViewProjection(view, proj);
    }
}

void RenderVoxelList::setModelMatrix(const mat4f& model_matrix) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->mesh_renderer.setModelMatrix(model_matrix);
        it->second->voxel_renderer.setModelMatrix(model_matrix);
    }
}

void RenderVoxelList::setMeshAxisVisible(bool visible) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->mesh_renderer.showAxis = visible;
    }
}

void RenderVoxelList::setVoxelAxisVisible(bool visible) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->voxel_renderer.showAxis = visible;
    }
}

void RenderVoxelList::setMeshVisible(bool visible) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->showMesh = visible;
    }
}
void RenderVoxelList::setVoxelsVisible(bool visible) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->showVoxel = visible;
    }
}
void RenderVoxelList::setCollisionVisible(bool visible) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->showCollision = visible;
    }
}
void RenderVoxelList::setCollisionBoundsVisible(bool visible) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->showCollisionBounds = visible;
    }
}

// 交互

RenderVoxelList::RenderVoxelItem* RenderVoxelList::create_item() {
    auto item = std::make_unique<RenderVoxelItem>();
    item->manager = this;
    item->id = current_id++;
    std::cout << "[create_item] id=" << item->id
              << " write_count=" << item->write_count.load()
              << " ref_count=" << item->ref_count.load() << std::endl;
    auto item_ptr = item.get();
    {
        std::lock_guard<std::mutex> lock(locker);
        items[item->id] = std::move(item);
        update_nav_node_status = true;
    }
    return item_ptr;
}

void RenderVoxelList::setRenderId(int id) {
    std::lock_guard<std::mutex> lock(locker);
    this->setRenderId_unsafe(id);
}

void RenderVoxelList::setRenderId_unsafe(int id) {
    auto it = items.find(id);
    if (it != items.end()) {
        render_id = id;
    }
}

void RenderVoxelList::brush_marked_voxels(
    const sinriv::kigstudio::voxel::vec3f& world_pos,
    float range, bool remove) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it == items.end()) return;
    auto& item = *it->second;
    if (!item.voxel_picking_enabled || !item.surface_cache_ready) return;

    item.marked_voxels.global_position = item.voxel_grid_data.global_position;
    item.marked_voxels.voxel_size = item.voxel_grid_data.voxel_size;

    auto center = item.voxel_grid_data.worldToVoxel(world_pos);
    float range_sq = range * range;
    std::vector<sinriv::kigstudio::voxel::Vec3i> to_mark;
    for (const auto& v : item.surface_voxels) {
        float dx = float(v.x - center.x);
        float dy = float(v.y - center.y);
        float dz = float(v.z - center.z);
        if (dx * dx + dy * dy + dz * dz <= range_sq) {
            to_mark.push_back(v);
        }
    }
    if (!to_mark.empty()) {
        if (remove) {
            item.marked_voxels.removeMany(to_mark);
        } else {
            item.marked_voxels.insertMany(to_mark);
        }
        item.marked_voxels_dirty = true;
    }
}

void RenderVoxelList::update_mouse_pos(RenderDeferred& deferred_renderer) {
    if (mouse_world_pos_picked_auto_snapping &&
        deferred_renderer.mouse_highlight_[0] > 0.5f) {
        auto it = items.find(render_id);
        if (it != items.end()) {
            auto kdtree_ptr = &it->second->mesh_kd_tree;
            if (kdtree_ptr->empty()) {
                auto pit = items.find(it->second->root_id);
                if (pit != items.end()) {
                    kdtree_ptr = &pit->second->mesh_kd_tree;
                }
            }
            if (!kdtree_ptr->empty()) {
                auto p = kdtree_ptr->nearest_point(
                    {deferred_renderer.mouse_pos_[0],
                     deferred_renderer.mouse_pos_[1],
                     deferred_renderer.mouse_pos_[2]});
                deferred_renderer.mouse_pos_[0] = p[0];
                deferred_renderer.mouse_pos_[1] = p[1];
                deferred_renderer.mouse_pos_[2] = p[2];
            }
        }
    }
    mouse_world_pos_valid = deferred_renderer.mouse_highlight_[0] > 0.5f;
    if (mouse_world_pos_valid) {
        mouse_world_pos.x = deferred_renderer.mouse_pos_[0];
        mouse_world_pos.y = deferred_renderer.mouse_pos_[1];
        mouse_world_pos.z = deferred_renderer.mouse_pos_[2];
    } else {
        mouse_world_pos = {0, 0, 0};
    }
}

void RenderVoxelList::pick_skeleton_point_from_mouse() {
    if (!mouse_world_pos_picked || !mouse_world_pos_valid)
        return;

    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it == items.end())
        return;

    auto& item = *it->second;
    if (item.surface_skeleton_cache.empty())
        return;

    const float pick_range = std::max(mouse_highlight_range, item.voxel_pick_range);
    const float pick_range_sq = pick_range * pick_range;
    float best_dist_sq = std::numeric_limits<float>::max();
    const sinriv::kigstudio::voxel::vec3f* best_skeleton = nullptr;

    for (const auto& [surface_voxel, skeleton_pos] :
         item.surface_skeleton_cache) {
        const auto surface_world =
            item.voxel_grid_data.voxelCenterToWorld(surface_voxel);
        const float dx = surface_world.x - mouse_world_pos.x;
        const float dy = surface_world.y - mouse_world_pos.y;
        const float dz = surface_world.z - mouse_world_pos.z;
        const float dist_sq = dx * dx + dy * dy + dz * dz;
        if (dist_sq > pick_range_sq || dist_sq >= best_dist_sq)
            continue;

        best_dist_sq = dist_sq;
        best_skeleton = &skeleton_pos;
    }

    if (best_skeleton == nullptr)
        return;

    item.picked_skeleton_points.push_back(*best_skeleton);
}

void RenderVoxelList::begin_marked_edit(int item_id) {
    if (pending_marked_undo.has_value() && pending_marked_undo->item_id == item_id) {
        return;
    }
    pending_marked_undo.reset();
    auto it = items.find(item_id);
    if (it != items.end()) {
        pending_marked_undo = PendingMarkedUndo{
            item_id, MarkedVoxelsSnapshot{it->second->marked_voxels, ""}};
    }
}

void RenderVoxelList::end_marked_edit(int item_id, const std::string& desc) {
    if (!pending_marked_undo.has_value() || pending_marked_undo->item_id != item_id) {
        return;
    }
    auto it = items.find(item_id);
    if (it != items.end()) {
        // Only push if state actually changed
        const auto& before = pending_marked_undo->snapshot.marked_voxels;
        const auto& after = it->second->marked_voxels;
        bool changed = (before.chunks.size() != after.chunks.size());
        if (!changed) {
            for (const auto& [key, chunk] : before.chunks) {
                auto ait = after.chunks.find(key);
                if (ait == after.chunks.end()) {
                    changed = true;
                    break;
                }
                if (memcmp(chunk.data, ait->second.data, sizeof(chunk.data)) != 0) {
                    changed = true;
                    break;
                }
            }
            if (!changed) {
                for (const auto& [key, chunk] : after.chunks) {
                    if (before.chunks.find(key) == before.chunks.end()) {
                        changed = true;
                        break;
                    }
                }
            }
        }
        if (changed) {
            it->second->marked_undo_stack.push_back(
                MarkedVoxelsSnapshot{before, desc});
            it->second->marked_redo_stack.clear();
            if (it->second->marked_undo_stack.size() > kMaxUndoSize) {
                it->second->marked_undo_stack.erase(
                    it->second->marked_undo_stack.begin());
            }
        }
    }
    pending_marked_undo.reset();
}

void RenderVoxelList::push_marked_undo_now(int item_id, const std::string& desc) {
    auto it = items.find(item_id);
    if (it == items.end()) return;
    it->second->marked_undo_stack.push_back(
        MarkedVoxelsSnapshot{it->second->marked_voxels, desc});
    it->second->marked_redo_stack.clear();
    if (it->second->marked_undo_stack.size() > kMaxUndoSize) {
        it->second->marked_undo_stack.erase(
            it->second->marked_undo_stack.begin());
    }
    pending_marked_undo.reset();
}

void RenderVoxelList::undo_marked(int item_id) {
    auto it = items.find(item_id);
    if (it == items.end() || it->second->marked_undo_stack.empty()) return;
    it->second->marked_redo_stack.push_back(
        MarkedVoxelsSnapshot{it->second->marked_voxels, ""});
    it->second->marked_voxels = it->second->marked_undo_stack.back().marked_voxels;
    it->second->marked_undo_stack.pop_back();
    it->second->marked_voxels_dirty = true;
}

void RenderVoxelList::redo_marked(int item_id) {
    auto it = items.find(item_id);
    if (it == items.end() || it->second->marked_redo_stack.empty()) return;
    it->second->marked_undo_stack.push_back(
        MarkedVoxelsSnapshot{it->second->marked_voxels, ""});
    it->second->marked_voxels = it->second->marked_redo_stack.back().marked_voxels;
    it->second->marked_redo_stack.pop_back();
    it->second->marked_voxels_dirty = true;
}

bool RenderVoxelList::can_undo_marked(int item_id) const {
    auto it = items.find(item_id);
    return it != items.end() && !it->second->marked_undo_stack.empty();
}

bool RenderVoxelList::can_redo_marked(int item_id) const {
    auto it = items.find(item_id);
    return it != items.end() && !it->second->marked_redo_stack.empty();
}

}  // namespace sinriv::ui::render
