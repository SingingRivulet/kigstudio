#include "render_voxel_list.h"
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
    mouse_world_pos.x = deferred_renderer.mouse_pos_[0];
    mouse_world_pos.y = deferred_renderer.mouse_pos_[1];
    mouse_world_pos.z = deferred_renderer.mouse_pos_[2];
    mouse_world_pos_valid = deferred_renderer.mouse_highlight_[0] > 0.5f;
}

}  // namespace sinriv::ui::render
