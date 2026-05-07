#include "render_voxel_list.h"
namespace sinriv::ui::render {

CollisionEditorSnapshot RenderVoxelList::capture_snapshot(
    const RenderVoxelItem& item) const {
    return {item.collision_group, item.plane, item.concave_cone,
            item.concave_cone_expanded_vertices, item.segment_mode};
}

void RenderVoxelList::apply_snapshot(RenderVoxelItem& item,
                                     const CollisionEditorSnapshot& snapshot) {
    item.collision_group = snapshot.collision_group;
    item.plane = snapshot.plane;
    item.concave_cone = snapshot.concave_cone;
    item.concave_cone_expanded_vertices =
        snapshot.concave_cone_expanded_vertices;
    item.segment_mode =
        static_cast<RenderVoxelItem::SegmentMode>(snapshot.segment_mode);
}

void RenderVoxelList::begin_edit(int item_id) {
    if (pending_undo.has_value() && pending_undo->item_id == item_id) {
        return;  // already have pending undo for this item
    }
    // discard pending for different item
    pending_undo.reset();
    auto it = items.find(item_id);
    if (it != items.end()) {
        pending_undo = PendingUndo{item_id, capture_snapshot(*it->second)};
    }
}

void RenderVoxelList::end_edit(int item_id, const std::string& desc) {
    if (!pending_undo.has_value() || pending_undo->item_id != item_id) {
        return;
    }
    auto it = items.find(item_id);
    if (it != items.end()) {
        it->second->undo_stack.push_back(pending_undo->snapshot);
        it->second->undo_stack.back().description = desc;
        it->second->redo_stack.clear();
        it->second->dirty = true;
        if (it->second->undo_stack.size() > kMaxUndoSize) {
            it->second->undo_stack.erase(it->second->undo_stack.begin());
        }
    }
    pending_undo.reset();
}

void RenderVoxelList::push_undo_now(
    int item_id,
    const std::optional<CollisionEditorSnapshot>& before,
    const std::string& desc) {
    auto it = items.find(item_id);
    if (it == items.end())
        return;
    it->second->undo_stack.push_back(
        before.value_or(capture_snapshot(*it->second)));
    it->second->undo_stack.back().description = desc;
    it->second->redo_stack.clear();
    it->second->dirty = true;
    if (it->second->undo_stack.size() > kMaxUndoSize) {
        it->second->undo_stack.erase(it->second->undo_stack.begin());
    }
    pending_undo.reset();
}

bool RenderVoxelList::undo(int item_id) {
    auto it = items.find(item_id);
    if (it == items.end() || it->second->undo_stack.empty())
        return false;
    // push current state to redo stack
    it->second->redo_stack.push_back(capture_snapshot(*it->second));
    // apply undo snapshot
    apply_snapshot(*it->second, it->second->undo_stack.back());
    it->second->undo_stack.pop_back();
    it->second->dirty = true;
    return true;
}

bool RenderVoxelList::redo(int item_id) {
    auto it = items.find(item_id);
    if (it == items.end() || it->second->redo_stack.empty())
        return false;
    // push current state to undo stack
    it->second->undo_stack.push_back(capture_snapshot(*it->second));
    // apply redo snapshot
    apply_snapshot(*it->second, it->second->redo_stack.back());
    it->second->redo_stack.pop_back();
    it->second->dirty = true;
    return true;
}

bool RenderVoxelList::can_undo(int item_id) const {
    auto it = items.find(item_id);
    return it != items.end() && !it->second->undo_stack.empty();
}

bool RenderVoxelList::can_redo(int item_id) const {
    auto it = items.find(item_id);
    return it != items.end() && !it->second->redo_stack.empty();
}

bool RenderVoxelList::has_dirty_items() const {
    for (const auto& [id, item] : items) {
        if (item->dirty)
            return true;
    }
    return false;
}

void RenderVoxelList::clear_all_dirty() {
    for (auto& [id, item] : items) {
        item->dirty = false;
    }
}

}  // namespace sinriv::ui::render