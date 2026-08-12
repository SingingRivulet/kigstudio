#include "render_voxel_list.h"
namespace sinriv::ui::render {

CollisionEditorSnapshot RenderVoxelList::capture_snapshot(
    const RenderVoxelItem& item) const {
    return {item.collision_group,
            item.plane,
            item.concave_cone,
            item.concave_cone_expanded_vertices,
            item.segment_mode,
            "",
            item.sdf_split_target_id,
            item.sdf_split_translation,
            item.sdf_split_rotation,
            item.sdf_split_scale,
            item.chain_min_radius,
            item.use_cgal_skeleton,
            item.picked_skeleton_points,
            item.skeleton_lines,
            item.stl_path,
            item.stl_load_mode,
            item.load_as_sdf,
            item.voxel_precision,
            item.sdf_precision_cache,
            item.mesh_only,
            item.source_type,
            item.source_node_id,
            item.node_source_data_type,
            item.node_source_sdf_subdivisions,
            item.node_source_sdf_simplify,
            item.node_source_sdf_simplify_ratio,
            item.silhouette_center,
            item.showSilhouetteCenter,
            item.addon_center_point,
            item.show_addon_center,
            item.auto_hair_root,
            item.common_hair_root_point,
            item.hair_root_center_offset,
            item.hair_root_vector_length,
            item.hairline_plane_enabled,
            item.hairline_plane_use_y,
            item.hairline_plane_y,
            {item.hairline_plane_points[0], item.hairline_plane_points[1],
             item.hairline_plane_points[2]},
            item.hairline_spindle_scale,
            item.silhouette_shape_mode,
            item.silhouette_subdivision,
            item.silhouette_edge_subdiv,
            item.inner_wall_radius,
            item.simplify_ratio,
            item.repair_mode,
            item.alpha_wrap_alpha,
            item.alpha_wrap_offset,
            item.subdivide_level,
            item.hair_strands,
            item.addon_reveal,
            item.addon_split,
            item.addon_sdf_boolean,
            item.addon_sdf_split,
            item.hair_angle_config,
            item.hair_north_pole,
            item.hair_front_reference,
            item.addon_base_node_id,
            item.drill_paths,
            item.show_connection_faces};
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
    item.sdf_split_target_id = snapshot.sdf_split_target_id;
    item.sdf_split_translation = snapshot.sdf_split_translation;
    item.sdf_split_rotation = snapshot.sdf_split_rotation;
    item.sdf_split_scale = snapshot.sdf_split_scale;
    item.chain_min_radius = snapshot.chain_min_radius;
    item.use_cgal_skeleton = snapshot.use_cgal_skeleton;
    item.picked_skeleton_points = snapshot.picked_skeleton_points;
    item.skeleton_lines = snapshot.skeleton_lines;
    item.stl_path = snapshot.stl_path;
    item.stl_load_mode = snapshot.stl_load_mode;
    item.load_as_sdf = snapshot.load_as_sdf;
    item.voxel_precision = snapshot.voxel_precision;
    item.mesh_only = snapshot.mesh_only;
    item.source_type = snapshot.source_type;
    item.source_node_id = snapshot.source_node_id;
    item.node_source_data_type = snapshot.node_source_data_type;
    item.node_source_sdf_subdivisions = snapshot.node_source_sdf_subdivisions;
    item.node_source_sdf_simplify = snapshot.node_source_sdf_simplify;
    item.node_source_sdf_simplify_ratio = snapshot.node_source_sdf_simplify_ratio;
    item.silhouette_center = snapshot.silhouette_center;
    item.showSilhouetteCenter = snapshot.show_silhouette_center;
    item.addon_center_point = snapshot.addon_center_point;
    item.show_addon_center = snapshot.show_addon_center;
    item.auto_hair_root = snapshot.auto_hair_root;
    item.common_hair_root_point = snapshot.common_hair_root_point;
    item.hair_root_center_offset = snapshot.hair_root_center_offset;
    item.hair_root_vector_length = snapshot.hair_root_vector_length;
    item.hairline_plane_enabled = snapshot.hairline_plane_enabled;
    item.hairline_plane_use_y = snapshot.hairline_plane_use_y;
    item.hairline_plane_y = snapshot.hairline_plane_y;
    item.hairline_plane_points[0] = snapshot.hairline_plane_points[0];
    item.hairline_plane_points[1] = snapshot.hairline_plane_points[1];
    item.hairline_plane_points[2] = snapshot.hairline_plane_points[2];
    item.hairline_spindle_scale = snapshot.hairline_spindle_scale;
    item.silhouette_shape_mode = snapshot.silhouette_shape_mode;
    item.silhouette_subdivision = snapshot.silhouette_subdivision;
    item.silhouette_edge_subdiv = snapshot.silhouette_edge_subdiv;
    item.inner_wall_radius = snapshot.inner_wall_radius;
    item.simplify_ratio = snapshot.simplify_ratio;
    item.repair_mode =
        static_cast<RenderVoxelItem::RepairMeshMode>(snapshot.repair_mode);
    item.alpha_wrap_alpha = snapshot.alpha_wrap_alpha;
    item.alpha_wrap_offset = snapshot.alpha_wrap_offset;
    item.subdivide_level = snapshot.subdivide_level;
    item.hair_strands = snapshot.hair_strands;
    item.addon_reveal = snapshot.addon_reveal;
    item.addon_split = snapshot.addon_split;
    item.addon_sdf_boolean = snapshot.addon_sdf_boolean;
    item.addon_sdf_split = snapshot.addon_sdf_split;
    item.hair_angle_config = snapshot.hair_angle_config;
    item.hair_north_pole = snapshot.hair_north_pole;
    item.hair_front_reference = snapshot.hair_front_reference;
    item.addon_base_node_id = snapshot.addon_base_node_id;
    item.drill_paths = snapshot.drill_paths;
    // Force rebuild of drill meshes after snapshot restore
    for (auto& dp : item.drill_paths)
        dp.mesh_dirty = true;
    item.show_connection_faces = snapshot.show_connection_faces;
    item.connection_faces_dirty = true;
    item.sdf_precision_cache = snapshot.sdf_precision_cache;
    item.joint_wireframe_dirty = true;

    // Validate active strand UUIDs after snapshot restore
    auto validate_strand_uuid = [&](std::string& uuid) {
        if (!uuid.empty() && !item.find_strand_by_uuid(uuid))
            uuid.clear();
    };
    validate_strand_uuid(item.active_guide_draw_strand);
    validate_strand_uuid(item.active_width_edit_strand);
    validate_strand_uuid(item.active_section_edit_strand);
    validate_strand_uuid(item.active_perpoint_section_edit_strand);
    // Validate active drill path UUID after snapshot restore
    if (!item.active_drill_path_uuid.empty() &&
        !item.find_drill_path_by_uuid(item.active_drill_path_uuid)) {
        item.active_drill_path_uuid.clear();
        item.drill_picking_active = false;
    }
    item.drill_last_picked_index = -1;
}

void RenderVoxelList::begin_edit(int item_id) {
    auto it = items.find(item_id);
    if (it == items.end())
        return;
    // collision_edit_active guard:
    //   - Normal drag: begin_edit fires on frame 1 (IsItemActivated),
    //     push_undo_now is blocked by collision_edit_active during frames 2..N,
    //     then end_edit fires on release and clears the flag.
    //   - Stale flag: if collision_edit_active is stuck true from a previous
    //     interaction that never received its deactivation event, auto-commit
    //     the stale pending_undo so no edit is silently lost, then proceed.
    if (it->second->collision_edit_active) {
        if (pending_undo.has_value() && pending_undo->item_id == item_id) {
            it->second->undo_stack.push_back(pending_undo->snapshot);
            it->second->undo_stack.back().description = "Edit";
            it->second->redo_stack.clear();
            it->second->dirty = true;
            if (it->second->undo_stack.size() > kMaxUndoSize)
                it->second->undo_stack.erase(it->second->undo_stack.begin());
        }
    }
    pending_undo.reset();
    pending_undo = PendingUndo{item_id, capture_snapshot(*it->second)};
    it->second->collision_edit_active = true;
    it->second->auto_segment_update = false;
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
        it->second->collision_edit_active = false;
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
    it->second->auto_segment_update = false;
    it->second->collision_edit_active = false;
    if (it->second->undo_stack.size() > kMaxUndoSize) {
        it->second->undo_stack.erase(it->second->undo_stack.begin());
    }
    pending_undo.reset();
}

bool RenderVoxelList::undo(int item_id) {
    auto it = items.find(item_id);
    if (it == items.end() || it->second->undo_stack.empty())
        return false;
    // push current state to redo stack, preserving the undo entry's description
    // so redo/undo filtering can track the editing context across cycles
    auto redo_snapshot = capture_snapshot(*it->second);
    redo_snapshot.description = it->second->undo_stack.back().description;
    it->second->redo_stack.push_back(std::move(redo_snapshot));
    // apply undo snapshot
    apply_snapshot(*it->second, it->second->undo_stack.back());
    it->second->undo_stack.pop_back();
    it->second->dirty = true;
    it->second->auto_segment_update = false;
    // Callers are responsible for marking only the affected strands
    // mesh_dirty, so that adding a width/guide point to one strand
    // doesn't trigger an O(n) rebuild of every strand on undo/redo.
    return true;
}

bool RenderVoxelList::redo(int item_id) {
    auto it = items.find(item_id);
    if (it == items.end() || it->second->redo_stack.empty())
        return false;
    // push current state to undo stack, preserving the redo entry's description
    auto undo_snapshot = capture_snapshot(*it->second);
    undo_snapshot.description = it->second->redo_stack.back().description;
    it->second->undo_stack.push_back(std::move(undo_snapshot));
    // apply redo snapshot
    apply_snapshot(*it->second, it->second->redo_stack.back());
    it->second->redo_stack.pop_back();
    it->second->dirty = true;
    it->second->auto_segment_update = false;
    // Callers are responsible for marking only the affected strands
    // mesh_dirty, so that adding a width/guide point to one strand
    // doesn't trigger an O(n) rebuild of every strand on undo/redo.
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
