#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>
#include <stb/stb_truetype.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <unordered_set>
#include <variant>
#ifdef _WIN32
#include <windows.h>
#endif
#include "kigstudio/sdf/sdf_chain_joint.h"
#include "kigstudio/cgal/mesh_simplification.h"
#include "kigstudio/utils/vec3.h"
#include "locale.h"
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {
namespace {

bool autoDetectJointRadius(RenderVoxelList::RenderVoxelItem& item,
                           size_t joint_index) {
    if (joint_index >= item.picked_skeleton_points.size()) {
        return false;
    }

    using Vec3f = sinriv::kigstudio::sdf::joint::Vec3f;
    using Frame = sinriv::kigstudio::sdf::joint::Frame;

    auto& picked = item.picked_skeleton_points[joint_index];

    auto get_pos = [&](size_t idx) -> Vec3f {
        const auto& pos = item.picked_skeleton_points[idx].position;
        return {pos.x, pos.y, pos.z};
    };

    Vec3f start = get_pos(joint_index);
    Vec3f end;
    if (picked.use_custom_direction) {
        end = Vec3f(picked.custom_direction_end.x,
                    picked.custom_direction_end.y,
                    picked.custom_direction_end.z);
    } else {
        if (joint_index + 1 < item.picked_skeleton_points.size()) {
            end = get_pos(joint_index + 1);
        } else if (item.picked_skeleton_points.size() >= 2) {
            Vec3f prev = get_pos(joint_index - 1);
            end = start + (start - prev);
        } else {
            end = start + Vec3f(0, 0, 10);
        }
    }
    if ((end - start).length() < 1e-6f) {
        end = start + Vec3f(0, 0, 1);
    }

    Frame frame;
    if (picked.use_custom_direction) {
        frame = sinriv::kigstudio::sdf::joint::buildFrame(
            start, end, picked.rotation_angle);
    } else {
        frame = sinriv::kigstudio::sdf::joint::buildFrameAlignedY(start, end);
    }

    const auto& voxel_size = item.voxel_grid_data.voxel_size;
    const float half_thickness =
        std::max({voxel_size.x, voxel_size.y, voxel_size.z}) * 0.75f;
    float socket_radius = -1.0f;
    float head_radius = -1.0f;

    for (const auto& voxel : item.voxel_grid_data) {
        const auto world = item.voxel_grid_data.voxelCenterToWorld(voxel);
        const Vec3f local = frame.worldToLocal({world.x, world.y, world.z});
        const float radius =
            std::sqrt(local.x * local.x + local.y * local.y);

        if (std::abs(local.z - picked.socket_cone_offset) <= half_thickness) {
            socket_radius = std::max(socket_radius, radius);
        }
        if (std::abs(local.z - picked.head_cone_offset) <= half_thickness) {
            head_radius = std::max(head_radius, radius);
        }
    }

    bool changed = false;
    if (socket_radius > 0.0f &&
        std::abs(socket_radius - picked.socket_cone_radius) > 1e-4f) {
        picked.socket_cone_radius = socket_radius*1.2f;
        changed = true;
    }
    if (head_radius > 0.0f &&
        std::abs(head_radius - picked.head_cone_radius) > 1e-4f) {
        picked.head_cone_radius = head_radius*1.2f;
        changed = true;
    }

    const float socket_base_z =
        picked.socket_cone_offset +
        picked.socket_cone_radius / std::tan(picked.socket_cone_angle);
    const float cylinder_offset =
        (picked.head_cone_offset + socket_base_z) * 0.5f;
    if (std::abs(cylinder_offset - picked.male_cylinder_offset) > 1e-4f) {
        picked.male_cylinder_offset = cylinder_offset;
        changed = true;
    }

    return changed;
}

}  // namespace

void RenderVoxelList::render_object_editor() {
    ImGui::SetNextWindowPos(ImVec2((float)window_width, (float)menu_height),
                            ImGuiCond_Always, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(360, 620), ImGuiCond_Once);
    if (ImGui::Begin(get_locale_cstr("window.object_editor"))) {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it == items.end()) {
            ImGui::TextUnformatted(get_locale_cstr("label.no_active_item"));
        } else {
            RenderVoxelItem& item = *item_it->second;
            bool is_updating = item.write_count != 0;
            if (is_updating) {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "%s",
                                   get_locale_cstr("label.updating"));
            }
            if (is_updating)
                ImGui::BeginDisabled();

            render_object_editor_toolbar(item);

            ImGui::Text(get_locale_cstr("label.render_item"), item.id);
            ImGui::Separator();

            if (ImGui::BeginTabBar("ObjectEditorTabs", ImGuiTabBarFlags_None)) {
                ImGuiTabItemFlags flags_collision = 0;
                ImGuiTabItemFlags flags_voxel = 0;
                if (last_object_editor_tab != object_editor_tab) {
                    if (object_editor_tab == 0)
                        flags_collision = ImGuiTabItemFlags_SetSelected;
                    else
                        flags_voxel = ImGuiTabItemFlags_SetSelected;
                    last_object_editor_tab = object_editor_tab;
                }

                // ===== Tab: Collision Edit =====
                if (ImGui::BeginTabItem(get_locale_cstr("tab.collision_edit"),
                                        nullptr, flags_collision)) {
                    object_editor_tab = 0;
                    render_object_editor_collision_tab_content(item);
                    ImGui::EndTabItem();
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip(
                        get_locale_cstr("tooltip.collision_edit"));
                }

                // ===== Tab: Voxel Picking =====
                if (ImGui::BeginTabItem(get_locale_cstr("tab.voxel_picking"),
                                        nullptr, flags_voxel)) {
                    object_editor_tab = 1;
                    render_object_editor_voxel_tab_content(item);
                    ImGui::EndTabItem();
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip(get_locale_cstr("tooltip.voxel_picking"));
                }
                ImGui::EndTabBar();
            }

            if (is_updating)
                ImGui::EndDisabled();
        }
    }
    ImGui::End();
}

void RenderVoxelList::render_object_editor_toolbar(RenderVoxelItem& item) {
    if (ImGui::Button(get_locale_cstr("action.delete"))) {
        pending_delete_item_id = item.id;
        show_delete_confirm = true;
    }
    ImGui::SameLine();
    if (!item.stl_path.empty()) {
        if (ImGui::Button(get_locale_cstr("action.reload_stl"))) {
            show_reload_stl_dialog = true;
            reload_stl_item_id = item.id;
            reload_stl_voxel_size = item.stl_voxel_size;
        }
        ImGui::SameLine();
    }
    const std::string export_popup_title =
        localize_id("dialog.choose_export_method", item.id);
    if (ImGui::Button(get_locale_cstr("action.save_as_stl"))) {
        ImGui::OpenPopup(export_popup_title.c_str());
    }
    if (ImGui::BeginPopupModal(export_popup_title.c_str(), nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted(
            get_locale_cstr("dialog.choose_export_method"));

        // Export mode selection
        ImGui::RadioButton(get_locale_cstr("label.export_mode_standard"),
                           &export_stl_mode, 0);
        ImGui::RadioButton(get_locale_cstr("label.export_mode_smooth"),
                           &export_stl_mode, 1);

        ImGui::Separator();

        // Simplification option
        ImGui::Checkbox(get_locale_cstr("label.simplify_model"),
                        &export_stl_simplify);
        if (export_stl_simplify) {
            ImGui::Indent();
            ImGui::SliderFloat(
                get_locale_cstr("label.simplification_ratio"),
                &export_stl_simplify_ratio, 0.01f, 1.0f,
                "%.2f");
            ImGui::TextUnformatted(
                get_locale_cstr("hint.simplification_ratio"));
            ImGui::Unindent();
        }
        
        if (export_stl_mode == 1) {
            ImGui::SliderInt(
                get_locale_cstr("label.subdivisions_ratio"),
                &export_stl_subdivisions, 1, 8);
        }

        ImGui::Separator();

        if (ImGui::Button(get_locale_cstr("action.save_as_stl"))) {
            ImGui::CloseCurrentPopup();
            const char* filters[] = {"*.stl"};
            const char* file = tinyfd_saveFileDialog(
                get_locale_cstr("dialog.save_voxel_as_stl"),
                "node_voxel.stl", 1, filters,
                get_locale_cstr("dialog.stl_files"));
            if (file) {
                queue_export_stl(
                    item.id, tinyfd_path_to_utf8(file),
                    export_stl_mode, export_stl_simplify,
                    export_stl_simplify_ratio,
                    export_stl_subdivisions);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.cancel"))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    ImGui::SameLine();
    if (mouse_world_pos_picked_auto_snapping) {
        if (ImGui::Button(get_locale_cstr(
                "action.pick_pos_auto_snapping_stop"))) {
            mouse_world_pos_picked_auto_snapping = false;
        }
    } else {
        if (ImGui::Button(
                get_locale_cstr("action.pick_pos_auto_snapping"))) {
            mouse_world_pos_picked_auto_snapping = true;
        }
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            get_locale_cstr("tooltip.pick_pos_auto_snapping"));
    }
}

void RenderVoxelList::render_object_editor_collision_tab_content(
    RenderVoxelItem& item) {
    // Undo / Redo buttons (collision)
    bool undo_disabled = !can_undo(item.id);
    bool redo_disabled = !can_redo(item.id);
    if (undo_disabled)
        ImGui::BeginDisabled();
    if (ImGui::Button(get_locale_cstr("action.undo"))) {
        undo(item.id);
    }
    if (undo_disabled)
        ImGui::EndDisabled();
    ImGui::SameLine();
    if (redo_disabled)
        ImGui::BeginDisabled();
    if (ImGui::Button(get_locale_cstr("action.redo"))) {
        redo(item.id);
    }
    if (redo_disabled)
        ImGui::EndDisabled();
    ImGui::Separator();

    ImGui::Checkbox(
        get_locale_cstr("label.auto_segment_update"),
        &item.auto_segment_update);

    const char* segment_mode_names[] = {
        get_locale_cstr("mode.collision"),
        get_locale_cstr("mode.plane"),
        get_locale_cstr("mode.concave_cone"),
        get_locale_cstr("mode.split_disconnected"),
        get_locale_cstr("mode.neighbor"),
        get_locale_cstr("mode.fill_interior"),
        get_locale_cstr("mode.chain")};
    const enum RenderVoxelItem::SegmentMode segment_modes[] = {
        RenderVoxelItem::COLLISION,
        RenderVoxelItem::PLANE,
        RenderVoxelItem::CONCAVE_CONE,
        RenderVoxelItem::SPLIT_DISCONNECTED,
        RenderVoxelItem::NEIGHBOR,
        RenderVoxelItem::FILL_INTERIOR,
        RenderVoxelItem::CHAIN};
    int current_segment_mode =
        segment_modes[(int)item.segment_mode];
    if (ImGui::Combo(get_locale_cstr("label.segment_mode"),
                     &current_segment_mode, segment_mode_names,
                     IM_ARRAYSIZE(segment_mode_names))) {
        push_undo_now(item.id, std::nullopt,
                      get_locale_string("label.segment_mode"));
        item.segment_mode = segment_modes[current_segment_mode];
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        const char* tooltip_key = nullptr;
        switch (item.segment_mode) {
            case RenderVoxelItem::COLLISION:
                tooltip_key = "tooltip.mode.collision";
                break;
            case RenderVoxelItem::PLANE:
                tooltip_key = "tooltip.mode.plane";
                break;
            case RenderVoxelItem::CONCAVE_CONE:
                tooltip_key = "tooltip.mode.concave_cone";
                break;
            case RenderVoxelItem::SPLIT_DISCONNECTED:
                tooltip_key = "tooltip.mode.split_disconnected";
                break;
            case RenderVoxelItem::NEIGHBOR:
                tooltip_key = "tooltip.mode.neighbor";
                break;
            case RenderVoxelItem::FILL_INTERIOR:
                tooltip_key = "tooltip.mode.fill_interior";
                break;
            case RenderVoxelItem::CHAIN:
                tooltip_key = "tooltip.mode.chain";
                break;
        }
        if (tooltip_key) {
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(ImGui::GetFontSize() *
                                   35.0f);
            ImGui::TextUnformatted(
                get_locale_cstr(tooltip_key));
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }
    }

    if (item.segment_mode == RenderVoxelItem::PLANE) {
        render_plane_editor(item);
    } else if (item.segment_mode ==
               RenderVoxelItem::COLLISION) {
        render_collision_body_editor(item);
    } else if (item.segment_mode ==
               RenderVoxelItem::CONCAVE_CONE) {
        render_concave_cone_editor(item);
    } else if (item.segment_mode == RenderVoxelItem::NEIGHBOR) {
        ImGui::DragInt(
            get_locale_cstr("label.neighbor_max_distance"),
            &item.neighbor_max_distance, 1, 1, 100);
    } else if (item.segment_mode ==
               RenderVoxelItem::FILL_INTERIOR) {
        ImGui::TextUnformatted(
            get_locale_cstr("tooltip.mode.fill_interior"));
    } else if (item.segment_mode == RenderVoxelItem::CHAIN) {
        render_object_editor_chain_mode(item);
    }
}

void RenderVoxelList::render_object_editor_chain_mode(
    RenderVoxelItem& item) {
    EditResult chain_edit_result;

    // chain_min_radius
    ImGui::DragInt(
        get_locale_cstr("label.chain_min_radius"),
        &item.chain_min_radius, 1, 1, 20);
    chain_edit_result.activated |= ImGui::IsItemActivated();
    chain_edit_result.deactivated_after_edit |=
        ImGui::IsItemDeactivatedAfterEdit();

    // use_cgal_skeleton
    if (!item.stl_path.empty()) {
        auto before = capture_snapshot(item);
        if (ImGui::Checkbox(
                get_locale_cstr("label.use_cgal_skeleton"),
                &item.use_cgal_skeleton)) {
            push_undo_now(item.id, before,
                          get_locale_string(
                              "label.use_cgal_skeleton"));
        }
    }
    if (ImGui::Button(
            get_locale_cstr("action.extract_skeleton"))) {
        queue_extract_skeleton(item.id);
    }
    ImGui::Separator();
    ImGui::Text(get_locale_cstr("label.picked_skeleton_points"),
                static_cast<int>(item.picked_skeleton_points.size()));
    ImGui::SameLine();
    const std::string clear_picked_label =
        get_locale_string("action.clear_vertices") + "##PickedSkeletonPoints";
    if (ImGui::Button(clear_picked_label.c_str())) {
        push_undo_now(
            item.id, std::nullopt,
            get_locale_string("action.clear_vertices"));
        item.picked_skeleton_points.clear();
        item.joint_wireframe_dirty = true;
    }
    ImGui::SameLine();
    const std::string init_radii_label =
        get_locale_string("action.init_all_joint_radii") +
        "##InitAllJointRadii";
    if (ImGui::Button(init_radii_label.c_str())) {
        const auto before = capture_snapshot(item);
        bool changed = false;
        for (size_t idx = 0; idx < item.picked_skeleton_points.size(); ++idx) {
            changed |= autoDetectJointRadius(item, idx);
        }
        if (changed) {
            push_undo_now(item.id, before,
                          get_locale_string("action.init_all_joint_radii"));
            item.joint_wireframe_dirty = true;
        }
    }
    int erase_picked_skeleton_index = -1;
    bool moved_picked_skeleton_point = false;
    static int pick_direction_index = -1;
    for (size_t i = 0;
         i < item.picked_skeleton_points.size(); ++i) {
        auto& picked = item.picked_skeleton_points[i];
        const auto& p = picked.position;
        ImGui::PushID(static_cast<int>(i));
        if (ImGui::Button("<")) {
            push_undo_now(item.id, std::nullopt,
                          get_locale_string("action.move"));
            item.move_picked_skeleton_point(i, -1);
            moved_picked_skeleton_point = true;
        }
        ImGui::SameLine();
        if (ImGui::Button(">")) {
            push_undo_now(item.id, std::nullopt,
                          get_locale_string("action.move"));
            item.move_picked_skeleton_point(i, 1);
            moved_picked_skeleton_point = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("X")) {
            push_undo_now(
                item.id, std::nullopt,
                get_locale_string("action.delete"));
            erase_picked_skeleton_index =
                static_cast<int>(i);
            item.joint_wireframe_dirty = true;
        }
        ImGui::SameLine();
        ImGui::Text("#%d order=%d: %.3f, %.3f, %.3f",
                    static_cast<int>(i), picked.order, p.x,
                    p.y, p.z);

        // ===== Joint Editor =====
        char joint_label[64];
        snprintf(joint_label, sizeof(joint_label), "%s #%d",
                 get_locale_cstr("label.joint"),
                 static_cast<int>(i));
        if (ImGui::CollapsingHeader(joint_label)) {
            bool dirty = false;

            const std::string auto_detect_label =
                get_locale_string("action.auto_detect_joint_radius") +
                "##AutoDetectJointRadius";
            if (ImGui::Button(auto_detect_label.c_str())) {
                const auto before = capture_snapshot(item);
                if (autoDetectJointRadius(item, i)) {
                    push_undo_now(
                        item.id, before,
                        get_locale_string("action.auto_detect_joint_radius"));
                    dirty = true;
                }
            }

            // Custom direction
            if (ImGui::Checkbox(
                    get_locale_cstr(
                        "label.custom_direction"),
                    &picked.use_custom_direction)) {
                dirty = true;
            }
            chain_edit_result.activated |=
                ImGui::IsItemActivated();
            chain_edit_result.deactivated_after_edit |=
                ImGui::IsItemDeactivatedAfterEdit();
            if (picked.use_custom_direction) {
                auto r = edit_local_position_stepper(
                    get_locale_cstr("label.direction_end"),
                    picked.custom_direction_end, 0.1f,
                    false, false);
                chain_edit_result.activated |= r.activated;
                chain_edit_result.deactivated_after_edit |=
                    r.deactivated_after_edit;
                if (r.value_changed)
                    dirty = true;

                if (pick_direction_index == (int)i) {
                    if (ImGui::Button(get_locale_cstr(
                            "action.stop_picking_"
                            "direction"))) {
                        pick_direction_index = -1;
                    }
                } else {
                    if (ImGui::Button(get_locale_cstr(
                            "action.pick_direction"))) {
                        pick_direction_index = (int)i;
                    }
                }
            }

            // Socket cone
            if (ImGui::CollapsingHeader(
                    get_locale_cstr("label.socket_cone"))) {
                ImGui::PushID("SocketCone");
                if (ImGui::DragFloat(
                        get_locale_cstr("label.offset"),
                        &picked.socket_cone_offset, 0.1f,
                        0.0f, 100.0f))
                    dirty = true;
                chain_edit_result.activated |=
                    ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(
                        get_locale_cstr("label.angle"),
                        &picked.socket_cone_angle, 0.01f,
                        0.01f, 1.5f))
                    dirty = true;
                chain_edit_result.activated |=
                    ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(
                        get_locale_cstr("label.radius"),
                        &picked.socket_cone_radius, 0.1f,
                        0.1f, 50.0f))
                    dirty = true;
                chain_edit_result.activated |=
                    ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                ImGui::PopID();
            }

            // Head cone
            if (ImGui::CollapsingHeader(
                    get_locale_cstr("label.head_cone"))) {
                ImGui::PushID("HeadCone");
                if (ImGui::DragFloat(
                        get_locale_cstr("label.offset"),
                        &picked.head_cone_offset, 0.1f,
                        0.0f, 100.0f))
                    dirty = true;
                chain_edit_result.activated |=
                    ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(
                        get_locale_cstr("label.radius"),
                        &picked.head_cone_radius, 0.1f,
                        0.1f, 50.0f))
                    dirty = true;
                chain_edit_result.activated |=
                    ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                ImGui::PopID();
            }

            // Support cones
            if (ImGui::CollapsingHeader(get_locale_cstr(
                    "label.support_cones"))) {
                if (ImGui::DragFloat(
                        get_locale_cstr(
                            "label.socket_support_offset"),
                        &picked.socket_support_offset, 0.1f,
                        0.0f, 100.0f))
                    dirty = true;
                chain_edit_result.activated |=
                    ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(
                        get_locale_cstr(
                            "label.socket_support_radius"),
                        &picked.socket_support_radius, 0.1f,
                        0.1f, 50.0f))
                    dirty = true;
                chain_edit_result.activated |=
                    ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(
                        get_locale_cstr(
                            "label.head_support_offset"),
                        &picked.head_support_offset, 0.1f,
                        0.0f, 100.0f))
                    dirty = true;
                chain_edit_result.activated |=
                    ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(
                        get_locale_cstr(
                            "label.head_support_radius"),
                        &picked.head_support_radius, 0.1f,
                        0.1f, 50.0f))
                    dirty = true;
                chain_edit_result.activated |=
                    ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
            }

            // Cylinder
            if (ImGui::CollapsingHeader(
                    get_locale_cstr("label.cylinder"))) {
                if (ImGui::DragFloat(
                        get_locale_cstr(
                            "label.cylinder_offset"),
                        &picked.male_cylinder_offset, 0.1f,
                        0.0f, 100.0f))
                    dirty = true;
                chain_edit_result.activated |=
                    ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(
                        get_locale_cstr(
                            "label.cylinder_radius"),
                        &picked.male_cylinder_radius, 0.1f,
                        0.1f, 50.0f))
                    dirty = true;
                chain_edit_result.activated |=
                    ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(
                        get_locale_cstr("label.female_gap"),
                        &picked.female_gap, 0.01f, 0.0f,
                        10.0f))
                    dirty = true;
                chain_edit_result.activated |=
                    ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
            }

            // Slot
            if (ImGui::DragFloat(
                    get_locale_cstr("label.slot_extra"),
                    &picked.slot_extra, 0.1f, 0.0f, 10.0f))
                dirty = true;
            chain_edit_result.activated |=
                ImGui::IsItemActivated();
            chain_edit_result.deactivated_after_edit |=
                ImGui::IsItemDeactivatedAfterEdit();

            // Rotation
            if (ImGui::DragFloat(
                    get_locale_cstr("label.rotation_angle"),
                    &picked.rotation_angle, 0.01f, -3.14f,
                    3.14f))
                dirty = true;
            chain_edit_result.activated |=
                ImGui::IsItemActivated();
            chain_edit_result.deactivated_after_edit |=
                ImGui::IsItemDeactivatedAfterEdit();

            if (dirty) {
                item.joint_wireframe_dirty = true;
            }
        }

        ImGui::PopID();
        if (moved_picked_skeleton_point) {
            item.joint_wireframe_dirty = true;
            break;
        }
    }

    // Handle direction picking
    if (pick_direction_index >= 0 &&
        pick_direction_index <
            (int)item.picked_skeleton_points.size() &&
        mouse_world_pos_valid && mouse_world_pos_picked) {
        auto& picked = item.picked_skeleton_points
                           [pick_direction_index];
        push_undo_now(
            item.id, std::nullopt,
            get_locale_string("label.pick_point_by_mouse"));
        picked.custom_direction_end = mouse_world_pos;
        item.joint_wireframe_dirty = true;
        pick_direction_index = -1;
    }
    if (erase_picked_skeleton_index >= 0) {
        item.picked_skeleton_points.erase(
            item.picked_skeleton_points.begin() +
            erase_picked_skeleton_index);
        item.sort_picked_skeleton_points();
        item.joint_wireframe_dirty = true;
    }
    if (mouse_world_pos_picked) {
        push_undo_now(
            item.id, std::nullopt,
            get_locale_string("label.pick_point_by_mouse"));
        pick_skeleton_point_from_mouse();
        item.joint_wireframe_dirty = true;
    }

    if (chain_edit_result.activated)
        begin_edit(item.id);
    if (chain_edit_result.deactivated_after_edit)
        end_edit(item.id, get_locale_string("label.joint"));
}

void RenderVoxelList::render_object_editor_voxel_tab_content(
    RenderVoxelItem& item) {
    ImGui::Checkbox(get_locale_cstr("label.voxel_picking"),
                    &item.voxel_picking_enabled);
    if (item.voxel_picking_enabled) {
        if (!item.surface_cache_ready &&
            !item.surface_cache_computing) {
            item.surface_cache_computing = true;
            item.surface_cache_progress = 0.0f;
            // 在后台线程初始化表面缓存
            std::thread([this, id = item.id]() {
                auto it = this->items.find(id);
                if (it == this->items.end())
                    return;
                auto& target = *it->second;
                auto surface =
                    target.voxel_grid_data.getSurfaceVoxels();
                target.surface_voxels = std::move(surface);
                target.surface_cache_ready = true;
                target.surface_cache_computing = false;
            }).detach();
        }
        if (item.surface_cache_computing) {
            ImGui::Text("Surface cache: %.0f%%",
                        item.surface_cache_progress * 100.0f);
        } else if (item.surface_cache_ready) {
            ImGui::TextUnformatted(
                get_locale_cstr("label.surface_cache_ready"));
            ImGui::DragFloat(
                get_locale_cstr("label.pick_range"),
                &item.voxel_pick_range, 0.1f, 0.1f, 20.0f,
                "%.1f");
        }
        ImGui::Separator();

        // Undo / Redo buttons (marked voxels)
        bool marked_undo_disabled = !can_undo_marked(item.id);
        bool marked_redo_disabled = !can_redo_marked(item.id);
        if (marked_undo_disabled)
            ImGui::BeginDisabled();
        if (ImGui::Button(get_locale_cstr("action.undo"))) {
            this->undo_marked(item.id);
        }
        if (marked_undo_disabled)
            ImGui::EndDisabled();
        ImGui::SameLine();
        if (marked_redo_disabled)
            ImGui::BeginDisabled();
        if (ImGui::Button(get_locale_cstr("action.redo"))) {
            this->redo_marked(item.id);
        }
        if (marked_redo_disabled)
            ImGui::EndDisabled();
        ImGui::Separator();

        if (ImGui::Button(
                get_locale_cstr("action.save_marked_voxels"))) {
            const char* filters[] = {"*.vxgrid"};
            const char* file = tinyfd_saveFileDialog(
                get_locale_cstr("dialog.save_marked_voxels"),
                "marked.vxgrid", 1, filters,
                get_locale_cstr("dialog.marked_voxels_file"));
            if (file) {
                std::string error;
                if (!sinriv::kigstudio::save(
                        file, item.marked_voxels, &error)) {
                    tinyfd_messageBox(
                        "Error",
                        utf8_to_ansi(error.c_str()).c_str(),
                        "ok", "error", 1);
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button(
                get_locale_cstr("action.load_marked_voxels"))) {
            const char* filters[] = {"*.vxgrid"};
            const char* file = tinyfd_openFileDialog(
                get_locale_cstr("dialog.load_marked_voxels"),
                "", 1, filters,
                get_locale_cstr("dialog.marked_voxels_file"),
                0);
            if (file) {
                this->push_marked_undo_now(item.id,
                                           "Load marked");
                if (!sinriv::kigstudio::load(
                        file, item.marked_voxels)) {
                    tinyfd_messageBox(
                        "Error",
                        utf8_to_ansi(
                            get_locale_cstr(
                                "error.load_marked_failed"))
                            .c_str(),
                        "ok", "error", 1);
                } else {
                    item.marked_voxels.global_position =
                        item.voxel_grid_data.global_position;
                    item.marked_voxels.voxel_size =
                        item.voxel_grid_data.voxel_size;
                    item.marked_voxels_dirty = true;
                }
            }
        }
    }
    ImGui::Checkbox(
        get_locale_cstr("label.disable_camera_on_pick"),
        &disable_camera_on_pick);
    ImGui::DragFloat(
        get_locale_cstr("label.mouse_highlight_range"),
        &mouse_highlight_range, 0.1f, 0.0f, 20.0f, "%.1f");
}

}  // namespace sinriv::ui::render
