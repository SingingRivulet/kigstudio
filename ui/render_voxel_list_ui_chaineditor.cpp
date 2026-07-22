#include "kigstudio/sdf/sdf_mesh.h"
#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <SDL.h>
#include <cstring>
#include <vector>
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
#include "kigstudio/cgal/mesh_simplification.h"
#include "kigstudio/sdf/sdf_chain_joint.h"
#include "kigstudio/utils/locale.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/voxel/voxel2mesh.h"
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
        end =
            Vec3f(picked.custom_direction_end.x, picked.custom_direction_end.y,
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
        const float radius = std::sqrt(local.x * local.x + local.y * local.y);

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
        picked.socket_cone_radius = socket_radius * 1.2f;
        changed = true;
    }
    if (head_radius > 0.0f &&
        std::abs(head_radius - picked.head_cone_radius) > 1e-4f) {
        picked.head_cone_radius = head_radius * 1.2f;
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

    // Initialize socket fillet cylinder
    const float new_fillet_radius = picked.socket_cone_radius;
    const float new_fillet_offset = 0.0f;
    const float new_fillet_height =
        std::max(0.0f, socket_base_z - picked.male_cylinder_offset) / 3.0f;
    if (std::abs(new_fillet_radius - picked.socket_fillet_radius) > 1e-4f ||
        std::abs(new_fillet_offset - picked.socket_fillet_offset) > 1e-4f ||
        std::abs(new_fillet_height - picked.socket_fillet_height) > 1e-4f) {
        picked.socket_fillet_radius = new_fillet_radius;
        picked.socket_fillet_offset = new_fillet_offset;
        picked.socket_fillet_height = new_fillet_height;
        changed = true;
    }

    // Initialize head fillet cone
    // height = distance between cone apexes + distance from male cylinder to
    // head cone apex / 3
    const float new_head_fillet_height =
        (picked.head_cone_offset - picked.socket_cone_offset) +
        (picked.male_cylinder_offset - picked.head_cone_offset) / 3.0f;
    if (std::abs(new_head_fillet_height - picked.head_fillet_height) > 1e-4f) {
        picked.head_fillet_height = new_head_fillet_height;
        changed = true;
    }

    return changed;
}

}  // namespace


void RenderVoxelList::render_object_editor_chain_mode(RenderVoxelItem& item) {
    EditResult chain_edit_result;

    // chain_min_radius
    ImGui::DragInt(get_locale_cstr("label.chain_min_radius"),
                   &item.chain_min_radius, 1, 1, 20);
    chain_edit_result.activated |= ImGui::IsItemActivated();
    chain_edit_result.deactivated_after_edit |=
        ImGui::IsItemDeactivatedAfterEdit();

    // use_cgal_skeleton
    if (!item.stl_path.empty()) {
        auto before = capture_snapshot(item);
        if (ImGui::Checkbox(get_locale_cstr("label.use_cgal_skeleton"),
                            &item.use_cgal_skeleton)) {
            push_undo_now(item.id, before,
                          get_locale_string("label.use_cgal_skeleton"));
        }
    }
    if (ImGui::Button(get_locale_cstr("action.extract_skeleton"))) {
        queue_extract_skeleton(item.id);
    }
    ImGui::Separator();
    ImGui::Text(get_locale_cstr("label.picked_skeleton_points"),
                static_cast<int>(item.picked_skeleton_points.size()));
    ImGui::SameLine();
    const std::string clear_picked_label =
        get_locale_string("action.clear_vertices") + "##PickedSkeletonPoints";
    if (ImGui::Button(clear_picked_label.c_str())) {
        push_undo_now(item.id, std::nullopt,
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
    for (size_t i = 0; i < item.picked_skeleton_points.size(); ++i) {
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
            push_undo_now(item.id, std::nullopt,
                          get_locale_string("action.delete"));
            erase_picked_skeleton_index = static_cast<int>(i);
            item.joint_wireframe_dirty = true;
        }
        ImGui::SameLine();
        ImGui::Text("#%d order=%d: %.3f, %.3f, %.3f", static_cast<int>(i),
                    picked.order, p.x, p.y, p.z);

        // ===== Joint Editor =====
        char joint_label[64];
        snprintf(joint_label, sizeof(joint_label), "%s #%d",
                 get_locale_cstr("label.joint"), static_cast<int>(i));
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
            if (ImGui::Checkbox(get_locale_cstr("label.custom_direction"),
                                &picked.use_custom_direction)) {
                dirty = true;
            }
            chain_edit_result.activated |= ImGui::IsItemActivated();
            chain_edit_result.deactivated_after_edit |=
                ImGui::IsItemDeactivatedAfterEdit();
            if (picked.use_custom_direction) {
                auto r = edit_local_position_stepper(
                    get_locale_cstr("label.direction_end"),
                    picked.custom_direction_end, 0.1f, false, false);
                chain_edit_result.activated |= r.activated;
                chain_edit_result.deactivated_after_edit |=
                    r.deactivated_after_edit;
                if (r.value_changed)
                    dirty = true;

                if (pick_direction_index == (int)i) {
                    if (ImGui::Button(get_locale_cstr("action.stop_picking_"
                                                      "direction"))) {
                        pick_direction_index = -1;
                    }
                } else {
                    if (ImGui::Button(
                            get_locale_cstr("action.pick_direction"))) {
                        pick_direction_index = (int)i;
                    }
                }
            }

            // Socket cone
            if (ImGui::CollapsingHeader(get_locale_cstr("label.socket_cone"))) {
                ImGui::PushID("SocketCone");
                if (ImGui::DragFloat(get_locale_cstr("label.offset"),
                                     &picked.socket_cone_offset, 0.1f, 0.0f,
                                     100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.angle"),
                                     &picked.socket_cone_angle, 0.01f, 0.01f,
                                     1.5f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.radius"),
                                     &picked.socket_cone_radius, 0.1f, 0.1f,
                                     50.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.socket_fillet_radius"),
                                     &picked.socket_fillet_radius, 0.1f, 0.0f,
                                     50.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.socket_fillet_height"),
                                     &picked.socket_fillet_height, 0.1f, 0.0f,
                                     100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.socket_fillet_offset"),
                                     &picked.socket_fillet_offset, 0.1f, 0.0f,
                                     100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                ImGui::PopID();
            }

            // Head cone
            if (ImGui::CollapsingHeader(get_locale_cstr("label.head_cone"))) {
                ImGui::PushID("HeadCone");
                if (ImGui::DragFloat(get_locale_cstr("label.offset"),
                                     &picked.head_cone_offset, 0.1f, 0.0f,
                                     100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.radius"),
                                     &picked.head_cone_radius, 0.1f, 0.1f,
                                     50.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.head_fillet_height"),
                                     &picked.head_fillet_height, 0.1f, 0.0f,
                                     100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                ImGui::PopID();
            }

            // Support cones
            if (ImGui::CollapsingHeader(
                    get_locale_cstr("label.support_cones"))) {
                if (ImGui::DragFloat(
                        get_locale_cstr("label.socket_support_offset"),
                        &picked.socket_support_offset, 0.1f, 0.0f, 100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(
                        get_locale_cstr("label.socket_support_radius"),
                        &picked.socket_support_radius, 0.1f, 0.1f, 50.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(
                        get_locale_cstr("label.head_support_offset"),
                        &picked.head_support_offset, 0.1f, 0.0f, 100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(
                        get_locale_cstr("label.head_support_radius"),
                        &picked.head_support_radius, 0.1f, 0.1f, 50.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
            }

            // Cylinder
            if (ImGui::CollapsingHeader(get_locale_cstr("label.cylinder"))) {
                if (ImGui::DragFloat(get_locale_cstr("label.cylinder_offset"),
                                     &picked.male_cylinder_offset, 0.1f, 0.0f,
                                     100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.cylinder_radius"),
                                     &picked.male_cylinder_radius, 0.1f, 0.1f,
                                     50.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.female_gap"),
                                     &picked.female_gap, 0.01f, 0.0f, 10.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
            }

            // Slot
            if (ImGui::DragFloat(get_locale_cstr("label.slot_extra"),
                                 &picked.slot_extra, 0.1f, 0.0f, 10.0f))
                dirty = true;
            chain_edit_result.activated |= ImGui::IsItemActivated();
            chain_edit_result.deactivated_after_edit |=
                ImGui::IsItemDeactivatedAfterEdit();

            // Rotation
            if (ImGui::DragFloat(get_locale_cstr("label.rotation_angle"),
                                 &picked.rotation_angle, 0.01f, -3.14f, 3.14f))
                dirty = true;
            chain_edit_result.activated |= ImGui::IsItemActivated();
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
        pick_direction_index < (int)item.picked_skeleton_points.size() &&
        mouse_world_pos_valid && mouse_world_pos_picked) {
        auto& picked = item.picked_skeleton_points[pick_direction_index];
        push_undo_now(item.id, std::nullopt,
                      get_locale_string("label.pick_point_by_mouse"));
        picked.custom_direction_end = mouse_world_pos;
        item.joint_wireframe_dirty = true;
        pick_direction_index = -1;
    }
    if (erase_picked_skeleton_index >= 0) {
        item.picked_skeleton_points.erase(item.picked_skeleton_points.begin() +
                                          erase_picked_skeleton_index);
        item.sort_picked_skeleton_points();
        item.joint_wireframe_dirty = true;
    }
    if (mouse_world_pos_picked) {
        push_undo_now(item.id, std::nullopt,
                      get_locale_string("label.pick_point_by_mouse"));
        pick_skeleton_point_from_mouse();
        item.joint_wireframe_dirty = true;
    }

    if (chain_edit_result.activated)
        begin_edit(item.id);
    if (chain_edit_result.deactivated_after_edit)
        end_edit(item.id, get_locale_string("label.joint"));
}

}  // namespace sinriv::ui::render
