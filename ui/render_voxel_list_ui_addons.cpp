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
// Use stb_image from bimg 3rdparty for overlay image loading
#define STB_IMAGE_IMPLEMENTATION
#include "../../dep/bgfx.cmake/bimg/3rdparty/stb/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../../dep/bgfx.cmake/bimg/3rdparty/stb/stb_image_write.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <set>
#include <sys/stat.h>
#include <unordered_set>
#include <variant>
#ifdef _WIN32
#include <windows.h>
#endif
#include "kigstudio/cgal/mesh_simplification.h"
#include "kigstudio/sdf/sdf_chain_joint.h"
#include "kigstudio/utils/locale.h"
#include "kigstudio/utils/triangle.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/voxel/voxel2mesh.h"
#include "kigstudio/agent/agent_handlers.h"
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {

void RenderVoxelList::render_guide_curve_window() {
    if (!show_addon_window)
        return;
    if (!show_guide_curve_window)
        return;

    // 互斥：打开引导曲线窗口时关闭宽度编辑器
    if (show_width_editor_window) {
        auto wit = items.find(render_id);
        if (wit != items.end()) {
            wit->second->width_editing_active = false;
            wit->second->active_width_edit_strand.clear();
        }
        show_width_editor_window = false;
    }
    // 互斥：打开引导曲线窗口时关闭截面编辑器
    if (show_cross_section_editor_window) {
        auto sit = items.find(render_id);
        if (sit != items.end()) {
            sit->second->active_section_edit_strand.clear();
        }
        show_cross_section_editor_window = false;
    }
    // 互斥：打开引导曲线窗口时关闭逐点截面编辑器
    if (show_perpoint_section_editor_window) {
        auto pit = items.find(render_id);
        if (pit != items.end()) {
            pit->second->perpoint_section_editing_active = false;
            pit->second->active_perpoint_section_edit_strand.clear();
            pit->second->active_perpoint_section_edit_width_idx = -1;
        }
        show_perpoint_section_editor_window = false;
    }
    // 互斥：打开引导曲线窗口时关闭自动宽度窗口
    if (show_hairline_plane_window) {
        auto hit = items.find(render_id);
        if (hit != items.end()) {
            hit->second->hairline_point_picking_active = false;
        }
        show_hairline_plane_window = false;
    }

    ImGui::SetNextWindowSize(ImVec2(520, 400), ImGuiCond_Once);
    bool window_open = true;
    if (!ImGui::Begin(get_locale_cstr("window.guide_curve"), &window_open)) {
        ImGui::End();
        return;
    }

    // 点击 X 关闭按钮
    if (!window_open) {
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->guide_curve_drawing_active = false;
            it->second->active_guide_draw_strand.clear();
        }
        show_guide_curve_window = false;
        ImGui::End();
        return;
    }

    std::lock_guard<std::mutex> lock(locker);
    auto item_it = items.find(render_id);
    if (item_it == items.end() || item_it->second->source_type != 2) {
        ImGui::TextUnformatted(get_locale_cstr("label.no_active_item"));
        ImGui::End();
        return;
    }

    RenderVoxelItem& item = *item_it->second;
    std::string strand_uuid = item.active_guide_draw_strand;

    if (strand_uuid.empty() || !item.find_strand_by_uuid(strand_uuid) ||
        !item.guide_curve_drawing_active) {
        show_guide_curve_window = false;
        ImGui::End();
        return;
    }

    auto& strand = *item.find_strand_by_uuid(strand_uuid);
    ImGui::Text("%s", strand.name.c_str());
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.08f, 1.0f), "%s",
                       get_locale_cstr("action.draw_guide_curve"));

    // Undo / Redo buttons
    {
        bool undo_disabled = !can_undo(item.id);
        bool redo_disabled = !can_redo(item.id);
        if (undo_disabled)
            ImGui::BeginDisabled();
        if (ImGui::SmallButton(get_locale_cstr("action.undo"))) {
            undo(item.id);
            for (auto& s : item.hair_strands) s.mesh_dirty = true;
        }
        if (undo_disabled)
            ImGui::EndDisabled();
        ImGui::SameLine();
        if (redo_disabled)
            ImGui::BeginDisabled();
        if (ImGui::SmallButton(get_locale_cstr("action.redo"))) {
            redo(item.id);
            for (auto& s : item.hair_strands) s.mesh_dirty = true;
        }
        if (redo_disabled)
            ImGui::EndDisabled();

        // Reverse guide points order button
        {
            bool reverse_disabled = strand.guide_points.size() < 2;
            if (reverse_disabled)
                ImGui::BeginDisabled();
            ImGui::SameLine();
            if (ImGui::SmallButton(get_locale_cstr("action.reverse_guide_points"))) {
                push_undo_now(item.id, std::nullopt, "Reverse Guide Points");
                std::reverse(strand.guide_points.begin(), strand.guide_points.end());
                strand.mesh_dirty = true;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", get_locale_cstr("tooltip.reverse_guide_points"));
            if (reverse_disabled)
                ImGui::EndDisabled();
        }
    }

    // Keyboard shortcuts (Ctrl+Z / Ctrl+Y) are handled globally
    // by the SDL event loop in ui.hpp to avoid double-firing.

    ImGui::Separator();

    // ---- Auto hair root checkbox (only when north_pole is configured) ----
    {
        float np_len =
            std::sqrt(item.hair_north_pole.x * item.hair_north_pole.x +
                      item.hair_north_pole.y * item.hair_north_pole.y +
                      item.hair_north_pole.z * item.hair_north_pole.z);
        if (np_len > 0.001f) {
            bool prev_auto = item.auto_hair_root;
            if (ImGui::Checkbox(get_locale_cstr("label.auto_hair_root"),
                                &item.auto_hair_root)) {
                if (item.auto_hair_root) {
                    // Compute ray from north-pole direction toward center,
                    // find first hit on base model triangles
                    vec3f dir = {item.hair_north_pole.x / np_len,
                                 item.hair_north_pole.y / np_len,
                                 item.hair_north_pole.z / np_len};
                    vec3f origin = {item.addon_center_point.x + dir.x * 500.0f,
                                    item.addon_center_point.y + dir.y * 500.0f,
                                    item.addon_center_point.z + dir.z * 500.0f};
                    vec3f ray_dir = {-dir.x, -dir.y, -dir.z};
                    vec3f hit = {item.addon_center_point.x,
                                 item.addon_center_point.y,
                                 item.addon_center_point.z};
                    bool found = false;
                    float best_t = 1e30f;

                    if (item.addon_base_node_id >= 0) {
                        auto base_it = items.find(item.addon_base_node_id);
                        if (base_it != items.end()) {
                            auto& base = *base_it->second;
                            auto test_tri = [&](const vec3f& v0,
                                                const vec3f& v1,
                                                const vec3f& v2) {
                                float t;
                                if (ray_triangle_intersect(origin, ray_dir, v0,
                                                           v1, v2, t) &&
                                    t < best_t) {
                                    best_t = t;
                                    hit = {origin.x + ray_dir.x * t,
                                           origin.y + ray_dir.y * t,
                                           origin.z + ray_dir.z * t};
                                    found = true;
                                }
                            };
                            if (!base.cached_mesh.empty()) {
                                for (const auto& entry : base.cached_mesh) {
                                    const auto& tri = std::get<0>(entry);
                                    test_tri({std::get<0>(tri).x, std::get<0>(tri).y, std::get<0>(tri).z},
                                             {std::get<1>(tri).x, std::get<1>(tri).y, std::get<1>(tri).z},
                                             {std::get<2>(tri).x, std::get<2>(tri).y, std::get<2>(tri).z});
                                }
                            } else {
                                for (const auto& tri : base.source_triangles) {
                                    test_tri({std::get<0>(tri).x, std::get<0>(tri).y, std::get<0>(tri).z},
                                             {std::get<1>(tri).x, std::get<1>(tri).y, std::get<1>(tri).z},
                                             {std::get<2>(tri).x, std::get<2>(tri).y, std::get<2>(tri).z});
                                }
                            }
                        }
                    }
                    if (!found) {
                        hit = {item.addon_center_point.x - dir.x * 10.0f,
                               item.addon_center_point.y - dir.y * 10.0f,
                               item.addon_center_point.z - dir.z * 10.0f};
                    }
                    item.common_hair_root_point = hit;
                    push_undo_now(item.id, std::nullopt, "Auto Hair Root");
                }
                // Propagate root point to each enabled strand's hidden_guide_points_start
                {
                    vec3f effective_root = item.common_hair_root_point;
                    {
                        vec3f to_center = {
                            item.addon_center_point.x - effective_root.x,
                            item.addon_center_point.y - effective_root.y,
                            item.addon_center_point.z - effective_root.z};
                        float dist = std::sqrt(to_center.x * to_center.x +
                                               to_center.y * to_center.y +
                                               to_center.z * to_center.z);
                        if (dist > 0.001f && item.hair_root_center_offset > 0.0f) {
                            vec3f dir = {to_center.x / dist, to_center.y / dist,
                                         to_center.z / dist};
                            float offset = item.hair_root_center_offset;
                            if (offset > dist) offset = dist;
                            effective_root = {effective_root.x + dir.x * offset,
                                              effective_root.y + dir.y * offset,
                                              effective_root.z + dir.z * offset};
                        }
                    }
                    if (item.auto_hair_root) {
                        strand.hidden_guide_points_start = {effective_root};
                        strand.hair_root_enabled = true;
                    } else {
                        strand.hidden_guide_points_start.clear();
                        strand.hair_root_enabled = false;
                    }
                    strand.mesh_dirty = true;
                }
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", get_locale_cstr("tooltip.auto_hair_root"));
        }
    }

    ImGui::Text(get_locale_cstr("label.guide_curve_points"),
                static_cast<int>(strand.guide_points.size()));

    if (strand.guide_points.empty()) {
        ImGui::TextWrapped("%s",
                           get_locale_cstr("label.no_guide_points"));
    } else {
        // Reserve space at bottom for the separator and clear button
        float bottom_reserve = ImGui::GetFrameHeightWithSpacing() +
                               ImGui::GetStyle().ItemSpacing.y;

        auto before_edit = capture_snapshot(item);
        EditResult all_edits;
        int delete_point = -1;
        int swap_up = -1;
        int swap_down = -1;

        // Reset per-point hover highlight each frame
        item.hovered_guide_point_strand_uuid.clear();
        item.hovered_guide_point_index = -1;

        if (ImGui::BeginTable("##gp_table", 5,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_ScrollY,
                              ImVec2(0, -bottom_reserve))) {
            ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 30.0f);
            ImGui::TableSetupColumn(get_locale_cstr("label.guide_point"),
                                    ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("##ctr_col",
                                    ImGuiTableColumnFlags_WidthFixed, 65.0f);
            ImGui::TableSetupColumn("##ord_col",
                                    ImGuiTableColumnFlags_WidthFixed, 40.0f);
            ImGui::TableSetupColumn("##act_col",
                                    ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableHeadersRow();

            for (size_t pi = 0; pi < strand.guide_points.size(); ++pi) {
                ImGui::PushID(static_cast<int>(pi));
                ImGui::TableNextRow();
                bool point_hovered = false;

                char label_buf[64];
                snprintf(label_buf, sizeof(label_buf),
                         get_locale_cstr("label.guide_point"),
                         static_cast<int>(pi + 1));

                // Column 1: Point number
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::Text("%d", static_cast<int>(pi + 1));

                // Column 2: Coordinates + Edit button
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::Text("(%.1f, %.1f, %.1f)",
                            strand.guide_points[pi].x,
                            strand.guide_points[pi].y,
                            strand.guide_points[pi].z);
                if (!point_hovered && ImGui::IsItemHovered())
                    point_hovered = true;
                ImGui::SameLine();
                char edit_popup_id[64];
                snprintf(edit_popup_id, sizeof(edit_popup_id), "Edit##gpe_%zu", pi);
                if (ImGui::SmallButton(edit_popup_id))
                    ImGui::OpenPopup(edit_popup_id);
                if (!point_hovered && ImGui::IsItemHovered())
                    point_hovered = true;

                // Edit popup (same as before)
                if (ImGui::BeginPopup(edit_popup_id)) {
                    ImGui::Text("%s", label_buf);
                    ImGui::Separator();
                    auto r = edit_vec3_stepper("", strand.guide_points[pi],
                                               0.5f, false, true);
                    all_edits.activated |= r.activated;
                    all_edits.deactivated_after_edit |= r.deactivated_after_edit;
                    all_edits.value_changed |= r.value_changed;

                    if (item.show_addon_center) {
                        vec3f to_center =
                            item.addon_center_point - strand.guide_points[pi];
                        float dist = to_center.length();
                        ImGui::TextDisabled("dist=%.2f", dist);

                        if (dist > 0.0001f) {
                            vec3f dir = to_center / dist;
                            static float kp_move_step = 0.5f;

                            if (ImGui::SmallButton("+##gpe_ctr")) {
                                strand.guide_points[pi] =
                                    strand.guide_points[pi] + dir * kp_move_step;
                                all_edits.value_changed = true;
                            }
                            if (ImGui::IsItemHovered())
                                ImGui::SetTooltip("%s", get_locale_cstr("tooltip.move_toward_center"));
                            if (ImGui::IsItemClicked(ImGuiMouseButton_Right))
                                ImGui::OpenPopup("kp_cmenu##gpe");
                            ImGui::SameLine();
                            if (ImGui::SmallButton("-##gpe_ctr")) {
                                strand.guide_points[pi] =
                                    strand.guide_points[pi] - dir * kp_move_step;
                                all_edits.value_changed = true;
                            }
                            if (ImGui::IsItemHovered())
                                ImGui::SetTooltip("%s", get_locale_cstr("tooltip.move_away_from_center"));
                            if (ImGui::IsItemClicked(ImGuiMouseButton_Right))
                                ImGui::OpenPopup("kp_cmenu##gpe");

                            if (ImGui::BeginPopup("kp_cmenu##gpe")) {
                                float new_dist = dist;
                                ImGui::SetNextItemWidth(120);
                                ImGui::DragFloat(get_locale_cstr("label.dist_to_center"),
                                                 &new_dist, 0.01f, 0.0001f, 100000.0f, "%.3f");
                                if (ImGui::IsItemActivated())
                                    all_edits.activated = true;
                                if (ImGui::IsItemDeactivatedAfterEdit())
                                    all_edits.deactivated_after_edit = true;
                                if (new_dist != dist) {
                                    strand.guide_points[pi] =
                                        item.addon_center_point - dir * new_dist;
                                    strand.mesh_dirty = true;
                                }
                                ImGui::SetNextItemWidth(120);
                                ImGui::DragFloat(get_locale_cstr("label.move_step"),
                                                 &kp_move_step, 0.01f, 0.01f, 10.0f, "%.2f");
                                ImGui::EndPopup();
                            }
                        }
                    }
                    ImGui::EndPopup();
                }

                // Column 3: Center +/- buttons (inline, compact)
                ImGui::TableNextColumn();
                if (item.show_addon_center) {
                    vec3f to_center =
                        item.addon_center_point - strand.guide_points[pi];
                    float dist = to_center.length();
                    ImGui::TextDisabled("dist=%.2f", dist);
                    if (dist > 0.0001f) {
                        static float kp_move_step = 0.5f;
                        vec3f dir = to_center / dist;

                        if (ImGui::SmallButton("+##gpi")) {
                            strand.guide_points[pi] =
                                strand.guide_points[pi] + dir * kp_move_step;
                            all_edits.value_changed = true;
                        }
                        if (ImGui::IsItemHovered()) {
                            ImGui::SetTooltip("%s", get_locale_cstr("tooltip.move_toward_center"));
                            point_hovered = true;
                        }
                        if (ImGui::IsItemClicked(ImGuiMouseButton_Right))
                            ImGui::OpenPopup("kp_cmenu_inline");
                        ImGui::SameLine();
                        if (ImGui::SmallButton("-##gpi")) {
                            strand.guide_points[pi] =
                                strand.guide_points[pi] - dir * kp_move_step;
                            all_edits.value_changed = true;
                        }
                        if (ImGui::IsItemHovered()) {
                            ImGui::SetTooltip("%s", get_locale_cstr("tooltip.move_away_from_center"));
                            point_hovered = true;
                        }
                        if (ImGui::IsItemClicked(ImGuiMouseButton_Right))
                            ImGui::OpenPopup("kp_cmenu_inline");

                        if (ImGui::BeginPopup("kp_cmenu_inline")) {
                            float new_dist = dist;
                            ImGui::SetNextItemWidth(120);
                            ImGui::DragFloat(get_locale_cstr("label.dist_to_center"),
                                             &new_dist, 0.01f, 0.0001f, 100000.0f, "%.3f");
                            if (ImGui::IsItemActivated())
                                all_edits.activated = true;
                            if (ImGui::IsItemDeactivatedAfterEdit())
                                all_edits.deactivated_after_edit = true;
                            if (new_dist != dist) {
                                strand.guide_points[pi] =
                                    item.addon_center_point - dir * new_dist;
                                strand.mesh_dirty = true;
                            }
                            ImGui::SetNextItemWidth(120);
                            ImGui::DragFloat(get_locale_cstr("label.move_step"),
                                             &kp_move_step, 0.01f, 0.01f, 10.0f, "%.2f");
                            ImGui::EndPopup();
                        }
                    } else {
                        ImGui::TextDisabled("-");
                    }
                }

                // Column 4: Reorder buttons (up/down)
                ImGui::TableNextColumn();
                if (pi > 0) {
                    if (ImGui::SmallButton("^")) swap_up = static_cast<int>(pi);
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.move_point_up"));
                        point_hovered = true;
                    }
                }
                if (pi < strand.guide_points.size() - 1) {
                    if (pi > 0) ImGui::SameLine();
                    if (ImGui::SmallButton("v")) swap_down = static_cast<int>(pi);
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.move_point_down"));
                        point_hovered = true;
                    }
                }

                // Column 5: Delete button
                ImGui::TableNextColumn();
                if (ImGui::SmallButton("X")) delete_point = static_cast<int>(pi);

                if (point_hovered) {
                    item.hovered_guide_point_strand_uuid = strand.uuid;
                    item.hovered_guide_point_index = static_cast<int>(pi);
                }

                ImGui::PopID();
            }
            ImGui::EndTable();
        }

        // 处理坐标编辑的撤销
        if (all_edits.activated) {
            begin_edit(item.id);
        }
        if (all_edits.deactivated_after_edit) {
            end_edit(item.id, "Guide Point Edit");
            strand.mesh_dirty = true;
        } else if (all_edits.value_changed && !item.collision_edit_active) {
            push_undo_now(item.id, before_edit, "Guide Point Edit");
            strand.mesh_dirty = true;
        }

        // 处理上移
        if (swap_up >= 0) {
            push_undo_now(item.id, std::nullopt, "Move Guide Point Up");
            std::swap(strand.guide_points[swap_up],
                      strand.guide_points[swap_up - 1]);
            strand.mesh_dirty = true;
        }
        // 处理下移
        if (swap_down >= 0) {
            push_undo_now(item.id, std::nullopt, "Move Guide Point Down");
            std::swap(strand.guide_points[swap_down],
                      strand.guide_points[swap_down + 1]);
            strand.mesh_dirty = true;
        }
        // 处理删除
        if (delete_point >= 0) {
            push_undo_now(item.id, std::nullopt, "Delete Guide Point");
            strand.guide_points.erase(
                strand.guide_points.begin() + delete_point);
            strand.mesh_dirty = true;
        }
    }

    ImGui::Separator();
    if (ImGui::Button(get_locale_cstr("action.clear_guide_points"))) {
        push_undo_now(item.id, std::nullopt, "Clear Guide Points");
        strand.guide_points.clear();
        strand.mesh_dirty = true;
    }

    ImGui::End();
}

void RenderVoxelList::render_width_editor_window() {
    if (!show_addon_window)
        return;
    if (!show_width_editor_window)
        return;

    // 互斥：打开宽度编辑器时关闭引导曲线窗口
    if (show_guide_curve_window) {
        auto git = items.find(render_id);
        if (git != items.end()) {
            git->second->guide_curve_drawing_active = false;
            git->second->active_guide_draw_strand.clear();
        }
        show_guide_curve_window = false;
    }
    // 互斥：打开宽度编辑器时关闭自动宽度窗口
    if (show_hairline_plane_window) {
        auto hit = items.find(render_id);
        if (hit != items.end()) {
            hit->second->hairline_point_picking_active = false;
        }
        show_hairline_plane_window = false;
    }

    ImGui::SetNextWindowSize(ImVec2(380, 400), ImGuiCond_Once);
    bool window_open = true;
    if (!ImGui::Begin(get_locale_cstr("window.width_editor"), &window_open)) {
        ImGui::End();
        return;
    }

    // 点击 X 关闭按钮
    if (!window_open) {
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->width_editing_active = false;
            it->second->active_width_edit_strand.clear();
            it->second->perpoint_section_editing_active = false;
            it->second->active_perpoint_section_edit_strand.clear();
            it->second->active_perpoint_section_edit_width_idx = -1;
        }
        show_width_editor_window = false;
        show_perpoint_section_editor_window = false;
        ImGui::End();
        return;
    }

    std::lock_guard<std::mutex> lock(locker);
    auto item_it = items.find(render_id);
    if (item_it == items.end() || item_it->second->source_type != 2) {
        ImGui::End();
        return;
    }

    RenderVoxelItem& item = *item_it->second;
    std::string strand_uuid = item.active_width_edit_strand;

    if (strand_uuid.empty() || !item.find_strand_by_uuid(strand_uuid) ||
        !item.width_editing_active) {
        show_width_editor_window = false;
        show_perpoint_section_editor_window = false;
        item.perpoint_section_editing_active = false;
        item.active_perpoint_section_edit_strand.clear();
        item.active_perpoint_section_edit_width_idx = -1;
        ImGui::End();
        return;
    }

    auto& strand = *item.find_strand_by_uuid(strand_uuid);
    ImGui::Text("%s", strand.name.c_str());
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.2f, 0.7f, 0.3f, 1.0f), "%s",
                       get_locale_cstr("action.edit_width"));

    // Undo / Redo buttons
    {
        bool undo_disabled = !can_undo(item.id);
        bool redo_disabled = !can_redo(item.id);
        if (undo_disabled)
            ImGui::BeginDisabled();
        if (ImGui::SmallButton(get_locale_cstr("action.undo"))) {
            undo(item.id);
            for (auto& s : item.hair_strands) s.mesh_dirty = true;
        }
        if (undo_disabled)
            ImGui::EndDisabled();
        ImGui::SameLine();
        if (redo_disabled)
            ImGui::BeginDisabled();
        if (ImGui::SmallButton(get_locale_cstr("action.redo"))) {
            redo(item.id);
            for (auto& s : item.hair_strands) s.mesh_dirty = true;
        }
        if (redo_disabled)
            ImGui::EndDisabled();
    }

    // Keyboard shortcuts (Ctrl+Z / Ctrl+Y) are handled globally
    // by the SDL event loop in ui.hpp to avoid double-firing.

    ImGui::Separator();
    ImGui::Text(get_locale_cstr("label.width_points"),
                static_cast<int>(strand.width_points.size()));

    if (strand.width_points.empty()) {
        ImGui::TextWrapped("%s",
                           get_locale_cstr("label.no_width_points"));
    } else {
        // 高度随窗口变化，负高度为底部的分隔线和清空按钮预留空间
        float bottom_reserve = ImGui::GetFrameHeightWithSpacing() +
                               ImGui::GetStyle().ItemSpacing.y;
        ImGui::BeginChild("WidthPointsList", ImVec2(0, -bottom_reserve), true);

        auto before_edit = capture_snapshot(item);
        EditResult all_edits;
        int delete_wp = -1;

        // Reset hover highlight each frame
        item.hovered_width_point_index = -1;

        for (size_t wi = 0; wi < strand.width_points.size(); ++wi) {
            auto& wp = strand.width_points[wi];
            ImGui::PushID(static_cast<int>(wi));

            // 点信息：显示到曲线的实际距离（scale 即当前宽度/距离）
            ImGui::Text(get_locale_cstr("label.width_point_entry"),
                        static_cast<int>(wi + 1),
                        static_cast<int>(wp.curve_id));
            ImGui::SameLine();
            char dist_buf[64];
            snprintf(dist_buf, sizeof(dist_buf), "dist=%.2f",
                     static_cast<double>(wp.scale));
            ImGui::TextDisabled("%s", dist_buf);

            // Save row top for hover→cyan line in 3D viewport
            ImVec2 row_min = ImGui::GetItemRectMin();

            // 向量长度（scale = 从引导曲线到表面的距离）
            float old_scale = wp.scale;
            ImGui::SetNextItemWidth(140);
            ImGui::DragFloat(get_locale_cstr("label.width_vector_length"),
                             &wp.scale, 0.01f, 0.01f, 10.0f, "%.2f");
            if (ImGui::IsItemActivated())
                all_edits.activated = true;
            if (ImGui::IsItemDeactivatedAfterEdit())
                all_edits.deactivated_after_edit = true;
            if (old_scale != wp.scale)
                all_edits.value_changed = true;

            // 方向向量编辑器（允许手动调整方向）
            {
                auto dir_edit = edit_vec3_stepper(
                    get_locale_cstr("label.width_direction"),
                    wp.direction, 0.1f, true);
                all_edits.activated |= dir_edit.activated;
                all_edits.deactivated_after_edit |=
                    dir_edit.deactivated_after_edit;
                all_edits.value_changed |= dir_edit.value_changed;
            }

            // --- 自动旋转按钮（仅当中心点启用时显示）---
            if (item.show_addon_center) {
                if (ImGui::SmallButton(
                        get_locale_cstr("action.auto_rotate_section"))) {
                    int strand_idx_auto = static_cast<int>(&strand - item.hair_strands.data());
                    auto sample = item.sample_guide_curve_at(strand_idx_auto, wp.curve_id);
                    float angle_deg = 0.0f;
                    if (compute_auto_section_rotation(
                            sample.position, sample.tangent,
                            item.addon_center_point, angle_deg)) {
                        push_undo_now(item.id, std::nullopt,
                                      "Auto-Rotate Section");
                        strand.section_rotation = angle_deg;
                        strand.mesh_dirty = true;
                    }
                }

                // --- 沿中心点连线移动（端点朝/背中心点方向移动）---
                // 移动步长（静态变量，所有宽度向量共享，右键菜单中可调）
                static float wp_move_step = 0.5f;

                auto move_along_center = [&](float sign) {
                    int strand_idx_mv = static_cast<int>(&strand - item.hair_strands.data());
                    auto sample =
                        item.sample_guide_curve_at(strand_idx_mv, wp.curve_id);
                    vec3f P = sample.position;
                    vec3f W = P + wp.direction * wp.scale;  // 向量端点
                    vec3f to_center = item.addon_center_point - W;
                    float dist = to_center.length();
                    if (dist < 0.0001f)
                        return;
                    vec3f dir = to_center / dist;
                    vec3f new_W = W + dir * (sign * wp_move_step);
                    vec3f v = new_W - P;
                    float len = v.length();
                    if (len < 0.0001f)
                        return;
                    wp.scale = len;
                    wp.direction = v / len;
                    all_edits.value_changed = true;
                };
                ImGui::TextUnformatted(get_locale_cstr("label.radial_move"));
                ImGui::SameLine();
                if (ImGui::SmallButton("+")) {
                    move_along_center(1.0f);
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "%s", get_locale_cstr("tooltip.move_toward_center"));
                if (ImGui::IsItemClicked(ImGuiMouseButton_Right))
                    ImGui::OpenPopup("wp_center_menu");
                ImGui::SameLine();
                if (ImGui::SmallButton("-")) {
                    move_along_center(-1.0f);
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "%s", get_locale_cstr("tooltip.move_away_from_center"));
                if (ImGui::IsItemClicked(ImGuiMouseButton_Right))
                    ImGui::OpenPopup("wp_center_menu");

                // 右键菜单：直接编辑端点离中心距离与移动步长
                if (ImGui::BeginPopup("wp_center_menu")) {
                    int strand_idx_popup = static_cast<int>(&strand - item.hair_strands.data());
                    auto sample = item.sample_guide_curve_at(strand_idx_popup, wp.curve_id);
                    vec3f P = sample.position;
                    vec3f W = P + wp.direction * wp.scale;  // 向量端点
                    vec3f to_center = item.addon_center_point - W;
                    float dist = to_center.length();
                    if (dist > 0.0001f) {
                        vec3f dir = to_center / dist;
                        float new_dist = dist;
                        ImGui::SetNextItemWidth(120);
                        ImGui::DragFloat(
                            get_locale_cstr("label.dist_to_center"),
                            &new_dist, 0.01f, 0.0001f, 100000.0f, "%.3f");
                        if (ImGui::IsItemActivated())
                            all_edits.activated = true;
                        if (ImGui::IsItemDeactivatedAfterEdit())
                            all_edits.deactivated_after_edit = true;
                        if (new_dist != dist) {
                            vec3f new_W =
                                item.addon_center_point - dir * new_dist;
                            vec3f v = new_W - P;
                            float len = v.length();
                            if (len > 0.0001f) {
                                wp.scale = len;
                                wp.direction = v / len;
                                // 历史记录由 activated/deactivated 在释放时创建，
                                // 拖动过程中只更新网格，避免每帧产生历史记录
                                strand.mesh_dirty = true;
                            }
                        }
                    }
                    ImGui::SetNextItemWidth(120);
                    ImGui::DragFloat(get_locale_cstr("label.move_step"),
                                     &wp_move_step, 0.01f, 0.01f, 10.0f,
                                     "%.2f");
                    ImGui::EndPopup();
                }
            }

            // --- Per-point section editor button ---
            bool is_perpoint_editing =
                (item.active_perpoint_section_edit_strand == strand_uuid &&
                 item.active_perpoint_section_edit_width_idx ==
                     static_cast<int>(wi));
            if (is_perpoint_editing) {
                ImGui::PushStyleColor(ImGuiCol_Button,
                                      ImVec4(0.5f, 0.5f, 0.9f, 1.0f));
            }
            if (ImGui::SmallButton(
                    is_perpoint_editing
                        ? get_locale_cstr("action.stop_edit_perpoint_section")
                        : get_locale_cstr("action.edit_perpoint_section"))) {
                if (is_perpoint_editing) {
                    item.perpoint_section_editing_active = false;
                    item.active_perpoint_section_edit_strand.clear();
                    item.active_perpoint_section_edit_width_idx = -1;
                    show_perpoint_section_editor_window = false;
                } else {
                    // Close global section editor (mutual exclusion)
                    if (show_cross_section_editor_window) {
                        item.active_section_edit_strand.clear();
                        show_cross_section_editor_window = false;
                    }
                    item.perpoint_section_editing_active = true;
                    item.active_perpoint_section_edit_strand = strand_uuid;
                    item.active_perpoint_section_edit_width_idx =
                        static_cast<int>(wi);
                    show_perpoint_section_editor_window = true;
                }
            }
            
            if (ImGui::IsItemHovered()) {
                if (is_perpoint_editing){
                    ImGui::SetTooltip("%s",
                        get_locale_cstr("tooltip.stop_edit_perpoint_section"));
                } else {
                    ImGui::SetTooltip("%s",
                        get_locale_cstr("tooltip.edit_perpoint_section"));
                }
            }
            if (is_perpoint_editing) {
                ImGui::PopStyleColor();
            }

            // Show [custom] indicator if this width point has an override
            if (wp.section_state.vertices.size() >= 3) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.5f, 0.9f, 0.5f, 1.0f), "%s",
                                   get_locale_cstr(
                                       "label.perpoint_section_indicator"));
            }

            ImGui::SameLine();
            if (ImGui::SmallButton(
                    get_locale_cstr("action.delete_width_point"))) {
                delete_wp = static_cast<int>(wi);
            }

            ImGui::Separator();

            // If mouse is over this row, highlight the corresponding
            // width line in the 3D viewport in cyan (0, 1, 1).
            ImVec2 row_max = ImGui::GetItemRectMax();
            row_min.x = ImGui::GetWindowPos().x;
            row_max.x = ImGui::GetWindowPos().x +
                        ImGui::GetWindowWidth();
            if (ImGui::IsMouseHoveringRect(row_min, row_max)) {
                item.hovered_width_point_index = static_cast<int>(wi);
            }

            ImGui::PopID();
        }

        // 处理缩放的撤销
        if (all_edits.activated) {
            begin_edit(item.id);
        }
        if (all_edits.deactivated_after_edit) {
            end_edit(item.id, "Width Scale Edit");
            strand.mesh_dirty = true;
        } else if (all_edits.value_changed && !item.collision_edit_active) {
            // Discrete change (keyboard input, +/- buttons) — no drag session
            push_undo_now(item.id, before_edit, "Width Scale Edit");
            strand.mesh_dirty = true;
        }

        // 处理删除
        if (delete_wp >= 0) {
            // Clean up per-point section editor if the deleted point was
            // being edited
            if (item.active_perpoint_section_edit_strand == strand_uuid &&
                item.active_perpoint_section_edit_width_idx == delete_wp) {
                item.perpoint_section_editing_active = false;
                item.active_perpoint_section_edit_strand.clear();
                item.active_perpoint_section_edit_width_idx = -1;
                show_perpoint_section_editor_window = false;
            } else if (item.active_perpoint_section_edit_strand == strand_uuid &&
                       item.active_perpoint_section_edit_width_idx >
                           delete_wp) {
                item.active_perpoint_section_edit_width_idx--;
            }
            push_undo_now(item.id, std::nullopt, "Delete Width Point");
            strand.width_points.erase(
                strand.width_points.begin() + delete_wp);
            strand.mesh_dirty = true;
        }

        ImGui::EndChild();
    }

    ImGui::Separator();
    if (ImGui::Button(get_locale_cstr("action.clear_width_points"))) {
        push_undo_now(item.id, std::nullopt, "Clear Width Points");
        strand.width_points.clear();
        strand.mesh_dirty = true;
    }

    ImGui::End();
}

void RenderVoxelList::render_object_editor_addons() {
    // 窗口是否可见取决于当前选中节点是否为附加件模式
    {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it == items.end() || item_it->second->source_type != 2) {
            show_addon_window = false;
            return;
        }
    }

    show_addon_window = true;

    // 无关闭按钮，窗口随附加件模式自动显示/隐藏
    // 初始位置：右上角顶点与物体编辑器左上角顶点重合
    ImGui::SetNextWindowPos(
        ImVec2(static_cast<float>(window_width) - 360.0f,
               static_cast<float>(menu_height)),
        ImGuiCond_Once, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(360, 400), ImGuiCond_Once);
    if (!ImGui::Begin(get_locale_cstr("window.addon_editor"), nullptr,
                      ImGuiWindowFlags_NoCollapse)) {
        ImGui::End();
        return;
    }

    std::lock_guard<std::mutex> lock(locker);
    auto item_it = items.find(render_id);
    if (item_it == items.end()) {
        ImGui::TextUnformatted(get_locale_cstr("label.no_active_item"));
        ImGui::End();
        return;
    }

    RenderVoxelItem& item = *item_it->second;

    // 确保当前节点是附加件模式（二次检查，持有锁）
    if (item.source_type != 2) {
        ImGui::End();
        return;
    }

    // 底模可见性（独立变量，与右下角按钮OR逻辑）
    if (ImGui::Checkbox(get_locale_cstr("label.show_origin_mesh"),
                        &item.showOriginMeshAddon)) {
        // 仅影响附加件自身的显示状态，不联动按钮
    }
    ImGui::SameLine();
    if (item.addon_base_node_id >= 0 && !item.origin_mesh_renderer.empty()) {
        ImGui::Text(get_locale_cstr("label.addon_base_applied"),
                    item.addon_base_node_id);
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.3f, 1.0f), "%s",
                           get_locale_cstr("label.addon_no_base_selected"));
    }

    ImGui::Separator();

    // 自动宽度按钮（打开自动宽度窗口）
    if (ImGui::Button(get_locale_cstr("action.auto_width"))) {
        show_hairline_plane_window = true;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s",
            get_locale_cstr("tooltip.auto_width"));
    }

    // 发际线平面窗口存在时，自动启用发际线平面
    if (show_hairline_plane_window) {
        item.hairline_plane_enabled = true;
    }

    ImGui::SameLine();

    // 坐标系配置编辑器按钮
    if (ImGui::Button(get_locale_cstr("action.angle_config"))) {
        show_angle_config_window = true;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s",
            get_locale_cstr("tooltip.angle_config"));
    }

    ImGui::SameLine();

    // 发根编辑按钮
    if (ImGui::Button(get_locale_cstr("action.hair_root_edit"))) {
        show_hair_root_window = true;
        item.hair_root_edit_active = true;
        item.show_addon_center = true;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s",
            get_locale_cstr("tooltip.hair_root_edit"));
    }

    ImGui::Separator();

    // 附加件类型下拉框
    const char* addon_type_names[] = {
        get_locale_cstr("label.addon_type_hair"),
    };
    if (ImGui::Combo(get_locale_cstr("label.addon_type"), &item.addon_type,
                     addon_type_names,
                     static_cast<int>(AddonType::COUNT))) {
        push_undo_now(item.id, std::nullopt, "Addon Type");
    }

    ImGui::Separator();

    // ===== 毛发模式 =====
    if (item.addon_type == static_cast<int>(AddonType::HAIR)) {
        // 添加发束按钮
        if (ImGui::Button(get_locale_cstr("action.add_hair_strand"))) {
            push_undo_now(item.id, std::nullopt, "Add Hair Strand");
            HairStrand strand;
            strand.uuid = generate_uuid();
            strand.name = "Strand " + std::to_string(item.hair_strands.size() + 1);
            strand.expanded = true;
            item.hair_strands.push_back(strand);
        }

        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.ortho_projection"))) {
            show_ortho_setup_window = true;
            ortho_state.viewport_size_defaulted = false;
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", get_locale_cstr("window.ortho_projection_setup"));

        ImGui::Separator();

        // 发束列表
        int delete_idx = -1;
        item.hovered_strand_uuid.clear();  // reset hover highlight each frame
        for (size_t i = 0; i < item.hair_strands.size(); ++i) {
            auto& strand = item.hair_strands[i];
            ImGui::PushID(static_cast<int>(i));
            bool strand_hovered = false;

            char header_label[64];
            if (!strand.name.empty()) {
                snprintf(header_label, sizeof(header_label), "%s##strand_%zu",
                         strand.name.c_str(), i);
            } else {
                snprintf(header_label, sizeof(header_label),
                         get_locale_cstr("label.hair_strand"),
                         static_cast<int>(i + 1));
            }
            int header_flags = ImGuiTreeNodeFlags_AllowOverlap;
            if (strand.expanded)
                header_flags |= ImGuiTreeNodeFlags_DefaultOpen;
            bool expanded = ImGui::CollapsingHeader(header_label, header_flags);
            strand.expanded = expanded;
            if (ImGui::IsItemHovered()) strand_hovered = true;

            // Show warning indicator when alpha_wrap repair failed for this strand
            if (strand.repair_failed) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.0f, 1.0f), " %s",
                                   get_locale_cstr("label.repair_failed"));
            }

            // Visibility toggle (display only, collision unaffected)
            ImGui::SameLine();
            bool old_vis = strand.visible;
            ImGui::Checkbox("##strand_vis", &strand.visible);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", get_locale_cstr("tooltip.strand_visible"));
            if (old_vis != strand.visible) strand.mesh_dirty = true;

            if (expanded) {
                // Snapshot for undo/redo of parameter edits
                auto param_snapshot = capture_snapshot(item);
                EditResult param_edits;

                // 发束生成类型选择
                const char* gen_type_names[] = {
                    get_locale_cstr("label.strand_type_normal"),
                    get_locale_cstr("label.strand_type_candy"),
                    get_locale_cstr("label.strand_type_braid"),
                };
                int type_int = static_cast<int>(strand.gen_type);
                ImGui::SetNextItemWidth(140);
                if (ImGui::Combo("##strand_type", &type_int, gen_type_names, 3)) {
                    strand.gen_type =
                        static_cast<HairStrandGenType>(type_int);
                    strand.mesh_dirty = true;
                    param_edits.value_changed = true;
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", get_locale_cstr("tooltip.strand_type"));

                bool is_normal =
                    (strand.gen_type == HairStrandGenType::NORMAL);

                // 三个按钮行
                // 上移
                if (i > 0) {
                    if (ImGui::Button(get_locale_cstr("action.move_up"))) {
                        push_undo_now(item.id, std::nullopt,
                                      "Move Strand Up");
                        std::swap(item.hair_strands[i],
                                  item.hair_strands[i - 1]);
                        item.hair_strands[i].mesh_dirty = true;
                        item.hair_strands[i - 1].mesh_dirty = true;
                    }
                    ImGui::SameLine();
                }
                // 下移
                if (i < item.hair_strands.size() - 1) {
                    if (ImGui::Button(get_locale_cstr("action.move_down"))) {
                        push_undo_now(item.id, std::nullopt,
                                      "Move Strand Down");
                        std::swap(item.hair_strands[i],
                                  item.hair_strands[i + 1]);
                        item.hair_strands[i].mesh_dirty = true;
                        item.hair_strands[i + 1].mesh_dirty = true;
                    }
                    ImGui::SameLine();
                }
                
                // 绘制引导曲线（自锁按钮）
                ImGui::SameLine();
                bool is_drawing =
                    (item.active_guide_draw_strand == item.hair_strands[i].uuid &&
                     item.guide_curve_drawing_active);
                if (is_drawing) {
                    ImGui::PushStyleColor(ImGuiCol_Button,
                                          ImVec4(0.8f, 0.3f, 0.3f, 1.0f));
                }
                if (ImGui::Button(
                        is_drawing
                            ? get_locale_cstr("action.stop_drawing")
                            : get_locale_cstr("action.draw_guide_curve"))) {
                    if (is_drawing) {
                        item.guide_curve_drawing_active = false;
                        item.active_guide_draw_strand.clear();
                        show_guide_curve_window = false;
                    } else {
                        // 互斥：打开引导曲线时关闭宽度编辑器
                        if (item.width_editing_active) {
                            item.width_editing_active = false;
                            item.active_width_edit_strand.clear();
                            show_width_editor_window = false;
                        }
                        item.guide_curve_drawing_active = true;
                        item.active_guide_draw_strand = item.hair_strands[i].uuid;
                        show_guide_curve_window = true;
                    }
                }
                if (is_drawing) {
                    ImGui::PopStyleColor();
                }

                // --- 编辑宽度向量（仅普通发束） ---
                if (is_normal) {
                    ImGui::SameLine();
                    bool is_width_editing_popup =
                        (item.active_width_edit_strand == item.hair_strands[i].uuid &&
                            item.width_editing_active);
                    if (ImGui::Button(
                            is_width_editing_popup
                                ? get_locale_cstr("action.stop_width_edit")
                                : get_locale_cstr("action.edit_width"))) {
                        if (is_width_editing_popup) {
                            item.width_editing_active = false;
                            item.active_width_edit_strand.clear();
                            show_width_editor_window = false;
                        } else {
                            if (item.guide_curve_drawing_active) {
                                item.guide_curve_drawing_active = false;
                                item.active_guide_draw_strand.clear();
                                show_guide_curve_window = false;
                            }
                            item.width_editing_active = true;
                            item.active_width_edit_strand = item.hair_strands[i].uuid;
                            show_width_editor_window = true;
                        }
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.edit_width"));
                }

                // 省略号菜单按钮（始终显示，弹出菜单包含编辑宽度/截面/删除/清空）
                ImGui::SameLine();
                char more_menu_id[64];
                snprintf(more_menu_id, sizeof(more_menu_id), "...##strand_more_%zu", i);
                if (ImGui::Button(more_menu_id)) {
                    ImGui::OpenPopup(more_menu_id);
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", get_locale_cstr("tooltip.strand_more"));
                if (ImGui::BeginPopup(more_menu_id)) {

                    // --- 编辑截面（仅普通发束可用） ---
                    if (!is_normal) ImGui::BeginDisabled();
                    bool is_section_editing_popup =
                        (item.active_section_edit_strand == item.hair_strands[i].uuid);
                    if (ImGui::MenuItem(
                            is_section_editing_popup
                                ? get_locale_cstr("action.stop_edit_section")
                                : get_locale_cstr("action.edit_section"))) {
                        if (is_section_editing_popup) {
                            item.active_section_edit_strand.clear();
                            show_cross_section_editor_window = false;
                        } else {
                            bool has_overrides = false;
                            for (const auto& wp :
                                 item.hair_strands[i].width_points) {
                                if (wp.section_state.vertices.size() >= 3) {
                                    has_overrides = true;
                                    break;
                                }
                            }
                            if (has_overrides) {
                                show_perpoint_confirm_global_open = true;
                                pending_global_section_strand =
                                    static_cast<int>(i);
                            } else {
                                item.active_section_edit_strand =
                                    item.hair_strands[i].uuid;
                                show_cross_section_editor_window = true;
                            }
                        }
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.edit_section"));
                    if (!is_normal) ImGui::EndDisabled();

                    ImGui::Separator();

                    // --- 重命名发束 ---
                    if (ImGui::MenuItem(get_locale_cstr("action.rename_strand"))) {
                        pending_rename_strand_uuid = item.hair_strands[i].uuid;
                        strncpy(rename_buffer, item.hair_strands[i].name.c_str(),
                                sizeof(rename_buffer) - 1);
                        rename_buffer[sizeof(rename_buffer) - 1] = '\0';
                        // OpenPopup is called after EndPopup below
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.rename_strand"));

                    // --- 删除发束 ---
                    if (ImGui::MenuItem(get_locale_cstr("action.delete_strand"))) {
                        delete_idx = static_cast<int>(i);
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.delete_strand"));

                    // --- 清空引导点 ---
                    if (ImGui::MenuItem(get_locale_cstr("action.clear_guide_points"))) {
                        push_undo_now(item.id, std::nullopt, "Clear Guide Points");
                        strand.guide_points.clear();
                        strand.mesh_dirty = true;
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.clear_guide_points"));

                    ImGui::EndPopup();
                }

                // 显示点数信息（所有类型共用引导曲线点数）
                ImGui::Text(get_locale_cstr("label.guide_curve_points"),
                            static_cast<int>(strand.guide_points.size()));

                if (is_normal) {
                    // --- NORMAL type: existing params ---
                    ImGui::SameLine();
                    ImGui::Text(get_locale_cstr("label.width_points"),
                                static_cast<int>(strand.width_points.size()));

                    // Section rotation slider
                    ImGui::SetNextItemWidth(160);
                    float old_rot = strand.section_rotation;
                    ImGui::SliderFloat(get_locale_cstr("label.section_rotation"),
                                       &strand.section_rotation, -180.0f, 180.0f,
                                       "%.0f deg");
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_rot != strand.section_rotation) {
                        strand.mesh_dirty = true;
                        param_edits.value_changed = true;
                    }

                    // Section subdiv
                    ImGui::SetNextItemWidth(160);
                    int old_section_subdiv = strand.section_subdiv;
                    ImGui::SliderInt(get_locale_cstr("label.section_subdiv"),
                                     &strand.section_subdiv, 1, 32);
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_section_subdiv != strand.section_subdiv) {
                        strand.mesh_dirty = true;
                        param_edits.value_changed = true;
                    }
                } else if (strand.gen_type ==
                           HairStrandGenType::CANDIED_HAWTHORN) {
                    // --- 糖葫芦 parameters ---
                    ImGui::SetNextItemWidth(160);
                    float old_ccr = strand.candy_cylinder_radius;
                    ImGui::DragFloat(get_locale_cstr("label.candy_cylinder_radius"),
                                     &strand.candy_cylinder_radius, 0.1f, 0.1f,
                                     20.0f, "%.1f");
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_ccr != strand.candy_cylinder_radius) { strand.mesh_dirty = true; param_edits.value_changed = true; }

                    ImGui::SetNextItemWidth(160);
                    float old_ces = strand.candy_ellipsoid_spacing;
                    ImGui::DragFloat(get_locale_cstr("label.candy_ellipsoid_spacing"),
                                     &strand.candy_ellipsoid_spacing, 0.5f, 0.5f,
                                     50.0f, "%.1f");
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_ces != strand.candy_ellipsoid_spacing) { strand.mesh_dirty = true; param_edits.value_changed = true; }

                    ImGui::SetNextItemWidth(160);
                    float old_era = strand.candy_ellipsoid_radius_a;
                    ImGui::DragFloat(get_locale_cstr("label.candy_ellipsoid_radius_a"),
                                     &strand.candy_ellipsoid_radius_a, 0.1f, 0.1f,
                                     20.0f, "%.1f");
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_era != strand.candy_ellipsoid_radius_a) { strand.mesh_dirty = true; param_edits.value_changed = true; }

                    ImGui::SetNextItemWidth(160);
                    float old_erb = strand.candy_ellipsoid_radius_b;
                    ImGui::DragFloat(get_locale_cstr("label.candy_ellipsoid_radius_b"),
                                     &strand.candy_ellipsoid_radius_b, 0.1f, 0.2f,
                                     30.0f, "%.1f");
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_erb != strand.candy_ellipsoid_radius_b) { strand.mesh_dirty = true; param_edits.value_changed = true; }

                    // Joint toggle
                    bool old_cj = strand.candy_use_joints;
                    ImGui::Checkbox(get_locale_cstr("label.candy_use_joints"),
                                    &strand.candy_use_joints);
                    if (old_cj != strand.candy_use_joints) { strand.mesh_dirty = true; param_edits.value_changed = true; }

                    // Tip params (shared between special types)
                    ImGui::SeparatorText(get_locale_cstr("label.tip_params"));
                    ImGui::SetNextItemWidth(160);
                    float old_tl = strand.special_tip_length;
                    ImGui::DragFloat(get_locale_cstr("label.special_tip_length"),
                                     &strand.special_tip_length, 0.1f, 0.1f, 30.0f,
                                     "%.1f");
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_tl != strand.special_tip_length) { strand.mesh_dirty = true; param_edits.value_changed = true; }

                    ImGui::SetNextItemWidth(160);
                    float old_tr = strand.special_tip_radius;
                    ImGui::DragFloat(get_locale_cstr("label.special_tip_radius"),
                                     &strand.special_tip_radius, 0.1f, 0.1f, 20.0f,
                                     "%.1f");
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_tr != strand.special_tip_radius) { strand.mesh_dirty = true; param_edits.value_changed = true; }
                } else if (strand.gen_type == HairStrandGenType::BRAID) {
                    // --- 麻花辫 parameters ---
                    ImGui::SetNextItemWidth(160);
                    float old_bcr = strand.braid_core_radius;
                    ImGui::DragFloat(get_locale_cstr("label.braid_core_radius"),
                                     &strand.braid_core_radius, 0.1f, 0.1f, 10.0f,
                                     "%.1f");
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_bcr != strand.braid_core_radius) { strand.mesh_dirty = true; param_edits.value_changed = true; }

                    ImGui::SetNextItemWidth(160);
                    float old_bsr = strand.braid_strand_radius;
                    ImGui::DragFloat(get_locale_cstr("label.braid_strand_radius"),
                                     &strand.braid_strand_radius, 0.05f, 0.1f, 10.0f,
                                     "%.2f");
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_bsr != strand.braid_strand_radius) { strand.mesh_dirty = true; param_edits.value_changed = true; }

                    ImGui::SetNextItemWidth(160);
                    float old_bbr = strand.braid_braid_radius;
                    ImGui::DragFloat(get_locale_cstr("label.braid_braid_radius"),
                                     &strand.braid_braid_radius, 0.1f, 0.2f, 20.0f,
                                     "%.1f");
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_bbr != strand.braid_braid_radius) { strand.mesh_dirty = true; param_edits.value_changed = true; }

                    ImGui::SetNextItemWidth(160);
                    float old_btp = strand.braid_twist_pitch;
                    ImGui::DragFloat(get_locale_cstr("label.braid_twist_pitch"),
                                     &strand.braid_twist_pitch, 1.0f, 2.0f, 200.0f,
                                     "%.1f");
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_btp != strand.braid_twist_pitch) { strand.mesh_dirty = true; param_edits.value_changed = true; }

                    ImGui::SetNextItemWidth(160);
                    int old_bsc = strand.braid_strand_count;
                    ImGui::DragInt(get_locale_cstr("label.braid_strand_count"),
                                   &strand.braid_strand_count, 1, 2, 6);
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_bsc != strand.braid_strand_count) { strand.mesh_dirty = true; param_edits.value_changed = true; }

                    // Joint toggle
                    bool old_bj = strand.braid_use_joints;
                    ImGui::Checkbox(get_locale_cstr("label.braid_use_joints"),
                                    &strand.braid_use_joints);
                    if (old_bj != strand.braid_use_joints) { strand.mesh_dirty = true; param_edits.value_changed = true; }

                    // Tip params (shared between special types)
                    ImGui::SeparatorText(get_locale_cstr("label.tip_params"));
                    ImGui::SetNextItemWidth(160);
                    float old_tl = strand.special_tip_length;
                    ImGui::DragFloat(get_locale_cstr("label.special_tip_length"),
                                     &strand.special_tip_length, 0.1f, 0.1f, 30.0f,
                                     "%.1f");
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_tl != strand.special_tip_length) { strand.mesh_dirty = true; param_edits.value_changed = true; }

                    ImGui::SetNextItemWidth(160);
                    float old_tr = strand.special_tip_radius;
                    ImGui::DragFloat(get_locale_cstr("label.special_tip_radius"),
                                     &strand.special_tip_radius, 0.1f, 0.1f, 20.0f,
                                     "%.1f");
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_tr != strand.special_tip_radius) { strand.mesh_dirty = true; param_edits.value_changed = true; }
                }

                // 几何体细分精度（特殊发束类型：糖葫芦/麻花辫）
                if (!is_normal) {
                    ImGui::SetNextItemWidth(160);
                    int old_sq = strand.special_quality;
                    ImGui::SliderInt(get_locale_cstr("label.special_quality"),
                                     &strand.special_quality, 4, 64);
                    if (ImGui::IsItemActivated()) param_edits.activated = true;
                    if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                    if (old_sq != strand.special_quality) {
                        strand.mesh_dirty = true;
                        param_edits.value_changed = true;
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.special_quality"));
                }

                // 细分精度：引导曲线贝塞尔插值（所有类型共用）
                ImGui::SetNextItemWidth(160);
                int old_guide_subdiv = strand.guide_samples_per_segment;
                ImGui::SliderInt(get_locale_cstr("label.guide_subdiv"),
                                 &strand.guide_samples_per_segment, 4, 128);
                if (ImGui::IsItemActivated()) param_edits.activated = true;
                if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                if (old_guide_subdiv != strand.guide_samples_per_segment) {
                    strand.mesh_dirty = true;
                    param_edits.value_changed = true;
                }

                // Alpha wrap 修复参数（所有类型共用）
                ImGui::SetNextItemWidth(160);
                float old_repair_alpha = strand.repair_alpha;
                ImGui::SliderFloat(get_locale_cstr("label.alpha_wrap_alpha"),
                                   &strand.repair_alpha, 0.01f, 100.0f, "%.2f");
                if (ImGui::IsItemActivated()) param_edits.activated = true;
                if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                if (old_repair_alpha != strand.repair_alpha) {
                    strand.mesh_dirty = true;
                    param_edits.value_changed = true;
                }
                // ImGui::SameLine();
                ImGui::SetNextItemWidth(160);
                float old_repair_offset = strand.repair_offset;
                ImGui::SliderFloat(get_locale_cstr("label.alpha_wrap_offset"),
                                   &strand.repair_offset, 0.001f, 10.0f, "%.3f");
                if (ImGui::IsItemActivated()) param_edits.activated = true;
                if (ImGui::IsItemDeactivatedAfterEdit()) param_edits.deactivated_after_edit = true;
                if (old_repair_offset != strand.repair_offset) {
                    strand.mesh_dirty = true;
                    param_edits.value_changed = true;
                }

                // --- Push undo for parameter edits ---
                // begin_edit sets collision_edit_active=true, which blocks
                // push_undo_now during multi-frame drags. end_edit clears it
                // on release so the undo entry captures exactly one pre-edit
                // snapshot per drag gesture.
                if (param_edits.activated) {
                    begin_edit(item.id);
                }
                if (param_edits.deactivated_after_edit) {
                    end_edit(item.id, "Strand Parameter Edit");
                    strand.mesh_dirty = true;
                } else if (param_edits.value_changed && !item.collision_edit_active) {
                    // Discrete change (keyboard input, +/- buttons) — no
                    // drag session active, so push immediately.
                    push_undo_now(item.id, param_snapshot, "Strand Parameter Edit");
                    strand.mesh_dirty = true;
                }
            }

            if (strand_hovered) item.hovered_strand_uuid = strand.uuid;
            ImGui::PopID();
        }

        // 延迟删除
        if (delete_idx >= 0) {
            // 如果正在绘制/编辑被删除的发束，先停止
            if (item.active_guide_draw_strand == item.hair_strands[delete_idx].uuid) {
                item.guide_curve_drawing_active = false;
                item.active_guide_draw_strand.clear();
                show_guide_curve_window = false;
            }
            if (item.active_width_edit_strand == item.hair_strands[delete_idx].uuid) {
                item.width_editing_active = false;
                item.active_width_edit_strand.clear();
                show_width_editor_window = false;
            }
            if (item.active_section_edit_strand == item.hair_strands[delete_idx].uuid) {
                item.active_section_edit_strand.clear();
                show_cross_section_editor_window = false;
            }
            if (item.active_perpoint_section_edit_strand == item.hair_strands[delete_idx].uuid) {
                item.perpoint_section_editing_active = false;
                item.active_perpoint_section_edit_strand.clear();
                item.active_perpoint_section_edit_width_idx = -1;
                show_perpoint_section_editor_window = false;
            }
            push_undo_now(item.id, std::nullopt, "Delete Hair Strand");
            item.hair_strands.erase(item.hair_strands.begin() + delete_idx);
            for (auto& s : item.hair_strands) s.mesh_dirty = true;
        }

    }

    // --- Rename strand window (non-modal, so it doesn't block the UI) ---
    if (!pending_rename_strand_uuid.empty()) {
        ImGui::SetNextWindowSize(ImVec2(380, 120), ImGuiCond_Once);
        char win_title[128];
        snprintf(win_title, sizeof(win_title), "%s##RenameStrandWin",
                 get_locale_cstr("action.rename_strand"));
        bool rename_open = true;
        if (ImGui::Begin(win_title, &rename_open,
                         ImGuiWindowFlags_NoCollapse |
                             ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextUnformatted(
                get_locale_cstr("label.rename_strand_prompt"));

            ImGui::SetNextItemWidth(280);
            if (ImGui::IsWindowAppearing())
                ImGui::SetKeyboardFocusHere();
            ImGui::InputText("##rename_input", rename_buffer,
                             sizeof(rename_buffer));

            bool confirm_disabled = rename_buffer[0] == '\0';
            if (confirm_disabled) ImGui::BeginDisabled();
            if (ImGui::Button(get_locale_cstr("action.ok")) ||
                (!confirm_disabled &&
                 ImGui::IsKeyPressed(ImGuiKey_Enter))) {
                auto* s = item.find_strand_by_uuid(
                    pending_rename_strand_uuid);
                if (s) {
                    std::string old_uuid = s->uuid;
                    std::string new_uuid_val = generate_uuid();
                    s->name = rename_buffer;
                    item.rename_strand(old_uuid, new_uuid_val);
                    s->mesh_dirty = true;
                }
                pending_rename_strand_uuid.clear();
                rename_buffer[0] = '\0';
            }
            if (confirm_disabled) ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button(get_locale_cstr("action.cancel")) ||
                ImGui::IsKeyPressed(ImGuiKey_Escape)) {
                pending_rename_strand_uuid.clear();
                rename_buffer[0] = '\0';
            }
        }
        if (!rename_open) {
            pending_rename_strand_uuid.clear();
            rename_buffer[0] = '\0';
        }
        ImGui::End();
    }

    ImGui::End();
}

void RenderVoxelList::process_ai_export() {
    auto& s = ortho_state;
    if (s.ai_export_stage == 0)
        return;  // idle

    // Stage 1: Submit GPU blit + readback request
    if (s.ai_export_stage == 1) {
        if (!s.view_tex_ready || !bgfx::isValid(s.view_tex)) {
            std::cerr << "[ai_readback] View texture not ready" << std::endl;
            s.ai_export_stage = 0;
            s.ai_export_pending = false;
            return;
        }

        int res = s.render_resolution;
        if (!bgfx::isValid(s.ai_readback_tex)) {
            s.ai_readback_tex = bgfx::createTexture2D(
                static_cast<uint16_t>(res), static_cast<uint16_t>(res), false, 1,
                bgfx::TextureFormat::BGRA8,
                BGFX_TEXTURE_READ_BACK | BGFX_TEXTURE_BLIT_DST);
            s.ai_readback_buffer.resize(static_cast<size_t>(res) * res * 4);
        }

        if (bgfx::isValid(s.ai_readback_tex)) {
            bgfx::blit(kOrthoBlitView, s.ai_readback_tex, 0, 0,
                       s.view_tex, 0, 0,
                       static_cast<uint16_t>(res), static_cast<uint16_t>(res));
            bgfx::touch(kOrthoBlitView);
            bgfx::readTexture(s.ai_readback_tex, s.ai_readback_buffer.data());
            s.ai_readback_pending = true;
            s.ai_readback_frame_wait = 2;  // need 2 frames for GPU → CPU readback
            std::cout << "[ai_readback] Blit + readback submitted, res=" << res << std::endl;
        } else {
            std::cerr << "[ai_readback] Failed to create readback texture" << std::endl;
            s.ai_export_stage = 0;
            s.ai_export_pending = false;
            return;
        }

        s.ai_export_stage = 2;  // wait for next frames
        return;
    }

    // Stage 2: Wait for bgfx readback to complete (needs 2+ frames),
    // then push pixels to API cache — no files written to disk.
    if (s.ai_export_stage == 2) {
        if (s.ai_readback_frame_wait > 0) {
            s.ai_readback_frame_wait--;
            return;  // still waiting for GPU readback
        }

        if (s.ai_readback_pending && !s.ai_readback_buffer.empty()) {
            int res = s.render_resolution;

            // Convert BGRA → RGBA
            size_t pixel_count = static_cast<size_t>(res) * res;
            std::vector<uint8_t> rgba(pixel_count * 4);
            for (size_t i = 0; i < pixel_count; i++) {
                rgba[i * 4 + 0] = s.ai_readback_buffer[i * 4 + 2];  // R ← B
                rgba[i * 4 + 1] = s.ai_readback_buffer[i * 4 + 1];  // G ← G
                rgba[i * 4 + 2] = s.ai_readback_buffer[i * 4 + 0];  // B ← R
                rgba[i * 4 + 3] = s.ai_readback_buffer[i * 4 + 3];  // A ← A
            }

            // Push clean render to API cache.
            // Guide curve overlay is applied on-demand in /ortho/render handler.
            if (agent_server_ptr && agent_server_ptr->is_running())
                agent_server_ptr->setOrthoRenderData(rgba.data(), res, res);

            s.ai_readback_pending = false;
        }

        // Cleanup readback resources
        if (bgfx::isValid(s.ai_readback_tex)) {
            bgfx::destroy(s.ai_readback_tex);
            s.ai_readback_tex = BGFX_INVALID_HANDLE;
        }
        s.ai_readback_buffer.clear();
        s.ai_export_stage = 0;
        s.ai_export_pending = false;
    }
}

void RenderVoxelList::update_api_server_caches() {
    if (!agent_server_ptr || !agent_server_ptr->is_running()) return;

    // Overlay params are stored in a fixed 600px reference space.
    // Convert to render-pixel space for the API blend (render_resolution × render_resolution).
    constexpr float kRefDisplaySize = 600.0f;
    float render_scale = (float)ortho_state.render_resolution / kRefDisplaySize;
    agent_server_ptr->setOrthoOverlayParams(
        ortho_state.overlay_offset.x * render_scale,
        ortho_state.overlay_offset.y * render_scale,
        ortho_state.overlay_scale_x * render_scale,
        ortho_state.overlay_scale_y * render_scale,
        ortho_state.blend_ratio);
    agent_server_ptr->setOrthoOverlayActive(ortho_state.overlay_enabled);

    // Update overlay CPU data if we have it cached
    if (!overlay_cpu_rgba_.empty()) {
        agent_server_ptr->setOrthoOverlayData(overlay_cpu_rgba_.data(),
                                  overlay_cpu_w_, overlay_cpu_h_);
    }

    // Build state JSON for /state endpoint
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "version", 1);
    cJSON_AddNumberToObject(root, "viewport_size",
                            static_cast<double>(ortho_state.viewport_size));
    cJSON_AddNumberToObject(root, "resolution", ortho_state.render_resolution);

    cJSON* center_arr = cJSON_AddArrayToObject(root, "center");
    cJSON_AddItemToArray(center_arr, cJSON_CreateNumber(static_cast<double>(ortho_state._center.x)));
    cJSON_AddItemToArray(center_arr, cJSON_CreateNumber(static_cast<double>(ortho_state._center.y)));
    cJSON_AddItemToArray(center_arr, cJSON_CreateNumber(static_cast<double>(ortho_state._center.z)));

    cJSON* right_arr = cJSON_AddArrayToObject(root, "cam_right");
    cJSON_AddItemToArray(right_arr, cJSON_CreateNumber(static_cast<double>(ortho_state._cam_right.x)));
    cJSON_AddItemToArray(right_arr, cJSON_CreateNumber(static_cast<double>(ortho_state._cam_right.y)));
    cJSON_AddItemToArray(right_arr, cJSON_CreateNumber(static_cast<double>(ortho_state._cam_right.z)));

    cJSON* up_arr = cJSON_AddArrayToObject(root, "cam_up");
    cJSON_AddItemToArray(up_arr, cJSON_CreateNumber(static_cast<double>(ortho_state._cam_up.x)));
    cJSON_AddItemToArray(up_arr, cJSON_CreateNumber(static_cast<double>(ortho_state._cam_up.y)));
    cJSON_AddItemToArray(up_arr, cJSON_CreateNumber(static_cast<double>(ortho_state._cam_up.z)));

    cJSON* overlay = cJSON_AddObjectToObject(root, "overlay");
    cJSON_AddStringToObject(overlay, "image_path",
                            ortho_state.overlay_image_path.c_str());
    cJSON_AddNumberToObject(overlay, "img_width", ortho_state.overlay_img_width);
    cJSON_AddNumberToObject(overlay, "img_height", ortho_state.overlay_img_height);
    // Report overlay params in render-pixel space for API consumers
    cJSON_AddNumberToObject(overlay, "offset_x", static_cast<double>(ortho_state.overlay_offset.x * render_scale));
    cJSON_AddNumberToObject(overlay, "offset_y", static_cast<double>(ortho_state.overlay_offset.y * render_scale));
    cJSON_AddNumberToObject(overlay, "scale_x", static_cast<double>(ortho_state.overlay_scale_x * render_scale));
    cJSON_AddNumberToObject(overlay, "scale_y", static_cast<double>(ortho_state.overlay_scale_y * render_scale));
    cJSON_AddNumberToObject(overlay, "blend_ratio", static_cast<double>(ortho_state.blend_ratio));
    cJSON_AddBoolToObject(overlay, "enabled", ortho_state.overlay_enabled);
    cJSON_AddBoolToObject(overlay, "locked", ortho_state.overlay_locked);
    cJSON_AddNumberToObject(overlay, "canvas_display_size", static_cast<double>(ortho_state.canvas_display_size));
    cJSON_AddNumberToObject(overlay, "render_resolution", ortho_state.render_resolution);

    char* json_str = cJSON_Print(root);
    if (json_str) {
        agent_server_ptr->setOrthoState(json_str);
        cJSON_free(json_str);
    }
    cJSON_Delete(root);

    // Sync guide curve export flags.
    // The draw callback is installed once at startup (ui.hpp);
    // we only need to keep the boolean flags in sync here.
    agent_server_ptr->setGuideCurveFlags(
        ortho_state.export_show_guide_curves,
        ortho_state.export_color_code_strands);
}

}  // namespace sinriv::ui::render
