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
#include <set>
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
            wit->second->active_width_edit_strand = -1;
        }
        show_width_editor_window = false;
    }
    // 互斥：打开引导曲线窗口时关闭截面编辑器
    if (show_cross_section_editor_window) {
        auto sit = items.find(render_id);
        if (sit != items.end()) {
            sit->second->active_section_edit_strand = -1;
        }
        show_cross_section_editor_window = false;
    }
    // 互斥：打开引导曲线窗口时关闭逐点截面编辑器
    if (show_perpoint_section_editor_window) {
        auto pit = items.find(render_id);
        if (pit != items.end()) {
            pit->second->perpoint_section_editing_active = false;
            pit->second->active_perpoint_section_edit_strand = -1;
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
            it->second->active_guide_draw_strand = -1;
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
    int idx = item.active_guide_draw_strand;

    if (idx < 0 || idx >= static_cast<int>(item.hair_strands.size()) ||
        !item.guide_curve_drawing_active) {
        show_guide_curve_window = false;
        ImGui::End();
        return;
    }

    auto& strand = item.hair_strands[idx];
    ImGui::Text(get_locale_cstr("label.hair_strand"), idx + 1);
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
    }

    // Keyboard shortcuts (Ctrl+Z / Ctrl+Y)
    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Z)) {
        if (can_undo(item.id)) {
            undo(item.id);
            for (auto& s : item.hair_strands) s.mesh_dirty = true;
        }
    }
    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Y)) {
        if (can_redo(item.id)) {
            redo(item.id);
            for (auto& s : item.hair_strands) s.mesh_dirty = true;
        }
    }

    ImGui::Separator();
    ImGui::Text(get_locale_cstr("label.guide_curve_points"),
                static_cast<int>(strand.guide_points.size()));

    if (strand.guide_points.empty()) {
        ImGui::TextWrapped("%s",
                           get_locale_cstr("label.no_guide_points"));
    } else {
        // 可滚动的点列表（可编辑坐标），高度随窗口变化，
        // 负高度为底部的分隔线和清空按钮预留空间
        float bottom_reserve = ImGui::GetFrameHeightWithSpacing() +
                               ImGui::GetStyle().ItemSpacing.y;
        ImGui::BeginChild("GuidePointsList", ImVec2(0, -bottom_reserve), true);

        // 在循环前保存快照，用于撤销按钮+/-等即时修改
        auto before_edit = capture_snapshot(item);
        EditResult all_edits;
        int delete_point = -1;
        int swap_up = -1;
        int swap_down = -1;

        for (size_t pi = 0; pi < strand.guide_points.size(); ++pi) {
            ImGui::PushID(static_cast<int>(pi));

            char label_buf[64];
            snprintf(label_buf, sizeof(label_buf),
                     get_locale_cstr("label.guide_point"),
                     static_cast<int>(pi + 1));

            auto r = edit_vec3_stepper(label_buf, strand.guide_points[pi],
                                       0.5f, false, true);
            all_edits.activated |= r.activated;
            all_edits.deactivated_after_edit |= r.deactivated_after_edit;
            all_edits.value_changed |= r.value_changed;

            // --- 中心点方向移动控件（仅当中心点启用时显示）---
            if (item.show_addon_center) {
                vec3f to_center =
                    item.addon_center_point - strand.guide_points[pi];
                float dist = to_center.length();
                ImGui::SameLine();
                ImGui::TextDisabled("dist=%.2f", dist);

                if (dist > 0.0001f) {
                    vec3f dir = to_center / dist;

                    // 移动步长（静态变量，所有关键点共享，右键菜单中可调）
                    static float kp_move_step = 0.5f;

                    ImGui::SameLine();
                    if (ImGui::SmallButton("+")) {
                        strand.guide_points[pi] =
                            strand.guide_points[pi] + dir * kp_move_step;
                        all_edits.value_changed = true;
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "%s",
                            get_locale_cstr("tooltip.move_toward_center"));
                    if (ImGui::IsItemClicked(ImGuiMouseButton_Right))
                        ImGui::OpenPopup("kp_center_menu");
                    ImGui::SameLine();
                    if (ImGui::SmallButton("-")) {
                        strand.guide_points[pi] =
                            strand.guide_points[pi] - dir * kp_move_step;
                        all_edits.value_changed = true;
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "%s",
                            get_locale_cstr("tooltip.move_away_from_center"));
                    if (ImGui::IsItemClicked(ImGuiMouseButton_Right))
                        ImGui::OpenPopup("kp_center_menu");

                    // 右键菜单：直接编辑离中心距离与移动步长
                    if (ImGui::BeginPopup("kp_center_menu")) {
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
                            strand.guide_points[pi] =
                                item.addon_center_point - dir * new_dist;
                            // 历史记录由 activated/deactivated 在释放时创建，
                            // 拖动过程中只更新网格，避免每帧产生历史记录
                            strand.mesh_dirty = true;
                        }
                        ImGui::SetNextItemWidth(120);
                        ImGui::DragFloat(get_locale_cstr("label.move_step"),
                                         &kp_move_step, 0.01f, 0.01f, 10.0f,
                                         "%.2f");
                        ImGui::EndPopup();
                    }
                }
            }

            // 操作按钮放在同一行
            if (pi > 0) {
                ImGui::SameLine();
                if (ImGui::SmallButton("^")) {
                    swap_up = static_cast<int>(pi);
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "%s", get_locale_cstr("tooltip.move_point_up"));
            }
            if (pi < strand.guide_points.size() - 1) {
                ImGui::SameLine();
                if (ImGui::SmallButton("v")) {
                    swap_down = static_cast<int>(pi);
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "%s", get_locale_cstr("tooltip.move_point_down"));
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("X")) {
                delete_point = static_cast<int>(pi);
            }

            ImGui::PopID();
        }

        // 处理坐标编辑的撤销
        if (all_edits.activated) {
            begin_edit(item.id);
        }
        if (all_edits.deactivated_after_edit) {
            end_edit(item.id, "Guide Point Edit");
            strand.mesh_dirty = true;
        } else if (all_edits.value_changed && !item.collision_edit_active) {
            // Discrete change (keyboard input, +/- buttons) — no drag session
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

        ImGui::EndChild();
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
            git->second->active_guide_draw_strand = -1;
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
            it->second->active_width_edit_strand = -1;
            it->second->perpoint_section_editing_active = false;
            it->second->active_perpoint_section_edit_strand = -1;
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
    int idx = item.active_width_edit_strand;

    if (idx < 0 || idx >= static_cast<int>(item.hair_strands.size()) ||
        !item.width_editing_active) {
        show_width_editor_window = false;
        show_perpoint_section_editor_window = false;
        item.perpoint_section_editing_active = false;
        item.active_perpoint_section_edit_strand = -1;
        item.active_perpoint_section_edit_width_idx = -1;
        ImGui::End();
        return;
    }

    auto& strand = item.hair_strands[idx];
    ImGui::Text(get_locale_cstr("label.hair_strand"), idx + 1);
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

    // Keyboard shortcuts (Ctrl+Z / Ctrl+Y)
    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Z)) {
        if (can_undo(item.id)) {
            undo(item.id);
            for (auto& s : item.hair_strands) s.mesh_dirty = true;
        }
    }
    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Y)) {
        if (can_redo(item.id)) {
            redo(item.id);
            for (auto& s : item.hair_strands) s.mesh_dirty = true;
        }
    }

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
                    auto sample = item.sample_guide_curve_at(idx, wp.curve_id);
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
                    auto sample =
                        item.sample_guide_curve_at(idx, wp.curve_id);
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
                    auto sample = item.sample_guide_curve_at(idx, wp.curve_id);
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
                (item.active_perpoint_section_edit_strand == idx &&
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
                    item.active_perpoint_section_edit_strand = -1;
                    item.active_perpoint_section_edit_width_idx = -1;
                    show_perpoint_section_editor_window = false;
                } else {
                    // Close global section editor (mutual exclusion)
                    if (show_cross_section_editor_window) {
                        item.active_section_edit_strand = -1;
                        show_cross_section_editor_window = false;
                    }
                    item.perpoint_section_editing_active = true;
                    item.active_perpoint_section_edit_strand = idx;
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
            if (item.active_perpoint_section_edit_strand == idx &&
                item.active_perpoint_section_edit_width_idx == delete_wp) {
                item.perpoint_section_editing_active = false;
                item.active_perpoint_section_edit_strand = -1;
                item.active_perpoint_section_edit_width_idx = -1;
                show_perpoint_section_editor_window = false;
            } else if (item.active_perpoint_section_edit_strand == idx &&
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
            strand.name = "Strand " + std::to_string(item.hair_strands.size() + 1);
            strand.expanded = true;
            item.hair_strands.push_back(strand);
        }

        ImGui::Separator();

        // 发束列表
        int delete_idx = -1;
        for (size_t i = 0; i < item.hair_strands.size(); ++i) {
            auto& strand = item.hair_strands[i];
            ImGui::PushID(static_cast<int>(i));

            char header_label[64];
            snprintf(header_label, sizeof(header_label),
                     get_locale_cstr("label.hair_strand"),
                     static_cast<int>(i + 1));
            int header_flags = ImGuiTreeNodeFlags_AllowOverlap;
            if (strand.expanded)
                header_flags |= ImGuiTreeNodeFlags_DefaultOpen;
            bool expanded = ImGui::CollapsingHeader(header_label, header_flags);
            strand.expanded = expanded;

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
                    (item.active_guide_draw_strand == static_cast<int>(i) &&
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
                        item.active_guide_draw_strand = -1;
                        show_guide_curve_window = false;
                    } else {
                        // 互斥：打开引导曲线时关闭宽度编辑器
                        if (item.width_editing_active) {
                            item.width_editing_active = false;
                            item.active_width_edit_strand = -1;
                            show_width_editor_window = false;
                        }
                        item.guide_curve_drawing_active = true;
                        item.active_guide_draw_strand = static_cast<int>(i);
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
                        (item.active_width_edit_strand == static_cast<int>(i) &&
                            item.width_editing_active);
                    if (ImGui::Button(
                            is_width_editing_popup
                                ? get_locale_cstr("action.stop_width_edit")
                                : get_locale_cstr("action.edit_width"))) {
                        if (is_width_editing_popup) {
                            item.width_editing_active = false;
                            item.active_width_edit_strand = -1;
                            show_width_editor_window = false;
                        } else {
                            if (item.guide_curve_drawing_active) {
                                item.guide_curve_drawing_active = false;
                                item.active_guide_draw_strand = -1;
                                show_guide_curve_window = false;
                            }
                            item.width_editing_active = true;
                            item.active_width_edit_strand = static_cast<int>(i);
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
                        (item.active_section_edit_strand == static_cast<int>(i));
                    if (ImGui::MenuItem(
                            is_section_editing_popup
                                ? get_locale_cstr("action.stop_edit_section")
                                : get_locale_cstr("action.edit_section"))) {
                        if (is_section_editing_popup) {
                            item.active_section_edit_strand = -1;
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
                                    static_cast<int>(i);
                                show_cross_section_editor_window = true;
                            }
                        }
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.edit_section"));
                    if (!is_normal) ImGui::EndDisabled();

                    ImGui::Separator();

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

            ImGui::PopID();
        }

        // 延迟删除
        if (delete_idx >= 0) {
            // 如果正在绘制/编辑被删除的发束，先停止
            if (item.active_guide_draw_strand == delete_idx) {
                item.guide_curve_drawing_active = false;
                item.active_guide_draw_strand = -1;
                show_guide_curve_window = false;
            } else if (item.active_guide_draw_strand > delete_idx) {
                item.active_guide_draw_strand--;
            }
            if (item.active_width_edit_strand == delete_idx) {
                item.width_editing_active = false;
                item.active_width_edit_strand = -1;
                show_width_editor_window = false;
            } else if (item.active_width_edit_strand > delete_idx) {
                item.active_width_edit_strand--;
            }
            if (item.active_section_edit_strand == delete_idx) {
                item.active_section_edit_strand = -1;
                show_cross_section_editor_window = false;
            } else if (item.active_section_edit_strand > delete_idx) {
                item.active_section_edit_strand--;
            }
            // Clean up per-point section editor
            if (item.active_perpoint_section_edit_strand == delete_idx) {
                item.perpoint_section_editing_active = false;
                item.active_perpoint_section_edit_strand = -1;
                item.active_perpoint_section_edit_width_idx = -1;
                show_perpoint_section_editor_window = false;
            } else if (item.active_perpoint_section_edit_strand >
                       delete_idx) {
                item.active_perpoint_section_edit_strand--;
            }
            push_undo_now(item.id, std::nullopt, "Delete Hair Strand");
            item.hair_strands.erase(item.hair_strands.begin() + delete_idx);
            for (auto& s : item.hair_strands) s.mesh_dirty = true;
        }

    }

    ImGui::End();
}

void RenderVoxelList::render_hairline_plane_window() {
    if (!show_addon_window)
        return;
    if (!show_hairline_plane_window)
        return;

    // Mutual exclusion with other editor windows (but NOT angle config)
    if (show_guide_curve_window) {
        auto git = items.find(render_id);
        if (git != items.end()) {
            git->second->guide_curve_drawing_active = false;
            git->second->active_guide_draw_strand = -1;
        }
        show_guide_curve_window = false;
    }
    if (show_width_editor_window) {
        auto wit = items.find(render_id);
        if (wit != items.end()) {
            wit->second->width_editing_active = false;
            wit->second->active_width_edit_strand = -1;
        }
        show_width_editor_window = false;
    }
    // NOTE: no longer close show_angle_config_window

    // 初始位置：中心点位于屏幕中心
    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Once, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(320, 180), ImGuiCond_Once);
    bool window_open = true;
    if (!ImGui::Begin(get_locale_cstr("window.auto_width"), &window_open)) {
        ImGui::End();
        return;
    }

    if (!window_open) {
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->hairline_point_picking_active = false;
        }
        show_hairline_plane_window = false;
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

    if (!item.hairline_plane_enabled) {
        ImGui::TextDisabled("%s",
            get_locale_cstr("label.hairline_plane_disabled_hint"));
        ImGui::End();
        return;
    }

    ImGui::Separator();

    // ---- Scale factor ----
    ImGui::SetNextItemWidth(200);
    ImGui::DragFloat(get_locale_cstr("label.hairline_spindle_scale"),
                     &item.hairline_spindle_scale,
                     0.01f, 0.0f, 1.0f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.hairline_spindle_scale"));

    ImGui::Separator();

    // ---- Apply button ----
    if (ImGui::Button(get_locale_cstr("action.apply_hairline_spindle"),
                      ImVec2(-1, 0))) {
        push_undo_now(item.id, std::nullopt, "Apply Hairline Spindle");
        item.apply_hairline_spindle();
        for (auto& s : item.hair_strands) s.mesh_dirty = true;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s",
            get_locale_cstr("tooltip.apply_hairline_spindle"));
    }

    ImGui::End();
}

// ============================================================
// Semantic coordinate anchor point definitions (from api-reference.md)
// X-axis (Y=0): left-to-right lateral anchors
// Y-axis (X=0): front-to-back midline anchors
// ============================================================
struct AnchorPoint { int x; int y; const char* name_en; const char* name_zh; };
static const AnchorPoint kAnchorPoints[] = {
    // X axis (Y=0) — lateral cross-section
    {-10,0,"Posterior midline (L)","后正中线（左）"},
    {-9,0,"Lateral occipital (L)","枕骨外侧（左）"},
    {-8,0,"Mastoid process (L)","乳突（左）"},
    {-7,0,"Helix outer edge (L)","耳轮外缘（左）"},
    {-6,0,"Tragus (L)","耳屏（左）"},
    {-5,0,"Zygomatic arch (L)","颧弓最外侧（左）"},
    {-4,0,"Outer canthus (L)","外眼角（左）"},
    {-3,0,"Pupil center (L)","瞳孔中心（左）"},
    {-2,0,"Inner canthus (L)","内眼角（左）"},
    {-1,0,"Ala of nose (L)","鼻翼外缘（左）"},
    {0,0,"Midline / Midsagittal","鼻中线 / 前正中线"},
    {1,0,"Ala of nose (R)","鼻翼外缘（右）"},
    {2,0,"Inner canthus (R)","内眼角（右）"},
    {3,0,"Pupil center (R)","瞳孔中心（右）"},
    {4,0,"Outer canthus (R)","外眼角（右）"},
    {5,0,"Zygomatic arch (R)","颧弓最外侧（右）"},
    {6,0,"Tragus (R)","耳屏（右）"},
    {7,0,"Helix outer edge (R)","耳轮外缘（右）"},
    {8,0,"Mastoid process (R)","乳突（右）"},
    {9,0,"Lateral occipital (R)","枕骨外侧（右）"},
    {10,0,"Posterior midline (R)","后正中线（右）"},
    // Y axis front (X=0) — frontal midline
    {0,1,"Forehead hairline","额头发际线"},
    {0,2,"Upper brow","眉毛上缘"},
    {0,3,"Lower brow","眉毛下缘"},
    {0,4,"Nasion","鼻根"},
    {0,5,"Upper eye","眼上缘"},
    {0,6,"Lower eye","眼下缘"},
    {0,7,"Nose tip","鼻尖"},
    {0,8,"Nasal base","鼻底"},
    {0,9,"Upper lip","嘴唇上缘"},
    {0,10,"Oral fissure","口裂"},
    {0,11,"Lower lip","嘴唇下缘"},
    {0,12,"Chin (Menton)","颏部"},
    {0,13,"Mandible border","下颌下缘"},
    {0,14,"Anterior neck","颈前部"},
    // Y axis back (X=0) — posterior midline
    {0,-1,"Coronal suture","冠状缝附近"},
    {0,-2,"Parietal center","顶骨中央"},
    {0,-3,"Lambda","顶枕点"},
    {0,-4,"Upper occipital","枕骨上部"},
    {0,-5,"External Occipital Protuberance","枕外隆凸"},
    {0,-6,"Superior nuchal line","上项线"},
    {0,-7,"Lower occipital","枕骨下部"},
    {0,-8,"Posterior hairline","后发际线"},
    {0,-9,"Posterior neck junction","颈后连接处"},
    {0,-10,"Lower posterior neck","颈后下部"},
};

static const AnchorPoint* find_anchor(int x, int y) {
    for (const auto& a : kAnchorPoints)
        if (a.x == x && a.y == y) return &a;
    return nullptr;
}

static const char* get_anchor_name(int x, int y) {
    auto* a = find_anchor(x, y);
    return a ? (get_system_language() == "zh" ? a->name_zh : a->name_en) : nullptr;
}

// Always returns Chinese name (used in table display)
static const char* get_anchor_name_cn(int x, int y) {
    auto* a = find_anchor(x, y);
    return a ? a->name_zh : nullptr;
}

// ============================================================
// Cross-validation: ensure no grid lines cross
// ============================================================
static bool validate_angle_grid(
    const std::map<std::pair<float, float>, HairAngleEntry>& config,
    int proposed_x, int proposed_y,
    float new_theta, float new_phi)
{
    // Build temporary config with the proposed value
    auto tmp = config;
    tmp[{static_cast<float>(proposed_x), static_cast<float>(proposed_y)}] =
        HairAngleEntry{new_theta, new_phi};

    // 1. Theta monotonicity per row (fixed Y)
    for (int y = -10; y <= 14; ++y) {
        std::vector<std::pair<int, float>> row;  // (X, theta)
        for (int x = -10; x <= 10; ++x) {
            auto it = tmp.find({static_cast<float>(x), static_cast<float>(y)});
            if (it != tmp.end()) {
                row.push_back({x, it->second.theta});
            }
        }
        if (row.size() < 2) continue;
        bool increasing = true, decreasing = true;
        for (size_t i = 1; i < row.size(); ++i) {
            if (row[i].second <= row[i-1].second) increasing = false;
            if (row[i].second >= row[i-1].second) decreasing = false;
        }
        if (!increasing && !decreasing) return false;
    }

    // 2. Phi monotonicity per column (fixed X)
    for (int x = -10; x <= 10; ++x) {
        std::vector<std::pair<int, float>> col;  // (Y, phi)
        for (int y = -10; y <= 14; ++y) {
            auto it = tmp.find({static_cast<float>(x), static_cast<float>(y)});
            if (it != tmp.end()) {
                col.push_back({y, it->second.phi});
            }
        }
        if (col.size() < 2) continue;
        bool increasing = true, decreasing = true;
        for (size_t i = 1; i < col.size(); ++i) {
            if (col[i].second <= col[i-1].second) increasing = false;
            if (col[i].second >= col[i-1].second) decreasing = false;
        }
        if (!increasing && !decreasing) return false;
    }

    // 3. Midline separation: X=0 and X=±10 must not have overlapping theta
    for (int y = -10; y <= 14; ++y) {
        auto it0  = tmp.find({0.0f, static_cast<float>(y)});
        auto it10 = tmp.find({10.0f, static_cast<float>(y)});
        auto itm10 = tmp.find({-10.0f, static_cast<float>(y)});
        float t0   = (it0 != tmp.end()) ? it0->second.theta : std::numeric_limits<float>::quiet_NaN();
        float t10  = (it10 != tmp.end()) ? it10->second.theta : std::numeric_limits<float>::quiet_NaN();
        float tm10 = (itm10 != tmp.end()) ? itm10->second.theta : std::numeric_limits<float>::quiet_NaN();
        if (!std::isnan(t0) && !std::isnan(t10)) {
            if (std::abs(t0 - t10) < 10.0f) return false;
        }
        if (!std::isnan(t0) && !std::isnan(tm10)) {
            if (std::abs(t0 - tm10) < 10.0f) return false;
        }
    }

    return true;
}

// ============================================================
// Inverse of spherical_to_dir: convert world-space direction →
// (theta, phi) using the same N/U/V frame.
// Returns false when direction is parallel to north pole (theta undefined).
// ============================================================
static bool dir_to_spherical(
    const vec3f& world_dir,
    const vec3f& north_pole,
    const vec3f& front_reference,
    float& out_theta_deg,
    float& out_phi_deg)
{
    vec3f d = world_dir.normalize();
    vec3f N = north_pole.normalize();

    // phi = asin(dot(d, N))
    float dot_d_n = d.dot(N);
    dot_d_n = std::max(-1.0f, std::min(1.0f, dot_d_n));
    float phi_rad = std::asin(dot_d_n);
    constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;
    out_phi_deg = phi_rad * kRadToDeg;

    // Project direction onto equatorial plane
    vec3f d_uv = d - N * dot_d_n;
    float uv_len2 = d_uv.length2();
    constexpr float kEps = 1e-10f;

    if (uv_len2 < kEps) {
        // Direction is parallel to north pole → theta is undefined
        out_theta_deg = 0.0f;
        return false;
    }

    // Build U, V frame (same as spherical_to_dir)
    vec3f F = front_reference.normalize();
    float f_dot_n = F.dot(N);
    vec3f V = F - N * f_dot_n;
    float v_len2 = V.length2();

    if (v_len2 < 1e-10f) {
        vec3f A = (std::abs(N.z) < 0.99f)
            ? vec3f(0.0f, 0.0f, 1.0f)
            : vec3f(1.0f, 0.0f, 0.0f);
        V = A - N * A.dot(N);
        v_len2 = V.length2();
    }
    V = V / std::sqrt(v_len2);
    vec3f U = cross(N, V);

    // sin(theta) = dot(d_uv_norm, U), cos(theta) = dot(d_uv_norm, V)
    float inv_len = 1.0f / std::sqrt(uv_len2);
    float sin_t = d_uv.dot(U) * inv_len;
    float cos_t = d_uv.dot(V) * inv_len;
    float theta_rad = std::atan2(sin_t, cos_t);
    out_theta_deg = theta_rad * kRadToDeg;

    return true;
}

// ============================================================
// Semantic coordinate angle config editor window
// ============================================================
void RenderVoxelList::render_angle_config_window() {
    if (!show_addon_window) return;
    if (!show_angle_config_window) return;

    // Mutual exclusion: close sibling windows (but NOT auto-width/hairline plane)
    if (show_guide_curve_window) {
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->guide_curve_drawing_active = false;
            it->second->active_guide_draw_strand = -1;
        }
        show_guide_curve_window = false;
    }
    if (show_width_editor_window) {
        auto wit = items.find(render_id);
        if (wit != items.end()) {
            wit->second->width_editing_active = false;
            wit->second->active_width_edit_strand = -1;
        }
        show_width_editor_window = false;
    }
    if (show_cross_section_editor_window) {
        auto sit = items.find(render_id);
        if (sit != items.end()) sit->second->active_section_edit_strand = -1;
        show_cross_section_editor_window = false;
    }
    if (show_perpoint_section_editor_window) {
        auto pit = items.find(render_id);
        if (pit != items.end()) {
            pit->second->perpoint_section_editing_active = false;
            pit->second->active_perpoint_section_edit_strand = -1;
            pit->second->active_perpoint_section_edit_width_idx = -1;
        }
        show_perpoint_section_editor_window = false;
    }
    // NOTE: no longer close show_hairline_plane_window — they can coexist

    // 初始位置：中心点位于屏幕中心
    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Once, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(900, 680), ImGuiCond_Once);
    bool window_open = true;
    if (!ImGui::Begin(get_locale_cstr("window.angle_config"), &window_open)) {
        ImGui::End();
        return;
    }

    if (!window_open) {
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->angle_config_editing_x =
                RenderVoxelItem::kAngleConfigSentinel;
            it->second->angle_config_editing_y =
                RenderVoxelItem::kAngleConfigSentinel;
        }
        show_angle_config_window = false;
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
    auto& item = *item_it->second;

    // Auto-enable center point and hairline plane when this window is open
    item.show_addon_center = true;
    item.hairline_plane_enabled = true;

    constexpr int kXMin = -10, kXMax = 10;
    constexpr int kYMin = -10, kYMax = 14;
    constexpr int kSentinel = RenderVoxelItem::kAngleConfigSentinel;

    // ================================================================
    // Axis table rendering lambda (used in right panel below)
    // ================================================================
    auto render_axis_table = [&](const char* title, bool is_x_axis) {
        ImGui::TextUnformatted(title);

        ImGuiTableFlags tbl_flags = ImGuiTableFlags_Borders |
            ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
            ImGuiTableFlags_SizingFixedFit;

        if (!ImGui::BeginTable(is_x_axis ? "##AxisTableX" : "##AxisTableY",
                5, tbl_flags, ImVec2(0, 0))) {
            return;
        }
        ImGui::TableSetupColumn(get_locale_cstr("label.angle_id"),
            ImGuiTableColumnFlags_WidthFixed, 30.0f);
        ImGui::TableSetupColumn(get_locale_cstr("label.angle_organ"),
            ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn(get_locale_cstr("label.angle_theta"),
            ImGuiTableColumnFlags_WidthFixed, 70.0f);
        ImGui::TableSetupColumn(get_locale_cstr("label.angle_phi"),
            ImGuiTableColumnFlags_WidthFixed, 70.0f);
        ImGui::TableSetupColumn("",
            ImGuiTableColumnFlags_WidthFixed, 100.0f);
        ImGui::TableHeadersRow();

        int row_id = 0;
        int del_x = kSentinel, del_y = kSentinel;

        int lo = is_x_axis ? kXMin : kYMax;
        int hi = is_x_axis ? kXMax : kYMin;
        int step = is_x_axis ? 1 : -1;  // Y goes top-down

        for (int v = lo; (is_x_axis ? v <= hi : v >= hi); v += step) {
            int x = is_x_axis ? v : 0;
            int y = is_x_axis ? 0 : v;

            auto it = item.hair_angle_config.find(
                {static_cast<float>(x), static_cast<float>(y)});
            bool configured = (it != item.hair_angle_config.end());
            ++row_id;

            auto open_edit = [&](float def_theta, float def_phi) {
                item.angle_config_editing_x = x;
                item.angle_config_editing_y = y;
                item.angle_config_preview_theta = def_theta;
                item.angle_config_preview_phi = def_phi;
            };

            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%d", row_id);

            // Organ name
            ImGui::TableSetColumnIndex(1);
            const char* name_cn = get_anchor_name_cn(x, y);
            if (name_cn)
                ImGui::Text("%s  (%+d)", name_cn, v);
            else
                ImGui::Text("(%+d)", v);

            // Theta
            ImGui::TableSetColumnIndex(2);
            if (configured)
                ImGui::Text("%.0f°", it->second.theta);
            else {
                ImGui::PushStyleColor(ImGuiCol_Text,
                    ImVec4(0.45f, 0.45f, 0.45f, 1.0f));
                ImGui::TextUnformatted("-");
                ImGui::PopStyleColor();
            }

            // Phi
            ImGui::TableSetColumnIndex(3);
            if (configured)
                ImGui::Text("%.0f°", it->second.phi);
            else {
                ImGui::PushStyleColor(ImGuiCol_Text,
                    ImVec4(0.45f, 0.45f, 0.45f, 1.0f));
                ImGui::TextUnformatted("-");
                ImGui::PopStyleColor();
            }

            // Action buttons
            ImGui::TableSetColumnIndex(4);
            char btn_id[48];
            if (configured) {
                snprintf(btn_id, sizeof(btn_id), "%s##e%d_%d",
                    get_locale_cstr("action.angle_edit"), x, y);
                if (ImGui::SmallButton(btn_id))
                    open_edit(it->second.theta, it->second.phi);
                ImGui::SameLine();
                snprintf(btn_id, sizeof(btn_id), "%s##d%d_%d",
                    get_locale_cstr("action.angle_delete"), x, y);
                if (ImGui::SmallButton(btn_id)) {
                    del_x = x;
                    del_y = y;
                }
            } else {
                snprintf(btn_id, sizeof(btn_id), "%s##a%d_%d",
                    get_locale_cstr("action.angle_add_entry"), x, y);
                if (ImGui::SmallButton(btn_id))
                    open_edit((x == 0) ? 0.0f : x * 9.0f, y * 6.0f);
            }
        }

        ImGui::EndTable();

        // Handle delete outside table
        if (del_x != kSentinel) {
            push_undo_now(item.id, std::nullopt, "Angle Config Delete");
            item.hair_angle_config.erase(
                {static_cast<float>(del_x), static_cast<float>(del_y)});
            for (auto& s : item.hair_strands) s.mesh_dirty = true;
            if (item.angle_config_editing_x == del_x &&
                item.angle_config_editing_y == del_y) {
                item.angle_config_editing_x = kSentinel;
                item.angle_config_editing_y = kSentinel;
            }
        }
    };

    // ================================================================
    // Two-column layout: left = controls (fixed width),
    //                     right = axis tables (fill remaining)
    // ================================================================
    const float kLeftPanelWidth = 300.0f;
    if (ImGui::BeginTable("##AngleConfigLayout", 2,
            ImGuiTableFlags_Resizable)) {
        ImGui::TableSetupColumn("##LeftCol",
            ImGuiTableColumnFlags_WidthFixed, kLeftPanelWidth);
        ImGui::TableSetupColumn("##RightCol",
            ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableNextRow();

        // ---- Left panel: controls ----
        ImGui::TableSetColumnIndex(0);
        if (ImGui::BeginChild("##AngleConfigLeft", ImVec2(0, 0), false)) {

            // ---- Center Point ----
            if (ImGui::CollapsingHeader(
                    get_locale_cstr("label.addon_center_point"),
                    ImGuiTreeNodeFlags_DefaultOpen)) {
                auto cp_edit = edit_vec3_stepper(
                    get_locale_cstr("label.addon_center_point"),
                    item.addon_center_point, 0.1f);
                if (cp_edit.activated) begin_edit(item.id);
                if (cp_edit.deactivated_after_edit) {
                    end_edit(item.id, "Center Point Edit");
                    for (auto& s : item.hair_strands) s.mesh_dirty = true;
                } else if (cp_edit.value_changed) {
                    push_undo_now(item.id, std::nullopt, "Center Point Edit");
                    for (auto& s : item.hair_strands) s.mesh_dirty = true;
                }
            }

            // ---- Hairline Plane ----
            if (ImGui::CollapsingHeader(
                    get_locale_cstr("label.hairline_plane"),
                    ImGuiTreeNodeFlags_DefaultOpen)) {
                const char* plane_mode_items[] = {
                    get_locale_cstr("label.hairline_y_plane"),
                    get_locale_cstr("label.hairline_3point_plane"),
                };
                int plane_mode = item.hairline_plane_use_y ? 0 : 1;
                ImGui::SetNextItemWidth(120);
                if (ImGui::Combo(get_locale_cstr("label.hairline_plane_mode"),
                                &plane_mode, plane_mode_items, 2)) {
                    push_undo_now(item.id, std::nullopt,
                                  "Hairline Plane Mode");
                    item.hairline_plane_use_y = (plane_mode == 0);
                }

                if (item.hairline_plane_use_y) {
                    float old_y = item.hairline_plane_y;
                    ImGui::SetNextItemWidth(160);
                    ImGui::DragFloat(get_locale_cstr("label.hairline_y"),
                                     &item.hairline_plane_y, 0.1f);
                    if (ImGui::IsItemActivated()) begin_edit(item.id);
                    if (ImGui::IsItemDeactivatedAfterEdit()) {
                        end_edit(item.id, "Hairline Y Edit");
                        for (auto& s : item.hair_strands) s.mesh_dirty = true;
                    } else if (old_y != item.hairline_plane_y) {
                        push_undo_now(item.id, std::nullopt,
                                      "Hairline Y Edit");
                    }
                    ImGui::TextDisabled("%s",
                        get_locale_cstr("label.hairline_preview_triangle"));
                } else {
                    bool pt_activated = false;
                    bool pt_deactivated = false;
                    bool pt_changed = false;

                    for (int pi = 0; pi < 3; ++pi) {
                        ImGui::PushID(pi);
                        char label_buf[64];
                        snprintf(label_buf, sizeof(label_buf),
                                 get_locale_cstr("label.hairline_point"),
                                 pi + 1);
                        auto r = edit_vec3_stepper(label_buf,
                            item.hairline_plane_points[pi], 0.1f);
                        pt_activated |= r.activated;
                        pt_deactivated |= r.deactivated_after_edit;
                        pt_changed |= r.value_changed;

                        ImGui::SameLine();
                        bool is_picking =
                            item.hairline_point_picking_active &&
                            item.hairline_picking_point_index == pi;
                        if (is_picking) {
                            ImGui::PushStyleColor(ImGuiCol_Button,
                                ImVec4(0.2f, 0.5f, 1.0f, 1.0f));
                        }
                        char pick_label[64];
                        snprintf(pick_label, sizeof(pick_label),
                                 is_picking
                                     ? get_locale_cstr("action.picking")
                                     : get_locale_cstr("action.pick_point"),
                                 pi + 1);
                        if (ImGui::SmallButton(pick_label)) {
                            if (is_picking) {
                                item.hairline_point_picking_active = false;
                            } else {
                                item.hairline_point_picking_active = true;
                                item.hairline_picking_point_index = pi;
                            }
                        }
                        if (is_picking) ImGui::PopStyleColor();
                        if (ImGui::IsItemHovered() && !is_picking)
                            ImGui::SetTooltip("%s",
                                get_locale_cstr("tooltip.pick_point"));
                        ImGui::PopID();
                    }

                    if (pt_activated) begin_edit(item.id);
                    if (pt_deactivated) {
                        end_edit(item.id, "Hairline Points Edit");
                        for (auto& s : item.hair_strands) s.mesh_dirty = true;
                    } else if (pt_changed) {
                        push_undo_now(item.id, std::nullopt,
                                      "Hairline Points Edit");
                    }

                    // Degeneracy warning
                    {
                        const auto& p0 = item.hairline_plane_points[0];
                        const auto& p1 = item.hairline_plane_points[1];
                        const auto& p2 = item.hairline_plane_points[2];
                        vec3f e1{p1.x - p0.x, p1.y - p0.y, p1.z - p0.z};
                        vec3f e2{p2.x - p0.x, p2.y - p0.y, p2.z - p0.z};
                        float area = std::sqrt(
                            (e1.y*e2.z - e1.z*e2.y)*(e1.y*e2.z - e1.z*e2.y) +
                            (e1.z*e2.x - e1.x*e2.z)*(e1.z*e2.x - e1.x*e2.z) +
                            (e1.x*e2.y - e1.y*e2.x)*(e1.x*e2.y - e1.y*e2.x));
                        if (area < 1e-6f) {
                            ImGui::TextColored(
                                ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "%s",
                                get_locale_cstr("label.hairline_degenerate"));
                        }
                    }
                }
            }

            // ---- North Pole & Front Reference ----
            if (ImGui::CollapsingHeader(
                    get_locale_cstr("label.spherical_frame"),
                    ImGuiTreeNodeFlags_DefaultOpen)) {
                auto np_edit = edit_vec3_stepper(
                    get_locale_cstr("label.north_pole"),
                    item.hair_north_pole, 0.1f);
                if (np_edit.activated) begin_edit(item.id);
                if (np_edit.deactivated_after_edit) {
                    end_edit(item.id, "North Pole Edit");
                    for (auto& s : item.hair_strands) s.mesh_dirty = true;
                } else if (np_edit.value_changed) {
                    push_undo_now(item.id, std::nullopt, "North Pole Edit");
                    for (auto& s : item.hair_strands) s.mesh_dirty = true;
                }

                auto fr_edit = edit_vec3_stepper(
                    get_locale_cstr("label.front_reference"),
                    item.hair_front_reference, 0.1f);
                if (fr_edit.activated) begin_edit(item.id);
                if (fr_edit.deactivated_after_edit) {
                    end_edit(item.id, "Front Reference Edit");
                    for (auto& s : item.hair_strands) s.mesh_dirty = true;
                } else if (fr_edit.value_changed) {
                    push_undo_now(item.id, std::nullopt,
                                  "Front Reference Edit");
                    for (auto& s : item.hair_strands) s.mesh_dirty = true;
                }
            }

        }
        ImGui::EndChild(); // AngleConfigLeft

        // ---- Right panel: axis tables (tabbed) ----
        ImGui::TableSetColumnIndex(1);
        if (ImGui::BeginChild("##AngleConfigRight", ImVec2(0, 0), false)) {
            if (ImGui::BeginTabBar("##AxisTabs")) {
                if (ImGui::BeginTabItem(
                        get_locale_cstr("label.angle_x_axis"))) {
                    render_axis_table(
                        get_locale_cstr("label.angle_x_axis"), true);
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem(
                        get_locale_cstr("label.angle_y_axis"))) {
                    render_axis_table(
                        get_locale_cstr("label.angle_y_axis"), false);
                    ImGui::EndTabItem();
                }
                ImGui::EndTabBar();
            }
        }
        ImGui::EndChild(); // AngleConfigRight

        ImGui::EndTable(); // AngleConfigLayout
    }

    // ================================================================
    // Direct model-click pick: when editor popup is active and user
    // clicked the 3D model, compute (theta, phi) from the picked point.
    // The popup may be auto-closed by the click (ImGui popup behavior),
    // so we process the pick BEFORE calling OpenPopup to reopen it.
    // ================================================================
    if (item.angle_config_editing_x != kSentinel && mouse_world_pos_picked) {
        vec3f pick_dir = mouse_world_pos - item.addon_center_point;
        float dir_len2 = pick_dir.length2();
        if (dir_len2 > 0.0001f) {
            pick_dir = pick_dir / std::sqrt(dir_len2);
            float pick_theta, pick_phi;
            dir_to_spherical(pick_dir, item.hair_north_pole,
                             item.hair_front_reference,
                             pick_theta, pick_phi);
            item.angle_config_preview_theta = pick_theta;
            item.angle_config_preview_phi = pick_phi;
        }
        mouse_world_pos_picked = false; // consume the event
    }

    // ================================================================
    // Editor window — uses a regular ImGui window (not a popup) so
    // 3D model clicks pass through for direction picking.
    // ================================================================
    if (item.angle_config_editing_x != kSentinel) {
        ImVec2 win_pos = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(win_pos, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(400, 240), ImGuiCond_Appearing);

        char win_title[128];
        snprintf(win_title, sizeof(win_title), "%s##AngleConfigEdit",
                 get_locale_cstr("label.angle_edit_title"));
        bool edit_open = true;
        if (ImGui::Begin(win_title, &edit_open)) {
            int ex = item.angle_config_editing_x;
            int ey = item.angle_config_editing_y;
            if (!edit_open || ex == kSentinel) {
                item.angle_config_editing_x = kSentinel;
                item.angle_config_editing_y = kSentinel;
                ImGui::End();
                ImGui::End();  // outer window
                return;
            }

            auto cfg_it = item.hair_angle_config.find(
                {static_cast<float>(ex), static_cast<float>(ey)});
            bool is_new = (cfg_it == item.hair_angle_config.end());

            const char* anchor = get_anchor_name_cn(ex, ey);
            if (anchor)
                ImGui::Text("%s (%+d, %+d)  %s",
                    anchor, ex, ey, is_new ? "(new)" : "");
            else
                ImGui::Text("%s: (%+d, %+d)  %s",
                    get_locale_cstr("label.angle_edit_title"), ex, ey,
                    is_new ? "(new)" : "");
            ImGui::Separator();

            float& theta = item.angle_config_preview_theta;
            float& phi    = item.angle_config_preview_phi;
            ImGui::DragFloat(get_locale_cstr("label.angle_theta"), &theta,
                1.0f, -180.0f, 180.0f, "%.1f deg");
            ImGui::DragFloat(get_locale_cstr("label.angle_phi"), &phi,
                1.0f, -90.0f, 90.0f, "%.1f deg");

            // Show direction preview
            auto dir = sinriv::kigstudio::agent::spherical_to_dir(theta, phi,
                item.hair_north_pole, item.hair_front_reference);
            ImGui::Text("%s: [%.2f, %.2f, %.2f]",
                get_locale_cstr("label.angle_direction"),
                dir.x, dir.y, dir.z);

            // Hint: click on model to pick
            ImGui::TextDisabled("%s",
                get_locale_cstr("label.angle_picking"));

            ImGui::Separator();

            bool validation_failed = false;
            auto& config = item.hair_angle_config;

            if (ImGui::Button(get_locale_cstr("action.angle_apply"))) {
                if (validate_angle_grid(config, ex, ey, theta, phi)) {
                    push_undo_now(item.id, std::nullopt,
                                  "Angle Config Edit");
                    config[{static_cast<float>(ex), static_cast<float>(ey)}] =
                        HairAngleEntry{theta, phi};
                    for (auto& s : item.hair_strands) s.mesh_dirty = true;
                    item.angle_config_editing_x = kSentinel;
                    item.angle_config_editing_y = kSentinel;
                } else {
                    validation_failed = true;
                }
            }

            ImGui::SameLine();
            if (ImGui::Button(get_locale_cstr("action.angle_cancel"))) {
                item.angle_config_editing_x = kSentinel;
                item.angle_config_editing_y = kSentinel;
            }

            // Handle window close via the X button
            if (!edit_open) {
                item.angle_config_editing_x = kSentinel;
                item.angle_config_editing_y = kSentinel;
            }

            if (validation_failed) {
                show_toast(get_locale_cstr("error.angle_grid_cross"), 2500.0f);
            }

            ImGui::End();
        }
    }

    ImGui::End();
}
}  // namespace sinriv::ui::render
