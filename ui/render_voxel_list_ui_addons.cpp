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

    // Undo / Redo buttons (只撤销引导曲线相关的操作)
    {
        auto can_undo_guide = [&]() {
            if (!can_undo(item.id)) return false;
            return item.undo_stack.back().description.find("Guide Point") !=
                   std::string::npos;
        };
        auto can_redo_guide = [&]() {
            if (!can_redo(item.id)) return false;
            return item.redo_stack.back().description.find("Guide Point") !=
                   std::string::npos;
        };
        bool undo_disabled = !can_undo_guide();
        bool redo_disabled = !can_redo_guide();
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
        } else if (all_edits.value_changed) {
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
    // 互斥：打开宽度编辑器时关闭截面编辑器
    if (show_cross_section_editor_window) {
        auto sit = items.find(render_id);
        if (sit != items.end()) {
            sit->second->active_section_edit_strand = -1;
        }
        show_cross_section_editor_window = false;
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

    // Undo / Redo buttons (只撤销宽度编辑相关的操作)
    {
        auto can_undo_width = [&]() {
            if (!can_undo(item.id)) return false;
            return item.undo_stack.back().description.find("Width") !=
                   std::string::npos;
        };
        auto can_redo_width = [&]() {
            if (!can_redo(item.id)) return false;
            return item.redo_stack.back().description.find("Width") !=
                   std::string::npos;
        };
        bool undo_disabled = !can_undo_width();
        bool redo_disabled = !can_redo_width();
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
                ImGui::SameLine();
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
            ImGui::SameLine();
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
        } else if (all_edits.value_changed) {
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

    // 底模可见性
    if (ImGui::Checkbox(get_locale_cstr("label.show_origin_mesh"),
                        &item.showOriginMesh)) {
        showOriginMesh = item.showOriginMesh;
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

    // 中心点（发根汇聚点，所有发束共享）
    {
        bool old_show = item.show_addon_center;
        ImGui::Checkbox(get_locale_cstr("label.addon_center_point"),
                        &item.show_addon_center);
        if (old_show != item.show_addon_center) {
            push_undo_now(item.id, std::nullopt, "Toggle Center Point");
        }
        if (item.show_addon_center) {
            auto cp_edit = edit_vec3_stepper(
                get_locale_cstr("label.addon_center_point"),
                item.addon_center_point, 0.1f);
            if (cp_edit.activated) {
                begin_edit(item.id);
            }
            if (cp_edit.deactivated_after_edit) {
                end_edit(item.id, "Center Point Edit");
                for (auto& s : item.hair_strands) s.mesh_dirty = true;
            } else if (cp_edit.value_changed) {
                push_undo_now(item.id, std::nullopt, "Center Point Edit");
                for (auto& s : item.hair_strands) s.mesh_dirty = true;
            }
        }
    }

    ImGui::Separator();

    // 自动宽度按钮（打开发际线平面窗口）
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
            HairStrand strand;
            strand.name = "Strand " + std::to_string(item.hair_strands.size() + 1);
            strand.expanded = true;
            item.hair_strands.push_back(strand);
            push_undo_now(item.id, std::nullopt, "Add Hair Strand");
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
            bool expanded = ImGui::CollapsingHeader(header_label,
                                                     strand.expanded
                                                         ? ImGuiTreeNodeFlags_DefaultOpen
                                                         : 0);
            strand.expanded = expanded;

            // Show warning indicator when alpha_wrap repair failed for this strand
            if (strand.repair_failed) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.0f, 1.0f), " %s",
                                   get_locale_cstr("label.repair_failed"));
            }

            if (expanded) {
                // 三个按钮行
                // 上移
                if (i > 0) {
                    if (ImGui::Button(get_locale_cstr("action.move_up"))) {
                        std::swap(item.hair_strands[i],
                                  item.hair_strands[i - 1]);
                        push_undo_now(item.id, std::nullopt,
                                      "Move Strand Up");
                        item.hair_strands[i].mesh_dirty = true;
                        item.hair_strands[i - 1].mesh_dirty = true;
                    }
                    ImGui::SameLine();
                }
                // 下移
                if (i < item.hair_strands.size() - 1) {
                    if (ImGui::Button(get_locale_cstr("action.move_down"))) {
                        std::swap(item.hair_strands[i],
                                  item.hair_strands[i + 1]);
                        push_undo_now(item.id, std::nullopt,
                                      "Move Strand Down");
                        item.hair_strands[i].mesh_dirty = true;
                        item.hair_strands[i + 1].mesh_dirty = true;
                    }
                    ImGui::SameLine();
                }

                // 绘制引导曲线（自锁按钮）
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

                ImGui::SameLine();
                // 编辑宽度（自锁按钮）
                bool is_width_editing =
                    (item.active_width_edit_strand == static_cast<int>(i) &&
                     item.width_editing_active);
                if (is_width_editing) {
                    ImGui::PushStyleColor(ImGuiCol_Button,
                                          ImVec4(0.2f, 0.7f, 0.3f, 1.0f));
                }
                if (ImGui::Button(
                        is_width_editing
                            ? get_locale_cstr("action.stop_width_edit")
                            : get_locale_cstr("action.edit_width"))) {
                    if (is_width_editing) {
                        item.width_editing_active = false;
                        item.active_width_edit_strand = -1;
                        show_width_editor_window = false;
                    } else {
                        // 互斥：打开宽度编辑器时关闭引导曲线
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
                if (is_width_editing) {
                    ImGui::PopStyleColor();
                }

                ImGui::SameLine();
                // 编辑截面（自锁按钮）
                bool is_section_editing =
                    (item.active_section_edit_strand == static_cast<int>(i));
                if (is_section_editing) {
                    ImGui::PushStyleColor(ImGuiCol_Button,
                                          ImVec4(0.5f, 0.5f, 0.9f, 1.0f));
                }
                if (ImGui::Button(
                        is_section_editing
                            ? get_locale_cstr("action.stop_edit_section")
                            : get_locale_cstr("action.edit_section"))) {
                    if (is_section_editing) {
                        item.active_section_edit_strand = -1;
                        show_cross_section_editor_window = false;
                    } else {
                        // Check for per-point section overrides before
                        // opening global section editor
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
                            // 互斥：打开截面编辑器时关闭引导曲线和宽度编辑器
                            if (item.guide_curve_drawing_active) {
                                item.guide_curve_drawing_active = false;
                                item.active_guide_draw_strand = -1;
                                show_guide_curve_window = false;
                            }
                            if (item.width_editing_active) {
                                item.width_editing_active = false;
                                item.active_width_edit_strand = -1;
                                show_width_editor_window = false;
                            }
                            item.active_section_edit_strand =
                                static_cast<int>(i);
                            show_cross_section_editor_window = true;
                        }
                    }
                }
                if (is_section_editing) {
                    ImGui::PopStyleColor();
                }

                ImGui::SameLine();
                // 删除发束
                if (ImGui::Button(get_locale_cstr("action.delete_strand"))) {
                    delete_idx = static_cast<int>(i);
                }

                // 清空引导点
                ImGui::SameLine();
                if (ImGui::Button(get_locale_cstr("action.clear_guide_points"))) {
                    strand.guide_points.clear();
                }

                // 显示点数信息
                ImGui::Text(get_locale_cstr("label.guide_curve_points"),
                            static_cast<int>(strand.guide_points.size()));
                ImGui::SameLine();
                ImGui::Text(get_locale_cstr("label.width_points"),
                            static_cast<int>(strand.width_points.size()));

                // Section rotation slider
                ImGui::SetNextItemWidth(160);
                float old_rot = strand.section_rotation;
                ImGui::SliderFloat(get_locale_cstr("label.section_rotation"),
                                   &strand.section_rotation, -180.0f, 180.0f,
                                   "%.0f deg");
                if (old_rot != strand.section_rotation) {
                    strand.mesh_dirty = true;
                }

                // 细分精度：引导曲线贝塞尔插值 / 截面贝塞尔平滑
                ImGui::SetNextItemWidth(160);
                int old_guide_subdiv = strand.guide_samples_per_segment;
                ImGui::SliderInt(get_locale_cstr("label.guide_subdiv"),
                                 &strand.guide_samples_per_segment, 4, 128);
                if (old_guide_subdiv != strand.guide_samples_per_segment) {
                    strand.mesh_dirty = true;
                }
                // ImGui::SameLine();
                ImGui::SetNextItemWidth(160);
                int old_section_subdiv = strand.section_subdiv;
                ImGui::SliderInt(get_locale_cstr("label.section_subdiv"),
                                 &strand.section_subdiv, 1, 32);
                if (old_section_subdiv != strand.section_subdiv) {
                    strand.mesh_dirty = true;
                }

                // Alpha wrap 修复参数（每根发束独立调节）
                ImGui::SetNextItemWidth(160);
                float old_repair_alpha = strand.repair_alpha;
                ImGui::SliderFloat(get_locale_cstr("label.alpha_wrap_alpha"),
                                   &strand.repair_alpha, 0.01f, 100.0f, "%.2f");
                if (old_repair_alpha != strand.repair_alpha) {
                    strand.mesh_dirty = true;
                }
                // ImGui::SameLine();
                ImGui::SetNextItemWidth(160);
                float old_repair_offset = strand.repair_offset;
                ImGui::SliderFloat(get_locale_cstr("label.alpha_wrap_offset"),
                                   &strand.repair_offset, 0.001f, 10.0f, "%.3f");
                if (old_repair_offset != strand.repair_offset) {
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
            item.hair_strands.erase(item.hair_strands.begin() + delete_idx);
            push_undo_now(item.id, std::nullopt, "Delete Hair Strand");
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

    // 互斥：打开发际线窗口时关闭其他编辑窗口
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

    ImGui::SetNextWindowSize(ImVec2(420, 320), ImGuiCond_Once);
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

    // 发际线平面启用开关
    bool old_enabled = item.hairline_plane_enabled;
    ImGui::Checkbox(get_locale_cstr("label.hairline_plane_enable"),
                    &item.hairline_plane_enabled);
    if (old_enabled != item.hairline_plane_enabled) {
        push_undo_now(item.id, std::nullopt, "Toggle Hairline Plane");
    }

    if (!item.hairline_plane_enabled) {
        ImGui::TextDisabled("%s",
            get_locale_cstr("label.hairline_plane_disabled_hint"));
        ImGui::End();
        return;
    }

    ImGui::Separator();

    // 平面模式
    const char* plane_mode_items[] = {
        get_locale_cstr("label.hairline_y_plane"),
        get_locale_cstr("label.hairline_3point_plane"),
    };
    int plane_mode = item.hairline_plane_use_y ? 0 : 1;
    ImGui::SetNextItemWidth(160);
    if (ImGui::Combo(get_locale_cstr("label.hairline_plane_mode"),
                    &plane_mode, plane_mode_items,
                    IM_ARRAYSIZE(plane_mode_items))) {
        push_undo_now(item.id, std::nullopt, "Hairline Plane Mode");
        item.hairline_plane_use_y = (plane_mode == 0);
    }

    ImGui::Separator();

    if (item.hairline_plane_use_y) {
        // Y 水平面模式
        float old_y = item.hairline_plane_y;
        ImGui::SetNextItemWidth(200);
        ImGui::DragFloat(get_locale_cstr("label.hairline_y"),
                         &item.hairline_plane_y, 0.1f);
        if (ImGui::IsItemActivated()) begin_edit(item.id);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            end_edit(item.id, "Hairline Y Edit");
            for (auto& s : item.hair_strands) s.mesh_dirty = true;
        } else if (old_y != item.hairline_plane_y) {
            push_undo_now(item.id, std::nullopt, "Hairline Y Edit");
        }

        ImGui::TextDisabled("%s",
            get_locale_cstr("label.hairline_preview_triangle"));
    } else {
        // 三点平面模式：每行一个点 + [拾取] 按钮
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

            // 拾取按钮
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
                    // Cancel picking
                    item.hairline_point_picking_active = false;
                } else {
                    // Start picking this point
                    item.hairline_point_picking_active = true;
                    item.hairline_picking_point_index = pi;
                }
            }
            if (is_picking) {
                ImGui::PopStyleColor();
            }
            if (ImGui::IsItemHovered() && !is_picking) {
                ImGui::SetTooltip("%s",
                    get_locale_cstr("tooltip.pick_point"));
            }

            ImGui::PopID();
        }

        if (pt_activated) begin_edit(item.id);
        if (pt_deactivated) {
            end_edit(item.id, "Hairline Points Edit");
            for (auto& s : item.hair_strands) s.mesh_dirty = true;
        } else if (pt_changed) {
            push_undo_now(item.id, std::nullopt, "Hairline Points Edit");
        }

        // 三点平面退化提示
        {
            const auto& p0 = item.hairline_plane_points[0];
            const auto& p1 = item.hairline_plane_points[1];
            const auto& p2 = item.hairline_plane_points[2];
            vec3f e1 = {p1.x - p0.x, p1.y - p0.y, p1.z - p0.z};
            vec3f e2 = {p2.x - p0.x, p2.y - p0.y, p2.z - p0.z};
            float area = std::sqrt(
                (e1.y * e2.z - e1.z * e2.y) * (e1.y * e2.z - e1.z * e2.y) +
                (e1.z * e2.x - e1.x * e2.z) * (e1.z * e2.x - e1.x * e2.z) +
                (e1.x * e2.y - e1.y * e2.x) * (e1.x * e2.y - e1.y * e2.x));
            if (area < 1e-6f) {
                ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "%s",
                    get_locale_cstr("label.hairline_degenerate"));
            }
        }
    }

    ImGui::Separator();

    // 应用按钮
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

}  // namespace sinriv::ui::render
