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
        // 可滚动的点列表（可编辑坐标）
        ImGui::BeginChild("GuidePointsList", ImVec2(0, 280), true);

        // 在循环前保存快照，用于撤销按钮+/-等即时修改
        auto before_edit = capture_snapshot(item);
        EditResult all_edits;
        int delete_point = -1;
        int swap_up = -1;
        int swap_down = -1;

        for (size_t pi = 0; pi < strand.guide_points.size(); ++pi) {
            ImGui::PushID(static_cast<int>(pi));

            char label_buf[64];
            snprintf(label_buf, sizeof(label_buf), "%s %zu",
                     get_locale_cstr("label.guide_point"), pi + 1);

            auto r = edit_vec3_stepper(label_buf, strand.guide_points[pi],
                                       0.5f, false, true);
            all_edits.activated |= r.activated;
            all_edits.deactivated_after_edit |= r.deactivated_after_edit;
            all_edits.value_changed |= r.value_changed;

            // 操作按钮放在同一行
            if (pi > 0) {
                ImGui::SameLine();
                if (ImGui::SmallButton("^")) {
                    swap_up = static_cast<int>(pi);
                }
            }
            if (pi < strand.guide_points.size() - 1) {
                ImGui::SameLine();
                if (ImGui::SmallButton("v")) {
                    swap_down = static_cast<int>(pi);
                }
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
        }
        show_width_editor_window = false;
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
        ImGui::BeginChild("WidthPointsList", ImVec2(0, 280), true);

        auto before_edit = capture_snapshot(item);
        EditResult all_edits;
        int delete_wp = -1;

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

            // 缩放滑条
            ImGui::SetNextItemWidth(140);
            ImGui::DragFloat(get_locale_cstr("label.width_scale"),
                             &wp.scale, 0.01f, 0.01f, 10.0f, "%.2f");
            if (ImGui::IsItemActivated())
                all_edits.activated = true;
            if (ImGui::IsItemDeactivatedAfterEdit())
                all_edits.deactivated_after_edit = true;

            ImGui::SameLine();
            if (ImGui::SmallButton(
                    get_locale_cstr("action.delete_width_point"))) {
                delete_wp = static_cast<int>(wi);
            }

            ImGui::Separator();
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
                        item.active_section_edit_strand = static_cast<int>(i);
                        show_cross_section_editor_window = true;
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
            item.hair_strands.erase(item.hair_strands.begin() + delete_idx);
            push_undo_now(item.id, std::nullopt, "Delete Hair Strand");
            for (auto& s : item.hair_strands) s.mesh_dirty = true;
        }

    }

    ImGui::End();
}

}  // namespace sinriv::ui::render
