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
#include "kigstudio/utils/vec3.h"
#include "kigstudio/voxel/voxel2mesh.h"
#include "kigstudio/agent/agent_handlers.h"
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {

// Forward declaration for ray-triangle intersection (defined later in this file)
static bool ray_triangle_intersect(const vec3f& ray_origin,
                                    const vec3f& ray_dir,
                                    const vec3f& v0, const vec3f& v1,
                                    const vec3f& v2, float& t);

// Helper: load stb_image from a UTF-8 path on Windows.
// On Windows, fopen doesn't accept UTF-8 paths by default, so we read the
// file ourselves via the wide-char API and use stbi_load_from_memory.
#ifdef _WIN32
static unsigned char* stbi_load_utf8(const char* utf8_path, int* w, int* h,
                                     int* comp, int req_comp) {
    int wlen =
        MultiByteToWideChar(CP_UTF8, 0, utf8_path, -1, nullptr, 0);
    if (wlen <= 0) return nullptr;
    std::wstring wpath(static_cast<size_t>(wlen), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8_path, -1, &wpath[0], wlen);
    // Remove trailing null from std::wstring length calculation
    if (!wpath.empty() && wpath.back() == L'\0')
        wpath.pop_back();

    HANDLE hFile =
        CreateFileW(wpath.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
                    OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (hFile == INVALID_HANDLE_VALUE) return nullptr;

    DWORD size = GetFileSize(hFile, nullptr);
    if (size == INVALID_FILE_SIZE || size == 0) {
        CloseHandle(hFile);
        return nullptr;
    }

    std::vector<unsigned char> buffer(size);
    DWORD read = 0;
    BOOL ok = ReadFile(hFile, buffer.data(), size, &read, nullptr);
    CloseHandle(hFile);
    if (!ok || read != size) return nullptr;

    return stbi_load_from_memory(buffer.data(), static_cast<int>(size), w, h,
                                 comp, req_comp);
}
#endif

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

        // Reset per-point hover highlight each frame
        item.hovered_guide_point_strand_uuid.clear();
        item.hovered_guide_point_index = -1;

        for (size_t pi = 0; pi < strand.guide_points.size(); ++pi) {
            ImGui::PushID(static_cast<int>(pi));
            bool point_hovered = false;

            char label_buf[64];
            snprintf(label_buf, sizeof(label_buf),
                     get_locale_cstr("label.guide_point"),
                     static_cast<int>(pi + 1));

            auto r = edit_vec3_stepper(label_buf, strand.guide_points[pi],
                                       0.5f, false, true);
            all_edits.activated |= r.activated;
            all_edits.deactivated_after_edit |= r.deactivated_after_edit;
            all_edits.value_changed |= r.value_changed;
            if (!point_hovered && ImGui::IsItemHovered())
                point_hovered = true;

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
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip(
                            "%s",
                            get_locale_cstr("tooltip.move_toward_center"));
                        point_hovered = true;
                    }
                    if (ImGui::IsItemClicked(ImGuiMouseButton_Right))
                        ImGui::OpenPopup("kp_center_menu");
                    ImGui::SameLine();
                    if (ImGui::SmallButton("-")) {
                        strand.guide_points[pi] =
                            strand.guide_points[pi] - dir * kp_move_step;
                        all_edits.value_changed = true;
                    }
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip(
                            "%s",
                            get_locale_cstr("tooltip.move_away_from_center"));
                        point_hovered = true;
                    }
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
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip(
                        "%s", get_locale_cstr("tooltip.move_point_up"));
                    point_hovered = true;
                }
            }
            if (pi < strand.guide_points.size() - 1) {
                ImGui::SameLine();
                if (ImGui::SmallButton("v")) {
                    swap_down = static_cast<int>(pi);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip(
                        "%s", get_locale_cstr("tooltip.move_point_down"));
                    point_hovered = true;
                }
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("X")) {
                delete_point = static_cast<int>(pi);
            }

            if (point_hovered) {
                item.hovered_guide_point_strand_uuid = strand.uuid;
                item.hovered_guide_point_index = static_cast<int>(pi);
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
            git->second->active_guide_draw_strand.clear();
        }
        show_guide_curve_window = false;
    }
    if (show_width_editor_window) {
        auto wit = items.find(render_id);
        if (wit != items.end()) {
            wit->second->width_editing_active = false;
            wit->second->active_width_edit_strand.clear();
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
    {0,0,"Midline / Midsagittal","头顶 / 前正中线"},
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
    // Phi peaks at the crown (Y≈0) and decreases toward both front neck
    // and back neck, so we check monotonicity separately for Y≤0 and Y≥0.
    for (int x = -10; x <= 10; ++x) {
        std::vector<std::pair<int, float>> col;  // (Y, phi)
        for (int y = -10; y <= 14; ++y) {
            auto it = tmp.find({static_cast<float>(x), static_cast<float>(y)});
            if (it != tmp.end()) {
                col.push_back({y, it->second.phi});
            }
        }
        if (col.size() < 2) continue;

        // Split at Y=0: back region (Y≤0) and front region (Y≥0).
        // Each region independently must be monotonic.
        auto check_monotonic = [](const std::vector<std::pair<int, float>>& seg) -> bool {
            if (seg.size() < 2) return true;
            bool increasing = true, decreasing = true;
            for (size_t i = 1; i < seg.size(); ++i) {
                if (seg[i].second <= seg[i-1].second) increasing = false;
                if (seg[i].second >= seg[i-1].second) decreasing = false;
            }
            return increasing || decreasing;
        };

        std::vector<std::pair<int, float>> back_region, front_region;
        for (const auto& p : col) {
            if (p.first <= 0) back_region.push_back(p);
            if (p.first >= 0) front_region.push_back(p);
        }
        if (!check_monotonic(back_region)) return false;
        if (!check_monotonic(front_region)) return false;
    }

    // 3. Midline separation: X=0 and X=±10 must not have overlapping theta.
    // Only valid for Y ≥ 0 (front of head) where X=0 is anterior midline and
    // X=±10 is posterior midline. For Y < 0 (top/back of head), all three
    // reference the same posterior region so the separation check is skipped.
    for (int y = -10; y <= 14; ++y) {
        if (y < 0) continue;  // skip back-of-head rows
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
            it->second->active_guide_draw_strand.clear();
        }
        show_guide_curve_window = false;
    }
    if (show_width_editor_window) {
        auto wit = items.find(render_id);
        if (wit != items.end()) {
            wit->second->width_editing_active = false;
            wit->second->active_width_edit_strand.clear();
        }
        show_width_editor_window = false;
    }
    if (show_cross_section_editor_window) {
        auto sit = items.find(render_id);
        if (sit != items.end()) sit->second->active_section_edit_strand.clear();
        show_cross_section_editor_window = false;
    }
    if (show_perpoint_section_editor_window) {
        auto pit = items.find(render_id);
        if (pit != items.end()) {
            pit->second->perpoint_section_editing_active = false;
            pit->second->active_perpoint_section_edit_strand.clear();
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

    // Helper: rebuild hair BVH from base node's mesh triangles.
    // Called whenever hair_angle_config is modified so that crosshair
    // markers can raycast to the base model surface.
    auto rebuild_hair_bvh = [&]() {
        if (item.addon_base_node_id < 0) return;
        auto base_it = this->items.find(item.addon_base_node_id);
        if (base_it == this->items.end()) return;
        auto& base = *base_it->second;
        std::vector<sinriv::kigstudio::voxel::Triangle> tris;
        if (!base.source_triangles.empty()) {
            tris = base.source_triangles;
        } else if (!base.cached_mesh.empty()) {
            tris.reserve(base.cached_mesh.size());
            for (const auto& [tri, _] : base.cached_mesh)
                tris.push_back(tri);
        }
        if (tris.empty()) return;
        auto bvh = std::make_unique<
            sinriv::kigstudio::voxel::triangle_bvh<float>>();
        for (const auto& tri : tris)
            bvh->insert(tri);
        item.hair_bvh = std::move(bvh);
        item.hair_bvh_base_node_id = item.addon_base_node_id;
    };

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
            bool is_origin = (x == 0 && y == 0);
            if (configured) {
                snprintf(btn_id, sizeof(btn_id), "%s##e%d_%d",
                    get_locale_cstr("action.angle_edit"), x, y);
                if (ImGui::SmallButton(btn_id))
                    open_edit(it->second.theta, it->second.phi);
                ImGui::SameLine();
                if (is_origin)
                    ImGui::BeginDisabled();
                snprintf(btn_id, sizeof(btn_id), "%s##d%d_%d",
                    get_locale_cstr("action.angle_delete"), x, y);
                if (ImGui::SmallButton(btn_id)) {
                    del_x = x;
                    del_y = y;
                }
                if (is_origin)
                    ImGui::EndDisabled();
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
            rebuild_hair_bvh();
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

            bool is_origin = (ex == 0 && ey == 0);
            if (is_origin) {
                // (0,0) is the origin: X=0 midline → theta must be locked at 0°
                theta = 0.0f;
                ImGui::BeginDisabled();
                ImGui::DragFloat(get_locale_cstr("label.angle_theta"), &theta,
                    1.0f, -180.0f, 180.0f, "%.1f deg");
                ImGui::EndDisabled();
                ImGui::SameLine();
                ImGui::TextDisabled("(locked)");
            } else {
                ImGui::DragFloat(get_locale_cstr("label.angle_theta"), &theta,
                    1.0f, -180.0f, 180.0f, "%.1f deg");
            }
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
                    rebuild_hair_bvh();
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
// ============================================================================
// Orthographic Projection Edit Mode
// ============================================================================

// Möller–Trumbore ray-triangle intersection
// Returns true and sets t if ray hits triangle (v0,v1,v2), false otherwise.
static bool ray_triangle_intersect(const vec3f& ray_origin,
                                    const vec3f& ray_dir,
                                    const vec3f& v0, const vec3f& v1,
                                    const vec3f& v2, float& t) {
    const float eps = 1e-8f;
    vec3f e1 = {v1.x - v0.x, v1.y - v0.y, v1.z - v0.z};
    vec3f e2 = {v2.x - v0.x, v2.y - v0.y, v2.z - v0.z};
    vec3f pvec = {
        ray_dir.y * e2.z - ray_dir.z * e2.y,
        ray_dir.z * e2.x - ray_dir.x * e2.z,
        ray_dir.x * e2.y - ray_dir.y * e2.x
    };
    float det = e1.x * pvec.x + e1.y * pvec.y + e1.z * pvec.z;
    if (std::abs(det) < eps) return false;
    float inv_det = 1.0f / det;
    vec3f tvec = {ray_origin.x - v0.x, ray_origin.y - v0.y, ray_origin.z - v0.z};
    float u = (tvec.x * pvec.x + tvec.y * pvec.y + tvec.z * pvec.z) * inv_det;
    if (u < 0.0f || u > 1.0f) return false;
    vec3f qvec = {
        tvec.y * e1.z - tvec.z * e1.y,
        tvec.z * e1.x - tvec.x * e1.z,
        tvec.x * e1.y - tvec.y * e1.x
    };
    float v = (ray_dir.x * qvec.x + ray_dir.y * qvec.y + ray_dir.z * qvec.z) * inv_det;
    if (v < 0.0f || u + v > 1.0f) return false;
    t = (e2.x * qvec.x + e2.y * qvec.y + e2.z * qvec.z) * inv_det;
    return t > eps;
}

// Raycast from ortho camera through image pixel (px,py) against base model triangles.
// Returns true and sets world_pos to the nearest intersection point.
static bool ortho_raycast(const OrthoProjectionState& state,
                           int px, int py, vec3f& world_pos) {
    if (state._base_triangles.empty()) return false;

    int res = state.render_resolution;
    float half = state.viewport_size * 0.5f;
    float u = (static_cast<float>(px) / res - 0.5f);
    float v = (0.5f - static_cast<float>(py) / res);

    // World-space point on the image plane
    vec3f plane_pt = {
        state._center.x + state._cam_right.x * u * state.viewport_size +
                          state._cam_up.x * v * state.viewport_size,
        state._center.y + state._cam_right.y * u * state.viewport_size +
                          state._cam_up.y * v * state.viewport_size,
        state._center.z + state._cam_right.z * u * state.viewport_size +
                          state._cam_up.z * v * state.viewport_size
    };

    // Ray direction (from camera position toward the model, same as look_dir)
    vec3f ray_dir = {
        state.projection_dir.x,
        state.projection_dir.y,
        state.projection_dir.z
    };
    float rl = std::sqrt(ray_dir.x * ray_dir.x + ray_dir.y * ray_dir.y +
                         ray_dir.z * ray_dir.z);
    if (rl < 1e-8f) return false;
    ray_dir.x /= rl; ray_dir.y /= rl; ray_dir.z /= rl;

    // Move the ray origin from the center plane back onto the camera plane
    // (through _cam_pos, perpendicular to ray_dir).  Surfaces between the
    // camera and the center plane (e.g. the face in a front view) must be
    // hittable too — otherwise the first surface found along the ray is on
    // the far side of the model (back of the head).
    float cam_off = (plane_pt.x - state._cam_pos.x) * ray_dir.x +
                    (plane_pt.y - state._cam_pos.y) * ray_dir.y +
                    (plane_pt.z - state._cam_pos.z) * ray_dir.z;
    plane_pt.x -= ray_dir.x * cam_off;
    plane_pt.y -= ray_dir.y * cam_off;
    plane_pt.z -= ray_dir.z * cam_off;

    float best_t = 1e30f;
    bool hit = false;
    for (const auto& tri : state._base_triangles) {
        float t;
        if (ray_triangle_intersect(plane_pt, ray_dir,
                                    std::get<0>(tri), std::get<1>(tri),
                                    std::get<2>(tri), t)) {
            if (t < best_t) {
                best_t = t;
                hit = true;
            }
        }
    }

    if (hit) {
        world_pos = {
            plane_pt.x + ray_dir.x * best_t,
            plane_pt.y + ray_dir.y * best_t,
            plane_pt.z + ray_dir.z * best_t
        };
    }
    return hit;
}

/// Extrapolate a new guide point from existing 3D curve when raycast misses
/// the model.  Builds a curvature-preserving tangent from the last 2+ guide
/// points, then finds the closest approach between the camera ray and the
/// extrapolated line.  Returns true and sets out_pt (on the ray).
static bool extrapolate_guide_along_ray(const vec3f& ro, const vec3f& rd,
                                        const std::vector<vec3f>& existing,
                                        vec3f& out_pt, float vp_size) {
    const size_t n = existing.size();
    if (n < 2) return false;

    const vec3f& P_last = existing.back();

    // ---- Build curvature-preserving tangent direction ----
    vec3f T;
    if (n == 2) {
        const vec3f& A = existing[n - 2];
        T = vec3f{P_last.x - A.x, P_last.y - A.y, P_last.z - A.z};
    } else {
        // Blend last 2-3 normalised segment directions, weighted toward recent
        size_t seg_count = std::min(n - 1, size_t(3));
        float w_sum = 0.0f;
        T = vec3f{0, 0, 0};
        for (size_t i = 0; i < seg_count; ++i) {
            size_t idx = n - seg_count - 1 + i;
            const vec3f& A = existing[idx];
            const vec3f& B = existing[idx + 1];
            vec3f dir{B.x - A.x, B.y - A.y, B.z - A.z};
            float len = std::sqrt(dir.x * dir.x + dir.y * dir.y +
                                  dir.z * dir.z);
            if (len < 1e-8f) continue;
            dir.x /= len; dir.y /= len; dir.z /= len;
            float w = static_cast<float>(i + 1);  // linear ramp
            T.x += dir.x * w; T.y += dir.y * w; T.z += dir.z * w;
            w_sum += w;
        }
        if (w_sum > 0.0f) {
            T.x /= w_sum; T.y /= w_sum; T.z /= w_sum;
        } else {
            const vec3f& A = existing[n - 2];
            T = vec3f{P_last.x - A.x, P_last.y - A.y, P_last.z - A.z};
        }
    }

    // Normalise
    float t_len = std::sqrt(T.x * T.x + T.y * T.y + T.z * T.z);
    if (t_len < 1e-8f) return false;
    T.x /= t_len; T.y /= t_len; T.z /= t_len;

    // ---- Two-line closest-point (ray vs extrapolation line) ----
    float b = rd.x * T.x + rd.y * T.y + rd.z * T.z;
    if (std::abs(b) > 0.9999f) return false;  // near-parallel

    vec3f v{ro.x - P_last.x, ro.y - P_last.y, ro.z - P_last.z};
    float d = -(v.x * rd.x + v.y * rd.y + v.z * rd.z);
    float e = -(v.x * T.x + v.y * T.y + v.z * T.z);
    float inv_det = 1.0f / (b * b - 1.0f);
    float s = (d - b * e) * inv_det;
    float t = (b * d - e) * inv_det;

    // Clamp to forward direction (s≥0: in front of image plane;
    // t≥0: forward along the extrapolated curve)
    if (s < 0.0f) s = 0.0f;
    if (t < 0.0f) t = 0.0f;

    // Constrained closest-point pair
    vec3f c_ray{ro.x + s * rd.x, ro.y + s * rd.y, ro.z + s * rd.z};
    vec3f c_line{P_last.x + t * T.x, P_last.y + t * T.y,
                 P_last.z + t * T.z};
    float dist = std::sqrt((c_ray.x - c_line.x) * (c_ray.x - c_line.x) +
                           (c_ray.y - c_line.y) * (c_ray.y - c_line.y) +
                           (c_ray.z - c_line.z) * (c_ray.z - c_line.z));

    if (dist > vp_size * 0.5f) return false;

    out_pt = c_ray;
    return true;
}

void RenderVoxelList::destroy_ortho_resources() {
    // NOTE: overlay_tex and overlay_cpu_rgba_ are intentionally left alone.
    // They are independent of the base-model render and should survive
    // re-renders triggered by depth-colour toggles, viewport changes, etc.
    if (bgfx::isValid(ortho_state.view_fb)) {
        bgfx::destroy(ortho_state.view_fb);
        ortho_state.view_fb = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(ortho_state.view_tex)) {
        bgfx::destroy(ortho_state.view_tex);
        ortho_state.view_tex = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(ortho_state.view_depth_tex)) {
        bgfx::destroy(ortho_state.view_depth_tex);
        ortho_state.view_depth_tex = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(ortho_state.ai_readback_tex)) {
        bgfx::destroy(ortho_state.ai_readback_tex);
        ortho_state.ai_readback_tex = BGFX_INVALID_HANDLE;
    }
    ortho_state.coord_map_ready = false;
    ortho_state.view_tex_ready = false;
    ortho_state.render_dirty = true;
    ortho_state.ortho_render_stage = 0;
    ortho_state._base_triangles.clear();
    ortho_state.ai_export_stage = 0;
    ortho_state.ai_readback_pending = false;
    ortho_state.ai_export_pending = false;

    // Release shader programs while bgfx context is still valid
    if (ortho_shader_) {
        ortho_shader_->release();
        ortho_shader_.reset();
    }
}

void RenderVoxelList::perform_ortho_render(RenderVoxelItem& item,
                                            RenderVoxelItem& base_item) {
    destroy_ortho_resources();

    // Build the orthographic camera matrices (stored for CPU-side raycasting).
    // projection_dir now means "look direction" (from camera toward center).
    // We derive a "from-center" direction for constructing the camera basis.
    vec3f look_dir = ortho_state.projection_dir;
    vec3f from_center = {-look_dir.x, -look_dir.y, -look_dir.z};
    vec3f center = item.addon_center_point;

    float half = ortho_state.viewport_size * 0.5f;
    // Use the semantic coordinate frame's north-pole as the preferred
    // camera-up direction.  Fall back only when the projection direction
    // is nearly parallel to the north pole.
    vec3f np = {item.hair_north_pole.x, item.hair_north_pole.y, item.hair_north_pole.z};
    float np_len = std::sqrt(np.x*np.x + np.y*np.y + np.z*np.z);
    vec3f world_up = (np_len > 1e-8f) ? np : vec3f{0, 1, 0};
    if (std::abs(from_center.x * world_up.x + from_center.y * world_up.y + from_center.z * world_up.z) > 0.99f)
        world_up = vec3f{0, 0, 1};

    // cam_right = normalize(cross(from_center, world_up))
    vec3f cam_right = {
        from_center.y * world_up.z - from_center.z * world_up.y,
        from_center.z * world_up.x - from_center.x * world_up.z,
        from_center.x * world_up.y - from_center.y * world_up.x
    };
    float cr_len = std::sqrt(cam_right.x * cam_right.x +
                             cam_right.y * cam_right.y +
                             cam_right.z * cam_right.z);
    if (cr_len > 1e-8f) {
        cam_right.x /= cr_len; cam_right.y /= cr_len; cam_right.z /= cr_len;
    } else {
        cam_right = {1, 0, 0};
    }

    // cam_up = normalize(cross(cam_right, from_center))
    vec3f cam_up = {
        cam_right.y * from_center.z - cam_right.z * from_center.y,
        cam_right.z * from_center.x - cam_right.x * from_center.z,
        cam_right.x * from_center.y - cam_right.y * from_center.x
    };
    float cu_len = std::sqrt(cam_up.x * cam_up.x +
                             cam_up.y * cam_up.y +
                             cam_up.z * cam_up.z);
    if (cu_len > 1e-8f) {
        cam_up.x /= cu_len; cam_up.y /= cu_len; cam_up.z /= cu_len;
    } else {
        cam_up = {0, 1, 0};
    }

    // Camera position: move opposite the look direction from center.
    // projection_dir = camera→center (inward), so:
    // cam = center - dir * 1000 puts camera on the face/viewer side.
    vec3f cam_pos = {center.x - look_dir.x * 1000.0f,
                     center.y - look_dir.y * 1000.0f,
                     center.z - look_dir.z * 1000.0f};

    // Store ortho camera params for CPU raycasting
    ortho_state._cam_right = cam_right;
    ortho_state._cam_up = cam_up;
    ortho_state._cam_pos = cam_pos;
    ortho_state._center = center;

    // Copy base model triangles for CPU-side raycasting
    ortho_state._base_triangles.clear();
    if (!base_item.cached_mesh.empty()) {
        ortho_state._base_triangles.reserve(base_item.cached_mesh.size());
        for (const auto& [tri, n] : base_item.cached_mesh) {
            (void)n;
            ortho_state._base_triangles.push_back(tri);
        }
    } else if (!base_item.source_triangles.empty()) {
        ortho_state._base_triangles = base_item.source_triangles;
    }

    ortho_state.coord_map_ready = true;
    ortho_state._base_triangle_count = ortho_state._base_triangles.size();

    // ---- Create GPU off-screen render resources ----
    int res = ortho_state.render_resolution;
    constexpr uint64_t tex_flags =
        BGFX_TEXTURE_RT | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP |
        BGFX_SAMPLER_MIN_ANISOTROPIC | BGFX_SAMPLER_MAG_ANISOTROPIC;

    // Create the render texture with mipmaps.  BGFX_RESOLVE_AUTO_GEN_MIPS
    // auto-generates mip levels after rendering.  When the render is
    // displayed at ~574 px via the font shader's texture2D(), the GPU
    // auto-selects the appropriate mip LOD (e.g. log2(2048/574) ≈ 1.83)
    // and trilinearly blends between levels, providing effective AA
    // even at high minification ratios.
    bool has_mips = true;
    ortho_state.view_tex = bgfx::createTexture2D(
        static_cast<uint16_t>(res), static_cast<uint16_t>(res), has_mips, 1,
        bgfx::TextureFormat::BGRA8, tex_flags);
    ortho_state.view_depth_tex = bgfx::createTexture2D(
        static_cast<uint16_t>(res), static_cast<uint16_t>(res), false, 1,
        bgfx::TextureFormat::D32F, tex_flags);

    bgfx::Attachment fbo_att[2];
    fbo_att[0].init(ortho_state.view_tex, bgfx::Access::Write, 0, 1, 0,
                    BGFX_RESOLVE_AUTO_GEN_MIPS);
    fbo_att[1].init(ortho_state.view_depth_tex, bgfx::Access::Write);
    ortho_state.view_fb =
        bgfx::createFrameBuffer(2, fbo_att, false);

    // Create ortho shader (view 200 for off-screen render)
    if (!ortho_shader_) {
        ortho_shader_ = std::make_unique<RenderMeshShader>(kOrthoViewView, 0);
    }

    // Kick off multi-frame render
    ortho_state.ortho_base_item_id = base_item.id;
    ortho_state.ortho_render_stage = 1;  // RENDER
    ortho_state.render_dirty = false;

    std::cout << "[ortho_render] Setup off-screen render res=" << res
              << " with " << ortho_state._base_triangles.size()
              << " triangles, look_dir=("
              << look_dir.x << "," << look_dir.y << "," << look_dir.z << ")"
              << std::endl;
}

void RenderVoxelList::process_ortho_render() {
    if (ortho_state.ortho_render_stage == 0)
        return;  // IDLE

    // Stage 1: Submit render commands
    if (ortho_state.ortho_render_stage == 1) {
        if (!bgfx::isValid(ortho_state.view_fb) || !ortho_shader_) {
            ortho_state.ortho_render_stage = 0;
            return;
        }

        int render_mode = ortho_state.ortho_render_mode;

        if (render_mode == 1) {  // Depth
            if (!ortho_shader_->ensureOrthoDepthProgram()) {
                std::cerr << "[ortho_render] Failed to load ortho depth shader" << std::endl;
                ortho_state.ortho_render_stage = 0;
                return;
            }
        } else {  // Contour (0) or Lighting (2) — both use GBuffer + u_lightingMode
            if (!ortho_shader_->ensureGBufferProgram()) {
                std::cerr << "[ortho_render] Failed to load GBuffer shader" << std::endl;
                ortho_state.ortho_render_stage = 0;
                return;
            }
        }

        // Find the base item
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(ortho_state.ortho_base_item_id);
        if (it == items.end()) {
            ortho_state.ortho_render_stage = 0;
            return;
        }
        auto& base_item = it->second;

        // Check renderer availability.
        // The item-level mesh_renderer holds the smooth source mesh for BOTH
        // mesh_only and voxel items (voxel_renderer's own main mesh is never
        // populated for voxel items — only its chunked voxel surface is).
        // The chunked surface is midpoint-snapped binary marching cubes
        // (only axis-aligned / 45° edges at voxel resolution), which looks
        // blocky / "mosaic-like" in ortho projection, so use it only as a
        // last resort when no smooth mesh exists at all.
        bool use_chunked = false;  // true → render chunked voxel surface
        if (base_item->mesh_renderer.empty() && !base_item->cached_mesh.empty()) {
            // Smooth mesh data exists CPU-side but was never uploaded
            base_item->mesh_renderer.loadGeometry(base_item->cached_mesh);
        }
        if (base_item->mesh_renderer.empty()) {
            if (base_item->mesh_only || base_item->voxel_renderer.empty()) {
                std::cerr << "[ortho_render] Base item has no renderer" << std::endl;
                ortho_state.ortho_render_stage = 0;
                return;
            }
            use_chunked = true;
        }

        // Build orthographic view and projection matrices
        vec3f center = ortho_state._center;
        float half = ortho_state.viewport_size * 0.5f;

        // View matrix: look at center from the stored camera position
        vec3f cam_pos = ortho_state._cam_pos;
        vec3f cam_up = ortho_state._cam_up;

        float view[16];
        bx::mtxLookAt(view,
                      bx::Vec3{cam_pos.x, cam_pos.y, cam_pos.z},
                      bx::Vec3{center.x, center.y, center.z},
                      bx::Vec3{cam_up.x, cam_up.y, cam_up.z});

        float proj[16];
        // bottom > top compensates for the implicit Y-flip when the OpenGL
        // framebuffer (bottom-left origin) is displayed via ImGui::Image
        // (top-left origin).  This way the rendered image orientation
        // matches the CPU-side raycasting coordinate system.
        bx::mtxOrtho(proj, -half, half, half, -half, -2000.0f, 2000.0f, 0.0f,
                     bgfx::getCaps()->homogeneousDepth);

        int res = ortho_state.render_resolution;
        bgfx::setViewRect(kOrthoViewView, 0, 0, static_cast<uint16_t>(res),
                          static_cast<uint16_t>(res));
        bgfx::setViewFrameBuffer(kOrthoViewView, ortho_state.view_fb);
        bgfx::setViewClear(kOrthoViewView,
                           BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                           0x303030ff, 1.0f, 0);
        bgfx::setViewTransform(kOrthoViewView, view, proj);

        float identity[16];
        bx::mtxIdentity(identity);

        // Depth / Lighting uniforms (used by depth and lighting modes)
        float view_dir_arr[3] = {0, 0, 0};
        float center_arr[3] = {center.x, center.y, center.z};
        float depth_scale = ortho_state.viewport_size > 1e-8f
                                ? 1.0f / ortho_state.viewport_size
                                : 0.01f;
        vec3f look_dir = ortho_state.projection_dir;
        float fl = std::sqrt(look_dir.x*look_dir.x + look_dir.y*look_dir.y + look_dir.z*look_dir.z);
        if (fl > 1e-8f) {
            look_dir.x /= fl; look_dir.y /= fl; look_dir.z /= fl;
        }
        view_dir_arr[0] = look_dir.x;
        view_dir_arr[1] = look_dir.y;
        view_dir_arr[2] = look_dir.z;

        if (render_mode == 1) {  // Depth heatmap
            if (use_chunked) {
                base_item->voxel_renderer.renderDepthColor(identity, *ortho_shader_,
                                                           view_dir_arr, center_arr,
                                                           depth_scale);
            } else {
                base_item->mesh_renderer.renderDepthColor(identity, *ortho_shader_,
                                                          view_dir_arr, center_arr,
                                                          depth_scale);
            }
        } else {  // Contour or Lighting — both use GBuffer program
            // Set lighting mode: 0.0 = contour, 1.0 = lighting
            float lighting_vec[4] = { (render_mode == 2) ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f };
            ortho_shader_->ensureUniforms();
            bgfx::setUniform(ortho_shader_->u_lighting_mode_, lighting_vec);
            if (use_chunked) {
                base_item->voxel_renderer.renderGBuffer(identity, *ortho_shader_);
            } else {
                base_item->mesh_renderer.renderGBuffer(identity, *ortho_shader_);
            }
        }
        bgfx::touch(kOrthoViewView);

        ortho_state.ortho_render_stage = 2;  // WAIT
        ortho_state.ortho_wait_frames = 2;
        return;
    }

    // Stage 2: Wait for render to complete
    if (ortho_state.ortho_render_stage == 2) {
        if (ortho_state.ortho_wait_frames > 0) {
            ortho_state.ortho_wait_frames--;
        }
        if (ortho_state.ortho_wait_frames <= 0) {
            ortho_state.ortho_render_stage = 3;  // DONE
        }
        return;
    }

    // Stage 3: Done
    if (ortho_state.ortho_render_stage == 3) {
        ortho_state.view_tex_ready = true;
        ortho_state.api_render_dirty = true;  // trigger GPU readback for API cache
        ortho_state.ortho_render_stage = 0;  // back to IDLE
        std::cout << "[ortho_render] View texture ready" << std::endl;
    }
}

// Draw a pixel on an RGBA buffer (clamped to bounds).
static void draw_pixel(std::vector<uint8_t>& rgba, int w, int h,
                       int px, int py, uint8_t r, uint8_t g, uint8_t b,
                       uint8_t a = 255) {
    if (px < 0 || px >= w || py < 0 || py >= h) return;
    size_t idx = (static_cast<size_t>(py) * w + px) * 4;
    // Alpha blend
    float sa = a / 255.0f;
    float da = rgba[idx + 3] / 255.0f;
    float out_a = sa + da * (1.0f - sa);
    if (out_a < 0.001f) return;
    rgba[idx + 0] = static_cast<uint8_t>(
        (r * sa + rgba[idx + 0] * da * (1.0f - sa)) / out_a);
    rgba[idx + 1] = static_cast<uint8_t>(
        (g * sa + rgba[idx + 1] * da * (1.0f - sa)) / out_a);
    rgba[idx + 2] = static_cast<uint8_t>(
        (b * sa + rgba[idx + 2] * da * (1.0f - sa)) / out_a);
    rgba[idx + 3] = static_cast<uint8_t>(out_a * 255.0f);
}

// Bresenham line draw on RGBA buffer (thin, single-pixel).
static void draw_line(std::vector<uint8_t>& rgba, int w, int h,
                      int x0, int y0, int x1, int y1,
                      uint8_t r, uint8_t g, uint8_t b) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        draw_pixel(rgba, w, h, x0, y0, r, g, b);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// Draw a thick line by stamping circles at each Bresenham step.
static void draw_circle_marker(std::vector<uint8_t>& rgba, int w, int h,
                               int cx, int cy, int radius,
                               uint8_t r, uint8_t g, uint8_t b) {
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (dx * dx + dy * dy <= radius * radius) {
                draw_pixel(rgba, w, h, cx + dx, cy + dy, r, g, b);
            }
        }
    }
}

static void draw_thick_line(std::vector<uint8_t>& rgba, int w, int h,
                            int x0, int y0, int x1, int y1,
                            int thickness,
                            uint8_t r, uint8_t g, uint8_t b) {
    if (thickness <= 1) {
        draw_line(rgba, w, h, x0, y0, x1, y1, r, g, b);
        return;
    }
    float radius = (thickness - 1) * 0.5f;
    // Collect all Bresenham steps, then draw filled circles at each step.
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int cx = x0, cy = y0;
    while (true) {
        draw_circle_marker(rgba, w, h, cx, cy, static_cast<int>(radius + 0.5f),
                           r, g, b);
        if (cx == x1 && cy == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; cx += sx; }
        if (e2 <= dx) { err += dx; cy += sy; }
    }
}

// ---- stb_truetype font rendering ----
static stbtt_fontinfo g_font_info;
static std::vector<uint8_t> g_font_data;
static bool g_font_loaded = false;
static float g_font_scale = 14.0f;  // cached after load

static bool try_load_font_file(const char* path) {
    if (g_font_loaded) return true;
#ifdef _WIN32
    FILE* f = fopen(path, "rb");
#else
    FILE* f = fopen(path, "rb");
#endif
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size <= 0 || size > 64 * 1024 * 1024) { fclose(f); return false; }
    g_font_data.resize(static_cast<size_t>(size));
    size_t rd = fread(g_font_data.data(), 1, static_cast<size_t>(size), f);
    fclose(f);
    if (rd != static_cast<size_t>(size)) return false;

    int offset = stbtt_GetFontOffsetForIndex(g_font_data.data(), 0);
    g_font_loaded = (offset >= 0) &&
        stbtt_InitFont(&g_font_info, g_font_data.data(), offset);
    if (g_font_loaded) {
        std::cout << "[draw_text] Loaded font: " << path
                  << " (" << size << " bytes)" << std::endl;
    }
    return g_font_loaded;
}

static void ensure_font_loaded(float font_size) {
    if (g_font_loaded) {
        g_font_scale = stbtt_ScaleForPixelHeight(&g_font_info, font_size);
        return;
    }
    // Try multiple common font paths.
    // A CJK-capable font (like msyh.ttc) is preferred for Chinese strand names.
    const char* kFontPaths[] = {
        // Windows CJK fonts (TrueType Collection)
        "C:/Windows/Fonts/msyh.ttc",
        "C:/Windows/Fonts/simsun.ttc",
        "C:/Windows/Fonts/msgothic.ttc",
        // Windows basic fonts
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/consola.ttf",
        "C:/Windows/Fonts/segoeui.ttf",
        // Bundled font (relative to working directory)
        "dep/bgfx.cmake/bgfx/examples/runtime/font/droidsans.ttf",
        "../dep/bgfx.cmake/cmake/bgfx/Release/font/droidsans.ttf",
        "build/dep/bgfx.cmake/cmake/bgfx/Release/font/droidsans.ttf",
        // macOS
        "/System/Library/Fonts/Helvetica.ttc",
        "/System/Library/Fonts/PingFang.ttc",
        // Linux
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
    };
    for (const char* path : kFontPaths) {
        // For .ttc files (TrueType Collection), we need a different approach:
        // try loading, and if it fails with offset=0, try with offset from
        // stbtt_GetFontOffsetForIndex.
        FILE* f = fopen(path, "rb");
        if (!f) continue;
        fseek(f, 0, SEEK_END);
        long size = ftell(f);
        fseek(f, 0, SEEK_SET);
        if (size <= 0 || size > 64 * 1024 * 1024) { fclose(f); continue; }
        g_font_data.resize(static_cast<size_t>(size));
        size_t rd = fread(g_font_data.data(), 1, static_cast<size_t>(size), f);
        fclose(f);
        if (rd != static_cast<size_t>(size)) continue;

        // Try each collection index (up to 4) for .ttc files
        bool ok = false;
        for (int idx = 0; idx < 4; ++idx) {
            int off = stbtt_GetFontOffsetForIndex(g_font_data.data(), idx);
            if (off < 0) break;
            ok = stbtt_InitFont(&g_font_info, g_font_data.data(), off);
            if (ok) {
                g_font_loaded = true;
                g_font_scale = stbtt_ScaleForPixelHeight(&g_font_info, font_size);
                std::cout << "[draw_text] Loaded font: " << path
                          << " (idx=" << idx << ", " << size << " bytes)"
                          << std::endl;
                return;
            }
        }
        g_font_data.clear();  // bad file, retry next
    }
    // No font found; text rendering will be silently skipped.
    std::cerr << "[draw_text] WARNING: No TTF font found. "
              << "Text labels will not be rendered." << std::endl;
}

// Decode a UTF-8 codepoint; returns the codepoint and advances `s`.
static int utf8_decode(const char** s) {
    unsigned char c = static_cast<unsigned char>(**s);
    if (c < 0x80) { (*s)++; return c; }
    int n;
    unsigned cp;
    if ((c & 0xE0) == 0xC0) { n = 2; cp = c & 0x1F; }
    else if ((c & 0xF0) == 0xE0) { n = 3; cp = c & 0x0F; }
    else if ((c & 0xF8) == 0xF0) { n = 4; cp = c & 0x07; }
    else { (*s)++; return 0xFFFD; }  // replacement char
    for (int i = 1; i < n; i++) {
        unsigned char nc = static_cast<unsigned char>((*s)[i]);
        if ((nc & 0xC0) != 0x80) { (*s)++; return 0xFFFD; }
        cp = (cp << 6) | (nc & 0x3F);
    }
    (*s) += n;
    return static_cast<int>(cp);
}

// Draw a text string at (x,y) using stb_truetype rasterization.
// (x,y) is the top-left of the first glyph's bounding box.
static void draw_text(std::vector<uint8_t>& rgba, int img_w, int img_h,
                      int x, int y, const char* text,
                      uint8_t r, uint8_t g, uint8_t b,
                      float font_size) {
    if (!text || !*text) return;
    ensure_font_loaded(font_size);
    if (!g_font_loaded) return;

    float scale = stbtt_ScaleForPixelHeight(&g_font_info, font_size);
    g_font_scale = scale;

    int ascent, descent, line_gap;
    stbtt_GetFontVMetrics(&g_font_info, &ascent, &descent, &line_gap);
    float baseline = y + ascent * scale;

    float xpos = static_cast<float>(x);
    const char* p = text;
    int prev_glyph = 0;
    while (*p) {
        int cp = utf8_decode(&p);
        if (cp == 0xFFFD) { prev_glyph = 0; continue; }

        int glyph = stbtt_FindGlyphIndex(&g_font_info, cp);

        // Kerning with previous glyph
        if (prev_glyph) {
            xpos += scale * stbtt_GetGlyphKernAdvance(&g_font_info, prev_glyph, glyph);
        }
        prev_glyph = glyph;

        int advance, lsb;
        stbtt_GetGlyphHMetrics(&g_font_info, glyph, &advance, &lsb);

        float x_shift = xpos - std::floor(xpos);
        int c_x1, c_y1, c_x2, c_y2;
        stbtt_GetGlyphBitmapBoxSubpixel(&g_font_info, glyph,
            scale, scale, x_shift, 0, &c_x1, &c_y1, &c_x2, &c_y2);

        int gw = c_x2 - c_x1;
        int gh = c_y2 - c_y1;
        if (gw > 0 && gh > 0) {
            if (gw > 4096 || gh > 4096) {
                xpos += advance * scale;
                continue;
            }
            std::vector<uint8_t> bitmap(static_cast<size_t>(gw) * gh);
            stbtt_MakeGlyphBitmapSubpixel(&g_font_info,
                bitmap.data(), gw, gh, gw,
                scale, scale, x_shift, 0, glyph);

            int dst_x0 = static_cast<int>(xpos) + c_x1;
            int dst_y0 = static_cast<int>(baseline) + c_y1;
            for (int by = 0; by < gh; by++) {
                int dst_y = dst_y0 + by;
                if (dst_y < 0 || dst_y >= img_h) continue;
                for (int bx = 0; bx < gw; bx++) {
                    uint8_t alpha = bitmap[by * gw + bx];
                    if (alpha > 0) {
                        int dst_x = dst_x0 + bx;
                        draw_pixel(rgba, img_w, img_h, dst_x, dst_y,
                                   r, g, b, alpha);
                    }
                }
            }
        }

        xpos += advance * scale;
    }
}

// Draw a small filled circle (for control point markers).

// Draw guide curves on an RGBA pixel buffer using ortho camera projection.
void draw_guide_curves_on_buffer(
    std::vector<uint8_t>& rgba, int w, int h,
    const OrthoProjectionState& ortho_state,
    const std::vector<HairStrand>& hair_strands,
    bool color_code,
    int line_thickness,
    float font_size) {
    // Color palette (matching hair_guides.py)
    static const uint32_t kPalette[] = {
        0xff4040ff, 0x40c8ffff, 0xffe040ff, 0x60ff80ff,
        0xff80d0ff, 0xffa040ff, 0xa080ffff, 0x40ffc8ff,
        0xffff80ff, 0xff8080ff, 0x80a0ffff, 0x80ff40ff,
    };
    constexpr int kPaletteSize =
        sizeof(kPalette) / sizeof(kPalette[0]);

    // Camera parameters
    vec3f center = ortho_state._center;
    vec3f cam_right = ortho_state._cam_right;
    vec3f cam_up = ortho_state._cam_up;
    float half = ortho_state.viewport_size * 0.5f;

    auto project = [&](const vec3f& world) -> std::pair<int, int> {
        float rx = (world.x - center.x) * cam_right.x +
                   (world.y - center.y) * cam_right.y +
                   (world.z - center.z) * cam_right.z;
        float ry = (world.x - center.x) * cam_up.x +
                   (world.y - center.y) * cam_up.y +
                   (world.z - center.z) * cam_up.z;
        float ndc_x = rx / half;  // [-1, 1]
        float ndc_y = ry / half;  // [-1, 1]
        int px = static_cast<int>((ndc_x * 0.5f + 0.5f) * w);
        int py = static_cast<int>((0.5f - ndc_y * 0.5f) * h);
        return {px, py};
    };

    std::cout << "[draw_guide_curves] center=(" << center.x << "," << center.y
              << "," << center.z << ") half=" << half << " res=" << w
              << "x" << h << " line_thickness=" << line_thickness
              << " font_size=" << font_size << std::endl;

    int color_idx = 0;
    for (const auto& strand : hair_strands) {
        if (!strand.visible || strand.guide_points.size() < 2)
            continue;

        // Pick color
        uint32_t col = color_code
                           ? kPalette[color_idx % kPaletteSize]
                           : 0xffffffff;  // white
        ++color_idx;
        uint8_t cr = static_cast<uint8_t>((col >> 24) & 0xff);
        uint8_t cg = static_cast<uint8_t>((col >> 16) & 0xff);
        uint8_t cb = static_cast<uint8_t>((col >> 8) & 0xff);

        // Sample bezier curve
        auto sampled = sample_bezier_guide_curve(
            strand.guide_points,
            std::max(strand.guide_samples_per_segment, 1));

        // Draw line segments with configurable thickness
        for (size_t pi = 0; pi + 1 < sampled.size(); ++pi) {
            auto [px0, py0] = project(sampled[pi]);
            auto [px1, py1] = project(sampled[pi + 1]);
            draw_thick_line(rgba, w, h, px0, py0, px1, py1,
                            line_thickness, cr, cg, cb);
        }

        // Draw control point markers (slightly larger for thicker lines)
        int marker_radius = 2 + line_thickness / 2;
        for (const auto& p : strand.guide_points) {
            auto [px, py] = project(p);
            draw_circle_marker(rgba, w, h, px, py, marker_radius, cr, cg, cb);
        }

        // Draw strand name label at the first guide point
        if (font_size > 0.0f && !strand.name.empty()) {
            auto [px, py] = project(strand.guide_points.front());
            // Offset the label so it sits above the control point marker
            int label_x = px + marker_radius + 4;
            int label_y = py - marker_radius - static_cast<int>(font_size) - 2;
            draw_text(rgba, w, h, label_x, label_y, strand.name.c_str(),
                      cr, cg, cb, font_size);
        }
    }
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

void RenderVoxelList::render_hair_root_window() {
    if (!show_hair_root_window)
        return;

    // Do NOT close other windows (non-exclusive per requirement)

    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Once, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(420, 520), ImGuiCond_Once);
    bool window_open = true;
    if (!ImGui::Begin(get_locale_cstr("window.hair_root_edit"), &window_open)) {
        ImGui::End();
        return;
    }

    if (!window_open) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->hair_root_edit_active = false;
        }
        show_hair_root_window = false;
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

    item.hair_root_edit_active = true;

    // ---- Common Hair Root Point (shared by all strands) ----

    // Auto hair root toggle (only when north_pole direction is configured)
    {
        float np_len = std::sqrt(item.hair_north_pole.x * item.hair_north_pole.x +
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
                                if (ray_triangle_intersect(origin, ray_dir,
                                                           v0, v1, v2, t) &&
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
                                    auto tv0 = std::get<0>(tri);
                                    auto tv1 = std::get<1>(tri);
                                    auto tv2 = std::get<2>(tri);
                                    test_tri({tv0.x, tv0.y, tv0.z},
                                             {tv1.x, tv1.y, tv1.z},
                                             {tv2.x, tv2.y, tv2.z});
                                }
                            } else {
                                for (const auto& tri : base.source_triangles) {
                                    auto tv0 = std::get<0>(tri);
                                    auto tv1 = std::get<1>(tri);
                                    auto tv2 = std::get<2>(tri);
                                    test_tri({tv0.x, tv0.y, tv0.z},
                                             {tv1.x, tv1.y, tv1.z},
                                             {tv2.x, tv2.y, tv2.z});
                                }
                            }
                        }
                    }
                    if (!found) {
                        // Fallback: project center along north pole direction
                        hit = {item.addon_center_point.x - dir.x * 10.0f,
                               item.addon_center_point.y - dir.y * 10.0f,
                               item.addon_center_point.z - dir.z * 10.0f};
                    }
                    item.common_hair_root_point = hit;
                    push_undo_now(item.id, std::nullopt, "Auto Hair Root");
                }
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", get_locale_cstr("tooltip.auto_hair_root"));
        } else {
            ImGui::TextWrapped("%s",
                get_locale_cstr("label.need_north_pole_for_hair_root"));
        }
    }

    // Display common root point position (read-only for now)
    ImGui::Text("%s: (%.2f, %.2f, %.2f)",
                get_locale_cstr("label.common_hair_root_point"),
                static_cast<double>(item.common_hair_root_point.x),
                static_cast<double>(item.common_hair_root_point.y),
                static_cast<double>(item.common_hair_root_point.z));

    ImGui::Separator();

    // Center offset slider (moves the root point toward center)
    float prev_offset = item.hair_root_center_offset;
    ImGui::SetNextItemWidth(200);
    ImGui::SliderFloat(get_locale_cstr("label.hair_root_center_offset"),
                       &item.hair_root_center_offset, 0.0f, 50.0f, "%.1f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.hair_root_center_offset"));
    if (prev_offset != item.hair_root_center_offset) {
        push_undo_now(item.id, std::nullopt, "Hair Root Offset");
    }

    ImGui::Separator();

    // Strand list with checkboxes
    ImGui::TextUnformatted(get_locale_cstr("label.hair_strands"));
    ImGui::Separator();

    for (size_t i = 0; i < item.hair_strands.size(); ++i) {
        auto& strand = item.hair_strands[i];
        ImGui::PushID(static_cast<int>(i));

        bool prev_enabled = strand.hair_root_enabled;
        char label[128];
        if (!strand.name.empty()) {
            snprintf(label, sizeof(label), "%s##hr_%zu", strand.name.c_str(), i);
        } else {
            snprintf(label, sizeof(label), "%s %zu##hr_%zu",
                     get_locale_cstr("label.hair_strand"),
                     i + 1, i);
        }
        ImGui::Checkbox(label, &strand.hair_root_enabled);
        if (prev_enabled != strand.hair_root_enabled) {
            strand.mesh_dirty = true;
        }

        ImGui::PopID();
    }

    ImGui::Separator();

    // "Update All Hair Roots" button — propagate common root point to enabled strands
    if (ImGui::Button(get_locale_cstr("action.update_all_hair_roots"),
                      ImVec2(-1, 0))) {
        push_undo_now(item.id, std::nullopt, "Update All Hair Roots");

        // Compute effective root point: common_hair_root_point moved toward center by offset
        vec3f effective_root = item.common_hair_root_point;
        {
            vec3f to_center = {
                item.addon_center_point.x - effective_root.x,
                item.addon_center_point.y - effective_root.y,
                item.addon_center_point.z - effective_root.z
            };
            float dist = std::sqrt(to_center.x * to_center.x +
                                   to_center.y * to_center.y +
                                   to_center.z * to_center.z);
            if (dist > 0.001f && item.hair_root_center_offset > 0.0f) {
                vec3f dir = {to_center.x / dist, to_center.y / dist,
                             to_center.z / dist};
                float offset = item.hair_root_center_offset;
                if (offset > dist) offset = dist;
                effective_root = {
                    effective_root.x + dir.x * offset,
                    effective_root.y + dir.y * offset,
                    effective_root.z + dir.z * offset
                };
            }
        }

        // Assign the common root point to each enabled strand's hidden_guide_points_start
        for (auto& strand : item.hair_strands) {
            if (strand.hair_root_enabled) {
                strand.hidden_guide_points_start = {effective_root};
                strand.mesh_dirty = true;
            } else {
                // Clear hidden points for disabled strands
                strand.hidden_guide_points_start.clear();
                strand.mesh_dirty = true;
            }
        }
    }

    ImGui::End();
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

void RenderVoxelList::render_ortho_setup_window() {
    if (!show_ortho_setup_window)
        return;

    ImGui::SetNextWindowSize(ImVec2(420, 320), ImGuiCond_Once);
    if (!ImGui::Begin(get_locale_cstr("window.ortho_projection_setup"),
                      &show_ortho_setup_window)) {
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

    // Check base model
    if (item.addon_base_node_id < 0) {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s",
                           get_locale_cstr("label.ortho_no_base_model"));
        ImGui::End();
        return;
    }

    // ---- Initialize viewport size & render resolution from
    //     persisted per-node values, or auto-calculate on first open ----
    if (!ortho_state.viewport_size_defaulted) {
        // Viewport: use saved value if present, otherwise auto-calc
        if (item.ortho_viewport_size > 0.0f) {
            ortho_state.viewport_size = item.ortho_viewport_size;
        } else {
            auto base_it = items.find(item.addon_base_node_id);
            if (base_it != items.end()) {
                const auto& base = *base_it->second;
                float max_dist2 = 0.0f;
                auto check_vertex = [&](const vec3f& v) {
                    vec3f rel = {v.x - item.addon_center_point.x,
                                 v.y - item.addon_center_point.y,
                                 v.z - item.addon_center_point.z};
                    float d2 = rel.x*rel.x + rel.y*rel.y + rel.z*rel.z;
                    if (d2 > max_dist2) max_dist2 = d2;
                };
                if (!base.source_triangles.empty()) {
                    for (const auto& tri : base.source_triangles) {
                        check_vertex(std::get<0>(tri));
                        check_vertex(std::get<1>(tri));
                        check_vertex(std::get<2>(tri));
                    }
                } else if (!base.cached_mesh.empty()) {
                    for (const auto& tri_n : base.cached_mesh) {
                        const auto& tri = std::get<0>(tri_n);
                        check_vertex(std::get<0>(tri));
                        check_vertex(std::get<1>(tri));
                        check_vertex(std::get<2>(tri));
                    }
                }
                if (max_dist2 > 0.0f) {
                    float sphere_r = std::sqrt(max_dist2) * 1.05f;
                    ortho_state.viewport_size = sphere_r * 2.2f;
                }
            }
        }
        // Render resolution: use saved value if present, else default
        if (item.ortho_render_resolution > 0)
            ortho_state.render_resolution = item.ortho_render_resolution;

        // Sync projection_dir with the default six_view_index on first open.
        // Otherwise the default projection_dir {0,1,0} (top-down) is used
        // while the combo shows "Front" (index 0), causing a mismatch.
        ortho_state.projection_dir =
            six_view_direction(ortho_state.six_view_index,
                               item.hair_front_reference,
                               item.hair_north_pole);

        ortho_state.viewport_size_defaulted = true;
    }

    // ---- Direction mode selection ----
    ImGui::TextUnformatted(get_locale_cstr("label.vector_mode"));
    ImGui::SameLine();
    const char* mode_names[] = {
        get_locale_cstr("label.vector_mode_six"),
        get_locale_cstr("label.vector_mode_pick"),
    };
    ImGui::Combo("##vector_mode", &ortho_state.vector_mode, mode_names, 2);

    ImGui::Separator();

    if (ortho_state.vector_mode == 0) {
        // ---- Six view selection ----
        const char* view_names[] = {
            get_locale_cstr("label.six_view_front"),
            get_locale_cstr("label.six_view_back"),
            get_locale_cstr("label.six_view_left"),
            get_locale_cstr("label.six_view_right"),
            get_locale_cstr("label.six_view_top"),
            get_locale_cstr("label.six_view_bottom"),
        };
        if (ImGui::Combo(get_locale_cstr("label.projection_direction"),
                         &ortho_state.six_view_index, view_names, 6)) {
            ortho_state.projection_dir =
                six_view_direction(ortho_state.six_view_index,
                                   item.hair_front_reference,
                                   item.hair_north_pole);
            // Direction changed: the off-screen texture must be re-rendered
            ortho_state.render_dirty = true;
        }
    } else {
        // ---- Pick point on model ----
        bool was_picking = ortho_state.is_picking_point;
        if (ImGui::Button(get_locale_cstr("action.pick_projection"))) {
            ortho_state.is_picking_point = true;
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", get_locale_cstr("action.picking_direction"));

        if (ortho_state.is_picking_point && !was_picking) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.08f, 1.0f), "%s",
                               get_locale_cstr("action.picking_direction"));
        }
    }

    // Show current direction
    ImGui::Text("%s: (%.2f, %.2f, %.2f)",
                get_locale_cstr("label.projection_direction"),
                ortho_state.projection_dir.x,
                ortho_state.projection_dir.y,
                ortho_state.projection_dir.z);

    ImGui::Separator();

    // ---- Viewport size ----
    if (ImGui::DragFloat(get_locale_cstr("label.viewport_size"),
                         &ortho_state.viewport_size, 1.0f, 10.0f, 500.0f, "%.1f")) {
        ortho_state.render_dirty = true;
        item.ortho_viewport_size = ortho_state.viewport_size;
    }

    // ---- Render resolution ----
    int res = ortho_state.render_resolution;
    const int res_options[] = {512, 1024, 2048, 4096};
    const char* res_names[] = {"512", "1024", "2048", "4096"};
    int res_idx = 2;
    for (int i = 0; i < 4; ++i) {
        if (res == res_options[i]) { res_idx = i; break; }
    }
    if (ImGui::Combo(get_locale_cstr("label.render_resolution"),
                     &res_idx, res_names, 4)) {
        ortho_state.render_resolution = res_options[res_idx];
        item.ortho_render_resolution = ortho_state.render_resolution;
        ortho_state.render_dirty = true;
    }

    ImGui::Separator();

    // ---- Render button ----
    if (ImGui::Button(get_locale_cstr("action.ortho_render"),
                      ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
        auto base_it = items.find(item.addon_base_node_id);
        if (base_it == items.end()) {
            show_toast(get_locale_string("label.ortho_no_base_model"), 3000.0f);
        } else {
            RenderVoxelItem& base_item = *base_it->second;
            if (base_item.source_triangles.empty() &&
                base_item.cached_mesh.empty()) {
                show_toast(get_locale_string("label.ortho_no_mesh_data"), 3000.0f);
            } else {
                perform_ortho_render(item, base_item);
                ortho_state.active = true;
                ortho_state.edit_window_open = true;
                show_ortho_setup_window = false;
                show_ortho_edit_window = true;

                // Overlay: only six-view presets carry a saved reference image.
                // Picked-vector mode starts without any overlay; manual loads
                // in that mode are temporary and never persisted.
                if (ortho_state.vector_mode == 0) {
                    int vi = ortho_state.six_view_index;
                    const auto& saved = item.ortho_overlay[vi];
                    ortho_state.overlay_image_path = saved.image_path;
                    ortho_state.overlay_img_width = saved.img_width;
                    ortho_state.overlay_img_height = saved.img_height;
                    ortho_state.overlay_enabled = saved.enabled;
                    ortho_state.overlay_offset =
                        ImVec2(saved.offset_x, saved.offset_y);
                    ortho_state.overlay_scale_x = saved.scale_x;
			    ortho_state.overlay_scale_y = saved.scale_y;
                    ortho_state.blend_ratio = saved.blend_ratio;
                    ortho_state.overlay_locked = saved.locked;

                    // Reload the image texture if there's a saved path
                    if (!saved.image_path.empty()) {
                        int w, h, comp;
                        unsigned char* data =
#ifdef _WIN32
                            stbi_load_utf8(saved.image_path.c_str(), &w, &h, &comp, 4);
#else
                            stbi_load(saved.image_path.c_str(), &w, &h, &comp, 4);
#endif
                        if (data) {
                            if (bgfx::isValid(ortho_state.overlay_tex))
                                bgfx::destroy(ortho_state.overlay_tex);
                            ortho_state.overlay_tex = bgfx::createTexture2D(
                                static_cast<uint16_t>(w),
                                static_cast<uint16_t>(h), false, 1,
                                bgfx::TextureFormat::RGBA8,
                                BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP);
                            bgfx::updateTexture2D(
                                ortho_state.overlay_tex, 0, 0, 0, 0,
                                static_cast<uint16_t>(w),
                                static_cast<uint16_t>(h),
                                bgfx::copy(data, w * h * 4));
                            // Keep CPU-side copy for API blending
                            size_t cpu_sz = static_cast<size_t>(w) * h * 4;
                            overlay_cpu_rgba_.resize(cpu_sz);
                            memcpy(overlay_cpu_rgba_.data(), data, cpu_sz);
                            overlay_cpu_w_ = w;
                            overlay_cpu_h_ = h;
                            stbi_image_free(data);
                            ortho_state.overlay_img_width = w;
                            ortho_state.overlay_img_height = h;
                        }
                    }
                } else {
                	// Picked-vector mode: discard any overlay left over from a
                	// previous six-view session.
                	if (bgfx::isValid(ortho_state.overlay_tex))
                		bgfx::destroy(ortho_state.overlay_tex);
                	ortho_state.overlay_tex = BGFX_INVALID_HANDLE;
                	ortho_state.overlay_image_path.clear();
                	ortho_state.overlay_enabled = false;
                	ortho_state.overlay_img_width = 0;
                	ortho_state.overlay_img_height = 0;
                	ortho_state.overlay_offset = ImVec2(0, 0);
                	ortho_state.overlay_scale_x = 1.0f;
                	ortho_state.overlay_scale_y = 1.0f;
                	ortho_state.blend_ratio = 0.5f;
                	ortho_state.overlay_locked = false;
                	overlay_cpu_rgba_.clear();
                	overlay_cpu_w_ = 0;
                	overlay_cpu_h_ = 0;
                }
            }
        }
    }

    ImGui::End();
}

void RenderVoxelList::render_ortho_edit_window() {
    if (!show_ortho_edit_window || !ortho_state.edit_window_open)
        return;

    ImGui::SetNextWindowSize(ImVec2(600, 700), ImGuiCond_Once);
    bool window_open = true;
    if (!ImGui::Begin(get_locale_cstr("window.ortho_edit"), &window_open)) {
        ImGui::End();
        return;
    }

    // Helper: sync current overlay state back to the owning item (six-view only)
    auto sync_overlay_to_item = [&]() {
        if (ortho_state.vector_mode != 0) return;
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it == items.end() || it->second->source_type != 2) return;
        auto& ol = it->second->ortho_overlay[ortho_state.six_view_index];
        ol.image_path = ortho_state.overlay_image_path;
        ol.img_width = ortho_state.overlay_img_width;
        ol.img_height = ortho_state.overlay_img_height;
        ol.enabled = ortho_state.overlay_enabled;
        ol.offset_x = ortho_state.overlay_offset.x;
        ol.offset_y = ortho_state.overlay_offset.y;
        ol.scale_x = ortho_state.overlay_scale_x;
	    ol.scale_y = ortho_state.overlay_scale_y;
        ol.blend_ratio = ortho_state.blend_ratio;
        ol.locked = ortho_state.overlay_locked;
    };

    if (!window_open) {
        sync_overlay_to_item();
        show_ortho_edit_window = false;
        ortho_state.edit_window_open = false;
        ortho_state.active = false;
        destroy_ortho_resources();
        ImGui::End();
        return;
    }

    // ---- Top toolbar: Load reference image ----
    if (ortho_state.view_tex_ready) {
        ImGui::SameLine();
        ImGui::TextDisabled("res=%d", ortho_state.render_resolution);
    }
    if (ImGui::Button(get_locale_cstr("action.load_reference_image"))) {
        const char* filters[] = {"*.png", "*.jpg", "*.jpeg", "*.bmp"};
        const char* path = tinyfd_openFileDialog(
            get_locale_cstr("action.load_reference_image"), "", 4, filters,
            get_locale_cstr("action.load_reference_image"), 0);
        if (path) {
            std::string utf8_path = tinyfd_path_to_utf8(path);
            int w, h, comp;
            unsigned char* data =
#ifdef _WIN32
                stbi_load_utf8(utf8_path.c_str(), &w, &h, &comp, 4);
#else
                stbi_load(utf8_path.c_str(), &w, &h, &comp, 4);
#endif
            if (data) {
                if (bgfx::isValid(ortho_state.overlay_tex))
                    bgfx::destroy(ortho_state.overlay_tex);
                ortho_state.overlay_tex = bgfx::createTexture2D(
                    static_cast<uint16_t>(w), static_cast<uint16_t>(h), false, 1,
                    bgfx::TextureFormat::RGBA8,
                    BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP);
                bgfx::updateTexture2D(ortho_state.overlay_tex, 0, 0, 0, 0,
                                      static_cast<uint16_t>(w),
                                      static_cast<uint16_t>(h),
                                      bgfx::copy(data, w * h * 4));
                // Keep CPU-side copy for API blending
                size_t data_sz = static_cast<size_t>(w) * h * 4;
                overlay_cpu_rgba_.resize(data_sz);
                memcpy(overlay_cpu_rgba_.data(), data, data_sz);
                overlay_cpu_w_ = w;
                overlay_cpu_h_ = h;
                stbi_image_free(data);
                ortho_state.overlay_image_path = utf8_path;
                ortho_state.overlay_img_width = w;
                ortho_state.overlay_img_height = h;
                ortho_state.overlay_enabled = true;
                ortho_state.overlay_offset = ImVec2(0, 0);
                ortho_state.overlay_scale_x = 1.0f;
		ortho_state.overlay_scale_y = 1.0f;
                sync_overlay_to_item();
            } else {
                show_toast("Failed to load image: " + utf8_path, 3000.0f);
            }
        }
    }

    // Overlay enable checkbox (only shown when image loaded)
    bool overlay_changed = false;
    if (bgfx::isValid(ortho_state.overlay_tex)) {
        ImGui::SameLine();
        if (ImGui::Checkbox(get_locale_cstr("label.enable_overlay"),
                            &ortho_state.overlay_enabled))
            overlay_changed = true;

        // Blend slider
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        if (ImGui::SliderFloat(get_locale_cstr("label.blend_ratio"),
                               &ortho_state.blend_ratio, 0.0f, 1.0f))
            overlay_changed = true;

        // Scale X/Y sliders (independent axis scaling)
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        if (ImGui::DragFloat("##scale_x", &ortho_state.overlay_scale_x,
                             0.01f, 0.1f, 10.0f, "SX:%.2f"))
            overlay_changed = true;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        if (ImGui::DragFloat("##scale_y", &ortho_state.overlay_scale_y,
                             0.01f, 0.1f, 10.0f, "SY:%.2f"))
            overlay_changed = true;

        // Lock button (toggle overlay drag/resize)
        ImGui::SameLine();
        if (ImGui::Button(ortho_state.overlay_locked
                              ? get_locale_cstr("label.overlay_unlock")
                              : get_locale_cstr("label.overlay_lock"))) {
            ortho_state.overlay_locked = !ortho_state.overlay_locked;
            overlay_changed = true;
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", get_locale_cstr("tooltip.overlay_lock"));
    }
    if (overlay_changed)
        sync_overlay_to_item();

    // ImGui::Separator();

    // // ---- API status (shared with main Agent API) ----
    // if (agent_server_ptr && agent_server_ptr->is_running()) {
    //     ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.3f, 1.0f),
    //                        "● API:%d", agent_server_ptr->port());
    //     if (ImGui::IsItemHovered())
    //         ImGui::SetTooltip("Ortho endpoints: http://127.0.0.1:%d/api/v1/ortho",
    //                           agent_server_ptr->port());
    // }

    // Keep API server caches in sync (ortho render, overlay params, state JSON)
    if (agent_server_ptr && agent_server_ptr->is_running())
        update_api_server_caches();

    // Auto-trigger GPU readback when a new render is available,
    // pushing pixels to the API cache (no disk files).
    if (ortho_state.api_render_dirty && ortho_state.ai_export_stage == 0) {
        ortho_state.api_render_dirty = false;
        ortho_state.ai_export_pending = true;
        ortho_state.ai_export_stage = 1;
    }

    // Process AI export readback (if pending)
    process_ai_export();

    ImGui::Separator();

    // ---- Handle re-render requests ----
    // Triggered by: depth-colour toggle, base-model changes, etc.
    bool need_render = false;
    if (ortho_state.render_dirty && ortho_state.view_tex_ready) {
        need_render = true;
    }
    // Also detect base-model geometry changes (e.g. after voxel edit)
    if (!need_render && ortho_state.view_tex_ready && ortho_state._base_triangle_count > 0) {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it != items.end() && item_it->second->source_type == 2) {
            auto& item = *item_it->second;
            if (item.addon_base_node_id >= 0) {
                auto base_it = items.find(item.addon_base_node_id);
                if (base_it != items.end()) {
                    auto& base = *base_it->second;
                    size_t cur_count = base.cached_mesh.empty()
                                           ? base.source_triangles.size()
                                           : base.cached_mesh.size();
                    if (cur_count != ortho_state._base_triangle_count) {
                        ortho_state.render_dirty = true;
                        need_render = true;
                    }
                }
            }
        }
    }
    if (need_render) {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it != items.end() && item_it->second->source_type == 2) {
            auto& item = *item_it->second;
            if (item.addon_base_node_id >= 0) {
                auto base_it = items.find(item.addon_base_node_id);
                if (base_it != items.end()) {
                    perform_ortho_render(item, *base_it->second);
                }
            }
        }
    }

    // ---- Canvas display area ----
    if (!ortho_state.coord_map_ready) {
        ImGui::TextUnformatted("No render data. Open Setup to render.");
        ImGui::End();
        return;
    }

    // Compute display size from window width (stable, avoids ContentRegionAvail
    // fluctuations that can cause flicker from scrollbar appear/disappear).
    float avail_w = ImGui::GetWindowWidth() - 30.0f;
    float display_size = std::max(200.0f, avail_w);  // min 200px, no upper cap

    // Overlay params (offset, scale) are stored in a fixed 600px reference space.
    // They are NEVER auto-modified by window resize — only by user interaction
    // or explicit API calls.  At display time we convert:
    //   screen  = ref × (display_size / 600)
    //   render  = ref × (render_resolution / 600)
    constexpr float kRefDisplaySize = 600.0f;
    float ref_to_display = display_size / kRefDisplaySize;
    float display_to_ref = kRefDisplaySize / std::max(display_size, 1.0f);

    // Track actual display size for API state reporting
    ortho_state.canvas_display_size = display_size;

    int res = ortho_state.render_resolution;

    // Display the rendered view image, or dark fallback if not ready yet
    if (ortho_state.view_tex_ready && bgfx::isValid(ortho_state.view_tex)) {
        ImGui::Image(ImGui::toId(ortho_state.view_tex, 0, 0),
                     ImVec2(display_size, display_size));
    } else {
        // Dark canvas fallback (before GPU render completes, or on re-render)
        ImVec2 fb_cursor = ImGui::GetCursorScreenPos();
        ImDrawList* fb_dl = ImGui::GetWindowDrawList();
        fb_dl->AddRectFilled(fb_cursor,
            ImVec2(fb_cursor.x + display_size, fb_cursor.y + display_size),
            ImGui::ColorConvertFloat4ToU32(ImVec4(0.18f, 0.18f, 0.20f, 1.0f)));
        fb_dl->AddRect(fb_cursor,
            ImVec2(fb_cursor.x + display_size, fb_cursor.y + display_size),
            ImGui::ColorConvertFloat4ToU32(ImVec4(0.35f, 0.35f, 0.38f, 1.0f)));
        ImGui::Dummy(ImVec2(display_size, display_size));
    }

    // Get the actual screen-space rect of the displayed image/fallback
    ImVec2 img_cursor = ImGui::GetItemRectMin();
    ImVec2 img_end = ImGui::GetItemRectMax();
    // Recompute display_size from the actual rendered item
    display_size = img_end.x - img_cursor.x;

    // Invisible button over the entire image canvas.  It captures mouse
    // events so ImGui won't see "void" clicks as window-drag starts.
    // The overlay already has its own InvisibleButton when unlocked.
    // All guide-point / overlay-interaction code below uses raw
    // ImGui::IsMouse* checks, which are unaffected by InvisibleButton.
    ImGui::SetCursorScreenPos(img_cursor);
    ImGui::InvisibleButton("##canvas_interact",
                           ImVec2(display_size, display_size));

    ImDrawList* dl = ImGui::GetWindowDrawList();

    // ---- Overlay image rendering ----
    // Use ImGui::Image (same rendering path as the base model image) so
    // alpha blending via the tint colour composes correctly.
    // Save/restore cursor so GetItemRectMin/Max still refers to the base
    // model image for coordinate mapping below.
    ImVec2 prev_cursor_screen = ImGui::GetCursorScreenPos();
    if (bgfx::isValid(ortho_state.overlay_tex) && ortho_state.overlay_enabled) {
        float overlay_w = ortho_state.overlay_img_width * ortho_state.overlay_scale_x * ref_to_display;
        float overlay_h = ortho_state.overlay_img_height * ortho_state.overlay_scale_y * ref_to_display;
        ImVec2 overlay_pos = ImVec2(img_cursor.x + ortho_state.overlay_offset.x * ref_to_display,
                                    img_cursor.y + ortho_state.overlay_offset.y * ref_to_display);

        ImGui::SetCursorScreenPos(overlay_pos);
        ImGui::Image(
            ortho_state.overlay_tex,
            ImVec2(overlay_w, overlay_h),
            ImVec2(0, 0), ImVec2(1, 1),
            ImVec4(1.0f, 1.0f, 1.0f, ortho_state.blend_ratio));

        // Invisible button covering the entire overlay to capture left-clicks
        // and prevent the parent window from being dragged when the user
        // interacts with the overlay image.
        if (!ortho_state.overlay_locked) {
            ImGui::SetCursorScreenPos(overlay_pos);
            ImGui::InvisibleButton("##overlay_interact",
                                   ImVec2(overlay_w, overlay_h));
        }
    }
    ImGui::SetCursorScreenPos(prev_cursor_screen);

    // ---- Strand preview overlays ----
    // Project a world-space point onto the 2D image using the camera basis
    // that was computed in perform_ortho_render().
    auto project_world_to_image = [&](const vec3f& wp) -> ImVec2 {
        vec3f rel = {wp.x - ortho_state._center.x,
                      wp.y - ortho_state._center.y,
                      wp.z - ortho_state._center.z};
        float h = ortho_state.viewport_size * 0.5f;
        float rx = (rel.x * ortho_state._cam_right.x +
                    rel.y * ortho_state._cam_right.y +
                    rel.z * ortho_state._cam_right.z) / h;
        float ry = (rel.x * ortho_state._cam_up.x +
                    rel.y * ortho_state._cam_up.y +
                    rel.z * ortho_state._cam_up.z) / h;
        return ImVec2(img_cursor.x + (rx * 0.5f + 0.5f) * display_size,
                       img_cursor.y + (0.5f - ry * 0.5f) * display_size);
    };

    // --- Occlusion cache: avoid recomputing per-strand occlusion every frame ---
    // Invalidate when camera, model, or guide points change.
    {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it != items.end()) {
            auto& item = *item_it->second;

            // Build a simple hash of the camera + model state
            size_t state_hash = 0;
            auto hash_combine = [&](float v) {
                state_hash ^= std::hash<float>{}(v) + 0x9e3779b9 +
                              (state_hash << 6) + (state_hash >> 2);
            };
            hash_combine(ortho_state._center.x);
            hash_combine(ortho_state._center.y);
            hash_combine(ortho_state._center.z);
            hash_combine(ortho_state._cam_pos.x);
            hash_combine(ortho_state._cam_pos.y);
            hash_combine(ortho_state._cam_pos.z);
            hash_combine(ortho_state.projection_dir.x);
            hash_combine(ortho_state.projection_dir.y);
            hash_combine(ortho_state.projection_dir.z);
            hash_combine(ortho_state._cam_right.x);
            hash_combine(ortho_state._cam_right.y);
            hash_combine(ortho_state._cam_right.z);
            hash_combine(ortho_state._cam_up.x);
            hash_combine(ortho_state._cam_up.y);
            hash_combine(ortho_state._cam_up.z);
            hash_combine(ortho_state.viewport_size);
            state_hash ^=
                std::hash<size_t>{}(ortho_state._base_triangles.size());

            // Also hash guide point positions for all strands
            for (const auto& s : item.hair_strands) {
                hash_combine(static_cast<float>(s.guide_points.size()));
                for (const auto& gp : s.guide_points) {
                    hash_combine(gp.x);
                    hash_combine(gp.y);
                    hash_combine(gp.z);
                }
            }

            // Recompute occlusion cache if state changed
            if (state_hash != item._ortho_occlusion_hash) {
                item._ortho_occlusion_hash = state_hash;
                item._ortho_strand_occluded.resize(item.hair_strands.size());
                item._ortho_point_occluded.clear();
                item._ortho_point_occluded.resize(item.hair_strands.size());

                // DEBUG: occlusion recompute trigger
                static int occ_recompute_count = 0;
                occ_recompute_count++;
                bool log_this = (occ_recompute_count <= 5);
                if (log_this) {
                    fprintf(stderr, "[OCCL] recompute #%d: hash=%zu strands=%zu tris=%zu\n",
                        occ_recompute_count, state_hash,
                        item.hair_strands.size(), ortho_state._base_triangles.size());
                    fprintf(stderr, "[OCCL]   proj_dir=(%.4f,%.4f,%.4f) center=(%.2f,%.2f,%.2f) cam_pos=(%.1f,%.1f,%.1f) vp=%.2f\n",
                        ortho_state.projection_dir.x, ortho_state.projection_dir.y,
                        ortho_state.projection_dir.z,
                        ortho_state._center.x, ortho_state._center.y,
                        ortho_state._center.z,
                        ortho_state._cam_pos.x, ortho_state._cam_pos.y,
                        ortho_state._cam_pos.z, ortho_state.viewport_size);
                }

                // projection_dir = camera→center (inward), so it is already
                // the direction from viewer toward center — no negation.
                float dlen = std::sqrt(
                    ortho_state.projection_dir.x * ortho_state.projection_dir.x +
                    ortho_state.projection_dir.y * ortho_state.projection_dir.y +
                    ortho_state.projection_dir.z * ortho_state.projection_dir.z);
                vec3f ray_dir_n = {
                    ortho_state.projection_dir.x / dlen,
                    ortho_state.projection_dir.y / dlen,
                    ortho_state.projection_dir.z / dlen
                };

                // Reference plane on the viewer side (opposite to ray_dir_n
                // from center). This ensures t_wp > 0 for all visible points.
                const float kCamDist = 1000.0f;
                vec3f cam_plane_pt = {
                    ortho_state._center.x - ray_dir_n.x * kCamDist,
                    ortho_state._center.y - ray_dir_n.y * kCamDist,
                    ortho_state._center.z - ray_dir_n.z * kCamDist
                };

                const float kOccTolerance = 0.15f;
                const float half_vp = ortho_state.viewport_size * 0.5f;

                if (log_this) {
                    fprintf(stderr, "[OCCL]   ray_dir_n=(%.4f,%.4f,%.4f) cam_plane_pt=(%.1f,%.1f,%.1f)\n",
                        ray_dir_n.x, ray_dir_n.y, ray_dir_n.z,
                        cam_plane_pt.x, cam_plane_pt.y, cam_plane_pt.z);
                }

                for (size_t si = 0; si < item.hair_strands.size(); ++si) {
                    const auto& strand = item.hair_strands[si];
                    if (strand.guide_points.size() < 2) {
                        item._ortho_strand_occluded[si] = false;
                        continue;
                    }

                    auto& pt_occ = item._ortho_point_occluded[si];
                    pt_occ.resize(strand.guide_points.size(), false);

                    bool all_occluded = true;
                    int behind_cam = 0, outside_vp = 0, no_hit = 0, occluded_cnt = 0, visible_cnt = 0;
                    for (size_t pi = 0; pi < strand.guide_points.size(); ++pi) {
                        const auto& wp = strand.guide_points[pi];

                        // Per-point ray origin: project wp onto the camera
                        // plane. This ensures the ray passes through wp.
                        float t_wp =
                            (wp.x - cam_plane_pt.x) * ray_dir_n.x +
                            (wp.y - cam_plane_pt.y) * ray_dir_n.y +
                            (wp.z - cam_plane_pt.z) * ray_dir_n.z;
                        vec3f ray_origin = {
                            wp.x - t_wp * ray_dir_n.x,
                            wp.y - t_wp * ray_dir_n.y,
                            wp.z - t_wp * ray_dir_n.z
                        };

                        // t_wp is the (positive) distance from cam plane to wp
                        if (t_wp <= 0.0f) {
                            all_occluded = false;
                            behind_cam++;
                            continue;
                        }

                        // Viewport check: project wp onto near plane (through
                        // _center, perpendicular to ray_dir_n) and verify the
                        // projection lands inside the viewport rectangle.
                        float t_center = (wp.x - ortho_state._center.x) * ray_dir_n.x +
                                         (wp.y - ortho_state._center.y) * ray_dir_n.y +
                                         (wp.z - ortho_state._center.z) * ray_dir_n.z;
                        vec3f plane_pt = {
                            wp.x - t_center * ray_dir_n.x,
                            wp.y - t_center * ray_dir_n.y,
                            wp.z - t_center * ray_dir_n.z
                        };
                        vec3f rel = {plane_pt.x - ortho_state._center.x,
                                      plane_pt.y - ortho_state._center.y,
                                      plane_pt.z - ortho_state._center.z};
                        float rx = (rel.x * ortho_state._cam_right.x +
                                    rel.y * ortho_state._cam_right.y +
                                    rel.z * ortho_state._cam_right.z);
                        float ry = (rel.x * ortho_state._cam_up.x +
                                    rel.y * ortho_state._cam_up.y +
                                    rel.z * ortho_state._cam_up.z);
                        if (std::abs(rx) > half_vp || std::abs(ry) > half_vp) {
                            all_occluded = false;
                            outside_vp++;
                            continue;
                        }

                        // Raycast from the per-point origin along look dir.
                        // Find the first model surface hit.
                        float best_t = 1e30f;
                        for (const auto& tri : ortho_state._base_triangles) {
                            float t;
                            if (ray_triangle_intersect(ray_origin, ray_dir_n,
                                                        std::get<0>(tri),
                                                        std::get<1>(tri),
                                                        std::get<2>(tri), t)) {
                                if (t < best_t) best_t = t;
                            }
                        }
                        if (best_t >= 1e29f) {
                            no_hit++;
                        }

                        // DEBUG: log first few points of first strand
                        if (log_this && si == 0 && pi < 3) {
                            fprintf(stderr, "[OCCL]     pt[%zu] wp=(%.2f,%.2f,%.2f) ro=(%.1f,%.1f,%.1f) t_wp=%.4f best_t=%.4f => %s\n",
                                pi, wp.x, wp.y, wp.z,
                                ray_origin.x, ray_origin.y, ray_origin.z,
                                t_wp, best_t,
                                (best_t < t_wp - kOccTolerance) ? "OCCLUDED" : "visible");
                        }

                        // Occluded if a model surface lies between camera and wp
                        bool occluded = (best_t < t_wp - kOccTolerance);
                        pt_occ[pi] = occluded;
                        if (!occluded) {
                            all_occluded = false;
                            visible_cnt++;
                        } else {
                            occluded_cnt++;
                        }
                    }

                    if (log_this && si == 0) {
                        fprintf(stderr, "[OCCL]   strand[0] summary: behind_cam=%d outside_vp=%d "
                            "no_hit=%d occluded=%d visible=%d => all_occluded=%d\n",
                            behind_cam, outside_vp, no_hit, occluded_cnt, visible_cnt,
                            all_occluded ? 1 : 0);
                    }

                    item._ortho_strand_occluded[si] = all_occluded;
                }

                // DEBUG: summary of all strands
                if (log_this) {
                    int total_occ = 0;
                    for (size_t si = 0; si < item._ortho_strand_occluded.size(); ++si)
                        if (item._ortho_strand_occluded[si]) total_occ++;
                    fprintf(stderr, "[OCCL] TOTAL: %d/%zu strands fully occluded\n",
                        total_occ, item._ortho_strand_occluded.size());
                }
            }
        }
    }

    if (ortho_state.show_guide_curves) {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it != items.end()) {
            auto& item = *item_it->second;
            for (size_t si = 0; si < item.hair_strands.size(); ++si) {
                const auto& strand = item.hair_strands[si];
                if (strand.guide_points.size() < 2) continue;

                // Use cached occlusion result
                if (si < item._ortho_strand_occluded.size() &&
                    item._ortho_strand_occluded[si])
                    continue;  // Hide fully occluded strand

                bool is_active =
                    (item.guide_curve_drawing_active &&
                     item.active_guide_draw_strand == strand.uuid);
                bool is_hovered =
                    !item.hovered_strand_uuid.empty() &&
                    item.hovered_strand_uuid == strand.uuid;
                ImU32 color = (is_active || is_hovered)
                    ? ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 0.2f, 0.2f, 1.0f))
                    : ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 1.0f, 1.0f, 0.7f));

                auto sampled = sample_bezier_guide_curve(
                    strand.guide_points,
                    std::max(strand.guide_samples_per_segment, 1));
                for (size_t pi = 0; pi + 1 < sampled.size(); ++pi) {
                    ImVec2 a = project_world_to_image(sampled[pi]);
                    ImVec2 b = project_world_to_image(sampled[pi + 1]);
                    dl->AddLine(a, b, color, 1.5f);
                }

                ImU32 marker_color = (is_active || is_hovered)
                    ? ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 0.2f, 0.2f, 1.0f))
                    : ImGui::ColorConvertFloat4ToU32(ImVec4(0.8f, 0.8f, 0.8f, 0.5f));
                const auto& pt_occ =
                    (si < item._ortho_point_occluded.size())
                        ? item._ortho_point_occluded[si]
                        : std::vector<bool>{};
                for (size_t pi = 0; pi < strand.guide_points.size(); ++pi) {
                    // Use cached per-point occlusion
                    if (pi < pt_occ.size() && pt_occ[pi]) continue;
                    ImVec2 pimg = project_world_to_image(strand.guide_points[pi]);
                    dl->AddCircleFilled(pimg, 3.0f, marker_color);
                }
            }
        }
    }

    if (ortho_state.show_width_vectors) {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it != items.end()) {
            ImU32 green = ImGui::ColorConvertFloat4ToU32(ImVec4(0.2f, 0.9f, 0.3f, 1.0f));
            for (const auto& strand : item_it->second->hair_strands) {
                if (strand.guide_points.size() < 2) continue;
                for (const auto& wp : strand.width_points) {
                    if (wp.curve_id < 0.0f) continue;
                    float max_id = static_cast<float>(strand.guide_points.size() - 1);
                    if (wp.curve_id > max_id) continue;

                    size_t seg_idx = static_cast<size_t>(wp.curve_id);
                    if (seg_idx >= strand.guide_points.size() - 1)
                        seg_idx = strand.guide_points.size() - 2;
                    float t = wp.curve_id - static_cast<float>(seg_idx);

                    const auto& gpts = strand.guide_points;
                    size_t n = gpts.size();
                    vec3f p0 = gpts[seg_idx], p3 = gpts[seg_idx + 1], p1, p2;
                    if (seg_idx == 0)
                        p1 = p0 + (p3 - p0) * (1.0f / 3.0f);
                    else
                        p1 = p0 + (p3 - gpts[seg_idx - 1]) * (1.0f / 6.0f);
                    if (seg_idx + 2 >= n)
                        p2 = p3 - (p3 - p0) * (1.0f / 3.0f);
                    else
                        p2 = p3 - (gpts[seg_idx + 2] - p0) * (1.0f / 6.0f);

                    vec3f curve_pos = bezier_eval(p0, p1, p2, p3, t);
                    vec3f end_pos = curve_pos + wp.direction * wp.scale;
                    dl->AddLine(project_world_to_image(curve_pos),
                                project_world_to_image(end_pos), green, 1.0f);
                }
            }
        }
    }

    // ---- Mouse interaction (CPU raycasting) ----
    // Raycasting is always active within the image area.  Whether clicks
    // place guide/width points depends on the overlay lock state:
    //   locked   → click always passes through to the model
    //   unlocked → click on overlay = resize/drag; click on canvas = model
    ImVec2 mouse = ImGui::GetMousePos();
    bool mouse_in_image =
        (mouse.x >= img_cursor.x && mouse.x < img_cursor.x + display_size &&
         mouse.y >= img_cursor.y && mouse.y < img_cursor.y + display_size);

    // Compute overlay bounds (when visible) — converted from reference to screen space
    bool overlay_visible = ortho_state.overlay_enabled &&
                           bgfx::isValid(ortho_state.overlay_tex);
    float overlay_w = overlay_visible
        ? ortho_state.overlay_img_width * ortho_state.overlay_scale_x * ref_to_display : 0.0f;
    float overlay_h = overlay_visible
        ? ortho_state.overlay_img_height * ortho_state.overlay_scale_y * ref_to_display : 0.0f;
    ImVec2 overlay_pos = ImVec2(img_cursor.x + ortho_state.overlay_offset.x * ref_to_display,
                                img_cursor.y + ortho_state.overlay_offset.y * ref_to_display);
    ImVec2 overlay_end = ImVec2(overlay_pos.x + overlay_w,
                                overlay_pos.y + overlay_h);
    bool mouse_in_overlay = overlay_visible &&
        (mouse.x >= overlay_pos.x && mouse.x < overlay_end.x &&
         mouse.y >= overlay_pos.y && mouse.y < overlay_end.y);

    // ---- Corner resize handles on overlay (draw list, drawn after overlay) ----
    if (overlay_visible && !ortho_state.overlay_locked) {
        const float handle_r = 5.0f;
        ImU32 col_fill =
            ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 1.0f, 1.0f, 0.9f));
        ImU32 col_border =
            ImGui::ColorConvertFloat4ToU32(ImVec4(0.2f, 0.5f, 0.9f, 1.0f));

        ImVec2 corners[4] = {
            ImVec2(overlay_pos.x, overlay_pos.y),                  // 0: TL
            ImVec2(overlay_end.x, overlay_pos.y),                  // 1: TR
            ImVec2(overlay_pos.x, overlay_end.y),                  // 2: BL
            ImVec2(overlay_end.x, overlay_end.y)                   // 3: BR
        };
        for (int i = 0; i < 4; i++) {
            dl->AddCircleFilled(corners[i], handle_r, col_border);
            dl->AddCircleFilled(corners[i], handle_r - 1.5f, col_fill);
        }
    }

    // Four corner resize zones — centred on each corner so the visual
    // handle circles (radius 5 px) sit inside the hit-test area.
    const float corner_r = 12.0f;  // hit-test radius around corner centre
    int hovered_corner = -1;       // -1=none, 0=TL, 1=TR, 2=BL, 3=BR
    if (overlay_visible && !ortho_state.overlay_locked) {
        auto in_range = [](float v, float c, float r) -> bool {
            return v >= c - r && v < c + r;
        };
        // TL — centred at (overlay_pos.x, overlay_pos.y)
        if (in_range(mouse.x, overlay_pos.x, corner_r) &&
            in_range(mouse.y, overlay_pos.y, corner_r))
            hovered_corner = 0;
        // TR — centred at (overlay_end.x, overlay_pos.y)
        else if (in_range(mouse.x, overlay_end.x, corner_r) &&
                 in_range(mouse.y, overlay_pos.y, corner_r))
            hovered_corner = 1;
        // BL — centred at (overlay_pos.x, overlay_end.y)
        else if (in_range(mouse.x, overlay_pos.x, corner_r) &&
                 in_range(mouse.y, overlay_end.y, corner_r))
            hovered_corner = 2;
        // BR — centred at (overlay_end.x, overlay_end.y)
        else if (in_range(mouse.x, overlay_end.x, corner_r) &&
                 in_range(mouse.y, overlay_end.y, corner_r))
            hovered_corner = 3;
    }

    // ---- Cursor feedback (SDL directly — bgfx's imgui backend
    //     does not forward ImGui cursor requests to the OS). ----
    {
        static int s_active_cursor = -1;  // last cursor we set (-1 = none)
        int desired = -1;

        if (mouse_in_image) {
            if (hovered_corner == 0 || hovered_corner == 3)
                desired = SDL_SYSTEM_CURSOR_SIZENWSE;
            else if (hovered_corner == 1 || hovered_corner == 2)
                desired = SDL_SYSTEM_CURSOR_SIZENESW;
            else if (mouse_in_overlay && !ortho_state.overlay_locked)
                desired = SDL_SYSTEM_CURSOR_SIZEALL;
            else
                desired = SDL_SYSTEM_CURSOR_ARROW;
        }

        // Reset to arrow when mouse left the image area and we had
        // previously set a non-default cursor.
        if (desired == -1 && s_active_cursor != -1)
            desired = SDL_SYSTEM_CURSOR_ARROW;

        if (desired != -1 && desired != s_active_cursor) {
            s_active_cursor = desired;
            static SDL_Cursor* s_cached = nullptr;
            if (s_cached) SDL_FreeCursor(s_cached);
            s_cached = SDL_CreateSystemCursor(
                static_cast<SDL_SystemCursor>(desired));
            SDL_SetCursor(s_cached);
        }
    }

    // ---- Raycasting (always active) ----
    if (mouse_in_image && ortho_state.coord_map_ready) {
        int px = static_cast<int>((mouse.x - img_cursor.x) / display_size * res);
        int py = static_cast<int>((mouse.y - img_cursor.y) / display_size * res);
        px = std::max(0, std::min(px, res - 1));
        py = std::max(0, std::min(py, res - 1));

        vec3f hit_pos;
        bool valid = ortho_raycast(ortho_state, px, py, hit_pos);

        // Try curvature-preserving extrapolation when raycast misses
        if (!valid) {
            auto item_it = items.find(render_id);
            if (item_it != items.end()) {
                auto& item = *item_it->second;
                if (item.guide_curve_drawing_active &&
                    !item.active_guide_draw_strand.empty() &&
                    item.find_strand_by_uuid(item.active_guide_draw_strand)) {
                    auto& strand =
                        *item.find_strand_by_uuid(item.active_guide_draw_strand);
                    if (strand.guide_points.size() >= 2) {
                        // Construct camera ray from pixel
                        float u = (static_cast<float>(px) / res - 0.5f);
                        float v = (0.5f - static_cast<float>(py) / res);
                        vec3f plane_pt = {
                            ortho_state._center.x +
                                ortho_state._cam_right.x * u *
                                    ortho_state.viewport_size +
                                ortho_state._cam_up.x * v *
                                    ortho_state.viewport_size,
                            ortho_state._center.y +
                                ortho_state._cam_right.y * u *
                                    ortho_state.viewport_size +
                                ortho_state._cam_up.y * v *
                                    ortho_state.viewport_size,
                            ortho_state._center.z +
                                ortho_state._cam_right.z * u *
                                    ortho_state.viewport_size +
                                ortho_state._cam_up.z * v *
                                    ortho_state.viewport_size};
                        vec3f ray_dir = {ortho_state.projection_dir.x,
                                         ortho_state.projection_dir.y,
                                         ortho_state.projection_dir.z};
                        // Same camera-plane offset as ortho_raycast: the
                        // ray must start in front of the model, not on the
                        // center plane, otherwise closest-approach points
                        // on the camera side get clamped onto the plane.
                        float rl = std::sqrt(ray_dir.x * ray_dir.x +
                                             ray_dir.y * ray_dir.y +
                                             ray_dir.z * ray_dir.z);
                        if (rl > 1e-8f) {
                            ray_dir.x /= rl; ray_dir.y /= rl; ray_dir.z /= rl;
                            float cam_off =
                                (plane_pt.x - ortho_state._cam_pos.x) * ray_dir.x +
                                (plane_pt.y - ortho_state._cam_pos.y) * ray_dir.y +
                                (plane_pt.z - ortho_state._cam_pos.z) * ray_dir.z;
                            plane_pt.x -= ray_dir.x * cam_off;
                            plane_pt.y -= ray_dir.y * cam_off;
                            plane_pt.z -= ray_dir.z * cam_off;
                        }

                        vec3f extrapolated_pt;
                        if (extrapolate_guide_along_ray(
                                plane_pt, ray_dir, strand.guide_points,
                                extrapolated_pt,
                                ortho_state.viewport_size)) {
                            valid = true;
                            hit_pos = extrapolated_pt;
                        }
                    }
                }
            }
        }

        ortho_state.is_hovering_model = valid;
        ortho_state.hovered_px = px;
        ortho_state.hovered_py = py;
        if (valid) {
            ortho_state.hovered_world_pos = hit_pos;
            mouse_world_pos_valid = true;
            mouse_world_pos = {hit_pos.x, hit_pos.y, hit_pos.z};

            // Click-through to place guide/width points:
            // Only when overlay is locked, or mouse is outside the overlay
            // (but still inside the canvas area).
            bool pass_through =
                !overlay_visible || ortho_state.overlay_locked ||
                !mouse_in_overlay;
            if (pass_through &&
                ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                auto item_it = items.find(render_id);
                if (item_it != items.end()) {
                    auto& item = *item_it->second;

                    if (item.guide_curve_drawing_active &&
                        !item.active_guide_draw_strand.empty() &&
                        item.find_strand_by_uuid(item.active_guide_draw_strand)) {
                        push_undo_now(render_id, std::nullopt, "Add Guide Point");
                        auto& strand =
                            *item.find_strand_by_uuid(item.active_guide_draw_strand);
                        strand.guide_points.push_back(hit_pos);
                        strand.mesh_dirty = true;
                    } else if (item.width_editing_active &&
                               !item.active_width_edit_strand.empty()) {
                        auto* w_strand_ptr = item.find_strand_by_uuid(item.active_width_edit_strand);
                        if (w_strand_ptr) {
                            push_undo_now(render_id, std::nullopt, "Add Width Point");
                            int strand_idx = static_cast<int>(w_strand_ptr - item.hair_strands.data());
                            item.add_width_point_at(strand_idx, hit_pos);
                            w_strand_ptr->mesh_dirty = true;
                        }
                    }
                }
            }
        } else {
            if (ortho_state.is_hovering_model) {
                mouse_world_pos_valid = false;
            }
            ortho_state.is_hovering_model = false;
        }
    } else if (!mouse_in_image && ortho_state.is_hovering_model) {
        ortho_state.is_hovering_model = false;
    }

    // ---- Overlay interaction (left-drag body to move, left-drag corner to resize) ----
    if (overlay_visible && !ortho_state.overlay_locked && mouse_in_image) {
        bool on_corner = (hovered_corner >= 0);

        // Left-click on a corner → start resize; on body → start drag
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            if (on_corner) {
                ortho_state.resize_corner = hovered_corner;
                ortho_state.resize_start_mouse = mouse;
                ortho_state.resize_start_scale_x = ortho_state.overlay_scale_x;
		        ortho_state.resize_start_scale_y = ortho_state.overlay_scale_y;
                ortho_state.resize_start_offset = ortho_state.overlay_offset;
            } else if (mouse_in_overlay) {
                ortho_state.is_dragging_overlay = true;
                ortho_state.drag_start_mouse = mouse;
                ortho_state.drag_start_offset = ortho_state.overlay_offset;
            }
        }

        // Resize (4-corner, free-drag — independent X/Y scaling)
        if (ortho_state.resize_corner >= 0) {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                int corner = ortho_state.resize_corner;
                float iw = static_cast<float>(ortho_state.overlay_img_width);
                float ih = static_cast<float>(ortho_state.overlay_img_height);
                if (iw < 1e-6f) iw = 1.0f;
                if (ih < 1e-6f) ih = 1.0f;

                // Mouse delta in reference space
                float dmx_ref = (mouse.x - ortho_state.resize_start_mouse.x) *
                                display_to_ref;
                float dmy_ref = (mouse.y - ortho_state.resize_start_mouse.y) *
                                display_to_ref;

                float new_sx = ortho_state.resize_start_scale_x;
                float new_sy = ortho_state.resize_start_scale_y;
                float new_ox = ortho_state.resize_start_offset.x;
                float new_oy = ortho_state.resize_start_offset.y;

                // Each corner independently controls X and Y based on
                // mouse delta relative to the opposite (anchor) corner.
                switch (corner) {
                case 0: // TL — anchor is BR
                    new_sx = ortho_state.resize_start_scale_x -
                             dmx_ref / iw;
                    new_sy = ortho_state.resize_start_scale_y -
                             dmy_ref / ih;
                    new_ox = ortho_state.resize_start_offset.x + dmx_ref;
                    new_oy = ortho_state.resize_start_offset.y + dmy_ref;
                    break;
                case 1: // TR — anchor is BL
                    new_sx = ortho_state.resize_start_scale_x +
                             dmx_ref / iw;
                    new_sy = ortho_state.resize_start_scale_y -
                             dmy_ref / ih;
                    new_oy = ortho_state.resize_start_offset.y + dmy_ref;
                    break;
                case 2: // BL — anchor is TR
                    new_sx = ortho_state.resize_start_scale_x -
                             dmx_ref / iw;
                    new_sy = ortho_state.resize_start_scale_y +
                             dmy_ref / ih;
                    new_ox = ortho_state.resize_start_offset.x + dmx_ref;
                    break;
                case 3: // BR — anchor is TL
                default:
                    new_sx = ortho_state.resize_start_scale_x +
                             dmx_ref / iw;
                    new_sy = ortho_state.resize_start_scale_y +
                             dmy_ref / ih;
                    break;
                }

                ortho_state.overlay_scale_x =
                    std::max(0.1f, std::min(10.0f, new_sx));
                ortho_state.overlay_scale_y =
                    std::max(0.1f, std::min(10.0f, new_sy));
                ortho_state.overlay_offset.x = new_ox;
                ortho_state.overlay_offset.y = new_oy;
            } else {
                ortho_state.resize_corner = -1;
            }
        }
        // Drag (left-button on body) to move the overlay
        if (ortho_state.is_dragging_overlay) {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                // Mouse delta (screen pixels) → reference space
                ortho_state.overlay_offset.x =
                    ortho_state.drag_start_offset.x +
                    (mouse.x - ortho_state.drag_start_mouse.x) * display_to_ref;
                ortho_state.overlay_offset.y =
                    ortho_state.drag_start_offset.y +
                    (mouse.y - ortho_state.drag_start_mouse.y) * display_to_ref;
            } else {
                ortho_state.is_dragging_overlay = false;
            }
        }
    }

    // Clear overlay drag state when mouse leaves image area
    if (!mouse_in_image) {
        ortho_state.is_dragging_overlay = false;
        ortho_state.resize_corner = -1;
    }

    // ---- Footer toggles ----
    ImGui::Separator();
    ImGui::Checkbox(get_locale_cstr("label.show_guide_curves_2d"),
                    &ortho_state.show_guide_curves);
    ImGui::SameLine();
    ImGui::Checkbox(get_locale_cstr("label.show_width_vectors_2d"),
                    &ortho_state.show_width_vectors);

    ImGui::SameLine();
    if (ImGui::Checkbox(get_locale_cstr("label.export_guide_curves"),
                        &ortho_state.export_show_guide_curves))
        ortho_state.api_render_dirty = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.export_guide_curves"));

    ImGui::SameLine();
    if (!ortho_state.export_show_guide_curves) ImGui::BeginDisabled();
    if (ImGui::Checkbox(get_locale_cstr("label.export_color_code"),
                        &ortho_state.export_color_code_strands))
        ortho_state.api_render_dirty = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.export_color_code"));
    if (!ortho_state.export_show_guide_curves) ImGui::EndDisabled();

    ImGui::SameLine();
    const char* render_mode_names[] = {
        get_locale_cstr("label.render_mode_contour"),
        get_locale_cstr("label.render_mode_depth"),
        get_locale_cstr("label.render_mode_lighting"),
    };
    ImGui::SetNextItemWidth(120);
    if (ImGui::Combo("##render_mode", &ortho_state.ortho_render_mode,
                     render_mode_names, 3)) {
        if (ortho_state.view_tex_ready) {
            ortho_state.render_dirty = true;
        }
    }

    ImGui::SameLine();
    if (mouse_in_image) {
        int api_x = ortho_state.hovered_px;
        int api_y = ortho_state.hovered_py;
        if (ortho_state.is_hovering_model) {
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f),
                               " (%d,%d) \xE2\x86\x92 (%.1f, %.1f, %.1f)",
                               api_x, api_y,
                               ortho_state.hovered_world_pos.x,
                               ortho_state.hovered_world_pos.y,
                               ortho_state.hovered_world_pos.z);
        } else {
            ImGui::TextDisabled(" (%d,%d)", api_x, api_y);
        }
    }

    ImGui::End();
}

}  // namespace sinriv::ui::render
