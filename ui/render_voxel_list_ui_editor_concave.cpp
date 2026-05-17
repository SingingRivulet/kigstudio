#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>
#include <stb/stb_truetype.h>
#include <type_traits>
#include <unordered_set>
#include <variant>
#ifdef _WIN32
#include <windows.h>
#endif
#include "kigstudio/cgal/poisson_reconstruction.h"
#include "kigstudio/cgal/poisson_utils.h"
#include "kigstudio/utils/vec3.h"
#include "locale.h"
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {
void RenderVoxelList::render_concave_cone_editor(RenderVoxelItem& item) {
    enum class PickMode { None, Append, Replace, InsertBefore, InsertAfter };

    static PickMode pick_mode = PickMode::None;
    static int pick_index = -1;
    auto before = capture_snapshot(item);
    EditResult edit_result;

    // ===== apex =====
    if (ImGui::CollapsingHeader(get_locale_cstr("label.concave_cone_apex"),
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        auto r = edit_local_position_stepper(get_locale_cstr("label.apex"),
                                             item.concave_cone.apex);
        edit_result.activated |= r.activated;
        edit_result.deactivated_after_edit |= r.deactivated_after_edit;
        edit_result.value_changed |= r.value_changed;
    }

    // ===== error info =====
    if (!item.err_info.empty()) {
        ImGui::TextColored(ImVec4(1, 0, 0, 1), "%s", item.err_info.c_str());
    }

    // ===== vertices =====
    if (ImGui::CollapsingHeader(get_locale_cstr("label.concave_cone_vertices"),
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        item.concave_cone_expanded_vertices.clear();

        // --- append 模式 ---
        if (pick_mode == PickMode::Append) {
            if (ImGui::Button(get_locale_cstr("action.add_vertex_picking"))) {
                pick_mode = PickMode::None;
                pick_index = -1;
            }
        } else {
            if (ImGui::Button(get_locale_cstr("action.add_vertex"))) {
                pick_mode = PickMode::Append;
                pick_index = -1;
            }
        }

        // --- clear ---
        if (ImGui::Button(get_locale_cstr("action.clear_vertices"))) {
            push_undo_now(item.id, std::nullopt,
                          get_locale_string("action.clear_vertices"));
            item.concave_cone.base_vertices.clear();
            item.err_info.clear();
            item.concave_cone.check(item.err_info);
        }

        // ===== vertex list =====
        int erase_index = -1;

        for (int i = 0; i < (int)item.concave_cone.base_vertices.size(); ++i) {
            auto& v = item.concave_cone.base_vertices[i];

            char label[64];
            snprintf(label, sizeof(label), get_locale_cstr("label.vertex"), i,
                     i);

            if (ImGui::CollapsingHeader(label)) {
                item.concave_cone_expanded_vertices.push_back(i);
                // ImGui::Text("pos: %.3f %.3f %.3f", v.x, v.y, v.z);

                auto r = edit_local_position_stepper(
                    localize_id("label.edit", i).c_str(), v, 0.1f, false,
                    false);
                edit_result.activated |= r.activated;
                edit_result.deactivated_after_edit |= r.deactivated_after_edit;
                edit_result.value_changed |= r.value_changed;

                // --- delete ---
                if (ImGui::Button(localize_id("action.delete", i)
                                      .c_str())) {  // 需要增加二次确认
                    erase_index = i;
                }

                ImGui::SameLine();

                // --- replace ---
                if (pick_mode == PickMode::Replace && pick_index == i) {
                    if (ImGui::Button(
                            localize_id("action.replace_picking", i).c_str())) {
                        pick_mode = PickMode::None;
                        pick_index = -1;
                    }
                } else {
                    if (ImGui::Button(
                            localize_id("action.replace", i).c_str())) {
                        pick_mode = PickMode::Replace;
                        pick_index = i;
                    }
                }

                ImGui::SameLine();

                // --- insert after ---
                if (pick_mode == PickMode::InsertAfter && pick_index == i) {
                    if (ImGui::Button(
                            localize_id("action.insert_after_picking", i)
                                .c_str())) {
                        pick_mode = PickMode::None;
                        pick_index = -1;
                    }
                } else {
                    if (ImGui::Button(
                            localize_id("action.insert_after", i).c_str())) {
                        pick_mode = PickMode::InsertAfter;
                        pick_index = i;
                    }
                }

                ImGui::SameLine();

                // --- insert before ---
                if (pick_mode == PickMode::InsertBefore && pick_index == i) {
                    if (ImGui::Button(
                            localize_id("action.insert_before_picking", i)
                                .c_str())) {
                        pick_mode = PickMode::None;
                        pick_index = -1;
                    }
                } else {
                    if (ImGui::Button(
                            localize_id("action.insert_before", i).c_str())) {
                        pick_mode = PickMode::InsertBefore;
                        pick_index = i;
                    }
                }
            }
        }

        // ===== 删除处理 =====
        if (erase_index >= 0 &&
            erase_index < (int)item.concave_cone.base_vertices.size()) {
            push_undo_now(item.id, std::nullopt,
                          get_locale_string("action.delete"));
            item.concave_cone.base_vertices.erase(
                item.concave_cone.base_vertices.begin() + erase_index);

            item.err_info.clear();
            item.concave_cone.check(item.err_info);
        }

        // ===== picking 执行（统一入口）=====
        if (pick_mode != PickMode::None && mouse_world_pos_valid &&
            mouse_world_pos_picked)  // ⚠ 必须是点击瞬间
        {
            push_undo_now(item.id, std::nullopt,
                          get_locale_string("label.pick_point_by_mouse"));
            auto& verts = item.concave_cone.base_vertices;

            switch (pick_mode) {
                case PickMode::Append:
                    verts.push_back(mouse_world_pos);
                    break;

                case PickMode::Replace:
                    if (pick_index >= 0 && pick_index < (int)verts.size())
                        verts[pick_index] = mouse_world_pos;
                    pick_mode = PickMode::None;
                    break;

                case PickMode::InsertAfter:
                    if (pick_index >= 0 && pick_index < (int)verts.size())
                        verts.insert(verts.begin() + pick_index + 1,
                                     mouse_world_pos);
                    pick_mode = PickMode::None;
                    break;

                case PickMode::InsertBefore:
                    if (pick_index >= 0 && pick_index < (int)verts.size())
                        verts.insert(verts.begin() + pick_index,
                                     mouse_world_pos);
                    pick_mode = PickMode::None;
                    break;

                default:
                    break;
            }

            item.err_info.clear();
            item.concave_cone.check(item.err_info);

            // 一次性消费
            pick_index = -1;
        }

    } else {
        // 折叠时重置
        item.concave_cone_expanded_vertices.clear();
        pick_mode = PickMode::None;
        pick_index = -1;
    }

    if (edit_result.value_changed) {
        push_undo_now(item.id, before);
    }
    if (edit_result.value_changed) {
        push_undo_now(item.id, before,
                      get_locale_string("label.concave_cone_apex") + "/" +
                          get_locale_string("label.concave_cone_vertices"));
    }
    if (edit_result.activated) {
        begin_edit(item.id);
    }
    if (edit_result.deactivated_after_edit) {
        end_edit(item.id, get_locale_string("label.concave_cone_apex") + "/" +
                              get_locale_string("label.concave_cone_vertices"));
    }
}
}