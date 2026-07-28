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

            // 快捷键：Ctrl+C 复制，Ctrl+V 粘贴
            // 只在 Object Editor 有焦点且没有文本输入框捕获键盘时触发
            if (ImGui::IsWindowFocused(
                    ImGuiFocusedFlags_RootAndChildWindows) &&
                !ImGui::GetIO().WantTextInput) {
                if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_C)) {
                    copy_node_config(item);
                } else if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl |
                                                    ImGuiKey_V)) {
                    paste_node_config(item);
                }
            }

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
                ImGuiTabItemFlags flags_file_status = 0;
                ImGuiTabItemFlags flags_comment = 0;
                if (last_object_editor_tab != object_editor_tab) {
                    // 重置所有 Tab 的标志位
                    flags_collision = 0;
                    flags_voxel = 0;
                    flags_file_status = 0;
                    flags_comment = 0;

                    // 根据当前选中的 Tab 设置对应的选中标志
                    if (object_editor_tab == 0)
                        flags_collision = ImGuiTabItemFlags_SetSelected;
                    else if (object_editor_tab == 1)
                        flags_voxel = ImGuiTabItemFlags_SetSelected;
                    else if (object_editor_tab == 2)
                        flags_file_status = ImGuiTabItemFlags_SetSelected;
                    else if (object_editor_tab == 3)
                        flags_comment = ImGuiTabItemFlags_SetSelected;

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

                // ===== Tab: Voxel Picking (hidden in addon mode) =====
                if (item.source_type != 2) {
                    if (ImGui::BeginTabItem(get_locale_cstr("tab.voxel_picking"),
                                            nullptr, flags_voxel)) {
                        object_editor_tab = 1;
                        render_object_editor_voxel_tab_content(item);
                        ImGui::EndTabItem();
                    }
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip(get_locale_cstr("tooltip.voxel_picking"));
                    }
                } else if (object_editor_tab == 1) {
                    // 附加件模式下不显示体素选择Tab，回退到碰撞编辑Tab
                    object_editor_tab = 0;
                }
                if (item_it->second->root_id == item_it->second->id) {
                    if (ImGui::BeginTabItem(get_locale_cstr("tab.file_status"),
                                            nullptr, flags_file_status)) {
                        object_editor_tab = 2;
                        render_file_status_tab(item);
                        ImGui::EndTabItem();
                    }
                }

                // ===== Tab: Comment =====
                if (ImGui::BeginTabItem(get_locale_cstr("tab.comment"),
                                        nullptr, flags_comment)) {
                    object_editor_tab = 3;
                    render_object_editor_comment_tab_content(item);
                    ImGui::EndTabItem();
                }

                ImGui::EndTabBar();
            }

            if (is_updating)
                ImGui::EndDisabled();
        }
    }
    ImGui::End();
}

void RenderVoxelList::render_object_editor_comment_tab_content(
    RenderVoxelItem& item) {
    // 标题输入框
    char title_buf[256];
    std::strncpy(title_buf, item.title.c_str(), sizeof(title_buf) - 1);
    title_buf[sizeof(title_buf) - 1] = '\0';
    if (ImGui::InputText(get_locale_cstr("label.title"), title_buf,
                         sizeof(title_buf))) {
        item.title = title_buf;
    }

    ImGui::Separator();

    // 注释文本输入框
    ImGui::TextUnformatted(get_locale_cstr("label.comment_text"));
    std::vector<char> comment_buf(item.comment_text.begin(),
                                  item.comment_text.end());
    comment_buf.push_back('\0');
    // 预留足够空间以容纳用户输入
    const size_t kMaxCommentSize = 4096;
    if (comment_buf.size() < kMaxCommentSize) {
        comment_buf.resize(kMaxCommentSize, '\0');
    }
    if (ImGui::InputTextMultiline("##comment_text", comment_buf.data(),
                                  comment_buf.size(),
                                  ImVec2(-FLT_MIN, 120.0f))) {
        item.comment_text = comment_buf.data();
    }
}

void RenderVoxelList::copy_node_config(const RenderVoxelItem& item) {
    auto snapshot = capture_snapshot(item);
    cJSON* config_json = snapshot_to_json(snapshot);
    cJSON* wrapper = cJSON_CreateObject();
    cJSON_AddItemToObject(wrapper, "kigstudio_node_config", config_json);
    char* json_str = cJSON_Print(wrapper);
    if (json_str) {
        SDL_SetClipboardText(json_str);
        cJSON_free(json_str);
    }
    cJSON_Delete(wrapper);
}

void RenderVoxelList::paste_node_config(RenderVoxelItem& item) {
    const char* clipboard = SDL_GetClipboardText();
    if (clipboard && clipboard[0] != '\0') {
        cJSON* wrapper = cJSON_Parse(clipboard);
        if (wrapper) {
            cJSON* config_json =
                cJSON_GetObjectItem(wrapper, "kigstudio_node_config");
            if (config_json) {
                auto snapshot = snapshot_from_json(config_json);
                if (snapshot.has_value()) {
                    push_undo_now(item.id, std::nullopt, "Paste config");
                    apply_snapshot(item, snapshot.value());
                    item.dirty = true;
                }
            }
            cJSON_Delete(wrapper);
        }
    }
    if (clipboard) {
        SDL_free(const_cast<char*>(clipboard));
    }
}

void RenderVoxelList::render_object_editor_toolbar(RenderVoxelItem& item) {
    if (ImGui::Button(get_locale_cstr("action.delete"))) {
        pending_delete_item_id = item.id;
        show_delete_confirm = true;
    }
    ImGui::SameLine();
    const std::string export_popup_title =
        localize_id("dialog.choose_export_method", item.id);
    if (ImGui::Button(get_locale_cstr("action.save_as_stl"))) {
        ImGui::OpenPopup(export_popup_title.c_str());
    }
    if (ImGui::BeginPopupModal(export_popup_title.c_str(), nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted(get_locale_cstr("dialog.choose_export_method"));

        // Export mode selection
        if (item.source_type == 2) {
            // 附加件节点：可直接导出 loft 网格，或走 SDF 平滑导出
            if (export_stl_mode == 0) export_stl_mode = 2;
            ImGui::RadioButton(get_locale_cstr("label.export_mode_mesh"),
                               &export_stl_mode, 2);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    get_locale_cstr("tooltip.export_mode_mesh"));
            }
            ImGui::RadioButton(get_locale_cstr("label.export_mode_smooth"),
                               &export_stl_mode, 1);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    get_locale_cstr("tooltip.export_mode_smooth"));
            }
        } else {
            ImGui::RadioButton(get_locale_cstr("label.export_mode_standard"),
                               &export_stl_mode, 0);
            ImGui::RadioButton(get_locale_cstr("label.export_mode_smooth"),
                               &export_stl_mode, 1);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    get_locale_cstr("tooltip.export_mode_smooth"));
            }
        }

        ImGui::Separator();

        // Simplification option
        ImGui::Checkbox(get_locale_cstr("label.simplify_model"),
                        &export_stl_simplify);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(get_locale_cstr("tooltip.simplify_model"));
        }
        if (export_stl_simplify) {
            ImGui::Indent();
            ImGui::SliderFloat(get_locale_cstr("label.simplification_ratio"),
                               &export_stl_simplify_ratio, 0.01f, 1.0f, "%.2f");
            ImGui::TextUnformatted(
                get_locale_cstr("hint.simplification_ratio"));
            ImGui::Unindent();
        }

        if (export_stl_mode == 1) {
            ImGui::SliderInt(get_locale_cstr("label.subdivisions_ratio"),
                             &export_stl_subdivisions, 1, 8);
        }

        ImGui::Separator();

        if (ImGui::Button(get_locale_cstr("action.save_as_stl"))) {
            ImGui::CloseCurrentPopup();
            const char* filters[] = {"*.stl"};
            const char* file = tinyfd_saveFileDialog(
                utf8_to_ansi(get_locale_cstr("dialog.save_voxel_as_stl"))
                    .c_str(),
                "node_voxel.stl", 1, filters,
                utf8_to_ansi(get_locale_cstr("dialog.stl_files")).c_str());
            if (file) {
                queue_export_stl(item.id, tinyfd_path_to_utf8(file),
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

    // SDF 预览渲染按钮
    if (item.sdf_data) {
        ImGui::SameLine();
        const std::string sdf_preview_popup_title =
            localize_id("dialog.sdf_preview", item.id);
        if (ImGui::Button(get_locale_cstr("action.render_sdf"))) {
            ImGui::OpenPopup(sdf_preview_popup_title.c_str());
        }
        if (ImGui::BeginPopupModal(sdf_preview_popup_title.c_str(), nullptr,
                                   ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextUnformatted(get_locale_cstr("dialog.sdf_preview_desc"));

            ImGui::Separator();

            // Simplification option
            ImGui::Checkbox(get_locale_cstr("label.simplify_model"),
                            &export_stl_simplify);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(get_locale_cstr("tooltip.simplify_model"));
            }
            if (export_stl_simplify) {
                ImGui::Indent();
                ImGui::SliderFloat(
                    get_locale_cstr("label.simplification_ratio"),
                    &export_stl_simplify_ratio, 0.01f, 1.0f, "%.2f");
                ImGui::TextUnformatted(
                    get_locale_cstr("hint.simplification_ratio"));
                ImGui::Unindent();
            }

            ImGui::SliderInt(get_locale_cstr("label.subdivisions_ratio"),
                             &export_stl_subdivisions, 1, 8);

            ImGui::Separator();

            if (ImGui::Button(get_locale_cstr("action.render_sdf"))) {
                ImGui::CloseCurrentPopup();
                queue_export_stl(item.id, "", 1, export_stl_simplify,
                                 export_stl_simplify_ratio,
                                 export_stl_subdivisions, false);
            }
            ImGui::SameLine();
            if (ImGui::Button(get_locale_cstr("action.cancel"))) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(get_locale_cstr("tooltip.render_sdf"));
        }
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
    ImGui::SameLine();
    if (ImGui::Button(get_locale_cstr("action.copy"))) {
        copy_node_config(item);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Copy collision edit config to clipboard as JSON\n"
                          "Shortcut: Ctrl+C");
    }
    ImGui::SameLine();
    if (ImGui::Button(get_locale_cstr("action.paste"))) {
        paste_node_config(item);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Paste collision edit config from clipboard JSON\n"
                          "Shortcut: Ctrl+V");
    }
    ImGui::Separator();

    ImGui::Checkbox(get_locale_cstr("label.auto_segment_update"),
                    &item.auto_segment_update);
    ImGui::Checkbox(get_locale_cstr("label.show_origin_mesh"),
                    &item.showOriginMesh);

    // 附加件模式：用显露/拆分勾选框替代下拉模式选择
    if (item.source_type == 2) {
        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.3f, 0.8f, 0.9f, 1.0f), "%s",
                           get_locale_cstr("label.source_addon"));

        ImGui::Checkbox(get_locale_cstr("label.addon_reveal"),
                        &item.addon_reveal);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(get_locale_cstr("tooltip.addon_reveal"));
        }
        if (item.addon_reveal) {
            ImGui::Indent();
            ImGui::Checkbox(get_locale_cstr("label.addon_sdf_boolean"),
                            &item.addon_sdf_boolean);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    get_locale_cstr("tooltip.addon_sdf_boolean"));
            }
            ImGui::Unindent();
        }

        ImGui::Checkbox(get_locale_cstr("label.addon_split"),
                        &item.addon_split);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(get_locale_cstr("tooltip.addon_split"));
        }
        if (item.addon_split) {
            ImGui::Indent();
            ImGui::Checkbox(get_locale_cstr("label.addon_sdf_split"),
                            &item.addon_sdf_split);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    get_locale_cstr("tooltip.addon_sdf_split"));
            }
            ImGui::Unindent();
        }

        ImGui::Separator();
        return;  // 附加件模式不显示后续的segment_mode下拉等UI
    }

    if (item.mesh_only) {
        // mesh_only 模型支持 Plane、Repair Mesh、Subdivide Mesh 与 Silhouette 四种处理模式
        const char* mesh_only_mode_names[] = {
            get_locale_cstr("mode.plane"),
            get_locale_cstr("mode.repair"),
            get_locale_cstr("mode.subdivide"),
            get_locale_cstr("mode.silhouette")};
        const enum RenderVoxelItem::SegmentMode mesh_only_modes[] = {
            RenderVoxelItem::PLANE,
            RenderVoxelItem::REPAIR_MESH,
            RenderVoxelItem::SUBDIVIDE_MESH,
            RenderVoxelItem::SILHOUETTE};
        int current_mesh_only_mode = 0;
        if (item.segment_mode == RenderVoxelItem::REPAIR_MESH)
            current_mesh_only_mode = 1;
        else if (item.segment_mode == RenderVoxelItem::SUBDIVIDE_MESH)
            current_mesh_only_mode = 2;
        else if (item.segment_mode == RenderVoxelItem::SILHOUETTE)
            current_mesh_only_mode = 3;
        if (ImGui::Combo(get_locale_cstr("label.segment_mode"),
                         &current_mesh_only_mode, mesh_only_mode_names,
                         IM_ARRAYSIZE(mesh_only_mode_names))) {
            push_undo_now(item.id, std::nullopt,
                          get_locale_string("label.segment_mode"));
            item.segment_mode = mesh_only_modes[current_mesh_only_mode];
        }
    } else {
        const char* segment_mode_names[] = {
            get_locale_cstr("mode.collision"),
            get_locale_cstr("mode.plane"),
            get_locale_cstr("mode.concave_cone"),
            get_locale_cstr("mode.split_disconnected"),
            get_locale_cstr("mode.neighbor"),
            get_locale_cstr("mode.fill_interior"),
            get_locale_cstr("mode.chain"),
            get_locale_cstr("mode.sdf_node_split"),
            get_locale_cstr("mode.subdivide")};
        const enum RenderVoxelItem::SegmentMode segment_modes[] = {
            RenderVoxelItem::COLLISION,    RenderVoxelItem::PLANE,
            RenderVoxelItem::CONCAVE_CONE, RenderVoxelItem::SPLIT_DISCONNECTED,
            RenderVoxelItem::NEIGHBOR,     RenderVoxelItem::FILL_INTERIOR,
            RenderVoxelItem::CHAIN,        RenderVoxelItem::SDF_NODE_SPLIT,
            RenderVoxelItem::SUBDIVIDE_MESH};
        int current_segment_mode = 0;
        for (int i = 0; i < static_cast<int>(IM_ARRAYSIZE(segment_modes));
             ++i) {
            if (segment_modes[i] == item.segment_mode) {
                current_segment_mode = i;
                break;
            }
        }
        if (ImGui::Combo(get_locale_cstr("label.segment_mode"),
                         &current_segment_mode, segment_mode_names,
                         IM_ARRAYSIZE(segment_mode_names))) {
            push_undo_now(item.id, std::nullopt,
                          get_locale_string("label.segment_mode"));
            item.segment_mode = segment_modes[current_segment_mode];
        }
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
            case RenderVoxelItem::SDF_NODE_SPLIT:
                tooltip_key = "tooltip.mode.sdf_node_split";
                break;
            case RenderVoxelItem::REPAIR_MESH:
                tooltip_key = "tooltip.mode.repair";
                break;
            case RenderVoxelItem::SUBDIVIDE_MESH:
                tooltip_key = "tooltip.mode.subdivide";
                break;
            case RenderVoxelItem::SILHOUETTE:
                tooltip_key = "tooltip.mode.silhouette";
                break;
        }
        if (tooltip_key) {
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
            ImGui::TextUnformatted(get_locale_cstr(tooltip_key));
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }
    }

    if (item.segment_mode == RenderVoxelItem::PLANE) {
        render_plane_editor(item);
    } else if (item.segment_mode == RenderVoxelItem::COLLISION) {
        render_collision_body_editor(item);
    } else if (item.segment_mode == RenderVoxelItem::CONCAVE_CONE) {
        render_concave_cone_editor(item);
    } else if (item.segment_mode == RenderVoxelItem::NEIGHBOR) {
        ImGui::DragInt(get_locale_cstr("label.neighbor_max_distance"),
                       &item.neighbor_max_distance, 1, 1, 100);
    } else if (item.segment_mode == RenderVoxelItem::FILL_INTERIOR) {
        ImGui::TextUnformatted(get_locale_cstr("tooltip.mode.fill_interior"));
    } else if (item.segment_mode == RenderVoxelItem::CHAIN) {
        render_object_editor_chain_mode(item);
    } else if (item.segment_mode == RenderVoxelItem::SDF_NODE_SPLIT) {
        render_object_editor_sdf_node_split_mode(item);
    } else if (item.segment_mode == RenderVoxelItem::REPAIR_MESH) {
        render_object_editor_repair_mode(item);
    } else if (item.segment_mode == RenderVoxelItem::SUBDIVIDE_MESH) {
        render_object_editor_subdivide_mode(item);
    } else if (item.segment_mode == RenderVoxelItem::SILHOUETTE) {
        render_object_editor_silhouette_mode(item);
    }
}

void RenderVoxelList::render_object_editor_sdf_node_split_mode(
    RenderVoxelItem& item) {
    std::vector<std::pair<int, std::string>> candidates;
    if (item.manager) {
        for (auto& [other_id, other] : item.manager->items) {
            if (other_id != item.id && other->root_id != item.root_id &&
                other->sdf_data) {
                candidates.push_back(
                    {other_id, "Node " + std::to_string(other_id)});
            }
        }
    }
    int current_target = item.sdf_split_target_id;
    int selected_idx = -1;
    std::vector<const char*> labels;
    labels.push_back("<None>");
    for (size_t i = 0; i < candidates.size(); ++i) {
        labels.push_back(candidates[i].second.c_str());
        if (candidates[i].first == current_target)
            selected_idx = static_cast<int>(i);
    }
    int combo_idx = selected_idx >= 0 ? selected_idx + 1 : 0;
    if (ImGui::Combo(get_locale_cstr("label.sdf_split_target"), &combo_idx,
                     labels.data(), static_cast<int>(labels.size()))) {
        push_undo_now(item.id, std::nullopt,
                      get_locale_string("label.sdf_split_target"));
        item.sdf_split_target_id =
            (combo_idx > 0) ? candidates[combo_idx - 1].first : -1;
    }
    ImGui::Separator();
    ImGui::TextUnformatted("Source Transform");
    auto before_transform = capture_snapshot(item);
    EditResult transform_edit_result;
    auto translation_result =
        edit_vec3_stepper(get_locale_cstr("label.position"),
                          item.sdf_split_translation, 0.5f);
    transform_edit_result.activated |= translation_result.activated;
    transform_edit_result.deactivated_after_edit |=
        translation_result.deactivated_after_edit;
    transform_edit_result.value_changed |= translation_result.value_changed;

    auto rotation_result =
        edit_vec3_stepper(get_locale_cstr("label.rotation_deg"),
                          item.sdf_split_rotation, 1.0f);
    transform_edit_result.activated |= rotation_result.activated;
    transform_edit_result.deactivated_after_edit |=
        rotation_result.deactivated_after_edit;
    transform_edit_result.value_changed |= rotation_result.value_changed;

    auto scale_result =
        edit_vec3_stepper("Scale", item.sdf_split_scale, 0.1f);
    transform_edit_result.activated |= scale_result.activated;
    transform_edit_result.deactivated_after_edit |=
        scale_result.deactivated_after_edit;
    transform_edit_result.value_changed |= scale_result.value_changed;

    item.sdf_split_scale.x = std::max(0.001f, item.sdf_split_scale.x);
    item.sdf_split_scale.y = std::max(0.001f, item.sdf_split_scale.y);
    item.sdf_split_scale.z = std::max(0.001f, item.sdf_split_scale.z);
    if (transform_edit_result.activated) {
        begin_edit(item.id);
    }
    if (transform_edit_result.deactivated_after_edit) {
        end_edit(item.id, "Source Transform");
    } else if (transform_edit_result.value_changed) {
        push_undo_now(item.id, before_transform, "Source Transform");
    }
}

void RenderVoxelList::render_object_editor_subdivide_mode(
    RenderVoxelItem& item) {
    ImGui::DragInt(get_locale_cstr("label.subdivide_level"),
                   &item.subdivide_level, 1, 1, 10);
}

void RenderVoxelList::render_object_editor_silhouette_mode(
    RenderVoxelItem& item) {
    item.showSilhouetteCenter = true;

    // 轮廓中心
    auto center_result =
        edit_vec3_stepper(get_locale_cstr("label.silhouette_center"),
                          item.silhouette_center, 0.1f);
    if (center_result.deactivated_after_edit) {
        push_undo_now(item.id, std::nullopt, "Silhouette Center");
    }

    // Shape mode selector
    const char* shape_mode_names[] = {
        get_locale_cstr("label.silhouette_shape.icosahedron"),
        get_locale_cstr("label.silhouette_shape.delaunay"),
    };
    int shape_mode = static_cast<int>(item.silhouette_shape_mode);
    if (ImGui::Combo(get_locale_cstr("label.silhouette_shape_mode"),
                     &shape_mode, shape_mode_names,
                     IM_ARRAYSIZE(shape_mode_names))) {
        push_undo_now(item.id, std::nullopt, "Silhouette Shape Mode");
        item.silhouette_shape_mode =
            static_cast<SilhouetteShapeMode>(shape_mode);
    }

    // Edge subdivision: only meaningful for Delaunay sphere mode.
    if (item.silhouette_shape_mode == SilhouetteShapeMode::DELAUNAY_SPHERE) {
        const float btn_w = ImGui::GetFrameHeight();
        ImGui::TextUnformatted(
            get_locale_cstr("label.silhouette_edge_subdiv"));
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                get_locale_cstr("tooltip.silhouette_edge_subdiv"));
        }
        ImGui::SameLine();
        if (ImGui::Button("-##silhouette_edge_subdiv", ImVec2(btn_w, 0))) {
            if (item.silhouette_edge_subdiv > 0) {
                --item.silhouette_edge_subdiv;
                push_undo_now(item.id, std::nullopt,
                              "Silhouette Edge Subdivision");
            }
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0f);
        if (ImGui::InputInt("##silhouette_edge_subdiv_val",
                            &item.silhouette_edge_subdiv, 0, 0,
                            ImGuiInputTextFlags_CharsDecimal)) {
            if (item.silhouette_edge_subdiv < 0)
                item.silhouette_edge_subdiv = 0;
            push_undo_now(item.id, std::nullopt,
                          "Silhouette Edge Subdivision");
        }
        ImGui::SameLine();
        if (ImGui::Button("+##silhouette_edge_subdiv", ImVec2(btn_w, 0))) {
            ++item.silhouette_edge_subdiv;
            push_undo_now(item.id, std::nullopt,
                          "Silhouette Edge Subdivision");
        }
    }

    const float btn_w = ImGui::GetFrameHeight();
    ImGui::TextUnformatted(get_locale_cstr("label.silhouette_subdivision"));
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(get_locale_cstr("tooltip.silhouette_subdivision"));
    }
    ImGui::SameLine();
    if (ImGui::Button("-##silhouette_subdiv", ImVec2(btn_w, 0))) {
        if (item.silhouette_subdivision > 1) {
            --item.silhouette_subdivision;
            push_undo_now(item.id, std::nullopt, "Silhouette Subdivision");
        }
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80.0f);
    if (ImGui::InputInt("##silhouette_subdiv_val",
                        &item.silhouette_subdivision, 0, 0,
                        ImGuiInputTextFlags_CharsDecimal)) {
        if (item.silhouette_subdivision < 1)
            item.silhouette_subdivision = 1;
        push_undo_now(item.id, std::nullopt, "Silhouette Subdivision");
    }
    ImGui::SameLine();
    if (ImGui::Button("+##silhouette_subdiv", ImVec2(btn_w, 0))) {
        ++item.silhouette_subdivision;
        push_undo_now(item.id, std::nullopt, "Silhouette Subdivision");
    }

    // Inner wall radius
    ImGui::TextUnformatted(get_locale_cstr("label.inner_wall_radius"));
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(get_locale_cstr("tooltip.inner_wall_radius"));
    }
    ImGui::SameLine();
    if (ImGui::Button("-##inner_wall", ImVec2(btn_w, 0))) {
        item.inner_wall_radius =
            std::max(0.0f, item.inner_wall_radius - 0.5f);
        push_undo_now(item.id, std::nullopt, "Inner Wall Radius");
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80.0f);
    if (ImGui::InputFloat("##inner_wall_val", &item.inner_wall_radius, 0.0f,
                          0.0f, "%.1f", ImGuiInputTextFlags_CharsDecimal)) {
        if (item.inner_wall_radius < 0.0f)
            item.inner_wall_radius = 0.0f;
        push_undo_now(item.id, std::nullopt, "Inner Wall Radius");
    }
    ImGui::SameLine();
    if (ImGui::Button("+##inner_wall", ImVec2(btn_w, 0))) {
        item.inner_wall_radius += 0.5f;
        push_undo_now(item.id, std::nullopt, "Inner Wall Radius");
    }
    ImGui::SameLine();
    if (ImGui::Button(get_locale_cstr("label.inner_wall_reset"))) {
        // do_segment 生成的 mesh_only 子节点没有 origin mesh，回退用当前 mesh
        const auto& distance_mesh =
            item.origin_mesh_renderer.empty() ? item.mesh_renderer
                                              : item.origin_mesh_renderer;
        float nearest = distance_mesh.get_min_distance(item.silhouette_center);
        item.inner_wall_radius = std::max(0.0f, nearest - 1.f);
        push_undo_now(item.id, std::nullopt, "Inner Wall Radius");
    }

    // Simplify checkbox + slider
    bool simplify_enabled = (item.simplify_ratio >= 0.0f);
    if (ImGui::Checkbox("##simplify_enable", &simplify_enabled)) {
        item.simplify_ratio = simplify_enabled ? 0.15f : -1.0f;
        push_undo_now(item.id, std::nullopt, "Simplify");
    }
    ImGui::SameLine();
    ImGui::TextUnformatted(get_locale_cstr("label.simplify_ratio"));
    ImGui::SameLine();
    if (!simplify_enabled) ImGui::BeginDisabled();
    ImGui::SetNextItemWidth(120.0f);
    if (ImGui::SliderFloat("##simplify_slider", &item.simplify_ratio, 0.01f,
                           1.0f, "%.2f")) {
        if (item.simplify_ratio < 0.01f) item.simplify_ratio = 0.01f;
        push_undo_now(item.id, std::nullopt, "Simplify Ratio");
    }
    if (!simplify_enabled) ImGui::EndDisabled();
}

void RenderVoxelList::render_object_editor_repair_mode(
    RenderVoxelItem& item) {
    const char* repair_mode_names[] = {
        get_locale_cstr("mode.repair.alpha_wrap"),
        get_locale_cstr("mode.repair.fill_holes"),
        get_locale_cstr("mode.repair.stitch_borders"),
        get_locale_cstr("mode.repair.merge_duplicate_vertices"),
        get_locale_cstr("mode.repair.orient_volume")};
    const enum RenderVoxelItem::RepairMeshMode repair_modes[] = {
        RenderVoxelItem::ALPHA_WRAP,
        RenderVoxelItem::FILL_HOLES,
        RenderVoxelItem::STITCH_BORDERS,
        RenderVoxelItem::MERGE_DUPLICATE_VERTICES,
        RenderVoxelItem::ORIENT_VOLUME};
    int current_repair_mode = static_cast<int>(item.repair_mode);
    if (ImGui::Combo(get_locale_cstr("label.repair_mode"), &current_repair_mode,
                     repair_mode_names, IM_ARRAYSIZE(repair_mode_names))) {
        push_undo_now(item.id, std::nullopt,
                      get_locale_string("label.repair_mode"));
        item.repair_mode = repair_modes[current_repair_mode];
    }

    if (item.repair_mode == RenderVoxelItem::ALPHA_WRAP) {
        ImGui::DragFloat(get_locale_cstr("label.alpha_wrap_alpha"),
                         &item.alpha_wrap_alpha, 0.01f, 0.01f, 100.0f);
        ImGui::DragFloat(get_locale_cstr("label.alpha_wrap_offset"),
                         &item.alpha_wrap_offset, 0.001f, 0.001f, 10.0f);
    }
}

void RenderVoxelList::render_object_editor_voxel_tab_content(
    RenderVoxelItem& item) {
    ImGui::Checkbox(get_locale_cstr("label.voxel_picking"),
                    &item.voxel_picking_enabled);
    if (item.voxel_picking_enabled) {
        if (!item.surface_cache_ready && !item.surface_cache_computing) {
            item.surface_cache_computing = true;
            item.surface_cache_progress = 0.0f;
            // 在后台线程初始化表面缓存
            std::thread([this, id = item.id]() {
                auto it = this->items.find(id);
                if (it == this->items.end())
                    return;
                auto& target = *it->second;
                auto surface = target.voxel_grid_data.getSurfaceVoxels();
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
            ImGui::DragFloat(get_locale_cstr("label.pick_range"),
                             &item.voxel_pick_range, 0.1f, 0.1f, 20.0f, "%.1f");
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

        if (ImGui::Button(get_locale_cstr("action.save_marked_voxels"))) {
            const char* filters[] = {"*.vxgrid"};
            const char* file = tinyfd_saveFileDialog(
                get_locale_cstr("dialog.save_marked_voxels"), "marked.vxgrid",
                1, filters, get_locale_cstr("dialog.marked_voxels_file"));
            if (file) {
                std::string error;
                if (!sinriv::kigstudio::save(file, item.marked_voxels,
                                             &error)) {
                    tinyfd_messageBox("Error",
                                      utf8_to_ansi(error.c_str()).c_str(), "ok",
                                      "error", 1);
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.load_marked_voxels"))) {
            const char* filters[] = {"*.vxgrid"};
            const char* file = tinyfd_openFileDialog(
                get_locale_cstr("dialog.load_marked_voxels"), "", 1, filters,
                get_locale_cstr("dialog.marked_voxels_file"), 0);
            if (file) {
                this->push_marked_undo_now(item.id, "Load marked");
                if (!sinriv::kigstudio::load(file, item.marked_voxels)) {
                    tinyfd_messageBox(
                        "Error",
                        utf8_to_ansi(
                            get_locale_cstr("error.load_marked_failed"))
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
    ImGui::Checkbox(get_locale_cstr("label.disable_camera_on_pick"),
                    &disable_camera_on_pick);
    ImGui::DragFloat(get_locale_cstr("label.mouse_highlight_range"),
                     &mouse_highlight_range, 0.1f, 0.0f, 20.0f, "%.1f");
}

}  // namespace sinriv::ui::render
