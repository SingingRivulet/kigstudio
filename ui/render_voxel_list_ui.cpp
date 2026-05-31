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
#include "kigstudio/utils/vec3.h"

#include "kigstudio/utils/locale.h"
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {

void RenderVoxelList::render_ui() {
    processThumbnails();
    float item_status_height = 0;
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.f);
    if (ImGui::Begin(get_locale_cstr("window.stl_loader"), nullptr,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar |
                         ImGuiWindowFlags_MenuBar |
                         ImGuiWindowFlags_NoBringToFrontOnFocus)) {
        if (ImGui::BeginMenuBar()) {
            if (ImGui::BeginMenu(get_locale_cstr("menu.file"))) {
                if (ImGui::BeginMenu(get_locale_cstr("menu.open_stl"))) {
                    if (ImGui::MenuItem(get_locale_cstr("menu.open_stl"))) {
                        show_file_loader = true;
                    }
                    if (ImGui::MenuItem(
                            get_locale_cstr("menu.import_vxgrid"))) {
                        show_import_vxgrid_dialog = true;
                    }
                    ImGui::EndMenu();
                }
                if (ImGui::MenuItem(get_locale_cstr("menu.save_project"))) {
                    if (!project_path.empty()) {
                        if (!save_current_project()) {
                            std::string msg =
                                get_locale_string("error.save_failed") + "\n" +
                                last_save_error;
                            tinyfd_messageBox("Error",
                                              utf8_to_ansi(msg.c_str()).c_str(),
                                              "ok", "error", 1);
                        }
                    } else {
                        show_save_dialog = true;
                    }
                }
                if (ImGui::MenuItem(get_locale_cstr("menu.save_project_as"))) {
                    show_save_as_dialog = true;
                }
                if (ImGui::MenuItem(get_locale_cstr("menu.load_project"))) {
                    show_load_dialog = true;
                }
                if (project_path.empty()) {
                    ImGui::BeginDisabled();
                    ImGui::MenuItem(get_locale_cstr("menu.export_stl_all"));
                    ImGui::EndDisabled();
                    if (ImGui::IsItemHovered(
                            ImGuiHoveredFlags_AllowWhenDisabled)) {
                        ImGui::SetTooltip(get_locale_cstr(
                            "tooltip.export_stl_all_no_project"));
                    }
                } else {
                    if (ImGui::MenuItem(
                            get_locale_cstr("menu.export_stl_all"))) {
                        pending_open_export_stl_all_dialog = true;
                    }
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu(get_locale_cstr("menu.view"))) {
                ImGui::Checkbox(get_locale_cstr("menu.history"),
                                &show_history_window);
                ImGui::Checkbox(get_locale_cstr("menu.log"), &show_log_window);
                if (ImGui::BeginMenu(get_locale_cstr("menu.body"))) {
                    ImGui::Checkbox(get_locale_cstr("label.show_mesh"),
                                    &showMesh);
                    ImGui::Checkbox(get_locale_cstr("label.show_exported_mesh"),
                                    &showExportedMesh);
                    ImGui::Checkbox(get_locale_cstr("label.show_collision"),
                                    &showCollision);
                    ImGui::Checkbox(get_locale_cstr("label.show_voxels"),
                                    &showVoxels);
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu(get_locale_cstr("menu.axis"))) {
                    ImGui::Checkbox(get_locale_cstr("label.show_mesh_axis"),
                                    &showMeshAxis);
                    ImGui::Checkbox(get_locale_cstr("label.show_voxel_axis"),
                                    &showVoxelAxis);
                    ImGui::Checkbox(
                        get_locale_cstr("label.show_collision_axis"),
                        &showCollisionAxis);
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu(get_locale_cstr("menu.bound"))) {
                    ImGui::Checkbox(
                        get_locale_cstr("label.show_collision_bounds"),
                        &showCollisionBounds);
                    ImGui::EndMenu();
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu(get_locale_cstr("menu.tools"))) {
                if (ImGui::MenuItem(
                        get_locale_cstr("menu.check_non_manifold"))) {
                    // 自动显示日志窗口
                    show_log_window = true;
                    // 对当前选中的 item 执行检测
                    std::lock_guard<std::mutex> lock(locker);
                    auto it = items.find(render_id);
                    if (it != items.end() && it->second->write_count == 0) {
                        queue_check_non_manifold(render_id);
                    } else {
                        append_queue_logf("log.queue.skip_check_busy");
                    }
                }
                if (ImGui::BeginMenu(get_locale_cstr("menu.debug"))) {
                    if (ImGui::MenuItem(
                            get_locale_cstr("menu.debug_voxel_picking"))) {
                        debug.show_voxel_pick_debug =
                            !debug.show_voxel_pick_debug;
                    }
                    ImGui::EndMenu();
                }
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }

        // Batch export STL dialog (must be outside MenuBar)
        if (pending_open_export_stl_all_dialog) {
            ImGui::OpenPopup(get_locale_cstr("dialog.export_stl_all"));
            pending_open_export_stl_all_dialog = false;
        }
        if (ImGui::BeginPopupModal(get_locale_cstr("dialog.export_stl_all"),
                                   nullptr,
                                   ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextUnformatted(
                get_locale_cstr("dialog.choose_export_method"));

            ImGui::RadioButton(get_locale_cstr("label.export_mode_standard"),
                               &export_stl_mode, 0);
            ImGui::RadioButton(get_locale_cstr("label.export_mode_smooth"),
                               &export_stl_mode, 1);

            ImGui::Separator();

            ImGui::Checkbox(get_locale_cstr("label.simplify_model"),
                            &export_stl_simplify);
            if (export_stl_simplify) {
                ImGui::Indent();
                ImGui::SliderFloat(
                    get_locale_cstr("label.simplification_ratio"),
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

            if (ImGui::Button(get_locale_cstr("action.export_stl_all"))) {
                ImGui::CloseCurrentPopup();
                std::filesystem::path export_dir =
                    utf8_path(project_path) / "exported_stl";
                queue_export_stl_all(path_to_utf8(export_dir), export_stl_mode,
                                     export_stl_simplify,
                                     export_stl_simplify_ratio,
                                     export_stl_subdivisions);
            }
            ImGui::SameLine();
            if (ImGui::Button(get_locale_cstr("action.cancel"))) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        const char* update_button_key = "action.update_collision";
        {
            std::lock_guard<std::mutex> lock(locker);
            auto it = items.find(render_id);
            ImGui::BeginDisabled(it == items.end() ||
                                 it->second->write_count != 0);
            if (ImGui::Button(get_locale_cstr(update_button_key))) {
                std::cout << "update collision" << std::endl;
                // 应用碰撞体到两个结果体素
                bool need_confirm = false;
                if (it != items.end()) {
                    std::function<bool(int)> has_child_auto_update_off;
                    has_child_auto_update_off = [&](int id) -> bool {
                        auto node = items.find(id);
                        if (node == items.end())
                            return false;
                        for (int child_id : node->second->children) {
                            if (child_id < 0)
                                continue;
                            auto child = items.find(child_id);
                            if (child == items.end())
                                continue;
                            if (!child->second->auto_segment_update)
                                return true;
                            if (has_child_auto_update_off(child_id))
                                return true;
                        }
                        return false;
                    };
                    need_confirm = has_child_auto_update_off(render_id);
                }
                if (need_confirm) {
                    show_manual_update_confirm = true;
                } else {
                    this->queue_do_segment_unsafe();
                }
            }
        }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(get_locale_cstr("tooltip.update_collision"));
        }
        ImGui::SameLine();
        if (mouse_world_pos_picked_auto_snapping) {
            if (ImGui::Button(
                    get_locale_cstr("action.pick_pos_auto_snapping_stop"))) {
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
        menu_height = static_cast<int>(
            ImGui::CalcWindowNextAutoFitSize(ImGui::GetCurrentWindow()).y);
        ImGui::SetWindowSize(ImVec2((float)window_width, (float)menu_height));
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(0.0f, (float)window_height),
                            ImGuiCond_Always, ImVec2(0.0f, 1.0f));
    if (ImGui::Begin(get_locale_cstr("window.item_status"), nullptr,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar |
                         ImGuiWindowFlags_NoBringToFrontOnFocus)) {
        ImGui::Text(get_locale_cstr("label.items_tasks"), this->get_num_items(),
                    this->queue_num);
        // 显示鼠标的三维位置
        ImGui::SameLine();
        ImGui::Text(get_locale_cstr("label.mouse_world_pos"), mouse_world_pos.x,
                    mouse_world_pos.y, mouse_world_pos.z);
        ImGui::SameLine();
        ImGui::Text(get_locale_cstr("label.current_memory_status"),
                    memory_current / 1024.0f / 1024.0f,
                    memory_peak / 1024.0f / 1024.0f);
        ImGui::SameLine();
        ImGui::Text(get_locale_cstr("label.current_fps"), fps);

        item_status_height =
            ImGui::GetCursorPosY() + ImGui::GetStyle().WindowPadding.y;

        ImGui::SetWindowSize(
            ImVec2((float)window_width, (float)item_status_height));
    }
    ImGui::End();

    ImGui::PopStyleVar();

    if (this->isQueueRunning()) {
        float async_y = window_height - item_status_height - 10.0f;
        ImGui::SetNextWindowPos(ImVec2((float)window_width, async_y),
                                ImGuiCond_Always, ImVec2(1.0f, 1.0f));
        ImGui::SetNextWindowSize(ImVec2(320, 70), ImGuiCond_Always);
        if (ImGui::Begin(get_locale_cstr("window.async_voxel_loader"), nullptr,
                         ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoBringToFrontOnFocus)) {
            ImGui::Text("%s", this->getQueueStatus().c_str());
            const char* cancel_label = get_locale_cstr("action.cancel");
            ImVec2 button_size = ImGui::CalcTextSize(cancel_label);
            button_size.x += ImGui::GetStyle().FramePadding.x * 2;
            button_size.y = 0;
            float progress_width = ImGui::GetContentRegionAvail().x -
                                   button_size.x -
                                   ImGui::GetStyle().ItemSpacing.x;
            ImGui::ProgressBar(this->getQueueProgress(),
                               ImVec2(progress_width, 0));
            ImGui::SameLine();
            if (ImGui::Button(cancel_label, button_size)) {
                this->queue_should_continue = false;
            }
        }
        ImGui::End();
    }

    render_nav_map();
    render_object_editor();
    render_file_loader();
    render_reload_stl_dialog();
    render_save_dialog();
    render_load_dialog();
    render_import_vxgrid_dialog();
    render_history_window();
    render_log_window();
    render_debug_voxel_pick_window();

    // Delete confirm modal
    if (show_delete_confirm) {
        ImGui::OpenPopup(get_locale_cstr("dialog.confirm_delete_title"));
        show_delete_confirm = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal(get_locale_cstr("dialog.confirm_delete_title"),
                               nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted(get_locale_cstr("dialog.confirm_delete"));
        ImGui::Separator();
        if (ImGui::Button(get_locale_cstr("action.delete"))) {
            {
                std::lock_guard<std::mutex> lock(locker);
                auto it = items.find(pending_delete_item_id);
                if (it != items.end()) {
                    it->second->queue_release = true;
                }
            }
            pending_delete_item_id = -1;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.cancel"))) {
            pending_delete_item_id = -1;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // Manual update confirm modal
    if (show_manual_update_confirm) {
        ImGui::OpenPopup(get_locale_cstr("dialog.confirm_manual_update_title"));
        show_manual_update_confirm = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal(
            get_locale_cstr("dialog.confirm_manual_update_title"), nullptr,
            ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted(
            get_locale_cstr("dialog.confirm_manual_update_message"));
        ImGui::Separator();
        if (ImGui::Button(get_locale_cstr("action.apply"))) {
            {
                std::lock_guard<std::mutex> lock(locker);
                auto it = items.find(render_id);
                if (it != items.end()) {
                    this->push_marked_undo_now(render_id, "Clear marks");
                    it->second->auto_segment_update = true;
                    it->second->marked_voxels =
                        sinriv::kigstudio::voxel::VoxelGrid();
                    it->second->marked_voxels_dirty = true;
                }
            }
            this->queue_do_segment();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.cancel"))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    this->setMeshAxisVisible(showMeshAxis);
    this->setVoxelAxisVisible(showVoxelAxis);
    this->setMeshVisible(showMesh);
    this->setExportedMeshVisible(showExportedMesh);
    this->setVoxelsVisible(showVoxels);
    this->setCollisionVisible(showCollision);
    this->setCollisionBoundsVisible(showCollisionBounds);
    this->update_nav_node_position();
}

void RenderVoxelList::render_file_loader() {
    static std::string stl_file_path;
    static float voxel_size = 1.0f;
    if (show_file_loader) {
        ImGuiViewport* vp = ImGui::GetMainViewport();
        ImVec2 center = vp->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Once, ImVec2(0.5f, 0.5f));
        if (ImGui::Begin(get_locale_cstr("window.load_stl_file"), nullptr,
                         ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextUnformatted(get_locale_cstr("label.load_stl_hint"));
            if (ImGui::Button(get_locale_cstr("action.open_file_dialog"))) {
                const char* filters[] = {"*.stl"};
                const char* file = tinyfd_openFileDialog(
                    utf8_to_ansi(get_locale_cstr("dialog.open_stl_title"))
                        .c_str(),
                    "", 1, filters,
                    utf8_to_ansi(get_locale_cstr("dialog.stl_file")).c_str(),
                    0);
                if (file) {
                    stl_file_path = tinyfd_path_to_utf8(file);
                }
            }
            if (!stl_file_path.empty()) {
                ImGui::Text(get_locale_cstr("label.selected_file"),
                            stl_file_path.c_str());
            } else {
                ImGui::TextUnformatted(
                    get_locale_cstr("label.no_file_selected"));
            }
            ImGui::Checkbox(get_locale_cstr("label.load_as_sdf"), &load_as_sdf);
            const float button_size = ImGui::GetFrameHeight();
            ImGui::TextUnformatted(get_locale_cstr("label.voxel_size"));
            ImGui::SameLine();
            if (ImGui::Button("-", ImVec2(button_size, 0))) {
                auto voxel_size_tmp = voxel_size / 2.0f;
                if (voxel_size_tmp >= 0.0001f) {
                    voxel_size = voxel_size_tmp;
                }
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(80.0f);
            ImGui::BeginDisabled(true);
            ImGui::DragFloat("##Voxel Size", &voxel_size, 0.1f, 0.0f, 0.0f,
                             "%.4f");
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("+", ImVec2(button_size, 0))) {
                voxel_size = voxel_size * 2.0f;
                if (voxel_size > 1000.0f) {
                    voxel_size = 1000.0f;
                }
            }
            ImGui::BeginDisabled(stl_file_path.empty());
            if (ImGui::Button(get_locale_cstr("action.open"))) {
                this->queue_load_stl(stl_file_path, voxel_size, load_as_sdf);
                show_file_loader = false;
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button(get_locale_cstr("action.cancel"))) {
                show_file_loader = false;
            }
        }
        ImGui::End();
    }
}

void RenderVoxelList::render_reload_stl_dialog() {
    if (!show_reload_stl_dialog)
        return;

    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImVec2 center = vp->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Once, ImVec2(0.5f, 0.5f));
    if (ImGui::Begin(get_locale_cstr("window.reload_stl"),
                     &show_reload_stl_dialog,
                     ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted(get_locale_cstr("label.reload_stl_hint"));
        ImGui::Checkbox(get_locale_cstr("label.load_as_sdf"), &load_as_sdf);
        ImGui::Separator();
        ImGui::TextUnformatted(get_locale_cstr("label.voxel_size"));
        ImGui::SameLine();
        const float button_size = ImGui::GetFrameHeight();
        if (ImGui::Button("-##reload", ImVec2(button_size, 0))) {
            auto tmp = reload_stl_voxel_size / 2.0f;
            if (tmp >= 0.0001f) {
                reload_stl_voxel_size = tmp;
            }
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0f);
        ImGui::DragFloat("##ReloadVoxelSize", &reload_stl_voxel_size, 0.1f,
                         0.0f, 0.0f, "%.4f");
        ImGui::SameLine();
        if (ImGui::Button("+##reload", ImVec2(button_size, 0))) {
            reload_stl_voxel_size = reload_stl_voxel_size * 2.0f;
            if (reload_stl_voxel_size > 1000.0f) {
                reload_stl_voxel_size = 1000.0f;
            }
        }
        ImGui::Separator();
        {
            std::lock_guard<std::mutex> lock(locker);
            auto it = items.find(reload_stl_item_id);
            if (it != items.end()) {
                it->second->stl_voxel_size = reload_stl_voxel_size;
            }
        }
        if (ImGui::Button(get_locale_cstr("action.apply"))) {
            {
                std::lock_guard<std::mutex> lock(locker);
                auto it = items.find(reload_stl_item_id);
                if (it != items.end()) {
                    it->second->stl_voxel_size = reload_stl_voxel_size;
                    this->queue_reload_stl(reload_stl_item_id,
                                           reload_stl_voxel_size,
                                           it->second->stl_path, load_as_sdf);
                }
            }
            show_reload_stl_dialog = false;
        }
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.cancel"))) {
            show_reload_stl_dialog = false;
        }
    }
    ImGui::End();
}

void RenderVoxelList::render_collision_body_editor(RenderVoxelItem& item) {
    auto before = capture_snapshot(item);
    EditResult edit_result;

    if (ImGui::CollapsingHeader(get_locale_cstr("label.collision_root"),
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        auto r = edit_transform_controls(item.collision_group.transform);
        edit_result.activated |= r.activated;
        edit_result.deactivated_after_edit |= r.deactivated_after_edit;
        edit_result.value_changed |= r.value_changed;
    }

    if (ImGui::CollapsingHeader(get_locale_cstr("label.collision_group"),
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        static int new_geometry_type = 0;
        const char* geometry_types[] = {
            get_locale_cstr("shape.sphere"), get_locale_cstr("shape.cylinder"),
            get_locale_cstr("shape.capsule"), get_locale_cstr("shape.box")};

        ImGui::SetNextItemWidth(140.0f);
        ImGui::Combo(get_locale_cstr("label.new_shape"), &new_geometry_type,
                     geometry_types, IM_ARRAYSIZE(geometry_types));
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.add_shape"))) {
            push_undo_now(item.id, std::nullopt,
                          get_locale_string("action.add_shape"));
            add_collision_geometry(item.collision_group, new_geometry_type);
        }

        auto& geometries = item.collision_group.geometries();
        int remove_index = -1;
        for (int member_idx = 0;
             member_idx < static_cast<int>(geometries.size()); ++member_idx) {
            auto& instance = geometries[member_idx];
            ImGui::PushID(member_idx);
            const std::string header =
                std::string(geometry_type_name(instance)) + " [" +
                std::to_string(member_idx) + "]";
            if (ImGui::CollapsingHeader(header.c_str(),
                                        ImGuiTreeNodeFlags_DefaultOpen)) {
                auto r1 = edit_transform_controls(instance.transform);
                edit_result.activated |= r1.activated;
                edit_result.deactivated_after_edit |= r1.deactivated_after_edit;
                edit_result.value_changed |= r1.value_changed;
                ImGui::Separator();
                auto r2 = edit_geometry_shape(instance);
                edit_result.activated |= r2.activated;
                edit_result.deactivated_after_edit |= r2.deactivated_after_edit;
                edit_result.value_changed |= r2.value_changed;
                ImGui::Separator();
                if (ImGui::Button(get_locale_cstr("action.delete"))) {
                    remove_index = member_idx;
                }
            }
            ImGui::PopID();
        }

        if (remove_index >= 0) {
            push_undo_now(item.id, std::nullopt,
                          get_locale_string("action.delete"));
            geometries.erase(geometries.begin() + remove_index);
        }

        if (geometries.empty()) {
            ImGui::TextUnformatted(
                get_locale_cstr("label.no_collision_shapes"));
        }
    }

    if (edit_result.value_changed) {
        push_undo_now(item.id, before,
                      get_locale_string("label.collision_root") + "/" +
                          get_locale_string("label.collision_group"));
    }
    if (edit_result.activated) {
        begin_edit(item.id);
    }
    if (edit_result.deactivated_after_edit) {
        end_edit(item.id, get_locale_string("label.collision_root") + "/" +
                              get_locale_string("label.collision_group"));
    }
}

void RenderVoxelList::render_plane_editor(RenderVoxelItem& item) {
    static int plane_editor_item_id = -1;
    static int plane_input_mode = 0;
    static vec3f plane_point = {0.0f, 0.0f, 0.0f};
    static vec3f plane_normal = {0.0f, 1.0f, 0.0f};
    static vec3f plane_p1 = {0.0f, 0.0f, 0.0f};
    static vec3f plane_p2 = {1.0f, 0.0f, 0.0f};
    static vec3f plane_p3 = {0.0f, 1.0f, 0.0f};
    static bool pick_point_by_mouse = false;
    static bool pick_normal_by_mouse = false;
    static bool pick_p1_by_mouse = false;
    static bool pick_p2_by_mouse = false;
    static bool pick_p3_by_mouse = false;
    static std::string plane_error_message;
    auto before = capture_snapshot(item);
    EditResult edit_result;

    if (ImGui::CollapsingHeader(get_locale_cstr("label.segment_plane"),
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("A: %.4f", item.plane.A);
        ImGui::Text("B: %.4f", item.plane.B);
        ImGui::Text("C: %.4f", item.plane.C);
        ImGui::Text("D: %.4f", item.plane.D);
        if (ImGui::Button(get_locale_cstr("action.edit_plane"))) {
            plane_editor_item_id = item.id;
            auto [point, normal] = item.plane.getPointNormalForm();
            vec3f p1, p2, p3;
            {
                vec3f tangent = normal.cross(vec3f(0.0f, 0.0f, 1.0f));
                if (tangent.length() < 1e-6f) {
                    tangent = normal.cross(vec3f(0.0f, 1.0f, 0.0f));
                }
                tangent =
                    sinriv::kigstudio::voxel::collision::safeNormalize(tangent);
                vec3f bitangent =
                    sinriv::kigstudio::voxel::collision::safeNormalize(
                        normal.cross(tangent));
                p1 = point;
                p2 = point + tangent;
                p3 = point + bitangent;
            }
            plane_point = point;
            plane_normal = normal;
            plane_p1 = p1;
            plane_p2 = p2;
            plane_p3 = p3;
            plane_error_message.clear();
            show_edit_segment_plane = true;
        }

        if (show_edit_segment_plane) {
            if (ImGui::Begin(get_locale_cstr("window.edit_segment_plane"),
                             nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                if (plane_editor_item_id != item.id) {
                    ImGui::TextUnformatted(
                        get_locale_cstr("label.plane_editor_bound_other"));
                } else {
                    const char* plane_modes[] = {
                        get_locale_cstr("mode.three_point"),
                        get_locale_cstr("mode.point_normal")};
                    ImGui::Combo(get_locale_cstr("label.input_mode"),
                                 &plane_input_mode, plane_modes,
                                 IM_ARRAYSIZE(plane_modes));
                    ImGui::Separator();

                    if (plane_input_mode == 0) {
                        auto r1 = edit_vec3_stepper("P1", plane_p1);
                        edit_result.activated |= r1.activated;
                        edit_result.deactivated_after_edit |=
                            r1.deactivated_after_edit;
                        edit_result.value_changed |= r1.value_changed;
                        if (ImGui::Checkbox(
                                get_locale_cstr("label.pick_p1_by_mouse"),
                                &pick_p1_by_mouse)) {
                            if (pick_p1_by_mouse) {
                                pick_p2_by_mouse = false;
                                pick_p3_by_mouse = false;
                            }
                        }
                        auto r2 = edit_vec3_stepper("P2", plane_p2);
                        edit_result.activated |= r2.activated;
                        edit_result.deactivated_after_edit |=
                            r2.deactivated_after_edit;
                        edit_result.value_changed |= r2.value_changed;
                        if (ImGui::Checkbox(
                                get_locale_cstr("label.pick_p2_by_mouse"),
                                &pick_p2_by_mouse)) {
                            if (pick_p2_by_mouse) {
                                pick_p1_by_mouse = false;
                                pick_p3_by_mouse = false;
                            }
                        }
                        auto r3 = edit_vec3_stepper("P3", plane_p3);
                        edit_result.activated |= r3.activated;
                        edit_result.deactivated_after_edit |=
                            r3.deactivated_after_edit;
                        edit_result.value_changed |= r3.value_changed;
                        if (ImGui::Checkbox(
                                get_locale_cstr("label.pick_p3_by_mouse"),
                                &pick_p3_by_mouse)) {
                            if (pick_p3_by_mouse) {
                                pick_p1_by_mouse = false;
                                pick_p2_by_mouse = false;
                            }
                        }

                        if (pick_p1_by_mouse && mouse_world_pos_valid &&
                            mouse_world_pos_picked) {
                            plane_p1 = mouse_world_pos;
                        }
                        if (pick_p2_by_mouse && mouse_world_pos_valid &&
                            mouse_world_pos_picked) {
                            plane_p2 = mouse_world_pos;
                        }
                        if (pick_p3_by_mouse && mouse_world_pos_valid &&
                            mouse_world_pos_picked) {
                            plane_p3 = mouse_world_pos;
                        }
                        hightlight_pos.push_back(
                            {plane_p1, {0.8f, 0.0f, 0.5f}});
                        hightlight_pos.push_back(
                            {plane_p2, {0.8f, 0.0f, 0.7f}});
                        hightlight_pos.push_back(
                            {plane_p3, {0.8f, 0.0f, 0.9f}});
                        const vec3f v1 = plane_p2 - plane_p1;
                        const vec3f v2 = plane_p3 - plane_p1;
                        vec3f normal = v1.cross(v2);
                        if (normal.length() < 1e-6f) {
                            plane_error_message = get_locale_string(
                                "error.three_points_collinear");
                        } else {
                            normal = sinriv::kigstudio::voxel::collision::
                                safeNormalize(normal);
                            item.plane = Plane(plane_p1, normal);
                            plane_error_message.clear();
                        }
                    } else if (plane_input_mode == 1) {
                        auto r1 = edit_vec3_stepper(
                            get_locale_cstr("label.point"), plane_point);
                        edit_result.activated |= r1.activated;
                        edit_result.deactivated_after_edit |=
                            r1.deactivated_after_edit;
                        edit_result.value_changed |= r1.value_changed;
                        if (ImGui::Checkbox(
                                get_locale_cstr("label.pick_point_by_mouse"),
                                &pick_point_by_mouse)) {
                            if (pick_point_by_mouse) {
                                pick_normal_by_mouse = false;
                            }
                        }
                        auto r2 =
                            edit_vec3_stepper(get_locale_cstr("label.normal"),
                                              plane_normal, 0.1f);
                        edit_result.activated |= r2.activated;
                        edit_result.deactivated_after_edit |=
                            r2.deactivated_after_edit;
                        edit_result.value_changed |= r2.value_changed;
                        if (ImGui::Checkbox(
                                get_locale_cstr("label.pick_normal_by_mouse"),
                                &pick_normal_by_mouse)) {
                            if (pick_normal_by_mouse) {
                                pick_point_by_mouse = false;
                            }
                        }
                        if (pick_point_by_mouse && mouse_world_pos_valid &&
                            mouse_world_pos_picked) {
                            plane_point = mouse_world_pos;
                        }
                        if (pick_normal_by_mouse && mouse_world_pos_valid &&
                            mouse_world_pos_picked) {
                            plane_normal = mouse_world_pos - plane_point;
                            plane_normal = sinriv::kigstudio::voxel::collision::
                                safeNormalize(plane_normal);
                        }
                        hightlight_pos.push_back(
                            {plane_point, {0.8f, 0.0f, 0.5f}});
                        hightlight_pos.push_back(
                            {plane_point + plane_normal * 2.0f,
                             {0.8f, 0.0f, 0.9f}});
                        try {
                            item.plane = Plane(plane_point, plane_normal);
                            plane_error_message.clear();
                        } catch (const std::exception& e) {
                            plane_error_message = e.what();
                        }
                    }

                    if (!plane_error_message.empty()) {
                        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%s",
                                           plane_error_message.c_str());
                    }
                }

                if (ImGui::Button(get_locale_cstr("action.close"))) {
                    plane_error_message.clear();
                    show_edit_segment_plane = false;
                }
            }
            ImGui::End();
        }
    }

    if (edit_result.value_changed) {
        push_undo_now(item.id, before,
                      get_locale_string("label.segment_mode") + " (Plane)");
    }
    if (edit_result.activated) {
        begin_edit(item.id);
    }
    if (edit_result.deactivated_after_edit) {
        end_edit(item.id, get_locale_string("label.segment_mode") + " (Plane)");
    }
}

void RenderVoxelList::render_save_dialog() {
    auto do_save = [&](bool force_dialog) {
        const char* folder = tinyfd_selectFolderDialog(
            utf8_to_ansi(get_locale_cstr("dialog.save_project_title")).c_str(),
            force_dialog ? "" : project_path.c_str());
        if (folder) {
            std::string path = tinyfd_path_to_utf8(folder);
            if (save_project(path)) {
                project_path = path;
            } else {
                std::string msg = get_locale_string("error.save_failed") +
                                  "\n" + last_save_error;
                tinyfd_messageBox("Error", utf8_to_ansi(msg.c_str()).c_str(),
                                  "ok", "error", 1);
            }
        }
    };
    if (show_save_dialog) {
        do_save(false);
        show_save_dialog = false;
    }
    if (show_save_as_dialog) {
        do_save(true);
        show_save_as_dialog = false;
    }
}

void RenderVoxelList::render_load_dialog() {
    if (show_load_dialog) {
        const char* folder = tinyfd_selectFolderDialog(
            utf8_to_ansi(get_locale_cstr("dialog.load_project_title")).c_str(),
            "");
        if (folder) {
            std::string path = tinyfd_path_to_utf8(folder);
            if (!load_project(path)) {
                std::string msg = get_locale_string("error.load_failed") +
                                  "\n" + last_load_error;
                tinyfd_messageBox("Error", utf8_to_ansi(msg.c_str()).c_str(),
                                  "ok", "error", 1);
            }
        }
        show_load_dialog = false;
    }
}

void RenderVoxelList::render_import_vxgrid_dialog() {
    static std::string vxgrid_file_path;
    static float voxel_size = 1.0f;
    if (!show_import_vxgrid_dialog)
        return;

    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImVec2 center = vp->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Once, ImVec2(0.5f, 0.5f));
    if (ImGui::Begin(get_locale_cstr("menu.import_vxgrid"),
                     &show_import_vxgrid_dialog,
                     ImGuiWindowFlags_AlwaysAutoResize)) {
        if (ImGui::Button(get_locale_cstr("action.open_file_dialog"))) {
            const char* filters[] = {"*.vxgrid"};
            const char* file = tinyfd_openFileDialog(
                utf8_to_ansi(get_locale_cstr("menu.import_vxgrid")).c_str(), "",
                1, filters,
                utf8_to_ansi(get_locale_cstr("dialog.vxgrid_files")).c_str(),
                0);
            if (file) {
                vxgrid_file_path = tinyfd_path_to_utf8(file);
            }
        }
        if (!vxgrid_file_path.empty()) {
            ImGui::Text(get_locale_cstr("label.selected_file"),
                        vxgrid_file_path.c_str());
        } else {
            ImGui::TextUnformatted(get_locale_cstr("label.no_file_selected"));
        }

        const float button_size = ImGui::GetFrameHeight();
        ImGui::TextUnformatted(get_locale_cstr("label.voxel_size"));
        ImGui::SameLine();
        if (ImGui::Button("-", ImVec2(button_size, 0))) {
            auto voxel_size_tmp = voxel_size / 2.0f;
            if (voxel_size_tmp >= 0.0001f) {
                voxel_size = voxel_size_tmp;
            }
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0f);
        ImGui::BeginDisabled(true);
        ImGui::DragFloat("##Voxel Size", &voxel_size, 0.1f, 0.0f, 0.0f, "%.4f");
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("+", ImVec2(button_size, 0))) {
            voxel_size = voxel_size * 2.0f;
            if (voxel_size > 1000.0f) {
                voxel_size = 1000.0f;
            }
        }

        ImGui::BeginDisabled(vxgrid_file_path.empty());
        if (ImGui::Button(get_locale_cstr("action.open"))) {
            auto item = create_item();
            if (sinriv::kigstudio::load(utf8_path(vxgrid_file_path),
                                        item->voxel_grid_data)) {
                item->voxel_grid_data.voxel_size = {voxel_size, voxel_size,
                                                    voxel_size};
                if (item->voxel_grid_data.num_chunk() > 0) {
                    item->voxel_renderer.loadVoxelGridChunked(
                        item->voxel_grid_data, 0.5, true);
                }
                item->thumbnail_dirty = true;
                item->dirty = true;
                setRenderId(item->id);
                show_import_vxgrid_dialog = false;
            } else {
                {
                    std::lock_guard<std::mutex> lock(locker);
                    auto it = items.find(item->id);
                    if (it != items.end()) {
                        it->second->queue_release = true;
                    }
                }
                tinyfd_messageBox(
                    "Error",
                    utf8_to_ansi(get_locale_cstr("error.load_failed")).c_str(),
                    "ok", "error", 1);
            }
        }
        ImGui::EndDisabled();
    }
    ImGui::End();
}

void RenderVoxelList::render_debug_voxel_pick_window() {
    if (!debug.show_voxel_pick_debug)
        return;

    ImGui::SetNextWindowSize(ImVec2(450, 350), ImGuiCond_Once);
    if (ImGui::Begin(get_locale_cstr("window.debug_voxel_picking"),
                     &debug.show_voxel_pick_debug)) {
        ImGui::TextUnformatted(
            get_locale_cstr("label.debug_voxel_pick_timings"));
        ImGui::Separator();

        if (debug.voxel_pick_timings.empty()) {
            ImGui::TextDisabled(
                "No data yet. Pick some voxels to see timings.");
        } else {
            if (ImGui::BeginTable("##VoxelPickTimings", 5,
                                  ImGuiTableFlags_Borders |
                                      ImGuiTableFlags_RowBg |
                                      ImGuiTableFlags_ScrollY |
                                      ImGuiTableFlags_SizingStretchProp)) {
                ImGui::TableSetupScrollFreeze(0, 1);
                ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed,
                                        40.0f);
                ImGui::TableSetupColumn(
                    get_locale_cstr("label.debug_step_world_to_voxel"));
                ImGui::TableSetupColumn(
                    get_locale_cstr("label.debug_step_iterate_surface"));
                ImGui::TableSetupColumn(
                    get_locale_cstr("label.debug_step_mark_voxels"));
                ImGui::TableSetupColumn(
                    get_locale_cstr("label.debug_step_total"));
                ImGui::TableHeadersRow();

                int idx = 1;
                for (const auto& t : debug.voxel_pick_timings) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%d", idx++);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.3f", t.world_to_voxel_ms);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.3f", t.iterate_surface_ms);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%.3f", t.mark_voxels_ms);
                    ImGui::TableSetColumnIndex(4);
                    ImGui::Text("%.3f", t.total_ms);
                }
                ImGui::EndTable();
            }

            if (ImGui::Button("Clear")) {
                debug.voxel_pick_timings.clear();
            }
            ImGui::SameLine();
            ImGui::Text("Count: %zu / %zu", debug.voxel_pick_timings.size(),
                        debug.max_voxel_pick_timings);
        }
    }
    ImGui::End();
}

}  // namespace sinriv::ui::render
