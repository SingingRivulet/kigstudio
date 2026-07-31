#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>
#include <stb/stb_truetype.h>
#include <type_traits>
#include <cstdlib>
#include <unordered_set>
#include <variant>
#ifdef _WIN32
#include <windows.h>
#endif
#include "kigstudio/utils/vec3.h"
#include "license.h"

#include "kigstudio/utils/locale.h"
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {

void RenderVoxelList::render_ui() {
    static bool show_license_window = false;

    processThumbnails();
    item_status_height = 0;
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.f);
    if (ImGui::Begin(get_locale_cstr("window.stl_loader"), nullptr,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar |
                         ImGuiWindowFlags_MenuBar |
                         ImGuiWindowFlags_NoBringToFrontOnFocus)) {
        if (ImGui::BeginMenuBar()) {
            if (ImGui::BeginMenu(get_locale_cstr("menu.file"))) {
                if (ImGui::MenuItem(get_locale_cstr("menu.new_node"))) {
                    auto item = create_item();
                    setRenderId(item->id);
                }
                ImGui::Separator();
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
                render_recent_files_menu();
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
                    ImGui::Checkbox(get_locale_cstr("label.show_origin_mesh"),
                                    &showOriginMesh);
                    ImGui::Checkbox(get_locale_cstr("label.show_mesh"),
                                    &showMesh);
                    ImGui::Checkbox(get_locale_cstr("label.show_exported_mesh"),
                                    &showExportedMesh);
                    ImGui::Checkbox(get_locale_cstr("label.show_collision"),
                                    &showCollision);
                    ImGui::Checkbox(get_locale_cstr("label.show_voxels"),
                                    &showVoxels);
                    ImGui::Checkbox(get_locale_cstr("label.show_addon_mesh"),
                                    &showAddonMesh);
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
                    ImGui::Checkbox(
                        get_locale_cstr("label.show_voxel_chunk_bounds"),
                        &showVoxelChunkBounds);
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
                if (ImGui::MenuItem(get_locale_cstr("menu.flow_viewer"))) {
                    show_flow_viewer = true;
                }
                if (ImGui::MenuItem(get_locale_cstr("menu.extract_mmd"))) {
                    pending_open_extract_mmd_dialog = true;
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

            if (ImGui::BeginMenu(get_locale_cstr("menu.about"))) {
                if (ImGui::MenuItem(get_locale_cstr("menu.show_license"))) {
                    show_license_window = true;
                }
                if (ImGui::MenuItem(get_locale_cstr("menu.show_github"))) {
                    {
                        std::string url =
                            "https://github.com/SingingRivulet/kigstudio";
#ifdef _WIN32
                        std::string cmd = "start \"\" \"" + url + "\"";
#elif defined(__APPLE__)
                        std::string cmd = "open \"" + url + "\"";
#else
                        std::string cmd = "xdg-open \"" + url + "\"";
#endif

                        std::system(cmd.c_str());
                    }
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
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    get_locale_cstr("tooltip.export_mode_smooth"));
            }

            ImGui::Separator();

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

        // Extract MMD / PMX dialog (must be outside MenuBar)
        if (pending_open_extract_mmd_dialog) {
            ImGui::OpenPopup(get_locale_cstr("dialog.extract_mmd"));
            pending_open_extract_mmd_dialog = false;
        }
        render_extract_mmd_dialog();

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
        ImGui::SameLine();
        {
            const char* labels[] = {get_locale_cstr("label.show_addon_mesh"),
                                    get_locale_cstr("label.show_origin_mesh"),
                                    get_locale_cstr("label.show_mesh"),
                                    get_locale_cstr("label.show_exported_mesh"),
                                    get_locale_cstr("label.show_collision"),
                                    get_locale_cstr("label.show_voxels")};
            bool* states[] = {&showAddonMesh, &showOriginMesh, &showMesh,
                              &showExportedMesh, &showCollision, &showVoxels};
            constexpr int kNumButtons = 6;
            float buttonSpacing = ImGui::GetStyle().ItemSpacing.x;
            float totalWidth = 0;
            for (int i = 0; i < kNumButtons; ++i) {
                ImVec2 size = ImGui::CalcTextSize(labels[i]);
                totalWidth += size.x + ImGui::GetStyle().FramePadding.x * 2.0f;
                if (i < kNumButtons - 1)
                    totalWidth += buttonSpacing;
            }
            float windowWidth = ImGui::GetWindowSize().x;
            ImGui::SetCursorPosX(windowWidth - totalWidth -
                                 ImGui::GetStyle().WindowPadding.x);
            for (int i = 0; i < kNumButtons; ++i) {
                if (*states[i]) {
                    ImGui::PushStyleColor(ImGuiCol_Button,
                                          ImVec4(0.2f, 0.6f, 0.2f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                                          ImVec4(0.3f, 0.7f, 0.3f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                                          ImVec4(0.1f, 0.5f, 0.1f, 1.0f));
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Button,
                                          ImVec4(0.4f, 0.4f, 0.4f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                                          ImVec4(0.5f, 0.5f, 0.5f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                                          ImVec4(0.3f, 0.3f, 0.3f, 1.0f));
                }
                if (ImGui::SmallButton(labels[i])) {
                    *states[i] = !*states[i];
                }
                ImGui::PopStyleColor(3);
                if (i < kNumButtons - 1)
                    ImGui::SameLine();
            }
        }

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
        ImGui::SetNextWindowSize(ImVec2(420, 85), ImGuiCond_Always);
        if (ImGui::Begin(get_locale_cstr("window.async_voxel_loader"), nullptr,
                         ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoBringToFrontOnFocus)) {
            ImGui::TextWrapped("%s", this->getQueueStatus().c_str());
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
    render_object_editor_addons();
    render_guide_curve_window();
    render_width_editor_window();
    render_cross_section_editor();
    render_perpoint_section_editor();
    render_hairline_plane_window();
    render_file_loader();
    render_save_dialog();
    render_load_dialog();
    render_import_vxgrid_dialog();
    render_history_window();
    render_log_window();
    render_debug_voxel_pick_window();
    render_flow_viewer();

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

    // Per-point section confirmation: open global section editor
    if (show_perpoint_confirm_global_open) {
        ImGui::OpenPopup(
            get_locale_cstr("dialog.confirm_global_section_open_title"));
        show_perpoint_confirm_global_open = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal(
            get_locale_cstr("dialog.confirm_global_section_open_title"),
            nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted(
            get_locale_cstr("dialog.confirm_global_section_open_message"));
        ImGui::Separator();
        if (ImGui::Button(get_locale_cstr("action.yes"))) {
            {
                std::lock_guard<std::mutex> lock(locker);
                auto it = items.find(render_id);
                if (it != items.end() &&
                    pending_global_section_strand >= 0 &&
                    pending_global_section_strand <
                        static_cast<int>(
                            it->second->hair_strands.size())) {
                    // Clear all per-point overrides
                    for (auto& wp :
                         it->second
                             ->hair_strands[pending_global_section_strand]
                             .width_points) {
                        wp.section_state = SectionEditorState{};
                    }
                    it->second->hair_strands[pending_global_section_strand]
                        .mesh_dirty = true;
                    // Open global section editor
                    it->second->active_section_edit_strand =
                        pending_global_section_strand;
                    show_cross_section_editor_window = true;
                }
                pending_global_section_strand = -1;
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.cancel"))) {
            pending_global_section_strand = -1;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // Per-point section confirmation: apply global section
    if (show_perpoint_confirm_global_apply) {
        ImGui::OpenPopup(
            get_locale_cstr("dialog.confirm_global_section_apply_title"));
        show_perpoint_confirm_global_apply = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal(
            get_locale_cstr("dialog.confirm_global_section_apply_title"),
            nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted(
            get_locale_cstr("dialog.confirm_global_section_apply_message"));
        ImGui::Separator();
        if (ImGui::Button(get_locale_cstr("action.yes"))) {
            {
                std::lock_guard<std::mutex> lock(locker);
                auto it = items.find(render_id);
                if (it != items.end()) {
                    int idx = -1;
                    // Find the strand that triggered this confirmation
                    for (size_t si = 0;
                         si < it->second->hair_strands.size(); ++si) {
                        // Clear per-point overrides on all strands
                        for (auto& wp :
                             it->second->hair_strands[si].width_points) {
                            wp.section_state = SectionEditorState{};
                        }
                    }
                    it->second
                        ->hair_strands[pending_global_section_strand >= 0
                                           ? pending_global_section_strand
                                           : 0]
                        .mesh_dirty = true;
                }
                pending_global_section_strand = -1;
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.cancel"))) {
            pending_global_section_strand = -1;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (show_license_window) {
        ImGui::SetNextWindowSize(ImVec2(800, 600), ImGuiCond_Once);
        if (ImGui::Begin("License", &show_license_window,
                         ImGuiWindowFlags_NoCollapse)) {
            ImGui::BeginChild("LicenseText", ImVec2(0, 0), true);
            ImGui::PushTextWrapPos();
            ImGui::TextUnformatted(KIGSTUDIO_LICENSE_TEXT);
            ImGui::PopTextWrapPos();
            ImGui::EndChild();
        }

        ImGui::End();
    }

    this->setMeshAxisVisible(showMeshAxis);
    this->setVoxelAxisVisible(showVoxelAxis);
    this->setOriginMeshVisible(showOriginMesh);
    this->setMeshVisible(showMesh);
    this->setExportedMeshVisible(showExportedMesh);
    this->setVoxelsVisible(showVoxels);
    this->setCollisionVisible(showCollision);
    this->setCollisionBoundsVisible(showCollisionBounds);
    this->setVoxelChunkBoundsVisible(showVoxelChunkBounds);
    this->update_nav_node_position();

    // 渲染屏幕底部 toast 消息框
    render_toast();
}

void RenderVoxelList::render_file_loader() {
    static std::string stl_file_path;
    static float voxel_size = 1.0f;
    static int file_loader_load_mode = 0;
    static bool file_loader_load_as_sdf = false;
    static int file_loader_voxel_precision = 0; // Fast
    static bool last_show_file_loader = false;
    if (show_file_loader) {
        if (!last_show_file_loader) {
            stl_file_path.clear();
        }
        last_show_file_loader = true;
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
            const char* load_mode_names[] = {
                get_locale_cstr("label.stl_load_mode.default"),
                get_locale_cstr("label.stl_load_mode.surface_only"),
                get_locale_cstr("label.stl_load_mode.mesh_only"),
                get_locale_cstr("label.stl_load_mode.convex_hull"),
            };
            const int load_mode_values[] = {
                static_cast<int>(StlLoadMode::DEFAULT),
                static_cast<int>(StlLoadMode::SURFACE_ONLY),
                static_cast<int>(StlLoadMode::MESH_ONLY),
                static_cast<int>(StlLoadMode::CONVEX_HULL),
            };
            constexpr int load_mode_count =
                static_cast<int>(sizeof(load_mode_values) /
                                 sizeof(load_mode_values[0]));
            int current_load_mode_idx = 0;
            for (int i = 0; i < load_mode_count; ++i) {
                if (load_mode_values[i] == file_loader_load_mode) {
                    current_load_mode_idx = i;
                    break;
                }
            }
            if (ImGui::Combo(get_locale_cstr("label.stl_load_mode"),
                             &current_load_mode_idx, load_mode_names,
                             load_mode_count)) {
                file_loader_load_mode =
                    load_mode_values[current_load_mode_idx];
            }
            if (ImGui::IsItemHovered()) {
                const char* tooltip_key = nullptr;
                switch (file_loader_load_mode) {
                    case static_cast<int>(StlLoadMode::DEFAULT):
                        tooltip_key = "tooltip.stl_load_mode.default";
                        break;
                    case static_cast<int>(StlLoadMode::SURFACE_ONLY):
                        tooltip_key = "tooltip.stl_load_mode.surface_only";
                        break;
                    case static_cast<int>(StlLoadMode::MESH_ONLY):
                        tooltip_key = "tooltip.stl_load_mode.mesh_only";
                        break;
                    case static_cast<int>(StlLoadMode::CONVEX_HULL):
                        tooltip_key = "tooltip.stl_load_mode.convex_hull";
                        break;
                }
                if (tooltip_key) {
                    ImGui::SetTooltip(get_locale_cstr(tooltip_key));
                }
            }
            if (file_loader_load_mode ==
                    static_cast<int>(StlLoadMode::SURFACE_ONLY) ||
                file_loader_load_mode ==
                    static_cast<int>(StlLoadMode::MESH_ONLY)) {
                file_loader_load_as_sdf = false;
            }
            if (file_loader_load_mode !=
                    static_cast<int>(StlLoadMode::SURFACE_ONLY) &&
                file_loader_load_mode !=
                    static_cast<int>(StlLoadMode::MESH_ONLY)) {
                ImGui::Checkbox(get_locale_cstr("label.load_as_sdf"),
                                &file_loader_load_as_sdf);
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip(get_locale_cstr("tooltip.load_as_sdf"));
                }
                const char* vp_names[] = {
                    get_locale_cstr("label.sdf_precision.fast"),
                    get_locale_cstr("label.sdf_precision.precise"),
                    get_locale_cstr("label.sdf_precision.redundant"),
                };
                ImGui::Combo(
                    get_locale_cstr("label.voxel_precision"),
                    &file_loader_voxel_precision, vp_names, 3);
            }
            if (file_loader_load_mode !=
                static_cast<int>(StlLoadMode::MESH_ONLY)) {
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
            }
            ImGui::BeginDisabled(stl_file_path.empty());
            if (ImGui::Button(get_locale_cstr("action.open"))) {
                this->queue_load_stl(
                    stl_file_path, voxel_size,
                    file_loader_load_mode,
                    file_loader_load_as_sdf,
                    static_cast<sinriv::kigstudio::sdf::SDFPrecision>(
                        file_loader_voxel_precision));
                add_recent_file(stl_file_path);
                show_file_loader = false;
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button(get_locale_cstr("action.cancel"))) {
                show_file_loader = false;
            }
        }
        ImGui::End();
    } else {
        last_show_file_loader = false;
    }
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
                        hightlight_pos.emplace_back(
                            plane_p1, vec3f(0.8f, 0.0f, 0.5f), 1.0f);
                        hightlight_pos.emplace_back(
                            plane_p2, vec3f(0.8f, 0.0f, 0.7f), 1.0f);
                        hightlight_pos.emplace_back(
                            plane_p3, vec3f(0.8f, 0.0f, 0.9f), 1.0f);
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
                        hightlight_pos.emplace_back(
                            plane_point, vec3f(0.8f, 0.0f, 0.5f), 1.0f);
                        hightlight_pos.emplace_back(
                            plane_point + plane_normal * 2.0f,
                            vec3f(0.8f, 0.0f, 0.9f), 1.0f);
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
                add_recent_project(path);
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
            if (load_project(path)) {
                add_recent_project(path);
            } else {
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
                add_recent_file(vxgrid_file_path);
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

void RenderVoxelList::show_toast(const std::string& msg, float duration_ms) {
    std::lock_guard<std::mutex> lock(toast_mutex);
    ToastMessage toast;
    toast.text = msg;
    toast.start_time = std::chrono::steady_clock::now();
    toast.duration_ms = duration_ms;
    toast_queue.push_back(std::move(toast));
    while (toast_queue.size() > kMaxToastQueue) {
        toast_queue.pop_front();
    }
}

void RenderVoxelList::render_toast() {
    std::lock_guard<std::mutex> lock(toast_mutex);

    // 移除已过期的 toast
    auto now = std::chrono::steady_clock::now();
    while (!toast_queue.empty()) {
        auto& t = toast_queue.front();
        float elapsed_ms =
            std::chrono::duration<float, std::milli>(now - t.start_time)
                .count();
        if (elapsed_ms >= t.duration_ms) {
            toast_queue.pop_front();
        } else {
            break;
        }
    }

    if (toast_queue.empty())
        return;

    // 只渲染最早（最旧）的一条活动 toast
    auto& toast = toast_queue.front();
    float elapsed_ms =
        std::chrono::duration<float, std::milli>(now - toast.start_time)
            .count();
    float progress = elapsed_ms / toast.duration_ms;

    // 计算透明度：淡入 → 保持不透明 → 淡出
    float alpha;
    if (progress < kToastFadeInRatio) {
        alpha = progress / kToastFadeInRatio;
    } else if (progress < kToastFadeOutStartRatio) {
        alpha = 1.0f;
    } else {
        alpha = 1.0f - (progress - kToastFadeOutStartRatio) /
                           (1.0f - kToastFadeOutStartRatio);
    }

    // 位置：水平居中，垂直在靠下 1/4 处
    float cx = static_cast<float>(window_width) * 0.5f;
    float cy = static_cast<float>(window_height) * (3.0f / 4.0f);

    // 计算文字大小
    ImVec2 text_size = ImGui::CalcTextSize(toast.text.c_str());
    float padding_x = 24.0f;
    float padding_y = 10.0f;
    ImVec2 bg_size(text_size.x + padding_x * 2.0f,
                   text_size.y + padding_y * 2.0f);
    float rounding = bg_size.y * 0.5f;  // 胶囊形

    ImVec2 bg_min(cx - bg_size.x * 0.5f, cy - bg_size.y * 0.5f);
    ImVec2 bg_max(cx + bg_size.x * 0.5f, cy + bg_size.y * 0.5f);
    ImVec2 text_pos(cx - text_size.x * 0.5f, cy - text_size.y * 0.5f);

    // 半透明深色背景 + 白色文字
    ImU32 bg_color = IM_COL32(40, 40, 40, static_cast<int>(200.0f * alpha));
    ImU32 text_color =
        IM_COL32(255, 255, 255, static_cast<int>(255.0f * alpha));

    ImDrawList* draw_list = ImGui::GetForegroundDrawList();
    draw_list->AddRectFilled(bg_min, bg_max, bg_color, rounding);
    draw_list->AddText(nullptr, 0.0f, text_pos, text_color,
                       toast.text.c_str());
}

// ============================================================
// 最近打开的文件/工程状态管理
// ============================================================

std::filesystem::path RenderVoxelList::get_state_dir() const {
#ifdef _WIN32
    const char* appdata = std::getenv("APPDATA");
    if (appdata && appdata[0] != '\0') {
        return std::filesystem::path(utf8_to_wstring(appdata)) / L"kigstudio";
    }
    // 回退到 USERPROFILE
    const char* userprofile = std::getenv("USERPROFILE");
    if (userprofile && userprofile[0] != '\0') {
        return std::filesystem::path(utf8_to_wstring(userprofile)) /
               L".kigstudio";
    }
    return std::filesystem::temp_directory_path() / "kigstudio_state";
#else
    const char* xdg = std::getenv("XDG_CONFIG_HOME");
    if (xdg && xdg[0] != '\0') {
        return std::filesystem::path(xdg) / "kigstudio";
    }
    const char* home = std::getenv("HOME");
    if (home && home[0] != '\0') {
        return std::filesystem::path(home) / ".config" / "kigstudio";
    }
    return std::filesystem::temp_directory_path() / "kigstudio_state";
#endif
}

std::filesystem::path RenderVoxelList::get_state_file_path() const {
    return get_state_dir() / "recent.json";
}

void RenderVoxelList::load_recent_state() {
    recent_state_loaded = true;
    recent_files.clear();
    recent_projects.clear();

    auto path = get_state_file_path();
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec))
        return;

#ifdef _WIN32
    std::ifstream ifs(path.wstring());
#else
    std::ifstream ifs(path.c_str());
#endif
    if (!ifs.is_open())
        return;

    std::string json_str((std::istreambuf_iterator<char>(ifs)),
                         std::istreambuf_iterator<char>());
    ifs.close();

    // 去除 BOM (use unsigned char to avoid signed-char overflow UB)
    if (json_str.size() >= 3 &&
        static_cast<unsigned char>(json_str[0]) == 0xEF &&
        static_cast<unsigned char>(json_str[1]) == 0xBB &&
        static_cast<unsigned char>(json_str[2]) == 0xBF) {
        json_str.erase(0, 3);
    }

    cJSON* root = cJSON_Parse(json_str.c_str());
    if (!root)
        return;

    auto load_entries = [](cJSON* parent, const char* key,
                           std::vector<RecentEntry>& out) {
        cJSON* arr = cJSON_GetObjectItem(parent, key);
        if (!arr || !cJSON_IsArray(arr))
            return;
        cJSON* entry = nullptr;
        cJSON_ArrayForEach(entry, arr) {
            cJSON* path_item = cJSON_GetObjectItem(entry, "path");
            cJSON* time_item = cJSON_GetObjectItem(entry, "time");
            if (path_item && cJSON_IsString(path_item) && time_item &&
                cJSON_IsNumber(time_item)) {
                RecentEntry e;
                e.path = path_item->valuestring;
                e.timestamp = static_cast<int64_t>(time_item->valuedouble);
                out.push_back(std::move(e));
            }
        }
    };

    load_entries(root, "recent_files", recent_files);
    load_entries(root, "recent_projects", recent_projects);
    cJSON_Delete(root);
}

void RenderVoxelList::save_recent_state() const {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "version", 1);

    auto save_entries = [](cJSON* parent, const char* key,
                           const std::vector<RecentEntry>& entries) {
        cJSON* arr = cJSON_CreateArray();
        for (const auto& e : entries) {
            std::cerr << "[recent] save_entries key=" << key << " path='" << e.path << "' size=" << e.path.size() << std::endl;
            cJSON* obj = cJSON_CreateObject();
            cJSON_AddStringToObject(obj, "path", e.path.c_str());
            cJSON_AddNumberToObject(obj, "time",
                                    static_cast<double>(e.timestamp));
            cJSON_AddItemToArray(arr, obj);
        }
        cJSON_AddItemToObject(parent, key, arr);
    };

    std::cerr << "[recent] save_recent_state: recent_files=" << recent_files.size() << " recent_projects=" << recent_projects.size() << std::endl;
    save_entries(root, "recent_files", recent_files);
    save_entries(root, "recent_projects", recent_projects);

    auto dir = get_state_dir();
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);

    auto path = get_state_file_path();
    char* json_str = cJSON_Print(root);
    if (json_str) {
        const char utf8_bom[] = "\xEF\xBB\xBF";
#ifdef _WIN32
        std::ofstream ofs(path.wstring());
#else
        std::ofstream ofs(path.c_str());
#endif
        if (ofs.is_open()) {
            ofs.write(utf8_bom, 3);
            ofs << json_str;
        }
        cJSON_free(json_str);
    }
    cJSON_Delete(root);
}

static void add_recent_entry(std::vector<RenderVoxelList::RecentEntry>& entries,
                             const std::string& path_ref,
                             size_t max_entries) {
    // CRITICAL: copy path BEFORE erasing entries. The caller may pass
    // entries[i].path as the path argument, and erase would destroy
    // the referenced string (use-after-free).
    std::string path = path_ref;
    std::cerr << "[recent] add_recent_entry path='" << path << "' size=" << path.size() << std::endl;
    if (path.empty()) return;
    // 计算时间戳
    auto now = std::chrono::system_clock::now();
    int64_t timestamp =
        std::chrono::duration_cast<std::chrono::seconds>(
            now.time_since_epoch())
            .count();

    // 移除已有的同路径条目
    std::cerr << "[recent] before remove_if, entries.size()=" << entries.size() << std::endl;
    for (size_t di = 0; di < entries.size(); ++di) {
        std::cerr << "[recent]   entries[" << di << "].path='" << entries[di].path << "' size=" << entries[di].path.size() << std::endl;
    }
    entries.erase(
        std::remove_if(entries.begin(), entries.end(),
                       [&path](const RenderVoxelList::RecentEntry& e) {
                           bool match = (e.path == path);
                           std::cerr << "[recent] remove_if comparing e.path='" << e.path << "' with path='" << path << "' match=" << match << std::endl;
                           return match;
                       }),
        entries.end());
    std::cerr << "[recent] after remove_if+erase, entries.size()=" << entries.size() << std::endl;

    // 插入到最前面
    RenderVoxelList::RecentEntry entry;
    entry.path = path;
    std::cerr << "[recent] before insert, local entry.path='" << entry.path << "' size=" << entry.path.size() << " entries.size()=" << entries.size() << std::endl;
    entries.insert(entries.begin(), std::move(entry));
    std::cerr << "[recent] after insert, local entry.path='" << entry.path << "' size=" << entry.path.size() << std::endl;
    std::cerr << "[recent] after insert, entries[0].path='" << entries[0].path << "' size=" << entries[0].path.size() << std::endl;

    std::cerr << "[recent] after add, recent_projects has " << entries.size() << " entries" << std::endl;

    // 限制最大条目数
    while (entries.size() > max_entries) {
        entries.pop_back();
    }
}

void RenderVoxelList::add_recent_file(const std::string& path) {
    if (!recent_state_loaded)
        load_recent_state();
    add_recent_entry(recent_files, path, kMaxRecentEntries);
    save_recent_state();
}

void RenderVoxelList::add_recent_project(const std::string& path) {
    if (!recent_state_loaded)
        load_recent_state();
    add_recent_entry(recent_projects, path, kMaxRecentEntries);
    save_recent_state();
}

// ============================================================
// 最近打开的文件/工程子菜单
// ============================================================

void RenderVoxelList::render_recent_files_menu() {
    if (!recent_state_loaded)
        load_recent_state();

    // ---- 最近打开的文件 ----
    if (!recent_files.empty()) {
        if (ImGui::BeginMenu(get_locale_cstr("menu.recent_files"))) {
            for (size_t i = 0; i < recent_files.size(); ++i) {
                const auto& entry = recent_files[i];
                // 显示简短的文件名，完整路径作为 tooltip
                std::filesystem::path p = utf8_path(entry.path);
                std::string label = path_to_utf8(p.filename());
                if (label.empty())
                    label = entry.path;

                // 为每个条目添加序号以区分
                char menu_id[1024];
                snprintf(menu_id, sizeof(menu_id), "%s##rf%zu",
                         label.c_str(), i);

                if (ImGui::MenuItem(menu_id)) {
                    // 检查文件是否仍然存在
                    std::error_code ec;
                    if (std::filesystem::is_regular_file(p, ec)) {
                        show_file_loader = false;
                        queue_load_stl(entry.path, 1.0f);
                        // 刷新到最前面
                        add_recent_file(entry.path);
                    } else {
                        show_toast(
                            get_locale_string("toast.file_not_found") +
                                " " + entry.path,
                            3000.0f);
                        // 移除不存在的条目
                        std::vector<RecentEntry> filtered;
                        for (const auto& e : recent_files) {
                            if (e.path != entry.path)
                                filtered.push_back(e);
                        }
                        recent_files = std::move(filtered);
                        save_recent_state();
                    }
                    ImGui::EndMenu();
                    return;
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", entry.path.c_str());
                }
            }
            ImGui::Separator();
            if (ImGui::MenuItem(get_locale_cstr("menu.clear_recent"))) {
                recent_files.clear();
                save_recent_state();
            }
            ImGui::EndMenu();
        }
    }

    // ---- 最近打开的工程 ----
    if (!recent_projects.empty()) {
        if (ImGui::BeginMenu(get_locale_cstr("menu.recent_projects"))) {
            for (size_t i = 0; i < recent_projects.size(); ++i) {
                const auto& entry = recent_projects[i];
                std::filesystem::path p = utf8_path(entry.path);
                std::string label = path_to_utf8(p.filename());
                if (label.empty())
                    label = entry.path;

                char menu_id[1024];
                snprintf(menu_id, sizeof(menu_id), "%s##rp%zu",
                         label.c_str(), i);

                if (ImGui::MenuItem(menu_id)) {
                    std::cerr << "[recent] clicked entry i=" << i << " path='" << entry.path << "' size=" << entry.path.size() << std::endl;
                    std::error_code ec;
                    if (std::filesystem::is_directory(p, ec)) {
                        std::cerr << "[recent] is_directory=true, calling load_project" << std::endl;
                        if (load_project(entry.path)) {
                            std::cerr << "[recent] load_project succeeded, calling add_recent_project('" << entry.path << "')" <<  std::endl;
                            add_recent_project(entry.path);
                        } else {
                            std::string msg =
                                get_locale_string("error.load_failed") +
                                "\n" + last_load_error;
                            tinyfd_messageBox(
                                "Error",
                                utf8_to_ansi(msg.c_str()).c_str(), "ok",
                                "error", 1);
                        }
                    } else {
                        show_toast(
                            get_locale_string("toast.project_not_found") +
                                " " + entry.path,
                            3000.0f);
                        std::vector<RecentEntry> filtered;
                        for (const auto& e : recent_projects) {
                            if (e.path != entry.path)
                                filtered.push_back(e);
                        }
                        recent_projects = std::move(filtered);
                        save_recent_state();
                    }
                    ImGui::EndMenu();
                    return;
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", entry.path.c_str());
                }
            }
            ImGui::Separator();
            if (ImGui::MenuItem(get_locale_cstr("menu.clear_recent"))) {
                recent_projects.clear();
                save_recent_state();
            }
            ImGui::EndMenu();
        }
    }
}

}  // namespace sinriv::ui::render
