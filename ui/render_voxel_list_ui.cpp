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
#include "locale.h"
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {
namespace {

#ifdef _WIN32
// ANSI → UTF-16
std::wstring ansi_to_utf16(const char* str) {
    if (!str)
        return {};

    int len = MultiByteToWideChar(CP_ACP, 0, str, -1, nullptr, 0);
    if (len <= 1)
        return {};

    std::wstring w(len - 1, 0);

    MultiByteToWideChar(CP_ACP, 0, str, -1, &w[0], len);

    return w;
}

// UTF-16 → UTF-8
std::string utf16_to_utf8(const std::wstring& w) {
    if (w.empty())
        return {};

    int len = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0,
                                  nullptr, nullptr);
    if (len <= 1)
        return {};

    std::string s(len - 1, 0);

    WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, &s[0], len, nullptr,
                        nullptr);

    return s;
}

// ANSI → UTF-8
std::string ansi_to_utf8(const char* str) {
    return utf16_to_utf8(ansi_to_utf16(str));
}
#endif

std::string tinyfd_path_to_utf8(const char* path) {
#ifdef _WIN32
    return ansi_to_utf8(path);
#else
    return path ? std::string(path) : std::string();
#endif
}

std::string localize_id(const char* key, int id) {
    return get_locale_string(key) + "##" + std::to_string(id);
}

using CollisionGroup = sinriv::kigstudio::voxel::collision::CollisionGroup;
using GeometryInstance = sinriv::kigstudio::voxel::collision::GeometryInstance;
using Sphere = sinriv::kigstudio::voxel::collision::Sphere;
using Cylinder = sinriv::kigstudio::voxel::collision::Cylinder;
using Capsule = sinriv::kigstudio::voxel::collision::Capsule;
using Box = sinriv::kigstudio::voxel::collision::Box;
using Transform = sinriv::kigstudio::voxel::collision::Transform;
using vec3f = sinriv::kigstudio::voxel::collision::vec3f;
using Plane = sinriv::kigstudio::Plane<float>;

struct EditResult {
    bool activated = false;
    bool deactivated_after_edit = false;
    bool value_changed = false;  // 按钮点击等立即变化
};

EditResult edit_float_stepper(const char* label, float& value, float step = 0.5f) {
    EditResult result;
    const float button_size = ImGui::GetFrameHeight();
    ImGui::PushID(label);
    char buf[128];
    // 截断label中的##
    snprintf(buf, sizeof(buf), "%s", label);
    for (int i = 0; i < sizeof(buf) && buf[i] != '\0'; ++i) {
        if (buf[i] == '#') {
            buf[i] = '\0';
            break;
        }
    }
    ImGui::Text("%s", buf);
    ImGui::SameLine();
    if (ImGui::Button("-", ImVec2(button_size, 0))) {
        value -= step;
        result.value_changed = true;
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80.0f);
    snprintf(buf, sizeof(buf), "##%s", label);
    ImGui::DragFloat(buf, &value, step, 0.0f, 0.0f, "%.2f");
    if (ImGui::IsItemActivated()) result.activated = true;
    if (ImGui::IsItemDeactivatedAfterEdit()) result.deactivated_after_edit = true;
    ImGui::SameLine();
    if (ImGui::Button("+", ImVec2(button_size, 0))) {
        value += step;
        result.value_changed = true;
    }
    ImGui::PopID();
    return result;
}

EditResult edit_vec3_stepper(const char* label,
                       vec3f& value,
                       float step = 0.5f,
                       bool normalize = false) {
    EditResult result;
    const char* axis_names[] = {"X", "Y", "Z"};
    float values[3] = {value.x, value.y, value.z};
    ImGui::Text("%s", label);
    char buf[128];
    for (int i = 0; i < 3; ++i) {
        // if (i > 0) {
        //     ImGui::SameLine();
        // }
        snprintf(buf, sizeof(buf), "%s##%s", axis_names[i], label);
        auto r = edit_float_stepper(buf, values[i], step);
        result.activated |= r.activated;
        result.deactivated_after_edit |= r.deactivated_after_edit;
        result.value_changed |= r.value_changed;
    }
    value = {values[0], values[1], values[2]};
    if (normalize) {
        value = sinriv::kigstudio::voxel::collision::safeNormalize(value);
    }
    return result;
}
EditResult edit_local_position_stepper(const char* label,
                                 vec3f& value,
                                 float step = 0.5f,
                                 bool normalize = false,
                                 bool show_label = true) {
    EditResult result;
    const char* axis_names[] = {"X", "Y", "Z"};
    float values[3] = {value.x, -value.y, value.z};
    if (show_label) {
        ImGui::Text("%s", label);
    }
    char buf[128];
    for (int i = 0; i < 3; ++i) {
        snprintf(buf, sizeof(buf), "%s##%s", axis_names[i], label);
        auto r = edit_float_stepper(buf, values[i], step);
        result.activated |= r.activated;
        result.deactivated_after_edit |= r.deactivated_after_edit;
        result.value_changed |= r.value_changed;
    }
    value = {values[0], -values[1], values[2]};
    if (normalize) {
        value = sinriv::kigstudio::voxel::collision::safeNormalize(value);
    }
    return result;
}

EditResult edit_transform_controls(Transform& transform) {
    EditResult result;
    vec3f pos = transform.getPosition();
    auto r1 = edit_vec3_stepper(get_locale_cstr("label.position"), pos);
    result.activated |= r1.activated;
    result.deactivated_after_edit |= r1.deactivated_after_edit;
    result.value_changed |= r1.value_changed;
    transform.setPosition(pos);

    vec3f euler_rad = transform.getRotationEuler();
    vec3f euler_deg = {bx::toDeg(euler_rad.x), bx::toDeg(euler_rad.y),
                       bx::toDeg(euler_rad.z)};
    auto r2 = edit_vec3_stepper(get_locale_cstr("label.rotation_deg"), euler_deg);
    result.activated |= r2.activated;
    result.deactivated_after_edit |= r2.deactivated_after_edit;
    result.value_changed |= r2.value_changed;
    transform.setRotationEuler({bx::toRad(euler_deg.x), bx::toRad(euler_deg.y),
                                bx::toRad(euler_deg.z)});
    return result;
}

const char* geometry_type_name(const GeometryInstance& instance) {
    return std::visit(
        [](const auto& geometry) -> const char* {
            using T = std::decay_t<decltype(geometry)>;
            if constexpr (std::is_same_v<T, Sphere>) {
                return get_locale_cstr("shape.sphere");
            } else if constexpr (std::is_same_v<T, Cylinder>) {
                return get_locale_cstr("shape.cylinder");
            } else if constexpr (std::is_same_v<T, Capsule>) {
                return get_locale_cstr("shape.capsule");
            } else if constexpr (std::is_same_v<T, Box>) {
                return get_locale_cstr("shape.box");
            } else {
                return get_locale_cstr("shape.unknown");
            }
        },
        instance.geometry);
}

EditResult edit_geometry_shape(GeometryInstance& instance) {
    EditResult result;
    std::visit(
        [&result](auto& geometry) {
            using T = std::decay_t<decltype(geometry)>;
            if constexpr (std::is_same_v<T, Sphere>) {
                auto r1 = edit_local_position_stepper(get_locale_cstr("label.center"),
                                            geometry.center);
                result.activated |= r1.activated;
                result.deactivated_after_edit |= r1.deactivated_after_edit;
                result.value_changed |= r1.value_changed;
                auto r2 = edit_float_stepper(get_locale_cstr("label.radius"),
                                   geometry.radius);
                result.activated |= r2.activated;
                result.deactivated_after_edit |= r2.deactivated_after_edit;
                result.value_changed |= r2.value_changed;
                geometry.radius = bx::max(0.0f, geometry.radius);
            } else if constexpr (std::is_same_v<T, Cylinder>) {
                auto r1 = edit_local_position_stepper(get_locale_cstr("label.start"),
                                            geometry.start);
                result.activated |= r1.activated;
                result.deactivated_after_edit |= r1.deactivated_after_edit;
                result.value_changed |= r1.value_changed;
                auto r2 = edit_local_position_stepper(get_locale_cstr("label.end"),
                                            geometry.end);
                result.activated |= r2.activated;
                result.deactivated_after_edit |= r2.deactivated_after_edit;
                result.value_changed |= r2.value_changed;
                auto r3 = edit_float_stepper(get_locale_cstr("label.radius"),
                                   geometry.radius);
                result.activated |= r3.activated;
                result.deactivated_after_edit |= r3.deactivated_after_edit;
                result.value_changed |= r3.value_changed;
                geometry.radius = bx::max(0.0f, geometry.radius);
            } else if constexpr (std::is_same_v<T, Capsule>) {
                auto r1 = edit_local_position_stepper(get_locale_cstr("label.start"),
                                            geometry.start);
                result.activated |= r1.activated;
                result.deactivated_after_edit |= r1.deactivated_after_edit;
                result.value_changed |= r1.value_changed;
                auto r2 = edit_local_position_stepper(get_locale_cstr("label.end"),
                                            geometry.end);
                result.activated |= r2.activated;
                result.deactivated_after_edit |= r2.deactivated_after_edit;
                result.value_changed |= r2.value_changed;
                auto r3 = edit_float_stepper(get_locale_cstr("label.radius"),
                                   geometry.radius);
                result.activated |= r3.activated;
                result.deactivated_after_edit |= r3.deactivated_after_edit;
                result.value_changed |= r3.value_changed;
                geometry.radius = bx::max(0.0f, geometry.radius);
            } else if constexpr (std::is_same_v<T, Box>) {
                auto r = edit_vec3_stepper(get_locale_cstr("label.half_extent"),
                                  geometry.half_extent);
                result.activated |= r.activated;
                result.deactivated_after_edit |= r.deactivated_after_edit;
                result.value_changed |= r.value_changed;
                geometry.half_extent.x = bx::max(0.0f, geometry.half_extent.x);
                geometry.half_extent.y = bx::max(0.0f, geometry.half_extent.y);
                geometry.half_extent.z = bx::max(0.0f, geometry.half_extent.z);
            }
        },
        instance.geometry);
    return result;
}

void add_collision_geometry(CollisionGroup& group, int type_index) {
    switch (type_index) {
        case 0:
            group.add(Sphere{{0.0f, 0.0f, 0.0f}, 10.0f});
            break;
        case 1:
            group.add(
                Cylinder{{0.0f, 0.0f, -10.0f}, {0.0f, 0.0f, 10.0f}, 6.0f});
            break;
        case 2:
            group.add(Capsule{{-10.0f, 0.0f, 0.0f}, {10.0f, 0.0f, 0.0f}, 6.0f});
            break;
        case 3:
            group.add(Box{{8.0f, 8.0f, 8.0f}});
            break;
    }
}

}  // namespace

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
                if (ImGui::MenuItem(get_locale_cstr("menu.open_stl"))) {
                    show_file_loader = true;
                }
                if (ImGui::MenuItem(get_locale_cstr("menu.save_project"))) {
                    if (!project_path.empty()) {
                        if (!save_current_project()) {
                            std::string msg = get_locale_string("error.save_failed") + "\n" + last_save_error;
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
                    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                        ImGui::SetTooltip(get_locale_cstr("tooltip.export_stl_all_no_project"));
                    }
                } else {
                    if (ImGui::MenuItem(get_locale_cstr("menu.export_stl_all"))) {
                        std::filesystem::path export_dir = utf8_path(project_path) / "exported_stl";
                        std::filesystem::create_directories(export_dir);
                        std::vector<int> exported_ids;
                        {
                            std::lock_guard<std::mutex> lock(locker);
                            for (auto& [id, item] : items) {
                                bool is_leaf = true;
                                for (int cid : item->children) {
                                    if (cid >= 0) {
                                        is_leaf = false;
                                        break;
                                    }
                                }
                                if (is_leaf) {
                                    try {
                                        std::vector<std::tuple<sinriv::kigstudio::voxel::Triangle,
                                                               sinriv::kigstudio::voxel::vec3f>>
                                            mesh;
                                        int numTriangles = 0;
                                        for (auto triangles :
                                             sinriv::kigstudio::voxel::generateMesh(
                                                 item->voxel_grid_data, 0.5,
                                                 numTriangles, true)) {
                                            mesh.push_back(triangles);
                                        }
                                        std::string filename = "node_" + std::to_string(id) + ".stl";
                                        std::filesystem::path filepath = export_dir / filename;
                                        sinriv::kigstudio::voxel::saveMeshToASCIISTL(
                                            mesh, path_to_utf8(filepath));
                                        exported_ids.push_back(id);
                                    } catch (const std::exception& e) {
                                        std::cout << "Export STL failed for node " << id
                                                  << ": " << e.what() << std::endl;
                                    }
                                }
                            }
                        }
                        if (exported_ids.empty()) {
                            tinyfd_messageBox(
                                get_locale_cstr("dialog.info"),
                                utf8_to_ansi(get_locale_cstr("tooltip.export_stl_all_empty")).c_str(),
                                "ok", "info", 1);
                        } else {
                            std::cout << "Exported " << exported_ids.size()
                                      << " STL files to " << path_to_utf8(export_dir) << std::endl;
                        }
                    }
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu(get_locale_cstr("menu.view"))) {
                ImGui::Checkbox(get_locale_cstr("menu.history"),
                                &show_history_window);
                ImGui::Checkbox(get_locale_cstr("menu.log"),
                                &show_log_window);
                if (ImGui::BeginMenu(get_locale_cstr("menu.body"))) {
                    ImGui::Checkbox(get_locale_cstr("label.show_mesh"),
                                    &showMesh);
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
                    ImGui::Checkbox(get_locale_cstr("label.show_collision_bounds"),
                                    &showCollisionBounds);
                    ImGui::EndMenu();
                }
                ImGui::Checkbox(get_locale_cstr("label.disable_camera_on_pick"),
                                &disable_camera_on_pick);
                ImGui::DragFloat(get_locale_cstr("label.mouse_highlight_range"),
                                 &mouse_highlight_range, 0.1f, 0.0f, 20.0f, "%.1f");
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }
        if (ImGui::Button(get_locale_cstr("action.update_collision"))) {
            std::cout << "update collision" << std::endl;
            // 应用碰撞体到两个结果体素
            bool need_confirm = false;
            {
                std::lock_guard<std::mutex> lock(locker);
                auto it = items.find(render_id);
                if (it != items.end()) {
                    std::function<bool(int)> has_child_auto_update_off;
                    has_child_auto_update_off = [&](int id) -> bool {
                        auto node = items.find(id);
                        if (node == items.end()) return false;
                        for (int child_id : node->second->children) {
                            if (child_id < 0) continue;
                            auto child = items.find(child_id);
                            if (child == items.end()) continue;
                            if (!child->second->auto_segment_update) return true;
                            if (has_child_auto_update_off(child_id)) return true;
                        }
                        return false;
                    };
                    need_confirm = has_child_auto_update_off(render_id);
                }
            }
            if (need_confirm) {
                show_manual_update_confirm = true;
            } else {
                this->queue_do_segment();
            }
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
            ImGui::ProgressBar(this->getQueueProgress(), ImVec2(-1, 0));
        }
        ImGui::End();
    }

    render_nav_map();
    render_collision_node_editor();
    render_file_loader();
    render_reload_stl_dialog();
    render_save_dialog();
    render_load_dialog();
    render_history_window();
    render_log_window();

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
    if (ImGui::BeginPopupModal(get_locale_cstr("dialog.confirm_manual_update_title"),
                               nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted(
            get_locale_cstr("dialog.confirm_manual_update_message"));
        ImGui::Separator();
        if (ImGui::Button(get_locale_cstr("action.apply"))) {
            {
                std::lock_guard<std::mutex> lock(locker);
                auto it = items.find(render_id);
                if (it != items.end()) {
                    it->second->auto_segment_update = true;
                    it->second->marked_voxels = sinriv::kigstudio::voxel::VoxelGrid();
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
                const char* file =
                    tinyfd_openFileDialog(
                        utf8_to_ansi(get_locale_cstr("dialog.open_stl_title")).c_str(),
                        "", 1, filters,
                        utf8_to_ansi(get_locale_cstr("dialog.stl_file")).c_str(), 0);
                if (file) {
                    stl_file_path = tinyfd_path_to_utf8(file);
                }
            }
            if (!stl_file_path.empty()) {
                ImGui::Text(get_locale_cstr("label.selected_file"),
                            stl_file_path.c_str());
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
                this->queue_load_stl(stl_file_path, voxel_size);
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
    if (!show_reload_stl_dialog) return;

    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImVec2 center = vp->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Once, ImVec2(0.5f, 0.5f));
    if (ImGui::Begin(get_locale_cstr("window.reload_stl"), &show_reload_stl_dialog,
                     ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted(get_locale_cstr("label.reload_stl_hint"));
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
                                           it->second->stl_path);
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
        const char* geometry_types[] = {get_locale_cstr("shape.sphere"),
                                        get_locale_cstr("shape.cylinder"),
                                        get_locale_cstr("shape.capsule"),
                                        get_locale_cstr("shape.box")};

        ImGui::SetNextItemWidth(140.0f);
        ImGui::Combo(get_locale_cstr("label.new_shape"), &new_geometry_type,
                     geometry_types, IM_ARRAYSIZE(geometry_types));
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.add_shape"))) {
            push_undo_now(item.id, std::nullopt, get_locale_string("action.add_shape"));
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
            push_undo_now(item.id, std::nullopt, get_locale_string("action.delete"));
            geometries.erase(geometries.begin() + remove_index);
        }

        if (geometries.empty()) {
            ImGui::TextUnformatted(get_locale_cstr("label.no_collision_shapes"));
        }
    }

    if (edit_result.value_changed) {
        push_undo_now(item.id, before, get_locale_string("label.collision_root") + "/" + get_locale_string("label.collision_group"));
    }
    if (edit_result.activated) {
        begin_edit(item.id);
    }
    if (edit_result.deactivated_after_edit) {
        end_edit(item.id, get_locale_string("label.collision_root") + "/" + get_locale_string("label.collision_group"));
    }
}
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
            push_undo_now(item.id, std::nullopt, get_locale_string("action.clear_vertices"));
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
                    localize_id("label.edit", i).c_str(), v, 0.1f, false, false);
                edit_result.activated |= r.activated;
                edit_result.deactivated_after_edit |= r.deactivated_after_edit;
                edit_result.value_changed |= r.value_changed;

                // --- delete ---
                if (ImGui::Button(localize_id("action.delete", i).c_str())) { // 需要增加二次确认
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
            push_undo_now(item.id, std::nullopt, get_locale_string("action.delete"));
            item.concave_cone.base_vertices.erase(
                item.concave_cone.base_vertices.begin() + erase_index);

            item.err_info.clear();
            item.concave_cone.check(item.err_info);
        }

        // ===== picking 执行（统一入口）=====
        if (pick_mode != PickMode::None && mouse_world_pos_valid &&
            mouse_world_pos_picked)  // ⚠ 必须是点击瞬间
        {
            push_undo_now(item.id, std::nullopt, get_locale_string("label.pick_point_by_mouse"));
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
        push_undo_now(item.id, before, get_locale_string("label.concave_cone_apex") + "/" + get_locale_string("label.concave_cone_vertices"));
    }
    if (edit_result.activated) {
        begin_edit(item.id);
    }
    if (edit_result.deactivated_after_edit) {
        end_edit(item.id, get_locale_string("label.concave_cone_apex") + "/" + get_locale_string("label.concave_cone_vertices"));
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
            if (ImGui::Begin(get_locale_cstr("window.edit_segment_plane"), nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
                if (plane_editor_item_id != item.id) {
                    ImGui::TextUnformatted(
                        get_locale_cstr("label.plane_editor_bound_other"));
                } else {
                    const char* plane_modes[] = {get_locale_cstr("mode.three_point"),
                                                 get_locale_cstr("mode.point_normal")};
                    ImGui::Combo(get_locale_cstr("label.input_mode"),
                                 &plane_input_mode, plane_modes,
                                 IM_ARRAYSIZE(plane_modes));
                    ImGui::Separator();

                    if (plane_input_mode == 0) {
                        auto r1 = edit_vec3_stepper("P1", plane_p1);
                        edit_result.activated |= r1.activated;
                        edit_result.deactivated_after_edit |= r1.deactivated_after_edit;
                        edit_result.value_changed |= r1.value_changed;
                        if (ImGui::Checkbox(get_locale_cstr("label.pick_p1_by_mouse"),
                                            &pick_p1_by_mouse)) {
                            if (pick_p1_by_mouse) {
                                pick_p2_by_mouse = false;
                                pick_p3_by_mouse = false;
                            }
                        }
                        auto r2 = edit_vec3_stepper("P2", plane_p2);
                        edit_result.activated |= r2.activated;
                        edit_result.deactivated_after_edit |= r2.deactivated_after_edit;
                        edit_result.value_changed |= r2.value_changed;
                        if (ImGui::Checkbox(get_locale_cstr("label.pick_p2_by_mouse"),
                                            &pick_p2_by_mouse)) {
                            if (pick_p2_by_mouse) {
                                pick_p1_by_mouse = false;
                                pick_p3_by_mouse = false;
                            }
                        }
                        auto r3 = edit_vec3_stepper("P3", plane_p3);
                        edit_result.activated |= r3.activated;
                        edit_result.deactivated_after_edit |= r3.deactivated_after_edit;
                        edit_result.value_changed |= r3.value_changed;
                        if (ImGui::Checkbox(get_locale_cstr("label.pick_p3_by_mouse"),
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
                            plane_error_message =
                                get_locale_string("error.three_points_collinear");
                        } else {
                            normal = sinriv::kigstudio::voxel::collision::
                                safeNormalize(normal);
                            item.plane = Plane(plane_p1, normal);
                            plane_error_message.clear();
                        }
                    } else if (plane_input_mode == 1) {
                        auto r1 = edit_vec3_stepper(get_locale_cstr("label.point"),
                                          plane_point);
                        edit_result.activated |= r1.activated;
                        edit_result.deactivated_after_edit |= r1.deactivated_after_edit;
                        edit_result.value_changed |= r1.value_changed;
                        if (ImGui::Checkbox(
                                get_locale_cstr("label.pick_point_by_mouse"),
                                            &pick_point_by_mouse)) {
                            if (pick_point_by_mouse) {
                                pick_normal_by_mouse = false;
                            }
                        }
                        auto r2 = edit_vec3_stepper(get_locale_cstr("label.normal"),
                                          plane_normal, 0.1f);
                        edit_result.activated |= r2.activated;
                        edit_result.deactivated_after_edit |= r2.deactivated_after_edit;
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
        push_undo_now(item.id, before, get_locale_string("label.segment_mode") + " (Plane)");
    }
    if (edit_result.activated) {
        begin_edit(item.id);
    }
    if (edit_result.deactivated_after_edit) {
        end_edit(item.id, get_locale_string("label.segment_mode") + " (Plane)");
    }
}

void RenderVoxelList::render_collision_node_editor() {
    ImGui::SetNextWindowPos(ImVec2((float)window_width, (float)menu_height),
                            ImGuiCond_Always, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(360, 620), ImGuiCond_Once);
    if (ImGui::Begin(get_locale_cstr("window.collision_editor"))) {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it == items.end()) {
            ImGui::TextUnformatted(get_locale_cstr("label.no_active_item"));
        } else {
            RenderVoxelItem& item = *item_it->second;
            bool is_updating = item.write_count != 0;
            if (is_updating) {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "%s",
                                   get_locale_cstr("label.updating"));
            }
            if (is_updating) ImGui::BeginDisabled();

            if (ImGui::Button(get_locale_cstr("action.delete"))) {
                pending_delete_item_id = item.id;
                show_delete_confirm = true;
            }
            ImGui::SameLine();
            if (!item.stl_path.empty()) {
                if (ImGui::Button(get_locale_cstr("action.reload_stl"))) {
                    show_reload_stl_dialog = true;
                    reload_stl_item_id = item.id;
                    reload_stl_voxel_size = item.stl_voxel_size;
                }
                ImGui::SameLine();
            }
            if (ImGui::Button(get_locale_cstr("action.save_as_stl"))) {
                const char* filters[] = {"*.stl"};
                const char* file =
                    tinyfd_saveFileDialog(
                        get_locale_cstr("dialog.save_voxel_as_stl"),
                        "node_voxel.stl", 1, filters,
                        get_locale_cstr("dialog.stl_files"));
                if (file) {
                    std::vector<std::tuple<sinriv::kigstudio::voxel::Triangle,
                                           sinriv::kigstudio::voxel::vec3f>>
                        mesh;
                    int numTriangles = 0;
                    for (auto triangles :
                         sinriv::kigstudio::voxel::generateMesh(
                             item.voxel_grid_data, 0.5,
                             numTriangles, true)) {
                        mesh.push_back(triangles);
                    }
                    sinriv::kigstudio::voxel::saveMeshToASCIISTL(mesh, file);
                }
            }
            
            ImGui::SameLine();
            if (mouse_world_pos_picked_auto_snapping){
                if (ImGui::Button(get_locale_cstr("action.pick_pos_auto_snapping_stop"))) {
                    mouse_world_pos_picked_auto_snapping = false;
                }
            }else{
                if (ImGui::Button(get_locale_cstr("action.pick_pos_auto_snapping"))) {
                    mouse_world_pos_picked_auto_snapping = true;
                }
            }

            ImGui::Text(get_locale_cstr("label.render_item"), item.id);
            ImGui::Separator();

            // Undo / Redo buttons
            bool undo_disabled = !can_undo(item.id);
            bool redo_disabled = !can_redo(item.id);
            if (undo_disabled) ImGui::BeginDisabled();
            if (ImGui::Button(get_locale_cstr("action.undo"))) {
                undo(item.id);
            }
            if (undo_disabled) ImGui::EndDisabled();
            ImGui::SameLine();
            if (redo_disabled) ImGui::BeginDisabled();
            if (ImGui::Button(get_locale_cstr("action.redo"))) {
                redo(item.id);
            }
            if (redo_disabled) ImGui::EndDisabled();
            ImGui::Separator();

            ImGui::Checkbox(get_locale_cstr("label.auto_segment_update"),
                            &item.auto_segment_update);

            const char* segment_mode_names[] = {
                get_locale_cstr("mode.collision"), get_locale_cstr("mode.plane"),
                get_locale_cstr("mode.concave_cone"),
                get_locale_cstr("mode.split_disconnected"),
                get_locale_cstr("mode.neighbor")};
            const enum RenderVoxelItem::SegmentMode segment_modes[] = {
                RenderVoxelItem::COLLISION, RenderVoxelItem::PLANE,
                RenderVoxelItem::CONCAVE_CONE,
                RenderVoxelItem::SPLIT_DISCONNECTED,
                RenderVoxelItem::NEIGHBOR};
            int current_segment_mode = segment_modes[(int)item.segment_mode];
            if (ImGui::Combo(get_locale_cstr("label.segment_mode"),
                             &current_segment_mode,
                             segment_mode_names,
                             IM_ARRAYSIZE(segment_mode_names))) {
                push_undo_now(item.id, std::nullopt, get_locale_string("label.segment_mode"));
                item.segment_mode = segment_modes[current_segment_mode];
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
            }

            ImGui::Separator();

            // 体素刷选
            ImGui::Checkbox(get_locale_cstr("label.voxel_picking"),
                            &item.voxel_picking_enabled);
            if (item.voxel_picking_enabled) {
                if (!item.surface_cache_ready && !item.surface_cache_computing) {
                    item.surface_cache_computing = true;
                    item.surface_cache_progress = 0.0f;
                    // 在后台线程初始化表面缓存
                    std::thread([this, id = item.id]() {
                        auto it = this->items.find(id);
                        if (it == this->items.end()) return;
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
                    ImGui::TextUnformatted(get_locale_cstr("label.surface_cache_ready"));
                    ImGui::DragFloat(get_locale_cstr("label.pick_range"),
                                     &item.voxel_pick_range, 0.1f, 0.1f, 20.0f, "%.1f");
                }
                ImGui::Separator();
                if (ImGui::Button(get_locale_cstr("action.save_marked_voxels"))) {
                    const char* filters[] = {"*.vxgrid"};
                    const char* file = tinyfd_saveFileDialog(
                        get_locale_cstr("dialog.save_marked_voxels"),
                        "marked.vxgrid", 1, filters,
                        get_locale_cstr("dialog.marked_voxels_file"));
                    if (file) {
                        std::string error;
                        if (!sinriv::kigstudio::save(file, item.marked_voxels, &error)) {
                            tinyfd_messageBox("Error",
                                utf8_to_ansi(error.c_str()).c_str(),
                                "ok", "error", 1);
                        }
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button(get_locale_cstr("action.load_marked_voxels"))) {
                    const char* filters[] = {"*.vxgrid"};
                    const char* file = tinyfd_openFileDialog(
                        get_locale_cstr("dialog.load_marked_voxels"),
                        "", 1, filters,
                        get_locale_cstr("dialog.marked_voxels_file"), 0);
                    if (file) {
                        if (!sinriv::kigstudio::load(file, item.marked_voxels)) {
                            tinyfd_messageBox("Error",
                                utf8_to_ansi(get_locale_cstr("error.load_marked_failed")).c_str(),
                                "ok", "error", 1);
                        } else {
                            item.marked_voxels.global_position = item.voxel_grid_data.global_position;
                            item.marked_voxels.voxel_size = item.voxel_grid_data.voxel_size;
                            item.marked_voxels_dirty = true;
                        }
                    }
                }
            }

            if (is_updating) ImGui::EndDisabled();
        }
    }
    ImGui::End();
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
                std::string msg = get_locale_string("error.save_failed") + "\n" + last_save_error;
                tinyfd_messageBox("Error",
                    utf8_to_ansi(msg.c_str()).c_str(),
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
                std::string msg = get_locale_string("error.load_failed") + "\n" + last_load_error;
                tinyfd_messageBox("Error",
                    utf8_to_ansi(msg.c_str()).c_str(),
                    "ok", "error", 1);
            }
        }
        show_load_dialog = false;
    }
}

void RenderVoxelList::render_history_window() {
    if (!show_history_window) return;

    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it == items.end()) {
        if (ImGui::Begin(get_locale_cstr("window.history"), &show_history_window)) {
            ImGui::TextUnformatted(get_locale_cstr("label.no_active_item"));
        }
        ImGui::End();
        return;
    }

    RenderVoxelItem& item = *it->second;
    if (ImGui::Begin(get_locale_cstr("window.history"), &show_history_window)) {
        ImGui::Text(get_locale_cstr("label.render_item"), item.id);
        if (item.dirty) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "(*)");
        }
        ImGui::Separator();

        const size_t undo_count = item.undo_stack.size();
        const size_t redo_count = item.redo_stack.size();
        const int total = static_cast<int>(undo_count + redo_count + 1);
        const int current_idx = static_cast<int>(undo_count);

        ImGui::Text("%s: %d", get_locale_cstr("label.history_total"), total);
        ImGui::Separator();

        // Copy descriptions locally to avoid accessing modified stacks during iteration
        std::vector<std::string> undo_descs;
        undo_descs.reserve(undo_count);
        for (size_t i = 0; i < undo_count; ++i) {
            undo_descs.push_back(item.undo_stack[i].description.empty() ? "State" : item.undo_stack[i].description);
        }
        std::vector<std::string> redo_descs;
        redo_descs.reserve(redo_count);
        for (size_t i = 0; i < redo_count; ++i) {
            redo_descs.push_back(item.redo_stack[redo_count - 1 - i].description.empty() ? "State" : item.redo_stack[redo_count - 1 - i].description);
        }

        // Undo list (oldest first)
        if (undo_count > 0) {
            ImGui::TextDisabled("%s", get_locale_cstr("label.history_undo"));
            ImGui::PushID("undo");
            for (size_t i = 0; i < undo_count; ++i) {
                char label[128];
                snprintf(label, sizeof(label), "[%zu] %s",
                         i, undo_descs[i].c_str());
                ImGui::PushID(static_cast<int>(i));
                if (i == undo_count - 1) {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.8f, 1.0f, 1.0f));
                }
                if (ImGui::Selectable(label, i == undo_count - 1)) {
                    // Click to jump: undo until we reach this state
                    size_t steps = undo_count - 1 - i;
                    for (size_t s = 0; s < steps; ++s) {
                        undo(item.id);
                    }
                }
                if (i == undo_count - 1) {
                    ImGui::PopStyleColor();
                }
                ImGui::PopID();
            }
            ImGui::PopID();
        }

        // Current state
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.2f, 1.0f, 0.4f, 1.0f));
        ImGui::PushID("current");
        ImGui::Selectable(get_locale_cstr("label.history_current"), true,
                          ImGuiSelectableFlags_Disabled);
        ImGui::PopID();
        ImGui::PopStyleColor();

        // Redo list (newest first, displayed top to bottom)
        if (redo_count > 0) {
            ImGui::TextDisabled("%s", get_locale_cstr("label.history_redo"));
            ImGui::PushID("redo");
            for (size_t i = 0; i < redo_count; ++i) {
                char label[128];
                snprintf(label, sizeof(label), "[%zu] %s",
                         i, redo_descs[i].c_str());
                ImGui::PushID(static_cast<int>(i));
                if (ImGui::Selectable(label, false)) {
                    // Click to jump: redo until we reach this state
                    size_t steps = i + 1;
                    for (size_t s = 0; s < steps; ++s) {
                        redo(item.id);
                    }
                }
                ImGui::PopID();
            }
            ImGui::PopID();
        }

        if (undo_count == 0 && redo_count == 0) {
            ImGui::TextDisabled("%s", get_locale_cstr("label.history_empty"));
        }
    }
    ImGui::End();
}

void RenderVoxelList::render_log_window() {
    if (!show_log_window) return;

    std::vector<std::string> log_copy;
    {
        std::lock_guard<std::mutex> lock(queue_log_mutex);
        log_copy = queue_log;
    }

    ImGui::SetNextWindowSize(ImVec2(500, 300), ImGuiCond_Once);
    if (ImGui::Begin(get_locale_cstr("window.log"), &show_log_window)) {
        if (log_copy.empty()) {
            ImGui::TextDisabled("No log entries.");
        } else {
            ImGui::BeginChild("log_scrolling", ImVec2(0, 0), false,
                              ImGuiWindowFlags_HorizontalScrollbar);
            for (const auto& entry : log_copy) {
                ImGui::TextUnformatted(entry.c_str());
            }
            if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 5.0f) {
                ImGui::SetScrollHereY(1.0f);
            }
            ImGui::EndChild();
        }
    }
    ImGui::End();
}

}  // namespace sinriv::ui::render
