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
namespace {

bool autoDetectJointRadius(RenderVoxelList::RenderVoxelItem& item,
                           size_t joint_index) {
    if (joint_index >= item.picked_skeleton_points.size()) {
        return false;
    }

    using Vec3f = sinriv::kigstudio::sdf::joint::Vec3f;
    using Frame = sinriv::kigstudio::sdf::joint::Frame;

    auto& picked = item.picked_skeleton_points[joint_index];

    auto get_pos = [&](size_t idx) -> Vec3f {
        const auto& pos = item.picked_skeleton_points[idx].position;
        return {pos.x, pos.y, pos.z};
    };

    Vec3f start = get_pos(joint_index);
    Vec3f end;
    if (picked.use_custom_direction) {
        end =
            Vec3f(picked.custom_direction_end.x, picked.custom_direction_end.y,
                  picked.custom_direction_end.z);
    } else {
        if (joint_index + 1 < item.picked_skeleton_points.size()) {
            end = get_pos(joint_index + 1);
        } else if (item.picked_skeleton_points.size() >= 2) {
            Vec3f prev = get_pos(joint_index - 1);
            end = start + (start - prev);
        } else {
            end = start + Vec3f(0, 0, 10);
        }
    }
    if ((end - start).length() < 1e-6f) {
        end = start + Vec3f(0, 0, 1);
    }

    Frame frame;
    if (picked.use_custom_direction) {
        frame = sinriv::kigstudio::sdf::joint::buildFrame(
            start, end, picked.rotation_angle);
    } else {
        frame = sinriv::kigstudio::sdf::joint::buildFrameAlignedY(start, end);
    }

    const auto& voxel_size = item.voxel_grid_data.voxel_size;
    const float half_thickness =
        std::max({voxel_size.x, voxel_size.y, voxel_size.z}) * 0.75f;
    float socket_radius = -1.0f;
    float head_radius = -1.0f;

    for (const auto& voxel : item.voxel_grid_data) {
        const auto world = item.voxel_grid_data.voxelCenterToWorld(voxel);
        const Vec3f local = frame.worldToLocal({world.x, world.y, world.z});
        const float radius = std::sqrt(local.x * local.x + local.y * local.y);

        if (std::abs(local.z - picked.socket_cone_offset) <= half_thickness) {
            socket_radius = std::max(socket_radius, radius);
        }
        if (std::abs(local.z - picked.head_cone_offset) <= half_thickness) {
            head_radius = std::max(head_radius, radius);
        }
    }

    bool changed = false;
    if (socket_radius > 0.0f &&
        std::abs(socket_radius - picked.socket_cone_radius) > 1e-4f) {
        picked.socket_cone_radius = socket_radius * 1.2f;
        changed = true;
    }
    if (head_radius > 0.0f &&
        std::abs(head_radius - picked.head_cone_radius) > 1e-4f) {
        picked.head_cone_radius = head_radius * 1.2f;
        changed = true;
    }

    const float socket_base_z =
        picked.socket_cone_offset +
        picked.socket_cone_radius / std::tan(picked.socket_cone_angle);
    const float cylinder_offset =
        (picked.head_cone_offset + socket_base_z) * 0.5f;
    if (std::abs(cylinder_offset - picked.male_cylinder_offset) > 1e-4f) {
        picked.male_cylinder_offset = cylinder_offset;
        changed = true;
    }

    // Initialize socket fillet cylinder
    const float new_fillet_radius = picked.socket_cone_radius;
    const float new_fillet_offset = 0.0f;
    const float new_fillet_height =
        std::max(0.0f, socket_base_z - picked.male_cylinder_offset) / 3.0f;
    if (std::abs(new_fillet_radius - picked.socket_fillet_radius) > 1e-4f ||
        std::abs(new_fillet_offset - picked.socket_fillet_offset) > 1e-4f ||
        std::abs(new_fillet_height - picked.socket_fillet_height) > 1e-4f) {
        picked.socket_fillet_radius = new_fillet_radius;
        picked.socket_fillet_offset = new_fillet_offset;
        picked.socket_fillet_height = new_fillet_height;
        changed = true;
    }

    // Initialize head fillet cone
    // height = distance between cone apexes + distance from male cylinder to
    // head cone apex / 3
    const float new_head_fillet_height =
        (picked.head_cone_offset - picked.socket_cone_offset) +
        (picked.male_cylinder_offset - picked.head_cone_offset) / 3.0f;
    if (std::abs(new_head_fillet_height - picked.head_fillet_height) > 1e-4f) {
        picked.head_fillet_height = new_head_fillet_height;
        changed = true;
    }

    return changed;
}

}  // namespace

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

                // ===== Tab: Voxel Picking =====
                if (ImGui::BeginTabItem(get_locale_cstr("tab.voxel_picking"),
                                        nullptr, flags_voxel)) {
                    object_editor_tab = 1;
                    render_object_editor_voxel_tab_content(item);
                    ImGui::EndTabItem();
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip(get_locale_cstr("tooltip.voxel_picking"));
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

void RenderVoxelList::render_file_status_tab(RenderVoxelItem& item) {
    item.showSilhouetteCenter = false;

    // Source Type 单选按钮组
    ImGui::Separator();
    int source_type = item.source_type;
    if (ImGui::RadioButton(get_locale_cstr("label.source_file"), &source_type,
                           0)) {
        push_undo_now(item.id, std::nullopt, "Source Type");
        item.source_type = source_type;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton(get_locale_cstr("label.source_node"), &source_type,
                           1)) {
        push_undo_now(item.id, std::nullopt, "Source Type");
        item.source_type = source_type;
    }

    // 加载模式选择（File / Node 通用）
    const char* load_mode_names[] = {
        get_locale_cstr("label.stl_load_mode.default"),
        get_locale_cstr("label.stl_load_mode.silhouette"),
        get_locale_cstr("label.stl_load_mode.surface_only"),
        get_locale_cstr("label.stl_load_mode.mesh_only"),
        get_locale_cstr("label.stl_load_mode.convex_hull"),
    };
    int load_mode = item.stl_load_mode;
    if (ImGui::Combo(get_locale_cstr("label.stl_load_mode"), &load_mode,
                     load_mode_names, IM_ARRAYSIZE(load_mode_names))) {
        push_undo_now(item.id, std::nullopt, "STL Load Mode");
        item.stl_load_mode = load_mode;
        if (item.stl_load_mode ==
                static_cast<int>(StlLoadMode::SURFACE_ONLY) ||
            item.stl_load_mode ==
                static_cast<int>(StlLoadMode::MESH_ONLY)) {
            item.load_as_sdf = false;
        }
        if (item.stl_load_mode ==
            static_cast<int>(StlLoadMode::MESH_ONLY)) {
            item.mesh_only = true;
        } else {
            item.mesh_only = false;
        }
    }
    if (ImGui::IsItemHovered()) {
        const char* tooltip_key = nullptr;
        switch (load_mode) {
            case static_cast<int>(StlLoadMode::DEFAULT):
                tooltip_key = "tooltip.stl_load_mode.default";
                break;
            case static_cast<int>(StlLoadMode::SILHOUETTE):
                tooltip_key = "tooltip.stl_load_mode.silhouette";
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

    // Silhouette 中心设置
    if (item.stl_load_mode == static_cast<int>(StlLoadMode::SILHOUETTE)) {
        item.showSilhouetteCenter = true;
        auto center_result =
            edit_vec3_stepper(get_locale_cstr("label.silhouette_center"),
                              item.silhouette_center, 0.1f);
        if (center_result.deactivated_after_edit) {
            push_undo_now(item.id, std::nullopt, "Silhouette Center");
        }
        ImGui::TextUnformatted(get_locale_cstr("label.silhouette_subdivision"));
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(get_locale_cstr("tooltip.silhouette_subdivision"));
        }
        ImGui::SameLine();
        const float btn_w = ImGui::GetFrameHeight();
        if (ImGui::Button("-##silhouette_subdiv", ImVec2(btn_w, 0))) {
            if (item.silhouette_subdivision > 1) {
                --item.silhouette_subdivision;
                push_undo_now(item.id, std::nullopt,
                              "Silhouette Subdivision");
            }
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0f);
        if (ImGui::InputInt("##silhouette_subdiv_val",
                            &item.silhouette_subdivision,
                            0, 0,
                            ImGuiInputTextFlags_CharsDecimal)) {
            if (item.silhouette_subdivision < 1)
                item.silhouette_subdivision = 1;
            push_undo_now(item.id, std::nullopt,
                          "Silhouette Subdivision");
        }
        ImGui::SameLine();
        if (ImGui::Button("+##silhouette_subdiv", ImVec2(btn_w, 0))) {
            ++item.silhouette_subdivision;
            push_undo_now(item.id, std::nullopt,
                          "Silhouette Subdivision");
        }

        // Inner wall radius
        ImGui::TextUnformatted(get_locale_cstr("label.inner_wall_radius"));
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(get_locale_cstr("tooltip.inner_wall_radius"));
        }
        ImGui::SameLine();
        if (ImGui::Button("-##inner_wall", ImVec2(btn_w, 0))) {
            item.inner_wall_radius = std::max(0.0f, item.inner_wall_radius - 0.5f);
            push_undo_now(item.id, std::nullopt, "Inner Wall Radius");
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0f);
        if (ImGui::InputFloat("##inner_wall_val", &item.inner_wall_radius,
                              0.0f, 0.0f, "%.1f",
                              ImGuiInputTextFlags_CharsDecimal)) {
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
            float nearest = item.origin_mesh_renderer.get_min_distance(
                item.silhouette_center);
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
        if (ImGui::SliderFloat("##simplify_slider", &item.simplify_ratio,
                               0.01f, 1.0f, "%.2f")) {
            if (item.simplify_ratio < 0.01f) item.simplify_ratio = 0.01f;
            push_undo_now(item.id, std::nullopt, "Simplify Ratio");
        }
        if (!simplify_enabled) ImGui::EndDisabled();
    }

    if (item.source_type == 0) {
        // ===================== FILE MODE =====================
        // STL 路径编辑
        static char stl_path_buf[1024] = {};
        static int last_path_item_id = -1;
        if (last_path_item_id != item.id) {
            strncpy(stl_path_buf, item.stl_path.c_str(),
                    sizeof(stl_path_buf) - 1);
            stl_path_buf[sizeof(stl_path_buf) - 1] = '\0';
            last_path_item_id = item.id;
        }

        ImGui::TextUnformatted(get_locale_cstr("label.stl_path"));
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x -
                                ImGui::GetFrameHeight() -
                                ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputText("##stl_path", stl_path_buf, sizeof(stl_path_buf),
                             ImGuiInputTextFlags_EnterReturnsTrue)) {
            const std::string new_path = stl_path_buf;
            if (new_path != item.stl_path) {
                push_undo_now(item.id, std::nullopt, "STL Path");
                item.stl_path = new_path;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.browse"))) {
            const char* filters[] = {"*.stl"};
            const char* file = tinyfd_openFileDialog(
                utf8_to_ansi(get_locale_cstr("dialog.open_stl_title"))
                    .c_str(),
                "", 1, filters,
                utf8_to_ansi(get_locale_cstr("dialog.stl_file")).c_str(), 0);
            if (file) {
                const std::string new_path = tinyfd_path_to_utf8(file);
                if (new_path != item.stl_path) {
                    push_undo_now(item.id, std::nullopt, "STL Path");
                    item.stl_path = new_path;
                    strncpy(stl_path_buf, new_path.c_str(),
                            sizeof(stl_path_buf) - 1);
                    stl_path_buf[sizeof(stl_path_buf) - 1] = '\0';
                }
            }
        }

        // SDF 勾选框
        if (item.stl_load_mode !=
                static_cast<int>(StlLoadMode::SURFACE_ONLY) &&
            item.stl_load_mode !=
                static_cast<int>(StlLoadMode::MESH_ONLY)) {
            bool load_as_sdf = item.load_as_sdf;
            if (ImGui::Checkbox(get_locale_cstr("label.load_as_sdf"),
                                &load_as_sdf)) {
                push_undo_now(item.id, std::nullopt, "Load as SDF");
                item.load_as_sdf = load_as_sdf;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(get_locale_cstr("tooltip.load_as_sdf"));
            }
            
            // SDF precision mode — show whenever load_as_sdf is on.
            // Value is cached on the item and synced to SDF_Mesh on load.
            if (item.load_as_sdf) {
                int mode = static_cast<int>(item.sdf_precision_cache);
                const char* mode_names[] = {
                    get_locale_cstr("label.sdf_precision.fast"),
                    get_locale_cstr("label.sdf_precision.precise"),
                    get_locale_cstr("label.sdf_precision.redundant"),
                };
                if (ImGui::Combo(
                        get_locale_cstr("label.sdf_precision_mode"),
                        &mode, mode_names, 3)) {
                    item.sdf_precision_cache =
                        static_cast<sinriv::kigstudio::sdf::SDFPrecision>(
                            mode);
                    // Also sync to live SDF_Mesh if it exists
                    if (item.sdf_data) {
                        auto* sdf_mesh =
                            dynamic_cast<sinriv::kigstudio::sdf::SDF_Mesh*>(
                                item.sdf_data.get());
                        if (sdf_mesh)
                            sdf_mesh->precision_mode =
                                item.sdf_precision_cache;
                    }
                    push_undo_now(item.id, std::nullopt,
                                  "SDF Precision Mode");
                }
            }
            
            int voxel_prec = static_cast<int>(item.voxel_precision);
            const char* vp_names[] = {
                get_locale_cstr("label.sdf_precision.fast"),
                get_locale_cstr("label.sdf_precision.precise"),
                get_locale_cstr("label.sdf_precision.redundant"),
            };
            if (ImGui::Combo(
                    get_locale_cstr("label.voxel_precision"),
                    &voxel_prec, vp_names, 3)) {
                item.voxel_precision =
                    static_cast<sinriv::kigstudio::sdf::SDFPrecision>(
                        voxel_prec);
                push_undo_now(item.id, std::nullopt,
                              "Voxel Precision");
            }
        }

        // Voxel Size
        if (!item.mesh_only) {
            ImGui::Separator();
            ImGui::TextUnformatted(get_locale_cstr("label.voxel_size"));
            ImGui::SameLine();
            const float button_size = ImGui::GetFrameHeight();
            if (ImGui::Button("-##voxelsize", ImVec2(button_size, 0))) {
                auto tmp = item.stl_voxel_size / 2.0f;
                if (tmp >= 0.0001f) {
                    item.stl_voxel_size = tmp;
                }
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(80.0f);
            ImGui::DragFloat("##VoxelSize", &item.stl_voxel_size, 0.1f, 0.0f,
                             0.0f, "%.4f");
            ImGui::SameLine();
            if (ImGui::Button("+##voxelsize", ImVec2(button_size, 0))) {
                item.stl_voxel_size = item.stl_voxel_size * 2.0f;
                if (item.stl_voxel_size > 1000.0f) {
                    item.stl_voxel_size = 1000.0f;
                }
            }
        }

        // 重新加载按钮
        ImGui::Separator();
        bool first = true;
        if (!item.stl_path.empty()) {
            if (ImGui::Button(get_locale_cstr("action.reload_stl"))) {
                queue_reload_stl(item.id, item.stl_voxel_size, item.stl_path,
                                 item.stl_load_mode, item.load_as_sdf,
                                 item.voxel_precision);
            }
            first = false;
        }
        if (!item.source_triangles.empty()) {
            if (!first) {
                ImGui::SameLine();
            }
            if (ImGui::Button(
                    get_locale_cstr("action.export_source_stl"))) {
                const char* filters[] = {"*.stl"};
                const char* file = tinyfd_saveFileDialog(
                    utf8_to_ansi(get_locale_cstr("action.export_source_stl"))
                        .c_str(),
                    "source.stl", 1, filters,
                    utf8_to_ansi(get_locale_cstr("dialog.stl_files"))
                        .c_str());
                if (file) {
                    std::vector<std::tuple<sinriv::kigstudio::voxel::Triangle,
                                           sinriv::kigstudio::voxel::vec3f>>
                        mesh_triangles;
                    mesh_triangles.reserve(item.source_triangles.size());
                    for (const auto& tri : item.source_triangles) {
                        mesh_triangles.push_back(
                            {tri, sinriv::kigstudio::voxel::calcTriangleNormal(
                                      tri)});
                    }
                    sinriv::kigstudio::voxel::saveMeshToASCIISTL(
                        mesh_triangles, tinyfd_path_to_utf8(file));
                }
            }
        }

        // 后台加载进度条与取消按钮
        if (item.write_count > 0) {
            ImGui::Separator();
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
    } else {
        // ===================== NODE MODE =====================
        // 节点选择器
        std::vector<std::pair<int, std::string>> candidates;
        if (item.manager) {
            for (auto& [other_id, other] : item.manager->items) {
                if (other_id == item.id)
                    continue;
                // 避免 source-node 循环引用
                if (item.manager->would_form_source_cycle(item.id, other_id))
                    continue;
                candidates.push_back(
                    {other_id, "Node " + std::to_string(other_id)});
            }
        }
        int current_source = -1;
        std::vector<const char*> candidate_names;
        for (size_t i = 0; i < candidates.size(); ++i) {
            candidate_names.push_back(candidates[i].second.c_str());
            if (candidates[i].first == item.source_node_id) {
                current_source = static_cast<int>(i);
            }
        }
        if (ImGui::Combo(get_locale_cstr("label.source_node_id"),
                         &current_source, candidate_names.data(),
                         static_cast<int>(candidate_names.size()))) {
            push_undo_now(item.id, std::nullopt, "Source Node");
            if (current_source >= 0 &&
                current_source < static_cast<int>(candidates.size())) {
                item.source_node_id = candidates[current_source].first;
            }
        }

        // 数据类型选择
        if (item.source_node_id >= 0) {
            ImGui::Separator();
            bool has_mesh = false;
            bool has_sdf = false;
            bool has_voxel = false;
            if (item.manager) {
                auto src_it = item.manager->items.find(item.source_node_id);
                if (src_it != item.manager->items.end()) {
                    has_mesh = !src_it->second->source_triangles.empty();
                    has_sdf = src_it->second->sdf_data != nullptr;
                    has_voxel =
                        !src_it->second->voxel_grid_data.chunks.empty();
                }
            }

            // 如果当前选中的数据类型不可用，自动回退到第一个可用类型
            if (item.node_source_data_type == 0 && !has_mesh) {
                item.node_source_data_type = has_sdf ? 1 : (has_voxel ? 2 : 0);
            }
            if (item.node_source_data_type == 1 && !has_sdf) {
                item.node_source_data_type = has_mesh ? 0 : (has_voxel ? 2 : 0);
            }
            if (item.node_source_data_type == 2 && !has_voxel) {
                item.node_source_data_type = has_mesh ? 0 : (has_sdf ? 1 : 0);
            }

            bool any_data_available = has_mesh || has_sdf || has_voxel;

            int data_type = item.node_source_data_type;
            if (has_mesh) {
                if (ImGui::RadioButton(
                        get_locale_cstr("label.source_data_mesh"), &data_type,
                        0)) {
                    push_undo_now(item.id, std::nullopt,
                                  "Node Source Data Type");
                    item.node_source_data_type = data_type;
                }
            }
            if (has_sdf) {
                if (has_mesh)
                    ImGui::SameLine();
                if (ImGui::RadioButton(
                        get_locale_cstr("label.source_data_sdf"), &data_type,
                        1)) {
                    push_undo_now(item.id, std::nullopt,
                                  "Node Source Data Type");
                    item.node_source_data_type = data_type;
                }
            }
            if (has_voxel) {
                if (has_mesh || has_sdf)
                    ImGui::SameLine();
                if (ImGui::RadioButton(
                        get_locale_cstr("label.source_data_voxel"), &data_type,
                        2)) {
                    push_undo_now(item.id, std::nullopt,
                                  "Node Source Data Type");
                    item.node_source_data_type = data_type;
                }
            }

            if (!any_data_available) {
                ImGui::TextColored(
                    ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                    "Source node has no mesh/SDF/voxel data available.");
            }

            // SDF 细分比例与简化模型
            if (item.node_source_data_type == 1) {
                ImGui::Separator();
                ImGui::DragInt(
                    get_locale_cstr("label.node_source_sdf_subdivisions"),
                    &item.node_source_sdf_subdivisions, 1, 1, 8);
                if (ImGui::Checkbox(
                        get_locale_cstr("label.node_source_sdf_simplify"),
                        &item.node_source_sdf_simplify)) {
                    push_undo_now(item.id, std::nullopt,
                                  "Node Source SDF Simplify");
                }
                if (item.node_source_sdf_simplify) {
                    ImGui::DragFloat(
                        get_locale_cstr(
                            "label.node_source_sdf_simplify_ratio"),
                        &item.node_source_sdf_simplify_ratio, 0.01f, 0.01f,
                        1.0f, "%.2f");
                }
            }

            // Load as SDF checkbox for mesh/SDF node sources
            if ((item.node_source_data_type == 0 ||
                 item.node_source_data_type == 1) &&
                item.stl_load_mode !=
                    static_cast<int>(StlLoadMode::SURFACE_ONLY) &&
                item.stl_load_mode !=
                    static_cast<int>(StlLoadMode::MESH_ONLY)) {
                ImGui::Separator();
                bool load_as_sdf = item.load_as_sdf;
                if (ImGui::Checkbox(get_locale_cstr("label.load_as_sdf"),
                                    &load_as_sdf)) {
                    push_undo_now(item.id, std::nullopt, "Load as SDF");
                    item.load_as_sdf = load_as_sdf;
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip(get_locale_cstr("tooltip.load_as_sdf"));
                }
                int voxel_prec =
                    static_cast<int>(item.voxel_precision);
                const char* vp_names2[] = {
                    get_locale_cstr("label.sdf_precision.fast"),
                    get_locale_cstr("label.sdf_precision.precise"),
                    get_locale_cstr("label.sdf_precision.redundant"),
                };
                if (ImGui::Combo(
                        get_locale_cstr("label.voxel_precision"),
                        &voxel_prec, vp_names2, 3)) {
                    item.voxel_precision =
                        static_cast<
                            sinriv::kigstudio::sdf::SDFPrecision>(
                            voxel_prec);
                    push_undo_now(item.id, std::nullopt,
                                  "Voxel Precision");
                }
            }

            // 重新加载按钮
            ImGui::Separator();
            ImGui::BeginDisabled(!any_data_available);
            if (ImGui::Button(
                    get_locale_cstr("action.reload_from_node"))) {
                queue_reload_stl(
                    item.id, item.stl_voxel_size, item.stl_path,
                    item.stl_load_mode, item.load_as_sdf,
                    item.voxel_precision, item.source_node_id,
                    item.node_source_data_type,
                    item.node_source_sdf_subdivisions,
                    item.node_source_sdf_simplify,
                    item.node_source_sdf_simplify_ratio);
            }
            ImGui::EndDisabled();

            // 后台加载进度条与取消按钮
            if (item.write_count > 0) {
                ImGui::Separator();
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
        }
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
        ImGui::RadioButton(get_locale_cstr("label.export_mode_standard"),
                           &export_stl_mode, 0);
        ImGui::RadioButton(get_locale_cstr("label.export_mode_smooth"),
                           &export_stl_mode, 1);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(get_locale_cstr("tooltip.export_mode_smooth"));
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

    if (item.mesh_only) {
        // mesh_only 模型仅支持 Plane 与 Repair Mesh 两种处理模式
        const char* mesh_only_mode_names[] = {
            get_locale_cstr("mode.plane"),
            get_locale_cstr("mode.repair")};
        const enum RenderVoxelItem::SegmentMode mesh_only_modes[] = {
            RenderVoxelItem::PLANE, RenderVoxelItem::REPAIR_MESH};
        int current_mesh_only_mode =
            (item.segment_mode == RenderVoxelItem::REPAIR_MESH) ? 1 : 0;
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
            get_locale_cstr("mode.sdf_node_split")};
        const enum RenderVoxelItem::SegmentMode segment_modes[] = {
            RenderVoxelItem::COLLISION,    RenderVoxelItem::PLANE,
            RenderVoxelItem::CONCAVE_CONE, RenderVoxelItem::SPLIT_DISCONNECTED,
            RenderVoxelItem::NEIGHBOR,     RenderVoxelItem::FILL_INTERIOR,
            RenderVoxelItem::CHAIN,        RenderVoxelItem::SDF_NODE_SPLIT};
        int current_segment_mode = segment_modes[(int)item.segment_mode];
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

void RenderVoxelList::render_object_editor_chain_mode(RenderVoxelItem& item) {
    EditResult chain_edit_result;

    // chain_min_radius
    ImGui::DragInt(get_locale_cstr("label.chain_min_radius"),
                   &item.chain_min_radius, 1, 1, 20);
    chain_edit_result.activated |= ImGui::IsItemActivated();
    chain_edit_result.deactivated_after_edit |=
        ImGui::IsItemDeactivatedAfterEdit();

    // use_cgal_skeleton
    if (!item.stl_path.empty()) {
        auto before = capture_snapshot(item);
        if (ImGui::Checkbox(get_locale_cstr("label.use_cgal_skeleton"),
                            &item.use_cgal_skeleton)) {
            push_undo_now(item.id, before,
                          get_locale_string("label.use_cgal_skeleton"));
        }
    }
    if (ImGui::Button(get_locale_cstr("action.extract_skeleton"))) {
        queue_extract_skeleton(item.id);
    }
    ImGui::Separator();
    ImGui::Text(get_locale_cstr("label.picked_skeleton_points"),
                static_cast<int>(item.picked_skeleton_points.size()));
    ImGui::SameLine();
    const std::string clear_picked_label =
        get_locale_string("action.clear_vertices") + "##PickedSkeletonPoints";
    if (ImGui::Button(clear_picked_label.c_str())) {
        push_undo_now(item.id, std::nullopt,
                      get_locale_string("action.clear_vertices"));
        item.picked_skeleton_points.clear();
        item.joint_wireframe_dirty = true;
    }
    ImGui::SameLine();
    const std::string init_radii_label =
        get_locale_string("action.init_all_joint_radii") +
        "##InitAllJointRadii";
    if (ImGui::Button(init_radii_label.c_str())) {
        const auto before = capture_snapshot(item);
        bool changed = false;
        for (size_t idx = 0; idx < item.picked_skeleton_points.size(); ++idx) {
            changed |= autoDetectJointRadius(item, idx);
        }
        if (changed) {
            push_undo_now(item.id, before,
                          get_locale_string("action.init_all_joint_radii"));
            item.joint_wireframe_dirty = true;
        }
    }
    int erase_picked_skeleton_index = -1;
    bool moved_picked_skeleton_point = false;
    static int pick_direction_index = -1;
    for (size_t i = 0; i < item.picked_skeleton_points.size(); ++i) {
        auto& picked = item.picked_skeleton_points[i];
        const auto& p = picked.position;
        ImGui::PushID(static_cast<int>(i));
        if (ImGui::Button("<")) {
            push_undo_now(item.id, std::nullopt,
                          get_locale_string("action.move"));
            item.move_picked_skeleton_point(i, -1);
            moved_picked_skeleton_point = true;
        }
        ImGui::SameLine();
        if (ImGui::Button(">")) {
            push_undo_now(item.id, std::nullopt,
                          get_locale_string("action.move"));
            item.move_picked_skeleton_point(i, 1);
            moved_picked_skeleton_point = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("X")) {
            push_undo_now(item.id, std::nullopt,
                          get_locale_string("action.delete"));
            erase_picked_skeleton_index = static_cast<int>(i);
            item.joint_wireframe_dirty = true;
        }
        ImGui::SameLine();
        ImGui::Text("#%d order=%d: %.3f, %.3f, %.3f", static_cast<int>(i),
                    picked.order, p.x, p.y, p.z);

        // ===== Joint Editor =====
        char joint_label[64];
        snprintf(joint_label, sizeof(joint_label), "%s #%d",
                 get_locale_cstr("label.joint"), static_cast<int>(i));
        if (ImGui::CollapsingHeader(joint_label)) {
            bool dirty = false;

            const std::string auto_detect_label =
                get_locale_string("action.auto_detect_joint_radius") +
                "##AutoDetectJointRadius";
            if (ImGui::Button(auto_detect_label.c_str())) {
                const auto before = capture_snapshot(item);
                if (autoDetectJointRadius(item, i)) {
                    push_undo_now(
                        item.id, before,
                        get_locale_string("action.auto_detect_joint_radius"));
                    dirty = true;
                }
            }

            // Custom direction
            if (ImGui::Checkbox(get_locale_cstr("label.custom_direction"),
                                &picked.use_custom_direction)) {
                dirty = true;
            }
            chain_edit_result.activated |= ImGui::IsItemActivated();
            chain_edit_result.deactivated_after_edit |=
                ImGui::IsItemDeactivatedAfterEdit();
            if (picked.use_custom_direction) {
                auto r = edit_local_position_stepper(
                    get_locale_cstr("label.direction_end"),
                    picked.custom_direction_end, 0.1f, false, false);
                chain_edit_result.activated |= r.activated;
                chain_edit_result.deactivated_after_edit |=
                    r.deactivated_after_edit;
                if (r.value_changed)
                    dirty = true;

                if (pick_direction_index == (int)i) {
                    if (ImGui::Button(get_locale_cstr("action.stop_picking_"
                                                      "direction"))) {
                        pick_direction_index = -1;
                    }
                } else {
                    if (ImGui::Button(
                            get_locale_cstr("action.pick_direction"))) {
                        pick_direction_index = (int)i;
                    }
                }
            }

            // Socket cone
            if (ImGui::CollapsingHeader(get_locale_cstr("label.socket_cone"))) {
                ImGui::PushID("SocketCone");
                if (ImGui::DragFloat(get_locale_cstr("label.offset"),
                                     &picked.socket_cone_offset, 0.1f, 0.0f,
                                     100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.angle"),
                                     &picked.socket_cone_angle, 0.01f, 0.01f,
                                     1.5f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.radius"),
                                     &picked.socket_cone_radius, 0.1f, 0.1f,
                                     50.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.socket_fillet_radius"),
                                     &picked.socket_fillet_radius, 0.1f, 0.0f,
                                     50.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.socket_fillet_height"),
                                     &picked.socket_fillet_height, 0.1f, 0.0f,
                                     100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.socket_fillet_offset"),
                                     &picked.socket_fillet_offset, 0.1f, 0.0f,
                                     100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                ImGui::PopID();
            }

            // Head cone
            if (ImGui::CollapsingHeader(get_locale_cstr("label.head_cone"))) {
                ImGui::PushID("HeadCone");
                if (ImGui::DragFloat(get_locale_cstr("label.offset"),
                                     &picked.head_cone_offset, 0.1f, 0.0f,
                                     100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.radius"),
                                     &picked.head_cone_radius, 0.1f, 0.1f,
                                     50.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.head_fillet_height"),
                                     &picked.head_fillet_height, 0.1f, 0.0f,
                                     100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                ImGui::PopID();
            }

            // Support cones
            if (ImGui::CollapsingHeader(
                    get_locale_cstr("label.support_cones"))) {
                if (ImGui::DragFloat(
                        get_locale_cstr("label.socket_support_offset"),
                        &picked.socket_support_offset, 0.1f, 0.0f, 100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(
                        get_locale_cstr("label.socket_support_radius"),
                        &picked.socket_support_radius, 0.1f, 0.1f, 50.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(
                        get_locale_cstr("label.head_support_offset"),
                        &picked.head_support_offset, 0.1f, 0.0f, 100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(
                        get_locale_cstr("label.head_support_radius"),
                        &picked.head_support_radius, 0.1f, 0.1f, 50.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
            }

            // Cylinder
            if (ImGui::CollapsingHeader(get_locale_cstr("label.cylinder"))) {
                if (ImGui::DragFloat(get_locale_cstr("label.cylinder_offset"),
                                     &picked.male_cylinder_offset, 0.1f, 0.0f,
                                     100.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.cylinder_radius"),
                                     &picked.male_cylinder_radius, 0.1f, 0.1f,
                                     50.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
                if (ImGui::DragFloat(get_locale_cstr("label.female_gap"),
                                     &picked.female_gap, 0.01f, 0.0f, 10.0f))
                    dirty = true;
                chain_edit_result.activated |= ImGui::IsItemActivated();
                chain_edit_result.deactivated_after_edit |=
                    ImGui::IsItemDeactivatedAfterEdit();
            }

            // Slot
            if (ImGui::DragFloat(get_locale_cstr("label.slot_extra"),
                                 &picked.slot_extra, 0.1f, 0.0f, 10.0f))
                dirty = true;
            chain_edit_result.activated |= ImGui::IsItemActivated();
            chain_edit_result.deactivated_after_edit |=
                ImGui::IsItemDeactivatedAfterEdit();

            // Rotation
            if (ImGui::DragFloat(get_locale_cstr("label.rotation_angle"),
                                 &picked.rotation_angle, 0.01f, -3.14f, 3.14f))
                dirty = true;
            chain_edit_result.activated |= ImGui::IsItemActivated();
            chain_edit_result.deactivated_after_edit |=
                ImGui::IsItemDeactivatedAfterEdit();

            if (dirty) {
                item.joint_wireframe_dirty = true;
            }
        }

        ImGui::PopID();
        if (moved_picked_skeleton_point) {
            item.joint_wireframe_dirty = true;
            break;
        }
    }

    // Handle direction picking
    if (pick_direction_index >= 0 &&
        pick_direction_index < (int)item.picked_skeleton_points.size() &&
        mouse_world_pos_valid && mouse_world_pos_picked) {
        auto& picked = item.picked_skeleton_points[pick_direction_index];
        push_undo_now(item.id, std::nullopt,
                      get_locale_string("label.pick_point_by_mouse"));
        picked.custom_direction_end = mouse_world_pos;
        item.joint_wireframe_dirty = true;
        pick_direction_index = -1;
    }
    if (erase_picked_skeleton_index >= 0) {
        item.picked_skeleton_points.erase(item.picked_skeleton_points.begin() +
                                          erase_picked_skeleton_index);
        item.sort_picked_skeleton_points();
        item.joint_wireframe_dirty = true;
    }
    if (mouse_world_pos_picked) {
        push_undo_now(item.id, std::nullopt,
                      get_locale_string("label.pick_point_by_mouse"));
        pick_skeleton_point_from_mouse();
        item.joint_wireframe_dirty = true;
    }

    if (chain_edit_result.activated)
        begin_edit(item.id);
    if (chain_edit_result.deactivated_after_edit)
        end_edit(item.id, get_locale_string("label.joint"));
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
