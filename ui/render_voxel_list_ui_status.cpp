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
    ImGui::SameLine();
    if (ImGui::RadioButton(get_locale_cstr("label.source_addon"), &source_type,
                           2)) {
        push_undo_now(item.id, std::nullopt, "Source Type");
        item.source_type = source_type;
    }

    // 附加件模式：显示专用UI
    if (item.source_type == 2) {
        ImGui::Separator();
        // 底模节点选择器
        std::vector<std::pair<int, std::string>> sdf_candidates;
        if (item.manager) {
            for (auto& [other_id, other] : item.manager->items) {
                if (other_id == item.id)
                    continue;
                // 只列出有SDF数据的节点
                if (!other->sdf_data)
                    continue;
                sdf_candidates.push_back(
                    {other_id, "Node " + std::to_string(other_id)});
            }
        }
        if (sdf_candidates.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.3f, 1.0f), "%s",
                               get_locale_cstr("label.addon_no_sdf_nodes"));
        } else {
            int current_base = -1;
            std::vector<const char*> sdf_names;
            for (size_t i = 0; i < sdf_candidates.size(); ++i) {
                sdf_names.push_back(sdf_candidates[i].second.c_str());
                if (sdf_candidates[i].first == item.addon_base_node_id) {
                    current_base = static_cast<int>(i);
                }
            }
            if (ImGui::Combo(get_locale_cstr("label.addon_base_model"),
                             &current_base, sdf_names.data(),
                             static_cast<int>(sdf_candidates.size()))) {
                push_undo_now(item.id, std::nullopt, "Addon Base Node");
                if (current_base >= 0 &&
                    current_base < static_cast<int>(sdf_candidates.size())) {
                    item.addon_base_node_id = sdf_candidates[current_base].first;
                }
            }

            if (item.addon_base_node_id >= 0) {
                ImGui::Text(get_locale_cstr("label.addon_base_applied"),
                            item.addon_base_node_id);
            }

            ImGui::Separator();
            if (ImGui::Button(get_locale_cstr("action.addon_apply_base"))) {
                if (item.addon_base_node_id >= 0 && item.manager) {
                    auto base_it = item.manager->items.find(
                        item.addon_base_node_id);
                    if (base_it != item.manager->items.end()) {
                        auto& base = *base_it->second;
                        // 从源节点加载网格到origin_mesh_renderer
                        if (!base.cached_mesh.empty()) {
                            item.origin_mesh_renderer.clear();
                            item.origin_mesh_renderer.loadGeometry(
                                base.cached_mesh);
                        } else if (!base.source_triangles.empty()) {
                            item.origin_mesh_renderer.clear();
                            std::vector<std::tuple<
                                sinriv::kigstudio::voxel::Triangle,
                                sinriv::kigstudio::voxel::vec3f>>
                                triangles;
                            triangles.reserve(base.source_triangles.size());
                            for (const auto& tri : base.source_triangles) {
                                triangles.push_back(
                                    {tri,
                                     sinriv::kigstudio::voxel::calcTriangleNormal(
                                         tri)});
                            }
                            item.origin_mesh_renderer.loadGeometry(triangles);
                        } else if (!base.mesh_renderer.empty()) {
                            // 如果源节点没有source_triangles，尝试复制mesh
                            // 由于RenderMesh不可拷贝，此处通过重新体素化网格来获取
                            // 暂时跳过，等待后续完善
                        }
                        // 设置粉色
                        item.origin_mesh_renderer.setBaseColor(1.0f, 0.4f, 0.6f,
                                                              1.0f);
                        item.showOriginMesh = true;
                    }
                }
            }
        }
        // 仅在附加件模式下显示底部信息区域
        ImGui::Separator();
        if (item.addon_base_node_id < 0) {
            ImGui::TextWrapped("%s",
                               get_locale_cstr("label.addon_no_base_selected"));
        }
        return;  // 附加件模式不显示后面的通用加载模式等UI
    }

    // 加载模式选择（File / Node 通用）
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
        static_cast<int>(sizeof(load_mode_values) / sizeof(load_mode_values[0]));
    int load_mode_idx = 0;
    for (int i = 0; i < load_mode_count; ++i) {
        if (load_mode_values[i] == item.stl_load_mode) {
            load_mode_idx = i;
            break;
        }
    }
    if (ImGui::Combo(get_locale_cstr("label.stl_load_mode"), &load_mode_idx,
                     load_mode_names, load_mode_count)) {
        push_undo_now(item.id, std::nullopt, "STL Load Mode");
        item.stl_load_mode = load_mode_values[load_mode_idx];
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
        switch (item.stl_load_mode) {
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

}  // namespace sinriv::ui::render
