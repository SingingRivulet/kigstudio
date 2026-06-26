#include "render_voxel_list.h"
#include <dear-imgui/imgui_internal.h>
#include <imgui/imgui.h>
#include <algorithm>
#include <cstring>
#include <queue>
#include <unordered_set>
namespace sinriv::ui::render {

std::vector<int> RenderVoxelList::get_process_flow(
    const std::vector<int>& inputs,
    const std::vector<int>& outputs) const {
    std::vector<int> empty_result;

    std::unordered_map<int, std::vector<int>> forward_edges;
    std::unordered_map<int, std::vector<int>> reverse_edges;

    auto add_edge = [&](int from, int to) {
        if (from < 0 || to < 0)
            return;
        if (items.find(from) == items.end() || items.find(to) == items.end())
            return;
        forward_edges[from].push_back(to);
        reverse_edges[to].push_back(from);
    };

    for (const auto& [id, item] : items) {
        for (int child_id : item->children) {
            add_edge(id, child_id);
        }
        if (item->segment_mode == RenderVoxelItem::SDF_NODE_SPLIT &&
            item->sdf_split_target_id >= 0) {
            add_edge(item->sdf_split_target_id, id);
        }
        if (item->source_type == 1 && item->source_node_id >= 0) {
            add_edge(item->source_node_id, id);
        }
    }

    std::unordered_set<int> can_reach_outputs;
    std::queue<int> q;
    for (int output_id : outputs) {
        auto it = items.find(output_id);
        if (it != items.end() && can_reach_outputs.insert(output_id).second) {
            q.push(output_id);
        }
    }
    while (!q.empty()) {
        int cur = q.front();
        q.pop();
        auto it = reverse_edges.find(cur);
        if (it == reverse_edges.end()) continue;
        for (int prev : it->second) {
            if (can_reach_outputs.insert(prev).second) {
                q.push(prev);
            }
        }
    }

    std::unordered_set<int> reachable_from_inputs;
    for (int input_id : inputs) {
        auto it = items.find(input_id);
        if (it != items.end() &&
            reachable_from_inputs.insert(input_id).second) {
            q.push(input_id);
        }
    }
    while (!q.empty()) {
        int cur = q.front();
        q.pop();
        auto it = forward_edges.find(cur);
        if (it == forward_edges.end()) continue;
        for (int next : it->second) {
            if (reachable_from_inputs.insert(next).second) {
                q.push(next);
            }
        }
    }

    std::unordered_set<int> workflow_nodes;
    for (int id : can_reach_outputs) {
        if (reachable_from_inputs.count(id)) {
            workflow_nodes.insert(id);
        }
    }

    for (int output_id : outputs) {
        if (items.find(output_id) != items.end() &&
            !workflow_nodes.count(output_id)) {
            std::cerr << "[get_process_flow] output node " << output_id
                      << " is not reachable from inputs" << std::endl;
            return empty_result;
        }
    }

    if (workflow_nodes.empty()) return empty_result;

    std::unordered_map<int, int> in_degree;
    for (int id : workflow_nodes) in_degree[id] = 0;
    for (int id : workflow_nodes) {
        auto it = forward_edges.find(id);
        if (it == forward_edges.end()) continue;
        for (int next : it->second) {
            if (workflow_nodes.count(next)) ++in_degree[next];
        }
    }

    std::queue<int> zero_in_degree;
    for (const auto& [id, degree] : in_degree) {
        if (degree == 0) zero_in_degree.push(id);
    }

    std::vector<int> result;
    result.reserve(workflow_nodes.size());
    while (!zero_in_degree.empty()) {
        int cur = zero_in_degree.front();
        zero_in_degree.pop();
        result.push_back(cur);
        auto it = forward_edges.find(cur);
        if (it == forward_edges.end()) continue;
        for (int next : it->second) {
            if (!workflow_nodes.count(next)) continue;
            auto deg_it = in_degree.find(next);
            if (deg_it == in_degree.end()) continue;
            if (--deg_it->second == 0) zero_in_degree.push(next);
        }
    }

    if (result.size() != workflow_nodes.size()) {
        std::cerr << "[get_process_flow] cycle detected in workflow graph"
                  << std::endl;
        return empty_result;
    }
    return result;
}

void RenderVoxelList::execute_flow() {
    if (flow_inputs.empty() || flow_outputs.empty()) return;

    std::lock_guard<std::mutex> lock(queue_mutex);
    QueueTask task;
    task.type = TASK_EXECUTE_FLOW;
    task.flow_input_entries = flow_inputs;
    task.flow_output_entries = flow_outputs;
    queue.push(task);
    this->queue_num = static_cast<int>(queue.size());
}

void RenderVoxelList::render_flow_viewer() {
    if (!show_flow_viewer) return;
    ImGui::SetNextWindowSize(ImVec2(800, 600), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(get_locale_cstr("window.flow_viewer"),
                     &show_flow_viewer)) {
        std::lock_guard<std::mutex> lock(locker);

        std::vector<int> all_node_ids;
        all_node_ids.reserve(items.size());
        for (const auto& [id, item] : items) all_node_ids.push_back(id);
        std::sort(all_node_ids.begin(), all_node_ids.end());

        auto node_label = [&](int node_id) -> std::string {
            auto it = items.find(node_id);
            if (it == items.end())
                return std::to_string(node_id) + " (?)";
            const auto& item = *it->second;
            if (!item.title.empty())
                return std::to_string(node_id) + ": " + item.title;
            return std::to_string(node_id);
        };

        static int selected_input_node = -1;
        static int selected_output_node = -1;
        static char input_file_path_buf[512] = {};
        static char output_file_path_buf[512] = {};

        ImVec2 avail = ImGui::GetContentRegionAvail();
        float left_width = 300.0f;

        // ========== 左侧面板 ==========
        ImGui::BeginChild("FlowLeftPanel", ImVec2(left_width, avail.y), true);
        float half_h = ImGui::GetContentRegionAvail().y * 0.5f;

        // ----- 输入 -----
        ImGui::BeginChild("FlowInputsPanel", ImVec2(0, half_h), true);
        ImGui::TextUnformatted(get_locale_cstr("label.flow_inputs"));
        ImGui::Separator();

        // 节点选择
        std::string input_node_preview =
            selected_input_node >= 0
                ? node_label(selected_input_node)
                : get_locale_cstr("label.flow_select_node");
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 4);
        if (ImGui::BeginCombo("##flow_input_node",
                              input_node_preview.c_str())) {
            for (int node_id : all_node_ids) {
                bool is_sel = (selected_input_node == node_id);
                if (ImGui::Selectable(node_label(node_id).c_str(), is_sel))
                    selected_input_node = node_id;
                if (is_sel) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        // 文件路径
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 60);
        ImGui::InputTextWithHint("##flow_input_file",
                                 get_locale_cstr("label.flow_file_path"),
                                 input_file_path_buf,
                                 sizeof(input_file_path_buf));
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.browse"))) {
            const char* filters[] = {"*.stl", "*.obj"};
            const char* file = tinyfd_openFileDialog(
                utf8_to_ansi(get_locale_cstr("dialog.open_stl_title")).c_str(),
                "", 2, filters,
                utf8_to_ansi(get_locale_cstr("dialog.stl_file")).c_str(), 0);
            if (file) {
                std::string path = tinyfd_path_to_utf8(file);
                strncpy(input_file_path_buf, path.c_str(),
                        sizeof(input_file_path_buf) - 1);
                input_file_path_buf[sizeof(input_file_path_buf) - 1] = '\0';
            }
        }

        // ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.add"))) {
            std::string path(input_file_path_buf);
            if (selected_input_node >= 0 && !path.empty()) {
                bool dup = false;
                for (const auto& e : flow_inputs) {
                    if (e.node_id == selected_input_node &&
                        e.file_path == path) { dup = true; break; }
                }
                if (!dup) {
                    flow_inputs.push_back(
                        {selected_input_node, path});
                    flow_needs_recompute = true;
                }
            }
        }

        for (size_t i = 0; i < flow_inputs.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            ImGui::Text("%s  |  %s",
                        node_label(flow_inputs[i].node_id).c_str(),
                        flow_inputs[i].file_path.c_str());
            ImGui::SameLine();
            if (ImGui::Button(get_locale_cstr("action.remove"))) {
                flow_inputs.erase(flow_inputs.begin() + i);
                flow_needs_recompute = true;
                --i;
            }
            ImGui::PopID();
        }

        if (flow_inputs.empty())
            ImGui::TextDisabled(get_locale_cstr("label.flow_add_input_hint"));
        ImGui::EndChild();

        // ----- 输出 -----
        ImGui::BeginChild("FlowOutputsPanel", ImVec2(0, 0), true);
        ImGui::TextUnformatted(get_locale_cstr("label.flow_outputs"));
        ImGui::Separator();

        std::string output_node_preview =
            selected_output_node >= 0
                ? node_label(selected_output_node)
                : get_locale_cstr("label.flow_select_node");
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 4);
        if (ImGui::BeginCombo("##flow_output_node",
                              output_node_preview.c_str())) {
            for (int node_id : all_node_ids) {
                bool is_sel = (selected_output_node == node_id);
                if (ImGui::Selectable(node_label(node_id).c_str(), is_sel))
                    selected_output_node = node_id;
                if (is_sel) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 60);
        ImGui::InputTextWithHint("##flow_output_file",
                                 get_locale_cstr("label.flow_file_path"),
                                 output_file_path_buf,
                                 sizeof(output_file_path_buf));
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.browse"))) {
            const char* filters[] = {"*.stl"};
            const char* file = tinyfd_saveFileDialog(
                utf8_to_ansi(get_locale_cstr("dialog.open_stl_title")).c_str(),
                "", 1, filters,
                utf8_to_ansi(get_locale_cstr("dialog.stl_file")).c_str());
            if (file) {
                std::string path = tinyfd_path_to_utf8(file);
                if (path.size() < 4 ||
                    path.compare(path.size() - 4, 4, ".stl") != 0)
                    path += ".stl";
                strncpy(output_file_path_buf, path.c_str(),
                        sizeof(output_file_path_buf) - 1);
                output_file_path_buf[sizeof(output_file_path_buf) - 1] = '\0';
            }
        }

        // ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.add"))) {
            std::string path(output_file_path_buf);
            if (selected_output_node >= 0 && !path.empty()) {
                bool dup = false;
                for (const auto& e : flow_outputs) {
                    if (e.node_id == selected_output_node &&
                        e.file_path == path) { dup = true; break; }
                }
                if (!dup) {
                    flow_outputs.push_back(
                        {selected_output_node, path});
                    flow_needs_recompute = true;
                }
            }
        }

        for (size_t i = 0; i < flow_outputs.size(); ++i) {
            ImGui::PushID(static_cast<int>(i) + 1000);
            ImGui::Text("%s  |  %s",
                        node_label(flow_outputs[i].node_id).c_str(),
                        flow_outputs[i].file_path.c_str());
            ImGui::SameLine();
            if (ImGui::Button(get_locale_cstr("action.remove"))) {
                flow_outputs.erase(flow_outputs.begin() + i);
                flow_needs_recompute = true;
                --i;
            }
            ImGui::PopID();
        }

        if (flow_outputs.empty())
            ImGui::TextDisabled(get_locale_cstr("label.flow_add_output_hint"));
        ImGui::EndChild();

        ImGui::EndChild();  // FlowLeftPanel

        ImGui::SameLine();

        // ========== 右侧面板 ==========
        ImGui::BeginChild("FlowRightPanel", ImVec2(0, avail.y), true);
        ImGui::TextUnformatted(get_locale_cstr("label.flow_process_order"));
        ImGui::Separator();

        // 计算执行顺序
        if (flow_needs_recompute) {
            std::vector<int> in_ids, out_ids;
            for (const auto& e : flow_inputs)
                if (e.node_id >= 0) in_ids.push_back(e.node_id);
            for (const auto& e : flow_outputs)
                if (e.node_id >= 0) out_ids.push_back(e.node_id);
            flow_cached_order = get_process_flow(in_ids, out_ids);
            flow_needs_recompute = false;
        }

        // 判断执行按钮状态
        bool can_execute = true;
        if (flow_inputs.empty() || flow_outputs.empty()) {
            can_execute = false;
        } else {
            for (const auto& e : flow_inputs) {
                if (e.node_id < 0 || e.file_path.empty()) {
                    can_execute = false; break;
                }
            }
            for (const auto& e : flow_outputs) {
                if (e.node_id < 0 || e.file_path.empty()) {
                    can_execute = false; break;
                }
            }
            if (isQueueRunning()) can_execute = false;
        }

        if (flow_cached_order.empty()) {
            if (flow_inputs.empty())
                ImGui::TextDisabled(get_locale_cstr("label.flow_add_input_hint"));
            else if (flow_outputs.empty())
                ImGui::TextDisabled(get_locale_cstr("label.flow_add_output_hint"));
            else
                ImGui::TextDisabled(get_locale_cstr("label.flow_no_result"));
        } else {
            if (ImGui::BeginTable("FlowProcessTable", 3,
                    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                        ImGuiTableFlags_SizingStretchProp)) {
                ImGui::TableSetupColumn(get_locale_cstr("label.flow_node_id"),
                    ImGuiTableColumnFlags_WidthFixed, 60.0f);
                ImGui::TableSetupColumn(get_locale_cstr("label.flow_node_mode"),
                    ImGuiTableColumnFlags_WidthFixed, 120.0f);
                ImGui::TableSetupColumn(get_locale_cstr("label.flow_node_title"));
                ImGui::TableHeadersRow();

                for (size_t i = 0; i < flow_cached_order.size(); ++i) {
                    int node_id = flow_cached_order[i];
                    auto it = items.find(node_id);
                    if (it == items.end()) continue;
                    const auto& item = *it->second;

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%d", node_id);

                    ImGui::TableSetColumnIndex(1);
                    const bool is_input = std::any_of(
                        flow_inputs.begin(), flow_inputs.end(),
                        [node_id](const FlowEntry& e) {
                            return e.node_id == node_id;
                        });
                    const char* rel = nullptr;
                    if (is_input) {
                        rel = get_locale_cstr("label.flow_relation_load");
                    } else if (item.source_type == 1 &&
                               item.source_node_id >= 0) {
                        rel = get_locale_cstr("label.flow_relation_copy");
                    } else if (i > 0) {
                        int prev_id = flow_cached_order[i - 1];
                        auto prev_it = items.find(prev_id);
                        if (prev_it != items.end()) {
                            switch (prev_it->second->segment_mode) {
                                case RenderVoxelItem::COLLISION:
                                    rel = get_locale_cstr("mode.collision"); break;
                                case RenderVoxelItem::PLANE:
                                    rel = get_locale_cstr("mode.plane"); break;
                                case RenderVoxelItem::CONCAVE_CONE:
                                    rel = get_locale_cstr("mode.concave_cone"); break;
                                case RenderVoxelItem::SPLIT_DISCONNECTED:
                                    rel = get_locale_cstr("mode.split_disconnected"); break;
                                case RenderVoxelItem::NEIGHBOR:
                                    rel = get_locale_cstr("mode.neighbor"); break;
                                case RenderVoxelItem::FILL_INTERIOR:
                                    rel = get_locale_cstr("mode.fill_interior"); break;
                                case RenderVoxelItem::CHAIN:
                                    rel = get_locale_cstr("mode.chain"); break;
                                case RenderVoxelItem::SDF_NODE_SPLIT:
                                    rel = get_locale_cstr("mode.sdf_node_split"); break;
                                default: rel = "?"; break;
                            }
                        }
                    }
                    if (!rel) rel = "?";
                    ImGui::TextUnformatted(rel);

                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextUnformatted(item.title.c_str());
                }
                ImGui::EndTable();
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // ===== 执行按钮 =====
        if (!can_execute) ImGui::BeginDisabled();
        if (ImGui::Button(get_locale_cstr("action.flow_execute"),
                          ImVec2(ImGui::GetContentRegionAvail().x, 40))) {
            execute_flow();
        }
        if (!can_execute) ImGui::EndDisabled();

        if (isQueueRunning()) {
            ImGui::Spacing();
            ImGui::TextUnformatted(getQueueStatus().c_str());
            float prog = getQueueProgress();
            if (prog > 0) ImGui::ProgressBar(prog);
        }

        ImGui::EndChild();  // FlowRightPanel
    }
    ImGui::End();
}

}  // namespace sinriv::ui::render
