#include "render_voxel_list.h"
#include <dear-imgui/imgui_internal.h>
#include <imgui/imgui.h>
#include <algorithm>
#include <queue>
#include <unordered_set>
namespace sinriv::ui::render {

std::vector<int> RenderVoxelList::get_process_flow(
    const std::vector<int>& inputs,
    const std::vector<int>& outputs) const {
    std::vector<int> empty_result;

    // 构建有向图：forward_edges[a] 表示依赖 a 的节点（a -> b 意味着 b 需要 a
    // 的数据）
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
        // 父子边：父节点 -> 子节点
        for (int child_id : item->children) {
            add_edge(id, child_id);
        }

        // SDF 分割依赖：目标节点 -> 当前节点
        if (item->segment_mode == RenderVoxelItem::SDF_NODE_SPLIT &&
            item->sdf_split_target_id >= 0) {
            add_edge(item->sdf_split_target_id, id);
        }

        // Source Node 依赖：源节点 -> 当前节点
        if (item->source_type == 1 && item->source_node_id >= 0) {
            add_edge(item->source_node_id, id);
        }
    }

    // 1. 从 outputs 反向 BFS，找到所有能到达 outputs 的节点
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
        if (it == reverse_edges.end())
            continue;
        for (int prev : it->second) {
            if (can_reach_outputs.insert(prev).second) {
                q.push(prev);
            }
        }
    }

    // 2. 从 inputs 正向 BFS，找到所有从入口可达的节点
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
        if (it == forward_edges.end())
            continue;
        for (int next : it->second) {
            if (reachable_from_inputs.insert(next).second) {
                q.push(next);
            }
        }
    }

    // 3. 剪枝：只保留既能从 inputs 到达、又能到达 outputs 的节点
    std::unordered_set<int> workflow_nodes;
    for (int id : can_reach_outputs) {
        if (reachable_from_inputs.count(id)) {
            workflow_nodes.insert(id);
        }
    }

    // 验证所有输出节点都被覆盖
    for (int output_id : outputs) {
        if (items.find(output_id) != items.end() &&
            !workflow_nodes.count(output_id)) {
            std::cerr << "[get_process_flow] output node " << output_id
                      << " is not reachable from inputs" << std::endl;
            return empty_result;
        }
    }

    if (workflow_nodes.empty()) {
        return empty_result;
    }

    // 4. 对剪枝后的工作流节点进行拓扑排序（Kahn 算法）
    std::unordered_map<int, int> in_degree;
    for (int id : workflow_nodes) {
        in_degree[id] = 0;
    }
    for (int id : workflow_nodes) {
        auto it = forward_edges.find(id);
        if (it == forward_edges.end())
            continue;
        for (int next : it->second) {
            if (workflow_nodes.count(next)) {
                ++in_degree[next];
            }
        }
    }

    std::queue<int> zero_in_degree;
    for (const auto& [id, degree] : in_degree) {
        if (degree == 0) {
            zero_in_degree.push(id);
        }
    }

    std::vector<int> result;
    result.reserve(workflow_nodes.size());
    while (!zero_in_degree.empty()) {
        int cur = zero_in_degree.front();
        zero_in_degree.pop();
        result.push_back(cur);

        auto it = forward_edges.find(cur);
        if (it == forward_edges.end())
            continue;
        for (int next : it->second) {
            if (!workflow_nodes.count(next))
                continue;
            auto deg_it = in_degree.find(next);
            if (deg_it == in_degree.end())
                continue;
            if (--deg_it->second == 0) {
                zero_in_degree.push(next);
            }
        }
    }

    if (result.size() != workflow_nodes.size()) {
        std::cerr << "[get_process_flow] cycle detected in workflow graph"
                  << std::endl;
        return empty_result;
    }

    return result;
}
void RenderVoxelList::render_flow_viewer() {
    if (!show_flow_viewer)
        return;
    ImGui::SetNextWindowSize(ImVec2(800, 600), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(get_locale_cstr("window.flow_viewer"),
                     &show_flow_viewer)) {
        std::lock_guard<std::mutex> lock(locker);

        auto node_label = [&](int node_id) -> std::string {
            auto it = items.find(node_id);
            if (it == items.end())
                return std::to_string(node_id) + " (?)";
            const auto& item = *it->second;
            if (!item.title.empty())
                return std::to_string(node_id) + ": " + item.title;
            return std::to_string(node_id);
        };

        auto segment_mode_label =
            [&](RenderVoxelItem::SegmentMode mode) -> const char* {
                switch (mode) {
                    case RenderVoxelItem::COLLISION:
                        return get_locale_cstr("mode.collision");
                    case RenderVoxelItem::PLANE:
                        return get_locale_cstr("mode.plane");
                    case RenderVoxelItem::CONCAVE_CONE:
                        return get_locale_cstr("mode.concave_cone");
                    case RenderVoxelItem::SPLIT_DISCONNECTED:
                        return get_locale_cstr("mode.split_disconnected");
                    case RenderVoxelItem::NEIGHBOR:
                        return get_locale_cstr("mode.neighbor");
                    case RenderVoxelItem::FILL_INTERIOR:
                        return get_locale_cstr("mode.fill_interior");
                    case RenderVoxelItem::CHAIN:
                        return get_locale_cstr("mode.chain");
                    case RenderVoxelItem::SDF_NODE_SPLIT:
                        return get_locale_cstr("mode.sdf_node_split");
                    default:
                        return "?";
                }
            };

        // 收集所有可用节点 id（用于下拉选择）
        std::vector<int> all_node_ids;
        all_node_ids.reserve(items.size());
        for (const auto& [id, item] : items) {
            all_node_ids.push_back(id);
        }
        std::sort(all_node_ids.begin(), all_node_ids.end());

        static int selected_input_node = -1;
        static int selected_output_node = -1;

        ImVec2 avail = ImGui::GetContentRegionAvail();
        float left_width = 280.0f;

        // ========== 左侧面板 ==========
        ImGui::BeginChild("FlowLeftPanel", ImVec2(left_width, avail.y), true);

        float half_h = ImGui::GetContentRegionAvail().y * 0.5f;

        // ----- 输入节点 -----
        ImGui::BeginChild("FlowInputsPanel", ImVec2(0, half_h), true);
        ImGui::TextUnformatted(get_locale_cstr("label.flow_inputs"));
        ImGui::Separator();

        std::string input_preview =
            selected_input_node >= 0
                ? node_label(selected_input_node)
                : get_locale_cstr("label.flow_select_node");
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 60);
        if (ImGui::BeginCombo("##flow_input_select", input_preview.c_str())) {
            for (int node_id : all_node_ids) {
                bool is_selected = (selected_input_node == node_id);
                if (ImGui::Selectable(node_label(node_id).c_str(),
                                      is_selected)) {
                    selected_input_node = node_id;
                }
                if (is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.add"))) {
            if (selected_input_node >= 0 &&
                std::find(flow_inputs.begin(), flow_inputs.end(),
                          selected_input_node) == flow_inputs.end()) {
                flow_inputs.push_back(selected_input_node);
                flow_needs_recompute = true;
            }
        }

        for (size_t i = 0; i < flow_inputs.size(); ++i) {
            int node_id = flow_inputs[i];
            ImGui::PushID(static_cast<int>(i));
            ImGui::TextUnformatted(node_label(node_id).c_str());
            ImGui::SameLine();
            if (ImGui::Button(get_locale_cstr("action.remove"))) {
                flow_inputs.erase(flow_inputs.begin() + i);
                flow_needs_recompute = true;
                --i;
            }
            ImGui::PopID();
        }

        if (flow_inputs.empty()) {
            ImGui::TextDisabled(get_locale_cstr("label.flow_add_input_hint"));
        }
        ImGui::EndChild();

        // ----- 输出节点 -----
        ImGui::BeginChild("FlowOutputsPanel", ImVec2(0, 0), true);
        ImGui::TextUnformatted(get_locale_cstr("label.flow_outputs"));
        ImGui::Separator();

        std::string output_preview =
            selected_output_node >= 0
                ? node_label(selected_output_node)
                : get_locale_cstr("label.flow_select_node");
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 60);
        if (ImGui::BeginCombo("##flow_output_select",
                              output_preview.c_str())) {
            for (int node_id : all_node_ids) {
                bool is_selected = (selected_output_node == node_id);
                if (ImGui::Selectable(node_label(node_id).c_str(),
                                      is_selected)) {
                    selected_output_node = node_id;
                }
                if (is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.add"))) {
            if (selected_output_node >= 0 &&
                std::find(flow_outputs.begin(), flow_outputs.end(),
                          selected_output_node) == flow_outputs.end()) {
                flow_outputs.push_back(selected_output_node);
                flow_needs_recompute = true;
            }
        }

        for (size_t i = 0; i < flow_outputs.size(); ++i) {
            int node_id = flow_outputs[i];
            ImGui::PushID(static_cast<int>(i) + 1000);
            ImGui::TextUnformatted(node_label(node_id).c_str());
            ImGui::SameLine();
            if (ImGui::Button(get_locale_cstr("action.remove"))) {
                flow_outputs.erase(flow_outputs.begin() + i);
                flow_needs_recompute = true;
                --i;
            }
            ImGui::PopID();
        }

        if (flow_outputs.empty()) {
            ImGui::TextDisabled(get_locale_cstr("label.flow_add_output_hint"));
        }
        ImGui::EndChild();

        ImGui::EndChild();  // FlowLeftPanel

        ImGui::SameLine();

        // ========== 右侧面板 ==========
        ImGui::BeginChild("FlowRightPanel", ImVec2(0, avail.y), true);
        ImGui::TextUnformatted(get_locale_cstr("label.flow_process_order"));
        ImGui::Separator();

        if (flow_needs_recompute) {
            flow_cached_order = get_process_flow(flow_inputs, flow_outputs);
            flow_needs_recompute = false;
        }

        if (flow_cached_order.empty()) {
            if (!flow_inputs.empty() && !flow_outputs.empty()) {
                ImGui::TextDisabled(get_locale_cstr("label.flow_no_result"));
            } else if (flow_inputs.empty()) {
                ImGui::TextDisabled(
                    get_locale_cstr("label.flow_add_input_hint"));
            } else {
                ImGui::TextDisabled(
                    get_locale_cstr("label.flow_add_output_hint"));
            }
        } else {
            if (ImGui::BeginTable(
                    "FlowProcessTable", 3,
                    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                        ImGuiTableFlags_SizingStretchProp)) {
                ImGui::TableSetupColumn(
                    get_locale_cstr("label.flow_node_id"),
                    ImGuiTableColumnFlags_WidthFixed, 60.0f);
                ImGui::TableSetupColumn(
                    get_locale_cstr("label.flow_node_mode"),
                    ImGuiTableColumnFlags_WidthFixed, 120.0f);
                ImGui::TableSetupColumn(
                    get_locale_cstr("label.flow_node_title"));
                ImGui::TableHeadersRow();

                for (size_t i = 0; i < flow_cached_order.size(); ++i) {
                    int node_id = flow_cached_order[i];
                    auto it = items.find(node_id);
                    if (it == items.end())
                        continue;
                    const auto& item = *it->second;

                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%d", node_id);

                    ImGui::TableSetColumnIndex(1);
                    const char* relation_label = nullptr;
                    const bool is_input =
                        std::find(flow_inputs.begin(), flow_inputs.end(),
                                  node_id) != flow_inputs.end();
                    if (is_input) {
                        relation_label =
                            get_locale_cstr("label.flow_relation_load");
                    } else if (item.source_type == 1 &&
                               item.source_node_id >= 0) {
                        relation_label =
                            get_locale_cstr("label.flow_relation_copy");
                    } else if (i > 0) {
                        int prev_id = flow_cached_order[i - 1];
                        auto prev_it = items.find(prev_id);
                        if (prev_it != items.end()) {
                            relation_label = segment_mode_label(
                                prev_it->second->segment_mode);
                        }
                    }
                    if (!relation_label)
                        relation_label = "?";
                    ImGui::TextUnformatted(relation_label);

                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextUnformatted(item.title.c_str());
                }
                ImGui::EndTable();
            }
        }
        ImGui::EndChild();
    }
    ImGui::End();
}

}  // namespace sinriv::ui::render