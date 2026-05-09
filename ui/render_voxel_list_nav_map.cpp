#include "render_voxel_list.h"

#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>

#include <unordered_set>

#include "locale.h"

namespace sinriv::ui::render {

void RenderVoxelList::render_nav_map() {
    std::lock_guard<std::mutex> lock(locker);
    ImGui::SetNextWindowPos(ImVec2(0.f, (float)menu_height), ImGuiCond_Always,
                            ImVec2(0.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(300, 600), ImGuiCond_Once);
    if (!ImGui::Begin(get_locale_cstr("window.nav_node_map"))) {
        ImGui::End();
        return;
    }

    ImNodes::BeginNodeEditor();

    int link_id = 0;
    for (auto& [id, item] : this->items) {
        // 设置节点固定坐标
        ImNodes::SetNodeGridSpacePos(
            id, ImVec2((float)item->nav_node_position[1] * 1.5f,
                       (float)item->nav_node_position[0] * 1.5f));
        // 禁止拖动
        ImNodes::SetNodeDraggable(id, false);

        // 当前选中节点高亮
        bool is_current = (id == this->render_id);
        if (is_current) {
            ImNodes::PushColorStyle(ImNodesCol_TitleBar,
                                    IM_COL32(56, 120, 56, 255));
            ImNodes::PushColorStyle(ImNodesCol_TitleBarHovered,
                                    IM_COL32(66, 150, 66, 255));
            ImNodes::PushColorStyle(ImNodesCol_TitleBarSelected,
                                    IM_COL32(76, 170, 76, 255));
        }

        ImNodes::BeginNode(id);

        ImNodes::BeginNodeTitleBar();
        if (item->write_count > 0) {
            ImGui::Text(get_locale_cstr("label.node_updating"), id);
        } else {
            ImGui::Text(get_locale_cstr("label.node"), id);
        }
        ImNodes::EndNodeTitleBar();

        // Input attribute (来自父节点)
        ImNodes::BeginInputAttribute(id * 10 + 1, ImNodesPinShape_CircleFilled);
        ImGui::Text("");
        ImNodes::EndInputAttribute();

        // ImGui::Text(
        //     "%s",
        //     item->segment_mode == RenderVoxelItem::COLLISION ? "Collision"
        //                                                      : "Plane");

        // 缩略图
        if (bgfx::isValid(item->thumbnail_tex)) {
            ImGui::Image(item->thumbnail_tex, ImVec2(64.0f, 64.0f));
        } else {
            ImGui::Dummy(ImVec2(64.0f, 64.0f));
        }

        // Output attributes (连向子节点)
        for (size_t i = 0; i < item->children.size(); ++i) {
            ImNodes::BeginOutputAttribute(
                static_cast<int>(id * 10 + 2 + i), ImNodesPinShape_CircleFilled);
            ImGui::Text("");
            ImNodes::EndOutputAttribute();
        }

        ImNodes::EndNode();

        if (is_current) {
            ImNodes::PopColorStyle();
            ImNodes::PopColorStyle();
            ImNodes::PopColorStyle();
        }
    }

    // 绘制连线：父节点的 output 连向子节点的 input
    for (auto& [id, item] : this->items) {
        for (size_t i = 0; i < item->children.size(); ++i) {
            int child_id = item->children[i];
            if (this->items.find(child_id) != this->items.end()) {
                int parent_attr_id = static_cast<int>(id * 10 + 2 + i);
                int child_attr_id = child_id * 10 + 1;
                ImNodes::Link(link_id++, parent_attr_id, child_attr_id);
            }
        }
    }

    ImNodes::MiniMap(0.2f, ImNodesMiniMapLocation_BottomLeft);
    ImNodes::EndNodeEditor();

    // 点击节点切换 render_id
    int num_selected = ImNodes::NumSelectedNodes();
    if (num_selected > 0) {
        static std::vector<int> selected_nodes;
        selected_nodes.resize(num_selected);
        ImNodes::GetSelectedNodes(selected_nodes.data());
        if (!selected_nodes.empty()) {
            this->setRenderId_unsafe(selected_nodes[0]);
        }
        ImNodes::ClearNodeSelection();
    }

    ImGui::End();
}

struct LayoutContext {
    float next_x = 0.0f;
    float x_spacing = 120.0f;
    float y_spacing = 100.0f;
};

inline float layout_node(RenderVoxelList& mgr,
                         int node_id,
                         int depth,
                         LayoutContext& ctx,
                         std::unordered_map<int, int>& visit_state,
                         bool& has_cycle,
                         int root_id) {
    auto it = mgr.items.find(node_id);

    // 不存在 = 无效节点
    if (it == mgr.items.end()) {
        return -1.0f;
    }

    // 环检测
    if (visit_state[node_id] == 1) {
        std::cerr << "Cycle detected at node " << node_id << std::endl;
        has_cycle = true;
        return -1.0f;
    }

    // 已经处理过
    if (visit_state[node_id] == 2) {
        return (float)it->second->nav_node_position[0];
    }

    visit_state[node_id] = 1;

    auto& node = *it->second;
    node.root_id = root_id;

    std::vector<float> child_xs;
    for (int child_id : node.children) {
        if (child_id < 0) continue;
        float cx = layout_node(mgr, child_id, depth + 1, ctx, visit_state,
                               has_cycle, root_id);
        if (cx >= 0) child_xs.push_back(cx);
    }

    float my_x;
    if (child_xs.empty()) {
        my_x = (float)ctx.next_x;
        ctx.next_x += ctx.x_spacing;
    } else {
        float sum = 0.0f;
        for (float cx : child_xs) sum += cx;
        my_x = sum / static_cast<float>(child_xs.size());
    }

    node.nav_node_position[0] = (int)my_x;
    node.nav_node_position[1] =
        (int)(static_cast<float>(depth) * ctx.y_spacing);

    visit_state[node_id] = 2;

    return my_x;
}

inline void compute_layout(RenderVoxelList& mgr) {
    auto roots = mgr.find_roots();
    std::cout << "compute_layout roots: " << roots.size() << std::endl;

    LayoutContext ctx;
    std::unordered_map<int, int> visit_state;
    bool has_cycle = false;

    for (int root_id : roots) {
        layout_node(mgr, root_id, 0, ctx, visit_state, has_cycle, root_id);
        ctx.next_x += ctx.x_spacing * 2;
    }

    if (has_cycle) {
        std::cerr << "WARNING: Cycle detected in RenderVoxelItem graph!"
                  << std::endl;
    }
}

std::vector<int> RenderVoxelList::find_roots() {
    std::unordered_set<int> has_parent;

    for (auto& [id, item] : this->items) {
        for (int child : item->children) {
            if (this->items.find(child) != this->items.end()) {
                has_parent.insert(child);
            }
        }
    }

    std::vector<int> roots;

    for (auto& [id, item] : this->items) {
        if (!has_parent.count(id)) {
            roots.push_back(id);
        }
    }

    return roots;
}

void RenderVoxelList::update_nav_node_position() {
    if (update_nav_node_status) {
        std::cout << "update nav node position" << std::endl;
        compute_layout(*this);
    }
    update_nav_node_status = false;
}
}  // namespace sinriv::ui::render
