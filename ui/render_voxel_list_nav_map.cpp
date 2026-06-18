#include "render_voxel_list.h"

#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>

#include <cmath>
#include <unordered_set>

#include "kigstudio/utils/locale.h"

namespace sinriv::ui::render {

inline void compute_layout(RenderVoxelList& mgr);

void RenderVoxelList::render_nav_map() {
    std::lock_guard<std::mutex> lock(locker);
    ImGui::SetNextWindowPos(ImVec2(0.f, (float)menu_height), ImGuiCond_Always,
                            ImVec2(0.0f, 0.0f));
    float nav_map_height =
        (float)window_height - (float)menu_height - item_status_height;
    ImGui::SetNextWindowSize(ImVec2(300, nav_map_height), ImGuiCond_Once);
    if (!ImGui::Begin(get_locale_cstr("window.nav_node_map"))) {
        ImGui::End();
        return;
    }

    struct LayoutEdge {
        int from;
        int to;
    };
    std::vector<LayoutEdge> layout_edges;

    std::unordered_set<int> sdf_sources;
    std::unordered_set<int> node_sources;
    for (auto& [id, item] : this->items) {
        if (item->segment_mode == RenderVoxelItem::SDF_NODE_SPLIT &&
            item->sdf_split_target_id >= 0) {
            sdf_sources.insert(item->sdf_split_target_id);
        }
        if (item->source_type == 1 && item->source_node_id >= 0) {
            node_sources.insert(item->source_node_id);
        }
    }

    // 初始化力导向位置：只给新节点（还没有位置）分配树形布局位置，
    // 已有节点保持当前位置，避免新增/删除节点时整个图被重置。
    if (nav_layout_force_directed && !nav_layout_initialized) {
        compute_layout(*this);
        for (auto& [id, item] : this->items) {
            if (item->nav_layout_pinned || item->nav_layout_pos_set)
                continue;
            // nav_layout_pos 与 nav_node_position 同轴：
            // [0]=水平位置(my_x), [1]=深度(depth)
            item->nav_layout_pos[0] = (float)item->nav_node_position[0];
            item->nav_layout_pos[1] = (float)item->nav_node_position[1];
            item->nav_layout_vel[0] = 0.0f;
            item->nav_layout_vel[1] = 0.0f;
            item->nav_layout_pos_set = true;
        }
        nav_layout_initialized = true;
    }

    // 收集所有边（父子 + SDF 依赖 + Source Node 依赖）
    if (nav_layout_force_directed) {
        for (auto& [id, item] : this->items) {
            for (int child_id : item->children) {
                if (this->items.find(child_id) != this->items.end()) {
                    layout_edges.push_back({id, child_id});
                }
            }
        }
        for (auto& [id, item] : this->items) {
            if (item->segment_mode == RenderVoxelItem::SDF_NODE_SPLIT &&
                item->sdf_split_target_id >= 0 &&
                this->items.find(item->sdf_split_target_id) !=
                    this->items.end()) {
                layout_edges.push_back({item->sdf_split_target_id, id});
            }
        }
        for (auto& [id, item] : this->items) {
            if (item->source_type == 1 && item->source_node_id >= 0 &&
                this->items.find(item->source_node_id) != this->items.end()) {
                layout_edges.push_back({item->source_node_id, id});
            }
        }
    }

    // 力导向迭代一步
    if (nav_layout_force_directed && nav_layout_initialized &&
        this->items.size() > 1) {
        std::unordered_map<int, ImVec2> forces;
        for (auto& [id, item] : this->items) {
            forces[id] = ImVec2(0.0f, 0.0f);
        }

        // 节点间斥力
        for (auto it1 = this->items.begin(); it1 != this->items.end(); ++it1) {
            for (auto it2 = std::next(it1); it2 != this->items.end(); ++it2) {
                auto& a = it1->second;
                auto& b = it2->second;
                float dx = b->nav_layout_pos[0] - a->nav_layout_pos[0];
                float dy = b->nav_layout_pos[1] - a->nav_layout_pos[1];
                float dist_sq = dx * dx + dy * dy;
                if (dist_sq < 1.0f)
                    dist_sq = 1.0f;
                float dist = std::sqrt(dist_sq);
                bool same_root = (a->root_id == b->root_id) &&
                                 (a->root_id >= 0) && (b->root_id >= 0);
                float repulsion = same_root ? nav_layout_repulsion
                                            : nav_layout_repulsion_cross_root;
                float f = repulsion / dist_sq;
                float fx = f * dx / dist;
                float fy = f * dy / dist;
                forces[it1->first].x -= fx;
                forces[it1->first].y -= fy;
                forces[it2->first].x += fx;
                forces[it2->first].y += fy;
            }
        }

        // 边弹力
        for (auto& edge : layout_edges) {
            auto it_from = this->items.find(edge.from);
            auto it_to = this->items.find(edge.to);
            if (it_from == this->items.end() || it_to == this->items.end())
                continue;
            auto& a = it_from->second;
            auto& b = it_to->second;
            float dx = b->nav_layout_pos[0] - a->nav_layout_pos[0];
            float dy = b->nav_layout_pos[1] - a->nav_layout_pos[1];
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < 1.0f)
                dist = 1.0f;
            float displacement = dist - nav_layout_ideal_length;
            float f = nav_layout_spring * displacement;
            float fx = f * dx / dist;
            float fy = f * dy / dist;
            forces[edge.from].x += fx;
            forces[edge.from].y += fy;
            forces[edge.to].x -= fx;
            forces[edge.to].y -= fy;
        }

        // 父节点右侧虚拟点引力：让子节点倾向于位于父节点右边
        for (auto& edge : layout_edges) {
            auto it_from = this->items.find(edge.from);
            auto it_to = this->items.find(edge.to);
            if (it_from == this->items.end() || it_to == this->items.end())
                continue;
            auto& parent = it_from->second;
            auto& child = it_to->second;
            float virtual_pos0 = parent->nav_layout_pos[0];
            float virtual_pos1 =
                parent->nav_layout_pos[1] + nav_layout_right_offset;
            float d0 = virtual_pos0 - child->nav_layout_pos[0];
            float d1 = virtual_pos1 - child->nav_layout_pos[1];
            float dist = std::sqrt(d0 * d0 + d1 * d1);
            if (dist < 1.0f)
                dist = 1.0f;
            float f = nav_layout_right_pull * dist;
            float fx = f * d0 / dist;
            float fy = f * d1 / dist;
            forces[edge.to].x += fx;
            forces[edge.to].y += fy;
            // 补回父节点的反作用力，否则系统动量不守恒，会永远漂移
            forces[edge.from].x -= fx;
            forces[edge.from].y -= fy;
        }

        // 积分更新速度与位置（被拖动的节点固定）
        for (auto& [id, item] : this->items) {
            if (item->nav_layout_pinned)
                continue;
            auto& f = forces[id];
            // 微小向心力，把图拉向原点 (0,0)，让整体更紧凑
            forces[id].x += nav_layout_center_pull * (0.0f - item->nav_layout_pos[0]);
            forces[id].y += nav_layout_center_pull * (0.0f - item->nav_layout_pos[1]);
            item->nav_layout_vel[0] =
                (item->nav_layout_vel[0] + f.x * nav_layout_dt) *
                nav_layout_damping;
            item->nav_layout_vel[1] =
                (item->nav_layout_vel[1] + f.y * nav_layout_dt) *
                nav_layout_damping;
            float speed = std::sqrt(item->nav_layout_vel[0] * item->nav_layout_vel[0] +
                                    item->nav_layout_vel[1] * item->nav_layout_vel[1]);
            if (speed > nav_layout_max_speed) {
                item->nav_layout_vel[0] =
                    item->nav_layout_vel[0] / speed * nav_layout_max_speed;
                item->nav_layout_vel[1] =
                    item->nav_layout_vel[1] / speed * nav_layout_max_speed;
            } else if (speed < nav_layout_velocity_threshold) {
                item->nav_layout_vel[0] = 0.0f;
                item->nav_layout_vel[1] = 0.0f;
            }
            item->nav_layout_pos[0] += item->nav_layout_vel[0];
            item->nav_layout_pos[1] += item->nav_layout_vel[1];
        }
    }

    std::unordered_map<int, ImVec2> intended_positions;

    // 把节点编辑器限制在一个固定高度的子区域，底部留出控件空间
    float editor_height =
        std::max(ImGui::GetContentRegionAvail().y - 30.0f, 100.0f);
    ImGui::BeginChild("nav_map_editor", ImVec2(-1.0f, editor_height), false);

    ImNodes::BeginNodeEditor();

    int link_id = 0;
    for (auto& [id, item] : this->items) {
        // 计算目标位置
        ImVec2 target_pos;
        if (nav_layout_force_directed && nav_layout_initialized) {
            target_pos = ImVec2(item->nav_layout_pos[1] * 1.5f,
                                item->nav_layout_pos[0] * 1.5f);
            if (!item->nav_layout_pinned) {
                ImNodes::SetNodeGridSpacePos(id, target_pos);
            }
            ImNodes::SetNodeDraggable(id, true);
        } else {
            target_pos = ImVec2((float)item->nav_node_position[1] * 1.5f,
                                (float)item->nav_node_position[0] * 1.5f);
            ImNodes::SetNodeGridSpacePos(id, target_pos);
            ImNodes::SetNodeDraggable(id, false);
        }
        intended_positions[id] = target_pos;

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

        if (item->segment_mode == RenderVoxelItem::SDF_NODE_SPLIT) {
            ImNodes::BeginInputAttribute(id * 1000 + 1,
                                         ImNodesPinShape_CircleFilled);
            ImGui::Text("SDF");
            ImNodes::EndInputAttribute();
        }

        if (item->source_type == 1 && item->source_node_id >= 0) {
            ImNodes::BeginInputAttribute(id * 1000 + 3,
                                         ImNodesPinShape_CircleFilled);
            ImGui::Text("Src");
            ImNodes::EndInputAttribute();
        }

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

        bool first_icon = true;
        if (item->sdf_data && bgfx::isValid(icons.circles)) {
            bool has_sdf_cache = !item->cached_mesh.empty();
            ImGui::Image(has_sdf_cache ? icons.circles_white : icons.circles,
                         ImVec2(20.0f, 20.0f));
            first_icon = false;
            if (ImGui::BeginItemTooltip()) {
                ImGui::Text(get_locale_cstr("tooltip.sdf_resolution"), item->sdf_data->getInfo().c_str());
                ImGui::EndTooltip();
            }
        }
        if (!item->stl_path.empty() && bgfx::isValid(icons.hexagon)) {
            if (!first_icon) {
                ImGui::SameLine();
            }
            first_icon = false;
            ImGui::Image(icons.hexagon, ImVec2(20.0f, 20.0f));
            if (ImGui::BeginItemTooltip()) {
                ImGui::Text(get_locale_cstr("tooltip.triangle_count"),
                            item->source_triangles.size());
                ImGui::EndTooltip();
            }
        }

        // Output attribute (连向所有子节点的统一出口)
        if (!item->children.empty()) {
            ImNodes::BeginOutputAttribute(static_cast<int>(id * 10 + 2),
                                          ImNodesPinShape_CircleFilled);
            ImGui::Text("");
            ImNodes::EndOutputAttribute();
        }

        if (sdf_sources.count(id)) {
            ImNodes::BeginOutputAttribute(id * 1000 + 2,
                                          ImNodesPinShape_CircleFilled);
            ImGui::Text("");
            ImNodes::EndOutputAttribute();
        }

        if (node_sources.count(id)) {
            ImNodes::BeginOutputAttribute(id * 1000 + 3,
                                          ImNodesPinShape_CircleFilled);
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

    // 绘制连线：父节点的统一 output 连向所有子节点的 input
    for (auto& [id, item] : this->items) {
        if (item->children.empty())
            continue;
        int parent_attr_id = id * 10 + 2;
        for (int child_id : item->children) {
            if (this->items.find(child_id) != this->items.end()) {
                int child_attr_id = child_id * 10 + 1;
                ImNodes::Link(link_id++, parent_attr_id, child_attr_id);
            }
        }
    }

    // 绘制 SDF 分割依赖线
    for (auto& [id, item] : this->items) {
        if (item->segment_mode == RenderVoxelItem::SDF_NODE_SPLIT &&
            item->sdf_split_target_id >= 0 &&
            this->items.find(item->sdf_split_target_id) !=
                this->items.end()) {
            int source_attr = item->sdf_split_target_id * 1000 + 2;
            int target_attr = id * 1000 + 1;
            ImNodes::Link(link_id++, source_attr, target_attr);
        }
    }

    // 绘制 Source Node 依赖线
    for (auto& [id, item] : this->items) {
        if (item->source_type == 1 && item->source_node_id >= 0 &&
            this->items.find(item->source_node_id) != this->items.end()) {
            int source_attr = item->source_node_id * 1000 + 3;
            int target_attr = id * 1000 + 3;
            ImNodes::Link(link_id++, source_attr, target_attr);
        }
    }

    ImNodes::MiniMap(0.2f, ImNodesMiniMapLocation_BottomLeft);
    ImNodes::EndNodeEditor();
    ImGui::EndChild();

    // 检测用户拖动：实际位置与 intended 不一致时固定该节点
    if (nav_layout_force_directed && nav_layout_initialized) {
        for (auto& [id, item] : this->items) {
            if (item->nav_layout_pinned)
                continue;
            ImVec2 actual = ImNodes::GetNodeGridSpacePos(id);
            ImVec2 intended = intended_positions[id];
            float dx = actual.x - intended.x;
            float dy = actual.y - intended.y;
            if (dx * dx + dy * dy > 4.0f) {
                item->nav_layout_pinned = true;
                item->nav_layout_pos[0] = actual.y / 1.5f;
                item->nav_layout_pos[1] = actual.x / 1.5f;
                item->nav_layout_vel[0] = 0.0f;
                item->nav_layout_vel[1] = 0.0f;
            }
        }
    }

    // 底部控制栏：力导向开关 + 重置布局
    ImGui::Checkbox(get_locale_cstr("label.force_layout"),
                    &nav_layout_force_directed);
    if (!nav_layout_force_directed) {
        nav_layout_initialized = false;
    } else {
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.reset_layout"))) {
            for (auto& [id, item] : this->items) {
                item->nav_layout_pinned = false;
                item->nav_layout_pos_set = false;
                item->nav_layout_vel[0] = 0.0f;
                item->nav_layout_vel[1] = 0.0f;
            }
            nav_layout_initialized = false;
        }
    }

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

bool RenderVoxelList::is_descendant_of(int child_id, int ancestor_id) {
    if (child_id == ancestor_id)
        return true;
    // Caller must hold locker
    auto it = items.find(child_id);
    if (it == items.end())
        return false;
    for (int parent_id : it->second->children) {
        if (is_descendant_of(parent_id, ancestor_id)) {
            return true;
        }
    }
    return false;
}

bool RenderVoxelList::would_form_source_cycle(int from_id, int to_id) {
    if (from_id == to_id)
        return true;
    // Caller must hold locker
    int current = to_id;
    std::unordered_set<int> visited;
    while (current >= 0) {
        if (current == from_id)
            return true;
        if (!visited.insert(current).second)
            break;
        auto it = items.find(current);
        if (it == items.end())
            break;
        current = it->second->source_type == 1
                      ? it->second->source_node_id
                      : -1;
    }
    return false;
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
        if (nav_layout_force_directed) {
            // 力导向模式下重新以树形布局作为起点收敛，
            // 避免直接覆盖用户已固定的节点和当前物理状态。
            nav_layout_initialized = false;
        } else {
            compute_layout(*this);
        }
    }
    update_nav_node_status = false;
}
}  // namespace sinriv::ui::render
