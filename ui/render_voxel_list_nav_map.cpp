#include "render_voxel_list.h"

#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>

#include <cmath>
#include <cstdio>
#include <unordered_set>

#include "kigstudio/utils/locale.h"

namespace sinriv::ui::render {
inline void compute_layout(RenderVoxelList& mgr);

namespace {

// 折叠节点 id 取一个足够大的负值（真实节点 id >= 0）。这样折叠节点的所有
// 引脚 id（fold_id * 10 + k）始终为负，永远不会与真实节点的正数引脚 id
// 冲突——若用 -1 - parent_id，当父节点子节点很多时，引脚 id 会溢出为正数，
// 与真实引脚撞车，导致连线错乱。
constexpr int kFoldNodeBase = 1000000;
inline int fold_node_id(int parent_id) { return -kFoldNodeBase - parent_id; }
inline int fold_node_parent(int fold_id) { return -kFoldNodeBase - fold_id; }

// 收集需要隐藏的节点：某个节点子节点数 > 4 时，其整个子树
// （子节点及所有后代）都被折叠隐藏。
void collect_hidden_nodes(RenderVoxelList& mgr,
                          std::unordered_set<int>& hidden) {
    for (auto& [id, item] : mgr.items) {
        if (item->children.size() <= 4)
            continue;
        for (int child_id : item->children) {
            std::vector<int> stack{child_id};
            while (!stack.empty()) {
                int cur = stack.back();
                stack.pop_back();
                if (!hidden.insert(cur).second)
                    continue;
                auto it = mgr.items.find(cur);
                if (it == mgr.items.end())
                    continue;
                for (int gc : it->second->children)
                    stack.push_back(gc);
            }
        }
    }
}

}  // namespace

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
    std::unordered_set<int> addon_sources;
    for (auto& [id, item] : this->items) {
        if (item->segment_mode == RenderVoxelItem::SDF_NODE_SPLIT &&
            item->sdf_split_target_id >= 0) {
            sdf_sources.insert(item->sdf_split_target_id);
        }
        if (item->source_type == 1 && item->source_node_id >= 0) {
            node_sources.insert(item->source_node_id);
        }
        if (item->source_type == 2 && item->addon_base_node_id >= 0) {
            addon_sources.insert(item->addon_base_node_id);
        }
    }

    // 计算折叠隐藏节点集合（子节点数 > 4 的节点折叠其整个子树）
    std::unordered_set<int> hidden_nodes;
    collect_hidden_nodes(*this, hidden_nodes);

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
        nav_layout_velocity_threshold_frame = 0;
    }

    // 收集所有边（父子 + SDF 依赖 + Source Node 依赖）
    if (nav_layout_force_directed) {
        for (auto& [id, item] : this->items) {
            if (hidden_nodes.count(id))
                continue;
            for (int child_id : item->children) {
                if (!hidden_nodes.count(child_id) &&
                    this->items.find(child_id) != this->items.end()) {
                    layout_edges.push_back({id, child_id});
                }
            }
        }
        for (auto& [id, item] : this->items) {
            if (hidden_nodes.count(id))
                continue;
            if (item->segment_mode == RenderVoxelItem::SDF_NODE_SPLIT &&
                item->sdf_split_target_id >= 0 &&
                !hidden_nodes.count(item->sdf_split_target_id) &&
                this->items.find(item->sdf_split_target_id) !=
                    this->items.end()) {
                layout_edges.push_back({item->sdf_split_target_id, id});
            }
        }
        for (auto& [id, item] : this->items) {
            if (hidden_nodes.count(id))
                continue;
            if (item->source_type == 1 && item->source_node_id >= 0 &&
                !hidden_nodes.count(item->source_node_id) &&
                this->items.find(item->source_node_id) != this->items.end()) {
                layout_edges.push_back({item->source_node_id, id});
            }
        }
        for (auto& [id, item] : this->items) {
            if (hidden_nodes.count(id))
                continue;
            if (item->source_type == 2 && item->addon_base_node_id >= 0 &&
                !hidden_nodes.count(item->addon_base_node_id) &&
                this->items.find(item->addon_base_node_id) !=
                    this->items.end()) {
                layout_edges.push_back({item->addon_base_node_id, id});
            }
        }
    }

    // 力导向迭代一步
    if (nav_layout_force_directed && nav_layout_initialized &&
        this->items.size() > 1) {
        const bool use_velocity_threshold =
            nav_layout_velocity_threshold_frame >=
            nav_layout_velocity_threshold_start_frame;
        if (!use_velocity_threshold) {
            ++nav_layout_velocity_threshold_frame;
        }
        std::unordered_map<int, ImVec2> forces;
        for (auto& [id, item] : this->items) {
            forces[id] = ImVec2(0.0f, 0.0f);
        }

        // 节点间斥力
        for (auto it1 = this->items.begin(); it1 != this->items.end(); ++it1) {
            for (auto it2 = std::next(it1); it2 != this->items.end(); ++it2) {
                if (hidden_nodes.count(it1->first) ||
                    hidden_nodes.count(it2->first))
                    continue;
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
            if (item->nav_layout_pinned || hidden_nodes.count(id))
                continue;
            auto& f = forces[id];
            // 微小向心力，把图拉向原点 (0,0)，让整体更紧凑
            forces[id].x +=
                nav_layout_center_pull * (0.0f - item->nav_layout_pos[0]);
            forces[id].y +=
                nav_layout_center_pull * (0.0f - item->nav_layout_pos[1]);
            item->nav_layout_vel[0] =
                (item->nav_layout_vel[0] + f.x * nav_layout_dt) *
                nav_layout_damping;
            item->nav_layout_vel[1] =
                (item->nav_layout_vel[1] + f.y * nav_layout_dt) *
                nav_layout_damping;
            float speed =
                std::sqrt(item->nav_layout_vel[0] * item->nav_layout_vel[0] +
                          item->nav_layout_vel[1] * item->nav_layout_vel[1]);
            if (speed > nav_layout_max_speed) {
                item->nav_layout_vel[0] =
                    item->nav_layout_vel[0] / speed * nav_layout_max_speed;
                item->nav_layout_vel[1] =
                    item->nav_layout_vel[1] / speed * nav_layout_max_speed;
            } else if (use_velocity_threshold &&
                       speed < nav_layout_velocity_threshold) {
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
        if (hidden_nodes.count(id))
            continue;
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
            if (!item->title.empty()) {
                ImGui::Text("%s (%s)", item->title.c_str(),
                            get_locale_cstr("label.updating"));
            } else {
                ImGui::Text(get_locale_cstr("label.node_updating"), id);
            }
        } else {
            if (!item->title.empty()) {
                ImGui::TextUnformatted(item->title.c_str());
            } else {
                ImGui::Text(get_locale_cstr("label.node"), id);
            }
        }
        ImNodes::EndNodeTitleBar();

        // Input attribute (来自父节点)
        ImNodes::BeginInputAttribute(id * 10 + 1, ImNodesPinShape_CircleFilled);
        ImGui::Text("");
        ImNodes::EndInputAttribute();

        if (item->segment_mode == RenderVoxelItem::SDF_NODE_SPLIT) {
            ImGui::SameLine(0.0f, 4.0f);
            ImNodes::BeginInputAttribute(id * 1000 + 1,
                                         ImNodesPinShape_CircleFilled);
            ImGui::Text("");
            ImNodes::EndInputAttribute();
        }

        if (item->source_type == 1 && item->source_node_id >= 0) {
            ImGui::SameLine(0.0f, 4.0f);
            ImNodes::BeginInputAttribute(id * 1000 + 3,
                                         ImNodesPinShape_CircleFilled);
            ImGui::Text("");
            ImNodes::EndInputAttribute();
        }

        if (item->source_type == 2 && item->addon_base_node_id >= 0) {
            ImGui::SameLine(0.0f, 4.0f);
            ImNodes::BeginInputAttribute(id * 1000 + 4,
                                         ImNodesPinShape_CircleFilled);
            ImGui::Text("");
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
                ImGui::Text(get_locale_cstr("tooltip.sdf_resolution"),
                            item->sdf_data->getInfo().c_str());
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
        bool output_attr_on_line = false;
        if (!item->children.empty()) {
            ImNodes::BeginOutputAttribute(static_cast<int>(id * 10 + 2),
                                          ImNodesPinShape_CircleFilled);
            ImGui::Text("");
            ImNodes::EndOutputAttribute();
            output_attr_on_line = true;
        }

        if (sdf_sources.count(id)) {
            if (output_attr_on_line) {
                ImGui::SameLine(0.0f, 4.0f);
            }
            ImNodes::BeginOutputAttribute(id * 1000 + 2,
                                          ImNodesPinShape_CircleFilled);
            ImGui::Text("");
            ImNodes::EndOutputAttribute();
            output_attr_on_line = true;
        }

        if (node_sources.count(id)) {
            if (output_attr_on_line) {
                ImGui::SameLine(0.0f, 4.0f);
            }
            ImNodes::BeginOutputAttribute(id * 1000 + 3,
                                          ImNodesPinShape_CircleFilled);
            ImGui::Text("");
            ImNodes::EndOutputAttribute();
            output_attr_on_line = true;
        }

        if (addon_sources.count(id)) {
            if (output_attr_on_line) {
                ImGui::SameLine(0.0f, 4.0f);
            }
            ImNodes::BeginOutputAttribute(id * 1000 + 4,
                                          ImNodesPinShape_CircleFilled);
            ImGui::Text("");
            ImNodes::EndOutputAttribute();
            output_attr_on_line = true;
        }

        ImNodes::EndNode();

        if (is_current) {
            ImNodes::PopColorStyle();
            ImNodes::PopColorStyle();
            ImNodes::PopColorStyle();
        }
    }

    // 渲染折叠节点：子节点数 > 4 的父节点，用一个折叠节点代表其整个子树。
    // 与普通节点一致：一个入口（来自父节点 output）、一个出口（连向所有子节点），
    // 子节点本身被隐藏，故出口为悬空占位。
    for (auto& [id, item] : this->items) {
        if (hidden_nodes.count(id))
            continue;
        if (item->children.size() <= 4)
            continue;
        int fold_id = fold_node_id(id);
        ImVec2 parent_pos;
        if (nav_layout_force_directed && nav_layout_initialized) {
            parent_pos = ImVec2(item->nav_layout_pos[1] * 1.5f,
                                item->nav_layout_pos[0] * 1.5f);
        } else {
            parent_pos = ImVec2((float)item->nav_node_position[1] * 1.5f,
                                (float)item->nav_node_position[0] * 1.5f);
        }
        ImVec2 fold_pos = ImVec2(parent_pos.x + 180.0f, parent_pos.y);
        ImNodes::SetNodeGridSpacePos(fold_id, fold_pos);
        ImNodes::SetNodeDraggable(fold_id, false);

        ImNodes::BeginNode(fold_id);
        ImNodes::BeginNodeTitleBar();
        ImGui::Text(get_locale_cstr("label.nodes_folded"),
                    static_cast<int>(item->children.size()));
        ImNodes::EndNodeTitleBar();
        ImNodes::BeginInputAttribute(fold_id * 10 + 1,
                                     ImNodesPinShape_CircleFilled);
        ImGui::Text("");
        ImNodes::EndInputAttribute();
        ImGui::SameLine(0.0f, 4.0f);
        ImNodes::BeginOutputAttribute(fold_id * 10 + 2,
                                      ImNodesPinShape_CircleFilled);
        ImGui::Text("");
        ImNodes::EndOutputAttribute();
        ImNodes::EndNode();
    }

    // 绘制连线：父节点的统一 output 连向所有子节点的 input；
    // 折叠时连向折叠节点 input，被隐藏子树内的连线不再绘制。
    for (auto& [id, item] : this->items) {
        if (item->children.empty())
            continue;
        int parent_attr_id = id * 10 + 2;
        if (item->children.size() > 4) {
            int fold_in = fold_node_id(id) * 10 + 1;
            ImNodes::Link(link_id++, parent_attr_id, fold_in);
            continue;
        }
        for (int child_id : item->children) {
            if (!hidden_nodes.count(child_id) &&
                this->items.find(child_id) != this->items.end()) {
                int child_attr_id = child_id * 10 + 1;
                ImNodes::Link(link_id++, parent_attr_id, child_attr_id);
            }
        }
    }

    const ImU32 src_link_color = IM_COL32(160, 72, 220, 220);
    const ImU32 src_link_color_active = IM_COL32(190, 96, 255, 255);
    ImNodes::PushColorStyle(ImNodesCol_Link, src_link_color);
    ImNodes::PushColorStyle(ImNodesCol_LinkHovered, src_link_color_active);
    ImNodes::PushColorStyle(ImNodesCol_LinkSelected, src_link_color_active);

    // 绘制 SDF 分割依赖线
    for (auto& [id, item] : this->items) {
        if (item->segment_mode == RenderVoxelItem::SDF_NODE_SPLIT &&
            item->sdf_split_target_id >= 0 &&
            !hidden_nodes.count(id) &&
            !hidden_nodes.count(item->sdf_split_target_id) &&
            this->items.find(item->sdf_split_target_id) != this->items.end()) {
            int source_attr = item->sdf_split_target_id * 1000 + 2;
            int target_attr = id * 1000 + 1;
            ImNodes::Link(link_id++, source_attr, target_attr);
        }
    }

    // 绘制 Source Node 依赖线
    for (auto& [id, item] : this->items) {
        if (item->source_type == 1 && item->source_node_id >= 0 &&
            !hidden_nodes.count(id) &&
            !hidden_nodes.count(item->source_node_id) &&
            this->items.find(item->source_node_id) != this->items.end()) {
            int source_attr = item->source_node_id * 1000 + 3;
            int target_attr = id * 1000 + 3;
            ImNodes::Link(link_id++, source_attr, target_attr);
        }
    }

    ImNodes::PopColorStyle();
    ImNodes::PopColorStyle();
    ImNodes::PopColorStyle();

    // 绘制附加件底模依赖线（粉色）
    const ImU32 addon_link_color = IM_COL32(255, 100, 180, 220);
    const ImU32 addon_link_color_active = IM_COL32(255, 130, 210, 255);
    ImNodes::PushColorStyle(ImNodesCol_Link, addon_link_color);
    ImNodes::PushColorStyle(ImNodesCol_LinkHovered, addon_link_color_active);
    ImNodes::PushColorStyle(ImNodesCol_LinkSelected, addon_link_color_active);

    for (auto& [id, item] : this->items) {
        if (item->source_type == 2 && item->addon_base_node_id >= 0 &&
            !hidden_nodes.count(id) &&
            !hidden_nodes.count(item->addon_base_node_id) &&
            this->items.find(item->addon_base_node_id) != this->items.end()) {
            int source_attr = item->addon_base_node_id * 1000 + 4;
            int target_attr = id * 1000 + 4;
            ImNodes::Link(link_id++, source_attr, target_attr);
        }
    }

    ImNodes::PopColorStyle();
    ImNodes::PopColorStyle();
    ImNodes::PopColorStyle();

    ImNodes::MiniMap(0.2f, ImNodesMiniMapLocation_BottomLeft);
    ImNodes::EndNodeEditor();
    ImGui::EndChild();

    // 检测用户拖动：实际位置与 intended 不一致时固定该节点
    if (nav_layout_force_directed && nav_layout_initialized) {
        for (auto& [id, item] : this->items) {
            if (item->nav_layout_pinned || hidden_nodes.count(id))
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
            nav_layout_velocity_threshold_frame = 0;
        }
    }

    // 点击节点切换 render_id；点击折叠节点弹出折叠节点列表
    int num_selected = ImNodes::NumSelectedNodes();
    if (num_selected > 0) {
        static std::vector<int> selected_nodes;
        selected_nodes.resize(num_selected);
        ImNodes::GetSelectedNodes(selected_nodes.data());
        for (int sel : selected_nodes) {
            if (sel < 0) {
                int parent_id = fold_node_parent(sel);
                if (this->items.find(parent_id) != this->items.end()) {
                    nav_fold_popup_parent = parent_id;
                    ImGui::OpenPopup("folded_nodes");
                }
            } else {
                this->setRenderId_unsafe(sel);
            }
        }
        ImNodes::ClearNodeSelection();
    }

    // 折叠节点弹窗：列出被折叠子树内的所有节点，点击即可跳转（与点击原节点行为一致）。
    // 使用 BeginPopup：鼠标点击弹窗外部会自动关闭；列表区域限制高度 300 并带滚动条。
    if (ImGui::BeginPopup("folded_nodes")) {
        ImGui::TextUnformatted(get_locale_cstr("window.folded_nodes"));
        ImGui::Separator();
        if (nav_fold_popup_parent >= 0) {
            auto fold_it = this->items.find(nav_fold_popup_parent);
            if (fold_it != this->items.end()) {
                std::vector<int> folded;
                for (int child_id : fold_it->second->children) {
                    std::vector<int> stack{child_id};
                    while (!stack.empty()) {
                        int cur = stack.back();
                        stack.pop_back();
                        folded.push_back(cur);
                        auto cur_it = this->items.find(cur);
                        if (cur_it == this->items.end())
                            continue;
                        for (int gc : cur_it->second->children)
                            stack.push_back(gc);
                    }
                }
                ImGui::BeginChild("folded_list", ImVec2(320.0f, 300.0f), false,
                                  ImGuiWindowFlags_AlwaysVerticalScrollbar);
                const float icon_size = 20.0f;
                const float icon_spacing = 4.0f;
                for (int nid : folded) {
                    auto nit = this->items.find(nid);
                    if (nit == this->items.end())
                        continue;
                    auto* node = nit->second.get();

                    bool has_sdf = node->sdf_data && bgfx::isValid(icons.circles);
                    bool has_mesh =
                        !node->stl_path.empty() && bgfx::isValid(icons.hexagon);
                    int icon_count = (has_sdf ? 1 : 0) + (has_mesh ? 1 : 0);
                    float icons_width =
                        icon_count > 0
                            ? icon_count * icon_size +
                                  (icon_count - 1) * icon_spacing
                            : 0.0f;

                    char label_buf[256];
                    if (node->title.empty()) {
                        snprintf(label_buf, sizeof(label_buf),
                                 get_locale_cstr("label.node"), nid);
                    } else {
                        snprintf(label_buf, sizeof(label_buf), "%s",
                                 node->title.c_str());
                    }

                    float label_width = ImGui::GetContentRegionAvail().x -
                                        icons_width -
                                        ImGui::GetStyle().ItemSpacing.x;
                    if (label_width < 40.0f)
                        label_width = 40.0f;

                    if (ImGui::Selectable(label_buf, nid == render_id, 0,
                                          ImVec2(label_width, 0.0f))) {
                        this->setRenderId_unsafe(nid);
                    }

                    // 右侧显示 mesh / sdf 图标，鼠标悬停显示状态（与普通节点一致）
                    if (has_sdf) {
                        ImGui::SameLine();
                        bool has_sdf_cache = !node->cached_mesh.empty();
                        ImGui::Image(has_sdf_cache ? icons.circles_white
                                                   : icons.circles,
                                     ImVec2(icon_size, icon_size));
                        if (ImGui::BeginItemTooltip()) {
                            ImGui::Text(get_locale_cstr("tooltip.sdf_resolution"),
                                        node->sdf_data->getInfo().c_str());
                            ImGui::EndTooltip();
                        }
                    }
                    if (has_mesh) {
                        ImGui::SameLine();
                        ImGui::Image(icons.hexagon, ImVec2(icon_size, icon_size));
                        if (ImGui::BeginItemTooltip()) {
                            ImGui::Text(get_locale_cstr("tooltip.triangle_count"),
                                        node->source_triangles.size());
                            ImGui::EndTooltip();
                        }
                    }
                }
                ImGui::EndChild();
            }
        }
        ImGui::EndPopup();
    } else {
        nav_fold_popup_parent = -1;
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
        if (child_id < 0)
            continue;
        float cx = layout_node(mgr, child_id, depth + 1, ctx, visit_state,
                               has_cycle, root_id);
        if (cx >= 0)
            child_xs.push_back(cx);
    }

    float my_x;
    if (child_xs.empty()) {
        my_x = (float)ctx.next_x;
        ctx.next_x += ctx.x_spacing;
    } else {
        float sum = 0.0f;
        for (float cx : child_xs)
            sum += cx;
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
        current =
            it->second->source_type == 1 ? it->second->source_node_id : -1;
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
            nav_layout_velocity_threshold_frame = 0;
        } else {
            compute_layout(*this);
        }
    }
    update_nav_node_status = false;
}
}  // namespace sinriv::ui::render
