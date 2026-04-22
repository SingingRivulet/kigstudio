#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>
#include <stb/stb_truetype.h>
#include <unordered_set>
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {
void RenderVoxelList::render_ui() {
    float item_status_height = 0;
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.f);
    if (ImGui::Begin("STL Loader", nullptr,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar |
                         ImGuiWindowFlags_MenuBar |
                         ImGuiWindowFlags_NoBringToFrontOnFocus)) {
        if (ImGui::BeginMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Open STL (O)")) {
                    const char* file = tinyfd_openFileDialog(
                        "Open STL", "", 0, NULL, "STL file", 0);
                    if (file) {
                        this->queue_load_stl(file);
                    }
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("show")) {
                ImGui::Checkbox("show mesh", &showMesh);
                ImGui::Checkbox("show collision", &showCollision);
                ImGui::Checkbox("show voxels", &showVoxels);
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("axis")) {
                ImGui::Checkbox("show mesh axis", &showMeshAxis);
                ImGui::Checkbox("show voxel axis", &showVoxelAxis);
                ImGui::Checkbox("show collision axis", &showCollisionAxis);
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }
        if (ImGui::Button("update collision")) {
            std::cout << "update collision" << std::endl;
            // 应用碰撞体到两个结果体素
            this->queue_do_segment();
        }
        menu_height =
            ImGui::CalcWindowNextAutoFitSize(ImGui::GetCurrentWindow()).y;
        ImGui::SetWindowSize(ImVec2(window_width, menu_height));
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(0, (float)window_height), ImGuiCond_Always,
                            ImVec2(0.0f, 1.0f));
    if (ImGui::Begin("item status", nullptr,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar |
                         ImGuiWindowFlags_NoBringToFrontOnFocus)) {
        ImGui::Text("items:%d", this->get_num_items());

        item_status_height =
            ImGui::GetCursorPosY() + ImGui::GetStyle().WindowPadding.y;

        ImGui::SetWindowSize(ImVec2(window_width, item_status_height));
    }
    ImGui::End();

    ImGui::PopStyleVar();

    if (this->isQueueRunning()) {
        float async_y = window_height - item_status_height - 10.0f;
        ImGui::SetNextWindowPos(ImVec2((float)window_width, async_y),
                                ImGuiCond_Always, ImVec2(1.0f, 1.0f));
        ImGui::SetNextWindowSize(ImVec2(320, 70), ImGuiCond_Always);
        if (ImGui::Begin("async_voxel_loader", nullptr,
                         ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoBringToFrontOnFocus)) {
            ImGui::Text("%s", this->getQueueStatus().c_str());
            ImGui::ProgressBar(this->getQueueProgress(), ImVec2(-1, 0));
        }
        ImGui::End();
    }

    ImGui::SetNextWindowPos(ImVec2(0.f, (float)menu_height), ImGuiCond_Always,
                            ImVec2(0.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(300, 600), ImGuiCond_Once);
    if (ImGui::Begin("nav node map")) {
        ImNodes::BeginNodeEditor();

        ImNodes::EndNodeEditor();
    }
    ImGui::End();

    this->setMeshAxisVisible(showMeshAxis);
    this->setVoxelAxisVisible(showVoxelAxis);
    this->setMeshVisible(showMesh);
    this->setVoxelsVisible(showVoxels);
    this->setCollisionVisible(showCollision);
    this->update_nav_node_position();
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
                         bool& has_cycle) {
    auto it = mgr.items.find(node_id);

    // ✅ 不存在 = 无效节点
    if (it == mgr.items.end()) {
        return -1.0f;
    }

    // 🔴 环检测
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

    float left_x = layout_node(mgr, node.children[0], depth + 1, ctx,
                               visit_state, has_cycle);

    float my_x;

    if (left_x < 0) {
        my_x = (float)ctx.next_x;
        ctx.next_x += ctx.x_spacing;
    } else {
        my_x = left_x;
    }

    float right_x = layout_node(mgr, node.children[1], depth + 1, ctx,
                                visit_state, has_cycle);

    if (left_x >= 0 && right_x >= 0) {
        my_x = (left_x + right_x) * 0.5f;
    }

    node.nav_node_position[0] = (int)my_x;
    node.nav_node_position[1] = depth * ctx.y_spacing;

    visit_state[node_id] = 2;

    return my_x;
}

inline void compute_layout(RenderVoxelList& mgr) {
    auto roots = mgr.find_roots();

    LayoutContext ctx;
    std::unordered_map<int, int> visit_state;
    bool has_cycle = false;

    for (int root_id : roots) {
        layout_node(mgr, root_id, 0, ctx, visit_state, has_cycle);
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
        for (int i = 0; i < 2; ++i) {
            int child = item->children[i];
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