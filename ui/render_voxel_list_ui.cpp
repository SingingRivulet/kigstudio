#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <stb/stb_truetype.h>
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
        ImGui::End();
    }

    ImGui::SetNextWindowPos(ImVec2(0, (float)window_height), ImGuiCond_Always,
                            ImVec2(0.0f, 1.0f));

    if (ImGui::Begin("item status", nullptr,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar |
                         ImGuiWindowFlags_NoBringToFrontOnFocus)) {
        ImGui::Text("items:%d", this->get_num_items());

        item_status_height =
            ImGui::GetCursorPosY() + ImGui::GetStyle().WindowPadding.y;

        ImGui::SetWindowSize(ImVec2(window_width, item_status_height));

        ImGui::End();
    }

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
            ImGui::End();
        }
    }

    this->setMeshAxisVisible(showMeshAxis);
    this->setVoxelAxisVisible(showVoxelAxis);
    this->setMeshVisible(showMesh);
    this->setVoxelsVisible(showVoxels);
    this->setCollisionVisible(showCollision);
}
}  // namespace sinriv::ui::render