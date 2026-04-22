#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <stb/stb_truetype.h>
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {
void RenderVoxelList::render_ui() {
    ImGui::SetNextWindowPos(ImVec2(0.f, 0.f), ImGuiCond_Once);
    ImGui::Begin("STL Loader");

    ImGui::Text("items:%d", this->get_num_items());

    if (ImGui::Button("Open STL (O)")) {
        const char* file =
            tinyfd_openFileDialog("Open STL", "", 0, NULL, "STL file", 0);
        if (file) {
            this->queue_load_stl(file);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("update collision")) {
        std::cout << "update collision" << std::endl;
        // 应用碰撞体到两个结果体素
        this->queue_do_segment();
    }

    ImGui::Checkbox("show mesh", &showMesh);
    ImGui::Checkbox("show collision", &showCollision);
    ImGui::Checkbox("show voxels", &showVoxels);
    ImGui::Checkbox("show mesh axis", &showMeshAxis);
    ImGui::Checkbox("show voxel axis", &showVoxelAxis);
    ImGui::Checkbox("show collision axis", &showCollisionAxis);
    ImGui::End();

    if (this->isQueueRunning()) {
        ImGui::SetNextWindowPos(
            ImVec2((float)window_width, (float)window_height), ImGuiCond_Always,
            ImVec2(1.0f, 1.0f));
        ImGui::SetNextWindowSize(ImVec2(320, 70), ImGuiCond_Always);
        ImGui::Begin("async_voxel_loader", nullptr,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar |
                         ImGuiWindowFlags_NoBringToFrontOnFocus);
        ImGui::Text("%s", this->getQueueStatus().c_str());
        ImGui::ProgressBar(this->getQueueProgress(), ImVec2(-1, 0));
        ImGui::End();
    }

    this->setMeshAxisVisible(showMeshAxis);
    this->setVoxelAxisVisible(showVoxelAxis);
    this->setMeshVisible(showMesh);
    this->setVoxelsVisible(showVoxels);
    this->setCollisionVisible(showCollision);
}
}  // namespace sinriv::ui::render