#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>
#include <stb/stb_truetype.h>
#include <type_traits>
#include <unordered_set>
#include <variant>
#include "kigstudio/utils/vec3.h"
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {
namespace {

using CollisionGroup = sinriv::kigstudio::voxel::collision::CollisionGroup;
using GeometryInstance = sinriv::kigstudio::voxel::collision::GeometryInstance;
using Sphere = sinriv::kigstudio::voxel::collision::Sphere;
using Cylinder = sinriv::kigstudio::voxel::collision::Cylinder;
using Capsule = sinriv::kigstudio::voxel::collision::Capsule;
using Box = sinriv::kigstudio::voxel::collision::Box;
using Transform = sinriv::kigstudio::voxel::collision::Transform;
using vec3f = sinriv::kigstudio::voxel::collision::vec3f;
using Plane = sinriv::kigstudio::Plane<float>;

void edit_float_stepper(const char* label, float& value, float step = 0.5f) {
    const float button_size = ImGui::GetFrameHeight();
    ImGui::PushID(label);
    char buf[128];
    //截断label中的##
    snprintf(buf, sizeof(buf), "%s", label);
    for (int i = 0; i < sizeof(buf) && buf[i] != '\0'; ++i) {
        if (buf[i] == '#') {
            buf[i] = '\0';
            break;
        }
    }
    ImGui::Text("%s", buf);
    ImGui::SameLine();
    if (ImGui::Button("-", ImVec2(button_size, 0))) {
        value -= step;
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80.0f);
    snprintf(buf, sizeof(buf), "##%s", label);
    ImGui::DragFloat(buf, &value, step, 0.0f, 0.0f, "%.2f");
    ImGui::SameLine();
    if (ImGui::Button("+", ImVec2(button_size, 0))) {
        value += step;
    }
    ImGui::PopID();
}

void edit_vec3_stepper(const char* label,
                       vec3f& value,
                       float step = 0.5f,
                       bool normalize = false) {
    const char* axis_names[] = {"X", "Y", "Z"};
    float values[3] = {value.x, value.y, value.z};
    ImGui::Text("%s", label);
    char buf[128];
    for (int i = 0; i < 3; ++i) {
        // if (i > 0) {
        //     ImGui::SameLine();
        // }
        snprintf(buf, sizeof(buf), "%s##%s", axis_names[i], label);
        edit_float_stepper(buf, values[i], step);
    }
    value = {values[0], values[1], values[2]};
    if (normalize) {
        value = sinriv::kigstudio::voxel::collision::safeNormalize(value);
    }
}
void edit_local_position_stepper(const char* label,
                                 vec3f& value,
                                 float step = 0.5f,
                                 bool normalize = false) {
    const char* axis_names[] = {"X", "Y", "Z"};
    float values[3] = {value.x, -value.y, value.z};
    ImGui::Text("%s", label);
    char buf[128];
    for (int i = 0; i < 3; ++i) {
        snprintf(buf, sizeof(buf), "%s##%s", axis_names[i], label);
        edit_float_stepper(buf, values[i], step);
    }
    value = {values[0], -values[1], values[2]};
    if (normalize) {
        value = sinriv::kigstudio::voxel::collision::safeNormalize(value);
    }
}

void edit_transform_controls(Transform& transform) {
    vec3f pos = transform.getPosition();
    edit_vec3_stepper("Position", pos);
    transform.setPosition(pos);

    vec3f euler_rad = transform.getRotationEuler();
    vec3f euler_deg = {bx::toDeg(euler_rad.x), bx::toDeg(euler_rad.y),
                       bx::toDeg(euler_rad.z)};
    edit_vec3_stepper("Rotation (deg)", euler_deg);
    transform.setRotationEuler({bx::toRad(euler_deg.x), bx::toRad(euler_deg.y),
                                bx::toRad(euler_deg.z)});
}

const char* geometry_type_name(const GeometryInstance& instance) {
    return std::visit(
        [](const auto& geometry) -> const char* {
            using T = std::decay_t<decltype(geometry)>;
            if constexpr (std::is_same_v<T, Sphere>) {
                return "Sphere";
            } else if constexpr (std::is_same_v<T, Cylinder>) {
                return "Cylinder";
            } else if constexpr (std::is_same_v<T, Capsule>) {
                return "Capsule";
            } else if constexpr (std::is_same_v<T, Box>) {
                return "Box";
            } else {
                return "Unknown";
            }
        },
        instance.geometry);
}

void edit_geometry_shape(GeometryInstance& instance) {
    std::visit(
        [](auto& geometry) {
            using T = std::decay_t<decltype(geometry)>;
            if constexpr (std::is_same_v<T, Sphere>) {
                edit_local_position_stepper("Center", geometry.center);
                edit_float_stepper("Radius", geometry.radius);
                geometry.radius = bx::max(0.0f, geometry.radius);
            } else if constexpr (std::is_same_v<T, Cylinder>) {
                edit_local_position_stepper("Start", geometry.start);
                edit_local_position_stepper("End", geometry.end);
                edit_float_stepper("Radius", geometry.radius);
                geometry.radius = bx::max(0.0f, geometry.radius);
            } else if constexpr (std::is_same_v<T, Capsule>) {
                edit_local_position_stepper("Start", geometry.start);
                edit_local_position_stepper("End", geometry.end);
                edit_float_stepper("Radius", geometry.radius);
                geometry.radius = bx::max(0.0f, geometry.radius);
            } else if constexpr (std::is_same_v<T, Box>) {
                edit_vec3_stepper("Half Extent", geometry.half_extent);
                geometry.half_extent.x = bx::max(0.0f, geometry.half_extent.x);
                geometry.half_extent.y = bx::max(0.0f, geometry.half_extent.y);
                geometry.half_extent.z = bx::max(0.0f, geometry.half_extent.z);
            }
        },
        instance.geometry);
}

void add_collision_geometry(CollisionGroup& group, int type_index) {
    switch (type_index) {
        case 0:
            group.add(Sphere{{0.0f, 0.0f, 0.0f}, 10.0f});
            break;
        case 1:
            group.add(
                Cylinder{{0.0f, 0.0f, -10.0f}, {0.0f, 0.0f, 10.0f}, 6.0f});
            break;
        case 2:
            group.add(Capsule{{-10.0f, 0.0f, 0.0f}, {10.0f, 0.0f, 0.0f}, 6.0f});
            break;
        case 3:
            group.add(Box{{8.0f, 8.0f, 8.0f}});
            break;
    }
}

}  // namespace

void RenderVoxelList::render_ui() {
    processThumbnails();
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
                    show_file_loader = true;
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("View")) {
                if (ImGui::BeginMenu("Body")) {
                    ImGui::Checkbox("show mesh", &showMesh);
                    ImGui::Checkbox("show collision", &showCollision);
                    ImGui::Checkbox("show voxels", &showVoxels);
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu("Axis")) {
                    ImGui::Checkbox("show mesh axis", &showMeshAxis);
                    ImGui::Checkbox("show voxel axis", &showVoxelAxis);
                    ImGui::Checkbox("show collision axis", &showCollisionAxis);
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu("Bound")) {
                    ImGui::Checkbox("show collision bounds",
                                    &showCollisionBounds);
                    ImGui::EndMenu();
                }
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
        ImGui::Text("items:%d tasks:%d", this->get_num_items(),
                    this->queue_num);
        // 显示鼠标的三维位置
        ImGui::SameLine();
        ImGui::Text("mouse world pos:(%.2f,%.2f,%.2f)", mouse_world_pos.x,
                    mouse_world_pos.y, mouse_world_pos.z);

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

    render_nav_map();
    render_collision_node_editor();
    render_file_loader();

    this->setMeshAxisVisible(showMeshAxis);
    this->setVoxelAxisVisible(showVoxelAxis);
    this->setMeshVisible(showMesh);
    this->setVoxelsVisible(showVoxels);
    this->setCollisionVisible(showCollision);
    this->setCollisionBoundsVisible(showCollisionBounds);
    this->update_nav_node_position();
}

void RenderVoxelList::render_file_loader() {
    static std::string stl_file_path;
    static float voxel_size = 1.0f;
    if (show_file_loader) {
        ImGuiViewport* vp = ImGui::GetMainViewport();
        ImVec2 center = vp->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Once, ImVec2(0.5f, 0.5f));
        if (ImGui::Begin("Load STL File", nullptr,
                         ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("Click the button below to load an STL file.");
            if (ImGui::Button("Open File Dialog")) {
                const char* file = tinyfd_openFileDialog(
                    "Open STL", "", 0, NULL, "STL file", 0);
                if (file) {
                    stl_file_path = std::string(file);
                }
            }
            if (!stl_file_path.empty()) {
                ImGui::Text("Selected file: %s", stl_file_path.c_str());
            } else {
                ImGui::Text("No file selected.");
            }
            const float button_size = ImGui::GetFrameHeight();
            ImGui::Text("Voxel Size");
            ImGui::SameLine();
            if (ImGui::Button("-", ImVec2(button_size, 0))) {
                auto voxel_size_tmp = voxel_size / 2.0f;
                if (voxel_size_tmp >= 0.0001f) {
                    voxel_size = voxel_size_tmp;
                }
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(80.0f);
            ImGui::BeginDisabled(true);
            ImGui::DragFloat("##Voxel Size", &voxel_size, 0.1f, 0.0f, 0.0f, "%.4f");
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("+", ImVec2(button_size, 0))) {
                voxel_size = voxel_size * 2.0f;
                if (voxel_size > 1000.0f) {
                    voxel_size = 1000.0f;
                }
            }
            ImGui::BeginDisabled(stl_file_path.empty());
            if (ImGui::Button("Open")) {
                this->queue_load_stl(stl_file_path, voxel_size);
                show_file_loader = false;
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) {
                show_file_loader = false;
            }
        }
        ImGui::End();
    }
}

void RenderVoxelList::render_plane_editor(RenderVoxelItem& item) {
    static int plane_editor_item_id = -1;
    static int plane_input_mode = 0;
    static vec3f plane_point = {0.0f, 0.0f, 0.0f};
    static vec3f plane_normal = {0.0f, 1.0f, 0.0f};
    static vec3f plane_p1 = {0.0f, 0.0f, 0.0f};
    static vec3f plane_p2 = {1.0f, 0.0f, 0.0f};
    static vec3f plane_p3 = {0.0f, 1.0f, 0.0f};
    static bool pick_point_by_mouse = false;
    static bool pick_normal_by_mouse = false;
    static bool pick_p1_by_mouse = false;
    static bool pick_p2_by_mouse = false;
    static bool pick_p3_by_mouse = false;
    static std::string plane_error_message;
    if (ImGui::CollapsingHeader("segment plane",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("A: %.4f", item.plane.A);
        ImGui::Text("B: %.4f", item.plane.B);
        ImGui::Text("C: %.4f", item.plane.C);
        ImGui::Text("D: %.4f", item.plane.D);
        if (ImGui::Button("Edit plane")) {
            plane_editor_item_id = item.id;
            auto [point, normal] = item.plane.getPointNormalForm();
            vec3f p1, p2, p3;
            {
                vec3f tangent = normal.cross(vec3f(0.0f, 0.0f, 1.0f));
                if (tangent.length() < 1e-6f) {
                    tangent = normal.cross(vec3f(0.0f, 1.0f, 0.0f));
                }
                tangent =
                    sinriv::kigstudio::voxel::collision::safeNormalize(tangent);
                vec3f bitangent =
                    sinriv::kigstudio::voxel::collision::safeNormalize(
                        normal.cross(tangent));
                p1 = point;
                p2 = point + tangent;
                p3 = point + bitangent;
            }
            plane_point = point;
            plane_normal = normal;
            plane_p1 = p1;
            plane_p2 = p2;
            plane_p3 = p3;
            plane_error_message.clear();
            show_edit_segment_plane = true;
        }

        if (show_edit_segment_plane) {
            if (ImGui::Begin("Edit Segment Plane", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
                if (plane_editor_item_id != item.id) {
                    ImGui::TextUnformatted(
                        "Plane editor is bound to another item.");
                } else {
                    const char* plane_modes[] = {"Three-Point", "Point-Normal"};
                    ImGui::Combo("input mode", &plane_input_mode, plane_modes,
                                 IM_ARRAYSIZE(plane_modes));
                    ImGui::Separator();

                    if (plane_input_mode == 0) {
                        edit_vec3_stepper("P1", plane_p1);
                        if (ImGui::Checkbox("Pick P1 by mouse",
                                            &pick_p1_by_mouse)) {
                            if (pick_p1_by_mouse) {
                                pick_p2_by_mouse = false;
                                pick_p3_by_mouse = false;
                            }
                        }
                        edit_vec3_stepper("P2", plane_p2);
                        if (ImGui::Checkbox("Pick P2 by mouse",
                                            &pick_p2_by_mouse)) {
                            if (pick_p2_by_mouse) {
                                pick_p1_by_mouse = false;
                                pick_p3_by_mouse = false;
                            }
                        }
                        edit_vec3_stepper("P3", plane_p3);
                        if (ImGui::Checkbox("Pick P3 by mouse",
                                            &pick_p3_by_mouse)) {
                            if (pick_p3_by_mouse) {
                                pick_p1_by_mouse = false;
                                pick_p2_by_mouse = false;
                            }
                        }

                        if (pick_p1_by_mouse && mouse_world_pos_valid &&
                            mouse_world_pos_picked) {
                            plane_p1 = mouse_world_pos;
                        }
                        if (pick_p2_by_mouse && mouse_world_pos_valid &&
                            mouse_world_pos_picked) {
                            plane_p2 = mouse_world_pos;
                        }
                        if (pick_p3_by_mouse && mouse_world_pos_valid &&
                            mouse_world_pos_picked) {
                            plane_p3 = mouse_world_pos;
                        }
                        hightlight_pos.push_back(
                            {plane_p1, {0.8f, 0.0f, 0.5f}});
                        hightlight_pos.push_back(
                            {plane_p2, {0.8f, 0.0f, 0.7f}});
                        hightlight_pos.push_back(
                            {plane_p3, {0.8f, 0.0f, 0.9f}});
                        const vec3f v1 = plane_p2 - plane_p1;
                        const vec3f v2 = plane_p3 - plane_p1;
                        vec3f normal = v1.cross(v2);
                        if (normal.length() < 1e-6f) {
                            plane_error_message = "Three points are collinear.";
                        } else {
                            normal = sinriv::kigstudio::voxel::collision::
                                safeNormalize(normal);
                            item.plane = Plane(plane_p1, normal);
                            plane_error_message.clear();
                        }
                    } else if (plane_input_mode == 1) {
                        edit_vec3_stepper("Point", plane_point);
                        if (ImGui::Checkbox("Pick point by mouse",
                                            &pick_point_by_mouse)) {
                            if (pick_point_by_mouse) {
                                pick_normal_by_mouse = false;
                            }
                        }
                        edit_vec3_stepper("Normal", plane_normal, 0.1f);
                        if (ImGui::Checkbox("Pick normal by mouse",
                                            &pick_normal_by_mouse)) {
                            if (pick_normal_by_mouse) {
                                pick_point_by_mouse = false;
                            }
                        }
                        if (pick_point_by_mouse && mouse_world_pos_valid &&
                            mouse_world_pos_picked) {
                            plane_point = mouse_world_pos;
                        }
                        if (pick_normal_by_mouse && mouse_world_pos_valid &&
                            mouse_world_pos_picked) {
                            plane_normal = mouse_world_pos - plane_point;
                            plane_normal = sinriv::kigstudio::voxel::collision::
                                safeNormalize(plane_normal);
                        }
                        hightlight_pos.push_back(
                            {plane_point, {0.8f, 0.0f, 0.5f}});
                        hightlight_pos.push_back(
                            {plane_point + plane_normal * 2.0f,
                             {0.8f, 0.0f, 0.9f}});
                        try {
                            item.plane = Plane(plane_point, plane_normal);
                            plane_error_message.clear();
                        } catch (const std::exception& e) {
                            plane_error_message = e.what();
                        }
                    }

                    if (!plane_error_message.empty()) {
                        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%s",
                                           plane_error_message.c_str());
                    }
                }

                if (ImGui::Button("Close")) {
                    plane_error_message.clear();
                    show_edit_segment_plane = false;
                }
            }
            ImGui::End();
        }
    }
}

void RenderVoxelList::render_collision_node_editor() {
    ImGui::SetNextWindowPos(ImVec2((float)window_width, (float)menu_height),
                            ImGuiCond_Always, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(360, 620), ImGuiCond_Once);
    if (ImGui::Begin("collision editor")) {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it == items.end()) {
            ImGui::TextUnformatted("No active item.");
        } else {
            if (ImGui::Button("Delete")) {
                item_it->second->queue_release = true;
            }
            ImGui::SameLine();
            if (ImGui::Button("Save as STL")) {
                const char* filters[] = {"*.stl"};
                const char* file =
                    tinyfd_saveFileDialog("Save Voxel as STL", "node_voxel.stl",
                                          1, filters, "STL files");
                if (file) {
                    std::vector<std::tuple<sinriv::kigstudio::voxel::Triangle,
                                           sinriv::kigstudio::voxel::vec3f>>
                        mesh;
                    int numTriangles = 0;
                    for (auto triangles :
                         sinriv::kigstudio::voxel::generateMesh(
                             item_it->second->voxel_grid_data, 0.5,
                             numTriangles, true)) {
                        mesh.push_back(triangles);
                    }
                    sinriv::kigstudio::voxel::saveMeshToASCIISTL(mesh, file);
                }
            }
            RenderVoxelItem& item = *item_it->second;

            ImGui::Text("render item: %d", item.id);
            ImGui::Separator();

            const char* segment_modes[] = {"Collision", "Plane"};
            int current_segment_mode =
                item.segment_mode == RenderVoxelItem::PLANE ? 1 : 0;
            if (ImGui::Combo("segment mode", &current_segment_mode,
                             segment_modes, IM_ARRAYSIZE(segment_modes))) {
                item.segment_mode = current_segment_mode == 0
                                        ? RenderVoxelItem::COLLISION
                                        : RenderVoxelItem::PLANE;
            }
            render_plane_editor(item);

            if (ImGui::CollapsingHeader("collision root",
                                        ImGuiTreeNodeFlags_DefaultOpen)) {
                edit_transform_controls(item.collision_group.transform);
            }

            if (ImGui::CollapsingHeader("collision group",
                                        ImGuiTreeNodeFlags_DefaultOpen)) {
                static int new_geometry_type = 0;
                const char* geometry_types[] = {"Sphere", "Cylinder", "Capsule",
                                                "Box"};

                ImGui::SetNextItemWidth(140.0f);
                ImGui::Combo("new shape", &new_geometry_type, geometry_types,
                             IM_ARRAYSIZE(geometry_types));
                ImGui::SameLine();
                if (ImGui::Button("Add shape")) {
                    add_collision_geometry(item.collision_group,
                                           new_geometry_type);
                }

                auto& geometries = item.collision_group.geometries();
                int remove_index = -1;
                for (int member_idx = 0;
                     member_idx < static_cast<int>(geometries.size());
                     ++member_idx) {
                    auto& instance = geometries[member_idx];
                    ImGui::PushID(member_idx);
                    const std::string header =
                        std::string(geometry_type_name(instance)) + " [" +
                        std::to_string(member_idx) + "]";
                    if (ImGui::CollapsingHeader(
                            header.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
                        edit_transform_controls(instance.transform);
                        ImGui::Separator();
                        edit_geometry_shape(instance);
                        ImGui::Separator();
                        if (ImGui::Button("Delete")) {
                            remove_index = member_idx;
                        }
                    }
                    ImGui::PopID();
                }

                if (remove_index >= 0) {
                    geometries.erase(geometries.begin() + remove_index);
                }

                if (geometries.empty()) {
                    ImGui::TextUnformatted("No collision shapes.");
                }
            }
        }
    }
    ImGui::End();
}

void RenderVoxelList::render_nav_map() {
    std::lock_guard<std::mutex> lock(locker);
    ImGui::SetNextWindowPos(ImVec2(0.f, (float)menu_height), ImGuiCond_Always,
                            ImVec2(0.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(300, 600), ImGuiCond_Once);
    if (!ImGui::Begin("nav node map")) {
        ImGui::End();
        return;
    }

    ImNodes::BeginNodeEditor();

    int link_id = 0;
    for (auto& [id, item] : this->items) {
        // 设置节点固定坐标
        ImNodes::SetNodeGridSpacePos(
            id, ImVec2((float)item->nav_node_position[1] * 1.5,
                       (float)item->nav_node_position[0] * 1.5));
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
            ImGui::Text("Node %d (updating...)", id);
        } else {
            ImGui::Text("Node %d", id);
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
        ImNodes::BeginOutputAttribute(id * 10 + 2,
                                      ImNodesPinShape_CircleFilled);
        ImGui::Text("");
        ImNodes::EndOutputAttribute();

        ImNodes::BeginOutputAttribute(id * 10 + 3,
                                      ImNodesPinShape_CircleFilled);
        ImGui::Text("");
        ImNodes::EndOutputAttribute();

        ImNodes::EndNode();

        if (is_current) {
            ImNodes::PopColorStyle();
            ImNodes::PopColorStyle();
            ImNodes::PopColorStyle();
        }
    }

    // 绘制连线：父节点的 output 连向子节点的 input
    for (auto& [id, item] : this->items) {
        for (int i = 0; i < 2; ++i) {
            int child_id = item->children[i];
            if (this->items.find(child_id) != this->items.end()) {
                int parent_attr_id = id * 10 + (i == 0 ? 2 : 3);
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
                         bool& has_cycle) {
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
    std::cout << "compute_layout roots: " << roots.size() << std::endl;

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

void RenderVoxelList::ensureThumbnailResources() {
    if (bgfx::isValid(thumb_fb_)) {
        return;
    }
    constexpr uint16_t ts = 128;
    constexpr uint64_t flags =
        BGFX_TEXTURE_RT | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP;

    thumb_color_tex_ = bgfx::createTexture2D(ts, ts, false, 1,
                                             bgfx::TextureFormat::BGRA8, flags);
    thumb_depth_tex_ = bgfx::createTexture2D(ts, ts, false, 1,
                                             bgfx::TextureFormat::D32F, flags);

    bgfx::TextureHandle attachments[] = {thumb_color_tex_, thumb_depth_tex_};
    thumb_fb_ = bgfx::createFrameBuffer(2, attachments, false);

    thumb_shader_ = std::make_unique<RenderMeshShader>(100, 101);
}

void RenderVoxelList::destroyThumbnailResources() {
    if (bgfx::isValid(thumb_fb_)) {
        bgfx::destroy(thumb_fb_);
        thumb_fb_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(thumb_color_tex_)) {
        bgfx::destroy(thumb_color_tex_);
        thumb_color_tex_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(thumb_depth_tex_)) {
        bgfx::destroy(thumb_depth_tex_);
        thumb_depth_tex_ = BGFX_INVALID_HANDLE;
    }
    thumb_shader_.reset();
}

void RenderVoxelList::processThumbnails() {
    // 1. 为 dirty 的 items 入队
    {
        std::lock_guard<std::mutex> lock(locker);
        for (auto& [id, item] : items) {
            if (item->thumbnail_dirty) {
                // 避免重复入队
                bool already_queued = false;
                std::queue<ThumbnailTask> temp = thumbnail_queue;
                while (!temp.empty()) {
                    if (temp.front().item_id == id) {
                        already_queued = true;
                        break;
                    }
                    temp.pop();
                }
                if (!already_queued) {
                    thumbnail_queue.push({id, ThumbnailTask::RENDER, 0});
                }
                item->thumbnail_dirty = false;
            }
        }
    }

    if (thumbnail_queue.empty()) {
        return;
    }

    auto& task = thumbnail_queue.front();

    switch (task.stage) {
        case ThumbnailTask::RENDER: {
            std::lock_guard<std::mutex> lock(locker);
            auto it = items.find(task.item_id);
            if (it == items.end()) {
                thumbnail_queue.pop();
                return;
            }
            auto& item = it->second;

            // 如果 voxel_renderer 为空但有 voxel_grid_data，先生成 mesh（放到
            // queue 线程）
            if (item->voxel_renderer.empty() &&
                item->voxel_grid_data.num_chunk() > 0) {
                // 检查是否已有生成结果
                {
                    std::lock_guard<std::mutex> lock(thumbnail_mesh_mutex);
                    auto res_it = thumbnail_mesh_results.find(task.item_id);
                    if (res_it != thumbnail_mesh_results.end()) {
                        item->voxel_renderer.loadGeometry(res_it->second);
                        thumbnail_mesh_results.erase(res_it);
                        // 继续执行后续 RENDER 逻辑
                    } else {
                        // 没有结果，检查是否已提交任务
                        if (thumbnail_mesh_pending.find(task.item_id) ==
                            thumbnail_mesh_pending.end()) {
                            // 提交任务到 queue
                            std::lock_guard<std::mutex> qlock(queue_mutex);
                            QueueTask qtask;
                            qtask.type = TASK_GENERATE_THUMBNAIL_MESH;
                            qtask.index = task.item_id;
                            queue.push(qtask);
                            thumbnail_mesh_pending.insert(task.item_id);
                        }
                        return;  // 等待生成完成
                    }
                }
            }

            if (item->voxel_renderer.empty()) {
                thumbnail_queue.pop();
                return;
            }

            ensureThumbnailResources();
            if (!thumb_shader_ || !thumb_shader_->ensureGBufferProgram()) {
                return;
            }

            auto& mesh = item->voxel_renderer.getMeshRenderer();
            auto [min_b, max_b] = mesh.getLocalBounds();
            bx::Vec3 center((min_b.x + max_b.x) * 0.5f,
                            (min_b.y + max_b.y) * 0.5f,
                            (min_b.z + max_b.z) * 0.5f);
            float dx = max_b.x - min_b.x;
            float dy = max_b.y - min_b.y;
            float dz = max_b.z - min_b.z;
            float radius = bx::sqrt(dx * dx + dy * dy + dz * dz) * 0.5f;
            float dist = bx::max(radius * 2.5f, 1.0f);

            bx::Vec3 eye(center.x + dist, center.y + dist * 0.6f,
                         center.z + dist * 0.4f);
            float view[16];
            bx::mtxLookAt(view, eye, center);
            float proj[16];
            bx::mtxProj(proj, 45.0f, 1.0f, 0.1f, dist * 10.0f,
                        bgfx::getCaps()->homogeneousDepth);

            // 为 item 创建持久的缩略图纹理
            if (!bgfx::isValid(item->thumbnail_tex)) {
                item->thumbnail_tex = bgfx::createTexture2D(
                    128, 128, false, 1, bgfx::TextureFormat::BGRA8,
                    BGFX_TEXTURE_BLIT_DST | BGFX_SAMPLER_U_CLAMP |
                        BGFX_SAMPLER_V_CLAMP);
            }

            constexpr bgfx::ViewId kThumbView = 100;
            bgfx::setViewFrameBuffer(kThumbView, thumb_fb_);
            bgfx::setViewClear(kThumbView, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                               0x333333FF, 1.0f, 0);
            bgfx::setViewRect(kThumbView, 0, 0, 128, 128);
            bgfx::setViewTransform(kThumbView, view, proj);

            float identity[16];
            bx::mtxIdentity(identity);
            mesh.renderGBuffer(identity, *thumb_shader_);
            bgfx::touch(kThumbView);

            // blit 到 item 的持久纹理
            constexpr bgfx::ViewId kBlitView = 101;
            bgfx::blit(kBlitView, item->thumbnail_tex, 0, 0, thumb_color_tex_);

            task.stage = ThumbnailTask::WAIT;
            task.wait_frames = 1;
            break;
        }
        case ThumbnailTask::WAIT: {
            if (task.wait_frames > 0) {
                task.wait_frames--;
            }
            if (task.wait_frames <= 0) {
                task.stage = ThumbnailTask::DONE;
            }
            break;
        }
        case ThumbnailTask::DONE: {
            thumbnail_queue.pop();
            break;
        }
    }
}
}  // namespace sinriv::ui::render
