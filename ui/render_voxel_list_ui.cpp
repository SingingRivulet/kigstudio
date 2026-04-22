#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>
#include <stb/stb_truetype.h>
#include <type_traits>
#include <unordered_set>
#include <variant>
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {
namespace {

using CollisionGroup = sinriv::kigstudio::voxel::collision::CollisionGroup;
using GeometryInstance = sinriv::kigstudio::voxel::collision::GeometryInstance;
using Sphere = sinriv::kigstudio::voxel::collision::Sphere;
using Cylinder = sinriv::kigstudio::voxel::collision::Cylinder;
using Capsule = sinriv::kigstudio::voxel::collision::Capsule;
using OBB = sinriv::kigstudio::voxel::collision::OBB;
using Transform = sinriv::kigstudio::voxel::collision::Transform;
using vec3f = sinriv::kigstudio::voxel::collision::vec3f;
using Plane = sinriv::kigstudio::Plane<float>;

void edit_float_stepper(const char* label, float& value, float step = 0.5f) {
    const float button_size = ImGui::GetFrameHeight();
    ImGui::PushID(label);
    if (ImGui::Button("-", ImVec2(button_size, 0))) {
        value -= step;
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80.0f);
    ImGui::DragFloat(label, &value, step, 0.0f, 0.0f, "%.2f");
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
        if (i > 0) {
            ImGui::SameLine();
        }
        snprintf(buf, sizeof(buf), "%s##%s", axis_names[i], label);
        edit_float_stepper(buf, values[i], step);
    }
    value = {values[0], values[1], values[2]};
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
    transform.setRotationEuler(
        {bx::toRad(euler_deg.x), bx::toRad(euler_deg.y), bx::toRad(euler_deg.z)});
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
            } else if constexpr (std::is_same_v<T, OBB>) {
                return "OBB";
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
                edit_vec3_stepper("Center", geometry.center);
                edit_float_stepper("Radius", geometry.radius);
                geometry.radius = bx::max(0.0f, geometry.radius);
            } else if constexpr (std::is_same_v<T, Cylinder>) {
                edit_vec3_stepper("Start", geometry.start);
                edit_vec3_stepper("End", geometry.end);
                edit_float_stepper("Radius", geometry.radius);
                geometry.radius = bx::max(0.0f, geometry.radius);
            } else if constexpr (std::is_same_v<T, Capsule>) {
                edit_vec3_stepper("Start", geometry.start);
                edit_vec3_stepper("End", geometry.end);
                edit_float_stepper("Radius", geometry.radius);
                geometry.radius = bx::max(0.0f, geometry.radius);
            } else if constexpr (std::is_same_v<T, OBB>) {
                edit_vec3_stepper("Center", geometry.center);
                edit_vec3_stepper("Half Extent", geometry.half_extent);
                geometry.half_extent.x = bx::max(0.0f, geometry.half_extent.x);
                geometry.half_extent.y = bx::max(0.0f, geometry.half_extent.y);
                geometry.half_extent.z = bx::max(0.0f, geometry.half_extent.z);
                edit_vec3_stepper("Axis X", geometry.axis_x, 0.1f, true);
                edit_vec3_stepper("Axis Y", geometry.axis_y, 0.1f, true);
                edit_vec3_stepper("Axis Z", geometry.axis_z, 0.1f, true);
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
            group.add(Cylinder{{0.0f, 0.0f, -10.0f},
                               {0.0f, 0.0f, 10.0f},
                               6.0f});
            break;
        case 2:
            group.add(Capsule{{-10.0f, 0.0f, 0.0f},
                              {10.0f, 0.0f, 0.0f},
                              6.0f});
            break;
        case 3:
            group.add(OBB{{0.0f, 0.0f, 0.0f},
                          {8.0f, 8.0f, 8.0f},
                          {1.0f, 0.0f, 0.0f},
                          {0.0f, 1.0f, 0.0f},
                          {0.0f, 0.0f, 1.0f}});
            break;
    }
}

std::tuple<vec3f, vec3f, vec3f> build_plane_three_points(const Plane& plane) {
    auto [point, normal] = plane.getPointNormalForm();
    vec3f tangent = normal.cross(vec3f(0.0f, 0.0f, 1.0f));
    if (tangent.length() < 1e-6f) {
        tangent = normal.cross(vec3f(0.0f, 1.0f, 0.0f));
    }
    tangent = sinriv::kigstudio::voxel::collision::safeNormalize(tangent);
    vec3f bitangent =
        sinriv::kigstudio::voxel::collision::safeNormalize(normal.cross(tangent));
    return {point, point + tangent, point + bitangent};
}

void edit_plane_from_point_normal(Plane& plane,
                                  vec3f& point,
                                  vec3f& normal,
                                  std::string& error_message) {
    edit_vec3_stepper("Point", point);
    edit_vec3_stepper("Normal", normal, 0.1f);
    if (ImGui::Button("Apply point-normal")) {
        try {
            plane = Plane(point, normal);
            error_message.clear();
            ImGui::CloseCurrentPopup();
        } catch (const std::exception& e) {
            error_message = e.what();
        }
    }
}

void edit_plane_from_three_points(Plane& plane,
                                  vec3f& p1,
                                  vec3f& p2,
                                  vec3f& p3,
                                  std::string& error_message) {
    edit_vec3_stepper("P1", p1);
    edit_vec3_stepper("P2", p2);
    edit_vec3_stepper("P3", p3);
    if (ImGui::Button("Apply three-point")) {
        const vec3f v1 = p2 - p1;
        const vec3f v2 = p3 - p1;
        vec3f normal = v1.cross(v2);
        if (normal.length() < 1e-6f) {
            error_message = "Three points are collinear.";
        } else {
            normal =
                sinriv::kigstudio::voxel::collision::safeNormalize(normal);
            plane = Plane(p1, normal);
            error_message.clear();
            ImGui::CloseCurrentPopup();
        }
    }
}

}  // namespace

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

    ImGui::SetNextWindowPos(ImVec2((float)window_width, (float)menu_height),
                            ImGuiCond_Once, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(360, 620), ImGuiCond_Once);
    if (ImGui::Begin("Collision Members")) {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it == items.end()) {
            ImGui::TextUnformatted("No active item.");
        } else {
            RenderVoxelItem& item = *item_it->second;
            static int plane_editor_item_id = -1;
            static int plane_input_mode = 0;
            static vec3f plane_point = {0.0f, 0.0f, 0.0f};
            static vec3f plane_normal = {0.0f, 1.0f, 0.0f};
            static vec3f plane_p1 = {0.0f, 0.0f, 0.0f};
            static vec3f plane_p2 = {1.0f, 0.0f, 0.0f};
            static vec3f plane_p3 = {0.0f, 1.0f, 0.0f};
            static std::string plane_error_message;

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

            if (ImGui::CollapsingHeader("segment plane",
                                        ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::Text("A: %.4f", item.plane.A);
                ImGui::Text("B: %.4f", item.plane.B);
                ImGui::Text("C: %.4f", item.plane.C);
                ImGui::Text("D: %.4f", item.plane.D);
                if (ImGui::Button("Edit plane")) {
                    plane_editor_item_id = item.id;
                    auto [point, normal] = item.plane.getPointNormalForm();
                    auto [p1, p2, p3] = build_plane_three_points(item.plane);
                    plane_point = point;
                    plane_normal = normal;
                    plane_p1 = p1;
                    plane_p2 = p2;
                    plane_p3 = p3;
                    plane_error_message.clear();
                    ImGui::OpenPopup("Edit Segment Plane");
                }

                if (ImGui::BeginPopupModal("Edit Segment Plane", nullptr,
                                           ImGuiWindowFlags_AlwaysAutoResize)) {
                    if (plane_editor_item_id != item.id) {
                        ImGui::TextUnformatted(
                            "Plane editor is bound to another item.");
                    } else {
                        const char* plane_modes[] = {"Point-Normal",
                                                     "Three-Point"};
                        ImGui::Combo("input mode", &plane_input_mode,
                                     plane_modes, IM_ARRAYSIZE(plane_modes));
                        ImGui::Separator();

                        if (plane_input_mode == 0) {
                            edit_plane_from_point_normal(item.plane, plane_point,
                                                         plane_normal,
                                                         plane_error_message);
                        } else {
                            edit_plane_from_three_points(item.plane, plane_p1,
                                                         plane_p2, plane_p3,
                                                         plane_error_message);
                        }

                        if (!plane_error_message.empty()) {
                            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f),
                                               "%s",
                                               plane_error_message.c_str());
                        }
                    }

                    ImGui::SameLine();
                    if (ImGui::Button("Cancel")) {
                        plane_error_message.clear();
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndPopup();
                }
            }

            if (ImGui::CollapsingHeader("collision root",
                                        ImGuiTreeNodeFlags_DefaultOpen)) {
                edit_transform_controls(item.collision_group.transform);
            }

            if (ImGui::CollapsingHeader("collision group",
                                        ImGuiTreeNodeFlags_DefaultOpen)) {
                static int new_geometry_type = 0;
                const char* geometry_types[] = {"Sphere", "Cylinder",
                                                "Capsule", "OBB"};

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
                    if (ImGui::CollapsingHeader(header.c_str(),
                                                ImGuiTreeNodeFlags_DefaultOpen)) {
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
