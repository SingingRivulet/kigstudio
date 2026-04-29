#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>
#include <stb/stb_truetype.h>
#include <type_traits>
#include <unordered_set>
#include <variant>
#ifdef _WIN32
#include <windows.h>
#endif
#include "kigstudio/utils/vec3.h"
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {
namespace {

#ifdef _WIN32
// ANSI → UTF-16
std::wstring ansi_to_utf16(const char* str) {
    if (!str)
        return {};

    int len = MultiByteToWideChar(CP_ACP, 0, str, -1, nullptr, 0);

    std::wstring w(len, 0);

    MultiByteToWideChar(CP_ACP, 0, str, -1, &w[0], len);

    return w;
}

// UTF-16 → UTF-8
std::string utf16_to_utf8(const std::wstring& w) {
    if (w.empty())
        return {};

    int len = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0,
                                  nullptr, nullptr);

    std::string s(len, 0);

    WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, &s[0], len, nullptr,
                        nullptr);

    return s;
}

// ANSI → UTF-8
std::string ansi_to_utf8(const char* str) {
    return utf16_to_utf8(ansi_to_utf16(str));
}
#endif

std::string tinyfd_path_to_utf8(const char* path) {
#ifdef _WIN32
    return ansi_to_utf8(path);
#else
    return path ? std::string(path) : std::string();
#endif
}

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
    // 截断label中的##
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
                                 bool normalize = false,
                                 bool show_label = true) {
    const char* axis_names[] = {"X", "Y", "Z"};
    float values[3] = {value.x, -value.y, value.z};
    if (show_label) {
        ImGui::Text("%s", label);
    }
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
        menu_height = static_cast<int>(
            ImGui::CalcWindowNextAutoFitSize(ImGui::GetCurrentWindow()).y);
        ImGui::SetWindowSize(ImVec2((float)window_width, (float)menu_height));
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(0.0f, (float)window_height),
                            ImGuiCond_Always, ImVec2(0.0f, 1.0f));
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

        ImGui::SetWindowSize(
            ImVec2((float)window_width, (float)item_status_height));
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
                const char* file = tinyfd_openFileDialog("Open STL", "", 0,
                                                         NULL, "STL file", 0);
                if (file) {
                    stl_file_path = std::string(file);
                }
            }
            if (!stl_file_path.empty()) {
                ImGui::Text("Selected file: %s", tinyfd_path_to_utf8(stl_file_path.c_str()).c_str());
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
            ImGui::DragFloat("##Voxel Size", &voxel_size, 0.1f, 0.0f, 0.0f,
                             "%.4f");
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

void RenderVoxelList::render_collision_body_editor(RenderVoxelItem& item) {
    if (ImGui::CollapsingHeader("collision root",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        edit_transform_controls(item.collision_group.transform);
    }

    if (ImGui::CollapsingHeader("collision group",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        static int new_geometry_type = 0;
        const char* geometry_types[] = {"Sphere", "Cylinder", "Capsule", "Box"};

        ImGui::SetNextItemWidth(140.0f);
        ImGui::Combo("new shape", &new_geometry_type, geometry_types,
                     IM_ARRAYSIZE(geometry_types));
        ImGui::SameLine();
        if (ImGui::Button("Add shape")) {
            add_collision_geometry(item.collision_group, new_geometry_type);
        }

        auto& geometries = item.collision_group.geometries();
        int remove_index = -1;
        for (int member_idx = 0;
             member_idx < static_cast<int>(geometries.size()); ++member_idx) {
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
void RenderVoxelList::render_concave_cone_editor(RenderVoxelItem& item) {
    enum class PickMode { None, Append, Replace, InsertBefore, InsertAfter };

    static PickMode pick_mode = PickMode::None;
    static int pick_index = -1;

    // ===== apex =====
    if (ImGui::CollapsingHeader("concave cone apex",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        edit_local_position_stepper("apex", item.concave_cone.apex);
    }

    // ===== error info =====
    if (!item.err_info.empty()) {
        ImGui::TextColored(ImVec4(1, 0, 0, 1), "%s", item.err_info.c_str());
    }

    // ===== vertices =====
    if (ImGui::CollapsingHeader("concave cone vertices",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        // --- append 模式 ---
        if (pick_mode == PickMode::Append) {
            if (ImGui::Button("add vertex (click in scene...)")) {
                pick_mode = PickMode::None;
                pick_index = -1;
            }
        } else {
            if (ImGui::Button("add vertex")) {
                pick_mode = PickMode::Append;
                pick_index = -1;
            }
        }

        // --- clear ---
        if (ImGui::Button("clear vertices")) {
            item.concave_cone.base_vertices.clear();
            item.err_info.clear();
            item.concave_cone.check(item.err_info);
        }

        // ===== vertex list =====
        int erase_index = -1;

        for (int i = 0; i < (int)item.concave_cone.base_vertices.size(); ++i) {
            auto& v = item.concave_cone.base_vertices[i];

            char label[64];
            snprintf(label, sizeof(label), "vertex %d##%d", i, i);

            if (ImGui::CollapsingHeader(label)) {
                // ImGui::Text("pos: %.3f %.3f %.3f", v.x, v.y, v.z);

                edit_local_position_stepper(
                    ("edit##" + std::to_string(i)).c_str(), v, 0.1f, false,
                    false);

                // --- delete ---
                if (ImGui::Button(("delete##" + std::to_string(i)).c_str())) {
                    erase_index = i;
                }

                ImGui::SameLine();

                // --- replace ---
                if (pick_mode == PickMode::Replace && pick_index == i) {
                    if (ImGui::Button(
                            ("replace (...)##" + std::to_string(i)).c_str())) {
                        pick_mode = PickMode::None;
                        pick_index = -1;
                    }
                } else {
                    if (ImGui::Button(
                            ("replace##" + std::to_string(i)).c_str())) {
                        pick_mode = PickMode::Replace;
                        pick_index = i;
                    }
                }

                ImGui::SameLine();

                // --- insert after ---
                if (pick_mode == PickMode::InsertAfter && pick_index == i) {
                    if (ImGui::Button(
                            ("insert after (...)##" + std::to_string(i))
                                .c_str())) {
                        pick_mode = PickMode::None;
                        pick_index = -1;
                    }
                } else {
                    if (ImGui::Button(
                            ("insert after##" + std::to_string(i)).c_str())) {
                        pick_mode = PickMode::InsertAfter;
                        pick_index = i;
                    }
                }

                ImGui::SameLine();

                // --- insert before ---
                if (pick_mode == PickMode::InsertBefore && pick_index == i) {
                    if (ImGui::Button(
                            ("insert before (...)##" + std::to_string(i))
                                .c_str())) {
                        pick_mode = PickMode::None;
                        pick_index = -1;
                    }
                } else {
                    if (ImGui::Button(
                            ("insert before##" + std::to_string(i)).c_str())) {
                        pick_mode = PickMode::InsertBefore;
                        pick_index = i;
                    }
                }
            }
        }

        // ===== 删除处理 =====
        if (erase_index >= 0 &&
            erase_index < (int)item.concave_cone.base_vertices.size()) {
            item.concave_cone.base_vertices.erase(
                item.concave_cone.base_vertices.begin() + erase_index);

            item.err_info.clear();
            item.concave_cone.check(item.err_info);
        }

        // ===== picking 执行（统一入口）=====
        if (pick_mode != PickMode::None && mouse_world_pos_valid &&
            mouse_world_pos_picked)  // ⚠ 必须是点击瞬间
        {
            auto& verts = item.concave_cone.base_vertices;

            switch (pick_mode) {
                case PickMode::Append:
                    verts.push_back(mouse_world_pos);
                    break;

                case PickMode::Replace:
                    if (pick_index >= 0 && pick_index < (int)verts.size())
                        verts[pick_index] = mouse_world_pos;
                    pick_mode = PickMode::None;
                    break;

                case PickMode::InsertAfter:
                    if (pick_index >= 0 && pick_index < (int)verts.size())
                        verts.insert(verts.begin() + pick_index + 1,
                                     mouse_world_pos);
                    pick_mode = PickMode::None;
                    break;

                case PickMode::InsertBefore:
                    if (pick_index >= 0 && pick_index < (int)verts.size())
                        verts.insert(verts.begin() + pick_index,
                                     mouse_world_pos);
                    pick_mode = PickMode::None;
                    break;

                default:
                    break;
            }

            item.err_info.clear();
            item.concave_cone.check(item.err_info);

            // 一次性消费
            pick_index = -1;
        }

    } else {
        // 折叠时重置
        pick_mode = PickMode::None;
        pick_index = -1;
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

            const char* segment_mode_names[] = {"Collision", "Plane",
                                                "Concave Cone"};
            const enum RenderVoxelItem::segment_mode segment_modes[] = {
                RenderVoxelItem::COLLISION, RenderVoxelItem::PLANE,
                RenderVoxelItem::CONCAVE_CONE};
            int current_segment_mode = segment_modes[(int)item.segment_mode];
            if (ImGui::Combo("segment mode", &current_segment_mode,
                             segment_mode_names,
                             IM_ARRAYSIZE(segment_mode_names))) {
                item.segment_mode = segment_modes[current_segment_mode];
            }
            if (item.segment_mode == RenderVoxelItem::PLANE) {
                render_plane_editor(item);
            } else if (item.segment_mode == RenderVoxelItem::COLLISION) {
                render_collision_body_editor(item);
            } else if (item.segment_mode == RenderVoxelItem::CONCAVE_CONE) {
                render_concave_cone_editor(item);
            }
        }
    }
    ImGui::End();
}

}  // namespace sinriv::ui::render
