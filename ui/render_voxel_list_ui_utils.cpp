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
#include <iostream>
#include <variant>
#include "kigstudio/cgal/poisson_reconstruction.h"
#include "kigstudio/cgal/poisson_utils.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/utils/locale.h"
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {

EditResult edit_float_stepper(const char* label, float& value, float step) {
    EditResult result;
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
        result.value_changed = true;
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80.0f);
    snprintf(buf, sizeof(buf), "##%s", label);
    ImGui::DragFloat(buf, &value, step, 0.0f, 0.0f, "%.2f");
    if (ImGui::IsItemActivated())
        result.activated = true;
    if (ImGui::IsItemDeactivatedAfterEdit())
        result.deactivated_after_edit = true;
    ImGui::SameLine();
    if (ImGui::Button("+", ImVec2(button_size, 0))) {
        value += step;
        result.value_changed = true;
    }
    ImGui::PopID();
    return result;
}

EditResult edit_vec3_stepper(const char* label,
                             vec3f& value,
                             float step,
                             bool normalize) {
    EditResult result;
    const char* axis_names[] = {"X", "Y", "Z"};
    float values[3] = {value.x, value.y, value.z};
    ImGui::Text("%s", label);
    char buf[128];
    for (int i = 0; i < 3; ++i) {
        // if (i > 0) {
        //     ImGui::SameLine();
        // }
        snprintf(buf, sizeof(buf), "%s##%s", axis_names[i], label);
        auto r = edit_float_stepper(buf, values[i], step);
        result.activated |= r.activated;
        result.deactivated_after_edit |= r.deactivated_after_edit;
        result.value_changed |= r.value_changed;
    }
    value = {values[0], values[1], values[2]};
    if (normalize) {
        value = sinriv::kigstudio::voxel::collision::safeNormalize(value);
    }
    return result;
}
EditResult edit_local_position_stepper(const char* label,
                                       vec3f& value,
                                       float step,
                                       bool normalize,
                                       bool show_label) {
    EditResult result;
    const char* axis_names[] = {"X", "Y", "Z"};
    float values[3] = {value.x, -value.y, value.z};
    if (show_label) {
        ImGui::Text("%s", label);
    }
    char buf[128];
    for (int i = 0; i < 3; ++i) {
        snprintf(buf, sizeof(buf), "%s##%s", axis_names[i], label);
        auto r = edit_float_stepper(buf, values[i], step);
        result.activated |= r.activated;
        result.deactivated_after_edit |= r.deactivated_after_edit;
        result.value_changed |= r.value_changed;
    }
    value = {values[0], -values[1], values[2]};
    if (normalize) {
        value = sinriv::kigstudio::voxel::collision::safeNormalize(value);
    }
    return result;
}

EditResult edit_transform_controls(Transform& transform) {
    EditResult result;
    vec3f pos = transform.getPosition();
    auto r1 = edit_vec3_stepper(get_locale_cstr("label.position"), pos);
    result.activated |= r1.activated;
    result.deactivated_after_edit |= r1.deactivated_after_edit;
    result.value_changed |= r1.value_changed;
    transform.setPosition(pos);

    vec3f euler_rad = transform.getRotationEuler();
    vec3f euler_deg = {bx::toDeg(euler_rad.x), bx::toDeg(euler_rad.y),
                       bx::toDeg(euler_rad.z)};
    auto r2 =
        edit_vec3_stepper(get_locale_cstr("label.rotation_deg"), euler_deg);
    result.activated |= r2.activated;
    result.deactivated_after_edit |= r2.deactivated_after_edit;
    result.value_changed |= r2.value_changed;
    transform.setRotationEuler({bx::toRad(euler_deg.x), bx::toRad(euler_deg.y),
                                bx::toRad(euler_deg.z)});
    return result;
}

const char* geometry_type_name(const GeometryInstance& instance) {
    return std::visit(
        [](const auto& geometry) -> const char* {
            using T = std::decay_t<decltype(geometry)>;
            if constexpr (std::is_same_v<T, Sphere>) {
                return get_locale_cstr("shape.sphere");
            } else if constexpr (std::is_same_v<T, Cylinder>) {
                return get_locale_cstr("shape.cylinder");
            } else if constexpr (std::is_same_v<T, Capsule>) {
                return get_locale_cstr("shape.capsule");
            } else if constexpr (std::is_same_v<T, Box>) {
                return get_locale_cstr("shape.box");
            } else {
                return get_locale_cstr("shape.unknown");
            }
        },
        instance.geometry);
}

EditResult edit_geometry_shape(GeometryInstance& instance) {
    EditResult result;
    std::visit(
        [&result](auto& geometry) {
            using T = std::decay_t<decltype(geometry)>;
            if constexpr (std::is_same_v<T, Sphere>) {
                auto r1 = edit_local_position_stepper(
                    get_locale_cstr("label.center"), geometry.center);
                result.activated |= r1.activated;
                result.deactivated_after_edit |= r1.deactivated_after_edit;
                result.value_changed |= r1.value_changed;
                auto r2 = edit_float_stepper(get_locale_cstr("label.radius"),
                                             geometry.radius);
                result.activated |= r2.activated;
                result.deactivated_after_edit |= r2.deactivated_after_edit;
                result.value_changed |= r2.value_changed;
                geometry.radius = bx::max(0.0f, geometry.radius);
            } else if constexpr (std::is_same_v<T, Cylinder>) {
                auto r1 = edit_local_position_stepper(
                    get_locale_cstr("label.start"), geometry.start);
                result.activated |= r1.activated;
                result.deactivated_after_edit |= r1.deactivated_after_edit;
                result.value_changed |= r1.value_changed;
                auto r2 = edit_local_position_stepper(
                    get_locale_cstr("label.end"), geometry.end);
                result.activated |= r2.activated;
                result.deactivated_after_edit |= r2.deactivated_after_edit;
                result.value_changed |= r2.value_changed;
                auto r3 = edit_float_stepper(get_locale_cstr("label.radius"),
                                             geometry.radius);
                result.activated |= r3.activated;
                result.deactivated_after_edit |= r3.deactivated_after_edit;
                result.value_changed |= r3.value_changed;
                geometry.radius = bx::max(0.0f, geometry.radius);
            } else if constexpr (std::is_same_v<T, Capsule>) {
                auto r1 = edit_local_position_stepper(
                    get_locale_cstr("label.start"), geometry.start);
                result.activated |= r1.activated;
                result.deactivated_after_edit |= r1.deactivated_after_edit;
                result.value_changed |= r1.value_changed;
                auto r2 = edit_local_position_stepper(
                    get_locale_cstr("label.end"), geometry.end);
                result.activated |= r2.activated;
                result.deactivated_after_edit |= r2.deactivated_after_edit;
                result.value_changed |= r2.value_changed;
                auto r3 = edit_float_stepper(get_locale_cstr("label.radius"),
                                             geometry.radius);
                result.activated |= r3.activated;
                result.deactivated_after_edit |= r3.deactivated_after_edit;
                result.value_changed |= r3.value_changed;
                geometry.radius = bx::max(0.0f, geometry.radius);
            } else if constexpr (std::is_same_v<T, Box>) {
                auto r = edit_vec3_stepper(get_locale_cstr("label.half_extent"),
                                           geometry.half_extent);
                result.activated |= r.activated;
                result.deactivated_after_edit |= r.deactivated_after_edit;
                result.value_changed |= r.value_changed;
                geometry.half_extent.x = bx::max(0.0f, geometry.half_extent.x);
                geometry.half_extent.y = bx::max(0.0f, geometry.half_extent.y);
                geometry.half_extent.z = bx::max(0.0f, geometry.half_extent.z);
            }
        },
        instance.geometry);
    return result;
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

#ifdef _WIN32
// ANSI → UTF-16
std::wstring ansi_to_utf16(const char* str) {
    if (!str)
        return {};

    int len = MultiByteToWideChar(CP_ACP, 0, str, -1, nullptr, 0);
    if (len <= 1)
        return {};

    std::wstring w(len - 1, 0);

    MultiByteToWideChar(CP_ACP, 0, str, -1, &w[0], len);

    return w;
}

// UTF-16 → UTF-8
std::string utf16_to_utf8(const std::wstring& w) {
    if (w.empty())
        return {};

    int len = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0,
                                  nullptr, nullptr);
    if (len <= 1)
        return {};

    std::string s(len - 1, 0);

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

std::string localize_id(const char* key, int id) {
    return get_locale_string(key) + "##" + std::to_string(id);
}

}  // namespace sinriv::ui::render