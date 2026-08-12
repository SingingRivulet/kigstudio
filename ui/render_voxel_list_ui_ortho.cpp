#include <SDL.h>
#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>
#include <stb/stb_truetype.h>
#include <cstring>
#include <vector>
#include "kigstudio/sdf/sdf_mesh.h"
// stb_image implementation is in render_voxel_list_ui_addons.cpp
#include "../../dep/bgfx.cmake/bimg/3rdparty/stb/stb_image.h"
#include <sys/stat.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <type_traits>
#include <unordered_set>
#include <variant>
#include "../../dep/bgfx.cmake/bimg/3rdparty/stb/stb_image_write.h"
#ifdef _WIN32
#include <windows.h>
#endif
#include "kigstudio/agent/agent_handlers.h"
#include "kigstudio/cgal/mesh_simplification.h"
#include "kigstudio/sdf/sdf_chain_joint.h"
#include "kigstudio/utils/locale.h"
#include "kigstudio/utils/triangle.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/voxel/voxel2mesh.h"
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {

// Helper: load stb_image from a UTF-8 path on Windows.
// On Windows, fopen doesn't accept UTF-8 paths by default, so we read the
// file ourselves via the wide-char API and use stbi_load_from_memory.
#ifdef _WIN32
static unsigned char* stbi_load_utf8(const char* utf8_path,
                                     int* w,
                                     int* h,
                                     int* comp,
                                     int req_comp) {
    int wlen = MultiByteToWideChar(CP_UTF8, 0, utf8_path, -1, nullptr, 0);
    if (wlen <= 0)
        return nullptr;
    std::wstring wpath(static_cast<size_t>(wlen), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8_path, -1, &wpath[0], wlen);
    // Remove trailing null from std::wstring length calculation
    if (!wpath.empty() && wpath.back() == L'\0')
        wpath.pop_back();

    HANDLE hFile =
        CreateFileW(wpath.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
                    OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (hFile == INVALID_HANDLE_VALUE)
        return nullptr;

    DWORD size = GetFileSize(hFile, nullptr);
    if (size == INVALID_FILE_SIZE || size == 0) {
        CloseHandle(hFile);
        return nullptr;
    }

    std::vector<unsigned char> buffer(size);
    DWORD read = 0;
    BOOL ok = ReadFile(hFile, buffer.data(), size, &read, nullptr);
    CloseHandle(hFile);
    if (!ok || read != size)
        return nullptr;

    return stbi_load_from_memory(buffer.data(), static_cast<int>(size), w, h,
                                 comp, req_comp);
}
#endif

/// Extrapolate a new guide point from existing 3D curve when raycast misses
/// the model.  Builds a curvature-preserving tangent from the last 2+ guide
/// points, then finds the closest approach between the camera ray and the
/// extrapolated line.  Returns true and sets out_pt (on the ray).
static bool extrapolate_guide_along_ray(const vec3f& ro, const vec3f& rd,
                                        const std::vector<vec3f>& existing,
                                        vec3f& out_pt, float vp_size) {
    const size_t n = existing.size();
    if (n < 2) return false;

    const vec3f& P_last = existing.back();

    // ---- Build curvature-preserving tangent direction ----
    vec3f T;
    if (n == 2) {
        const vec3f& A = existing[n - 2];
        T = vec3f{P_last.x - A.x, P_last.y - A.y, P_last.z - A.z};
    } else {
        // Blend last 2-3 normalised segment directions, weighted toward recent
        size_t seg_count = std::min(n - 1, size_t(3));
        float w_sum = 0.0f;
        T = vec3f{0, 0, 0};
        for (size_t i = 0; i < seg_count; ++i) {
            size_t idx = n - seg_count - 1 + i;
            const vec3f& A = existing[idx];
            const vec3f& B = existing[idx + 1];
            vec3f dir{B.x - A.x, B.y - A.y, B.z - A.z};
            float len = std::sqrt(dir.x * dir.x + dir.y * dir.y +
                                  dir.z * dir.z);
            if (len < 1e-8f) continue;
            dir.x /= len; dir.y /= len; dir.z /= len;
            float w = static_cast<float>(i + 1);  // linear ramp
            T.x += dir.x * w; T.y += dir.y * w; T.z += dir.z * w;
            w_sum += w;
        }
        if (w_sum > 0.0f) {
            T.x /= w_sum; T.y /= w_sum; T.z /= w_sum;
        } else {
            const vec3f& A = existing[n - 2];
            T = vec3f{P_last.x - A.x, P_last.y - A.y, P_last.z - A.z};
        }
    }

    // Normalise
    float t_len = std::sqrt(T.x * T.x + T.y * T.y + T.z * T.z);
    if (t_len < 1e-8f) return false;
    T.x /= t_len; T.y /= t_len; T.z /= t_len;

    // ---- Two-line closest-point (ray vs extrapolation line) ----
    float b = rd.x * T.x + rd.y * T.y + rd.z * T.z;
    if (std::abs(b) > 0.9999f) return false;  // near-parallel

    vec3f v{ro.x - P_last.x, ro.y - P_last.y, ro.z - P_last.z};
    float d = -(v.x * rd.x + v.y * rd.y + v.z * rd.z);
    float e = -(v.x * T.x + v.y * T.y + v.z * T.z);
    float inv_det = 1.0f / (b * b - 1.0f);
    float s = (d - b * e) * inv_det;
    float t = (b * d - e) * inv_det;

    // Clamp to forward direction (s≥0: in front of image plane;
    // t≥0: forward along the extrapolated curve)
    if (s < 0.0f) s = 0.0f;
    if (t < 0.0f) t = 0.0f;

    // Constrained closest-point pair
    vec3f c_ray{ro.x + s * rd.x, ro.y + s * rd.y, ro.z + s * rd.z};
    vec3f c_line{P_last.x + t * T.x, P_last.y + t * T.y,
                 P_last.z + t * T.z};
    float dist = std::sqrt((c_ray.x - c_line.x) * (c_ray.x - c_line.x) +
                           (c_ray.y - c_line.y) * (c_ray.y - c_line.y) +
                           (c_ray.z - c_line.z) * (c_ray.z - c_line.z));

    if (dist > vp_size * 0.5f) return false;

    out_pt = c_ray;
    return true;
}

// Raycast from ortho camera through image pixel (px,py) against base model
// triangles. Returns true and sets world_pos to the nearest intersection point.
static bool ortho_raycast(const OrthoProjectionState& state,
                          int px,
                          int py,
                          vec3f& world_pos) {
    if (state._base_triangles.empty())
        return false;

    int res = state.render_resolution;
    float half = state.viewport_size * 0.5f;
    float u = (static_cast<float>(px) / res - 0.5f);
    float v = (0.5f - static_cast<float>(py) / res);

    // World-space point on the image plane
    vec3f plane_pt = {
        state._center.x + state._cam_right.x * u * state.viewport_size +
            state._cam_up.x * v * state.viewport_size,
        state._center.y + state._cam_right.y * u * state.viewport_size +
            state._cam_up.y * v * state.viewport_size,
        state._center.z + state._cam_right.z * u * state.viewport_size +
            state._cam_up.z * v * state.viewport_size};

    // Ray direction: go from the face side toward/through center.
    // Uses +projection_dir (not -projection_dir) so the ray origin
    // lands on the face side of the model.
    vec3f ray_dir = {state.projection_dir.x, state.projection_dir.y,
                     state.projection_dir.z};
    float rl = std::sqrt(ray_dir.x * ray_dir.x + ray_dir.y * ray_dir.y +
                         ray_dir.z * ray_dir.z);
    if (rl < 1e-8f)
        return false;
    ray_dir.x /= rl;
    ray_dir.y /= rl;
    ray_dir.z /= rl;

    // Put ray origin on the face side (opposite to _cam_pos).
    // Negating cam_off flips the origin across the center plane.
    float cam_off = (plane_pt.x - state._cam_pos.x) * ray_dir.x +
                    (plane_pt.y - state._cam_pos.y) * ray_dir.y +
                    (plane_pt.z - state._cam_pos.z) * ray_dir.z;
    plane_pt.x -= ray_dir.x * (-cam_off);
    plane_pt.y -= ray_dir.y * (-cam_off);
    plane_pt.z -= ray_dir.z * (-cam_off);

    float best_t = 1e30f;
    bool hit = false;
    for (const auto& tri : state._base_triangles) {
        float t;
        if (ray_triangle_intersect(plane_pt, ray_dir, std::get<0>(tri),
                                   std::get<1>(tri), std::get<2>(tri), t)) {
            if (t < best_t) {
                best_t = t;
                hit = true;
            }
        }
    }

    if (hit) {
        world_pos = {plane_pt.x + ray_dir.x * best_t,
                     plane_pt.y + ray_dir.y * best_t,
                     plane_pt.z + ray_dir.z * best_t};
    }
    return hit;
}



void RenderVoxelList::render_ortho_setup_window() {
    if (!show_ortho_setup_window)
        return;

    ImGui::SetNextWindowSize(ImVec2(420, 320), ImGuiCond_Once);
    if (!ImGui::Begin(get_locale_cstr("window.ortho_projection_setup"),
                      &show_ortho_setup_window)) {
        ImGui::End();
        return;
    }

    std::lock_guard<std::mutex> lock(locker);
    auto item_it = items.find(render_id);
    if (item_it == items.end() || item_it->second->source_type != 2) {
        ImGui::TextUnformatted(get_locale_cstr("label.no_active_item"));
        ImGui::End();
        return;
    }

    RenderVoxelItem& item = *item_it->second;

    // Helper: apply overlay state for the current view (six-view or
    // pick-point). Reloads the image texture so the reference image updates
    // immediately when the user switches views, matching the "Render" button
    // behaviour.
    auto apply_overlay_for_view = [&]() {
        if (ortho_state.vector_mode == 0) {
            int vi = ortho_state.six_view_index;
            const auto& saved = item.ortho_overlay[vi];
            ortho_state.overlay_image_path = saved.image_path;
            ortho_state.overlay_img_width = saved.img_width;
            ortho_state.overlay_img_height = saved.img_height;
            ortho_state.overlay_enabled = saved.enabled;
            ortho_state.overlay_offset = ImVec2(saved.offset_x, saved.offset_y);
            ortho_state.overlay_scale_x = saved.scale_x;
            ortho_state.overlay_scale_y = saved.scale_y;
            ortho_state.blend_ratio = saved.blend_ratio;
            ortho_state.overlay_locked = saved.locked;

            // Reload the image texture if there's a saved path
            if (!saved.image_path.empty()) {
                int w, h, comp;
                unsigned char* data =
#ifdef _WIN32
                    stbi_load_utf8(saved.image_path.c_str(), &w, &h, &comp, 4);
#else
                    stbi_load(saved.image_path.c_str(), &w, &h, &comp, 4);
#endif
                if (data) {
                    if (bgfx::isValid(ortho_state.overlay_tex))
                        bgfx::destroy(ortho_state.overlay_tex);
                    ortho_state.overlay_tex = bgfx::createTexture2D(
                        static_cast<uint16_t>(w), static_cast<uint16_t>(h),
                        false, 1, bgfx::TextureFormat::RGBA8,
                        BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP);
                    bgfx::updateTexture2D(ortho_state.overlay_tex, 0, 0, 0, 0,
                                          static_cast<uint16_t>(w),
                                          static_cast<uint16_t>(h),
                                          bgfx::copy(data, w * h * 4));
                    // Keep CPU-side copy for API blending
                    size_t cpu_sz = static_cast<size_t>(w) * h * 4;
                    overlay_cpu_rgba_.resize(cpu_sz);
                    memcpy(overlay_cpu_rgba_.data(), data, cpu_sz);
                    overlay_cpu_w_ = w;
                    overlay_cpu_h_ = h;
                    stbi_image_free(data);
                    ortho_state.overlay_img_width = w;
                    ortho_state.overlay_img_height = h;
                }
            } else {
                // No reference image for this view — destroy old texture
                if (bgfx::isValid(ortho_state.overlay_tex))
                    bgfx::destroy(ortho_state.overlay_tex);
                ortho_state.overlay_tex = BGFX_INVALID_HANDLE;
                overlay_cpu_rgba_.clear();
                overlay_cpu_w_ = 0;
                overlay_cpu_h_ = 0;
            }
        } else {
            // Pick-point mode: clear any overlay from a previous six-view
            // session
            if (bgfx::isValid(ortho_state.overlay_tex))
                bgfx::destroy(ortho_state.overlay_tex);
            ortho_state.overlay_tex = BGFX_INVALID_HANDLE;
            ortho_state.overlay_image_path.clear();
            ortho_state.overlay_enabled = false;
            ortho_state.overlay_img_width = 0;
            ortho_state.overlay_img_height = 0;
            ortho_state.overlay_offset = ImVec2(0, 0);
            ortho_state.overlay_scale_x = 1.0f;
            ortho_state.overlay_scale_y = 1.0f;
            ortho_state.blend_ratio = 0.5f;
            ortho_state.overlay_locked = false;
            overlay_cpu_rgba_.clear();
            overlay_cpu_w_ = 0;
            overlay_cpu_h_ = 0;
        }
    };

    // Check base model
    if (item.addon_base_node_id < 0) {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s",
                           get_locale_cstr("label.ortho_no_base_model"));
        ImGui::End();
        return;
    }

    // ---- Initialize viewport size & render resolution from
    //     persisted per-node values, or auto-calculate on first open ----
    if (!ortho_state.viewport_size_defaulted) {
        // Viewport: use saved value if present, otherwise auto-calc
        if (item.ortho_viewport_size > 0.0f) {
            ortho_state.viewport_size = item.ortho_viewport_size;
        } else {
            auto base_it = items.find(item.addon_base_node_id);
            if (base_it != items.end()) {
                const auto& base = *base_it->second;
                float max_dist2 = 0.0f;
                auto check_vertex = [&](const vec3f& v) {
                    vec3f rel = {v.x - item.addon_center_point.x,
                                 v.y - item.addon_center_point.y,
                                 v.z - item.addon_center_point.z};
                    float d2 = rel.x * rel.x + rel.y * rel.y + rel.z * rel.z;
                    if (d2 > max_dist2)
                        max_dist2 = d2;
                };
                if (!base.source_triangles.empty()) {
                    for (const auto& tri : base.source_triangles) {
                        check_vertex(std::get<0>(tri));
                        check_vertex(std::get<1>(tri));
                        check_vertex(std::get<2>(tri));
                    }
                } else if (!base.cached_mesh.empty()) {
                    for (const auto& tri_n : base.cached_mesh) {
                        const auto& tri = std::get<0>(tri_n);
                        check_vertex(std::get<0>(tri));
                        check_vertex(std::get<1>(tri));
                        check_vertex(std::get<2>(tri));
                    }
                }
                if (max_dist2 > 0.0f) {
                    float sphere_r = std::sqrt(max_dist2) * 1.05f;
                    ortho_state.viewport_size = sphere_r * 2.2f;
                }
            }
        }
        // Render resolution: use saved value if present, else default
        if (item.ortho_render_resolution > 0)
            ortho_state.render_resolution = item.ortho_render_resolution;

        // Sync projection_dir with the default six_view_index on first open.
        // Otherwise the default projection_dir {0,1,0} (top-down) is used
        // while the combo shows "Front" (index 0), causing a mismatch.
        ortho_state.projection_dir =
            six_view_direction(ortho_state.six_view_index,
                               item.hair_front_reference, item.hair_north_pole);

        ortho_state.viewport_size_defaulted = true;
    }

    // ---- Direction mode selection ----
    ImGui::TextUnformatted(get_locale_cstr("label.vector_mode"));
    ImGui::SameLine();
    const char* mode_names[] = {
        get_locale_cstr("label.vector_mode_six"),
        get_locale_cstr("label.vector_mode_pick"),
    };
    if (ImGui::Combo("##vector_mode", &ortho_state.vector_mode, mode_names,
                     2)) {
        apply_overlay_for_view();
    }

    ImGui::Separator();

    if (ortho_state.vector_mode == 0) {
        // ---- Six view selection ----
        const char* view_names[] = {
            get_locale_cstr("label.six_view_front"),
            get_locale_cstr("label.six_view_back"),
            get_locale_cstr("label.six_view_left"),
            get_locale_cstr("label.six_view_right"),
            get_locale_cstr("label.six_view_top"),
            get_locale_cstr("label.six_view_bottom"),
        };
        if (ImGui::Combo(get_locale_cstr("label.projection_direction"),
                         &ortho_state.six_view_index, view_names, 6)) {
            ortho_state.projection_dir = six_view_direction(
                ortho_state.six_view_index, item.hair_front_reference,
                item.hair_north_pole);
            // Direction changed: the off-screen texture must be re-rendered
            ortho_state.render_dirty = true;
            // Immediately apply the reference image for the new view
            apply_overlay_for_view();
        }
    } else {
        // ---- Pick point on model ----
        bool was_picking = ortho_state.is_picking_point;
        if (ImGui::Button(get_locale_cstr("action.pick_projection"))) {
            ortho_state.is_picking_point = true;
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s",
                              get_locale_cstr("action.picking_direction"));

        if (ortho_state.is_picking_point && !was_picking) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.08f, 1.0f), "%s",
                               get_locale_cstr("action.picking_direction"));
        }
    }

    // Show current direction
    ImGui::Text("%s: (%.2f, %.2f, %.2f)",
                get_locale_cstr("label.projection_direction"),
                ortho_state.projection_dir.x, ortho_state.projection_dir.y,
                ortho_state.projection_dir.z);

    ImGui::Separator();

    // ---- Viewport size ----
    if (ImGui::DragFloat(get_locale_cstr("label.viewport_size"),
                         &ortho_state.viewport_size, 1.0f, 10.0f, 500.0f,
                         "%.1f")) {
        ortho_state.render_dirty = true;
        item.ortho_viewport_size = ortho_state.viewport_size;
    }

    // ---- Render resolution ----
    int res = ortho_state.render_resolution;
    const int res_options[] = {512, 1024, 2048, 4096};
    const char* res_names[] = {"512", "1024", "2048", "4096"};
    int res_idx = 2;
    for (int i = 0; i < 4; ++i) {
        if (res == res_options[i]) {
            res_idx = i;
            break;
        }
    }
    if (ImGui::Combo(get_locale_cstr("label.render_resolution"), &res_idx,
                     res_names, 4)) {
        ortho_state.render_resolution = res_options[res_idx];
        item.ortho_render_resolution = ortho_state.render_resolution;
        ortho_state.render_dirty = true;
    }

    ImGui::Separator();

    // ---- Render button ----
    if (ImGui::Button(get_locale_cstr("action.ortho_render"),
                      ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
        auto base_it = items.find(item.addon_base_node_id);
        if (base_it == items.end()) {
            show_toast(get_locale_string("label.ortho_no_base_model"), 3000.0f);
        } else {
            RenderVoxelItem& base_item = *base_it->second;
            if (base_item.source_triangles.empty() &&
                base_item.cached_mesh.empty()) {
                show_toast(get_locale_string("label.ortho_no_mesh_data"),
                           3000.0f);
            } else {
                perform_ortho_render(item, base_item);
                ortho_state.active = true;
                ortho_state.edit_window_open = true;
                show_ortho_setup_window = false;
                show_ortho_edit_window = true;

                // Overlay: only six-view presets carry a saved reference image.
                // Picked-vector mode starts without any overlay; manual loads
                // in that mode are temporary and never persisted.
                if (ortho_state.vector_mode == 0) {
                    int vi = ortho_state.six_view_index;
                    const auto& saved = item.ortho_overlay[vi];
                    ortho_state.overlay_image_path = saved.image_path;
                    ortho_state.overlay_img_width = saved.img_width;
                    ortho_state.overlay_img_height = saved.img_height;
                    ortho_state.overlay_enabled = saved.enabled;
                    ortho_state.overlay_offset =
                        ImVec2(saved.offset_x, saved.offset_y);
                    ortho_state.overlay_scale_x = saved.scale_x;
                    ortho_state.overlay_scale_y = saved.scale_y;
                    ortho_state.blend_ratio = saved.blend_ratio;
                    ortho_state.overlay_locked = saved.locked;

                    // Reload the image texture if there's a saved path
                    if (!saved.image_path.empty()) {
                        int w, h, comp;
                        unsigned char* data =
#ifdef _WIN32
                            stbi_load_utf8(saved.image_path.c_str(), &w, &h,
                                           &comp, 4);
#else
                            stbi_load(saved.image_path.c_str(), &w, &h, &comp,
                                      4);
#endif
                        if (data) {
                            if (bgfx::isValid(ortho_state.overlay_tex))
                                bgfx::destroy(ortho_state.overlay_tex);
                            ortho_state.overlay_tex = bgfx::createTexture2D(
                                static_cast<uint16_t>(w),
                                static_cast<uint16_t>(h), false, 1,
                                bgfx::TextureFormat::RGBA8,
                                BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP);
                            bgfx::updateTexture2D(ortho_state.overlay_tex, 0, 0,
                                                  0, 0,
                                                  static_cast<uint16_t>(w),
                                                  static_cast<uint16_t>(h),
                                                  bgfx::copy(data, w * h * 4));
                            // Keep CPU-side copy for API blending
                            size_t cpu_sz = static_cast<size_t>(w) * h * 4;
                            overlay_cpu_rgba_.resize(cpu_sz);
                            memcpy(overlay_cpu_rgba_.data(), data, cpu_sz);
                            overlay_cpu_w_ = w;
                            overlay_cpu_h_ = h;
                            stbi_image_free(data);
                            ortho_state.overlay_img_width = w;
                            ortho_state.overlay_img_height = h;
                        }
                    }
                } else {
                    // Picked-vector mode: discard any overlay left over from a
                    // previous six-view session.
                    if (bgfx::isValid(ortho_state.overlay_tex))
                        bgfx::destroy(ortho_state.overlay_tex);
                    ortho_state.overlay_tex = BGFX_INVALID_HANDLE;
                    ortho_state.overlay_image_path.clear();
                    ortho_state.overlay_enabled = false;
                    ortho_state.overlay_img_width = 0;
                    ortho_state.overlay_img_height = 0;
                    ortho_state.overlay_offset = ImVec2(0, 0);
                    ortho_state.overlay_scale_x = 1.0f;
                    ortho_state.overlay_scale_y = 1.0f;
                    ortho_state.blend_ratio = 0.5f;
                    ortho_state.overlay_locked = false;
                    overlay_cpu_rgba_.clear();
                    overlay_cpu_w_ = 0;
                    overlay_cpu_h_ = 0;
                }
            }
        }
    }

    ImGui::End();
}

void RenderVoxelList::render_ortho_edit_window() {
    if (!show_ortho_edit_window || !ortho_state.edit_window_open)
        return;

    ImGui::SetNextWindowSize(ImVec2(600, 700), ImGuiCond_Once);
    bool window_open = true;
    if (!ImGui::Begin(get_locale_cstr("window.ortho_edit"), &window_open)) {
        ImGui::End();
        return;
    }

    // Helper: sync current overlay state back to the owning item (six-view
    // only)
    auto sync_overlay_to_item = [&]() {
        if (ortho_state.vector_mode != 0)
            return;
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it == items.end() || it->second->source_type != 2)
            return;
        auto& ol = it->second->ortho_overlay[ortho_state.six_view_index];
        ol.image_path = ortho_state.overlay_image_path;
        ol.img_width = ortho_state.overlay_img_width;
        ol.img_height = ortho_state.overlay_img_height;
        ol.enabled = ortho_state.overlay_enabled;
        ol.offset_x = ortho_state.overlay_offset.x;
        ol.offset_y = ortho_state.overlay_offset.y;
        ol.scale_x = ortho_state.overlay_scale_x;
        ol.scale_y = ortho_state.overlay_scale_y;
        ol.blend_ratio = ortho_state.blend_ratio;
        ol.locked = ortho_state.overlay_locked;
    };

    if (!window_open) {
        sync_overlay_to_item();
        show_ortho_edit_window = false;
        ortho_state.edit_window_open = false;
        ortho_state.active = false;
        destroy_ortho_resources();
        ImGui::End();
        return;
    }

    // ---- Top toolbar: Load reference image ----
    if (ortho_state.view_tex_ready) {
        ImGui::SameLine();
        ImGui::TextDisabled("res=%d", ortho_state.render_resolution);
    }
    // Hover coordinate display (moved to top for visibility)
    if (ortho_state.mouse_in_image) {
        ImGui::SameLine();
        if (ortho_state.is_hovering_model) {
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f),
                               " (%d,%d) \xE2\x86\x92 (%.1f, %.1f, %.1f)",
                               ortho_state.hovered_px, ortho_state.hovered_py,
                               ortho_state.hovered_world_pos.x,
                               ortho_state.hovered_world_pos.y,
                               ortho_state.hovered_world_pos.z);
        } else {
            ImGui::TextDisabled(" (%d,%d)", ortho_state.hovered_px,
                                ortho_state.hovered_py);
        }
    }
    if (ImGui::Button(get_locale_cstr("action.load_reference_image"))) {
        const char* filters[] = {"*.png", "*.jpg", "*.jpeg", "*.bmp"};
        const char* path = tinyfd_openFileDialog(
            get_locale_cstr("action.load_reference_image"), "", 4, filters,
            get_locale_cstr("action.load_reference_image"), 0);
        if (path) {
            std::string utf8_path = tinyfd_path_to_utf8(path);
            int w, h, comp;
            unsigned char* data =
#ifdef _WIN32
                stbi_load_utf8(utf8_path.c_str(), &w, &h, &comp, 4);
#else
                stbi_load(utf8_path.c_str(), &w, &h, &comp, 4);
#endif
            if (data) {
                if (bgfx::isValid(ortho_state.overlay_tex))
                    bgfx::destroy(ortho_state.overlay_tex);
                ortho_state.overlay_tex = bgfx::createTexture2D(
                    static_cast<uint16_t>(w), static_cast<uint16_t>(h), false,
                    1, bgfx::TextureFormat::RGBA8,
                    BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP);
                bgfx::updateTexture2D(ortho_state.overlay_tex, 0, 0, 0, 0,
                                      static_cast<uint16_t>(w),
                                      static_cast<uint16_t>(h),
                                      bgfx::copy(data, w * h * 4));
                // Keep CPU-side copy for API blending
                size_t data_sz = static_cast<size_t>(w) * h * 4;
                overlay_cpu_rgba_.resize(data_sz);
                memcpy(overlay_cpu_rgba_.data(), data, data_sz);
                overlay_cpu_w_ = w;
                overlay_cpu_h_ = h;
                stbi_image_free(data);
                ortho_state.overlay_image_path = utf8_path;
                ortho_state.overlay_img_width = w;
                ortho_state.overlay_img_height = h;
                ortho_state.overlay_enabled = true;
                ortho_state.overlay_offset = ImVec2(0, 0);
                ortho_state.overlay_scale_x = 1.0f;
                ortho_state.overlay_scale_y = 1.0f;
                sync_overlay_to_item();
            } else {
                show_toast("Failed to load image: " + utf8_path, 3000.0f);
            }
        }
    }

    // Overlay enable checkbox (only shown when image loaded)
    bool overlay_changed = false;
    if (bgfx::isValid(ortho_state.overlay_tex)) {
        ImGui::SameLine();
        if (ImGui::Checkbox(get_locale_cstr("label.enable_overlay"),
                            &ortho_state.overlay_enabled))
            overlay_changed = true;

        // Blend slider
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        if (ImGui::SliderFloat(get_locale_cstr("label.blend_ratio"),
                               &ortho_state.blend_ratio, 0.0f, 1.0f))
            overlay_changed = true;

        // Scale X/Y sliders (independent axis scaling)
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        if (ImGui::DragFloat("##scale_x", &ortho_state.overlay_scale_x, 0.01f,
                             0.1f, 10.0f, "SX:%.2f"))
            overlay_changed = true;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        if (ImGui::DragFloat("##scale_y", &ortho_state.overlay_scale_y, 0.01f,
                             0.1f, 10.0f, "SY:%.2f"))
            overlay_changed = true;

        // Lock button (toggle overlay drag/resize)
        ImGui::SameLine();
        if (ImGui::Button(ortho_state.overlay_locked
                              ? get_locale_cstr("label.overlay_unlock")
                              : get_locale_cstr("label.overlay_lock"))) {
            ortho_state.overlay_locked = !ortho_state.overlay_locked;
            overlay_changed = true;
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", get_locale_cstr("tooltip.overlay_lock"));
    }
    if (overlay_changed)
        sync_overlay_to_item();

    // ImGui::Separator();

    // // ---- API status (shared with main Agent API) ----
    // if (agent_server_ptr && agent_server_ptr->is_running()) {
    //     ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.3f, 1.0f),
    //                        "● API:%d", agent_server_ptr->port());
    //     if (ImGui::IsItemHovered())
    //         ImGui::SetTooltip("Ortho endpoints:
    //         http://127.0.0.1:%d/api/v1/ortho",
    //                           agent_server_ptr->port());
    // }

    // Keep API server caches in sync (ortho render, overlay params, state JSON)
    if (agent_server_ptr && agent_server_ptr->is_running())
        update_api_server_caches();

    // Auto-trigger GPU readback when a new render is available,
    // pushing pixels to the API cache (no disk files).
    if (ortho_state.api_render_dirty && ortho_state.ai_export_stage == 0) {
        ortho_state.api_render_dirty = false;
        ortho_state.ai_export_pending = true;
        ortho_state.ai_export_stage = 1;
    }

    // Process AI export readback (if pending)
    process_ai_export();

    ImGui::Separator();

    // ---- Handle re-render requests ----
    // Triggered by: depth-colour toggle, base-model changes, etc.
    bool need_render = false;
    if (ortho_state.render_dirty && ortho_state.view_tex_ready) {
        need_render = true;
    }
    // Also detect base-model geometry changes (e.g. after voxel edit)
    if (!need_render && ortho_state.view_tex_ready &&
        ortho_state._base_triangle_count > 0) {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it != items.end() && item_it->second->source_type == 2) {
            auto& item = *item_it->second;
            if (item.addon_base_node_id >= 0) {
                auto base_it = items.find(item.addon_base_node_id);
                if (base_it != items.end()) {
                    auto& base = *base_it->second;
                    size_t cur_count = base.cached_mesh.empty()
                                           ? base.source_triangles.size()
                                           : base.cached_mesh.size();
                    if (cur_count != ortho_state._base_triangle_count) {
                        ortho_state.render_dirty = true;
                        need_render = true;
                    }
                }
            }
        }
    }
    if (need_render) {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it != items.end() && item_it->second->source_type == 2) {
            auto& item = *item_it->second;
            if (item.addon_base_node_id >= 0) {
                auto base_it = items.find(item.addon_base_node_id);
                if (base_it != items.end()) {
                    perform_ortho_render(item, *base_it->second);
                }
            }
        }
    }

    // ---- Canvas display area ----
    if (!ortho_state.coord_map_ready) {
        ImGui::TextUnformatted("No render data. Open Setup to render.");
        ImGui::End();
        return;
    }

    // Compute display size from window width (stable, avoids ContentRegionAvail
    // fluctuations that can cause flicker from scrollbar appear/disappear).
    float avail_w = ImGui::GetWindowWidth() - 30.0f;
    float display_size = std::max(200.0f, avail_w);  // min 200px, no upper cap

    // Overlay params (offset, scale) are stored in a fixed 600px reference
    // space. They are NEVER auto-modified by window resize — only by user
    // interaction or explicit API calls.  At display time we convert:
    //   screen  = ref × (display_size / 600)
    //   render  = ref × (render_resolution / 600)
    constexpr float kRefDisplaySize = 600.0f;
    float ref_to_display = display_size / kRefDisplaySize;
    float display_to_ref = kRefDisplaySize / std::max(display_size, 1.0f);

    // Track actual display size for API state reporting
    ortho_state.canvas_display_size = display_size;

    int res = ortho_state.render_resolution;

    // Display the rendered view image, or dark fallback if not ready yet
    if (ortho_state.view_tex_ready && bgfx::isValid(ortho_state.view_tex)) {
        ImGui::Image(ImGui::toId(ortho_state.view_tex, 0, 0),
                     ImVec2(display_size, display_size));
    } else {
        // Dark canvas fallback (before GPU render completes, or on re-render)
        ImVec2 fb_cursor = ImGui::GetCursorScreenPos();
        ImDrawList* fb_dl = ImGui::GetWindowDrawList();
        fb_dl->AddRectFilled(
            fb_cursor,
            ImVec2(fb_cursor.x + display_size, fb_cursor.y + display_size),
            ImGui::ColorConvertFloat4ToU32(ImVec4(0.18f, 0.18f, 0.20f, 1.0f)));
        fb_dl->AddRect(
            fb_cursor,
            ImVec2(fb_cursor.x + display_size, fb_cursor.y + display_size),
            ImGui::ColorConvertFloat4ToU32(ImVec4(0.35f, 0.35f, 0.38f, 1.0f)));
        ImGui::Dummy(ImVec2(display_size, display_size));
    }

    // Get the actual screen-space rect of the displayed image/fallback
    ImVec2 img_cursor = ImGui::GetItemRectMin();
    ImVec2 img_end = ImGui::GetItemRectMax();
    // Recompute display_size from the actual rendered item
    display_size = img_end.x - img_cursor.x;

    // Invisible button over the entire image canvas.  It captures mouse
    // events so ImGui won't see "void" clicks as window-drag starts.
    // The overlay already has its own InvisibleButton when unlocked.
    // All guide-point / overlay-interaction code below uses raw
    // ImGui::IsMouse* checks, which are unaffected by InvisibleButton.
    ImGui::SetCursorScreenPos(img_cursor);
    ImGui::InvisibleButton("##canvas_interact",
                           ImVec2(display_size, display_size));

    ImDrawList* dl = ImGui::GetWindowDrawList();

    // ---- Overlay image rendering ----
    // Use ImGui::Image (same rendering path as the base model image) so
    // alpha blending via the tint colour composes correctly.
    // Save/restore cursor so GetItemRectMin/Max still refers to the base
    // model image for coordinate mapping below.
    ImVec2 prev_cursor_screen = ImGui::GetCursorScreenPos();
    if (bgfx::isValid(ortho_state.overlay_tex) && ortho_state.overlay_enabled) {
        float overlay_w = ortho_state.overlay_img_width *
                          ortho_state.overlay_scale_x * ref_to_display;
        float overlay_h = ortho_state.overlay_img_height *
                          ortho_state.overlay_scale_y * ref_to_display;
        ImVec2 overlay_pos = ImVec2(
            img_cursor.x + ortho_state.overlay_offset.x * ref_to_display,
            img_cursor.y + ortho_state.overlay_offset.y * ref_to_display);

        ImGui::SetCursorScreenPos(overlay_pos);
        ImGui::Image(ortho_state.overlay_tex, ImVec2(overlay_w, overlay_h),
                     ImVec2(0, 0), ImVec2(1, 1),
                     ImVec4(1.0f, 1.0f, 1.0f, ortho_state.blend_ratio));

        // Invisible button covering the entire overlay to capture left-clicks
        // and prevent the parent window from being dragged when the user
        // interacts with the overlay image.
        if (!ortho_state.overlay_locked) {
            ImGui::SetCursorScreenPos(overlay_pos);
            ImGui::InvisibleButton("##overlay_interact",
                                   ImVec2(overlay_w, overlay_h));
        }
    }
    ImGui::SetCursorScreenPos(prev_cursor_screen);

    // ---- Strand preview overlays ----
    // Project a world-space point onto the 2D image using the camera basis
    // that was computed in perform_ortho_render().
    auto project_world_to_image = [&](const vec3f& wp) -> ImVec2 {
        vec3f rel = {wp.x - ortho_state._center.x, wp.y - ortho_state._center.y,
                     wp.z - ortho_state._center.z};
        float h = ortho_state.viewport_size * 0.5f;
        float rx = (rel.x * ortho_state._cam_right.x +
                    rel.y * ortho_state._cam_right.y +
                    rel.z * ortho_state._cam_right.z) /
                   h;
        float ry =
            (rel.x * ortho_state._cam_up.x + rel.y * ortho_state._cam_up.y +
             rel.z * ortho_state._cam_up.z) /
            h;
        return ImVec2(img_cursor.x + (rx * 0.5f + 0.5f) * display_size,
                      img_cursor.y + (0.5f - ry * 0.5f) * display_size);
    };

    // --- Occlusion cache: avoid recomputing per-strand occlusion every frame
    // --- Invalidate when camera, model, or guide points change.
    {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it != items.end()) {
            auto& item = *item_it->second;

            // Build a simple hash of the camera + model state
            size_t state_hash = 0;
            auto hash_combine = [&](float v) {
                state_hash ^= std::hash<float>{}(v) + 0x9e3779b9 +
                              (state_hash << 6) + (state_hash >> 2);
            };
            hash_combine(ortho_state._center.x);
            hash_combine(ortho_state._center.y);
            hash_combine(ortho_state._center.z);
            hash_combine(ortho_state._cam_pos.x);
            hash_combine(ortho_state._cam_pos.y);
            hash_combine(ortho_state._cam_pos.z);
            hash_combine(ortho_state.projection_dir.x);
            hash_combine(ortho_state.projection_dir.y);
            hash_combine(ortho_state.projection_dir.z);
            hash_combine(ortho_state._cam_right.x);
            hash_combine(ortho_state._cam_right.y);
            hash_combine(ortho_state._cam_right.z);
            hash_combine(ortho_state._cam_up.x);
            hash_combine(ortho_state._cam_up.y);
            hash_combine(ortho_state._cam_up.z);
            hash_combine(ortho_state.viewport_size);
            state_hash ^=
                std::hash<size_t>{}(ortho_state._base_triangles.size());

            // Also hash guide point positions for all strands
            for (const auto& s : item.hair_strands) {
                hash_combine(static_cast<float>(s.guide_points.size()));
                for (const auto& gp : s.guide_points) {
                    hash_combine(gp.x);
                    hash_combine(gp.y);
                    hash_combine(gp.z);
                }
            }

            // Recompute occlusion cache if state changed
            if (state_hash != item._ortho_occlusion_hash) {
                item._ortho_occlusion_hash = state_hash;
                item._ortho_strand_occluded.resize(item.hair_strands.size());
                item._ortho_point_occluded.clear();
                item._ortho_point_occluded.resize(item.hair_strands.size());

                // DEBUG: occlusion recompute trigger
                static int occ_recompute_count = 0;
                occ_recompute_count++;
                bool log_this = (occ_recompute_count <= 5);
                if (log_this) {
                    fprintf(
                        stderr,
                        "[OCCL] recompute #%d: hash=%zu strands=%zu tris=%zu\n",
                        occ_recompute_count, state_hash,
                        item.hair_strands.size(),
                        ortho_state._base_triangles.size());
                    fprintf(stderr,
                            "[OCCL]   proj_dir=(%.4f,%.4f,%.4f) "
                            "center=(%.2f,%.2f,%.2f) cam_pos=(%.1f,%.1f,%.1f) "
                            "vp=%.2f\n",
                            ortho_state.projection_dir.x,
                            ortho_state.projection_dir.y,
                            ortho_state.projection_dir.z, ortho_state._center.x,
                            ortho_state._center.y, ortho_state._center.z,
                            ortho_state._cam_pos.x, ortho_state._cam_pos.y,
                            ortho_state._cam_pos.z, ortho_state.viewport_size);
                }

                // projection_dir = center→camera (outward).
                // Occlusion cam_plane_pt = center - ray_dir_n * kCamDist
                // must be on the viewer side, so we use +projection_dir.
                float dlen = std::sqrt(ortho_state.projection_dir.x *
                                           ortho_state.projection_dir.x +
                                       ortho_state.projection_dir.y *
                                           ortho_state.projection_dir.y +
                                       ortho_state.projection_dir.z *
                                           ortho_state.projection_dir.z);
                vec3f ray_dir_n = {ortho_state.projection_dir.x / dlen,
                                   ortho_state.projection_dir.y / dlen,
                                   ortho_state.projection_dir.z / dlen};

                // Reference plane on the viewer side (opposite to ray_dir_n
                // from center). This ensures t_wp > 0 for all visible points.
                const float kCamDist = 1000.0f;
                vec3f cam_plane_pt = {
                    ortho_state._center.x - ray_dir_n.x * kCamDist,
                    ortho_state._center.y - ray_dir_n.y * kCamDist,
                    ortho_state._center.z - ray_dir_n.z * kCamDist};

                const float kOccTolerance = 0.15f;
                const float half_vp = ortho_state.viewport_size * 0.5f;

                if (log_this) {
                    fprintf(stderr,
                            "[OCCL]   ray_dir_n=(%.4f,%.4f,%.4f) "
                            "cam_plane_pt=(%.1f,%.1f,%.1f)\n",
                            ray_dir_n.x, ray_dir_n.y, ray_dir_n.z,
                            cam_plane_pt.x, cam_plane_pt.y, cam_plane_pt.z);
                }

                for (size_t si = 0; si < item.hair_strands.size(); ++si) {
                    const auto& strand = item.hair_strands[si];
                    if (strand.guide_points.size() < 2) {
                        item._ortho_strand_occluded[si] = false;
                        continue;
                    }

                    auto& pt_occ = item._ortho_point_occluded[si];
                    pt_occ.resize(strand.guide_points.size(), false);

                    bool all_occluded = true;
                    int behind_cam = 0, outside_vp = 0, no_hit = 0,
                        occluded_cnt = 0, visible_cnt = 0;
                    for (size_t pi = 0; pi < strand.guide_points.size(); ++pi) {
                        const auto& wp = strand.guide_points[pi];

                        // Per-point ray origin: project wp onto the camera
                        // plane. This ensures the ray passes through wp.
                        float t_wp = (wp.x - cam_plane_pt.x) * ray_dir_n.x +
                                     (wp.y - cam_plane_pt.y) * ray_dir_n.y +
                                     (wp.z - cam_plane_pt.z) * ray_dir_n.z;
                        vec3f ray_origin = {wp.x - t_wp * ray_dir_n.x,
                                            wp.y - t_wp * ray_dir_n.y,
                                            wp.z - t_wp * ray_dir_n.z};

                        // t_wp is the (positive) distance from cam plane to wp
                        if (t_wp <= 0.0f) {
                            all_occluded = false;
                            behind_cam++;
                            continue;
                        }

                        // Viewport check: project wp onto near plane (through
                        // _center, perpendicular to ray_dir_n) and verify the
                        // projection lands inside the viewport rectangle.
                        float t_center =
                            (wp.x - ortho_state._center.x) * ray_dir_n.x +
                            (wp.y - ortho_state._center.y) * ray_dir_n.y +
                            (wp.z - ortho_state._center.z) * ray_dir_n.z;
                        vec3f plane_pt = {wp.x - t_center * ray_dir_n.x,
                                          wp.y - t_center * ray_dir_n.y,
                                          wp.z - t_center * ray_dir_n.z};
                        vec3f rel = {plane_pt.x - ortho_state._center.x,
                                     plane_pt.y - ortho_state._center.y,
                                     plane_pt.z - ortho_state._center.z};
                        float rx = (rel.x * ortho_state._cam_right.x +
                                    rel.y * ortho_state._cam_right.y +
                                    rel.z * ortho_state._cam_right.z);
                        float ry = (rel.x * ortho_state._cam_up.x +
                                    rel.y * ortho_state._cam_up.y +
                                    rel.z * ortho_state._cam_up.z);
                        if (std::abs(rx) > half_vp || std::abs(ry) > half_vp) {
                            all_occluded = false;
                            outside_vp++;
                            continue;
                        }

                        // Raycast from the per-point origin along look dir.
                        // Find the first model surface hit.
                        float best_t = 1e30f;
                        for (const auto& tri : ortho_state._base_triangles) {
                            float t;
                            if (ray_triangle_intersect(
                                    ray_origin, ray_dir_n, std::get<0>(tri),
                                    std::get<1>(tri), std::get<2>(tri), t)) {
                                if (t < best_t)
                                    best_t = t;
                            }
                        }
                        if (best_t >= 1e29f) {
                            no_hit++;
                        }

                        // DEBUG: log first few points of first strand
                        if (log_this && si == 0 && pi < 3) {
                            fprintf(stderr,
                                    "[OCCL]     pt[%zu] wp=(%.2f,%.2f,%.2f) "
                                    "ro=(%.1f,%.1f,%.1f) t_wp=%.4f best_t=%.4f "
                                    "=> %s\n",
                                    pi, wp.x, wp.y, wp.z, ray_origin.x,
                                    ray_origin.y, ray_origin.z, t_wp, best_t,
                                    (best_t < t_wp - kOccTolerance)
                                        ? "OCCLUDED"
                                        : "visible");
                        }

                        // Occluded if a model surface lies between camera and
                        // wp
                        bool occluded = (best_t < t_wp - kOccTolerance);
                        pt_occ[pi] = occluded;
                        if (!occluded) {
                            all_occluded = false;
                            visible_cnt++;
                        } else {
                            occluded_cnt++;
                        }
                    }

                    if (log_this && si == 0) {
                        fprintf(stderr,
                                "[OCCL]   strand[0] summary: behind_cam=%d "
                                "outside_vp=%d "
                                "no_hit=%d occluded=%d visible=%d => "
                                "all_occluded=%d\n",
                                behind_cam, outside_vp, no_hit, occluded_cnt,
                                visible_cnt, all_occluded ? 1 : 0);
                    }

                    item._ortho_strand_occluded[si] = all_occluded;
                }

                // DEBUG: summary of all strands
                if (log_this) {
                    int total_occ = 0;
                    for (size_t si = 0; si < item._ortho_strand_occluded.size();
                         ++si)
                        if (item._ortho_strand_occluded[si])
                            total_occ++;
                    fprintf(stderr,
                            "[OCCL] TOTAL: %d/%zu strands fully occluded\n",
                            total_occ, item._ortho_strand_occluded.size());
                }
            }
        }
    }

    if (ortho_state.show_guide_curves) {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it != items.end()) {
            auto& item = *item_it->second;
            for (size_t si = 0; si < item.hair_strands.size(); ++si) {
                const auto& strand = item.hair_strands[si];
                if (strand.guide_points.size() < 2)
                    continue;

                // Use cached occlusion result
                if (si < item._ortho_strand_occluded.size() &&
                    item._ortho_strand_occluded[si])
                    continue;  // Hide fully occluded strand

                bool is_active = (item.guide_curve_drawing_active &&
                                  item.active_guide_draw_strand == strand.uuid);
                bool is_hovered = !item.hovered_strand_uuid.empty() &&
                                  item.hovered_strand_uuid == strand.uuid;
                ImU32 color = (is_active || is_hovered)
                                  ? ImGui::ColorConvertFloat4ToU32(
                                        ImVec4(1.0f, 0.2f, 0.2f, 1.0f))
                                  : ImGui::ColorConvertFloat4ToU32(
                                        ImVec4(1.0f, 1.0f, 1.0f, 0.7f));

                auto sampled = sample_bezier_guide_curve(
                    strand.guide_points,
                    std::max(strand.guide_samples_per_segment, 1));
                for (size_t pi = 0; pi + 1 < sampled.size(); ++pi) {
                    ImVec2 a = project_world_to_image(sampled[pi]);
                    ImVec2 b = project_world_to_image(sampled[pi + 1]);
                    dl->AddLine(a, b, color, 1.5f);
                }

                ImU32 marker_color = (is_active || is_hovered)
                                         ? ImGui::ColorConvertFloat4ToU32(
                                               ImVec4(1.0f, 0.2f, 0.2f, 1.0f))
                                         : ImGui::ColorConvertFloat4ToU32(
                                               ImVec4(0.8f, 0.8f, 0.8f, 0.5f));
                const auto& pt_occ = (si < item._ortho_point_occluded.size())
                                         ? item._ortho_point_occluded[si]
                                         : std::vector<bool>{};
                for (size_t pi = 0; pi < strand.guide_points.size(); ++pi) {
                    // Use cached per-point occlusion
                    if (pi < pt_occ.size() && pt_occ[pi])
                        continue;
                    ImVec2 pimg =
                        project_world_to_image(strand.guide_points[pi]);
                    dl->AddCircleFilled(pimg, 3.0f, marker_color);
                }
            }
        }
    }

    if (ortho_state.show_width_vectors) {
        std::lock_guard<std::mutex> lock(locker);
        auto item_it = items.find(render_id);
        if (item_it != items.end()) {
            ImU32 green =
                ImGui::ColorConvertFloat4ToU32(ImVec4(0.2f, 0.9f, 0.3f, 1.0f));
            for (const auto& strand : item_it->second->hair_strands) {
                if (strand.guide_points.size() < 2)
                    continue;
                for (const auto& wp : strand.width_points) {
                    if (wp.curve_id < 0.0f)
                        continue;
                    float max_id =
                        static_cast<float>(strand.guide_points.size() - 1);
                    if (wp.curve_id > max_id)
                        continue;

                    size_t seg_idx = static_cast<size_t>(wp.curve_id);
                    if (seg_idx >= strand.guide_points.size() - 1)
                        seg_idx = strand.guide_points.size() - 2;
                    float t = wp.curve_id - static_cast<float>(seg_idx);

                    const auto& gpts = strand.guide_points;
                    size_t n = gpts.size();
                    vec3f p0 = gpts[seg_idx], p3 = gpts[seg_idx + 1], p1, p2;
                    if (seg_idx == 0)
                        p1 = p0 + (p3 - p0) * (1.0f / 3.0f);
                    else
                        p1 = p0 + (p3 - gpts[seg_idx - 1]) * (1.0f / 6.0f);
                    if (seg_idx + 2 >= n)
                        p2 = p3 - (p3 - p0) * (1.0f / 3.0f);
                    else
                        p2 = p3 - (gpts[seg_idx + 2] - p0) * (1.0f / 6.0f);

                    vec3f curve_pos = bezier_eval(p0, p1, p2, p3, t);
                    vec3f end_pos = curve_pos + wp.direction * wp.scale;
                    dl->AddLine(project_world_to_image(curve_pos),
                                project_world_to_image(end_pos), green, 1.0f);
                }
            }
        }
    }

    // ---- Mouse interaction (CPU raycasting) ----
    // Raycasting is always active within the image area.  Whether clicks
    // place guide/width points depends on the overlay lock state:
    //   locked   → click always passes through to the model
    //   unlocked → click on overlay = resize/drag; click on canvas = model
    ImVec2 mouse = ImGui::GetMousePos();
    bool mouse_in_image =
        (mouse.x >= img_cursor.x && mouse.x < img_cursor.x + display_size &&
         mouse.y >= img_cursor.y && mouse.y < img_cursor.y + display_size);
    ortho_state.mouse_in_image = mouse_in_image;

    // Compute overlay bounds (when visible) — converted from reference to
    // screen space
    bool overlay_visible =
        ortho_state.overlay_enabled && bgfx::isValid(ortho_state.overlay_tex);
    float overlay_w = overlay_visible
                          ? ortho_state.overlay_img_width *
                                ortho_state.overlay_scale_x * ref_to_display
                          : 0.0f;
    float overlay_h = overlay_visible
                          ? ortho_state.overlay_img_height *
                                ortho_state.overlay_scale_y * ref_to_display
                          : 0.0f;
    ImVec2 overlay_pos =
        ImVec2(img_cursor.x + ortho_state.overlay_offset.x * ref_to_display,
               img_cursor.y + ortho_state.overlay_offset.y * ref_to_display);
    ImVec2 overlay_end =
        ImVec2(overlay_pos.x + overlay_w, overlay_pos.y + overlay_h);
    bool mouse_in_overlay =
        overlay_visible &&
        (mouse.x >= overlay_pos.x && mouse.x < overlay_end.x &&
         mouse.y >= overlay_pos.y && mouse.y < overlay_end.y);

    // ---- Corner resize handles on overlay (draw list, drawn after overlay)
    // ----
    if (overlay_visible && !ortho_state.overlay_locked) {
        const float handle_r = 5.0f;
        ImU32 col_fill =
            ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 1.0f, 1.0f, 0.9f));
        ImU32 col_border =
            ImGui::ColorConvertFloat4ToU32(ImVec4(0.2f, 0.5f, 0.9f, 1.0f));

        ImVec2 corners[4] = {
            ImVec2(overlay_pos.x, overlay_pos.y),  // 0: TL
            ImVec2(overlay_end.x, overlay_pos.y),  // 1: TR
            ImVec2(overlay_pos.x, overlay_end.y),  // 2: BL
            ImVec2(overlay_end.x, overlay_end.y)   // 3: BR
        };
        for (int i = 0; i < 4; i++) {
            dl->AddCircleFilled(corners[i], handle_r, col_border);
            dl->AddCircleFilled(corners[i], handle_r - 1.5f, col_fill);
        }
    }

    // Four corner resize zones — centred on each corner so the visual
    // handle circles (radius 5 px) sit inside the hit-test area.
    const float corner_r = 12.0f;  // hit-test radius around corner centre
    int hovered_corner = -1;       // -1=none, 0=TL, 1=TR, 2=BL, 3=BR
    if (overlay_visible && !ortho_state.overlay_locked) {
        auto in_range = [](float v, float c, float r) -> bool {
            return v >= c - r && v < c + r;
        };
        // TL — centred at (overlay_pos.x, overlay_pos.y)
        if (in_range(mouse.x, overlay_pos.x, corner_r) &&
            in_range(mouse.y, overlay_pos.y, corner_r))
            hovered_corner = 0;
        // TR — centred at (overlay_end.x, overlay_pos.y)
        else if (in_range(mouse.x, overlay_end.x, corner_r) &&
                 in_range(mouse.y, overlay_pos.y, corner_r))
            hovered_corner = 1;
        // BL — centred at (overlay_pos.x, overlay_end.y)
        else if (in_range(mouse.x, overlay_pos.x, corner_r) &&
                 in_range(mouse.y, overlay_end.y, corner_r))
            hovered_corner = 2;
        // BR — centred at (overlay_end.x, overlay_end.y)
        else if (in_range(mouse.x, overlay_end.x, corner_r) &&
                 in_range(mouse.y, overlay_end.y, corner_r))
            hovered_corner = 3;
    }

    // ---- Cursor feedback (SDL directly — bgfx's imgui backend
    //     does not forward ImGui cursor requests to the OS). ----
    {
        static int s_active_cursor = -1;  // last cursor we set (-1 = none)
        int desired = -1;

        if (mouse_in_image) {
            if (hovered_corner == 0 || hovered_corner == 3)
                desired = SDL_SYSTEM_CURSOR_SIZENWSE;
            else if (hovered_corner == 1 || hovered_corner == 2)
                desired = SDL_SYSTEM_CURSOR_SIZENESW;
            else if (mouse_in_overlay && !ortho_state.overlay_locked)
                desired = SDL_SYSTEM_CURSOR_SIZEALL;
            else
                desired = SDL_SYSTEM_CURSOR_ARROW;
        }

        // Reset to arrow when mouse left the image area and we had
        // previously set a non-default cursor.
        if (desired == -1 && s_active_cursor != -1)
            desired = SDL_SYSTEM_CURSOR_ARROW;

        if (desired != -1 && desired != s_active_cursor) {
            s_active_cursor = desired;
            static SDL_Cursor* s_cached = nullptr;
            if (s_cached)
                SDL_FreeCursor(s_cached);
            s_cached =
                SDL_CreateSystemCursor(static_cast<SDL_SystemCursor>(desired));
            SDL_SetCursor(s_cached);
        }
    }

    // ---- Raycasting (always active) ----
    if (mouse_in_image && ortho_state.coord_map_ready) {
        int px =
            static_cast<int>((mouse.x - img_cursor.x) / display_size * res);
        int py =
            static_cast<int>((mouse.y - img_cursor.y) / display_size * res);
        px = std::max(0, std::min(px, res - 1));
        py = std::max(0, std::min(py, res - 1));

        vec3f hit_pos;
        bool valid = ortho_raycast(ortho_state, px, py, hit_pos);

        // Try curvature-preserving extrapolation when raycast misses
        if (!valid) {
            auto item_it = items.find(render_id);
            if (item_it != items.end()) {
                auto& item = *item_it->second;
                if (item.guide_curve_drawing_active &&
                    !item.active_guide_draw_strand.empty() &&
                    item.find_strand_by_uuid(item.active_guide_draw_strand)) {
                    auto& strand = *item.find_strand_by_uuid(
                        item.active_guide_draw_strand);
                    if (strand.guide_points.size() >= 2) {
                        // Construct camera ray from pixel
                        float u = (static_cast<float>(px) / res - 0.5f);
                        float v = (0.5f - static_cast<float>(py) / res);
                        vec3f plane_pt = {ortho_state._center.x +
                                              ortho_state._cam_right.x * u *
                                                  ortho_state.viewport_size +
                                              ortho_state._cam_up.x * v *
                                                  ortho_state.viewport_size,
                                          ortho_state._center.y +
                                              ortho_state._cam_right.y * u *
                                                  ortho_state.viewport_size +
                                              ortho_state._cam_up.y * v *
                                                  ortho_state.viewport_size,
                                          ortho_state._center.z +
                                              ortho_state._cam_right.z * u *
                                                  ortho_state.viewport_size +
                                              ortho_state._cam_up.z * v *
                                                  ortho_state.viewport_size};
                        vec3f ray_dir = {ortho_state.projection_dir.x,
                                         ortho_state.projection_dir.y,
                                         ortho_state.projection_dir.z};
                        float rl = std::sqrt(ray_dir.x * ray_dir.x +
                                             ray_dir.y * ray_dir.y +
                                             ray_dir.z * ray_dir.z);
                        if (rl > 1e-8f) {
                            ray_dir.x /= rl;
                            ray_dir.y /= rl;
                            ray_dir.z /= rl;
                            float cam_off =
                                (plane_pt.x - ortho_state._cam_pos.x) *
                                    ray_dir.x +
                                (plane_pt.y - ortho_state._cam_pos.y) *
                                    ray_dir.y +
                                (plane_pt.z - ortho_state._cam_pos.z) *
                                    ray_dir.z;
                            plane_pt.x -= ray_dir.x * (-cam_off);
                            plane_pt.y -= ray_dir.y * (-cam_off);
                            plane_pt.z -= ray_dir.z * (-cam_off);
                        }

                        vec3f extrapolated_pt;
                        if (extrapolate_guide_along_ray(
                                plane_pt, ray_dir, strand.guide_points,
                                extrapolated_pt, ortho_state.viewport_size)) {
                            valid = true;
                            hit_pos = extrapolated_pt;
                        }
                    }
                }
            }
        }

        ortho_state.is_hovering_model = valid;
        ortho_state.hovered_px = px;
        ortho_state.hovered_py = py;
        if (valid) {
            ortho_state.hovered_world_pos = hit_pos;
            mouse_world_pos_valid = true;
            mouse_world_pos = {hit_pos.x, hit_pos.y, hit_pos.z};

            // Click-through to place guide/width points:
            // Only when overlay is locked, or mouse is outside the overlay
            // (but still inside the canvas area).
            bool pass_through = !overlay_visible ||
                                ortho_state.overlay_locked || !mouse_in_overlay;
            if (pass_through && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                auto item_it = items.find(render_id);
                if (item_it != items.end()) {
                    auto& item = *item_it->second;

                    // Guide point pick: update existing point position
                    if (item.guide_point_pick_active &&
                        item.guide_point_pick_index >= 0 &&
                        !item.active_guide_draw_strand.empty()) {
                        auto* strand_ptr = item.find_strand_by_uuid(
                            item.active_guide_draw_strand);
                        if (strand_ptr &&
                            item.guide_point_pick_index <
                                static_cast<int>(
                                    strand_ptr->guide_points.size())) {
                            push_undo_now(render_id, std::nullopt,
                                          "Guide Point Edit");
                            strand_ptr->guide_points
                                [item.guide_point_pick_index] = hit_pos;
                            item.last_modified_guide_point_index =
                                item.guide_point_pick_index;
                            strand_ptr->mesh_dirty = true;
                        }
                        item.guide_point_pick_active = false;
                    } else if (item.guide_curve_drawing_active &&
                               !item.active_guide_draw_strand.empty() &&
                               item.find_strand_by_uuid(
                                   item.active_guide_draw_strand)) {
                        push_undo_now(render_id, std::nullopt,
                                      "Add Guide Point");
                        auto& strand = *item.find_strand_by_uuid(
                            item.active_guide_draw_strand);
                        strand.guide_points.push_back(hit_pos);
                        strand.mesh_dirty = true;
                    } else if (item.width_editing_active &&
                               !item.active_width_edit_strand.empty()) {
                        auto* w_strand_ptr = item.find_strand_by_uuid(
                            item.active_width_edit_strand);
                        if (w_strand_ptr) {
                            push_undo_now(render_id, std::nullopt,
                                          "Add Width Point");
                            int strand_idx = static_cast<int>(
                                w_strand_ptr - item.hair_strands.data());
                            item.add_width_point_at(strand_idx, hit_pos);
                            w_strand_ptr->mesh_dirty = true;
                        }
                    }
                }
            }
        } else {
            if (ortho_state.is_hovering_model) {
                mouse_world_pos_valid = false;
            }
            ortho_state.is_hovering_model = false;
        }
    } else if (!mouse_in_image && ortho_state.is_hovering_model) {
        ortho_state.is_hovering_model = false;
    }

    // ---- Overlay interaction (left-drag body to move, left-drag corner to
    // resize) ----
    if (overlay_visible && !ortho_state.overlay_locked && mouse_in_image) {
        bool on_corner = (hovered_corner >= 0);

        // Left-click on a corner → start resize; on body → start drag
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            if (on_corner) {
                ortho_state.resize_corner = hovered_corner;
                ortho_state.resize_start_mouse = mouse;
                ortho_state.resize_start_scale_x = ortho_state.overlay_scale_x;
                ortho_state.resize_start_scale_y = ortho_state.overlay_scale_y;
                ortho_state.resize_start_offset = ortho_state.overlay_offset;
            } else if (mouse_in_overlay) {
                ortho_state.is_dragging_overlay = true;
                ortho_state.drag_start_mouse = mouse;
                ortho_state.drag_start_offset = ortho_state.overlay_offset;
            }
        }

        // Resize (4-corner, free-drag — independent X/Y scaling)
        if (ortho_state.resize_corner >= 0) {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                int corner = ortho_state.resize_corner;
                float iw = static_cast<float>(ortho_state.overlay_img_width);
                float ih = static_cast<float>(ortho_state.overlay_img_height);
                if (iw < 1e-6f)
                    iw = 1.0f;
                if (ih < 1e-6f)
                    ih = 1.0f;

                // Mouse delta in reference space
                float dmx_ref = (mouse.x - ortho_state.resize_start_mouse.x) *
                                display_to_ref;
                float dmy_ref = (mouse.y - ortho_state.resize_start_mouse.y) *
                                display_to_ref;

                float new_sx = ortho_state.resize_start_scale_x;
                float new_sy = ortho_state.resize_start_scale_y;
                float new_ox = ortho_state.resize_start_offset.x;
                float new_oy = ortho_state.resize_start_offset.y;

                // Each corner independently controls X and Y based on
                // mouse delta relative to the opposite (anchor) corner.
                switch (corner) {
                    case 0:  // TL — anchor is BR
                        new_sx =
                            ortho_state.resize_start_scale_x - dmx_ref / iw;
                        new_sy =
                            ortho_state.resize_start_scale_y - dmy_ref / ih;
                        new_ox = ortho_state.resize_start_offset.x + dmx_ref;
                        new_oy = ortho_state.resize_start_offset.y + dmy_ref;
                        break;
                    case 1:  // TR — anchor is BL
                        new_sx =
                            ortho_state.resize_start_scale_x + dmx_ref / iw;
                        new_sy =
                            ortho_state.resize_start_scale_y - dmy_ref / ih;
                        new_oy = ortho_state.resize_start_offset.y + dmy_ref;
                        break;
                    case 2:  // BL — anchor is TR
                        new_sx =
                            ortho_state.resize_start_scale_x - dmx_ref / iw;
                        new_sy =
                            ortho_state.resize_start_scale_y + dmy_ref / ih;
                        new_ox = ortho_state.resize_start_offset.x + dmx_ref;
                        break;
                    case 3:  // BR — anchor is TL
                    default:
                        new_sx =
                            ortho_state.resize_start_scale_x + dmx_ref / iw;
                        new_sy =
                            ortho_state.resize_start_scale_y + dmy_ref / ih;
                        break;
                }

                ortho_state.overlay_scale_x =
                    std::max(0.1f, std::min(10.0f, new_sx));
                ortho_state.overlay_scale_y =
                    std::max(0.1f, std::min(10.0f, new_sy));
                ortho_state.overlay_offset.x = new_ox;
                ortho_state.overlay_offset.y = new_oy;
            } else {
                ortho_state.resize_corner = -1;
            }
        }
        // Drag (left-button on body) to move the overlay
        if (ortho_state.is_dragging_overlay) {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                // Mouse delta (screen pixels) → reference space
                ortho_state.overlay_offset.x =
                    ortho_state.drag_start_offset.x +
                    (mouse.x - ortho_state.drag_start_mouse.x) * display_to_ref;
                ortho_state.overlay_offset.y =
                    ortho_state.drag_start_offset.y +
                    (mouse.y - ortho_state.drag_start_mouse.y) * display_to_ref;
            } else {
                ortho_state.is_dragging_overlay = false;
            }
        }
    }

    // Clear overlay drag state when mouse leaves image area
    if (!mouse_in_image) {
        ortho_state.is_dragging_overlay = false;
        ortho_state.resize_corner = -1;
    }

    // ---- Footer toggles ----
    ImGui::Separator();
    ImGui::Checkbox(get_locale_cstr("label.show_guide_curves_2d"),
                    &ortho_state.show_guide_curves);
    ImGui::SameLine();
    ImGui::Checkbox(get_locale_cstr("label.show_width_vectors_2d"),
                    &ortho_state.show_width_vectors);

    ImGui::SameLine();
    if (ImGui::Checkbox(get_locale_cstr("label.export_guide_curves"),
                        &ortho_state.export_show_guide_curves))
        ortho_state.api_render_dirty = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.export_guide_curves"));

    ImGui::SameLine();
    if (!ortho_state.export_show_guide_curves)
        ImGui::BeginDisabled();
    if (ImGui::Checkbox(get_locale_cstr("label.export_color_code"),
                        &ortho_state.export_color_code_strands))
        ortho_state.api_render_dirty = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.export_color_code"));
    if (!ortho_state.export_show_guide_curves)
        ImGui::EndDisabled();

    ImGui::SameLine();
    const char* render_mode_names[] = {
        get_locale_cstr("label.render_mode_contour"),
        get_locale_cstr("label.render_mode_depth"),
        get_locale_cstr("label.render_mode_lighting"),
    };
    ImGui::SetNextItemWidth(120);
    if (ImGui::Combo("##render_mode", &ortho_state.ortho_render_mode,
                     render_mode_names, 3)) {
        if (ortho_state.view_tex_ready) {
            ortho_state.render_dirty = true;
        }
    }

    // Persist overlay state every frame so that project save (Ctrl+S)
    // always captures the latest drag/resize position — not just the
    // last slider/checkbox change.
    sync_overlay_to_item();

    ImGui::End();
}

void RenderVoxelList::destroy_ortho_resources() {
    // NOTE: overlay_tex and overlay_cpu_rgba_ are intentionally left alone.
    // They are independent of the base-model render and should survive
    // re-renders triggered by depth-colour toggles, viewport changes, etc.
    if (bgfx::isValid(ortho_state.view_fb)) {
        bgfx::destroy(ortho_state.view_fb);
        ortho_state.view_fb = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(ortho_state.view_tex)) {
        bgfx::destroy(ortho_state.view_tex);
        ortho_state.view_tex = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(ortho_state.view_depth_tex)) {
        bgfx::destroy(ortho_state.view_depth_tex);
        ortho_state.view_depth_tex = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(ortho_state.ai_readback_tex)) {
        bgfx::destroy(ortho_state.ai_readback_tex);
        ortho_state.ai_readback_tex = BGFX_INVALID_HANDLE;
    }
    ortho_state.coord_map_ready = false;
    ortho_state.view_tex_ready = false;
    ortho_state.render_dirty = true;
    ortho_state.ortho_render_stage = 0;
    ortho_state._base_triangles.clear();
    ortho_state.ai_export_stage = 0;
    ortho_state.ai_readback_pending = false;
    ortho_state.ai_export_pending = false;

    // Release shader programs while bgfx context is still valid
    if (ortho_shader_) {
        ortho_shader_->release();
        ortho_shader_.reset();
    }
}

void RenderVoxelList::perform_ortho_render(RenderVoxelItem& item,
                                            RenderVoxelItem& base_item) {
    destroy_ortho_resources();

    // Build the orthographic camera matrices (stored for CPU-side raycasting).
    // projection_dir = center→camera (outward).
    // Camera is placed at center + projection_dir * 1000.
    // from_center = -projection_dir = camera→center direction.
    vec3f look_dir = ortho_state.projection_dir;
    vec3f from_center = {-look_dir.x, -look_dir.y, -look_dir.z};
    vec3f center = item.addon_center_point;

    float half = ortho_state.viewport_size * 0.5f;
    // Use the semantic coordinate frame's north-pole as the preferred
    // camera-up direction.  Fall back only when the projection direction
    // is nearly parallel to the north pole.
    vec3f np = {item.hair_north_pole.x, item.hair_north_pole.y, item.hair_north_pole.z};
    float np_len = std::sqrt(np.x*np.x + np.y*np.y + np.z*np.z);
    vec3f world_up = (np_len > 1e-8f) ? np : vec3f{0, 1, 0};
    if (std::abs(from_center.x * world_up.x + from_center.y * world_up.y + from_center.z * world_up.z) > 0.99f)
        world_up = vec3f{0, 0, 1};

    // Match GPU's internal camera basis:
    //   GPU right = normalize(cross(up_hint, forward))
    //   GPU up    = cross(forward, right)
    // where forward = normalize(at - eye).
    // We must match those, not compute an independent basis.
    // cam_right = normalize(cross(world_up, from_center))
    vec3f cam_right = {
        world_up.y * from_center.z - world_up.z * from_center.y,
        world_up.z * from_center.x - world_up.x * from_center.z,
        world_up.x * from_center.y - world_up.y * from_center.x
    };
    float cr_len = std::sqrt(cam_right.x * cam_right.x +
                             cam_right.y * cam_right.y +
                             cam_right.z * cam_right.z);
    if (cr_len > 1e-8f) {
        cam_right.x /= cr_len; cam_right.y /= cr_len; cam_right.z /= cr_len;
    } else {
        cam_right = {1, 0, 0};
    }

    // cam_up = normalize(cross(from_center, cam_right))
    vec3f cam_up = {
        from_center.y * cam_right.z - from_center.z * cam_right.y,
        from_center.z * cam_right.x - from_center.x * cam_right.z,
        from_center.x * cam_right.y - from_center.y * cam_right.x
    };
    float cu_len = std::sqrt(cam_up.x * cam_up.x +
                             cam_up.y * cam_up.y +
                             cam_up.z * cam_up.z);
    if (cu_len > 1e-8f) {
        cam_up.x /= cu_len; cam_up.y /= cu_len; cam_up.z /= cu_len;
    } else {
        cam_up = {0, 1, 0};
    }

    // Camera position: same direction as look_dir from center.
    vec3f cam_pos = {center.x + look_dir.x * 1000.0f,
                     center.y + look_dir.y * 1000.0f,
                     center.z + look_dir.z * 1000.0f};

    // Store ortho camera params for CPU raycasting
    ortho_state._cam_right = cam_right;
    ortho_state._cam_up = cam_up;
    ortho_state._cam_pos = cam_pos;
    ortho_state._center = center;

    // Copy base model triangles for CPU-side raycasting
    ortho_state._base_triangles.clear();
    if (!base_item.cached_mesh.empty()) {
        ortho_state._base_triangles.reserve(base_item.cached_mesh.size());
        for (const auto& [tri, n] : base_item.cached_mesh) {
            (void)n;
            ortho_state._base_triangles.push_back(tri);
        }
    } else if (!base_item.source_triangles.empty()) {
        ortho_state._base_triangles = base_item.source_triangles;
    }

    ortho_state.coord_map_ready = true;
    ortho_state._base_triangle_count = ortho_state._base_triangles.size();

    // ---- Create GPU off-screen render resources ----
    int res = ortho_state.render_resolution;
    constexpr uint64_t tex_flags =
        BGFX_TEXTURE_RT | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP |
        BGFX_SAMPLER_MIN_ANISOTROPIC | BGFX_SAMPLER_MAG_ANISOTROPIC;

    // Create the render texture with mipmaps.  BGFX_RESOLVE_AUTO_GEN_MIPS
    // auto-generates mip levels after rendering.  When the render is
    // displayed at ~574 px via the font shader's texture2D(), the GPU
    // auto-selects the appropriate mip LOD (e.g. log2(2048/574) ≈ 1.83)
    // and trilinearly blends between levels, providing effective AA
    // even at high minification ratios.
    bool has_mips = true;
    ortho_state.view_tex = bgfx::createTexture2D(
        static_cast<uint16_t>(res), static_cast<uint16_t>(res), has_mips, 1,
        bgfx::TextureFormat::BGRA8, tex_flags);
    ortho_state.view_depth_tex = bgfx::createTexture2D(
        static_cast<uint16_t>(res), static_cast<uint16_t>(res), false, 1,
        bgfx::TextureFormat::D32F, tex_flags);

    bgfx::Attachment fbo_att[2];
    fbo_att[0].init(ortho_state.view_tex, bgfx::Access::Write, 0, 1, 0,
                    BGFX_RESOLVE_AUTO_GEN_MIPS);
    fbo_att[1].init(ortho_state.view_depth_tex, bgfx::Access::Write);
    ortho_state.view_fb =
        bgfx::createFrameBuffer(2, fbo_att, false);

    // Create ortho shader (view 200 for off-screen render)
    if (!ortho_shader_) {
        ortho_shader_ = std::make_unique<RenderMeshShader>(kOrthoViewView, 0);
    }

    // Kick off multi-frame render
    ortho_state.ortho_base_item_id = base_item.id;
    ortho_state.ortho_render_stage = 1;  // RENDER
    ortho_state.render_dirty = false;

    std::cout << "[ortho_render] Setup off-screen render res=" << res
              << " with " << ortho_state._base_triangles.size()
              << " triangles, look_dir=("
              << look_dir.x << "," << look_dir.y << "," << look_dir.z << ")"
              << std::endl;
}

void RenderVoxelList::process_ortho_render() {
    if (ortho_state.ortho_render_stage == 0)
        return;  // IDLE

    // Stage 1: Submit render commands
    if (ortho_state.ortho_render_stage == 1) {
        if (!bgfx::isValid(ortho_state.view_fb) || !ortho_shader_) {
            ortho_state.ortho_render_stage = 0;
            return;
        }

        int render_mode = ortho_state.ortho_render_mode;

        if (render_mode == 1) {  // Depth
            if (!ortho_shader_->ensureOrthoDepthProgram()) {
                std::cerr << "[ortho_render] Failed to load ortho depth shader" << std::endl;
                ortho_state.ortho_render_stage = 0;
                return;
            }
        } else {  // Contour (0) or Lighting (2) — both use GBuffer + u_lightingMode
            if (!ortho_shader_->ensureGBufferProgram()) {
                std::cerr << "[ortho_render] Failed to load GBuffer shader" << std::endl;
                ortho_state.ortho_render_stage = 0;
                return;
            }
        }

        // Find the base item
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(ortho_state.ortho_base_item_id);
        if (it == items.end()) {
            ortho_state.ortho_render_stage = 0;
            return;
        }
        auto& base_item = it->second;

        // Check renderer availability.
        // The item-level mesh_renderer holds the smooth source mesh for BOTH
        // mesh_only and voxel items (voxel_renderer's own main mesh is never
        // populated for voxel items — only its chunked voxel surface is).
        // The chunked surface is midpoint-snapped binary marching cubes
        // (only axis-aligned / 45° edges at voxel resolution), which looks
        // blocky / "mosaic-like" in ortho projection, so use it only as a
        // last resort when no smooth mesh exists at all.
        bool use_chunked = false;  // true → render chunked voxel surface
        if (base_item->mesh_renderer.empty() && !base_item->cached_mesh.empty()) {
            // Smooth mesh data exists CPU-side but was never uploaded
            base_item->mesh_renderer.loadGeometry(base_item->cached_mesh);
        }
        if (base_item->mesh_renderer.empty()) {
            if (base_item->mesh_only || base_item->voxel_renderer.empty()) {
                std::cerr << "[ortho_render] Base item has no renderer" << std::endl;
                ortho_state.ortho_render_stage = 0;
                return;
            }
            use_chunked = true;
        }

        // Build orthographic view and projection matrices
        vec3f center = ortho_state._center;
        float half = ortho_state.viewport_size * 0.5f;

        // View matrix: look at center from the stored camera position
        vec3f cam_pos = ortho_state._cam_pos;
        vec3f cam_up = ortho_state._cam_up;

        float view[16];
        bx::mtxLookAt(view,
                      bx::Vec3{cam_pos.x, cam_pos.y, cam_pos.z},
                      bx::Vec3{center.x, center.y, center.z},
                      bx::Vec3{cam_up.x, cam_up.y, cam_up.z});

        float proj[16];
        // bottom > top compensates for the implicit Y-flip when the OpenGL
        // framebuffer (bottom-left origin) is displayed via ImGui::Image
        // (top-left origin).  This way the rendered image orientation
        // matches the CPU-side raycasting coordinate system.
        bx::mtxOrtho(proj, -half, half, half, -half, -2000.0f, 2000.0f, 0.0f,
                     bgfx::getCaps()->homogeneousDepth);

        int res = ortho_state.render_resolution;
        bgfx::setViewRect(kOrthoViewView, 0, 0, static_cast<uint16_t>(res),
                          static_cast<uint16_t>(res));
        bgfx::setViewFrameBuffer(kOrthoViewView, ortho_state.view_fb);
        bgfx::setViewClear(kOrthoViewView,
                           BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                           0x303030ff, 1.0f, 0);
        bgfx::setViewTransform(kOrthoViewView, view, proj);

        float identity[16];
        bx::mtxIdentity(identity);

        // Depth / Lighting uniforms (used by depth and lighting modes)
        float view_dir_arr[3] = {0, 0, 0};
        float center_arr[3] = {center.x, center.y, center.z};
        float depth_scale = ortho_state.viewport_size > 1e-8f
                                ? 1.0f / ortho_state.viewport_size
                                : 0.01f;
        vec3f look_dir = ortho_state.projection_dir;
        float fl = std::sqrt(look_dir.x*look_dir.x + look_dir.y*look_dir.y + look_dir.z*look_dir.z);
        if (fl > 1e-8f) {
            look_dir.x /= fl; look_dir.y /= fl; look_dir.z /= fl;
        }
        view_dir_arr[0] = look_dir.x;
        view_dir_arr[1] = look_dir.y;
        view_dir_arr[2] = look_dir.z;

        if (render_mode == 1) {  // Depth heatmap
            if (use_chunked) {
                base_item->voxel_renderer.renderDepthColor(identity, *ortho_shader_,
                                                           view_dir_arr, center_arr,
                                                           depth_scale);
            } else {
                base_item->mesh_renderer.renderDepthColor(identity, *ortho_shader_,
                                                          view_dir_arr, center_arr,
                                                          depth_scale);
            }
        } else {  // Contour or Lighting — both use GBuffer program
            // Set lighting mode: 0.0 = contour, 1.0 = lighting
            float lighting_vec[4] = { (render_mode == 2) ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f };
            ortho_shader_->ensureUniforms();
            bgfx::setUniform(ortho_shader_->u_lighting_mode_, lighting_vec);
            if (use_chunked) {
                base_item->voxel_renderer.renderGBuffer(identity, *ortho_shader_);
            } else {
                base_item->mesh_renderer.renderGBuffer(identity, *ortho_shader_);
            }
        }
        bgfx::touch(kOrthoViewView);

        ortho_state.ortho_render_stage = 2;  // WAIT
        ortho_state.ortho_wait_frames = 2;
        return;
    }

    // Stage 2: Wait for render to complete
    if (ortho_state.ortho_render_stage == 2) {
        if (ortho_state.ortho_wait_frames > 0) {
            ortho_state.ortho_wait_frames--;
        }
        if (ortho_state.ortho_wait_frames <= 0) {
            ortho_state.ortho_render_stage = 3;  // DONE
        }
        return;
    }

    // Stage 3: Done
    if (ortho_state.ortho_render_stage == 3) {
        ortho_state.view_tex_ready = true;
        ortho_state.api_render_dirty = true;  // trigger GPU readback for API cache
        ortho_state.ortho_render_stage = 0;  // back to IDLE
        std::cout << "[ortho_render] View texture ready" << std::endl;
    }
}

}  // namespace sinriv::ui::render