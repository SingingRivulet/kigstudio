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
#include <sys/stat.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <type_traits>
#include <unordered_set>
#include <variant>
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

// ============================================================
// Inverse of spherical_to_dir: convert world-space direction →
// (theta, phi) using the same N/U/V frame.
// Returns false when direction is parallel to north pole (theta undefined).
// ============================================================
static bool dir_to_spherical(const vec3f& world_dir,
                             const vec3f& north_pole,
                             const vec3f& front_reference,
                             float& out_theta_deg,
                             float& out_phi_deg) {
    vec3f d = world_dir.normalize();
    vec3f N = north_pole.normalize();

    // phi = asin(dot(d, N))
    float dot_d_n = d.dot(N);
    dot_d_n = std::max(-1.0f, std::min(1.0f, dot_d_n));
    float phi_rad = std::asin(dot_d_n);
    constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;
    out_phi_deg = phi_rad * kRadToDeg;

    // Project direction onto equatorial plane
    vec3f d_uv = d - N * dot_d_n;
    float uv_len2 = d_uv.length2();
    constexpr float kEps = 1e-10f;

    if (uv_len2 < kEps) {
        // Direction is parallel to north pole → theta is undefined
        out_theta_deg = 0.0f;
        return false;
    }

    // Build U, V frame (same as spherical_to_dir)
    vec3f F = front_reference.normalize();
    float f_dot_n = F.dot(N);
    vec3f V = F - N * f_dot_n;
    float v_len2 = V.length2();

    if (v_len2 < 1e-10f) {
        vec3f A = (std::abs(N.z) < 0.99f) ? vec3f(0.0f, 0.0f, 1.0f)
                                          : vec3f(1.0f, 0.0f, 0.0f);
        V = A - N * A.dot(N);
        v_len2 = V.length2();
    }
    V = V / std::sqrt(v_len2);
    vec3f U = cross(N, V);

    // sin(theta) = dot(d_uv_norm, U), cos(theta) = dot(d_uv_norm, V)
    float inv_len = 1.0f / std::sqrt(uv_len2);
    float sin_t = d_uv.dot(U) * inv_len;
    float cos_t = d_uv.dot(V) * inv_len;
    float theta_rad = std::atan2(sin_t, cos_t);
    out_theta_deg = theta_rad * kRadToDeg;

    return true;
}

// ============================================================
// Cross-validation: ensure no grid lines cross
// ============================================================
static bool validate_angle_grid(
    const std::map<std::pair<float, float>, HairAngleEntry>& config,
    int proposed_x,
    int proposed_y,
    float new_theta,
    float new_phi) {
    // Build temporary config with the proposed value
    auto tmp = config;
    tmp[{static_cast<float>(proposed_x), static_cast<float>(proposed_y)}] =
        HairAngleEntry{new_theta, new_phi};

    // 1. Theta monotonicity per row (fixed Y)
    for (int y = -10; y <= 14; ++y) {
        std::vector<std::pair<int, float>> row;  // (X, theta)
        for (int x = -10; x <= 10; ++x) {
            auto it = tmp.find({static_cast<float>(x), static_cast<float>(y)});
            if (it != tmp.end()) {
                row.push_back({x, it->second.theta});
            }
        }
        if (row.size() < 2)
            continue;
        bool increasing = true, decreasing = true;
        for (size_t i = 1; i < row.size(); ++i) {
            if (row[i].second <= row[i - 1].second)
                increasing = false;
            if (row[i].second >= row[i - 1].second)
                decreasing = false;
        }
        if (!increasing && !decreasing)
            return false;
    }

    // 2. Phi monotonicity per column (fixed X)
    // Phi peaks at the crown (Y≈0) and decreases toward both front neck
    // and back neck, so we check monotonicity separately for Y≤0 and Y≥0.
    for (int x = -10; x <= 10; ++x) {
        std::vector<std::pair<int, float>> col;  // (Y, phi)
        for (int y = -10; y <= 14; ++y) {
            auto it = tmp.find({static_cast<float>(x), static_cast<float>(y)});
            if (it != tmp.end()) {
                col.push_back({y, it->second.phi});
            }
        }
        if (col.size() < 2)
            continue;

        // Split at Y=0: back region (Y≤0) and front region (Y≥0).
        // Each region independently must be monotonic.
        auto check_monotonic =
            [](const std::vector<std::pair<int, float>>& seg) -> bool {
            if (seg.size() < 2)
                return true;
            bool increasing = true, decreasing = true;
            for (size_t i = 1; i < seg.size(); ++i) {
                if (seg[i].second <= seg[i - 1].second)
                    increasing = false;
                if (seg[i].second >= seg[i - 1].second)
                    decreasing = false;
            }
            return increasing || decreasing;
        };

        std::vector<std::pair<int, float>> back_region, front_region;
        for (const auto& p : col) {
            if (p.first <= 0)
                back_region.push_back(p);
            if (p.first >= 0)
                front_region.push_back(p);
        }
        if (!check_monotonic(back_region))
            return false;
        if (!check_monotonic(front_region))
            return false;
    }

    // 3. Midline separation: X=0 and X=±10 must not have overlapping theta.
    // Only valid for Y ≥ 0 (front of head) where X=0 is anterior midline and
    // X=±10 is posterior midline. For Y < 0 (top/back of head), all three
    // reference the same posterior region so the separation check is skipped.
    for (int y = -10; y <= 14; ++y) {
        if (y < 0)
            continue;  // skip back-of-head rows
        auto it0 = tmp.find({0.0f, static_cast<float>(y)});
        auto it10 = tmp.find({10.0f, static_cast<float>(y)});
        auto itm10 = tmp.find({-10.0f, static_cast<float>(y)});
        float t0 = (it0 != tmp.end()) ? it0->second.theta
                                      : std::numeric_limits<float>::quiet_NaN();
        float t10 = (it10 != tmp.end())
                        ? it10->second.theta
                        : std::numeric_limits<float>::quiet_NaN();
        float tm10 = (itm10 != tmp.end())
                         ? itm10->second.theta
                         : std::numeric_limits<float>::quiet_NaN();
        if (!std::isnan(t0) && !std::isnan(t10)) {
            if (std::abs(t0 - t10) < 10.0f)
                return false;
        }
        if (!std::isnan(t0) && !std::isnan(tm10)) {
            if (std::abs(t0 - tm10) < 10.0f)
                return false;
        }
    }

    return true;
}

void RenderVoxelList::render_hairline_plane_window() {
    if (!show_addon_window)
        return;
    if (!show_hairline_plane_window)
        return;

    // Mutual exclusion with other editor windows (but NOT angle config)
    if (show_guide_curve_window) {
        auto git = items.find(render_id);
        if (git != items.end()) {
            git->second->guide_curve_drawing_active = false;
            git->second->active_guide_draw_strand.clear();
        }
        show_guide_curve_window = false;
    }
    if (show_width_editor_window) {
        auto wit = items.find(render_id);
        if (wit != items.end()) {
            wit->second->width_editing_active = false;
            wit->second->active_width_edit_strand.clear();
        }
        show_width_editor_window = false;
    }
    // NOTE: no longer close show_angle_config_window

    // 初始位置：中心点位于屏幕中心
    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Once, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(320, 180), ImGuiCond_Once);
    bool window_open = true;
    if (!ImGui::Begin(get_locale_cstr("window.auto_width"), &window_open)) {
        ImGui::End();
        return;
    }

    if (!window_open) {
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->hairline_point_picking_active = false;
        }
        show_hairline_plane_window = false;
        ImGui::End();
        return;
    }

    std::lock_guard<std::mutex> lock(locker);
    auto item_it = items.find(render_id);
    if (item_it == items.end() || item_it->second->source_type != 2) {
        ImGui::End();
        return;
    }

    RenderVoxelItem& item = *item_it->second;

    if (!item.hairline_plane_enabled) {
        ImGui::TextDisabled(
            "%s", get_locale_cstr("label.hairline_plane_disabled_hint"));
        ImGui::End();
        return;
    }

    ImGui::Separator();

    // ---- Scale factor ----
    ImGui::SetNextItemWidth(200);
    ImGui::DragFloat(get_locale_cstr("label.hairline_spindle_scale"),
                     &item.hairline_spindle_scale, 0.01f, 0.0f, 1.0f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s",
                          get_locale_cstr("tooltip.hairline_spindle_scale"));

    ImGui::Separator();

    // ---- Apply button ----
    if (ImGui::Button(get_locale_cstr("action.apply_hairline_spindle"),
                      ImVec2(-1, 0))) {
        push_undo_now(item.id, std::nullopt, "Apply Hairline Spindle");
        item.apply_hairline_spindle();
        for (auto& s : item.hair_strands)
            s.mesh_dirty = true;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s",
                          get_locale_cstr("tooltip.apply_hairline_spindle"));
    }

    ImGui::End();
}

// ============================================================
// Semantic coordinate anchor point definitions (from api-reference.md)
// X-axis (Y=0): left-to-right lateral anchors
// Y-axis (X=0): front-to-back midline anchors
// ============================================================
struct AnchorPoint {
    int x;
    int y;
    const char* name_en;
    const char* name_zh;
};
static const AnchorPoint kAnchorPoints[] = {
    // X axis (Y=0) — lateral cross-section
    {-10, 0, "Posterior midline (L)", "后正中线（左）"},
    {-9, 0, "Lateral occipital (L)", "枕骨外侧（左）"},
    {-8, 0, "Mastoid process (L)", "乳突（左）"},
    {-7, 0, "Helix outer edge (L)", "耳轮外缘（左）"},
    {-6, 0, "Tragus (L)", "耳屏（左）"},
    {-5, 0, "Zygomatic arch (L)", "颧弓最外侧（左）"},
    {-4, 0, "Outer canthus (L)", "外眼角（左）"},
    {-3, 0, "Pupil center (L)", "瞳孔中心（左）"},
    {-2, 0, "Inner canthus (L)", "内眼角（左）"},
    {-1, 0, "Ala of nose (L)", "鼻翼外缘（左）"},
    {0, 0, "Midline / Midsagittal", "头顶 / 前正中线"},
    {1, 0, "Ala of nose (R)", "鼻翼外缘（右）"},
    {2, 0, "Inner canthus (R)", "内眼角（右）"},
    {3, 0, "Pupil center (R)", "瞳孔中心（右）"},
    {4, 0, "Outer canthus (R)", "外眼角（右）"},
    {5, 0, "Zygomatic arch (R)", "颧弓最外侧（右）"},
    {6, 0, "Tragus (R)", "耳屏（右）"},
    {7, 0, "Helix outer edge (R)", "耳轮外缘（右）"},
    {8, 0, "Mastoid process (R)", "乳突（右）"},
    {9, 0, "Lateral occipital (R)", "枕骨外侧（右）"},
    {10, 0, "Posterior midline (R)", "后正中线（右）"},
    // Y axis front (X=0) — frontal midline
    {0, 1, "Forehead hairline", "额头发际线"},
    {0, 2, "Upper brow", "眉毛上缘"},
    {0, 3, "Lower brow", "眉毛下缘"},
    {0, 4, "Nasion", "鼻根"},
    {0, 5, "Upper eye", "眼上缘"},
    {0, 6, "Lower eye", "眼下缘"},
    {0, 7, "Nose tip", "鼻尖"},
    {0, 8, "Nasal base", "鼻底"},
    {0, 9, "Upper lip", "嘴唇上缘"},
    {0, 10, "Oral fissure", "口裂"},
    {0, 11, "Lower lip", "嘴唇下缘"},
    {0, 12, "Chin (Menton)", "颏部"},
    {0, 13, "Mandible border", "下颌下缘"},
    {0, 14, "Anterior neck", "颈前部"},
    // Y axis back (X=0) — posterior midline
    {0, -1, "Coronal suture", "冠状缝附近"},
    {0, -2, "Parietal center", "顶骨中央"},
    {0, -3, "Lambda", "顶枕点"},
    {0, -4, "Upper occipital", "枕骨上部"},
    {0, -5, "External Occipital Protuberance", "枕外隆凸"},
    {0, -6, "Superior nuchal line", "上项线"},
    {0, -7, "Lower occipital", "枕骨下部"},
    {0, -8, "Posterior hairline", "后发际线"},
    {0, -9, "Posterior neck junction", "颈后连接处"},
    {0, -10, "Lower posterior neck", "颈后下部"},
};

static const AnchorPoint* find_anchor(int x, int y) {
    for (const auto& a : kAnchorPoints)
        if (a.x == x && a.y == y)
            return &a;
    return nullptr;
}

static const char* get_anchor_name(int x, int y) {
    auto* a = find_anchor(x, y);
    return a ? (get_system_language() == "zh" ? a->name_zh : a->name_en)
             : nullptr;
}

// Always returns Chinese name (used in table display)
static const char* get_anchor_name_cn(int x, int y) {
    auto* a = find_anchor(x, y);
    return a ? a->name_zh : nullptr;
}

// Compute the effective hair root point: common_hair_root_point moved toward
// the addon center by hair_root_center_offset.
static vec3f compute_effective_hair_root(const RenderVoxelList::RenderVoxelItem& item) {
    vec3f effective_root = item.common_hair_root_point;
    vec3f to_center = {item.addon_center_point.x - effective_root.x,
                       item.addon_center_point.y - effective_root.y,
                       item.addon_center_point.z - effective_root.z};
    float dist = std::sqrt(to_center.x * to_center.x +
                           to_center.y * to_center.y +
                           to_center.z * to_center.z);
    if (dist > 0.001f && item.hair_root_center_offset > 0.0f) {
        vec3f dir = {to_center.x / dist, to_center.y / dist,
                     to_center.z / dist};
        float offset = item.hair_root_center_offset;
        if (offset > dist)
            offset = dist;
        effective_root = {effective_root.x + dir.x * offset,
                          effective_root.y + dir.y * offset,
                          effective_root.z + dir.z * offset};
    }
    return effective_root;
}

// Enable or disable the auto hair root guide point for a single strand.
// Mirrors the "Auto Hair Root" checkbox logic in the guide curve editor.
static void set_strand_hair_root(HairStrand& strand, bool enable,
                                 const vec3f& effective_root) {
    if (enable) {
        strand.hidden_guide_points_start = {effective_root};
        strand.hair_root_enabled = true;
        // Migrate width points to full-curve space when hidden points are
        // added for the first time.
        if (!strand.width_curve_id_v2 && !strand.width_points.empty()) {
            for (auto& wp : strand.width_points)
                wp.curve_id += 1.0f;
            strand.width_curve_id_v2 = true;
        }
    } else {
        // Before clearing hidden start points, remove width vectors placed
        // on them so orphaned curve_ids don't cause missing loft sections.
        size_t hidden_n = strand.hidden_guide_points_start.size();
        if (strand.width_curve_id_v2 && hidden_n > 0 &&
            !strand.width_points.empty()) {
            strand.width_points.erase(
                std::remove_if(strand.width_points.begin(),
                               strand.width_points.end(),
                               [hidden_n](const auto& wp) {
                                   return wp.curve_id <
                                          static_cast<float>(hidden_n) - 0.001f;
                               }),
                strand.width_points.end());
            float offset = static_cast<float>(hidden_n);
            for (auto& wp : strand.width_points)
                wp.curve_id -= offset;
            if (strand.hidden_guide_points_end.empty())
                strand.width_curve_id_v2 = false;
        }
        strand.hidden_guide_points_start.clear();
        strand.hair_root_enabled = false;
    }
    strand.mesh_dirty = true;
}

void RenderVoxelList::render_hair_root_window() {
    if (!show_hair_root_window)
        return;

    // Do NOT close other windows (non-exclusive per requirement)

    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Once, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(420, 520), ImGuiCond_Once);
    bool window_open = true;
    if (!ImGui::Begin(get_locale_cstr("window.hair_root_edit"), &window_open)) {
        ImGui::End();
        return;
    }

    if (!window_open) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->hair_root_edit_active = false;
        }
        show_hair_root_window = false;
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
    auto& item = *item_it->second;

    item.hair_root_edit_active = true;

    // ---- Common Hair Root Point (shared by all strands) ----

    // North-pole direction must be configured for auto hair root
    float np_len = std::sqrt(item.hair_north_pole.x * item.hair_north_pole.x +
                             item.hair_north_pole.y * item.hair_north_pole.y +
                             item.hair_north_pole.z * item.hair_north_pole.z);
    bool can_auto_root = np_len > 0.001f;

    // Auto hair root toggle (only when north_pole direction is configured)
    {
        if (can_auto_root) {
            bool prev_auto = item.auto_hair_root;
            if (ImGui::Checkbox(get_locale_cstr("label.auto_hair_root"),
                                &item.auto_hair_root)) {
                if (item.auto_hair_root) {
                    // Compute ray from north-pole direction toward center,
                    // find first hit on base model triangles
                    vec3f dir = {item.hair_north_pole.x / np_len,
                                 item.hair_north_pole.y / np_len,
                                 item.hair_north_pole.z / np_len};
                    vec3f origin = {item.addon_center_point.x + dir.x * 500.0f,
                                    item.addon_center_point.y + dir.y * 500.0f,
                                    item.addon_center_point.z + dir.z * 500.0f};
                    vec3f ray_dir = {-dir.x, -dir.y, -dir.z};
                    vec3f hit = {item.addon_center_point.x,
                                 item.addon_center_point.y,
                                 item.addon_center_point.z};
                    bool found = false;
                    float best_t = 1e30f;

                    if (item.addon_base_node_id >= 0) {
                        auto base_it = items.find(item.addon_base_node_id);
                        if (base_it != items.end()) {
                            auto& base = *base_it->second;
                            auto test_tri = [&](const vec3f& v0,
                                                const vec3f& v1,
                                                const vec3f& v2) {
                                float t;
                                if (ray_triangle_intersect(origin, ray_dir, v0,
                                                           v1, v2, t) &&
                                    t < best_t) {
                                    best_t = t;
                                    hit = {origin.x + ray_dir.x * t,
                                           origin.y + ray_dir.y * t,
                                           origin.z + ray_dir.z * t};
                                    found = true;
                                }
                            };
                            if (!base.cached_mesh.empty()) {
                                for (const auto& entry : base.cached_mesh) {
                                    const auto& tri = std::get<0>(entry);
                                    auto tv0 = std::get<0>(tri);
                                    auto tv1 = std::get<1>(tri);
                                    auto tv2 = std::get<2>(tri);
                                    test_tri({tv0.x, tv0.y, tv0.z},
                                             {tv1.x, tv1.y, tv1.z},
                                             {tv2.x, tv2.y, tv2.z});
                                }
                            } else {
                                for (const auto& tri : base.source_triangles) {
                                    auto tv0 = std::get<0>(tri);
                                    auto tv1 = std::get<1>(tri);
                                    auto tv2 = std::get<2>(tri);
                                    test_tri({tv0.x, tv0.y, tv0.z},
                                             {tv1.x, tv1.y, tv1.z},
                                             {tv2.x, tv2.y, tv2.z});
                                }
                            }
                        }
                    }
                    if (!found) {
                        // Fallback: project center along north pole direction
                        hit = {item.addon_center_point.x - dir.x * 10.0f,
                               item.addon_center_point.y - dir.y * 10.0f,
                               item.addon_center_point.z - dir.z * 10.0f};
                    }
                    item.common_hair_root_point = hit;
                }
                // Propagate to all strands immediately
                {
                    vec3f effective_root = compute_effective_hair_root(item);
                    // ImGui already toggled auto_hair_root; swap back so the
                    // undo snapshot captures the pre-toggle state.
                    std::swap(item.auto_hair_root, prev_auto);
                    push_undo_now(item.id, std::nullopt, "Guide Point / Width Auto Hair Root");
                    std::swap(item.auto_hair_root, prev_auto);
                    for (auto& s : item.hair_strands) {
                        set_strand_hair_root(s, item.auto_hair_root,
                                             effective_root);
                    }
                }
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("%s",
                                  get_locale_cstr("tooltip.auto_hair_root"));
        } else {
            ImGui::TextWrapped(
                "%s", get_locale_cstr("label.need_north_pole_for_hair_root"));
        }
    }

    // Display common root point position (read-only for now)
    ImGui::Text("%s: (%.2f, %.2f, %.2f)",
                get_locale_cstr("label.common_hair_root_point"),
                static_cast<double>(item.common_hair_root_point.x),
                static_cast<double>(item.common_hair_root_point.y),
                static_cast<double>(item.common_hair_root_point.z));

    ImGui::Separator();

    // Center offset slider (moves the root point toward center).
    // Changes propagate to enabled strands in real time; one undo entry is
    // recorded per drag gesture.
    {
        float prev_offset = item.hair_root_center_offset;
        ImGui::SetNextItemWidth(200);
        ImGui::SliderFloat(get_locale_cstr("label.hair_root_center_offset"),
                           &item.hair_root_center_offset, 0.0f, 50.0f, "%.1f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "%s", get_locale_cstr("tooltip.hair_root_center_offset"));
        if (ImGui::IsItemActivated())
            begin_edit(item.id);
        if (ImGui::IsItemDeactivatedAfterEdit())
            end_edit(item.id, "Hair Root Offset");
        if (prev_offset != item.hair_root_center_offset) {
            vec3f effective_root = compute_effective_hair_root(item);
            for (auto& s : item.hair_strands) {
                if (s.hair_root_enabled &&
                    !s.hidden_guide_points_start.empty()) {
                    s.hidden_guide_points_start[0] = effective_root;
                    s.mesh_dirty = true;
                }
            }
        }

        // Root vector length: length of the synthetic short width vector
        // injected at the strand start when "generate hair root" is on.
        // Changes rebuild affected strand meshes in real time.
        ImGui::SameLine();
        float prev_vlen = item.hair_root_vector_length;
        ImGui::SetNextItemWidth(100);
        ImGui::DragFloat(get_locale_cstr("label.hair_root_vector_length"),
                         &item.hair_root_vector_length, 0.01f, 0.0f, 10.0f,
                         "%.2f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "%s", get_locale_cstr("tooltip.hair_root_vector_length"));
        if (ImGui::IsItemActivated())
            begin_edit(item.id);
        if (ImGui::IsItemDeactivatedAfterEdit())
            end_edit(item.id, "Hair Root Vector Length");
        if (prev_vlen != item.hair_root_vector_length) {
            for (auto& s : item.hair_strands) {
                if (s.hair_root_generate)
                    s.mesh_dirty = true;
            }
        }
    }

    ImGui::Separator();

    // Strand status table. The per-strand checkbox is equivalent to the
    // "Auto Hair Root" checkbox in the guide curve editor and applies
    // immediately — no separate update button is needed.
    ImGui::TextUnformatted(get_locale_cstr("label.hair_strands"));

    vec3f effective_root = compute_effective_hair_root(item);

    float table_h = ImGui::GetContentRegionAvail().y;
    if (ImGui::BeginTable("##hr_table", 6,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_ScrollY,
                          ImVec2(0, table_h))) {
        ImGui::TableSetupColumn(get_locale_cstr("label.hair_root_col_enable"),
                                ImGuiTableColumnFlags_WidthFixed, 40.0f);
        ImGui::TableSetupColumn(get_locale_cstr("label.hair_root_col_strand"),
                                ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn(
            get_locale_cstr("label.hair_root_col_root_point"),
            ImGuiTableColumnFlags_WidthFixed, 130.0f);
        ImGui::TableSetupColumn(
            get_locale_cstr("label.hair_root_col_guide_count"),
            ImGuiTableColumnFlags_WidthFixed, 55.0f);
        ImGui::TableSetupColumn(
            get_locale_cstr("label.hair_root_col_width_count"),
            ImGuiTableColumnFlags_WidthFixed, 55.0f);
        ImGui::TableSetupColumn(
            get_locale_cstr("label.hair_root_col_generate"),
            ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableHeadersRow();

        for (size_t i = 0; i < item.hair_strands.size(); ++i) {
            auto& strand = item.hair_strands[i];
            ImGui::PushID(static_cast<int>(i));
            ImGui::TableNextRow();

            // Column 1: per-strand auto hair root checkbox
            ImGui::TableNextColumn();
            bool enabled = strand.hair_root_enabled;
            if (!can_auto_root)
                ImGui::BeginDisabled();
            if (ImGui::Checkbox("##hr_enable", &enabled)) {
                push_undo_now(item.id, std::nullopt, "Toggle Hair Root");
                set_strand_hair_root(strand, enabled, effective_root);
            }
            if (!can_auto_root)
                ImGui::EndDisabled();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "%s",
                    get_locale_cstr("tooltip.hair_root_strand_enable"));

            // Column 2: strand name
            ImGui::TableNextColumn();
            if (!strand.name.empty()) {
                ImGui::TextUnformatted(strand.name.c_str());
            } else {
                ImGui::Text(get_locale_cstr("label.hair_strand"),
                            static_cast<int>(i + 1));
            }

            // Column 3: root point position (real-time)
            ImGui::TableNextColumn();
            if (strand.hair_root_enabled &&
                !strand.hidden_guide_points_start.empty()) {
                const auto& rp = strand.hidden_guide_points_start[0];
                ImGui::Text("(%.2f, %.2f, %.2f)", static_cast<double>(rp.x),
                            static_cast<double>(rp.y),
                            static_cast<double>(rp.z));
            } else {
                ImGui::TextDisabled("-");
            }

            // Column 4: guide point count
            ImGui::TableNextColumn();
            ImGui::Text("%d", static_cast<int>(strand.guide_points.size()));

            // Column 5: width point count
            ImGui::TableNextColumn();
            ImGui::Text("%d", static_cast<int>(strand.width_points.size()));

            // Column 6: generate hair root checkbox (real-time mesh update)
            ImGui::TableNextColumn();
            bool gen_root = strand.hair_root_generate;
            if (ImGui::Checkbox("##hr_generate", &gen_root)) {
                push_undo_now(item.id, std::nullopt,
                              "Toggle Hair Root Generate");
                strand.hair_root_generate = gen_root;
                strand.mesh_dirty = true;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", get_locale_cstr(
                                            "tooltip.hair_root_generate"));

            ImGui::PopID();
        }
        ImGui::EndTable();
    }

    ImGui::End();
}

// ============================================================
// Semantic coordinate angle config editor window
// ============================================================
void RenderVoxelList::render_angle_config_window() {
    if (!show_addon_window)
        return;
    if (!show_angle_config_window)
        return;

    // Mutual exclusion: close sibling windows (but NOT auto-width/hairline
    // plane)
    if (show_guide_curve_window) {
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->guide_curve_drawing_active = false;
            it->second->active_guide_draw_strand.clear();
        }
        show_guide_curve_window = false;
    }
    if (show_width_editor_window) {
        auto wit = items.find(render_id);
        if (wit != items.end()) {
            wit->second->width_editing_active = false;
            wit->second->active_width_edit_strand.clear();
        }
        show_width_editor_window = false;
    }
    if (show_cross_section_editor_window) {
        auto sit = items.find(render_id);
        if (sit != items.end())
            sit->second->active_section_edit_strand.clear();
        show_cross_section_editor_window = false;
    }
    if (show_perpoint_section_editor_window) {
        auto pit = items.find(render_id);
        if (pit != items.end()) {
            pit->second->perpoint_section_editing_active = false;
            pit->second->active_perpoint_section_edit_strand.clear();
            pit->second->active_perpoint_section_edit_width_idx = -1;
        }
        show_perpoint_section_editor_window = false;
    }
    // NOTE: no longer close show_hairline_plane_window — they can coexist

    // 初始位置：中心点位于屏幕中心
    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Once, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(900, 680), ImGuiCond_Once);
    bool window_open = true;
    if (!ImGui::Begin(get_locale_cstr("window.angle_config"), &window_open)) {
        ImGui::End();
        return;
    }

    if (!window_open) {
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->angle_config_editing_x =
                RenderVoxelItem::kAngleConfigSentinel;
            it->second->angle_config_editing_y =
                RenderVoxelItem::kAngleConfigSentinel;
        }
        show_angle_config_window = false;
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
    auto& item = *item_it->second;

    // Helper: rebuild hair BVH from base node's mesh triangles.
    // Called whenever hair_angle_config is modified so that crosshair
    // markers can raycast to the base model surface.
    auto rebuild_hair_bvh = [&]() {
        if (item.addon_base_node_id < 0)
            return;
        auto base_it = this->items.find(item.addon_base_node_id);
        if (base_it == this->items.end())
            return;
        auto& base = *base_it->second;
        std::vector<sinriv::kigstudio::voxel::Triangle> tris;
        if (!base.source_triangles.empty()) {
            tris = base.source_triangles;
        } else if (!base.cached_mesh.empty()) {
            tris.reserve(base.cached_mesh.size());
            for (const auto& [tri, _] : base.cached_mesh)
                tris.push_back(tri);
        }
        if (tris.empty())
            return;
        auto bvh =
            std::make_unique<sinriv::kigstudio::voxel::triangle_bvh<float>>();
        for (const auto& tri : tris)
            bvh->insert(tri);
        item.hair_bvh = std::move(bvh);
        item.hair_bvh_base_node_id = item.addon_base_node_id;
    };

    // Auto-enable center point and hairline plane when this window is open
    item.show_addon_center = true;
    item.hairline_plane_enabled = true;

    constexpr int kXMin = -10, kXMax = 10;
    constexpr int kYMin = -10, kYMax = 14;
    constexpr int kSentinel = RenderVoxelItem::kAngleConfigSentinel;

    // ================================================================
    // Axis table rendering lambda (used in right panel below)
    // ================================================================
    auto render_axis_table = [&](const char* title, bool is_x_axis) {
        ImGui::TextUnformatted(title);

        ImGuiTableFlags tbl_flags =
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
            ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit;

        if (!ImGui::BeginTable(is_x_axis ? "##AxisTableX" : "##AxisTableY", 5,
                               tbl_flags, ImVec2(0, 0))) {
            return;
        }
        ImGui::TableSetupColumn(get_locale_cstr("label.angle_id"),
                                ImGuiTableColumnFlags_WidthFixed, 30.0f);
        ImGui::TableSetupColumn(get_locale_cstr("label.angle_organ"),
                                ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn(get_locale_cstr("label.angle_theta"),
                                ImGuiTableColumnFlags_WidthFixed, 70.0f);
        ImGui::TableSetupColumn(get_locale_cstr("label.angle_phi"),
                                ImGuiTableColumnFlags_WidthFixed, 70.0f);
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 100.0f);
        ImGui::TableHeadersRow();

        int row_id = 0;
        int del_x = kSentinel, del_y = kSentinel;

        int lo = is_x_axis ? kXMin : kYMax;
        int hi = is_x_axis ? kXMax : kYMin;
        int step = is_x_axis ? 1 : -1;  // Y goes top-down

        for (int v = lo; (is_x_axis ? v <= hi : v >= hi); v += step) {
            int x = is_x_axis ? v : 0;
            int y = is_x_axis ? 0 : v;

            auto it = item.hair_angle_config.find(
                {static_cast<float>(x), static_cast<float>(y)});
            bool configured = (it != item.hair_angle_config.end());
            ++row_id;

            auto open_edit = [&](float def_theta, float def_phi) {
                item.angle_config_editing_x = x;
                item.angle_config_editing_y = y;
                item.angle_config_preview_theta = def_theta;
                item.angle_config_preview_phi = def_phi;
            };

            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%d", row_id);

            // Organ name
            ImGui::TableSetColumnIndex(1);
            const char* name_cn = get_anchor_name_cn(x, y);
            if (name_cn)
                ImGui::Text("%s  (%+d)", name_cn, v);
            else
                ImGui::Text("(%+d)", v);

            // Theta
            ImGui::TableSetColumnIndex(2);
            if (configured)
                ImGui::Text("%.0f°", it->second.theta);
            else {
                ImGui::PushStyleColor(ImGuiCol_Text,
                                      ImVec4(0.45f, 0.45f, 0.45f, 1.0f));
                ImGui::TextUnformatted("-");
                ImGui::PopStyleColor();
            }

            // Phi
            ImGui::TableSetColumnIndex(3);
            if (configured)
                ImGui::Text("%.0f°", it->second.phi);
            else {
                ImGui::PushStyleColor(ImGuiCol_Text,
                                      ImVec4(0.45f, 0.45f, 0.45f, 1.0f));
                ImGui::TextUnformatted("-");
                ImGui::PopStyleColor();
            }

            // Action buttons
            ImGui::TableSetColumnIndex(4);
            char btn_id[48];
            bool is_origin = (x == 0 && y == 0);
            if (configured) {
                snprintf(btn_id, sizeof(btn_id), "%s##e%d_%d",
                         get_locale_cstr("action.angle_edit"), x, y);
                if (ImGui::SmallButton(btn_id))
                    open_edit(it->second.theta, it->second.phi);
                ImGui::SameLine();
                if (is_origin)
                    ImGui::BeginDisabled();
                snprintf(btn_id, sizeof(btn_id), "%s##d%d_%d",
                         get_locale_cstr("action.angle_delete"), x, y);
                if (ImGui::SmallButton(btn_id)) {
                    del_x = x;
                    del_y = y;
                }
                if (is_origin)
                    ImGui::EndDisabled();
            } else {
                snprintf(btn_id, sizeof(btn_id), "%s##a%d_%d",
                         get_locale_cstr("action.angle_add_entry"), x, y);
                if (ImGui::SmallButton(btn_id))
                    open_edit((x == 0) ? 0.0f : x * 9.0f, y * 6.0f);
            }
        }

        ImGui::EndTable();

        // Handle delete outside table
        if (del_x != kSentinel) {
            push_undo_now(item.id, std::nullopt, "Angle Config Delete");
            item.hair_angle_config.erase(
                {static_cast<float>(del_x), static_cast<float>(del_y)});
            rebuild_hair_bvh();
            for (auto& s : item.hair_strands)
                s.mesh_dirty = true;
            if (item.angle_config_editing_x == del_x &&
                item.angle_config_editing_y == del_y) {
                item.angle_config_editing_x = kSentinel;
                item.angle_config_editing_y = kSentinel;
            }
        }
    };

    // ================================================================
    // Two-column layout: left = controls (fixed width),
    //                     right = axis tables (fill remaining)
    // ================================================================
    const float kLeftPanelWidth = 300.0f;
    if (ImGui::BeginTable("##AngleConfigLayout", 2,
                          ImGuiTableFlags_Resizable)) {
        ImGui::TableSetupColumn("##LeftCol", ImGuiTableColumnFlags_WidthFixed,
                                kLeftPanelWidth);
        ImGui::TableSetupColumn("##RightCol",
                                ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableNextRow();

        // ---- Left panel: controls ----
        ImGui::TableSetColumnIndex(0);
        if (ImGui::BeginChild("##AngleConfigLeft", ImVec2(0, 0), false)) {
            // ---- Center Point ----
            if (ImGui::CollapsingHeader(
                    get_locale_cstr("label.addon_center_point"),
                    ImGuiTreeNodeFlags_DefaultOpen)) {
                auto cp_edit = edit_vec3_stepper(
                    get_locale_cstr("label.addon_center_point"),
                    item.addon_center_point, 0.1f);
                if (cp_edit.activated)
                    begin_edit(item.id);
                if (cp_edit.deactivated_after_edit) {
                    end_edit(item.id, "Center Point Edit");
                    for (auto& s : item.hair_strands)
                        s.mesh_dirty = true;
                } else if (cp_edit.value_changed) {
                    push_undo_now(item.id, std::nullopt, "Center Point Edit");
                    for (auto& s : item.hair_strands)
                        s.mesh_dirty = true;
                }
            }

            // ---- Hairline Plane ----
            if (ImGui::CollapsingHeader(get_locale_cstr("label.hairline_plane"),
                                        ImGuiTreeNodeFlags_DefaultOpen)) {
                const char* plane_mode_items[] = {
                    get_locale_cstr("label.hairline_y_plane"),
                    get_locale_cstr("label.hairline_3point_plane"),
                };
                int plane_mode = item.hairline_plane_use_y ? 0 : 1;
                ImGui::SetNextItemWidth(120);
                if (ImGui::Combo(get_locale_cstr("label.hairline_plane_mode"),
                                 &plane_mode, plane_mode_items, 2)) {
                    push_undo_now(item.id, std::nullopt, "Hairline Plane Mode");
                    item.hairline_plane_use_y = (plane_mode == 0);
                }

                if (item.hairline_plane_use_y) {
                    float old_y = item.hairline_plane_y;
                    ImGui::SetNextItemWidth(160);
                    ImGui::DragFloat(get_locale_cstr("label.hairline_y"),
                                     &item.hairline_plane_y, 0.1f);
                    if (ImGui::IsItemActivated())
                        begin_edit(item.id);
                    if (ImGui::IsItemDeactivatedAfterEdit()) {
                        end_edit(item.id, "Hairline Y Edit");
                        for (auto& s : item.hair_strands)
                            s.mesh_dirty = true;
                    } else if (old_y != item.hairline_plane_y) {
                        push_undo_now(item.id, std::nullopt, "Hairline Y Edit");
                    }
                    ImGui::TextDisabled(
                        "%s",
                        get_locale_cstr("label.hairline_preview_triangle"));
                } else {
                    bool pt_activated = false;
                    bool pt_deactivated = false;
                    bool pt_changed = false;

                    for (int pi = 0; pi < 3; ++pi) {
                        ImGui::PushID(pi);
                        char label_buf[64];
                        snprintf(label_buf, sizeof(label_buf),
                                 get_locale_cstr("label.hairline_point"),
                                 pi + 1);
                        auto r = edit_vec3_stepper(
                            label_buf, item.hairline_plane_points[pi], 0.1f);
                        pt_activated |= r.activated;
                        pt_deactivated |= r.deactivated_after_edit;
                        pt_changed |= r.value_changed;

                        ImGui::SameLine();
                        bool is_picking =
                            item.hairline_point_picking_active &&
                            item.hairline_picking_point_index == pi;
                        if (is_picking) {
                            ImGui::PushStyleColor(
                                ImGuiCol_Button,
                                ImVec4(0.2f, 0.5f, 1.0f, 1.0f));
                        }
                        char pick_label[64];
                        snprintf(pick_label, sizeof(pick_label),
                                 is_picking
                                     ? get_locale_cstr("action.picking")
                                     : get_locale_cstr("action.pick_point"),
                                 pi + 1);
                        if (ImGui::SmallButton(pick_label)) {
                            if (is_picking) {
                                item.hairline_point_picking_active = false;
                            } else {
                                item.hairline_point_picking_active = true;
                                item.hairline_picking_point_index = pi;
                            }
                        }
                        if (is_picking)
                            ImGui::PopStyleColor();
                        if (ImGui::IsItemHovered() && !is_picking)
                            ImGui::SetTooltip(
                                "%s", get_locale_cstr("tooltip.pick_point"));
                        ImGui::PopID();
                    }

                    if (pt_activated)
                        begin_edit(item.id);
                    if (pt_deactivated) {
                        end_edit(item.id, "Hairline Points Edit");
                        for (auto& s : item.hair_strands)
                            s.mesh_dirty = true;
                    } else if (pt_changed) {
                        push_undo_now(item.id, std::nullopt,
                                      "Hairline Points Edit");
                    }

                    // Degeneracy warning
                    {
                        const auto& p0 = item.hairline_plane_points[0];
                        const auto& p1 = item.hairline_plane_points[1];
                        const auto& p2 = item.hairline_plane_points[2];
                        vec3f e1{p1.x - p0.x, p1.y - p0.y, p1.z - p0.z};
                        vec3f e2{p2.x - p0.x, p2.y - p0.y, p2.z - p0.z};
                        float area = std::sqrt((e1.y * e2.z - e1.z * e2.y) *
                                                   (e1.y * e2.z - e1.z * e2.y) +
                                               (e1.z * e2.x - e1.x * e2.z) *
                                                   (e1.z * e2.x - e1.x * e2.z) +
                                               (e1.x * e2.y - e1.y * e2.x) *
                                                   (e1.x * e2.y - e1.y * e2.x));
                        if (area < 1e-6f) {
                            ImGui::TextColored(
                                ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "%s",
                                get_locale_cstr("label.hairline_degenerate"));
                        }
                    }
                }
            }

            // ---- North Pole & Front Reference ----
            if (ImGui::CollapsingHeader(
                    get_locale_cstr("label.spherical_frame"),
                    ImGuiTreeNodeFlags_DefaultOpen)) {
                auto np_edit =
                    edit_vec3_stepper(get_locale_cstr("label.north_pole"),
                                      item.hair_north_pole, 0.1f);
                if (np_edit.activated)
                    begin_edit(item.id);
                if (np_edit.deactivated_after_edit) {
                    end_edit(item.id, "North Pole Edit");
                    for (auto& s : item.hair_strands)
                        s.mesh_dirty = true;
                } else if (np_edit.value_changed) {
                    push_undo_now(item.id, std::nullopt, "North Pole Edit");
                    for (auto& s : item.hair_strands)
                        s.mesh_dirty = true;
                }

                auto fr_edit =
                    edit_vec3_stepper(get_locale_cstr("label.front_reference"),
                                      item.hair_front_reference, 0.1f);
                if (fr_edit.activated)
                    begin_edit(item.id);
                if (fr_edit.deactivated_after_edit) {
                    end_edit(item.id, "Front Reference Edit");
                    for (auto& s : item.hair_strands)
                        s.mesh_dirty = true;
                } else if (fr_edit.value_changed) {
                    push_undo_now(item.id, std::nullopt,
                                  "Front Reference Edit");
                    for (auto& s : item.hair_strands)
                        s.mesh_dirty = true;
                }
            }
        }
        ImGui::EndChild();  // AngleConfigLeft

        // ---- Right panel: axis tables (tabbed) ----
        ImGui::TableSetColumnIndex(1);
        if (ImGui::BeginChild("##AngleConfigRight", ImVec2(0, 0), false)) {
            if (ImGui::BeginTabBar("##AxisTabs")) {
                if (ImGui::BeginTabItem(
                        get_locale_cstr("label.angle_x_axis"))) {
                    render_axis_table(get_locale_cstr("label.angle_x_axis"),
                                      true);
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem(
                        get_locale_cstr("label.angle_y_axis"))) {
                    render_axis_table(get_locale_cstr("label.angle_y_axis"),
                                      false);
                    ImGui::EndTabItem();
                }
                ImGui::EndTabBar();
            }
        }
        ImGui::EndChild();  // AngleConfigRight

        ImGui::EndTable();  // AngleConfigLayout
    }

    // ================================================================
    // Direct model-click pick: when editor popup is active and user
    // clicked the 3D model, compute (theta, phi) from the picked point.
    // The popup may be auto-closed by the click (ImGui popup behavior),
    // so we process the pick BEFORE calling OpenPopup to reopen it.
    // ================================================================
    if (item.angle_config_editing_x != kSentinel && mouse_world_pos_picked) {
        vec3f pick_dir = mouse_world_pos - item.addon_center_point;
        float dir_len2 = pick_dir.length2();
        if (dir_len2 > 0.0001f) {
            pick_dir = pick_dir / std::sqrt(dir_len2);
            float pick_theta, pick_phi;
            dir_to_spherical(pick_dir, item.hair_north_pole,
                             item.hair_front_reference, pick_theta, pick_phi);
            item.angle_config_preview_theta = pick_theta;
            item.angle_config_preview_phi = pick_phi;
        }
        mouse_world_pos_picked = false;  // consume the event
    }

    // ================================================================
    // Editor window — uses a regular ImGui window (not a popup) so
    // 3D model clicks pass through for direction picking.
    // ================================================================
    if (item.angle_config_editing_x != kSentinel) {
        ImVec2 win_pos = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(win_pos, ImGuiCond_Appearing,
                                ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(400, 240), ImGuiCond_Appearing);

        char win_title[128];
        snprintf(win_title, sizeof(win_title), "%s##AngleConfigEdit",
                 get_locale_cstr("label.angle_edit_title"));
        bool edit_open = true;
        if (ImGui::Begin(win_title, &edit_open)) {
            int ex = item.angle_config_editing_x;
            int ey = item.angle_config_editing_y;
            if (!edit_open || ex == kSentinel) {
                item.angle_config_editing_x = kSentinel;
                item.angle_config_editing_y = kSentinel;
                ImGui::End();
                ImGui::End();  // outer window
                return;
            }

            auto cfg_it = item.hair_angle_config.find(
                {static_cast<float>(ex), static_cast<float>(ey)});
            bool is_new = (cfg_it == item.hair_angle_config.end());

            const char* anchor = get_anchor_name_cn(ex, ey);
            if (anchor)
                ImGui::Text("%s (%+d, %+d)  %s", anchor, ex, ey,
                            is_new ? "(new)" : "");
            else
                ImGui::Text("%s: (%+d, %+d)  %s",
                            get_locale_cstr("label.angle_edit_title"), ex, ey,
                            is_new ? "(new)" : "");
            ImGui::Separator();

            float& theta = item.angle_config_preview_theta;
            float& phi = item.angle_config_preview_phi;

            bool is_origin = (ex == 0 && ey == 0);
            if (is_origin) {
                // (0,0) is the origin: X=0 midline → theta must be locked at 0°
                theta = 0.0f;
                ImGui::BeginDisabled();
                ImGui::DragFloat(get_locale_cstr("label.angle_theta"), &theta,
                                 1.0f, -180.0f, 180.0f, "%.1f deg");
                ImGui::EndDisabled();
                ImGui::SameLine();
                ImGui::TextDisabled("(locked)");
            } else {
                ImGui::DragFloat(get_locale_cstr("label.angle_theta"), &theta,
                                 1.0f, -180.0f, 180.0f, "%.1f deg");
            }
            ImGui::DragFloat(get_locale_cstr("label.angle_phi"), &phi, 1.0f,
                             -90.0f, 90.0f, "%.1f deg");

            // Show direction preview
            auto dir = sinriv::kigstudio::agent::spherical_to_dir(
                theta, phi, item.hair_north_pole, item.hair_front_reference);
            ImGui::Text("%s: [%.2f, %.2f, %.2f]",
                        get_locale_cstr("label.angle_direction"), dir.x, dir.y,
                        dir.z);

            // Hint: click on model to pick
            ImGui::TextDisabled("%s", get_locale_cstr("label.angle_picking"));

            ImGui::Separator();

            bool validation_failed = false;
            auto& config = item.hair_angle_config;

            if (ImGui::Button(get_locale_cstr("action.angle_apply"))) {
                if (validate_angle_grid(config, ex, ey, theta, phi)) {
                    push_undo_now(item.id, std::nullopt, "Angle Config Edit");
                    config[{static_cast<float>(ex), static_cast<float>(ey)}] =
                        HairAngleEntry{theta, phi};
                    rebuild_hair_bvh();
                    for (auto& s : item.hair_strands)
                        s.mesh_dirty = true;
                    item.angle_config_editing_x = kSentinel;
                    item.angle_config_editing_y = kSentinel;
                } else {
                    validation_failed = true;
                }
            }

            ImGui::SameLine();
            if (ImGui::Button(get_locale_cstr("action.angle_cancel"))) {
                item.angle_config_editing_x = kSentinel;
                item.angle_config_editing_y = kSentinel;
            }

            // Handle window close via the X button
            if (!edit_open) {
                item.angle_config_editing_x = kSentinel;
                item.angle_config_editing_y = kSentinel;
            }

            if (validation_failed) {
                show_toast(get_locale_cstr("error.angle_grid_cross"), 2500.0f);
            }

            ImGui::End();
        }
    }

    ImGui::End();
}

// ============================================================
// Drill path editor window
// ============================================================
void RenderVoxelList::render_drill_window() {
    if (!show_drill_window)
        return;

    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Once, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(560, 480), ImGuiCond_Once);
    bool window_open = true;
    if (!ImGui::Begin(get_locale_cstr("window.drill_edit"), &window_open)) {
        ImGui::End();
        return;
    }

    if (!window_open) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->drill_picking_active = false;
            it->second->active_drill_path_uuid.clear();
        }
        show_drill_window = false;
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
    auto& item = *item_it->second;

    // ---- Connection faces toggle (split addon only) ----
    {
        bool can_show_conn = item.addon_split && item.hair_strands.size() >= 2;
        if (!can_show_conn)
            ImGui::BeginDisabled();
        bool show_conn = item.show_connection_faces;
        if (ImGui::Checkbox(get_locale_cstr("label.connection_faces"),
                            &show_conn)) {
            push_undo_now(item.id, std::nullopt, "Toggle Connection Faces");
            item.show_connection_faces = show_conn;
        }
        if (!can_show_conn)
            ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("%s", can_show_conn
                                        ? get_locale_cstr("tooltip.connection_faces")
                                        : get_locale_cstr("label.drill_need_split"));
    }

    ImGui::Separator();

    // ---- Add drill path ----
    if (ImGui::Button(get_locale_cstr("action.add_drill_path"))) {
        push_undo_now(item.id, std::nullopt, "Add Drill Path");
        DrillPath path;
        path.uuid = generate_uuid();
        char name_buf[64];
        std::snprintf(name_buf, sizeof(name_buf),
                      get_locale_cstr("label.drill_path"),
                      static_cast<int>(item.drill_paths.size()) + 1);
        path.name = name_buf;
        item.drill_paths.push_back(path);
    }

    // ---- Drill path table ----
    static float drill_move_step = 0.5f;
    std::string delete_uuid;
    std::string activate_uuid;  // path whose points table is shown this frame
    if (ImGui::BeginTable("##drill_paths", 6,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_ScrollY,
                          ImVec2(0, 140))) {
        ImGui::TableSetupColumn(get_locale_cstr("label.drill_col_visible"),
                                ImGuiTableColumnFlags_WidthFixed, 40.0f);
        ImGui::TableSetupColumn(get_locale_cstr("label.drill_col_name"),
                                ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn(get_locale_cstr("label.drill_col_points"),
                                ImGuiTableColumnFlags_WidthFixed, 40.0f);
        ImGui::TableSetupColumn(get_locale_cstr("label.drill_col_radius"),
                                ImGuiTableColumnFlags_WidthFixed, 70.0f);
        ImGui::TableSetupColumn(get_locale_cstr("label.drill_col_pick"),
                                ImGuiTableColumnFlags_WidthFixed, 80.0f);
        ImGui::TableSetupColumn(get_locale_cstr("label.drill_col_delete"),
                                ImGuiTableColumnFlags_WidthFixed, 50.0f);
        ImGui::TableHeadersRow();

        for (size_t i = 0; i < item.drill_paths.size(); ++i) {
            auto& path = item.drill_paths[i];
            ImGui::PushID(static_cast<int>(i));
            ImGui::TableNextRow();

            // Column 1: visible checkbox
            ImGui::TableNextColumn();
            bool vis = path.visible;
            if (ImGui::Checkbox("##vis", &vis)) {
                push_undo_now(item.id, std::nullopt, "Toggle Drill Visible");
                path.visible = vis;
                path.mesh_dirty = true;
            }

            // Column 2: name (editable)
            ImGui::TableNextColumn();
            {
                char name_buf[128];
                std::snprintf(name_buf, sizeof(name_buf), "%s",
                              path.name.c_str());
                ImGui::SetNextItemWidth(-1);
                if (ImGui::InputText("##name", name_buf, sizeof(name_buf),
                                     ImGuiInputTextFlags_EnterReturnsTrue)) {
                    push_undo_now(item.id, std::nullopt, "Rename Drill Path");
                    path.name = name_buf;
                }
            }

            // Column 3: point count (click to select as active path)
            ImGui::TableNextColumn();
            {
                bool is_active = item.active_drill_path_uuid == path.uuid;
                char cnt_buf[32];
                std::snprintf(cnt_buf, sizeof(cnt_buf), "%d",
                              static_cast<int>(path.points.size()));
                if (is_active) {
                    activate_uuid = path.uuid;
                    ImGui::PushStyleColor(ImGuiCol_Text,
                                          ImVec4(0.4f, 0.9f, 0.4f, 1.0f));
                }
                if (ImGui::Selectable("##cnt", is_active,
                                      ImGuiSelectableFlags_None,
                                      ImVec2(0, 0))) {
                    item.active_drill_path_uuid = path.uuid;
                }
                ImGui::SameLine(0, 0);
                ImGui::TextUnformatted(cnt_buf);
                if (is_active)
                    ImGui::PopStyleColor();
            }

            // Column 4: radius
            ImGui::TableNextColumn();
            {
                float prev_r = path.radius;
                ImGui::SetNextItemWidth(-1);
                ImGui::DragFloat("##radius", &path.radius, 0.02f, 0.05f,
                                 20.0f, "%.2f");
                if (ImGui::IsItemActivated())
                    begin_edit(item.id);
                if (ImGui::IsItemDeactivatedAfterEdit())
                    end_edit(item.id, "Drill Radius");
                if (prev_r != path.radius)
                    path.mesh_dirty = true;
            }

            // Column 5: pick toggle button
            ImGui::TableNextColumn();
            {
                bool picking = item.drill_picking_active &&
                               item.active_drill_path_uuid == path.uuid;
                if (ImGui::SmallButton(
                        picking
                            ? get_locale_cstr("action.stop_pick_drill_points")
                            : get_locale_cstr("action.pick_drill_points"))) {
                    if (picking) {
                        item.drill_picking_active = false;
                        // Keep active_drill_path_uuid so the point list
                        // stays editable after picking stops.
                    } else {
                        // Mutually exclusive with other picking modes
                        item.guide_curve_drawing_active = false;
                        item.active_guide_draw_strand.clear();
                        item.width_editing_active = false;
                        item.active_width_edit_strand.clear();
                        item.hairline_point_picking_active = false;
                        item.drill_picking_active = true;
                        item.active_drill_path_uuid = path.uuid;
                    }
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "%s", get_locale_cstr("tooltip.pick_drill_points"));
            }

            // Column 6: delete
            ImGui::TableNextColumn();
            if (ImGui::SmallButton("X")) {
                push_undo_now(item.id, std::nullopt, "Delete Drill Path");
                delete_uuid = path.uuid;
                if (item.active_drill_path_uuid == path.uuid) {
                    item.drill_picking_active = false;
                    item.active_drill_path_uuid.clear();
                }
            }

            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (!delete_uuid.empty()) {
        item.drill_paths.erase(
            std::remove_if(item.drill_paths.begin(), item.drill_paths.end(),
                           [&](const DrillPath& p) {
                               return p.uuid == delete_uuid;
                           }),
            item.drill_paths.end());
    }

    // ---- Active path point list ----
    DrillPath* active_path =
        item.find_drill_path_by_uuid(item.active_drill_path_uuid);
    // Fall back to last-touched path shown via activate_uuid
    if (!active_path && !activate_uuid.empty())
        active_path = item.find_drill_path_by_uuid(activate_uuid);

    if (active_path && !active_path->points.empty()) {
        ImGui::Separator();
        ImGui::Text("%s: %s", get_locale_cstr("label.drill_col_points"),
                    active_path->name.c_str());
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ImGui::DragFloat(get_locale_cstr("label.drill_move_step"),
                         &drill_move_step, 0.05f, 0.05f, 50.0f, "%.2f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", get_locale_cstr("tooltip.drill_move_step"));

        auto move_point = [&](int idx, float delta) {
            if (idx < 0 || idx >= static_cast<int>(active_path->points.size()))
                return;
            push_undo_now(item.id, std::nullopt, "Move Drill Point");
            auto& pt = active_path->points[idx];
            vec3f dir = {item.addon_center_point.x - pt.x,
                         item.addon_center_point.y - pt.y,
                         item.addon_center_point.z - pt.z};
            float len = std::sqrt(dir.x * dir.x + dir.y * dir.y +
                                  dir.z * dir.z);
            if (len > 1e-6f) {
                pt.x += dir.x / len * delta;
                pt.y += dir.y / len * delta;
                pt.z += dir.z / len * delta;
            }
            active_path->mesh_dirty = true;
        };

        // Keyboard +/- moves the last picked point
        if (!ImGui::GetIO().WantTextInput && item.drill_last_picked_index >= 0) {
            if (ImGui::IsKeyPressed(ImGuiKey_Equal, true) ||
                ImGui::IsKeyPressed(ImGuiKey_KeypadAdd, true)) {
                move_point(item.drill_last_picked_index, drill_move_step);
            }
            if (ImGui::IsKeyPressed(ImGuiKey_Minus, true) ||
                ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract, true)) {
                move_point(item.drill_last_picked_index, -drill_move_step);
            }
        }

        int delete_idx = -1;
        float pt_table_h = ImGui::GetContentRegionAvail().y;
        if (ImGui::BeginTable("##drill_pts", 4,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_ScrollY,
                              ImVec2(0, pt_table_h))) {
            ImGui::TableSetupColumn(get_locale_cstr("label.drill_col_index"),
                                    ImGuiTableColumnFlags_WidthFixed, 30.0f);
            ImGui::TableSetupColumn(get_locale_cstr("label.drill_col_position"),
                                    ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn(get_locale_cstr("label.drill_col_ops"),
                                    ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableSetupColumn(get_locale_cstr("label.drill_col_delete"),
                                    ImGuiTableColumnFlags_WidthFixed, 40.0f);
            ImGui::TableHeadersRow();

            for (size_t pi = 0; pi < active_path->points.size(); ++pi) {
                const auto& pt = active_path->points[pi];
                ImGui::PushID(static_cast<int>(pi));
                ImGui::TableNextRow();

                ImGui::TableNextColumn();
                if (item.drill_last_picked_index == static_cast<int>(pi))
                    ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1.0f), "%d",
                                       static_cast<int>(pi + 1));
                else
                    ImGui::Text("%d", static_cast<int>(pi + 1));

                ImGui::TableNextColumn();
                ImGui::Text("(%.2f, %.2f, %.2f)", static_cast<double>(pt.x),
                            static_cast<double>(pt.y),
                            static_cast<double>(pt.z));

                ImGui::TableNextColumn();
                if (ImGui::SmallButton("+")) {
                    item.drill_last_picked_index = static_cast<int>(pi);
                    move_point(static_cast<int>(pi), drill_move_step);
                }
                ImGui::SameLine();
                if (ImGui::SmallButton("-")) {
                    item.drill_last_picked_index = static_cast<int>(pi);
                    move_point(static_cast<int>(pi), -drill_move_step);
                }

                ImGui::TableNextColumn();
                if (ImGui::SmallButton("X"))
                    delete_idx = static_cast<int>(pi);

                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        if (delete_idx >= 0) {
            push_undo_now(item.id, std::nullopt, "Delete Drill Point");
            active_path->points.erase(active_path->points.begin() + delete_idx);
            active_path->mesh_dirty = true;
            if (item.drill_last_picked_index >= delete_idx)
                item.drill_last_picked_index =
                    static_cast<int>(active_path->points.size()) - 1;
        }
    } else if (active_path) {
        ImGui::TextDisabled("%s", get_locale_cstr("label.drill_no_points"));
    }

    ImGui::End();
}
}  // namespace sinriv::ui::render