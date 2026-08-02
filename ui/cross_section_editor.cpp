#include "cross_section_editor.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "imgui.h"
#include "render_voxel_list.h"

namespace sinriv::ui::render {

// ============================================================================
// SectionEditorState — undo/redo
// ============================================================================

void SectionEditorState::push_undo(const std::string& desc) {
    undo_stack.push_back({vertices, desc});
    redo_stack.clear();
    committed.clear();  // editing after Apply → no longer applied
    if (undo_stack.size() > kMaxUndoSize) {
        undo_stack.erase(undo_stack.begin());
    }
}

bool SectionEditorState::undo() {
    if (undo_stack.empty())
        return false;
    // Save current state to redo stack (preserve the undo entry's description)
    auto redo_snap = undo_stack.back();
    redo_snap.vertices = vertices;
    redo_stack.push_back(std::move(redo_snap));
    // Restore undo state
    vertices = undo_stack.back().vertices;
    undo_stack.pop_back();
    committed.clear();  // undo changes → no longer applied
    return true;
}

bool SectionEditorState::redo() {
    if (redo_stack.empty())
        return false;
    // Save current state to undo stack
    auto undo_snap = redo_stack.back();
    undo_snap.vertices = vertices;
    undo_stack.push_back(std::move(undo_snap));
    // Restore redo state
    vertices = redo_stack.back().vertices;
    redo_stack.pop_back();
    committed.clear();  // redo changes → no longer applied
    return true;
}

bool SectionEditorState::can_undo() const {
    return !undo_stack.empty();
}

bool SectionEditorState::can_redo() const {
    return !redo_stack.empty();
}

// ============================================================================
// Helper: point-to-segment squared distance (2D)
// ============================================================================
namespace {

float point_segment_distance_sq(const vec2f& p, const vec2f& a,
                                const vec2f& b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float len_sq = dx * dx + dy * dy;
    if (len_sq < 1e-12f)
        return (p.x - a.x) * (p.x - a.x) + (p.y - a.y) * (p.y - a.y);
    float t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq;
    t = std::clamp(t, 0.0f, 1.0f);
    float proj_x = a.x + t * dx;
    float proj_y = a.y + t * dy;
    return (p.x - proj_x) * (p.x - proj_x) +
           (p.y - proj_y) * (p.y - proj_y);
}

float point_distance_sq(const vec2f& a, const vec2f& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return dx * dx + dy * dy;
}

// Orientation of triplet (a, b, c). Positive = CCW, negative = CW, zero = collinear.
float orientation(const vec2f& a, const vec2f& b, const vec2f& c) {
    return (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
}

// Check if two segments (a,b) and (c,d) intersect (strict, excludes
// collinear/endpoint-only touches which are fine for our use case).
bool segments_intersect(const vec2f& a, const vec2f& b, const vec2f& c,
                        const vec2f& d) {
    float o1 = orientation(a, b, c);
    float o2 = orientation(a, b, d);
    float o3 = orientation(c, d, a);
    float o4 = orientation(c, d, b);
    // Strict intersection: signs must be opposite on both sides.
    return (o1 * o2 < 0.0f) && (o3 * o4 < 0.0f);
}

// Check if a polygon self-intersects (any two non-adjacent edges cross).
bool self_intersects(const std::vector<vec2f>& poly) {
    int n = static_cast<int>(poly.size());
    if (n < 4)
        return false;
    for (int i = 0; i < n; ++i) {
        int i1 = (i + 1) % n;
        for (int j = i + 2; j < n; ++j) {
            // Skip the closing edge that wraps back to i
            if (i == 0 && j == n - 1)
                continue;
            int j1 = (j + 1) % n;
            if (segments_intersect(poly[i], poly[i1], poly[j], poly[j1]))
                return true;
        }
    }
    return false;
}

}  // namespace

// ============================================================================
// Helper: ear-clipping triangulation for concave polygon fill
// ============================================================================
namespace {

// Signed area of triangle (a, b, c). Positive = CCW, negative = CW.
float signed_area(const ImVec2& a, const ImVec2& b, const ImVec2& c) {
    return (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
}

// Check if point p is inside triangle (a, b, c) using barycentric technique.
bool point_in_triangle(const ImVec2& p, const ImVec2& a, const ImVec2& b,
                       const ImVec2& c) {
    float d0 = signed_area(p, a, b);
    float d1 = signed_area(p, b, c);
    float d2 = signed_area(p, c, a);
    bool has_neg = (d0 < 0) || (d1 < 0) || (d2 < 0);
    bool has_pos = (d0 > 0) || (d1 > 0) || (d2 > 0);
    return !(has_neg && has_pos);
}

// Ear-clipping triangulation.  Returns a list of index triples into `poly`.
// Assumes the polygon is simple (no self-intersections) and vertices are
// in CCW order.
std::vector<std::array<int, 3>> triangulate_ear_clip(
    const std::vector<ImVec2>& poly) {
    std::vector<std::array<int, 3>> result;
    int n = static_cast<int>(poly.size());
    if (n < 3)
        return result;
    if (n == 3) {
        result.push_back({0, 1, 2});
        return result;
    }

    // Build working index list.
    std::vector<int> idx(n);
    for (int i = 0; i < n; ++i)
        idx[i] = i;

    // Determine polygon winding order.
    float area = 0;
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        area += poly[idx[i]].x * poly[idx[j]].y;
        area -= poly[idx[j]].x * poly[idx[i]].y;
    }
    bool ccw = (area > 0);

    // Ear-clipping loop.
    int iter = 0;
    int max_iter = n * n;  // safety limit
    while (static_cast<int>(idx.size()) > 3 && iter < max_iter) {
        ++iter;
        int m = static_cast<int>(idx.size());
        bool ear_found = false;
        for (int i = 0; i < m; ++i) {
            int prev = (i == 0) ? m - 1 : i - 1;
            int next = (i + 1) % m;

            const ImVec2& a = poly[idx[prev]];
            const ImVec2& b = poly[idx[i]];
            const ImVec2& c = poly[idx[next]];

            // Check if this vertex forms a convex (reflex-free) corner.
            float sa = signed_area(a, b, c);
            if (ccw && sa <= 0)
                continue;  // reflex or collinear when polygon is CCW
            if (!ccw && sa >= 0)
                continue;  // reflex or collinear when polygon is CW

            // Check that no other vertex lies inside triangle (a, b, c).
            bool is_ear = true;
            for (int j = 0; j < m; ++j) {
                if (j == prev || j == i || j == next)
                    continue;
                if (point_in_triangle(poly[idx[j]], a, b, c)) {
                    is_ear = false;
                    break;
                }
            }

            if (is_ear) {
                result.push_back({idx[prev], idx[i], idx[next]});
                idx.erase(idx.begin() + i);
                ear_found = true;
                break;
            }
        }
        if (!ear_found)
            break;  // should not happen for a simple polygon
    }

    // Last triangle.
    if (static_cast<int>(idx.size()) == 3) {
        result.push_back({idx[0], idx[1], idx[2]});
    }

    return result;
}

}  // namespace
static const ImU32 kColorBackground = IM_COL32(25, 25, 35, 255);
static const ImU32 kColorGrid = IM_COL32(45, 45, 55, 255);
static const ImU32 kColorPolyFill = IM_COL32(40, 80, 160, 60);
static const ImU32 kColorEdge = IM_COL32(200, 200, 210, 255);
static const ImU32 kColorEdgeHover = IM_COL32(255, 220, 80, 255);
static const ImU32 kColorVertex = IM_COL32(220, 220, 230, 255);
static const ImU32 kColorVertexHover = IM_COL32(255, 220, 80, 255);
static const ImU32 kColorVertexDrag = IM_COL32(255, 150, 50, 255);
static const ImU32 kColorBezierPreview = IM_COL32(100, 220, 255, 200);

// ============================================================================
// render_cross_section_editor
// ============================================================================
void RenderVoxelList::render_cross_section_editor() {
    if (!show_addon_window)
        return;
    if (!show_cross_section_editor_window)
        return;

    // Mutual exclusion: close other editors when section editor opens
    if (show_guide_curve_window) {
        auto git = items.find(render_id);
        if (git != items.end()) {
            git->second->guide_curve_drawing_active = false;
            git->second->active_guide_draw_strand = -1;
        }
        show_guide_curve_window = false;
    }
    if (show_perpoint_section_editor_window) {
        auto pit = items.find(render_id);
        if (pit != items.end()) {
            pit->second->perpoint_section_editing_active = false;
            pit->second->active_perpoint_section_edit_strand = -1;
            pit->second->active_perpoint_section_edit_width_idx = -1;
        }
        show_perpoint_section_editor_window = false;
    }
    if (show_hairline_plane_window) {
        auto hit = items.find(render_id);
        if (hit != items.end()) {
            hit->second->hairline_point_picking_active = false;
        }
        show_hairline_plane_window = false;
    }

    // Window setup
    ImGui::SetNextWindowSize(ImVec2(480, 520), ImGuiCond_Once);
    bool window_open = true;
    if (!ImGui::Begin(get_locale_cstr("window.cross_section_editor"),
                      &window_open)) {
        ImGui::End();
        return;
    }

    // Handle close button
    if (!window_open) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->active_section_edit_strand = -1;
        }
        show_cross_section_editor_window = false;
        ImGui::End();
        return;
    }

    // --- Acquire lock and validate state ---
    std::lock_guard<std::mutex> lock(locker);
    auto item_it = items.find(render_id);
    if (item_it == items.end() || item_it->second->source_type != 2) {
        ImGui::TextUnformatted(get_locale_cstr("label.no_active_item"));
        ImGui::End();
        return;
    }

    RenderVoxelItem& item = *item_it->second;
    int idx = item.active_section_edit_strand;

    if (idx < 0 || idx >= static_cast<int>(item.hair_strands.size())) {
        show_cross_section_editor_window = false;
        ImGui::End();
        return;
    }

    auto& strand = item.hair_strands[idx];
    auto& state = strand.section_state;
    auto& verts = state.vertices;

    // Ensure we have at least 3 vertices (closed polygon)
    if (verts.size() < 3) {
        // If committed path exists, use it; otherwise default rectangle (w=1, h=0.5)
        if (state.committed.size() >= 3) {
            verts = state.committed;
        } else {
            verts.clear();
            verts.push_back({-0.5f, -0.25f});
            verts.push_back({0.5f, -0.25f});
            verts.push_back({0.5f, 0.25f});
            verts.push_back({-0.5f, 0.25f});
        }
    }

    // --- Self-intersection check (run every frame while editing) ---
    bool is_self_intersecting = self_intersects(verts);

    // --- Title line ---
    ImGui::Text(get_locale_cstr("label.hair_strand"), idx + 1);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "%s",
                       get_locale_cstr("action.edit_section"));

    // --- Undo / Redo buttons ---
    {
        bool undo_disabled = !state.can_undo();
        bool redo_disabled = !state.can_redo();
        if (undo_disabled)
            ImGui::BeginDisabled();
        if (ImGui::SmallButton(get_locale_cstr("action.undo"))) {
            state.undo();
            strand.mesh_dirty = true;
        }
        if (undo_disabled)
            ImGui::EndDisabled();
        ImGui::SameLine();
        if (redo_disabled)
            ImGui::BeginDisabled();
        if (ImGui::SmallButton(get_locale_cstr("action.redo"))) {
            state.redo();
            strand.mesh_dirty = true;
        }
        if (redo_disabled)
            ImGui::EndDisabled();
    }

    ImGui::SameLine();
    ImGui::Text(get_locale_cstr("label.cross_section_vertices"),
                static_cast<int>(verts.size()));

    // Bézier interpolation toggle
    ImGui::SameLine();
    bool old_bezier = state.use_bezier_section;
    ImGui::Checkbox(get_locale_cstr("label.use_bezier_section"),
                    &state.use_bezier_section);
    if (old_bezier != state.use_bezier_section) {
        strand.mesh_dirty = true;
    }

    // Normalize mode combo
    ImGui::SameLine();
    int old_norm = static_cast<int>(state.normalize_mode);
    const char* norm_items[] = {
        get_locale_cstr("label.normalize_x"),
        get_locale_cstr("label.normalize_y"),
        get_locale_cstr("label.normalize_xy"),
    };
    ImGui::SetNextItemWidth(60);
    int norm_mode = old_norm;
    ImGui::Combo(get_locale_cstr("label.normalize_mode"), &norm_mode,
                 norm_items, IM_ARRAYSIZE(norm_items));
    if (norm_mode != old_norm) {
        state.normalize_mode = static_cast<NormalizeMode>(norm_mode);
    }

    // Section rotation
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    float old_rot = strand.section_rotation;
    ImGui::SliderFloat(get_locale_cstr("label.section_rotation"),
                       &strand.section_rotation, -180.0f, 180.0f, "%.0f");
    if (old_rot != strand.section_rotation) {
        strand.mesh_dirty = true;
    }

    ImGui::Separator();

    // --- Canvas ---
    const float kCanvasPadding = 20.0f;
    const float kFitMargin = 0.85f;  // polygon occupies 85% of canvas area
    const float kVertexHitRadius = 8.0f;
    const float kEdgeHitDist = 8.0f;
    const float kEdgeEndpointMargin = 12.0f;  // avoid edge hit near endpoints

    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    float canvas_w = avail.x - kCanvasPadding * 2;
    // Leave room for hint text + optional error + Apply button
    float canvas_h = avail.y - 60.0f;
    if (canvas_w < 100.0f)
        canvas_w = 300.0f;
    if (canvas_h < 200.0f)
        canvas_h = 300.0f;

    ImVec2 canvas_size(canvas_w, canvas_h);

    // --- InvisibleButton as interactive canvas ---
    ImGui::InvisibleButton("##SectionCanvas", canvas_size,
                           ImGuiButtonFlags_MouseButtonLeft |
                               ImGuiButtonFlags_MouseButtonRight);
    bool canvas_hovered = ImGui::IsItemHovered();
    bool canvas_active = ImGui::IsItemActive();
    bool canvas_activated = ImGui::IsItemActivated();
    bool canvas_deactivated = ImGui::IsItemDeactivatedAfterEdit();

    ImVec2 canvas_min = ImGui::GetItemRectMin();
    ImVec2 canvas_max = ImGui::GetItemRectMax();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // --- Coordinate mapping ---
    // Compute world-space bounding box
    float bb_min_x = verts[0].x, bb_min_y = verts[0].y;
    float bb_max_x = verts[0].x, bb_max_y = verts[0].y;
    for (const auto& v : verts) {
        bb_min_x = std::min(bb_min_x, v.x);
        bb_min_y = std::min(bb_min_y, v.y);
        bb_max_x = std::max(bb_max_x, v.x);
        bb_max_y = std::max(bb_max_y, v.y);
    }

    float bb_w = bb_max_x - bb_min_x;
    float bb_h = bb_max_y - bb_min_y;

    // Degenerate case: expand to a minimal viewport
    if (bb_w < 1e-6f)
        bb_w = 1.0f;
    if (bb_h < 1e-6f)
        bb_h = 1.0f;

    // Scale to fit canvas with margin
    float scale = std::min(canvas_w / bb_w, canvas_h / bb_h) * kFitMargin;

    // Center the polygon in world space, then offset
    float bb_cx = (bb_min_x + bb_max_x) * 0.5f;
    float bb_cy = (bb_min_y + bb_max_y) * 0.5f;
    float canvas_cx = canvas_min.x + canvas_w * 0.5f;
    float canvas_cy = canvas_min.y + canvas_h * 0.5f;

    auto world_to_canvas = [&](const vec2f& v) -> ImVec2 {
        return ImVec2(canvas_cx + (v.x - bb_cx) * scale,
                      canvas_cy + (v.y - bb_cy) * scale);
    };

    auto canvas_to_world = [&](const ImVec2& p) -> vec2f {
        return vec2f{bb_cx + (p.x - canvas_cx) / scale,
                     bb_cy + (p.y - canvas_cy) / scale};
    };

    // --- Draw background ---
    dl->AddRectFilled(canvas_min, canvas_max, kColorBackground);

    // --- Draw grid ---
    const float kGridSpacing = 30.0f;
    for (float gx = canvas_min.x; gx <= canvas_max.x; gx += kGridSpacing) {
        dl->AddLine(ImVec2(gx, canvas_min.y), ImVec2(gx, canvas_max.y),
                    kColorGrid, 0.5f);
    }
    for (float gy = canvas_min.y; gy <= canvas_max.y; gy += kGridSpacing) {
        dl->AddLine(ImVec2(canvas_min.x, gy), ImVec2(canvas_max.x, gy),
                    kColorGrid, 0.5f);
    }

    // --- Draw polygon fill (ear-clipping triangulation for concave support) ---
    if (verts.size() >= 3) {
        std::vector<ImVec2> canvas_verts;
        canvas_verts.reserve(verts.size());
        for (const auto& v : verts) {
            canvas_verts.push_back(world_to_canvas(v));
        }
        // Triangulate and draw each triangle individually
        auto tris = triangulate_ear_clip(canvas_verts);
        for (const auto& tri : tris) {
            dl->AddTriangleFilled(canvas_verts[tri[0]], canvas_verts[tri[1]],
                                  canvas_verts[tri[2]], kColorPolyFill);
        }
    }

    // --- Hit-testing (only when canvas is hovered) ---
    state.reset_frame_state();

    if (canvas_hovered) {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        float vert_hit_sq = kVertexHitRadius * kVertexHitRadius;
        float edge_hit_sq = kEdgeHitDist * kEdgeHitDist;
        float edge_ep_margin_sq = kEdgeEndpointMargin * kEdgeEndpointMargin;

        // Check vertices
        for (size_t i = 0; i < verts.size(); ++i) {
            ImVec2 cv = world_to_canvas(verts[i]);
            float dx = mouse_pos.x - cv.x;
            float dy = mouse_pos.y - cv.y;
            if (dx * dx + dy * dy < vert_hit_sq) {
                state.hovered_vertex = static_cast<int>(i);
                break;  // vertex priority
            }
        }

        // Check edges (only if no vertex is hovered)
        if (state.hovered_vertex < 0 && verts.size() >= 2) {
            for (size_t i = 0; i < verts.size(); ++i) {
                size_t j = (i + 1) % verts.size();
                vec2f mouse_world = canvas_to_world(mouse_pos);

                float dist_sq = point_segment_distance_sq(mouse_world, verts[i],
                                                          verts[j]);
                if (dist_sq < edge_hit_sq / (scale * scale)) {
                    // Exclude near-endpoint hits
                    float d1_sq = point_distance_sq(mouse_world, verts[i]);
                    float d2_sq = point_distance_sq(mouse_world, verts[j]);
                    if (d1_sq > edge_ep_margin_sq / (scale * scale) &&
                        d2_sq > edge_ep_margin_sq / (scale * scale)) {
                        state.hovered_edge = static_cast<int>(i);
                        break;
                    }
                }
            }
        }

        // --- Mouse interaction ---
        bool left_clicked =
            ImGui::IsMouseClicked(ImGuiMouseButton_Left) && canvas_hovered;
        bool right_clicked =
            ImGui::IsMouseClicked(ImGuiMouseButton_Right) && canvas_hovered;

        // Left click on edge → add vertex
        if (left_clicked && state.hovered_edge >= 0 &&
            state.hovered_vertex < 0 && state.dragged_vertex < 0) {
            vec2f new_vert = canvas_to_world(ImGui::GetMousePos());
            size_t insert_idx = static_cast<size_t>(state.hovered_edge) + 1;
            if (insert_idx > verts.size())
                insert_idx = verts.size();
            state.push_undo("Add Vertex");
            verts.insert(verts.begin() + static_cast<ptrdiff_t>(insert_idx),
                         new_vert);
            state.hovered_edge = -1;
        }

        // Left click on vertex → start drag
        if (left_clicked && state.hovered_vertex >= 0) {
            state.dragged_vertex = state.hovered_vertex;
            ImVec2 cv = world_to_canvas(verts[state.hovered_vertex]);
            state.drag_offset = vec2f{mouse_pos.x - cv.x, mouse_pos.y - cv.y};
        }

        // Right click on vertex → delete (if > 3 vertices)
        if (right_clicked && state.hovered_vertex >= 0) {
            if (verts.size() > 3) {
                state.push_undo("Delete Vertex");
                verts.erase(verts.begin() + state.hovered_vertex);
                state.hovered_vertex = -1;
                state.dragged_vertex = -1;
            }
        }
    }

    // --- Drag vertex ---
    if (state.dragged_vertex >= 0) {
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) && canvas_hovered) {
            // Push undo on drag start
            if (!state.edit_active) {
                state.push_undo("Drag Vertex");
                state.edit_active = true;
            }
            ImVec2 mouse_pos = ImGui::GetMousePos();
            vec2f new_pos = canvas_to_world(
                ImVec2(mouse_pos.x - state.drag_offset.x,
                       mouse_pos.y - state.drag_offset.y));
            verts[state.dragged_vertex] = new_pos;
        } else if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            // Mouse released → end drag
            state.dragged_vertex = -1;
            state.edit_active = false;
        }
    }

    // --- Draw edges ---
    for (size_t i = 0; i < verts.size(); ++i) {
        size_t j = (i + 1) % verts.size();
        ImVec2 a = world_to_canvas(verts[i]);
        ImVec2 b = world_to_canvas(verts[j]);
        bool is_hovered = (state.hovered_edge == static_cast<int>(i));
        ImU32 color = is_hovered ? kColorEdgeHover : kColorEdge;
        float thickness = is_hovered ? 2.5f : 1.5f;
        dl->AddLine(a, b, color, thickness);

        // Edge midpoint hint
        if (is_hovered) {
            ImVec2 mid((a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f);
            dl->AddCircleFilled(mid, 3.5f, kColorEdgeHover);
        }
    }

    // --- Draw Bézier curve preview overlay ---
    // When Bézier mode is on, overlay the Catmull-Rom smoothed curve so the
    // user can preview the shape the loft mesh will actually use. The formula
    // and kSubdiv=8 must match build_hair_strand_mesh() in render_voxel_render.cpp.
    if (state.use_bezier_section && verts.size() >= 3) {
        const int n = static_cast<int>(verts.size());
        constexpr int kSubdiv = 8;

        auto catmull_rom_2d = [](float p0, float p1, float p2, float p3,
                                 float t) -> float {
            float t2 = t * t;
            float t3 = t2 * t;
            return 0.5f *
                   ((2.0f * p1) + (-p0 + p2) * t +
                    (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                    (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
        };

        // Generate smoothed points on the closed polygon
        std::vector<ImVec2> smooth_pts;
        smooth_pts.reserve(n * kSubdiv + 1);
        for (int i = 0; i < n; ++i) {
            int i0 = (i - 1 + n) % n;
            int i1 = i;
            int i2 = (i + 1) % n;
            int i3 = (i + 2) % n;

            const auto& p0 = verts[i0];
            const auto& p1 = verts[i1];
            const auto& p2 = verts[i2];
            const auto& p3 = verts[i3];

            for (int s = 0; s < kSubdiv; ++s) {
                float t = static_cast<float>(s) /
                          static_cast<float>(kSubdiv);
                float sx = catmull_rom_2d(p0.x, p1.x, p2.x, p3.x, t);
                float sy = catmull_rom_2d(p0.y, p1.y, p2.y, p3.y, t);
                smooth_pts.push_back(world_to_canvas(vec2f{sx, sy}));
            }
        }

        // Close the loop and draw as a smooth polyline
        if (smooth_pts.size() >= 2) {
            smooth_pts.push_back(smooth_pts[0]);
            dl->AddPolyline(smooth_pts.data(),
                            static_cast<int>(smooth_pts.size()),
                            kColorBezierPreview, 0, 2.5f);
        }
    }

    // --- Draw vertices ---
    for (size_t i = 0; i < verts.size(); ++i) {
        ImVec2 cv = world_to_canvas(verts[i]);
        ImU32 color = kColorVertex;
        float radius = 5.0f;

        if (state.dragged_vertex == static_cast<int>(i)) {
            color = kColorVertexDrag;
            radius = 7.0f;
        } else if (state.hovered_vertex == static_cast<int>(i)) {
            color = kColorVertexHover;
            radius = 7.0f;
        }

        dl->AddCircleFilled(cv, radius, color);
        dl->AddCircle(cv, radius, IM_COL32(0, 0, 0, 100), 0, 1.5f);
    }

    ImGui::Separator();

    // --- Self-intersection error ---
    if (is_self_intersecting) {
        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%s",
                           get_locale_cstr("label.cross_section_error"));
    }

    // --- Apply button ---
    {
        bool apply_disabled = is_self_intersecting || verts.size() < 3;
        if (apply_disabled)
            ImGui::BeginDisabled();
        if (ImGui::Button(get_locale_cstr("action.apply_section"))) {
            // Check for per-point section overrides before applying
            bool has_overrides = false;
            for (const auto& wp : strand.width_points) {
                if (wp.section_state.vertices.size() >= 3) {
                    has_overrides = true;
                    break;
                }
            }
            if (has_overrides) {
                show_perpoint_confirm_global_apply = true;
                pending_global_section_strand = idx;
            } else {
            state.push_undo("Apply");
            // Normalize section vertices according to selected mode
            {
                float min_x = verts[0].x, max_x = verts[0].x;
                float min_y = verts[0].y, max_y = verts[0].y;
                for (const auto& v : verts) {
                    if (v.x < min_x) min_x = v.x;
                    if (v.x > max_x) max_x = v.x;
                    if (v.y < min_y) min_y = v.y;
                    if (v.y > max_y) max_y = v.y;
                }
                float range_x = max_x - min_x;
                float range_y = max_y - min_y;
                if (range_x < 1e-8f) range_x = 1e-8f;
                if (range_y < 1e-8f) range_y = 1e-8f;
                float cx = (min_x + max_x) * 0.5f;
                float cy = (min_y + max_y) * 0.5f;
                float half_x = range_x * 0.5f;
                float half_y = range_y * 0.5f;
                float scale;
                switch (state.normalize_mode) {
                case NormalizeMode::NORMALIZE_X:
                    scale = half_x;  // X half-extent → 1, Y scaled proportionally
                    break;
                case NormalizeMode::NORMALIZE_Y:
                    scale = half_y;  // Y half-extent → 1, X scaled proportionally
                    break;
                case NormalizeMode::NORMALIZE_XY:
                default:
                    scale = std::max(half_x, half_y);  // max dim half-extent → 1
                    break;
                }
                if (scale < 1e-8f) scale = 1e-8f;
                for (auto& v : verts) {
                    v.x = (v.x - cx) / scale;
                    v.y = (v.y - cy) / scale;
                }
            }
            state.committed = verts;
            strand.mesh_dirty = true;
            }  // else (no overrides → apply directly)
        }
        if (apply_disabled)
            ImGui::EndDisabled();

        ImGui::SameLine();
        if (state.committed.size() >= 3) {
            ImGui::TextColored(ImVec4(0.5f, 0.9f, 0.5f, 1.0f), "%s",
                               get_locale_cstr("label.cross_section_applied"));
        } else {
            ImGui::TextDisabled("%s",
                                get_locale_cstr("label.cross_section_not_applied"));
        }
    }

    // --- Hint text ---
    ImGui::TextWrapped("%s",
                       get_locale_cstr("label.cross_section_hint"));

    // --- Keyboard shortcuts (Ctrl+Z/Y) ---
    if (ImGui::IsWindowFocused()) {
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Z)) {
            state.undo();
            strand.mesh_dirty = true;
        }
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Y)) {
            state.redo();
            strand.mesh_dirty = true;
        }
    }

    ImGui::End();
}

void RenderVoxelList::render_perpoint_section_editor() {
    if (!show_addon_window)
        return;
    if (!show_perpoint_section_editor_window)
        return;

    // Mutual exclusion: close other editors when per-point section opens
    if (show_guide_curve_window) {
        auto git = items.find(render_id);
        if (git != items.end()) {
            git->second->guide_curve_drawing_active = false;
            git->second->active_guide_draw_strand = -1;
        }
        show_guide_curve_window = false;
    }
    if (show_cross_section_editor_window) {
        auto sit = items.find(render_id);
        if (sit != items.end()) {
            sit->second->active_section_edit_strand = -1;
        }
        show_cross_section_editor_window = false;
    }
    if (show_hairline_plane_window) {
        auto hit = items.find(render_id);
        if (hit != items.end()) {
            hit->second->hairline_point_picking_active = false;
        }
        show_hairline_plane_window = false;
    }

    ImGui::SetNextWindowSize(ImVec2(480, 520), ImGuiCond_Once);
    bool window_open = true;
    if (!ImGui::Begin(get_locale_cstr("window.perpoint_section_editor"),
                      &window_open)) {
        ImGui::End();
        return;
    }

    // Handle close button
    if (!window_open) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->perpoint_section_editing_active = false;
            it->second->active_perpoint_section_edit_strand = -1;
            it->second->active_perpoint_section_edit_width_idx = -1;
        }
        show_perpoint_section_editor_window = false;
        ImGui::End();
        return;
    }

    // --- Acquire lock and validate state ---
    std::lock_guard<std::mutex> lock(locker);
    auto item_it = items.find(render_id);
    if (item_it == items.end() || item_it->second->source_type != 2) {
        ImGui::TextUnformatted(get_locale_cstr("label.no_active_item"));
        ImGui::End();
        return;
    }

    RenderVoxelItem& item = *item_it->second;
    int strand_idx = item.active_perpoint_section_edit_strand;
    int wp_idx = item.active_perpoint_section_edit_width_idx;

    if (strand_idx < 0 ||
        strand_idx >= static_cast<int>(item.hair_strands.size()) ||
        wp_idx < 0 || wp_idx >= static_cast<int>(
                          item.hair_strands[strand_idx].width_points.size())) {
        show_perpoint_section_editor_window = false;
        ImGui::End();
        return;
    }

    auto& strand = item.hair_strands[strand_idx];
    auto& wp = strand.width_points[wp_idx];
    auto& state = wp.section_state;
    auto& verts = state.vertices;

    // Seed initial vertices from global section if unset
    if (verts.size() < 3) {
        if (strand.section_state.committed.size() >= 3)
            verts = strand.section_state.committed;
        else if (strand.section_state.vertices.size() >= 3)
            verts = strand.section_state.vertices;
        else {
            verts.clear();
            verts.push_back({-0.5f, -0.25f});
            verts.push_back({0.5f, -0.25f});
            verts.push_back({0.5f, 0.25f});
            verts.push_back({-0.5f, 0.25f});
        }
    }

    // Self-intersection check
    bool is_self_intersecting = self_intersects(verts);

    // --- Title line ---
    ImGui::Text(get_locale_cstr("label.hair_strand"), strand_idx + 1);
    ImGui::SameLine();
    ImGui::Text(get_locale_cstr("label.width_point_entry"),
                wp_idx + 1, static_cast<int>(wp.curve_id));

    // --- Undo / Redo buttons ---
    {
        bool undo_disabled = !state.can_undo();
        bool redo_disabled = !state.can_redo();
        if (undo_disabled)
            ImGui::BeginDisabled();
        if (ImGui::SmallButton(get_locale_cstr("action.undo"))) {
            state.undo();
            strand.mesh_dirty = true;
        }
        if (undo_disabled)
            ImGui::EndDisabled();
        ImGui::SameLine();
        if (redo_disabled)
            ImGui::BeginDisabled();
        if (ImGui::SmallButton(get_locale_cstr("action.redo"))) {
            state.redo();
            strand.mesh_dirty = true;
        }
        if (redo_disabled)
            ImGui::EndDisabled();
    }

    ImGui::SameLine();
    ImGui::Text(get_locale_cstr("label.cross_section_vertices"),
                static_cast<int>(verts.size()));

    // Bézier toggle (per-point)
    ImGui::SameLine();
    bool old_bezier = state.use_bezier_section;
    ImGui::Checkbox(get_locale_cstr("label.use_bezier_section"),
                    &state.use_bezier_section);
    if (old_bezier != state.use_bezier_section) {
        strand.mesh_dirty = true;
    }

    // Normalize mode combo (per-point)
    ImGui::SameLine();
    int old_norm_pp = static_cast<int>(state.normalize_mode);
    const char* norm_items_pp[] = {
        get_locale_cstr("label.normalize_x"),
        get_locale_cstr("label.normalize_y"),
        get_locale_cstr("label.normalize_xy"),
    };
    ImGui::SetNextItemWidth(60);
    int norm_mode_pp = old_norm_pp;
    ImGui::Combo(get_locale_cstr("label.normalize_mode"), &norm_mode_pp,
                 norm_items_pp, IM_ARRAYSIZE(norm_items_pp));
    if (norm_mode_pp != old_norm_pp) {
        state.normalize_mode = static_cast<NormalizeMode>(norm_mode_pp);
    }

    ImGui::Separator();

    // --- Canvas ---
    const float kCanvasPadding = 20.0f;
    const float kFitMargin = 0.85f;
    const float kVertexHitRadius = 8.0f;
    const float kEdgeHitDist = 8.0f;
    const float kEdgeEndpointMargin = 12.0f;

    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    float canvas_w = avail.x - kCanvasPadding * 2;
    float canvas_h = avail.y - 60.0f;
    if (canvas_w < 100.0f) canvas_w = 300.0f;
    if (canvas_h < 200.0f) canvas_h = 300.0f;

    ImVec2 canvas_size(canvas_w, canvas_h);

    ImGui::InvisibleButton("##PerPointSectionCanvas", canvas_size,
                           ImGuiButtonFlags_MouseButtonLeft |
                               ImGuiButtonFlags_MouseButtonRight);
    bool canvas_hovered = ImGui::IsItemHovered();

    ImVec2 canvas_min = ImGui::GetItemRectMin();
    ImVec2 canvas_max = ImGui::GetItemRectMax();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // Coordinate mapping
    float bb_min_x = verts[0].x, bb_min_y = verts[0].y;
    float bb_max_x = verts[0].x, bb_max_y = verts[0].y;
    for (const auto& v : verts) {
        bb_min_x = std::min(bb_min_x, v.x);
        bb_min_y = std::min(bb_min_y, v.y);
        bb_max_x = std::max(bb_max_x, v.x);
        bb_max_y = std::max(bb_max_y, v.y);
    }
    float bb_w = bb_max_x - bb_min_x;
    float bb_h = bb_max_y - bb_min_y;
    if (bb_w < 1e-6f) bb_w = 1.0f;
    if (bb_h < 1e-6f) bb_h = 1.0f;

    float scale = std::min(canvas_w / bb_w, canvas_h / bb_h) * kFitMargin;

    float bb_cx = (bb_min_x + bb_max_x) * 0.5f;
    float bb_cy = (bb_min_y + bb_max_y) * 0.5f;
    float canvas_cx = canvas_min.x + canvas_w * 0.5f;
    float canvas_cy = canvas_min.y + canvas_h * 0.5f;

    auto world_to_canvas = [&](const vec2f& v) -> ImVec2 {
        return ImVec2(canvas_cx + (v.x - bb_cx) * scale,
                      canvas_cy + (v.y - bb_cy) * scale);
    };
    auto canvas_to_world = [&](const ImVec2& p) -> vec2f {
        return vec2f{bb_cx + (p.x - canvas_cx) / scale,
                     bb_cy + (p.y - canvas_cy) / scale};
    };

    // Draw background
    dl->AddRectFilled(canvas_min, canvas_max, kColorBackground);

    // Draw grid
    const float kGridSpacing = 30.0f;
    for (float gx = canvas_min.x; gx <= canvas_max.x; gx += kGridSpacing)
        dl->AddLine(ImVec2(gx, canvas_min.y), ImVec2(gx, canvas_max.y),
                    kColorGrid, 0.5f);
    for (float gy = canvas_min.y; gy <= canvas_max.y; gy += kGridSpacing)
        dl->AddLine(ImVec2(canvas_min.x, gy), ImVec2(canvas_max.x, gy),
                    kColorGrid, 0.5f);

    // Draw polygon fill
    if (verts.size() >= 3) {
        std::vector<ImVec2> canvas_verts;
        canvas_verts.reserve(verts.size());
        for (const auto& v : verts)
            canvas_verts.push_back(world_to_canvas(v));
        auto tris = triangulate_ear_clip(canvas_verts);
        for (const auto& tri : tris)
            dl->AddTriangleFilled(canvas_verts[tri[0]], canvas_verts[tri[1]],
                                  canvas_verts[tri[2]], kColorPolyFill);
    }

    // --- Hit-testing (restricted: vertex hover + drag only) ---
    state.reset_frame_state();

    if (canvas_hovered) {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        float vert_hit_sq = kVertexHitRadius * kVertexHitRadius;

        // Check vertices
        for (size_t i = 0; i < verts.size(); ++i) {
            ImVec2 cv = world_to_canvas(verts[i]);
            float dx = mouse_pos.x - cv.x;
            float dy = mouse_pos.y - cv.y;
            if (dx * dx + dy * dy < vert_hit_sq) {
                state.hovered_vertex = static_cast<int>(i);
                break;
            }
        }

        // Restricted mode: no edge hover detection (no edge-click-to-add)
        // Restricted mode: no right-click-to-delete

        // Left click on vertex → start drag
        bool left_clicked =
            ImGui::IsMouseClicked(ImGuiMouseButton_Left) && canvas_hovered;
        if (left_clicked && state.hovered_vertex >= 0) {
            state.dragged_vertex = state.hovered_vertex;
            ImVec2 cv = world_to_canvas(verts[state.hovered_vertex]);
            state.drag_offset = vec2f{mouse_pos.x - cv.x, mouse_pos.y - cv.y};
        }
    }

    // --- Drag vertex ---
    if (state.dragged_vertex >= 0) {
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) && canvas_hovered) {
            if (!state.edit_active) {
                state.push_undo("Drag Vertex");
                state.edit_active = true;
            }
            ImVec2 mouse_pos = ImGui::GetMousePos();
            vec2f new_pos = canvas_to_world(
                ImVec2(mouse_pos.x - state.drag_offset.x,
                       mouse_pos.y - state.drag_offset.y));
            verts[state.dragged_vertex] = new_pos;
        } else if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            state.dragged_vertex = -1;
            state.edit_active = false;
        }
    }

    // --- Draw edges ---
    for (size_t i = 0; i < verts.size(); ++i) {
        size_t j = (i + 1) % verts.size();
        ImVec2 a = world_to_canvas(verts[i]);
        ImVec2 b = world_to_canvas(verts[j]);
        dl->AddLine(a, b, kColorEdge, 1.5f);
    }

    // --- Draw Bézier curve preview ---
    if (state.use_bezier_section && verts.size() >= 3) {
        const int n = static_cast<int>(verts.size());
        constexpr int kSubdiv = 8;

        auto catmull_rom_2d = [](float p0, float p1, float p2, float p3,
                                 float t) -> float {
            float t2 = t * t;
            float t3 = t2 * t;
            return 0.5f *
                   ((2.0f * p1) + (-p0 + p2) * t +
                    (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                    (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
        };

        std::vector<ImVec2> smooth_pts;
        smooth_pts.reserve(n * kSubdiv + 1);
        for (int i = 0; i < n; ++i) {
            int i0 = (i - 1 + n) % n;
            int i1 = i;
            int i2 = (i + 1) % n;
            int i3 = (i + 2) % n;
            const auto& p0 = verts[i0];
            const auto& p1 = verts[i1];
            const auto& p2 = verts[i2];
            const auto& p3 = verts[i3];
            for (int s = 0; s < kSubdiv; ++s) {
                float t = static_cast<float>(s) /
                          static_cast<float>(kSubdiv);
                float sx = catmull_rom_2d(p0.x, p1.x, p2.x, p3.x, t);
                float sy = catmull_rom_2d(p0.y, p1.y, p2.y, p3.y, t);
                smooth_pts.push_back(world_to_canvas(vec2f{sx, sy}));
            }
        }
        if (smooth_pts.size() >= 2) {
            smooth_pts.push_back(smooth_pts[0]);
            dl->AddPolyline(smooth_pts.data(),
                            static_cast<int>(smooth_pts.size()),
                            kColorBezierPreview, 0, 2.5f);
        }
    }

    // --- Draw vertices ---
    for (size_t i = 0; i < verts.size(); ++i) {
        ImVec2 cv = world_to_canvas(verts[i]);
        ImU32 color = kColorVertex;
        float radius = 5.0f;
        if (state.dragged_vertex == static_cast<int>(i)) {
            color = kColorVertexDrag;
            radius = 7.0f;
        } else if (state.hovered_vertex == static_cast<int>(i)) {
            color = kColorVertexHover;
            radius = 7.0f;
        }
        dl->AddCircleFilled(cv, radius, color);
        dl->AddCircle(cv, radius, IM_COL32(0, 0, 0, 100), 0, 1.5f);
    }

    ImGui::Separator();

    // --- Self-intersection error ---
    if (is_self_intersecting) {
        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%s",
                           get_locale_cstr("label.cross_section_error"));
    }

    // --- Apply + Clear Override buttons ---
    {
        bool apply_disabled = is_self_intersecting || verts.size() < 3;
        if (apply_disabled)
            ImGui::BeginDisabled();
        if (ImGui::Button(get_locale_cstr("action.apply_section"))) {
            state.push_undo("Apply");
            // Normalize section vertices according to selected mode
            {
                float min_x = verts[0].x, max_x = verts[0].x;
                float min_y = verts[0].y, max_y = verts[0].y;
                for (const auto& v : verts) {
                    if (v.x < min_x) min_x = v.x;
                    if (v.x > max_x) max_x = v.x;
                    if (v.y < min_y) min_y = v.y;
                    if (v.y > max_y) max_y = v.y;
                }
                float range_x = max_x - min_x;
                float range_y = max_y - min_y;
                if (range_x < 1e-8f) range_x = 1e-8f;
                if (range_y < 1e-8f) range_y = 1e-8f;
                float cx = (min_x + max_x) * 0.5f;
                float cy = (min_y + max_y) * 0.5f;
                float half_x = range_x * 0.5f;
                float half_y = range_y * 0.5f;
                float scale;
                switch (state.normalize_mode) {
                case NormalizeMode::NORMALIZE_X:
                    scale = half_x;
                    break;
                case NormalizeMode::NORMALIZE_Y:
                    scale = half_y;
                    break;
                case NormalizeMode::NORMALIZE_XY:
                default:
                    scale = std::max(half_x, half_y);
                    break;
                }
                if (scale < 1e-8f) scale = 1e-8f;
                for (auto& v : verts) {
                    v.x = (v.x - cx) / scale;
                    v.y = (v.y - cy) / scale;
                }
            }
            strand.mesh_dirty = true;
        }
        if (apply_disabled)
            ImGui::EndDisabled();

        ImGui::SameLine();
        // Clear Override: revert to global section
        if (ImGui::Button(get_locale_cstr("action.clear_perpoint_section"))) {
            state = SectionEditorState{};
            strand.mesh_dirty = true;
        }
    }

    // --- Hint text ---
    ImGui::TextWrapped("%s",
                       get_locale_cstr("label.perpoint_section_hint"));

    // --- Keyboard shortcuts ---
    if (ImGui::IsWindowFocused()) {
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Z)) {
            state.undo();
            strand.mesh_dirty = true;
        }
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Y)) {
            state.redo();
            strand.mesh_dirty = true;
        }
    }

    ImGui::End();
}

}  // namespace sinriv::ui::render
