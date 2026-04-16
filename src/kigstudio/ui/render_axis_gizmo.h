#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <vector>

#include "kigstudio/utils/mat.h"
#include "kigstudio/utils/vec3.h"

namespace sinriv::ui::render::axis_gizmo {
    using vec3f = sinriv::kigstudio::vec3<float>;
    using mat4f = sinriv::kigstudio::mat::matrix<float>;
    using vec4f = sinriv::kigstudio::mat::vec4<float>;

    enum class AxisHandle : uint8_t {
        None = 0,
        X,
        Y,
        Z,
    };

    struct ScreenPoint {
        float x = 0.0f;
        float y = 0.0f;
        bool valid = false;
    };

    struct LineSegment {
        vec3f start;
        vec3f end;
        AxisHandle axis = AxisHandle::None;
    };

    struct GizmoState {
        bool dragging = false;
        AxisHandle hovered_axis = AxisHandle::None;
        AxisHandle active_axis = AxisHandle::None;
        int viewport_width = 1280;
        int viewport_height = 720;
        float axis_length = 30.0f;
        float hit_radius_pixels = 12.0f;
        mat4f model_matrix;
        mat4f view_matrix;
        mat4f proj_matrix;

        GizmoState() {
            model_matrix.setIdentity();
            view_matrix.setIdentity();
            proj_matrix.setIdentity();
        }
    };

    inline float clamp(float value, float min_value, float max_value) {
        return std::max(min_value, std::min(max_value, value));
    }

    inline vec3f getAxisBasis(AxisHandle axis) {
        switch (axis) {
            case AxisHandle::X:
                return {1.0f, 0.0f, 0.0f};
            case AxisHandle::Y:
                return {0.0f, 1.0f, 0.0f};
            case AxisHandle::Z:
                return {0.0f, 0.0f, 1.0f};
            default:
                return {0.0f, 0.0f, 0.0f};
        }
    }

    inline vec3f transformPoint(const mat4f& matrix, const vec3f& point) {
        return (vec4f(point.x, point.y, point.z, 1.0f) * matrix).toVec3();
    }

    inline vec3f extractTranslation(const mat4f& matrix) {
        return {matrix[3][0], matrix[3][1], matrix[3][2]};
    }

    inline vec3f safeNormalize(const vec3f& value, const vec3f& fallback) {
        const float length = value.length();
        if (length <= 1e-6f) {
            return fallback;
        }
        return value / length;
    }

    inline vec3f extractAxisDirection(const mat4f& matrix, AxisHandle axis) {
        switch (axis) {
            case AxisHandle::X:
                return safeNormalize({matrix[0][0], matrix[0][1], matrix[0][2]},
                                     {1.0f, 0.0f, 0.0f});
            case AxisHandle::Y:
                return safeNormalize({matrix[1][0], matrix[1][1], matrix[1][2]},
                                     {0.0f, 1.0f, 0.0f});
            case AxisHandle::Z:
                return safeNormalize({matrix[2][0], matrix[2][1], matrix[2][2]},
                                     {0.0f, 0.0f, 1.0f});
            default:
                return {0.0f, 0.0f, 0.0f};
        }
    }

    inline vec3f orthogonalVector(const vec3f& direction) {
        const vec3f helper =
            std::fabs(direction.z) < 0.9f ? vec3f(0.0f, 0.0f, 1.0f)
                                          : vec3f(0.0f, 1.0f, 0.0f);
        return safeNormalize(direction.cross(helper), {1.0f, 0.0f, 0.0f});
    }

    inline std::vector<LineSegment> buildAxisSegments(const mat4f& model_matrix,
                                                      float axis_length) {
        const vec3f origin = extractTranslation(model_matrix);
        const float head_length = std::max(axis_length * 0.22f, 4.0f);
        const float head_width = head_length * 0.35f;
        std::vector<LineSegment> segments;
        segments.reserve(9);

        for (AxisHandle axis : {AxisHandle::X, AxisHandle::Y, AxisHandle::Z}) {
            const vec3f dir = extractAxisDirection(model_matrix, axis);
            const vec3f tip = origin + dir * axis_length;
            const vec3f side = orthogonalVector(dir) * head_width;
            const vec3f head_base = tip - dir * head_length;

            segments.push_back({origin, tip, axis});
            segments.push_back({tip, head_base + side, axis});
            segments.push_back({tip, head_base - side, axis});
        }

        return segments;
    }

    inline ScreenPoint projectToScreen(const vec3f& world_point,
                                       const GizmoState& state) {
        const vec4f clip =
            ((vec4f(world_point.x, world_point.y, world_point.z, 1.0f) *
              state.view_matrix) *
             state.proj_matrix);

        if (std::fabs(clip[3]) <= 1e-6f) {
            return {};
        }

        const float inv_w = 1.0f / clip[3];
        const float ndc_x = clip[0] * inv_w;
        const float ndc_y = clip[1] * inv_w;
        const float ndc_z = clip[2] * inv_w;
        if (!std::isfinite(ndc_x) || !std::isfinite(ndc_y) || !std::isfinite(ndc_z)) {
            return {};
        }

        return {((ndc_x * 0.5f) + 0.5f) * state.viewport_width,
                ((-ndc_y * 0.5f) + 0.5f) * state.viewport_height,
                ndc_z >= -1.5f && ndc_z <= 1.5f};
    }

    inline float computeAxisLengthForPixelSize(const GizmoState& state,
                                            AxisHandle axis,
                                            float target_pixels = 200.0f) {
        const vec3f origin = extractTranslation(state.model_matrix);
        const vec3f dir = extractAxisDirection(state.model_matrix, axis);

        const float epsilon = 1e-2f; // 小步长（世界单位）

        const ScreenPoint p0 = projectToScreen(origin, state);
        const ScreenPoint p1 = projectToScreen(origin + dir * epsilon, state);

        if (!p0.valid || !p1.valid) {
            return state.axis_length; // fallback
        }

        const float dx = p1.x - p0.x;
        const float dy = p1.y - p0.y;
        const float pixel_per_unit = std::sqrt(dx * dx + dy * dy) / epsilon;

        if (pixel_per_unit <= 1e-6f) {
            return state.axis_length;
        }

        return target_pixels / pixel_per_unit;
    }

    inline float computeAxisLengthStable(const GizmoState& state,
                                     float target_pixels = 200.0f) {
        const vec3f origin = extractTranslation(state.model_matrix);

        // 转到 view space
        vec4f view_pos = vec4f(origin.x, origin.y, origin.z, 1.0f) * state.view_matrix;
        float w = view_pos[3];
        float z = view_pos[2];
        float depth = std::fabs(z / w);
        if (depth < 1e-3f) depth = 1e-3f;

        float proj_yy = state.proj_matrix[1][1]; 

        float viewport_height = static_cast<float>(state.viewport_height);

        float world_size =
            target_pixels * depth / (proj_yy * viewport_height * 0.5f);

        return world_size;
    }

    inline float distancePointToSegmentSquared(float px,
                                               float py,
                                               const ScreenPoint& a,
                                               const ScreenPoint& b) {
        const float abx = b.x - a.x;
        const float aby = b.y - a.y;
        const float ab_len_sq = abx * abx + aby * aby;
        if (ab_len_sq <= 1e-6f) {
            const float dx = px - a.x;
            const float dy = py - a.y;
            return dx * dx + dy * dy;
        }

        const float apx = px - a.x;
        const float apy = py - a.y;
        const float t = clamp((apx * abx + apy * aby) / ab_len_sq, 0.0f, 1.0f);
        const float cx = a.x + abx * t;
        const float cy = a.y + aby * t;
        const float dx = px - cx;
        const float dy = py - cy;
        return dx * dx + dy * dy;
    }

    inline AxisHandle pickAxis(const GizmoState& state, int mouse_x, int mouse_y) {
        float axis_len = computeAxisLengthStable(state);
        const auto segments = buildAxisSegments(state.model_matrix, axis_len);
        const float max_distance_sq = state.hit_radius_pixels * state.hit_radius_pixels;
        float best_distance_sq = max_distance_sq;
        AxisHandle best_axis = AxisHandle::None;

        for (const auto& segment : segments) {
            const ScreenPoint a = projectToScreen(segment.start, state);
            const ScreenPoint b = projectToScreen(segment.end, state);
            if (!a.valid || !b.valid) {
                continue;
            }

            const float distance_sq =
                distancePointToSegmentSquared(static_cast<float>(mouse_x),
                                              static_cast<float>(mouse_y), a, b);
            if (distance_sq <= best_distance_sq) {
                best_distance_sq = distance_sq;
                best_axis = segment.axis;
            }
        }

        return best_axis;
    }

    inline std::tuple<int, int, int, int> getScreenBoundBox(const GizmoState& state) {
        float axis_len = computeAxisLengthStable(state);
        const auto segments = buildAxisSegments(state.model_matrix, axis_len);
        bool has_point = false;
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();

        for (const auto& segment : segments) {
            for (const vec3f& point : {segment.start, segment.end}) {
                const ScreenPoint screen_point = projectToScreen(point, state);
                if (!screen_point.valid) {
                    continue;
                }

                has_point = true;
                min_x = std::min(min_x, screen_point.x);
                min_y = std::min(min_y, screen_point.y);
                max_x = std::max(max_x, screen_point.x);
                max_y = std::max(max_y, screen_point.y);
            }
        }

        if (!has_point) {
            return {0, 0, 0, 0};
        }

        const int padding = static_cast<int>(std::ceil(state.hit_radius_pixels));
        return {static_cast<int>(std::floor(min_x)) - padding,
                static_cast<int>(std::floor(min_y)) - padding,
                static_cast<int>(std::ceil(max_x)) + padding,
                static_cast<int>(std::ceil(max_y)) + padding};
    }

    inline float getAxisScreenDelta(const GizmoState& state,
                                    AxisHandle axis,
                                    int from_x,
                                    int from_y,
                                    int to_x,
                                    int to_y) {
        if (axis == AxisHandle::None) {
            return 0.0f;
        }

        const vec3f origin = extractTranslation(state.model_matrix);
        const vec3f tip =
            origin + extractAxisDirection(state.model_matrix, axis) * state.axis_length;
        const ScreenPoint a = projectToScreen(origin, state);
        const ScreenPoint b = projectToScreen(tip, state);
        if (!a.valid || !b.valid) {
            return 0.0f;
        }

        const float axis_dx = b.x - a.x;
        const float axis_dy = b.y - a.y;
        const float axis_len_sq = axis_dx * axis_dx + axis_dy * axis_dy;
        if (axis_len_sq <= 1e-6f) {
            return 0.0f;
        }

        const float axis_len = std::sqrt(axis_len_sq);
        const float dir_x = axis_dx / axis_len;
        const float dir_y = axis_dy / axis_len;
        const float mouse_dx = static_cast<float>(to_x - from_x);
        const float mouse_dy = static_cast<float>(to_y - from_y);
        const float projected_pixels = mouse_dx * dir_x + mouse_dy * dir_y;
        return projected_pixels * (state.axis_length / axis_len);
    }

    inline vec3f getAxisWorldDelta(const GizmoState& state,
                                   AxisHandle axis,
                                   int from_x,
                                   int from_y,
                                   int to_x,
                                   int to_y) {
        return extractAxisDirection(state.model_matrix, axis) *
               getAxisScreenDelta(state, axis, from_x, from_y, to_x, to_y);
    }

    template <class VertexT>
    inline void appendAxisVertices(std::vector<VertexT>& vertices,
                                   const GizmoState& state) {
        float axis_len = computeAxisLengthStable(state);
        const auto segments = buildAxisSegments(state.model_matrix, axis_len);
        vertices.reserve(vertices.size() + segments.size() * 2);

        for (const auto& segment : segments) {
            const vec3f normal = safeNormalize(segment.end - segment.start,
                                               getAxisBasis(segment.axis));
            vertices.push_back(
                {segment.start.x, segment.start.y, segment.start.z, normal.x,
                 normal.y, normal.z});
            vertices.push_back({segment.end.x, segment.end.y, segment.end.z, normal.x,
                                normal.y, normal.z});
        }
    }

    inline std::array<vec3f, 8> buildBoundsCorners(const vec3f& min_bound,
                                                   const vec3f& max_bound) {
        return {{
            {min_bound.x, min_bound.y, min_bound.z},
            {max_bound.x, min_bound.y, min_bound.z},
            {max_bound.x, max_bound.y, min_bound.z},
            {min_bound.x, max_bound.y, min_bound.z},
            {min_bound.x, min_bound.y, max_bound.z},
            {max_bound.x, min_bound.y, max_bound.z},
            {max_bound.x, max_bound.y, max_bound.z},
            {min_bound.x, max_bound.y, max_bound.z},
        }};
    }

    inline float estimateAxisLengthFromBounds(const vec3f& min_bound,
                                              const vec3f& max_bound,
                                              float min_length = 20.0f) {
        const vec3f extent = max_bound - min_bound;
        return std::max(std::max(extent.x, std::max(extent.y, extent.z)) * 0.75f,
                        min_length);
    }

    inline std::tuple<int, int, int, int> projectBoundsToScreen(
        const std::array<vec3f, 8>& corners,
        const GizmoState& state) {
        bool has_point = false;
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();

        for (const auto& world_point : corners) {
            const ScreenPoint screen_point = projectToScreen(world_point, state);
            if (!screen_point.valid) {
                continue;
            }

            has_point = true;
            min_x = std::min(min_x, screen_point.x);
            min_y = std::min(min_y, screen_point.y);
            max_x = std::max(max_x, screen_point.x);
            max_y = std::max(max_y, screen_point.y);
        }

        if (!has_point) {
            return {0, 0, 0, 0};
        }

        return {static_cast<int>(std::floor(min_x)),
                static_cast<int>(std::floor(min_y)),
                static_cast<int>(std::ceil(max_x)),
                static_cast<int>(std::ceil(max_y))};
    }
}  // namespace sinriv::ui::render::axis_gizmo
