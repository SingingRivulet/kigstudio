#pragma once
#include <algorithm>
#include <cmath>
#include "kigstudio/utils/vec3.h"

namespace sinriv::kigstudio::sdf::joint {

// ============================================================
// Basic Vec3
// ============================================================

using Vec3f = sinriv::kigstudio::vec3<float>;

// ============================================================
// Local Frame
// ============================================================

struct Frame {
    Vec3f origin;

    Vec3f x_axis;
    Vec3f y_axis;
    Vec3f z_axis;

    inline Vec3f worldToLocal(const Vec3f& p) const {
        Vec3f d = p - origin;

        return {dot(d, x_axis), dot(d, y_axis), dot(d, z_axis)};
    }
};

inline Frame buildFrame(const Vec3f& start,
                        const Vec3f& end,
                        float rotation_angle_rad) {
    Frame f;

    f.origin = start;

    f.z_axis = normalize(end - start);

    Vec3f up = std::abs(f.z_axis.z) < 0.99f ? Vec3f(0, 0, 1) : Vec3f(1, 0, 0);

    Vec3f t0 = normalize(cross(up, f.z_axis));
    Vec3f t1 = cross(f.z_axis, t0);

    float c = std::cos(rotation_angle_rad);
    float s = std::sin(rotation_angle_rad);

    f.x_axis = normalize(t0 * c + t1 * s);
    f.y_axis = normalize(cross(f.z_axis, f.x_axis));

    return f;
}

// ============================================================
// SDF helpers
// ============================================================

inline float sdCappedCylinder(const Vec3f& p, float radius, float half_height) {
    float dx = std::sqrt(p.x * p.x + p.y * p.y) - radius;
    float dy = std::abs(p.z) - half_height;

    float ax = std::max(dx, 0.0f);
    float ay = std::max(dy, 0.0f);

    return std::min(std::max(dx, dy), 0.0f) + std::sqrt(ax * ax + ay * ay);
}

inline float sdCone(const Vec3f& p, float angle_rad) {
    float r = std::sqrt(p.x * p.x + p.y * p.y);

    return r - p.z * std::tan(angle_rad);
}

inline float opUnion(float a, float b) {
    return std::min(a, b);
}

inline float opSubtraction(float a, float b) {
    return std::max(a, -b);
}

inline float opIntersection(float a, float b) {
    return std::max(a, b);
}

// ============================================================
// Negative Joint Volume
// ============================================================

class JointNegativeSDF {
   public:
    Frame frame;

    // socket cone
    float socket_cone_offset = 5.f;
    float socket_cone_angle = 0.5f;
    float socket_cone_radius = 4.f;

    // female cylinder
    float female_gap = 0.3f;

    // male cylinder source
    float male_cylinder_offset = 3.f;
    float male_cylinder_radius = 1.5f;

    // head cone
    float head_cone_offset = 10.f;
    float head_cone_radius = 3.5f;

    // slot thickness
    float slot_extra = 0.5f;

    inline float sdf(const Vec3f& world_p) const {
        Vec3f p = frame.worldToLocal(world_p);

        // ============================================
        // socket cone
        // ============================================

        Vec3f cone_p = p;
        cone_p.z -= socket_cone_offset;

        float socket_cone = sdCone(cone_p, socket_cone_angle);

        socket_cone = opIntersection(socket_cone, cone_p.z);

        // ============================================
        // female cylinder
        // ============================================

        Vec3f cyl_p = p;

        cyl_p.x -= male_cylinder_offset;

        float female_cyl = sdCappedCylinder(
            cyl_p, male_cylinder_radius + female_gap, socket_cone_radius);

        // ============================================
        // slot
        // ============================================

        Vec3f head_p = p;
        head_p.z -= head_cone_offset;

        float head_cone = sdCone(head_p, socket_cone_angle);

        head_cone = opIntersection(head_cone, head_p.z);

        float slot = opIntersection(socket_cone, -head_cone);

        // ============================================
        // union
        // ============================================

        float result = socket_cone;

        result = opUnion(result, female_cyl);
        result = opUnion(result, slot);

        return result;
    }

    inline bool contains(const Vec3f& p) const { return sdf(p) < 0.f; }
};

// ============================================================
// Positive Joint Volume
// ============================================================

class JointPositiveSDF {
   public:
    Frame frame;

    // support cone
    float support_offset = 2.f;
    float support_angle = 0.5f;
    float support_radius = 5.f;

    // male cylinder
    float male_cylinder_offset = 3.f;
    float male_cylinder_radius = 1.5f;

    inline float sdf(const Vec3f& world_p) const {
        Vec3f p = frame.worldToLocal(world_p);

        // ============================================
        // support cone
        // ============================================

        Vec3f cone_p = p;
        cone_p.z -= support_offset;

        float support_cone = sdCone(cone_p, support_angle);

        support_cone = opIntersection(support_cone, cone_p.z);

        // ============================================
        // male cylinder
        // ============================================

        Vec3f cyl_p = p;

        cyl_p.x -= male_cylinder_offset;

        float male_cyl =
            sdCappedCylinder(cyl_p, male_cylinder_radius, support_radius);

        // ============================================
        // keep cylinder inside cone
        // ============================================

        male_cyl = opIntersection(male_cyl, -support_cone);

        // ============================================
        // union
        // ============================================

        return opUnion(support_cone, male_cyl);
    }

    inline bool contains(const Vec3f& p) const { return sdf(p) < 0.f; }
};

}  // namespace sinriv::kigstudio::sdf::joint