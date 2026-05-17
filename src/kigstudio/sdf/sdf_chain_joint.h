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

    inline Vec3f localToWorld(const Vec3f& p) const {
        return origin + x_axis * p.x + y_axis * p.y + z_axis * p.z;
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

// Build a frame whose z-axis points from start to end,
// and x-axis aligns with the world +Y axis (projected onto the plane
// perpendicular to z). This makes the male cylinder point toward +Y.
inline Frame buildFrameAlignedY(const Vec3f& start,
                                 const Vec3f& end) {
    Frame f;
    f.origin = start;
    f.z_axis = normalize(end - start);

    Vec3f world_y(0, 1, 0);
    Vec3f proj = world_y - f.z_axis * dot(world_y, f.z_axis);
    float proj_len = std::sqrt(dot(proj, proj));

    if (proj_len > 1e-6f) {
        f.x_axis = proj / proj_len;
    } else {
        // z is parallel to world_y, fall back to default
        Vec3f up = std::abs(f.z_axis.z) < 0.99f ? Vec3f(0, 0, 1) : Vec3f(1, 0, 0);
        f.x_axis = normalize(cross(up, f.z_axis));
    }

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

// Capped cylinder aligned with the local x-axis.
// The cylinder axis passes through the origin and points along +x.
inline float sdCappedCylinderX(const Vec3f& p, float radius, float half_height) {
    float r = std::sqrt(p.y * p.y + p.z * p.z);
    float dx = r - radius;
    float dy = std::abs(p.x) - half_height;

    float ax = std::max(dx, 0.0f);
    float ay = std::max(dy, 0.0f);

    return std::min(std::max(dx, dy), 0.0f) + std::sqrt(ax * ax + ay * ay);
}

// Infinite cone with vertex at origin, opening along +z.
// angle_rad is the half-opening angle.
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

// Finite cone: vertex at origin, opening along +z, truncated at z = height.
// The base radius at the truncation plane is height * tan(angle_rad).
inline float sdFiniteCone(const Vec3f& p, float angle_rad, float height) {
    float cone = sdCone(p, angle_rad);

    // Keep z >= 0 (above the vertex)
    cone = opIntersection(cone, -p.z);
    // Keep z <= height (below the truncation plane)
    cone = opIntersection(cone, p.z - height);

    return cone;
}

// ============================================================
// Negative Joint Volume
// ============================================================

class JointNegativeSDF {
   public:
    Frame frame;

    // socket cutting cone
    float socket_cone_offset = 5.f;
    float socket_cone_angle = 0.5f;
    float socket_cone_radius = 4.f;

    // female cylinder clearance (adds to male radius)
    float female_gap = 0.3f;

    // male cylinder source (shared axis/angle for female cylinder)
    float male_cylinder_offset = 3.f;
    float male_cylinder_radius = 1.5f;
    float male_cylinder_half_height = 10.f;

    // head cutting cone
    float head_cone_offset = 10.f;
    float head_cone_radius = 3.5f;

    // slot thickness between socket and head cones
    float slot_extra = 0.5f;

    inline float sdf(const Vec3f& world_p) const {
        Vec3f p = frame.worldToLocal(world_p);

        // ============================================
        // socket cutting cone (finite)
        // ============================================

        float socket_h = socket_cone_radius / std::tan(socket_cone_angle);
        Vec3f socket_p = p;
        socket_p.z -= socket_cone_offset;

        float socket_cone = sdFiniteCone(socket_p, socket_cone_angle, socket_h);

        // ============================================
        // head cutting cone (finite)
        // ============================================

        float head_h = head_cone_radius / std::tan(socket_cone_angle);
        Vec3f head_p = p;
        head_p.z -= head_cone_offset;

        float head_cone = sdFiniteCone(head_p, socket_cone_angle, head_h);

        // ============================================
        // female cylinder (shares axis with male cylinder)
        // Axis is the local x-axis, passing through (0,0,male_cylinder_offset).
        // ============================================

        Vec3f female_p = p;
        female_p.z -= male_cylinder_offset;

        float female_cyl = sdCappedCylinderX(
            female_p, male_cylinder_radius + female_gap, male_cylinder_half_height);

        // ============================================
        // slot: region inside socket cone but outside the
        // inflated head cone (provides clearance for movement)
        // ============================================

        float head_inflated = head_cone - slot_extra;
        float slot = opIntersection(socket_cone, -head_inflated);

        // ============================================
        // union: female cylinder + slot
        // ============================================

        return opUnion(female_cyl, slot);
    }

    inline bool contains(const Vec3f& p) const { return sdf(p) < 0.f; }
};

// ============================================================
// Positive Joint Volume
// ============================================================

class JointPositiveSDF {
   public:
    Frame frame;

    // socket support cone (fills behind socket cutting cone)
    float socket_support_offset = 2.f;
    float socket_support_angle = 0.5f;
    float socket_support_radius = 5.f;

    // head support cone (fills behind head cutting cone)
    float head_support_offset = 12.f;
    float head_support_angle = 0.5f;
    float head_support_radius = 5.f;

    // male cylinder
    float male_cylinder_offset = 3.f;
    float male_cylinder_radius = 1.5f;
    float male_cylinder_half_height = 10.f;

    inline float sdf(const Vec3f& world_p) const {
        Vec3f p = frame.worldToLocal(world_p);

        // ============================================
        // socket support cone (finite)
        // ============================================

        float socket_h =
            socket_support_radius / std::tan(socket_support_angle);
        Vec3f socket_p = p;
        socket_p.z -= socket_support_offset;

        float socket_support_cone =
            sdFiniteCone(socket_p, socket_support_angle, socket_h);

        // ============================================
        // head support cone (finite)
        // ============================================

        float head_h = head_support_radius / std::tan(head_support_angle);
        Vec3f head_p = p;
        head_p.z -= head_support_offset;

        float head_support_cone =
            sdFiniteCone(head_p, head_support_angle, head_h);

        // ============================================
        // male cylinder
        // Axis is the local x-axis, passing through (0,0,male_cylinder_offset).
        // The portion inside the socket support cone is removed so that
        // the cylinder terminates at the cone surface.
        // ============================================

        Vec3f male_p = p;
        male_p.z -= male_cylinder_offset;

        float male_cyl =
            sdCappedCylinderX(male_p, male_cylinder_radius, male_cylinder_half_height);

        // Remove portion inside socket support cone so the cylinder
        // terminates at the socket surface (not the head).
        male_cyl = opIntersection(male_cyl, -socket_support_cone);

        // ============================================
        // union
        // ============================================

        float result = opUnion(socket_support_cone, head_support_cone);
        result = opUnion(result, male_cyl);

        return result;
    }

    inline bool contains(const Vec3f& p) const { return sdf(p) < 0.f; }
};

// ============================================================
// Joint Wireframe Helpers
// ============================================================

inline void appendJointWireframe(
    std::vector<std::pair<Vec3f, Vec3f>>& segments,
    const JointNegativeSDF& neg,
    const JointPositiveSDF& pos) {

    auto addSeg = [&](const Vec3f& a, const Vec3f& b) {
        segments.emplace_back(a, b);
    };

    auto addCircleZ = [&](const Vec3f& center, float radius, int seg_count = 16) {
        for (int i = 0; i < seg_count; ++i) {
            float a0 = 2.0f * 3.14159265f * i / seg_count;
            float a1 = 2.0f * 3.14159265f * (i + 1) / seg_count;
            Vec3f p0(center.x + std::cos(a0) * radius,
                     center.y + std::sin(a0) * radius, center.z);
            Vec3f p1(center.x + std::cos(a1) * radius,
                     center.y + std::sin(a1) * radius, center.z);
            addSeg(p0, p1);
        }
    };

    // socket cutting cone
    float socket_h = neg.socket_cone_radius / std::tan(neg.socket_cone_angle);
    Vec3f socket_v(0, 0, neg.socket_cone_offset);
    Vec3f socket_base_c(0, 0, neg.socket_cone_offset + socket_h);
    addCircleZ(socket_base_c, neg.socket_cone_radius);
    for (int i = 0; i < 4; ++i) {
        float a = 2.0f * 3.14159265f * i / 4;
        Vec3f p(std::cos(a) * neg.socket_cone_radius,
                std::sin(a) * neg.socket_cone_radius, socket_base_c.z);
        addSeg(socket_v, p);
    }

    // head cutting cone
    float head_h = neg.head_cone_radius / std::tan(neg.socket_cone_angle);
    Vec3f head_v(0, 0, neg.head_cone_offset);
    Vec3f head_base_c(0, 0, neg.head_cone_offset + head_h);
    addCircleZ(head_base_c, neg.head_cone_radius);
    for (int i = 0; i < 4; ++i) {
        float a = 2.0f * 3.14159265f * i / 4;
        Vec3f p(std::cos(a) * neg.head_cone_radius,
                std::sin(a) * neg.head_cone_radius, head_base_c.z);
        addSeg(head_v, p);
    }

    // male cylinder (along x-axis)
    Vec3f cyl_c(0, 0, pos.male_cylinder_offset);
    float cyl_r = pos.male_cylinder_radius;
    for (int side = -1; side <= 1; side += 2) {
        Vec3f center(cyl_c.x + side * 2.0f, cyl_c.y, cyl_c.z);
        for (int i = 0; i < 16; ++i) {
            float a0 = 2.0f * 3.14159265f * i / 16;
            float a1 = 2.0f * 3.14159265f * (i + 1) / 16;
            Vec3f p0(center.x, center.y + std::cos(a0) * cyl_r,
                     center.z + std::sin(a0) * cyl_r);
            Vec3f p1(center.x, center.y + std::cos(a1) * cyl_r,
                     center.z + std::sin(a1) * cyl_r);
            addSeg(p0, p1);
        }
    }
    for (int i = 0; i < 4; ++i) {
        float a = 2.0f * 3.14159265f * i / 4;
        float y = std::cos(a) * cyl_r;
        float z = std::sin(a) * cyl_r;
        Vec3f p0(cyl_c.x - 2.0f, cyl_c.y + y, cyl_c.z + z);
        Vec3f p1(cyl_c.x + 2.0f, cyl_c.y + y, cyl_c.z + z);
        addSeg(p0, p1);
    }

    // socket support cone
    float socket_sup_h =
        pos.socket_support_radius / std::tan(pos.socket_support_angle);
    Vec3f socket_sup_v(0, 0, pos.socket_support_offset);
    Vec3f socket_sup_base(0, 0, pos.socket_support_offset + socket_sup_h);
    addCircleZ(socket_sup_base, pos.socket_support_radius);
    for (int i = 0; i < 4; ++i) {
        float a = 2.0f * 3.14159265f * i / 4;
        Vec3f p(std::cos(a) * pos.socket_support_radius,
                std::sin(a) * pos.socket_support_radius, socket_sup_base.z);
        addSeg(socket_sup_v, p);
    }

    // head support cone
    float head_sup_h =
        pos.head_support_radius / std::tan(pos.head_support_angle);
    Vec3f head_sup_v(0, 0, pos.head_support_offset);
    Vec3f head_sup_base(0, 0, pos.head_support_offset + head_sup_h);
    addCircleZ(head_sup_base, pos.head_support_radius);
    for (int i = 0; i < 4; ++i) {
        float a = 2.0f * 3.14159265f * i / 4;
        Vec3f p(std::cos(a) * pos.head_support_radius,
                std::sin(a) * pos.head_support_radius, head_sup_base.z);
        addSeg(head_sup_v, p);
    }
}

}  // namespace sinriv::kigstudio::sdf::joint
