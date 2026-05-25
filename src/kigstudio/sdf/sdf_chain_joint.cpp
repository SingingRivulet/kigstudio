#include "kigstudio/sdf/sdf_chain_joint.h"

namespace sinriv::kigstudio::sdf::joint {

Frame buildFrame(const Vec3f& start,
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

Frame buildFrameAlignedY(const Vec3f& start, const Vec3f& end) {
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
        Vec3f up =
            std::abs(f.z_axis.z) < 0.99f ? Vec3f(0, 0, 1) : Vec3f(1, 0, 0);
        f.x_axis = normalize(cross(up, f.z_axis));
    }

    f.y_axis = normalize(cross(f.z_axis, f.x_axis));
    return f;
}

float sdCappedCylinder(const Vec3f& p, float radius, float half_height) {
    float dx = std::sqrt(p.x * p.x + p.y * p.y) - radius;
    float dy = std::abs(p.z) - half_height;

    float ax = std::max(dx, 0.0f);
    float ay = std::max(dy, 0.0f);

    return std::min(std::max(dx, dy), 0.0f) + std::sqrt(ax * ax + ay * ay);
}

float sdCappedCylinderX(const Vec3f& p, float radius, float half_height) {
    float r = std::sqrt(p.y * p.y + p.z * p.z);
    float dx = r - radius;
    float dy = std::abs(p.x) - half_height;

    float ax = std::max(dx, 0.0f);
    float ay = std::max(dy, 0.0f);

    return std::min(std::max(dx, dy), 0.0f) + std::sqrt(ax * ax + ay * ay);
}

float sdCone(const Vec3f& p, float angle_rad) {
    float r = std::sqrt(p.x * p.x + p.y * p.y);

    return r - p.z * std::tan(angle_rad);
}

float opUnion(float a, float b) {
    return std::min(a, b);
}

float opSubtraction(float a, float b) {
    return std::max(a, -b);
}

float opIntersection(float a, float b) {
    return std::max(a, b);
}

float sdFiniteCone(const Vec3f& p, float angle_rad, float height) {
    float cone = sdCone(p, angle_rad);

    // Keep z >= 0 (above the vertex)
    cone = opIntersection(cone, -p.z);
    // Keep z <= height (below the truncation plane)
    cone = opIntersection(cone, p.z - height);

    return cone;
}

void appendJointWireframe(std::vector<std::pair<Vec3f, Vec3f>>& segments,
                          const JointNegativeSDF& neg,
                          const JointPositiveSDF& pos) {
    auto addSeg = [&](const Vec3f& a, const Vec3f& b) {
        segments.emplace_back(a, b);
    };

    auto addCircleZ = [&](const Vec3f& center, float radius,
                          int seg_count = 16) {
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

float SDF_FiniteCone::get(const Vec3f& p) const {
    return sdFiniteCone(p, angle_rad, height);
}

float SDF_CappedCylinderX::get(const Vec3f& p) const {
    return sdCappedCylinderX(p, radius, half_height);
}
float SDF_FrameTransform::get(const Vec3f& p) const {
    return child->get(frame.worldToLocal(p));
}

std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> JointNegativeSDF::buildTree()
    const {
    using namespace sinriv::kigstudio::sdf;

    // socket cutting cone
    float socket_h = socket_cone_radius / std::tan(socket_cone_angle);
    auto socket_cone =
        std::make_shared<SDF_FiniteCone>(socket_cone_angle, socket_h);
    auto socket_node = std::make_shared<SDF_Translate>(
        Vec3f(0, 0, socket_cone_offset), socket_cone);

    // head cutting cone
    float head_h = head_cone_radius / std::tan(socket_cone_angle);
    auto head_cone =
        std::make_shared<SDF_FiniteCone>(socket_cone_angle, head_h);
    auto head_trans = std::make_shared<SDF_Translate>(
        Vec3f(0, 0, head_cone_offset), head_cone);
    auto head_inflated = std::make_shared<SDF_Offset>(slot_extra, head_trans);

    // female cylinder
    auto female_cyl = std::make_shared<SDF_CappedCylinderX>(
        male_cylinder_radius + female_gap, male_cylinder_half_height);
    auto female_node = std::make_shared<SDF_Translate>(
        Vec3f(0, 0, male_cylinder_offset), female_cyl);

    // slot: socket minus inflated head
    auto slot = sdf_subtraction(socket_node, head_inflated);

    // union: female cylinder + slot
    auto local_tree = sdf_union(female_node, slot);

    return std::make_shared<SDF_FrameTransform>(frame, local_tree);
}

float JointNegativeSDF::get(const Vec3f& world_p) const {
    return buildTree()->get(world_p);
}

std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> JointPositiveSDF::buildTree()
    const {
    using namespace sinriv::kigstudio::sdf;

    // socket support cone
    float socket_h = socket_support_radius / std::tan(socket_support_angle);
    auto socket_support =
        std::make_shared<SDF_FiniteCone>(socket_support_angle, socket_h);
    auto socket_support_node = std::make_shared<SDF_Translate>(
        Vec3f(0, 0, socket_support_offset), socket_support);

    // male cylinder
    auto male_cyl = std::make_shared<SDF_CappedCylinderX>(
        male_cylinder_radius, effectiveHalfHeight());
    auto male_node = std::make_shared<SDF_Translate>(
        Vec3f(0, 0, male_cylinder_offset), male_cyl);

    // Original test logic: opSubtraction(male_cyl, -socket_support_cone)
    // = max(male_cyl, socket_support_cone) => intersection
    auto local_tree = sdf_intersection(male_node, socket_support_node);

    return std::make_shared<SDF_FrameTransform>(frame, local_tree);
}

float JointPositiveSDF::get(const Vec3f& world_p) const {
    return buildTree()->get(world_p);
}

}  // namespace sinriv::kigstudio::sdf::joint
