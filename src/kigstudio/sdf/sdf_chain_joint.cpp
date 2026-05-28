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
    using namespace sinriv::kigstudio::sdf;
    Vec3f p = frame.worldToLocal(world_p);

    float socket_h = socket_cone_radius / std::tan(socket_cone_angle);
    Vec3f socket_off(0, 0, socket_cone_offset);
    float socket = sdFiniteCone(p - socket_off, socket_cone_angle, socket_h);

    float head_h = head_cone_radius / std::tan(socket_cone_angle);
    Vec3f head_off(0, 0, head_cone_offset);
    float head = sdFiniteCone(p - head_off, socket_cone_angle, head_h);
    float head_inflated = head - slot_extra;

    float slot = opSubtraction(socket, head_inflated);

    Vec3f cyl_off(0, 0, male_cylinder_offset);
    float cyl = sdCappedCylinderX(p - cyl_off, male_cylinder_radius + female_gap, male_cylinder_half_height);

    float out = opUnion(cyl, slot);
    return out;
}

void JointNegativeSDF::get(const Vec3f& begin,
                          const Vec3f& voxelSize,
                          const Vec3i& voxelCount,
                          std::vector<float>& out) const {
    using namespace sinriv::kigstudio::sdf;
    if (voxelCount.x <= 0 || voxelCount.y <= 0 || voxelCount.z <= 0) {
        out.clear();
        return;
    }

    const size_t total = static_cast<size_t>(voxelCount.x) *
                         static_cast<size_t>(voxelCount.y) *
                         static_cast<size_t>(voxelCount.z);
    out.resize(total);

    size_t i = 0;
    for (int z = 0; z < voxelCount.z; ++z) {
        const float wz = begin.z + static_cast<float>(z) * voxelSize.z;
        for (int y = 0; y < voxelCount.y; ++y) {
            const float wy = begin.y + static_cast<float>(y) * voxelSize.y;
            for (int x = 0; x < voxelCount.x; ++x) {
                const float wx = begin.x + static_cast<float>(x) * voxelSize.x;
                Vec3f world_p(wx, wy, wz);
                Vec3f p = frame.worldToLocal(world_p);

                float socket_h = socket_cone_radius / std::tan(socket_cone_angle);
                Vec3f socket_off(0, 0, socket_cone_offset);
                float socket = sdFiniteCone(p - socket_off, socket_cone_angle, socket_h);

                float head_h = head_cone_radius / std::tan(socket_cone_angle);
                Vec3f head_off(0, 0, head_cone_offset);
                float head = sdFiniteCone(p - head_off, socket_cone_angle, head_h);
                float head_inflated = head - slot_extra;

                float slot = opSubtraction(socket, head_inflated);

                Vec3f cyl_off(0, 0, male_cylinder_offset);
                float cyl = sdCappedCylinderX(p - cyl_off, male_cylinder_radius + female_gap, male_cylinder_half_height);

                out[i++] = opUnion(cyl, slot);
            }
        }
    }
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
    using namespace sinriv::kigstudio::sdf;
    Vec3f p = frame.worldToLocal(world_p);

    float socket_h = socket_support_radius / std::tan(socket_support_angle);
    Vec3f socket_off(0, 0, socket_support_offset);
    float socket = sdFiniteCone(p - socket_off, socket_support_angle, socket_h);

    Vec3f cyl_off(0, 0, male_cylinder_offset);
    float cyl = sdCappedCylinderX(p - cyl_off, male_cylinder_radius, effectiveHalfHeight());

    float out = opIntersection(cyl, socket);
    return out;
}

void JointPositiveSDF::get(const Vec3f& begin,
                          const Vec3f& voxelSize,
                          const Vec3i& voxelCount,
                          std::vector<float>& out) const {
    using namespace sinriv::kigstudio::sdf;
    if (voxelCount.x <= 0 || voxelCount.y <= 0 || voxelCount.z <= 0) {
        out.clear();
        return;
    }

    const size_t total = static_cast<size_t>(voxelCount.x) *
                         static_cast<size_t>(voxelCount.y) *
                         static_cast<size_t>(voxelCount.z);
    out.resize(total);

    size_t i = 0;
    for (int z = 0; z < voxelCount.z; ++z) {
        const float wz = begin.z + static_cast<float>(z) * voxelSize.z;
        for (int y = 0; y < voxelCount.y; ++y) {
            const float wy = begin.y + static_cast<float>(y) * voxelSize.y;
            for (int x = 0; x < voxelCount.x; ++x) {
                const float wx = begin.x + static_cast<float>(x) * voxelSize.x;
                Vec3f world_p(wx, wy, wz);
                Vec3f p = frame.worldToLocal(world_p);

                float socket_h = socket_support_radius / std::tan(socket_support_angle);
                Vec3f socket_off(0, 0, socket_support_offset);
                float socket = sdFiniteCone(p - socket_off, socket_support_angle, socket_h);

                Vec3f cyl_off(0, 0, male_cylinder_offset);
                float cyl = sdCappedCylinderX(p - cyl_off, male_cylinder_radius, effectiveHalfHeight());

                out[i++] = opIntersection(cyl, socket);
            }
        }
    }
}

// ============================================================
// Serialization helpers
// ============================================================

namespace {

cJSON* vec3_to_json(const Vec3f& v) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(obj, "x", v.x);
    cJSON_AddNumberToObject(obj, "y", v.y);
    cJSON_AddNumberToObject(obj, "z", v.z);
    return obj;
}

Vec3f json_to_vec3(const cJSON* json) {
    return Vec3f(
        static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "x"))),
        static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "y"))),
        static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "z"))));
}

cJSON* frame_to_json(const Frame& f) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddItemToObject(obj, "origin", vec3_to_json(f.origin));
    cJSON_AddItemToObject(obj, "x_axis", vec3_to_json(f.x_axis));
    cJSON_AddItemToObject(obj, "y_axis", vec3_to_json(f.y_axis));
    cJSON_AddItemToObject(obj, "z_axis", vec3_to_json(f.z_axis));
    return obj;
}

Frame json_to_frame(const cJSON* json) {
    Frame f;
    const cJSON* origin = cJSON_GetObjectItem(json, "origin");
    if (origin) f.origin = json_to_vec3(origin);
    const cJSON* x_axis = cJSON_GetObjectItem(json, "x_axis");
    if (x_axis) f.x_axis = json_to_vec3(x_axis);
    const cJSON* y_axis = cJSON_GetObjectItem(json, "y_axis");
    if (y_axis) f.y_axis = json_to_vec3(y_axis);
    const cJSON* z_axis = cJSON_GetObjectItem(json, "z_axis");
    if (z_axis) f.z_axis = json_to_vec3(z_axis);
    return f;
}

}  // namespace

// ============================================================
// JointNegativeSDF
// ============================================================

std::string JointNegativeSDF::getInfo() const {
    return "JointNegativeSDF(socket_cone_offset=" +
           std::to_string(socket_cone_offset) +
           ", head_cone_offset=" + std::to_string(head_cone_offset) + ")";
}

cJSON* JointNegativeSDF::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "joint_negative");
    cJSON_AddItemToObject(obj, "frame", frame_to_json(frame));
    cJSON_AddNumberToObject(obj, "socket_cone_offset", socket_cone_offset);
    cJSON_AddNumberToObject(obj, "socket_cone_angle", socket_cone_angle);
    cJSON_AddNumberToObject(obj, "socket_cone_radius", socket_cone_radius);
    cJSON_AddNumberToObject(obj, "female_gap", female_gap);
    cJSON_AddNumberToObject(obj, "male_cylinder_offset", male_cylinder_offset);
    cJSON_AddNumberToObject(obj, "male_cylinder_radius", male_cylinder_radius);
    cJSON_AddNumberToObject(obj, "male_cylinder_half_height", male_cylinder_half_height);
    cJSON_AddNumberToObject(obj, "head_cone_offset", head_cone_offset);
    cJSON_AddNumberToObject(obj, "head_cone_radius", head_cone_radius);
    cJSON_AddNumberToObject(obj, "slot_extra", slot_extra);
    return obj;
}

void JointNegativeSDF::fromJSON(const cJSON* json) {
    const cJSON* frame_json = cJSON_GetObjectItem(json, "frame");
    if (frame_json) frame = json_to_frame(frame_json);
    socket_cone_offset = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "socket_cone_offset")));
    socket_cone_angle = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "socket_cone_angle")));
    socket_cone_radius = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "socket_cone_radius")));
    female_gap = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "female_gap")));
    male_cylinder_offset = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "male_cylinder_offset")));
    male_cylinder_radius = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "male_cylinder_radius")));
    male_cylinder_half_height = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "male_cylinder_half_height")));
    head_cone_offset = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "head_cone_offset")));
    head_cone_radius = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "head_cone_radius")));
    slot_extra = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "slot_extra")));
}

// ============================================================
// JointPositiveSDF
// ============================================================

std::string JointPositiveSDF::getInfo() const {
    return "JointPositiveSDF(socket_support_offset=" +
           std::to_string(socket_support_offset) +
           ", head_support_offset=" + std::to_string(head_support_offset) + ")";
}

cJSON* JointPositiveSDF::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "joint_positive");
    cJSON_AddItemToObject(obj, "frame", frame_to_json(frame));
    cJSON_AddNumberToObject(obj, "socket_support_offset", socket_support_offset);
    cJSON_AddNumberToObject(obj, "socket_support_angle", socket_support_angle);
    cJSON_AddNumberToObject(obj, "socket_support_radius", socket_support_radius);
    cJSON_AddNumberToObject(obj, "head_support_offset", head_support_offset);
    cJSON_AddNumberToObject(obj, "head_support_angle", head_support_angle);
    cJSON_AddNumberToObject(obj, "head_support_radius", head_support_radius);
    cJSON_AddNumberToObject(obj, "male_cylinder_offset", male_cylinder_offset);
    cJSON_AddNumberToObject(obj, "male_cylinder_radius", male_cylinder_radius);
    cJSON_AddNumberToObject(obj, "male_cylinder_half_height", male_cylinder_half_height);
    return obj;
}

void JointPositiveSDF::fromJSON(const cJSON* json) {
    const cJSON* frame_json = cJSON_GetObjectItem(json, "frame");
    if (frame_json) frame = json_to_frame(frame_json);
    socket_support_offset = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "socket_support_offset")));
    socket_support_angle = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "socket_support_angle")));
    socket_support_radius = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "socket_support_radius")));
    head_support_offset = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "head_support_offset")));
    head_support_angle = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "head_support_angle")));
    head_support_radius = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "head_support_radius")));
    male_cylinder_offset = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "male_cylinder_offset")));
    male_cylinder_radius = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "male_cylinder_radius")));
    male_cylinder_half_height = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "male_cylinder_half_height")));
}

// ============================================================
// Static registration
// ============================================================

static bool _register_joint_types = []() {
    sinriv::kigstudio::sdf::sdf_register_type("joint_negative", [](const cJSON* json) -> std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> {
        auto obj = std::make_shared<JointNegativeSDF>();
        obj->fromJSON(json);
        return obj;
    });
    sinriv::kigstudio::sdf::sdf_register_type("joint_positive", [](const cJSON* json) -> std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> {
        auto obj = std::make_shared<JointPositiveSDF>();
        obj->fromJSON(json);
        return obj;
    });
    return true;
}();

}  // namespace sinriv::kigstudio::sdf::joint

