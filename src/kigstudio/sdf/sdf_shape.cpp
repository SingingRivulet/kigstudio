#include "kigstudio/sdf/sdf_shape.h"

namespace sinriv::kigstudio::sdf {

static cJSON* vec3_to_json(const Vec3f& v);
static Vec3f json_to_vec3(const cJSON* json);

float SDF_FiniteCone::get(const Vec3f& p) const {
    return sdFiniteCone(p, angle_rad, height);
}

std::string SDF_FiniteCone::getInfo() const {
    return "SDF_FiniteCone(angle=" + std::to_string(angle_rad) +
           ", height=" + std::to_string(height) + ")";
}

cJSON* SDF_FiniteCone::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "finite_cone");
    cJSON_AddNumberToObject(obj, "angle_rad", angle_rad);
    cJSON_AddNumberToObject(obj, "height", height);
    return obj;
}

void SDF_FiniteCone::fromJSON(const cJSON* json) {
    angle_rad = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "angle_rad")));
    height = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "height")));
}

void SDF_FiniteCone::get(const Vec3f& begin,
                         const Vec3f& voxelSize,
                         const Vec3i& voxelCount,
                         std::vector<float>& out) const {
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
                out[i++] = get(Vec3f(wx, wy, wz));
            }
        }
    }
}

float SDF_CappedCylinderX::get(const Vec3f& p) const {
    return sdCappedCylinderX(p, radius, half_height);
}

std::string SDF_CappedCylinderX::getInfo() const {
    return "SDF_CappedCylinderX(radius=" + std::to_string(radius) +
           ", half_height=" + std::to_string(half_height) + ")";
}

cJSON* SDF_CappedCylinderX::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "capped_cylinder_x");
    cJSON_AddNumberToObject(obj, "radius", radius);
    cJSON_AddNumberToObject(obj, "half_height", half_height);
    return obj;
}

void SDF_CappedCylinderX::fromJSON(const cJSON* json) {
    radius = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "radius")));
    half_height = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "half_height")));
}

void SDF_CappedCylinderX::get(const Vec3f& begin,
                              const Vec3f& voxelSize,
                              const Vec3i& voxelCount,
                              std::vector<float>& out) const {
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
                out[i++] = get(Vec3f(wx, wy, wz));
            }
        }
    }
}

float SDF_Sphere::get(const Vec3f& p) const {
    return sdSphere(p - center, radius);
}

std::string SDF_Sphere::getInfo() const {
    return "SDF_Sphere(center=(" + std::to_string(center.x) + "," +
           std::to_string(center.y) + "," + std::to_string(center.z) +
           "), radius=" + std::to_string(radius) + ")";
}

cJSON* SDF_Sphere::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "sphere");
    cJSON_AddItemToObject(obj, "center", vec3_to_json(center));
    cJSON_AddNumberToObject(obj, "radius", radius);
    return obj;
}

void SDF_Sphere::fromJSON(const cJSON* json) {
    const cJSON* center_json = cJSON_GetObjectItem(json, "center");
    if (center_json) center = json_to_vec3(center_json);
    radius = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "radius")));
}

void SDF_Sphere::get(const Vec3f& begin,
                     const Vec3f& voxelSize,
                     const Vec3i& voxelCount,
                     std::vector<float>& out) const {
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
                out[i++] = get(Vec3f(wx, wy, wz));
            }
        }
    }
}

float SDF_Cylinder::get(const Vec3f& p) const {
    return sdCylinder(p, start, end, radius);
}

std::string SDF_Cylinder::getInfo() const {
    return "SDF_Cylinder(start=(" + std::to_string(start.x) + "," +
           std::to_string(start.y) + "," + std::to_string(start.z) +
           "), end=(" + std::to_string(end.x) + "," +
           std::to_string(end.y) + "," + std::to_string(end.z) +
           "), radius=" + std::to_string(radius) + ")";
}

cJSON* SDF_Cylinder::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "cylinder");
    cJSON_AddItemToObject(obj, "start", vec3_to_json(start));
    cJSON_AddItemToObject(obj, "end", vec3_to_json(end));
    cJSON_AddNumberToObject(obj, "radius", radius);
    return obj;
}

void SDF_Cylinder::fromJSON(const cJSON* json) {
    const cJSON* start_json = cJSON_GetObjectItem(json, "start");
    if (start_json) start = json_to_vec3(start_json);
    const cJSON* end_json = cJSON_GetObjectItem(json, "end");
    if (end_json) end = json_to_vec3(end_json);
    radius = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "radius")));
}

void SDF_Cylinder::get(const Vec3f& begin,
                      const Vec3f& voxelSize,
                      const Vec3i& voxelCount,
                      std::vector<float>& out) const {
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
                out[i++] = get(Vec3f(wx, wy, wz));
            }
        }
    }
}

float SDF_Capsule::get(const Vec3f& p) const {
    return sdCapsule(p, start, end, radius);
}

std::string SDF_Capsule::getInfo() const {
    return "SDF_Capsule(start=(" + std::to_string(start.x) + "," +
           std::to_string(start.y) + "," + std::to_string(start.z) +
           "), end=(" + std::to_string(end.x) + "," +
           std::to_string(end.y) + "," + std::to_string(end.z) +
           "), radius=" + std::to_string(radius) + ")";
}

cJSON* SDF_Capsule::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "capsule");
    cJSON_AddItemToObject(obj, "start", vec3_to_json(start));
    cJSON_AddItemToObject(obj, "end", vec3_to_json(end));
    cJSON_AddNumberToObject(obj, "radius", radius);
    return obj;
}

void SDF_Capsule::fromJSON(const cJSON* json) {
    const cJSON* start_json = cJSON_GetObjectItem(json, "start");
    if (start_json) start = json_to_vec3(start_json);
    const cJSON* end_json = cJSON_GetObjectItem(json, "end");
    if (end_json) end = json_to_vec3(end_json);
    radius = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "radius")));
}

void SDF_Capsule::get(const Vec3f& begin,
                      const Vec3f& voxelSize,
                      const Vec3i& voxelCount,
                      std::vector<float>& out) const {
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
                out[i++] = get(Vec3f(wx, wy, wz));
            }
        }
    }
}

float SDF_Box::get(const Vec3f& p) const {
    return sdBox(p, half_extent);
}

std::string SDF_Box::getInfo() const {
    return "SDF_Box(half_extent=(" + std::to_string(half_extent.x) + "," +
           std::to_string(half_extent.y) + "," + std::to_string(half_extent.z) +
           "))";
}

cJSON* SDF_Box::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "box");
    cJSON_AddItemToObject(obj, "half_extent", vec3_to_json(half_extent));
    return obj;
}

void SDF_Box::fromJSON(const cJSON* json) {
    const cJSON* half_extent_json = cJSON_GetObjectItem(json, "half_extent");
    if (half_extent_json) half_extent = json_to_vec3(half_extent_json);
}

void SDF_Box::get(const Vec3f& begin,
                  const Vec3f& voxelSize,
                  const Vec3i& voxelCount,
                  std::vector<float>& out) const {
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
                out[i++] = get(Vec3f(wx, wy, wz));
            }
        }
    }
}

static cJSON* vec3_to_json(const Vec3f& v) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(obj, "x", v.x);
    cJSON_AddNumberToObject(obj, "y", v.y);
    cJSON_AddNumberToObject(obj, "z", v.z);
    return obj;
}

static Vec3f json_to_vec3(const cJSON* json) {
    return Vec3f(
        static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "x"))),
        static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "y"))),
        static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "z"))));
}

static cJSON* frame_to_json(const Frame& f) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddItemToObject(obj, "origin", vec3_to_json(f.origin));
    cJSON_AddItemToObject(obj, "x_axis", vec3_to_json(f.x_axis));
    cJSON_AddItemToObject(obj, "y_axis", vec3_to_json(f.y_axis));
    cJSON_AddItemToObject(obj, "z_axis", vec3_to_json(f.z_axis));
    return obj;
}

static Frame json_to_frame(const cJSON* json) {
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

float SDF_FrameTransform::get(const Vec3f& p) const {
    return child ? child->get(frame.worldToLocal(p)) : 1e6f;
}

void SDF_FrameTransform::get(const Vec3f& begin,
                             const Vec3f& voxelSize,
                             const Vec3i& voxelCount,
                             std::vector<float>& out) const {
    if (!child) {
        out.clear();
        return;
    }

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
                out[i++] = child->get(frame.worldToLocal(world_p));
            }
        }
    }
}

std::string SDF_FrameTransform::getInfo() const {
    return "SDF_FrameTransform(origin=" + std::to_string(frame.origin.x) + "," +
           std::to_string(frame.origin.y) + "," + std::to_string(frame.origin.z) + ")";
}

cJSON* SDF_FrameTransform::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "frame_transform");
    cJSON_AddItemToObject(obj, "frame", frame_to_json(frame));
    if (child) cJSON_AddItemToObject(obj, "child", child->toJSON());
    return obj;
}

void SDF_FrameTransform::fromJSON(const cJSON* json) {
    const cJSON* frame_json = cJSON_GetObjectItem(json, "frame");
    if (frame_json) frame = json_to_frame(frame_json);
    const cJSON* cj = cJSON_GetObjectItem(json, "child");
    if (cj) child = sdf_from_json(cj);
}

static bool _register_shape_types = []() {
    sdf_register_type("sphere", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDF_Sphere>();
        obj->fromJSON(json);
        return obj;
    });
    sdf_register_type("cylinder", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDF_Cylinder>();
        obj->fromJSON(json);
        return obj;
    });
    sdf_register_type("capsule", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDF_Capsule>();
        obj->fromJSON(json);
        return obj;
    });
    sdf_register_type("finite_cone", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDF_FiniteCone>(0.0f, 0.0f);
        obj->fromJSON(json);
        return obj;
    });
    sdf_register_type("capped_cylinder_x", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDF_CappedCylinderX>(0.0f, 0.0f);
        obj->fromJSON(json);
        return obj;
    });
    sdf_register_type("box", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDF_Box>();
        obj->fromJSON(json);
        return obj;
    });
    sdf_register_type("frame_transform", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDF_FrameTransform>(Frame{}, nullptr);
        obj->fromJSON(json);
        return obj;
    });
    return true;
}();

}  // namespace sinriv::kigstudio::sdf
