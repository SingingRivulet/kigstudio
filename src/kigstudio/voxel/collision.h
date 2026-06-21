#pragma once
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <utility>
#include <variant>
#include <vector>
#include "kigstudio/sdf/sdf_shape.h"
#include "kigstudio/utils/mat.h"
#include "kigstudio/utils/vec3.h"

namespace sinriv::kigstudio {
namespace voxel::collision {
using vec3f = sinriv::kigstudio::vec3<float>;
using mat4f = sinriv::kigstudio::mat::matrix<float>;
using vec4f = sinriv::kigstudio::mat::vec4<float>;

inline float clamp01(float value) {
    return std::max(0.0f, std::min(1.0f, value));
}

inline float lengthSquared(const vec3f& v) {
    return v.dot(v);
}

inline vec3f closestPointOnSegment(const vec3f& point,
                                   const vec3f& start,
                                   const vec3f& end) {
    const vec3f segment = end - start;
    const float segment_length_sq = lengthSquared(segment);

    if (segment_length_sq <= 0.0f) {
        return start;
    }

    const float t = clamp01((point - start).dot(segment) / segment_length_sq);
    return start + segment * t;
}

struct Sphere : public sinriv::kigstudio::sdf::SDF_Sphere {
    using sinriv::kigstudio::sdf::SDF_Sphere::SDF_Sphere;
    inline bool contains(const vec3f& point) const {
        return get(point) <= 0.0f;
    }
};

struct Cylinder : public sinriv::kigstudio::sdf::SDF_Cylinder {
    using sinriv::kigstudio::sdf::SDF_Cylinder::SDF_Cylinder;
    inline bool contains(const vec3f& point) const {
        return get(point) <= 0.0f;
    }
};

struct Capsule : public sinriv::kigstudio::sdf::SDF_Capsule {
    using sinriv::kigstudio::sdf::SDF_Capsule::SDF_Capsule;
    inline bool contains(const vec3f& point) const {
        return get(point) <= 0.0f;
    }
};

struct Box : public sinriv::kigstudio::sdf::SDF_Box {
    using sinriv::kigstudio::sdf::SDF_Box::SDF_Box;
    inline bool contains(const vec3f& point) const {
        return get(point) <= 0.0f;
    }
};

inline bool pointIntersects(const vec3f& point, const Sphere& sphere) {
    return sphere.contains(point);
}

inline bool pointIntersects(const vec3f& point, const Cylinder& cylinder) {
    return cylinder.contains(point);
}

inline bool pointIntersects(const vec3f& point, const Capsule& capsule) {
    return capsule.contains(point);
}

inline bool pointIntersects(const vec3f& point, const Box& box) {
    return box.contains(point);
}

struct Quaternion {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;

    inline Quaternion normalized() const {
        const float norm = std::sqrt(x * x + y * y + z * z + w * w);
        if (norm <= 0.0f) {
            return {};
        }
        return {x / norm, y / norm, z / norm, w / norm};
    }
};

struct AxisAngle {
    vec3f axis = {1.0f, 0.0f, 0.0f};
    float angle = 0.0f;
};

inline vec3f safeNormalize(const vec3f& value,
                           const vec3f& fallback = vec3f(1.0f, 0.0f, 0.0f)) {
    const float len_sq = lengthSquared(value);
    if (len_sq <= 1e-12f) {
        return fallback;
    }
    return value / std::sqrt(len_sq);
}

inline Quaternion quaternionFromAxisAngle(const AxisAngle& axis_angle) {
    const vec3f axis = safeNormalize(axis_angle.axis);
    const float half = axis_angle.angle * 0.5f;
    const float s = std::sin(half);
    return Quaternion{axis.x * s, axis.y * s, axis.z * s, std::cos(half)}
        .normalized();
}

inline Quaternion quaternionFromEuler(const vec3f& euler_xyz) {
    const float cx = std::cos(euler_xyz.x * 0.5f);
    const float sx = std::sin(euler_xyz.x * 0.5f);
    const float cy = std::cos(euler_xyz.y * 0.5f);
    const float sy = std::sin(euler_xyz.y * 0.5f);
    const float cz = std::cos(euler_xyz.z * 0.5f);
    const float sz = std::sin(euler_xyz.z * 0.5f);

    Quaternion q;
    q.w = cx * cy * cz + sx * sy * sz;
    q.x = sx * cy * cz - cx * sy * sz;
    q.y = cx * sy * cz + sx * cy * sz;
    q.z = cx * cy * sz - sx * sy * cz;
    return q.normalized();
}

inline vec3f eulerFromQuaternion(const Quaternion& quaternion) {
    const Quaternion q = quaternion.normalized();

    const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    const float roll_x = std::atan2(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    float pitch_y = 0.0f;
    if (std::fabs(sinp) >= 1.0f) {
        pitch_y = std::copysign(3.14159265358979323846f / 2.0f, sinp);
    } else {
        pitch_y = std::asin(sinp);
    }

    const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    const float yaw_z = std::atan2(siny_cosp, cosy_cosp);

    return {roll_x, pitch_y, yaw_z};
}

inline AxisAngle axisAngleFromQuaternion(const Quaternion& quaternion) {
    const Quaternion q = quaternion.normalized();
    const float clamped_w = std::max(-1.0f, std::min(1.0f, q.w));
    const float angle = 2.0f * std::acos(clamped_w);
    const float s = std::sqrt(std::max(0.0f, 1.0f - clamped_w * clamped_w));

    if (s <= 1e-6f) {
        return {{1.0f, 0.0f, 0.0f}, 0.0f};
    }

    return {{q.x / s, q.y / s, q.z / s}, angle};
}

inline Quaternion quaternionFromRotationMatrixColumns(float r00,
                                                      float r01,
                                                      float r02,
                                                      float r10,
                                                      float r11,
                                                      float r12,
                                                      float r20,
                                                      float r21,
                                                      float r22) {
    Quaternion q;
    const float trace = r00 + r11 + r22;

    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (r21 - r12) / s;
        q.y = (r02 - r20) / s;
        q.z = (r10 - r01) / s;
    } else if (r00 > r11 && r00 > r22) {
        const float s = std::sqrt(1.0f + r00 - r11 - r22) * 2.0f;
        q.w = (r21 - r12) / s;
        q.x = 0.25f * s;
        q.y = (r01 + r10) / s;
        q.z = (r02 + r20) / s;
    } else if (r11 > r22) {
        const float s = std::sqrt(1.0f + r11 - r00 - r22) * 2.0f;
        q.w = (r02 - r20) / s;
        q.x = (r01 + r10) / s;
        q.y = 0.25f * s;
        q.z = (r12 + r21) / s;
    } else {
        const float s = std::sqrt(1.0f + r22 - r00 - r11) * 2.0f;
        q.w = (r10 - r01) / s;
        q.x = (r02 + r20) / s;
        q.y = (r12 + r21) / s;
        q.z = 0.25f * s;
    }

    return q.normalized();
}

inline mat4f composeMatrix(const vec3f& position,
                           const Quaternion& rotation,
                           const vec3f& scale) {
    const Quaternion q = rotation.normalized();
    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;

    const float r00 = 1.0f - 2.0f * (yy + zz);
    const float r01 = 2.0f * (xy - wz);
    const float r02 = 2.0f * (xz + wy);
    const float r10 = 2.0f * (xy + wz);
    const float r11 = 1.0f - 2.0f * (xx + zz);
    const float r12 = 2.0f * (yz - wx);
    const float r20 = 2.0f * (xz - wy);
    const float r21 = 2.0f * (yz + wx);
    const float r22 = 1.0f - 2.0f * (xx + yy);

    mat4f matrix;
    matrix.setIdentity();
    matrix[0][0] = r00 * scale.x;
    matrix[0][1] = r10 * scale.x;
    matrix[0][2] = r20 * scale.x;
    matrix[1][0] = r01 * scale.y;
    matrix[1][1] = r11 * scale.y;
    matrix[1][2] = r21 * scale.y;
    matrix[2][0] = r02 * scale.z;
    matrix[2][1] = r12 * scale.z;
    matrix[2][2] = r22 * scale.z;
    matrix[3][0] = position.x;
    matrix[3][1] = position.y;
    matrix[3][2] = position.z;
    return matrix;
}

inline vec3f transformPoint(const mat4f& matrix, const vec3f& point) {
    return (vec4f(point.x, point.y, point.z, 1.0f) * matrix).toVec3();
}

inline bool isAffineInvertible(const mat4f& matrix) {
    const float det =
        matrix[0][0] *
            (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
        matrix[0][1] *
            (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
        matrix[0][2] *
            (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    return std::fabs(det) > 1e-8f;
}

inline mat4f makeRenderSpaceFlipMatrix() {
    mat4f flip;
    flip.setIdentity();
    flip[1][1] = -1.0f;
    return flip;
}

inline mat4f toRenderSpaceMatrix(const mat4f& world_matrix) {
    const mat4f flip = makeRenderSpaceFlipMatrix();
    return flip * world_matrix * flip;
}

class Transform {
   public:
    inline void setPosition(const vec3f& value) { position_ = value; }
    inline vec3f getPosition() const { return position_; }

    inline void setRotationEuler(const vec3f& euler_xyz_radians) {
        rotation_ = quaternionFromEuler(euler_xyz_radians);
    }
    inline vec3f getRotationEuler() const {
        return eulerFromQuaternion(rotation_);
    }

    inline void setRotationQuaternion(const Quaternion& value) {
        rotation_ = value.normalized();
    }
    inline Quaternion getRotationQuaternion() const { return rotation_; }

    inline void setRotationAxisAngle(const AxisAngle& value) {
        rotation_ = quaternionFromAxisAngle(value);
    }
    inline AxisAngle getRotationAxisAngle() const {
        return axisAngleFromQuaternion(rotation_);
    }

    inline void setScale(const vec3f& value) { scale_ = value; }
    inline vec3f getScale() const { return scale_; }

    inline void setMatrix(const mat4f& matrix) {
        position_ = {matrix[3][0], matrix[3][1], matrix[3][2]};

        const float sx = std::sqrt(matrix[0][0] * matrix[0][0] +
                                   matrix[0][1] * matrix[0][1] +
                                   matrix[0][2] * matrix[0][2]);
        const float sy = std::sqrt(matrix[1][0] * matrix[1][0] +
                                   matrix[1][1] * matrix[1][1] +
                                   matrix[1][2] * matrix[1][2]);
        const float sz = std::sqrt(matrix[2][0] * matrix[2][0] +
                                   matrix[2][1] * matrix[2][1] +
                                   matrix[2][2] * matrix[2][2]);

        scale_ = {sx, sy, sz};

        if (sx <= 1e-8f || sy <= 1e-8f || sz <= 1e-8f) {
            rotation_ = {};
            return;
        }

        const float row00 = matrix[0][0] / sx;
        const float row01 = matrix[0][1] / sx;
        const float row02 = matrix[0][2] / sx;
        const float row10 = matrix[1][0] / sy;
        const float row11 = matrix[1][1] / sy;
        const float row12 = matrix[1][2] / sy;
        const float row20 = matrix[2][0] / sz;
        const float row21 = matrix[2][1] / sz;
        const float row22 = matrix[2][2] / sz;

        rotation_ = quaternionFromRotationMatrixColumns(
            row00, row10, row20, row01, row11, row21, row02, row12, row22);
    }

    inline mat4f getMatrix() const {
        return composeMatrix(position_, rotation_, scale_);
    }
    inline mat4f getRenderMatrix() const {
        return toRenderSpaceMatrix(getMatrix());
    }

    vec3f position_ = {0.0f, 0.0f, 0.0f};
    Quaternion rotation_;
    vec3f scale_ = {1.0f, 1.0f, 1.0f};
};

using Geometry = std::variant<Sphere, Cylinder, Capsule, Box>;

struct GeometryInstance {
    Geometry geometry;
    Transform transform;

    inline GeometryInstance(const Sphere& value, const Transform& local = {})
        : geometry(value), transform(local) {}
    inline GeometryInstance(const Cylinder& value, const Transform& local = {})
        : geometry(value), transform(local) {}
    inline GeometryInstance(const Capsule& value, const Transform& local = {})
        : geometry(value), transform(local) {}
    inline GeometryInstance(const Box& value, const Transform& local = {})
        : geometry(value), transform(local) {}
};

class CollisionGroup {
   public:
    Transform transform;
    inline void setTransform(const Transform& value) { transform = value; }
    inline const Transform& getTransform() const { return transform; }
    inline Transform& getTransform() { return transform; }

    inline void setPosition(const vec3f& value) {
        transform.setPosition(value);
    }
    inline vec3f getPosition() const { return transform.getPosition(); }

    inline void setRotationEuler(const vec3f& value) {
        transform.setRotationEuler(value);
    }
    inline vec3f getRotationEuler() const {
        return transform.getRotationEuler();
    }

    inline void setRotationQuaternion(const Quaternion& value) {
        transform.setRotationQuaternion(value);
    }
    inline Quaternion getRotationQuaternion() const {
        return transform.getRotationQuaternion();
    }

    inline void setRotationAxisAngle(const AxisAngle& value) {
        transform.setRotationAxisAngle(value);
    }
    inline AxisAngle getRotationAxisAngle() const {
        return transform.getRotationAxisAngle();
    }

    inline void setScale(const vec3f& value) { transform.setScale(value); }
    inline vec3f getScale() const { return transform.getScale(); }

    inline void setMatrix(const mat4f& value) { transform.setMatrix(value); }
    inline mat4f getMatrix() const { return transform.getMatrix(); }

    inline std::size_t add(const Sphere& shape, const Transform& local = {}) {
        geometries_.emplace_back(shape, local);
        return geometries_.size() - 1;
    }
    inline std::size_t add(const Cylinder& shape, const Transform& local = {}) {
        geometries_.emplace_back(shape, local);
        return geometries_.size() - 1;
    }
    inline std::size_t add(const Capsule& shape, const Transform& local = {}) {
        geometries_.emplace_back(shape, local);
        return geometries_.size() - 1;
    }
    inline std::size_t add(const Box& shape, const Transform& local = {}) {
        geometries_.emplace_back(shape, local);
        return geometries_.size() - 1;
    }

    inline const std::vector<GeometryInstance>& geometries() const {
        return geometries_;
    }
    inline std::vector<GeometryInstance>& geometries() { return geometries_; }

    inline bool contains(const vec3f& point) const {
        return containsImpl(point, true);
    }

    inline bool containsWorldPoint(const vec3f& point) const {
        return containsImpl(point, false);
    }

    std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> to_sdf() const;

   private:
    inline bool containsImpl(const vec3f& point, bool use_render_space) const {
        const mat4f global_matrix = use_render_space
                                        ? transform.getRenderMatrix()
                                        : transform.getMatrix();

        for (const GeometryInstance& geometry : geometries_) {
            mat4f world_matrix =
                (use_render_space ? geometry.transform.getRenderMatrix()
                                  : geometry.transform.getMatrix()) *
                global_matrix;
            // 这个是无效的
            // mat4f world_matrix = global_matrix *
            // geometry.transform.getMatrix();
            if (!isAffineInvertible(world_matrix)) {
                continue;
            }

            world_matrix.invert();
            const vec3f local_point = transformPoint(world_matrix, point);

            const bool hit = std::visit(
                [&](const auto& shape) { return shape.contains(local_point); },
                geometry.geometry);
            if (hit) {
                return true;
            }
        }

        return false;
    }

   private:
    std::vector<GeometryInstance> geometries_;
};

inline bool pointIntersects(const vec3f& point, const CollisionGroup& group) {
    return group.contains(point);
}

inline bool pointIntersectsWorld(const vec3f& point,
                                 const CollisionGroup& group) {
    return group.containsWorldPoint(point);
}

inline std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> CollisionGroup::to_sdf()
    const {
    using namespace sinriv::kigstudio::sdf;
    std::vector<std::shared_ptr<SDFBase>> children;
    children.reserve(geometries_.size());

    const mat4f global_matrix = transform.getRenderMatrix();

    for (const auto& geometry : geometries_) {
        const mat4f world_matrix =
            geometry.transform.getRenderMatrix() * global_matrix;
        if (!isAffineInvertible(world_matrix)) {
            continue;
        }

        mat4f inv_matrix = world_matrix;
        inv_matrix.invert();

        std::shared_ptr<SDFBase> shape_sdf = std::visit(
            [](const auto& shape) -> std::shared_ptr<SDFBase> {
                using ShapeType = std::decay_t<decltype(shape)>;
                return std::make_shared<ShapeType>(shape);
            },
            geometry.geometry);

        children.push_back(
            std::make_shared<SDF_AffineTransform>(inv_matrix, shape_sdf));
    }

    if (children.empty()) {
        return nullptr;
    }
    if (children.size() == 1) {
        return children.front();
    }
    return sdf_group(std::move(children));
}
}  // namespace voxel::collision
inline cJSON* to_json(const voxel::collision::Quaternion& q) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(obj, "x", q.x);
    cJSON_AddNumberToObject(obj, "y", q.y);
    cJSON_AddNumberToObject(obj, "z", q.z);
    cJSON_AddNumberToObject(obj, "w", q.w);
    return obj;
}

inline voxel::collision::Quaternion from_json_quat(const cJSON* obj) {
    float w = 0, x = 0, y = 0, z = 0;
    if (cJSON_IsObject(obj)) {
        const cJSON* child = nullptr;
        cJSON_ArrayForEach(child, obj) {
            if (!child->string || !cJSON_IsNumber(child))
                continue;

            const float value = (float)child->valuedouble;
            if (strcmp(child->string, "w") == 0) {
                w = value;
            } else if (strcmp(child->string, "x") == 0) {
                x = value;
            } else if (strcmp(child->string, "y") == 0) {
                y = value;
            } else if (strcmp(child->string, "z") == 0) {
                z = value;
            }
        }
    }
    return {x, y, z, w};
}
inline cJSON* to_json(const voxel::collision::Transform& t) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddItemToObject(obj, "position", to_json(t.getPosition()));
    cJSON_AddItemToObject(obj, "rotation", to_json(t.getRotationQuaternion()));
    cJSON_AddItemToObject(obj, "scale", to_json(t.getScale()));
    return obj;
}

inline voxel::collision::Transform from_json_transform(const cJSON* obj) {
    voxel::collision::Transform t;
    if (!cJSON_IsObject(obj))
        return t;

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, obj) {
        if (!child->string || !cJSON_IsObject(child))
            continue;

        if (strcmp(child->string, "position") == 0) {
            t.setPosition(vec3_from_json<voxel::collision::vec3f>(child));
        } else if (strcmp(child->string, "rotation") == 0) {
            t.setRotationQuaternion(from_json_quat(child));
        } else if (strcmp(child->string, "scale") == 0) {
            t.setScale(vec3_from_json<voxel::collision::vec3f>(child));
        }
    }
    return t;
}
inline cJSON* to_json(const voxel::collision::Sphere& s) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddItemToObject(obj, "center", to_json(s.center));
    cJSON_AddNumberToObject(obj, "radius", s.radius);
    return obj;
}

inline voxel::collision::Sphere from_json_sphere(const cJSON* obj) {
    voxel::collision::Sphere s;
    if (!cJSON_IsObject(obj))
        return s;

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, obj) {
        if (!child->string)
            continue;

        if (cJSON_IsObject(child) && strcmp(child->string, "center") == 0) {
            s.center = vec3_from_json<voxel::collision::vec3f>(child);
        } else if (cJSON_IsNumber(child) &&
                   strcmp(child->string, "radius") == 0) {
            s.radius = (float)child->valuedouble;
        }
    }
    return s;
}

inline cJSON* to_json(const voxel::collision::Cylinder& c) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddItemToObject(obj, "start", to_json(c.start));
    cJSON_AddItemToObject(obj, "end", to_json(c.end));
    cJSON_AddNumberToObject(obj, "radius", c.radius);
    return obj;
}

inline voxel::collision::Cylinder from_json_cylinder(const cJSON* obj) {
    voxel::collision::vec3f start;
    voxel::collision::vec3f end;
    float radius = 0.0f;

    if (cJSON_IsObject(obj)) {
        const cJSON* child = nullptr;
        cJSON_ArrayForEach(child, obj) {
            if (!child->string)
                continue;

            if (cJSON_IsObject(child)) {
                if (strcmp(child->string, "start") == 0) {
                    start = vec3_from_json<voxel::collision::vec3f>(child);
                } else if (strcmp(child->string, "end") == 0) {
                    end = vec3_from_json<voxel::collision::vec3f>(child);
                }
            } else if (cJSON_IsNumber(child) &&
                       strcmp(child->string, "radius") == 0) {
                radius = (float)child->valuedouble;
            }
        }
    }
    return voxel::collision::Cylinder(start, end, radius);
}

inline cJSON* to_json(const voxel::collision::Capsule& c) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddItemToObject(obj, "start", to_json(c.start));
    cJSON_AddItemToObject(obj, "end", to_json(c.end));
    cJSON_AddNumberToObject(obj, "radius", c.radius);
    return obj;
}

inline voxel::collision::Capsule from_json_capsule(const cJSON* obj) {
    voxel::collision::vec3f start;
    voxel::collision::vec3f end;
    float radius = 0.0f;

    if (cJSON_IsObject(obj)) {
        const cJSON* child = nullptr;
        cJSON_ArrayForEach(child, obj) {
            if (!child->string)
                continue;

            if (cJSON_IsObject(child)) {
                if (strcmp(child->string, "start") == 0) {
                    start = vec3_from_json<voxel::collision::vec3f>(child);
                } else if (strcmp(child->string, "end") == 0) {
                    end = vec3_from_json<voxel::collision::vec3f>(child);
                }
            } else if (cJSON_IsNumber(child) &&
                       strcmp(child->string, "radius") == 0) {
                radius = (float)child->valuedouble;
            }
        }
    }
    return voxel::collision::Capsule(start, end, radius);
}

inline cJSON* to_json(const voxel::collision::Box& b) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddItemToObject(obj, "half_extent", to_json(b.half_extent));
    return obj;
}

inline voxel::collision::Box from_json_box(const cJSON* obj) {
    voxel::collision::Box b;
    if (!cJSON_IsObject(obj))
        return b;

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, obj) {
        if (!child->string || !cJSON_IsObject(child))
            continue;

        if (strcmp(child->string, "half_extent") == 0) {
            b.half_extent = vec3_from_json<voxel::collision::vec3f>(child);
        }
    }
    return b;
}

inline cJSON* to_json(const voxel::collision::Geometry& g) {
    cJSON* obj = cJSON_CreateObject();

    std::visit(
        [&](auto&& shape) {
            using T = std::decay_t<decltype(shape)>;

            if constexpr (std::is_same_v<T, voxel::collision::Sphere>) {
                cJSON_AddStringToObject(obj, "type", "sphere");
                cJSON_AddItemToObject(obj, "data", to_json(shape));
            } else if constexpr (std::is_same_v<T,
                                                voxel::collision::Cylinder>) {
                cJSON_AddStringToObject(obj, "type", "cylinder");
                cJSON_AddItemToObject(obj, "data", to_json(shape));
            } else if constexpr (std::is_same_v<T, voxel::collision::Capsule>) {
                cJSON_AddStringToObject(obj, "type", "capsule");
                cJSON_AddItemToObject(obj, "data", to_json(shape));
            } else if constexpr (std::is_same_v<T, voxel::collision::Box>) {
                cJSON_AddStringToObject(obj, "type", "box");
                cJSON_AddItemToObject(obj, "data", to_json(shape));
            }
        },
        g);

    return obj;
}

inline voxel::collision::Geometry from_json_geometry(const cJSON* obj) {
    const char* type = nullptr;
    const cJSON* data = nullptr;

    if (cJSON_IsObject(obj)) {
        const cJSON* child = nullptr;
        cJSON_ArrayForEach(child, obj) {
            if (!child->string)
                continue;

            if (cJSON_IsString(child) && strcmp(child->string, "type") == 0) {
                type = child->valuestring;
            } else if (cJSON_IsObject(child) &&
                       strcmp(child->string, "data") == 0) {
                data = child;
            }
        }
    }

    if (!type || !data)
        return voxel::collision::Sphere{};

    if (strcmp(type, "sphere") == 0) {
        return from_json_sphere(data);
    } else if (strcmp(type, "cylinder") == 0) {
        return from_json_cylinder(data);
    } else if (strcmp(type, "capsule") == 0) {
        return from_json_capsule(data);
    } else if (strcmp(type, "box") == 0) {
        return from_json_box(data);
    }

    return voxel::collision::Sphere{};
}

inline cJSON* to_json(const voxel::collision::GeometryInstance& gi) {
    cJSON* obj = cJSON_CreateObject();

    cJSON_AddItemToObject(obj, "geometry", to_json(gi.geometry));
    cJSON_AddItemToObject(obj, "transform", to_json(gi.transform));

    return obj;
}

inline voxel::collision::GeometryInstance from_json_geometry_instance(
    const cJSON* obj) {
    voxel::collision::Geometry geometry;
    voxel::collision::Transform transform;

    if (cJSON_IsObject(obj)) {
        const cJSON* child = nullptr;
        cJSON_ArrayForEach(child, obj) {
            if (!child->string || !cJSON_IsObject(child))
                continue;

            if (strcmp(child->string, "geometry") == 0) {
                geometry = from_json_geometry(child);
            } else if (strcmp(child->string, "transform") == 0) {
                transform = from_json_transform(child);
            }
        }
    }

    return std::visit(
        [&](auto&& shape) {
            return voxel::collision::GeometryInstance(shape, transform);
        },
        geometry);
}

inline cJSON* to_json(const voxel::collision::CollisionGroup& group) {
    cJSON* obj = cJSON_CreateObject();

    cJSON_AddItemToObject(obj, "transform", to_json(group.getTransform()));

    cJSON* geometries = cJSON_CreateArray();
    for (const auto& gi : group.geometries()) {
        cJSON_AddItemToArray(geometries, to_json(gi));
    }
    cJSON_AddItemToObject(obj, "geometries", geometries);

    return obj;
}

inline voxel::collision::CollisionGroup from_json_collision_group(
    const cJSON* obj) {
    voxel::collision::CollisionGroup group;

    if (!cJSON_IsObject(obj))
        return group;

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, obj) {
        if (!child->string)
            continue;

        if (cJSON_IsObject(child) && strcmp(child->string, "transform") == 0) {
            group.setTransform(from_json_transform(child));
        } else if (cJSON_IsArray(child) &&
                   strcmp(child->string, "geometries") == 0) {
            const int count = cJSON_GetArraySize(child);
            for (int i = 0; i < count; ++i) {
                const cJSON* gi = cJSON_GetArrayItem(child, i);
                if (gi && cJSON_IsObject(gi)) {
                    group.geometries().push_back(
                        from_json_geometry_instance(gi));
                }
            }
        }
    }

    return group;
}

}  // namespace sinriv::kigstudio