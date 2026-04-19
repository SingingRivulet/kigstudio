#pragma once
#include "kigstudio/utils/vec3.h"
#include "kigstudio/utils/mat.h"
#include <algorithm>
#include <cmath>
#include <utility>
#include <variant>
#include <vector>

namespace sinriv::kigstudio::voxel::collision {
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

struct Sphere {
    vec3f center = {0.0f, 0.0f, 0.0f};
    float radius = 0.0f;

    inline bool contains(const vec3f& point) const {
        return lengthSquared(point - center) <= radius * radius;
    }
};

struct Cylinder {
    vec3f start = {0.0f, 0.0f, 0.0f};
    vec3f end = {0.0f, 0.0f, 0.0f};
    float radius = 0.0f;

    inline bool contains(const vec3f& point) const {
        const vec3f axis = end - start;
        const float axis_length_sq = lengthSquared(axis);

        if (axis_length_sq <= 0.0f) {
            return lengthSquared(point - start) <= radius * radius;
        }

        const float t = (point - start).dot(axis) / axis_length_sq;
        if (t < 0.0f || t > 1.0f) {
            return false;
        }

        const vec3f closest = start + axis * t;
        return lengthSquared(point - closest) <= radius * radius;
    }
};

struct Capsule {
    vec3f start = {0.0f, 0.0f, 0.0f};
    vec3f end = {0.0f, 0.0f, 0.0f};
    float radius = 0.0f;

    inline bool contains(const vec3f& point) const {
        const vec3f closest = closestPointOnSegment(point, start, end);
        return lengthSquared(point - closest) <= radius * radius;
    }
};

struct OBB {
    vec3f center = {0.0f, 0.0f, 0.0f};
    vec3f half_extent = {0.0f, 0.0f, 0.0f};
    vec3f axis_x = {1.0f, 0.0f, 0.0f};
    vec3f axis_y = {0.0f, 1.0f, 0.0f};
    vec3f axis_z = {0.0f, 0.0f, 1.0f};

    inline bool contains(const vec3f& point) const {
        const vec3f local = point - center;

        const float x = local.dot(axis_x);
        const float y = local.dot(axis_y);
        const float z = local.dot(axis_z);

        return std::fabs(x) <= half_extent.x &&
               std::fabs(y) <= half_extent.y &&
               std::fabs(z) <= half_extent.z;
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

inline bool pointIntersects(const vec3f& point, const OBB& obb) {
    return obb.contains(point);
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
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
        matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
        matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    return std::fabs(det) > 1e-8f;
}

class Transform {
   public:
    inline void setPosition(const vec3f& value) { position_ = value; }
    inline vec3f getPosition() const { return position_; }

    inline void setRotationEuler(const vec3f& euler_xyz_radians) {
        rotation_ = quaternionFromEuler(euler_xyz_radians);
    }
    inline vec3f getRotationEuler() const { return eulerFromQuaternion(rotation_); }

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
            row00, row10, row20,
            row01, row11, row21,
            row02, row12, row22);
    }

    inline mat4f getMatrix() const { return composeMatrix(position_, rotation_, scale_); }
    inline mat4f getBgfxMatrix() const {
        Quaternion tmp_rotation;
        tmp_rotation.x = -rotation_.x;
        tmp_rotation.y = -rotation_.y;
        tmp_rotation.z = -rotation_.z;
        tmp_rotation.w = rotation_.w;
        //bgfx渲染需要使用共轭的四元数，原因未知
        vec3f tmp_position;
        tmp_position.x = position_.x;
        tmp_position.y = -position_.y;
        tmp_position.z = position_.z;
        return composeMatrix(tmp_position, tmp_rotation, scale_); 
    }

   private:
    vec3f position_ = {0.0f, 0.0f, 0.0f};
    Quaternion rotation_;
    vec3f scale_ = {1.0f, 1.0f, 1.0f};
};

using Geometry = std::variant<Sphere, Cylinder, Capsule, OBB>;

struct GeometryInstance {
    Geometry geometry;
    Transform transform;

    inline GeometryInstance(const Sphere& value, const Transform& local = {})
        : geometry(value), transform(local) {}
    inline GeometryInstance(const Cylinder& value, const Transform& local = {})
        : geometry(value), transform(local) {}
    inline GeometryInstance(const Capsule& value, const Transform& local = {})
        : geometry(value), transform(local) {}
    inline GeometryInstance(const OBB& value, const Transform& local = {})
        : geometry(value), transform(local) {}
};

class CollisionGroup {
   public:
    Transform transform;
    inline void setTransform(const Transform& value) { transform = value; }
    inline const Transform& getTransform() const { return transform; }
    inline Transform& getTransform() { return transform; }

    inline void setPosition(const vec3f& value) { transform.setPosition(value); }
    inline vec3f getPosition() const { return transform.getPosition(); }

    inline void setRotationEuler(const vec3f& value) {
        transform.setRotationEuler(value);
    }
    inline vec3f getRotationEuler() const { return transform.getRotationEuler(); }

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
    inline std::size_t add(const OBB& shape, const Transform& local = {}) {
        geometries_.emplace_back(shape, local);
        return geometries_.size() - 1;
    }

    inline const std::vector<GeometryInstance>& geometries() const {
        return geometries_;
    }
    inline std::vector<GeometryInstance>& geometries() { return geometries_; }

    inline bool contains(const vec3f& point) const {
        const mat4f global_matrix = transform.getBgfxMatrix();

        for (const GeometryInstance& geometry : geometries_) {
            mat4f world_matrix = geometry.transform.getBgfxMatrix() * global_matrix;
            // 这个是无效的
            // mat4f world_matrix = global_matrix * geometry.transform.getMatrix();
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
}
