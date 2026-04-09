#pragma once
#include "kigstudio/utils/vec3.h"
#include <algorithm>
#include <cmath>

namespace sinriv::kigstudio::voxel::collision {
using vec3f = sinriv::kigstudio::vec3<float>;

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
}
