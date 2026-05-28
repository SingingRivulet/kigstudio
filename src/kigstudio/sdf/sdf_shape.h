#pragma once
#include <cJSON.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "kigstudio/sdf/sdf.h"
#include "kigstudio/utils/vec3.h"

namespace sinriv::kigstudio::sdf {

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

inline float sdCappedCylinder(const Vec3f& p, float radius, float half_height) {
    float dx = std::sqrt(p.x * p.x + p.y * p.y) - radius;
    float dy = std::abs(p.z) - half_height;

    float ax = std::max(dx, 0.0f);
    float ay = std::max(dy, 0.0f);

    return std::min(std::max(dx, dy), 0.0f) + std::sqrt(ax * ax + ay * ay);
}

inline float sdCappedCylinderX(const Vec3f& p, float radius, float half_height) {
    float r = std::sqrt(p.y * p.y + p.z * p.z);
    float dx = r - radius;
    float dy = std::abs(p.x) - half_height;

    float ax = std::max(dx, 0.0f);
    float ay = std::max(dy, 0.0f);

    return std::min(std::max(dx, dy), 0.0f) + std::sqrt(ax * ax + ay * ay);
}

inline float sdCone(const Vec3f& p, float angle_rad) {
    float r = std::sqrt(p.x * p.x + p.y * p.y);
    return r - p.z * std::tan(angle_rad);
}

inline float opUnion(float a, float b) { return std::min(a, b); }
inline float opSubtraction(float a, float b) { return std::max(a, -b); }
inline float opIntersection(float a, float b) { return std::max(a, b); }

inline float sdFiniteCone(const Vec3f& p, float angle_rad, float height) {
    float cone = sdCone(p, angle_rad);
    cone = opIntersection(cone, -p.z);
    cone = opIntersection(cone, p.z - height);
    return cone;
}

struct SDF_FiniteCone final : public SDFBase {
    float angle_rad;
    float height;

    inline SDF_FiniteCone(float angle_rad, float height)
        : angle_rad(angle_rad), height(height) {}

    float get(const Vec3f& p) const override;
    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo() const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
};

struct SDF_CappedCylinderX final : public SDFBase {
    float radius;
    float half_height;

    inline SDF_CappedCylinderX(float radius, float half_height)
        : radius(radius), half_height(half_height) {}

    float get(const Vec3f& p) const override;
    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo() const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
};

struct SDF_FrameTransform final : public SDFBase {
    Frame frame;
    std::shared_ptr<SDFBase> child;

    inline SDF_FrameTransform(const Frame& frame,
                              std::shared_ptr<SDFBase> child)
        : frame(frame), child(std::move(child)) {}

    float get(const Vec3f& p) const override;
    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo() const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
};

}  // namespace sinriv::kigstudio::sdf
