#pragma once
#include <cJSON.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "kigstudio/sdf/sdf.h"
#include "kigstudio/utils/mat.h"
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

inline float sdCappedCylinderX(const Vec3f& p,
                               float radius,
                               float half_height) {
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

inline float opUnion(float a, float b) {
    return std::min(a, b);
}
inline float opSubtraction(float a, float b) {
    return std::max(a, -b);
}
inline float opIntersection(float a, float b) {
    return std::max(a, b);
}

inline float sdFiniteCone(const Vec3f& p, float angle_rad, float height) {
    float cone = sdCone(p, angle_rad);
    cone = opIntersection(cone, -p.z);
    cone = opIntersection(cone, p.z - height);
    return cone;
}

inline float sdSphere(const Vec3f& p, float radius) {
    return std::sqrt(dot(p, p)) - radius;
}

inline float sdSegment(const Vec3f& p, const Vec3f& a, const Vec3f& b) {
    Vec3f ab = b - a;
    float denom = dot(ab, ab);
    float t = 0.0f;
    if (denom > 0.0f) {
        t = dot(p - a, ab) / denom;
        t = std::max(0.0f, std::min(1.0f, t));
    }
    Vec3f closest = a + ab * t;
    return std::sqrt(dot(p - closest, p - closest));
}

inline float sdCylinder(const Vec3f& p,
                        const Vec3f& start,
                        const Vec3f& end,
                        float radius) {
    return sdSegment(p, start, end) - radius;
}

inline float sdCapsule(const Vec3f& p,
                       const Vec3f& start,
                       const Vec3f& end,
                       float radius) {
    return sdSegment(p, start, end) - radius;
}

inline float sdBox(const Vec3f& p, const Vec3f& half_extent) {
    Vec3f d(
        std::fabs(p.x) - half_extent.x,
        std::fabs(p.y) - half_extent.y,
        std::fabs(p.z) - half_extent.z);
    float outside = std::sqrt(
        fmaxf(d.x, 0.0f) * fmaxf(d.x, 0.0f) +
        fmaxf(d.y, 0.0f) * fmaxf(d.y, 0.0f) +
        fmaxf(d.z, 0.0f) * fmaxf(d.z, 0.0f));
    return fminf(fmaxf(d.x, fmaxf(d.y, d.z)), 0.0f) + outside;
}

struct SDF_Box : public SDFBase {
    Vec3f half_extent;

    inline SDF_Box() : half_extent(0.0f, 0.0f, 0.0f) {}
    inline SDF_Box(const Vec3f& half_extent) : half_extent(half_extent) {}

    float get(const Vec3f& p) const override;
    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo(int indent = 0) const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
    inline bool contains(const Vec3f& p) const { return get(p) <= 0.0f; }
};

struct SDF_Sphere : public SDFBase {
    Vec3f center;
    float radius;

    inline SDF_Sphere() : center(0.0f, 0.0f, 0.0f), radius(0.0f) {}
    inline SDF_Sphere(const Vec3f& center, float radius)
        : center(center), radius(radius) {}

    float get(const Vec3f& p) const override;
    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo(int indent = 0) const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
    inline bool contains(const Vec3f& p) const { return get(p) <= 0.0f; }
};

struct SDF_Cylinder : public SDFBase {
    Vec3f start;
    Vec3f end;
    float radius;

    inline SDF_Cylinder()
        : start(0.0f, 0.0f, 0.0f), end(0.0f, 0.0f, 0.0f), radius(0.0f) {}
    inline SDF_Cylinder(const Vec3f& start, const Vec3f& end, float radius)
        : start(start), end(end), radius(radius) {}

    float get(const Vec3f& p) const override;
    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo(int indent = 0) const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
    inline bool contains(const Vec3f& p) const { return get(p) <= 0.0f; }
};

struct SDF_Capsule : public SDFBase {
    Vec3f start;
    Vec3f end;
    float radius;

    inline SDF_Capsule()
        : start(0.0f, 0.0f, 0.0f), end(0.0f, 0.0f, 0.0f), radius(0.0f) {}
    inline SDF_Capsule(const Vec3f& start, const Vec3f& end, float radius)
        : start(start), end(end), radius(radius) {}

    float get(const Vec3f& p) const override;
    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo(int indent = 0) const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
    inline bool contains(const Vec3f& p) const { return get(p) <= 0.0f; }
};

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

    std::string getInfo(int indent = 0) const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
};

struct SDF_PolyCone final : public SDFBase {
    Vec3f apex;
    std::vector<Vec3f> base_vertices;
    mutable std::vector<Vec3f> plane_normals;

    SDF_PolyCone() = default;
    SDF_PolyCone(const Vec3f& apex, const std::vector<Vec3f>& base_vertices)
        : apex(apex), base_vertices(base_vertices) {}

    void buildNormals() const;
    float get(const Vec3f& p) const override;
    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo(int indent = 0) const override;
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

    std::string getInfo(int indent = 0) const override;
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

    std::string getInfo(int indent = 0) const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
};

struct SDF_AffineTransform final : public SDFBase {
    sinriv::kigstudio::mat::matrix<float> inv_matrix;
    std::shared_ptr<SDFBase> child;

    inline SDF_AffineTransform(
        const sinriv::kigstudio::mat::matrix<float>& inv_matrix,
        std::shared_ptr<SDFBase> child)
        : inv_matrix(inv_matrix), child(std::move(child)) {}

    float get(const Vec3f& p) const override;
    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo(int indent = 0) const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
};

}  // namespace sinriv::kigstudio::sdf
