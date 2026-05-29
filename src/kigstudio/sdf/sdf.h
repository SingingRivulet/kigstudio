#pragma once
#include <cJSON.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "kigstudio/utils/vec3.h"

namespace sinriv::kigstudio::sdf {

using Vec3f = sinriv::kigstudio::vec3<float>;

struct SDF_QueryResult {
    float distance;
    Vec3f normal;
    Vec3f position;
};

struct SDFBase {
    virtual ~SDFBase() = default;
    virtual float get(const Vec3f& p) const = 0;

    inline float get(float x, float y, float z) const {
        return get(Vec3f(x, y, z));
    }
    virtual void get(const Vec3f& begin,
                     const Vec3f& voxelSize,
                     const Vec3i& voxelCount,
                     std::vector<float>& out) const;
    virtual std::string getInfo() const = 0;
    virtual cJSON* toJSON() const = 0;
    virtual void fromJSON(const cJSON* json) = 0;
};
using SDFBasePtr = std::shared_ptr<SDFBase>;

std::shared_ptr<SDFBase> sdf_from_json(const cJSON* json);
void sdf_register_type(
    const std::string& name,
    std::function<std::shared_ptr<SDFBase>(const cJSON*)> factory);

enum class SDFBoolOp { Union, Intersection, Subtraction };

struct SDF_bool : public SDFBase {
    SDFBoolOp op;
    std::shared_ptr<SDFBase> left;
    std::shared_ptr<SDFBase> right;

    SDF_bool(SDFBoolOp op,
             std::shared_ptr<SDFBase> left,
             std::shared_ptr<SDFBase> right)
        : op(op), left(std::move(left)), right(std::move(right)) {}

    float get(const Vec3f& p) const override {
        float a = left->get(p);
        float b = right->get(p);
        switch (op) {
            case SDFBoolOp::Union:
                return std::min(a, b);
            case SDFBoolOp::Intersection:
                return std::max(a, b);
            case SDFBoolOp::Subtraction:
                return std::max(a, -b);
        }
        return a;
    }

    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo() const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
};

std::shared_ptr<SDF_bool> sdf_union(std::shared_ptr<SDFBase> a,
                                    std::shared_ptr<SDFBase> b);
std::shared_ptr<SDF_bool> sdf_intersection(std::shared_ptr<SDFBase> a,
                                           std::shared_ptr<SDFBase> b);
std::shared_ptr<SDF_bool> sdf_subtraction(std::shared_ptr<SDFBase> a,
                                          std::shared_ptr<SDFBase> b);
std::shared_ptr<SDFBase> sdf_group(std::vector<std::shared_ptr<SDFBase>> children);

struct SDF_Group : public SDFBase {
    std::vector<std::shared_ptr<SDFBase>> children;

    SDF_Group() = default;
    explicit SDF_Group(std::vector<std::shared_ptr<SDFBase>> children)
        : children(std::move(children)) {}

    float get(const Vec3f& p) const override;
    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo() const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
};

struct SDF_Translate : public SDFBase {
    Vec3f offset;
    std::shared_ptr<SDFBase> child;

    SDF_Translate(const Vec3f& offset, std::shared_ptr<SDFBase> child)
        : offset(offset), child(std::move(child)) {}

    float get(const Vec3f& p) const override { return child->get(p - offset); }
    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo() const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
};

struct SDF_Offset : public SDFBase {
    float offset;
    std::shared_ptr<SDFBase> child;

    SDF_Offset(float offset, std::shared_ptr<SDFBase> child)
        : offset(offset), child(std::move(child)) {}

    float get(const Vec3f& p) const override { return child->get(p) - offset; }
    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo() const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
};

struct SDF_Plane : public SDFBase {
    // Plane equation: Ax + By + Cz + D = 0 (A²+B²+C²=1, normalized normal)
    // Signed distance = A*x + B*y + C*z + D (when normal is normalized)
    float A, B, C, D;

    SDF_Plane() : A(0), B(0), C(1), D(0) {}
    SDF_Plane(float a, float b, float c, float d) : A(a), B(b), C(c), D(d) {
        // Normalize the normal vector (A, B, C)
        float len = std::sqrt(A * A + B * B + C * C);
        if (len > 1e-6f) {
            A /= len;
            B /= len;
            C /= len;
            D /= len;
        }
    }

    float get(const Vec3f& p) const override {
        // Signed distance from point to plane
        return A * p.x + B * p.y + C * p.z + D;
    }

    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo() const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
};

struct SDFGrid : public SDFBase {
    Vec3i min_bound;
    Vec3i max_bound;

    int sx = 0;
    int sy = 0;
    int sz = 0;
    SDFGrid() = default;

    std::vector<float> sdf;

    inline int index(int x, int y, int z) const {
        return (z * sy + y) * sx + x;
    }

    inline bool inBounds(int x, int y, int z) const {
        return x >= 0 && y >= 0 && z >= 0 && x < sx && y < sy && z < sz;
    }

    inline float get(int x, int y, int z) const { return sdf[index(x, y, z)]; }

    inline void set(int x, int y, int z, float v) { sdf[index(x, y, z)] = v; }

    inline Vec3i denseToWorldVoxel(int x, int y, int z) const {
        return Vec3i(x + min_bound.x, y + min_bound.y, z + min_bound.z);
    }

    inline Vec3i worldVoxelToDense(const Vec3i& p) const {
        return Vec3i(p.x - min_bound.x, p.y - min_bound.y, p.z - min_bound.z);
    }

    float get(const Vec3f& p) const override;

    std::string getInfo() const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;
};

}  // namespace sinriv::kigstudio::sdf
