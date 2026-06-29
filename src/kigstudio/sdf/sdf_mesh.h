#pragma once
#include "kigstudio/sdf/sdf.h"
#include <memory>
#include <string>

namespace sinriv::kigstudio::sdf {

// SDF precision mode — shared between SDF mesh and voxelizer.
enum class SDFPrecision {
    Fast = 0,       // ray voting (3 axes), no AABB distance, no side_tester
    Precise = 1,    // AABB tree distance + side_tester verification
    Redundant = 2,  // ray voting (3 axes + 4 diagonals → 7 rays), need ≥4 inside
};

struct SDF_Mesh : public SDFBase {
    SDF_Mesh();
    ~SDF_Mesh() override;

    std::string path;

    bool loadSTL(const std::string& filename);

    using Triangle = std::tuple<
        sinriv::kigstudio::vec3<float>,
        sinriv::kigstudio::vec3<float>,
        sinriv::kigstudio::vec3<float>>;
    bool loadTriangles(const std::vector<Triangle>& triangles);

    float get(const Vec3f& p) const override;
    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    bool isInside(const Vec3f& p) const;
    bool hasInsideTester() const;

    // Precision mode for SDF computation.
    SDFPrecision precision_mode = SDFPrecision::Precise;

    std::string getInfo(int indent = 0) const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

}  // namespace sinriv::kigstudio::sdf
