#pragma once
#include "kigstudio/sdf/sdf.h"
#include <memory>
#include <string>

namespace sinriv::kigstudio::sdf {

struct SDF_Mesh : public SDFBase {
    SDF_Mesh();
    ~SDF_Mesh() override;

    std::string path;

    bool loadSTL(const std::string& filename);

    float get(const Vec3f& p) const override;
    void get(const Vec3f& begin,
             const Vec3f& voxelSize,
             const Vec3i& voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo() const override;
    cJSON* toJSON() const override;
    void fromJSON(const cJSON* json) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

}  // namespace sinriv::kigstudio::sdf
