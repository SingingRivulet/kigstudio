#include "kigstudio/sdf/sdf.h"

namespace sinriv::kigstudio::sdf {

std::shared_ptr<SDF_bool> sdf_union(std::shared_ptr<SDFBase> a,
                                    std::shared_ptr<SDFBase> b) {
    return std::make_shared<SDF_bool>(SDFBoolOp::Union, std::move(a),
                                      std::move(b));
}

std::shared_ptr<SDF_bool> sdf_intersection(std::shared_ptr<SDFBase> a,
                                           std::shared_ptr<SDFBase> b) {
    return std::make_shared<SDF_bool>(SDFBoolOp::Intersection, std::move(a),
                                      std::move(b));
}

std::shared_ptr<SDF_bool> sdf_subtraction(std::shared_ptr<SDFBase> a,
                                          std::shared_ptr<SDFBase> b) {
    return std::make_shared<SDF_bool>(SDFBoolOp::Subtraction, std::move(a),
                                      std::move(b));
}

float SDFGrid::get(const Vec3f& p) const {
    if (sx <= 0 || sy <= 0 || sz <= 0 || sdf.empty()) {
        return 1e6f;
    }

    float dx = p.x - static_cast<float>(min_bound.x);
    float dy = p.y - static_cast<float>(min_bound.y);
    float dz = p.z - static_cast<float>(min_bound.z);

    dx = std::clamp(dx, 0.0f, static_cast<float>(sx - 1));
    dy = std::clamp(dy, 0.0f, static_cast<float>(sy - 1));
    dz = std::clamp(dz, 0.0f, static_cast<float>(sz - 1));

    int x0 = static_cast<int>(std::floor(dx));
    int y0 = static_cast<int>(std::floor(dy));
    int z0 = static_cast<int>(std::floor(dz));
    int x1 = std::min(x0 + 1, sx - 1);
    int y1 = std::min(y0 + 1, sy - 1);
    int z1 = std::min(z0 + 1, sz - 1);

    float fx = dx - static_cast<float>(x0);
    float fy = dy - static_cast<float>(y0);
    float fz = dz - static_cast<float>(z0);

    float c000 = get(x0, y0, z0);
    float c100 = get(x1, y0, z0);
    float c010 = get(x0, y1, z0);
    float c110 = get(x1, y1, z0);
    float c001 = get(x0, y0, z1);
    float c101 = get(x1, y0, z1);
    float c011 = get(x0, y1, z1);
    float c111 = get(x1, y1, z1);

    float c00 = c000 * (1.0f - fx) + c100 * fx;
    float c10 = c010 * (1.0f - fx) + c110 * fx;
    float c01 = c001 * (1.0f - fx) + c101 * fx;
    float c11 = c011 * (1.0f - fx) + c111 * fx;

    float c0 = c00 * (1.0f - fy) + c10 * fy;
    float c1 = c01 * (1.0f - fy) + c11 * fy;

    return c0 * (1.0f - fz) + c1 * fz;
}

}  // namespace sinriv::kigstudio::sdf
