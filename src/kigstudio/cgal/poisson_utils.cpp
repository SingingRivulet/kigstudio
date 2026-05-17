#include "kigstudio/cgal/poisson_utils.h"

namespace sinriv::kigstudio::cgal {

std::vector<std::pair<vec3f, vec3f>>
samplePointCloudFromTriangles(const std::vector<Triangle>& triangles) {
    std::vector<std::pair<vec3f, vec3f>> result;
    result.reserve(triangles.size());
    for (const auto& tri : triangles) {
        auto a = std::get<0>(tri);
        auto b = std::get<1>(tri);
        auto c = std::get<2>(tri);
        vec3f center = (a + b + c) * (1.0f / 3.0f);
        vec3f normal = (b - a).cross(c - a).normalize();
        result.push_back({center, normal});
    }
    return result;
}

} // namespace sinriv::kigstudio::cgal
