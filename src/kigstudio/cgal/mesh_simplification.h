#pragma once
#include <vector>
#include <tuple>
#include "kigstudio/voxel/voxel2mesh.h"

namespace sinriv::kigstudio::cgal {

using vec3f = sinriv::kigstudio::vec3<float>;
using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;

/**
 * Simplify a triangle mesh using CGAL Surface Mesh Simplification (edge collapse).
 *
 * @param mesh   Input triangle mesh as vector of {triangle, normal}.
 * @param ratio  Target ratio of remaining edges (0.0 ~ 1.0). E.g. 0.1 keeps 10%.
 * @return       Simplified mesh. Empty on failure.
 */
std::vector<std::tuple<Triangle, vec3f>>
simplifyMesh(const std::vector<std::tuple<Triangle, vec3f>>& mesh, double ratio);

} // namespace sinriv::kigstudio::cgal
