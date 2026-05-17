#pragma once
#include <vector>
#include <tuple>
#include <utility>
#include "kigstudio/voxel/voxel2mesh.h"

namespace sinriv::kigstudio::cgal {

using vec3f = sinriv::kigstudio::vec3<float>;
using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;

/**
 * Perform Poisson surface reconstruction from an oriented point cloud.
 *
 * @param points_with_normals  Point cloud where each element is {position, normal}
 * @param spacing              Target spacing (typically voxel size)
 * @return                     Reconstructed triangle mesh. Empty on failure.
 */
std::vector<std::tuple<Triangle, vec3f>>
poissonReconstruct(const std::vector<std::pair<vec3f, vec3f>>& points_with_normals,
                   double spacing);

} // namespace sinriv::kigstudio::cgal
