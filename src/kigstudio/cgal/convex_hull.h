#pragma once
#include <vector>
#include "kigstudio/voxel/voxel2mesh.h"

namespace sinriv::kigstudio::cgal {

using vec3f = sinriv::kigstudio::vec3<float>;
using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;

/**
 * Compute the convex hull of a triangle mesh using CGAL::convex_hull_3.
 *
 * @param triangles Input triangle mesh.
 * @return          Convex hull as a triangle mesh. Empty on failure or degenerate input.
 */
std::vector<Triangle> convexHull3(const std::vector<Triangle>& triangles);

}  // namespace sinriv::kigstudio::cgal
