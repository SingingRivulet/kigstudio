#pragma once
#include <vector>
#include <utility>
#include "kigstudio/voxel/voxel2mesh.h"

namespace sinriv::kigstudio::cgal {

using vec3f = sinriv::kigstudio::vec3<float>;
using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;

/**
 * Extract a medial curve skeleton from a closed triangle mesh using
 * CGAL's Mean Curvature Flow Skeletonization.
 *
 * @param triangles   Input triangles (must form a closed, water-tight mesh)
 * @return            List of skeleton line segments. Empty if the mesh is
 *                    not closed or extraction fails.
 */
std::vector<std::pair<vec3f, vec3f>>
extractSkeletonFromMesh(const std::vector<Triangle>& triangles);

} // namespace sinriv::kigstudio::cgal
