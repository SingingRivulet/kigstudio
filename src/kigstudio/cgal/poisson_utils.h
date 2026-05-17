#pragma once
#include <vector>
#include <utility>
#include "kigstudio/voxel/voxel2mesh.h"

namespace sinriv::kigstudio::cgal {

using vec3f = sinriv::kigstudio::vec3<float>;
using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;

/**
 * Sample triangle centroids + face normals from a mesh.
 * Used as input for Poisson surface reconstruction.
 */
std::vector<std::pair<vec3f, vec3f>>
samplePointCloudFromTriangles(const std::vector<Triangle>& triangles);

} // namespace sinriv::kigstudio::cgal
