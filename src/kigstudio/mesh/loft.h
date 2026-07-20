#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "kigstudio/utils/vec2.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/voxel/voxel2mesh.h"

namespace sinriv::kigstudio::mesh::loft {

using vec2f = sinriv::kigstudio::vec2<float>;
using vec3f = sinriv::kigstudio::vec3<float>;
using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;

struct LoftSection {
	// Origin is guide_curve[guide_vertex_id].
	size_t guide_vertex_id = 0;

	// Local 2D axes in world space. They are normalized before use; path
	// coordinates keep their own length units.
	vec3f axis_u = {1.0f, 0.0f, 0.0f};
	vec3f axis_v = {0.0f, 1.0f, 0.0f};

	// Closed section path. All sections must have the same vertex count and
	// matching point order.
	std::vector<vec2f> path;
};

struct LoftOptions {
	bool cap_first = true;
	bool cap_last = true;
	bool orient_faces = true;
};

std::vector<Triangle> build_loft_mesh(
    const std::vector<vec3f>& guide_curve,
    const std::vector<LoftSection>& sections,
    const LoftOptions& options = {},
    const std::function<bool()>& should_continue = {});

}  // namespace sinriv::kigstudio::mesh::loft
