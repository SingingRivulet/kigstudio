#pragma once
#include "kigstudio/voxel/voxelizer.h"
#include "kigstudio/voxel/octree.h"
#include "kigstudio/voxel/voxel2mesh.h"

namespace sinriv::kigstudio::voxel {

    void draw_triangle(
        sinriv::kigstudio::octree::Octree& voxelData,
        const Triangle& triangle,
        float voxelsizex,      // Voxel size on X-axis
        float voxelsizey,      // Voxel size on Y-axis
        float voxelsizez,      // Voxel size on Z-axis
        float precision);

    Generator<vec3f> draw_triangle(
        Triangle triangle,
        float voxelsizex,      // Voxel size on X-axis
        float voxelsizey,      // Voxel size on Y-axis
        float voxelsizez,      // Voxel size on Z-axis
        float precision);
}
