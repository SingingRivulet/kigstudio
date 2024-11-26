#pragma once
#include "kigstudio/voxel/voxelizer.h"
#include "kigstudio/voxel/octree.h"
#include "kigstudio/voxel/voxel2mesh.h"

namespace sinriv::kigstudio::voxel {

    using vec3i = sinriv::kigstudio::octree::Vec3i;

    void draw_triangle(
        sinriv::kigstudio::octree::Octree& voxelData,
        const Triangle& triangle,
        const vec3f& normal,
        float voxelsizex,      // Voxel size on X-axis
        float voxelsizey,      // Voxel size on Y-axis
        float voxelsizez,      // Voxel size on Z-axis
        float precision,
        float solidization = 0);
    
    void draw_line(
        sinriv::kigstudio::octree::Octree& voxelData,
        const vec3f& start,
        const vec3f& end
    );

    Generator<vec3f> draw_triangle(
        Triangle triangle,
        float voxelsizex,      // Voxel size on X-axis
        float voxelsizey,      // Voxel size on Y-axis
        float voxelsizez,      // Voxel size on Z-axis
        float precision);
    
    Generator<vec3i> draw_line(
        vec3f start,
        vec3f end
    );
}
