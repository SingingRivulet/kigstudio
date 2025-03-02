#pragma once
#include "kigstudio/voxel/voxelizer.h"
#include "kigstudio/voxel/octree.h"
#include "kigstudio/voxel/voxel2mesh.h"
#include "kigstudio/utils/mat.h"

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
    
    inline void create_solid_mesh(
        sinriv::kigstudio::octree::Octree& voxelData,
        const std::string & path, 
        mat::matrix<float> transform, 
        float voxelsizex,      // Voxel size on X-axis
        float voxelsizey,      // Voxel size on Y-axis
        float voxelsizez,      // Voxel size on Z-axis
        float precision,
        float solidization = 0){
        for (auto [vertex, normal] : sinriv::kigstudio::voxel::readSTL(path)) {
            auto & [v1, v2, v3] = vertex;
            mat::vec4<float> tv1(v1.x, v1.y, v1.z, 1) , tv2(v2.x, v2.y, v2.z, 1), tv3(v3.x, v3.y, v3.z, 1);
            vec3f ov1 = (tv1 * transform).toVec3();
            vec3f ov2 = (tv2 * transform).toVec3();
            vec3f ov3 = (tv3 * transform).toVec3();

            draw_triangle(voxelData, Triangle(ov1, ov2, ov3), normal, voxelsizex, voxelsizey, voxelsizez, precision, solidization);
        }
    }
    
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
