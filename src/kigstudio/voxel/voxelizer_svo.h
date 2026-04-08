#pragma once
#include "kigstudio/voxel/mesh_voxelizer.h"
#include "kigstudio/voxel/octree.h"
#include "kigstudio/voxel/voxel2mesh.h"
#include "kigstudio/utils/mat.h"
#include "kigstudio/utils/dbvt3d.h"

#include <mutex>

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
    
    inline void create_surface_mesh(
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
    
    inline void create_solid_mesh(
        sinriv::kigstudio::octree::Octree& voxelData,
        sinriv::kigstudio::voxel::triangle_bvh<float>& bvh,
        float voxelsizex,      // Voxel size on X-axis
        float voxelsizey,      // Voxel size on Y-axis
        float voxelsizez,      // Voxel size on Z-axis
        voxel::triangle_bvh<float>::voxel_face_e face = triangle_bvh<float>::voxel_face_e::voxel_face_X
    ){
        voxelData.voxel_size.x = voxelsizex;
        voxelData.voxel_size.y = voxelsizey;
        voxelData.voxel_size.z = voxelsizez;
        voxelData.global_position.x = bvh.global_boundBox_min.x;
        voxelData.global_position.y = bvh.global_boundBox_min.y;
        voxelData.global_position.z = bvh.global_boundBox_min.z;

        std::mutex locker;
        bvh.getSolidByFace(
            voxelsizex, voxelsizey, voxelsizez, face,
            [&](auto start, auto end) {
                // std::cout << start << "->" << end << std::endl;
                int start_x = static_cast<int>(std::round((start.x-voxelData.global_position.x) / voxelsizex));
                int start_y = static_cast<int>(std::round((start.y-voxelData.global_position.y) / voxelsizey));
                int start_z = static_cast<int>(std::round((start.z-voxelData.global_position.z) / voxelsizez));
                int end_x = static_cast<int>(std::round((end.x-voxelData.global_position.x) / voxelsizex));
                int end_y = static_cast<int>(std::round((end.y-voxelData.global_position.y) / voxelsizey));
                int end_z = static_cast<int>(std::round((end.z-voxelData.global_position.z) / voxelsizez));

                locker.lock();
                for (int i=start_x; i<=end_x; i++){
                    for (int j=start_y; j<=end_y; j++){
                        for (int k=start_z; k<=end_z; k++){
                            if (i >= 0 && j >= 0 && k >= 0){
                                voxelData.insert({i, j, k});
                            }
                        }
                    }
                }
                locker.unlock();
            }
        );
    }

    inline void create_solid_mesh(
        sinriv::kigstudio::octree::Octree& voxelData,
        const std::string & path, 
        mat::matrix<float> transform, 
        float voxelsizex,      // Voxel size on X-axis
        float voxelsizey,      // Voxel size on Y-axis
        float voxelsizez,      // Voxel size on Z-axis
        voxel::triangle_bvh<float>::voxel_face_e face = triangle_bvh<float>::voxel_face_e::voxel_face_X
    ){
        sinriv::kigstudio::voxel::triangle_bvh<float> bvh;
        for (auto [vertex, normal] : sinriv::kigstudio::voxel::readSTL(path)) {
            auto & [v1, v2, v3] = vertex;
            mat::vec4<float> tv1(v1.x, v1.y, v1.z, 1) , tv2(v2.x, v2.y, v2.z, 1), tv3(v3.x, v3.y, v3.z, 1);
            vec3f ov1 = (tv1 * transform).toVec3();
            vec3f ov2 = (tv2 * transform).toVec3();
            vec3f ov3 = (tv3 * transform).toVec3();
            triangle_bvh<float>::triangle tri(ov1, ov2, ov3);
            bvh.insert(tri);
        }
        create_solid_mesh(voxelData, bvh, voxelsizex, voxelsizey, voxelsizez, face);
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
