#define VOXELIZER_IMPLEMENTATION
#include "kigstudio/voxel/voxelizer_svo.h"

namespace sinriv::kigstudio::voxel {

    void draw_triangle(
        sinriv::kigstudio::octree::Octree& voxelData,
        const Triangle& triangle,
        float voxelsizex,      // Voxel size on X-axis
        float voxelsizey,      // Voxel size on Y-axis
        float voxelsizez,      // Voxel size on Z-axis
        float precision) {
        for (auto point : draw_triangle(triangle, voxelsizex, voxelsizey, voxelsizez, precision)) {
            voxelData.insert({ (int)point.x, (int)point.y, (int)point.z });
        }
    }

    Generator<vec3f> draw_triangle(
        Triangle triangle_i,
        float voxelsizex,      // Voxel size on X-axis
        float voxelsizey,      // Voxel size on Y-axis
        float voxelsizez,      // Voxel size on Z-axis
        float precision) {
        vx_vertex_t vs = { {{voxelsizex, voxelsizey, voxelsizez}} };
        vx_vertex_t hvs = vs;
        vx__vec3_multiply(&hvs, 0.5f);

        vx_triangle_t triangle;
        triangle.p1 = vx_vertex_t{ std::get<0>(triangle_i).x,std::get<0>(triangle_i).y,std::get<0>(triangle_i).z };
        triangle.p2 = vx_vertex_t{ std::get<1>(triangle_i).x,std::get<1>(triangle_i).y,std::get<1>(triangle_i).z };
        triangle.p3 = vx_vertex_t{ std::get<2>(triangle_i).x,std::get<2>(triangle_i).y,std::get<2>(triangle_i).z };

        if (vx__triangle_area(&triangle) > VOXELIZER_EPSILON) {

            vx_aabb_t aabb = vx__triangle_aabb(&triangle);

            aabb.min.x = vx__map_to_voxel(aabb.min.x, vs.x, true);
            aabb.min.y = vx__map_to_voxel(aabb.min.y, vs.y, true);
            aabb.min.z = vx__map_to_voxel(aabb.min.z, vs.z, true);

            aabb.max.x = vx__map_to_voxel(aabb.max.x, vs.x, false);
            aabb.max.y = vx__map_to_voxel(aabb.max.y, vs.y, false);
            aabb.max.z = vx__map_to_voxel(aabb.max.z, vs.z, false);

            for (float x = aabb.min.x; x <= aabb.max.x; x += vs.x) {
                for (float y = aabb.min.y; y <= aabb.max.y; y += vs.y) {
                    for (float z = aabb.min.z; z <= aabb.max.z; z += vs.z) {
                        vx_aabb_t saabb;

                        saabb.min.x = x - hvs.x;
                        saabb.min.y = y - hvs.y;
                        saabb.min.z = z - hvs.z;
                        saabb.max.x = x + hvs.x;
                        saabb.max.y = y + hvs.y;
                        saabb.max.z = z + hvs.z;

                        vx_vertex_t boxcenter = vx__aabb_center(&saabb);
                        vx_vertex_t halfsize = vx__aabb_half_size(&saabb);

                        // HACK: some holes might appear, this
                        // precision factor reduces the artifact
                        halfsize.x += precision;
                        halfsize.y += precision;
                        halfsize.z += precision;

                        if (vx__triangle_box_overlap(boxcenter, halfsize, triangle)) {
                            co_yield vec3f{ x, y, z };
                        }
                    }
                }
            }
        }
    }
}
