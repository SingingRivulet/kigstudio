#pragma once
#include <iostream>
#include <string>

#include "kigstudio/ui/render_mesh.h"
#include "kigstudio/voxel/voxelizer_svo.h"

namespace sinriv::ui::render {
    class RenderVoxel {
       public:
        explicit RenderVoxel(bgfx::ViewId view_id = 0,
                             std::string shader_dir = "../../shader/base/")
            : mesh_renderer_(view_id, std::move(shader_dir)) {}

        inline void setViewId(bgfx::ViewId view_id) {
            mesh_renderer_.setViewId(view_id);
        }

        inline bgfx::ViewId getViewId() const {
            return mesh_renderer_.getViewId();
        }

        inline void setShaderDirectory(const std::string& shader_dir) {
            mesh_renderer_.setShaderDirectory(shader_dir);
        }

        inline void release() { mesh_renderer_.release(); }

        inline void clear() { mesh_renderer_.clear(); }

        inline bool empty() const { return mesh_renderer_.empty(); }

        inline void loadVoxelGrid(
            sinriv::kigstudio::voxel::VoxelGrid voxel_data,
            double isolevel = 0.5,
            bool smooth_normals = true) {
            int num_triangles = 0;
            auto geometry = sinriv::kigstudio::voxel::generateMesh(
                voxel_data, isolevel, num_triangles, smooth_normals);
            mesh_renderer_.loadGeometry(geometry);
        }

        inline void loadSTLAsVoxel(const std::string& filename,
                                   float voxel_size = 0.5f,
                                   double isolevel = 0.5,
                                   bool smooth_normals = true) {
            sinriv::kigstudio::voxel::triangle_bvh<float> bvh;
            for (auto [tri, n] : sinriv::kigstudio::voxel::readSTL(filename)) {
                (void)n;
                bvh.insert(tri);
            }

            sinriv::kigstudio::voxel::VoxelGrid voxel_data;
            sinriv::kigstudio::voxel::create_solid_mesh(
                voxel_data, bvh, voxel_size, voxel_size, voxel_size);
            std::cout << "create_solid_mesh success num_chunk="
                      << voxel_data.num_chunk() << std::endl;
            loadVoxelGrid(voxel_data, isolevel, smooth_normals);
        }

        inline void render(const float* transform) {
            mesh_renderer_.render(transform);
        }

       private:
        RenderMesh mesh_renderer_;
    };
}
