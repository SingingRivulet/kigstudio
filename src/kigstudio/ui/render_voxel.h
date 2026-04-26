#pragma once
#include <iostream>
#include <memory>
#include <string>

#include "kigstudio/ui/render_mesh.h"
#include "kigstudio/voxel/voxelizer_svo.h"

namespace sinriv::ui::render {
    class RenderVoxel {
       public:
        using AxisHandle = RenderMesh::AxisHandle;
        using vec3f = RenderMesh::vec3f;
        inline explicit RenderVoxel(){
            mesh_renderer_.setBaseColor(0.72f, 0.80f, 0.95f, 1.0f);
        }


        inline void setViewportSize(int width, int height) {
            mesh_renderer_.setViewportSize(width, height);
        }

        inline void setViewProjection(const float* view, const float* proj) {
            mesh_renderer_.setViewProjection(view, proj);
        }

        inline void setModelMatrix(const RenderMesh::mat4f& model_matrix) {
            mesh_renderer_.setModelMatrix(model_matrix);
        }

        inline void setAxisLength(float value) {
            mesh_renderer_.setAxisLength(value);
        }

        inline AxisHandle getHoveredAxis() const { return mesh_renderer_.getHoveredAxis(); }
        inline AxisHandle getActiveAxis() const { return mesh_renderer_.getActiveAxis(); }

        inline float getAxisScreenDelta(int from_x, int from_y, int to_x, int to_y) const {
            return mesh_renderer_.getAxisScreenDelta(from_x, from_y, to_x, to_y);
        }

        inline vec3f getAxisWorldDelta(int from_x, int from_y, int to_x, int to_y) const {
            return mesh_renderer_.getAxisWorldDelta(from_x, from_y, to_x, to_y);
        }

        inline void release() { 
            mesh_renderer_.release();
            collision_bvh.reset();
        }

        inline void clear() { mesh_renderer_.clear(); }

        inline bool empty() const { return mesh_renderer_.empty(); }

        inline RenderMesh& getMeshRenderer() { return mesh_renderer_; }
        inline const RenderMesh& getMeshRenderer() const { return mesh_renderer_; }

        inline void loadVoxelGrid(
            sinriv::kigstudio::voxel::VoxelGrid voxel_data,
            double isolevel = 0.5,
            bool smooth_normals = true) {
            int num_triangles = 0;
            auto geometry = sinriv::kigstudio::voxel::generateMesh(
                voxel_data, isolevel, num_triangles, smooth_normals);
            mesh_renderer_.loadGeometry(geometry);
        }

        inline void loadGeometry(const AsyncVoxelLoader::MeshData& data) {
            collision_bvh = std::make_unique<sinriv::kigstudio::voxel::triangle_bvh<float>>();
            collision_bvh->loadGeometry(data.vertices, data.indices);
            mesh_renderer_.loadGeometry(data.vertices, data.indices);
        }

        inline void renderGBuffer(const float* transform, RenderMeshShader & shader) {
            mesh_renderer_.showAxis = showAxis;
            mesh_renderer_.renderGBuffer(transform, shader);
        }

        inline void renderOverlay(RenderMeshShader & shader) {
            mesh_renderer_.showAxis = showAxis;
            mesh_renderer_.renderOverlay(shader);
        }

        inline void render(const float* transform, RenderMeshShader & shader) {
            renderGBuffer(transform, shader);
            renderOverlay(shader);
        }
        
        //计算物体在屏幕上覆盖的矩形区域（x1,y1,x2,y2），用于提前过滤鼠标事件
        inline std::tuple<int, int, int, int> getScreenBoundBox() {
            return mesh_renderer_.getScreenBoundBox();
        }
        //为false时，事件会继续传递给下一个渲染器
        inline bool onMouseMove(int x, int y) {
            mesh_renderer_.showAxis = showAxis;
            return mesh_renderer_.onMouseMove(x, y);
        }
        inline bool onMousePress(int x, int y) {
            mesh_renderer_.showAxis = showAxis;
            return mesh_renderer_.onMousePress(x, y);
        }
        inline bool onMouseRelease(int x, int y) {
            mesh_renderer_.showAxis = showAxis;
            return mesh_renderer_.onMouseRelease(x, y);
        }

        std::unique_ptr<sinriv::kigstudio::voxel::triangle_bvh<float>> collision_bvh;

        bool showAxis = false;

       private:
        RenderMesh mesh_renderer_;
    };
}
