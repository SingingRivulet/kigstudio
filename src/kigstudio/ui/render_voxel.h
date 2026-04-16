#pragma once
#include <iostream>
#include <string>

#include "kigstudio/ui/render_mesh.h"
#include "kigstudio/voxel/voxelizer_svo.h"

namespace sinriv::ui::render {
    class RenderVoxel {
       public:
        using AxisHandle = RenderMesh::AxisHandle;
        using vec3f = RenderMesh::vec3f;
        explicit RenderVoxel(bgfx::ViewId view_id = 0,
                             bgfx::ViewId overlay_view_id = 0,
                             std::string shader_dir = "../../shader/base/")
            : mesh_renderer_(view_id, overlay_view_id, std::move(shader_dir)) {
            mesh_renderer_.setBaseColor(0.72f, 0.80f, 0.95f, 1.0f);
        }

        inline void setViewId(bgfx::ViewId view_id) {
            mesh_renderer_.setViewId(view_id);
        }

        inline bgfx::ViewId getViewId() const {
            return mesh_renderer_.getViewId();
        }

        inline void setOverlayViewId(bgfx::ViewId view_id) {
            mesh_renderer_.setOverlayViewId(view_id);
        }

        inline bgfx::ViewId getOverlayViewId() const {
            return mesh_renderer_.getOverlayViewId();
        }

        inline void setShaderDirectory(const std::string& shader_dir) {
            mesh_renderer_.setShaderDirectory(shader_dir);
        }

        inline void setViewportSize(int width, int height) {
            mesh_renderer_.setViewportSize(width, height);
        }

        inline void setViewProjection(const float* view, const float* proj) {
            mesh_renderer_.setViewProjection(view, proj);
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

        inline void renderGBuffer(const float* transform) {
            mesh_renderer_.showAxis = showAxis;
            mesh_renderer_.renderGBuffer(transform);
        }

        inline void renderOverlay() {
            mesh_renderer_.showAxis = showAxis;
            mesh_renderer_.renderOverlay();
        }

        inline void render(const float* transform) {
            renderGBuffer(transform);
            renderOverlay();
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

        bool showAxis = false;

       private:
        RenderMesh mesh_renderer_;
    };
}
