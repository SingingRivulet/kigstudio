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
        using VoxelGrid = sinriv::kigstudio::voxel::VoxelGrid;
        using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;
        inline explicit RenderVoxel(){
            mesh_renderer_.setBaseColor(0.72f, 0.80f, 0.95f, 1.0f);
        }


        inline void setViewportSize(int width, int height) {
            mesh_renderer_.setViewportSize(width, height);
            for (auto& [key, mesh] : chunk_meshes_) {
                mesh.setViewportSize(width, height);
            }
        }

        inline void setViewProjection(const float* view, const float* proj) {
            mesh_renderer_.setViewProjection(view, proj);
            for (auto& [key, mesh] : chunk_meshes_) {
                mesh.setViewProjection(view, proj);
            }
        }

        inline void setModelMatrix(const RenderMesh::mat4f& model_matrix) {
            mesh_renderer_.setModelMatrix(model_matrix);
            for (auto& [key, mesh] : chunk_meshes_) {
                mesh.setModelMatrix(model_matrix);
            }
        }

        inline void setAxisLength(float value) {
            mesh_renderer_.setAxisLength(value);
            for (auto& [key, mesh] : chunk_meshes_) {
                mesh.setAxisLength(value);
            }
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
            for (auto& [key, mesh] : chunk_meshes_) {
                mesh.release();
            }
            collision_bvh.reset();
        }

        inline void clear() {
            mesh_renderer_.clear();
            chunk_meshes_.clear();
        }

        inline bool empty() const {
            if (!chunk_meshes_.empty()) {
                for (const auto& [key, mesh] : chunk_meshes_) {
                    if (!mesh.empty()) return false;
                }
                return true;
            }
            return mesh_renderer_.empty();
        }

        inline RenderMesh& getMeshRenderer() { return mesh_renderer_; }
        inline const RenderMesh& getMeshRenderer() const { return mesh_renderer_; }

        inline std::pair<vec3f, vec3f> getLocalBounds() const {
            vec3f min_b = {std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max()};
            vec3f max_b = {std::numeric_limits<float>::lowest(),
                           std::numeric_limits<float>::lowest(),
                           std::numeric_limits<float>::lowest()};
            bool has_bounds = false;

            auto [mmin, mmax] = mesh_renderer_.getLocalBounds();
            if (mmin.x != 0.0f || mmin.y != 0.0f || mmin.z != 0.0f ||
                mmax.x != 0.0f || mmax.y != 0.0f || mmax.z != 0.0f) {
                min_b = mmin;
                max_b = mmax;
                has_bounds = true;
            }

            for (const auto& [key, mesh] : chunk_meshes_) {
                auto [cmin, cmax] = mesh.getLocalBounds();
                if (cmin.x == 0.0f && cmin.y == 0.0f && cmin.z == 0.0f &&
                    cmax.x == 0.0f && cmax.y == 0.0f && cmax.z == 0.0f) {
                    continue;
                }
                if (!has_bounds) {
                    min_b = cmin;
                    max_b = cmax;
                    has_bounds = true;
                } else {
                    min_b.x = std::min(min_b.x, cmin.x);
                    min_b.y = std::min(min_b.y, cmin.y);
                    min_b.z = std::min(min_b.z, cmin.z);
                    max_b.x = std::max(max_b.x, cmax.x);
                    max_b.y = std::max(max_b.y, cmax.y);
                    max_b.z = std::max(max_b.z, cmax.z);
                }
            }

            if (!has_bounds) {
                return {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
            }
            return {min_b, max_b};
        }

        inline void loadVoxelGrid(
            VoxelGrid voxel_data,
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

        // ============ Chunked mesh ============
        inline void loadVoxelGridChunked(
            VoxelGrid& voxel_data,
            double isolevel = 0.5,
            bool smooth_normals = true,
            float expand = 0.0f) {
            chunk_meshes_.clear();
            for (const auto& [key, chunk] : voxel_data.chunks) {
                (void)chunk;
                int num_triangles = 0;
                auto generator = sinriv::kigstudio::voxel::generateMeshForChunk(
                    voxel_data, key, isolevel, num_triangles, smooth_normals, expand);
                std::vector<std::tuple<Triangle, sinriv::kigstudio::voxel::vec3f>> mesh;
                for (auto [tri, n] : generator) {
                    mesh.push_back({tri, n});
                }
                if (!mesh.empty()) {
                    chunk_meshes_[key].loadGeometry(mesh);
                }
            }
        }

        inline void loadChunkMeshes(
            const std::unordered_map<
                uint64_t,
                std::vector<std::tuple<Triangle, sinriv::kigstudio::voxel::vec3f>>>&
                meshes) {
            chunk_meshes_.clear();
            for (const auto& [key, mesh] : meshes) {
                if (!mesh.empty()) {
                    chunk_meshes_[key].loadGeometry(mesh);
                }
            }
        }

        inline void updateChunk(
            VoxelGrid& voxel_data,
            uint64_t chunk_key,
            double isolevel = 0.5,
            bool smooth_normals = true,
            float expand = 0.0f) {
            int num_triangles = 0;
            auto generator = sinriv::kigstudio::voxel::generateMeshForChunk(
                voxel_data, chunk_key, isolevel, num_triangles, smooth_normals, expand);
            std::vector<std::tuple<Triangle, sinriv::kigstudio::voxel::vec3f>> mesh;
            for (auto [tri, n] : generator) {
                mesh.push_back({tri, n});
            }
            auto it = chunk_meshes_.find(chunk_key);
            if (mesh.empty()) {
                if (it != chunk_meshes_.end()) {
                    chunk_meshes_.erase(it);
                }
            } else {
                chunk_meshes_[chunk_key].loadGeometry(mesh);
            }
        }

        inline void renderGBuffer(const float* transform, RenderMeshShader & shader) {
            if (!chunk_meshes_.empty()) {
                for (auto& [key, mesh] : chunk_meshes_) {
                    (void)key;
                    mesh.showAxis = showAxis;
                    mesh.renderGBuffer(transform, shader);
                }
            } else {
                mesh_renderer_.showAxis = showAxis;
                mesh_renderer_.renderGBuffer(transform, shader);
            }
        }

        inline void renderOverlay(RenderMeshShader & shader) {
            if (!chunk_meshes_.empty()) {
                for (auto& [key, mesh] : chunk_meshes_) {
                    (void)key;
                    mesh.showAxis = showAxis;
                    mesh.renderOverlay(shader);
                }
            } else {
                mesh_renderer_.showAxis = showAxis;
                mesh_renderer_.renderOverlay(shader);
            }
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
        std::unordered_map<uint64_t, RenderMesh> chunk_meshes_;
    };
}
