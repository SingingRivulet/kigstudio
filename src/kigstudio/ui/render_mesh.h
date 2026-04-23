#pragma once
#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <bx/math.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "kigstudio/ui/render_axis_gizmo.h"
#include "kigstudio/voxel/voxel2mesh.h"

namespace sinriv::ui::render {
    namespace mesh_detail {
        struct MeshHandle {
            bgfx::VertexBufferHandle vbh = BGFX_INVALID_HANDLE;
            bgfx::IndexBufferHandle ibh = BGFX_INVALID_HANDLE;
            uint32_t index_count = 0;

            inline void destroy() {
                if (bgfx::isValid(vbh)) {
                    bgfx::destroy(vbh);
                    vbh = BGFX_INVALID_HANDLE;
                }
                if (bgfx::isValid(ibh)) {
                    bgfx::destroy(ibh);
                    ibh = BGFX_INVALID_HANDLE;
                }
                index_count = 0;
            }
        };

        struct PosNormalVertex {
            float x, y, z;
            float nx, ny, nz;

            static inline void init(bgfx::VertexLayout& layout) {
                layout.begin()
                    .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
                    .add(bgfx::Attrib::Normal, 3, bgfx::AttribType::Float)
                    .end();
            }
        };

        struct ColorLineVertex {
            float x, y, z;
            uint32_t abgr;

            static inline void init(bgfx::VertexLayout& layout) {
                layout.begin()
                    .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
                    .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
                    .end();
            }
        };

        struct AsyncVoxelMeshData {
            std::vector<PosNormalVertex> vertices;
            std::vector<uint32_t> indices;
        };

        inline bgfx::ShaderHandle loadShader(const std::string& path) {
            std::cout << "load shader:" << path << std::endl;
            FILE* file = std::fopen(path.c_str(), "rb");
            if (!file) {
                return BGFX_INVALID_HANDLE;
            }

            std::fseek(file, 0, SEEK_END);
            const long size = std::ftell(file);
            std::fseek(file, 0, SEEK_SET);
            if (size <= 0) {
                std::fclose(file);
                return BGFX_INVALID_HANDLE;
            }

            std::vector<char> data(static_cast<std::size_t>(size));
            std::fread(data.data(), 1, data.size(), file);
            std::fclose(file);
            return bgfx::createShader(
                bgfx::copy(data.data(), static_cast<uint32_t>(data.size())));
        }
    }

    class RenderMeshShader{
      public:
        inline explicit RenderMeshShader(bgfx::ViewId view_id = 0,
                            bgfx::ViewId overlay_view_id = 0,
                            std::string shader_dir = "../../shader/base/")
            : view_id_(view_id),
              overlay_view_id_(overlay_view_id),
              shader_dir_(std::move(shader_dir)) {
            bx::mtxIdentity(identity_mtx_);
        }

        inline ~RenderMeshShader() {
            release();
        }
        inline void setViewId(bgfx::ViewId view_id) { view_id_ = view_id; }
        inline bgfx::ViewId getViewId() const { return view_id_; }
        inline void setOverlayViewId(bgfx::ViewId view_id) { overlay_view_id_ = view_id; }
        inline bgfx::ViewId getOverlayViewId() const { return overlay_view_id_; }

        inline void setShaderDirectory(const std::string& shader_dir) {
            shader_dir_ = shader_dir;
            destroyPrograms();
        }
        
        inline void destroyPrograms() {
            if (bgfx::isValid(gbuffer_program_)) {
                bgfx::destroy(gbuffer_program_);
                gbuffer_program_ = BGFX_INVALID_HANDLE;
                std::cout << "RenderMeshShader shader(gbuffer_program_) destroyed" << std::endl;
            }
            if (bgfx::isValid(line_program_)) {
                bgfx::destroy(line_program_);
                line_program_ = BGFX_INVALID_HANDLE;
                std::cout << "RenderMeshShader shader(line_program_) destroyed" << std::endl;
            }
        }

        inline void destroyUniforms() {
            if (bgfx::isValid(u_base_color_)) {
                bgfx::destroy(u_base_color_);
                u_base_color_ = BGFX_INVALID_HANDLE;
                std::cout << "RenderMeshShader uniform destroyed" << std::endl;
            }
        }

        inline void ensureUniforms() {
            if (!bgfx::isValid(u_base_color_)) {
                u_base_color_ = bgfx::createUniform("u_baseColor", bgfx::UniformType::Vec4);
            }
        }

        inline bool ensureGBufferProgram() {
            if (bgfx::isValid(gbuffer_program_)) {
                return true;
            }
            ensureUniforms();

            bgfx::ShaderHandle vs =
                mesh_detail::loadShader(shader_dir_ + "vs_mesh_gbuffer.bin");
            bgfx::ShaderHandle fs =
                mesh_detail::loadShader(shader_dir_ + "fs_mesh_gbuffer.bin");
            if (!bgfx::isValid(vs) || !bgfx::isValid(fs)) {
                if (bgfx::isValid(vs)) {
                    bgfx::destroy(vs);
                }
                if (bgfx::isValid(fs)) {
                    bgfx::destroy(fs);
                }
                std::cerr << "RenderMesh shader load failed from " << shader_dir_
                          << std::endl;
                return false;
            }

            gbuffer_program_ = bgfx::createProgram(vs, fs, true);
            return bgfx::isValid(gbuffer_program_);
        }

        inline bool ensureLineProgram() {
            if (bgfx::isValid(line_program_)) {
                return true;
            }
            bx::mtxIdentity(identity_mtx_);

            bgfx::ShaderHandle vs =
                mesh_detail::loadShader(shader_dir_ + "vs_color_line.bin");
            bgfx::ShaderHandle fs =
                mesh_detail::loadShader(shader_dir_ + "fs_color_line.bin");
            if (!bgfx::isValid(vs) || !bgfx::isValid(fs)) {
                if (bgfx::isValid(vs)) {
                    bgfx::destroy(vs);
                }
                if (bgfx::isValid(fs)) {
                    bgfx::destroy(fs);
                }
                std::cerr << "RenderMesh line shader load failed from " << shader_dir_
                          << std::endl;
                return false;
            }

            line_program_ = bgfx::createProgram(vs, fs, true);
            return bgfx::isValid(line_program_);
        }
        inline void release() {
            destroyPrograms();
            destroyUniforms();
        }

        bgfx::ViewId view_id_ = 0;
        bgfx::ViewId overlay_view_id_ = 0;
        std::string shader_dir_ = "../../shader/base/";
        bgfx::ProgramHandle gbuffer_program_ = BGFX_INVALID_HANDLE;
        bgfx::ProgramHandle line_program_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_base_color_ = BGFX_INVALID_HANDLE;
        float identity_mtx_[16]{};
    };

    class RenderMesh {
       public:
        using AxisHandle = axis_gizmo::AxisHandle;
        using vec3f = sinriv::kigstudio::vec3<float>;
        using mat4f = sinriv::kigstudio::mat::matrix<float>;

        inline explicit RenderMesh() {
            bx::mtxIdentity(identity_mtx_);
        }

        inline ~RenderMesh() {
            release();
        }


        inline void setBaseColor(float r, float g, float b, float a = 1.0f) {
            base_color_ = {r, g, b, a};
        }

        inline void setViewportSize(int width, int height) {
            axis_state_.viewport_width = std::max(width, 1);
            axis_state_.viewport_height = std::max(height, 1);
        }

        inline void setViewProjection(const float* view, const float* proj) {
            axis_state_.view_matrix = mat4f(view);
            axis_state_.proj_matrix = mat4f(proj);
        }

        inline void setModelMatrix(const mat4f& model_matrix) {
            axis_state_.model_matrix = model_matrix;
        }

        inline void setAxisLength(float value) {
            axis_state_.axis_length = std::max(value, 1.0f);
        }

        inline AxisHandle getHoveredAxis() const { return axis_state_.hovered_axis; }
        inline AxisHandle getActiveAxis() const { return axis_state_.active_axis; }

        inline float getAxisScreenDelta(int from_x, int from_y, int to_x, int to_y) const {
            return axis_gizmo::getAxisScreenDelta(axis_state_, axis_state_.active_axis,
                                                  from_x, from_y, to_x, to_y);
        }

        inline vec3f getAxisWorldDelta(int from_x, int from_y, int to_x, int to_y) const {
            return axis_gizmo::getAxisWorldDelta(axis_state_, axis_state_.active_axis,
                                                 from_x, from_y, to_x, to_y);
        }

        template <class T>
        void loadGeometry(T&& geometry) {
            if (!layout_initialized_) {
                mesh_detail::PosNormalVertex::init(layout_);
                layout_initialized_ = true;
            }

            resetBounds();
            std::vector<mesh_detail::PosNormalVertex> vertices;
            std::vector<uint32_t> indices;

            for (auto [tri, n] : geometry) {
                const uint32_t base = static_cast<uint32_t>(vertices.size());
                updateLocalBounds(
                    {std::get<0>(tri).x, std::get<0>(tri).y, std::get<0>(tri).z});
                updateLocalBounds(
                    {std::get<1>(tri).x, std::get<1>(tri).y, std::get<1>(tri).z});
                updateLocalBounds(
                    {std::get<2>(tri).x, std::get<2>(tri).y, std::get<2>(tri).z});
                vertices.push_back(
                    {std::get<0>(tri).x, std::get<0>(tri).y, std::get<0>(tri).z, n.x, n.y, n.z});
                vertices.push_back(
                    {std::get<1>(tri).x, std::get<1>(tri).y, std::get<1>(tri).z, n.x, n.y, n.z});
                vertices.push_back(
                    {std::get<2>(tri).x, std::get<2>(tri).y, std::get<2>(tri).z, n.x, n.y, n.z});

                indices.push_back(base);
                indices.push_back(base + 1);
                indices.push_back(base + 2);
            }

            mesh_.destroy();
            if (vertices.empty()) {
                return;
            }

            mesh_.vbh = bgfx::createVertexBuffer(
                bgfx::copy(vertices.data(),
                           static_cast<uint32_t>(vertices.size() * sizeof(mesh_detail::PosNormalVertex))),
                layout_);

            mesh_.ibh = bgfx::createIndexBuffer(
                bgfx::copy(indices.data(),
                           static_cast<uint32_t>(indices.size() * sizeof(uint32_t))),
                BGFX_BUFFER_INDEX32);

            mesh_.index_count = static_cast<uint32_t>(indices.size());
            axis_state_.axis_length =
                axis_gizmo::estimateAxisLengthFromBounds(local_bound_min_, local_bound_max_);
            std::cout << "mesh loaded: " << mesh_.index_count << " indices" << std::endl;
        }

        inline void loadSTL(const std::string& filename) {
            loadGeometry(sinriv::kigstudio::voxel::readSTL(filename));
        }

        inline void loadGeometry(const std::vector<mesh_detail::PosNormalVertex>& vertices,
                                 const std::vector<uint32_t>& indices) {
            if (!layout_initialized_) {
                mesh_detail::PosNormalVertex::init(layout_);
                layout_initialized_ = true;
            }

            resetBounds();
            mesh_.destroy();
            if (vertices.empty() || indices.empty()) {
                return;
            }

            for (const auto& v : vertices) {
                updateLocalBounds({v.x, v.y, v.z});
            }

            mesh_.vbh = bgfx::createVertexBuffer(
                bgfx::copy(vertices.data(),
                           static_cast<uint32_t>(vertices.size() * sizeof(mesh_detail::PosNormalVertex))),
                layout_);

            mesh_.ibh = bgfx::createIndexBuffer(
                bgfx::copy(indices.data(),
                           static_cast<uint32_t>(indices.size() * sizeof(uint32_t))),
                BGFX_BUFFER_INDEX32);

            mesh_.index_count = static_cast<uint32_t>(indices.size());
            axis_state_.axis_length =
                axis_gizmo::estimateAxisLengthFromBounds(local_bound_min_, local_bound_max_);
            std::cout << "mesh loaded: " << mesh_.index_count << " indices" << std::endl;
        }

        inline bool empty() const {
            return !bgfx::isValid(mesh_.vbh) || !bgfx::isValid(mesh_.ibh) ||
                   mesh_.index_count == 0;
        }

        inline std::pair<vec3f, vec3f> getLocalBounds() const {
            return {local_bound_min_, local_bound_max_};
        }

        inline void release() {
            mesh_.destroy();
        }

        inline void clear() {
            mesh_.destroy();
            resetBounds();
        }

        void renderGBuffer(const float* transform, RenderMeshShader & shader) {
            if (!layout_initialized_) {
                mesh_detail::PosNormalVertex::init(layout_);
                layout_initialized_ = true;
            }

            if (empty() || !shader.ensureGBufferProgram()) {
                return;
            }

            bgfx::setTransform(transform);
            bgfx::setVertexBuffer(0, mesh_.vbh);
            bgfx::setIndexBuffer(mesh_.ibh);
            shader.ensureUniforms();
            bgfx::setUniform(shader.u_base_color_, base_color_.data());
            bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                           BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS |
                           BGFX_STATE_CULL_CCW | BGFX_STATE_MSAA);
            bgfx::submit(shader.view_id_, shader.gbuffer_program_);
        }

        void renderOverlay(RenderMeshShader & shader) {
            if (!empty() && showAxis) {
                renderAxis(shader);
            }
        }

        void render(const float* transform, RenderMeshShader & shader) {
            renderGBuffer(transform, shader);
            renderOverlay(shader);
        }

        //计算物体在屏幕上覆盖的矩形区域（x1,y1,x2,y2），用于提前过滤鼠标事件
        inline std::tuple<int, int, int, int> getScreenBoundBox() {
            if (!has_local_bounds_) {
                return {0, 0, 0, 0};
            }
            return axis_gizmo::projectBoundsToScreen(getWorldBoundsCorners(), axis_state_);
        }
        //为false时，事件会继续传递给下一个渲染器
        inline bool onMouseMove(int x, int y) {
            axis_state_.hovered_axis =
                showAxis ? axis_gizmo::pickAxis(axis_state_, x, y) : AxisHandle::None;
            return axis_state_.dragging || axis_state_.hovered_axis != AxisHandle::None;
        }
        inline bool onMousePress(int x, int y) {
            if (!showAxis) {
                return false;
            }

            axis_state_.active_axis = axis_gizmo::pickAxis(axis_state_, x, y);
            axis_state_.hovered_axis = axis_state_.active_axis;
            axis_state_.dragging = axis_state_.active_axis != AxisHandle::None;
            return axis_state_.dragging;
        }
        inline bool onMouseRelease(int x, int y) {
            const bool was_dragging = axis_state_.dragging;
            axis_state_.dragging = false;
            axis_state_.active_axis = AxisHandle::None;
            axis_state_.hovered_axis =
                showAxis ? axis_gizmo::pickAxis(axis_state_, x, y) : AxisHandle::None;
            return was_dragging;
        }
        
        bool showAxis = false;
       private:
        inline void resetBounds() {
            has_local_bounds_ = false;
            local_bound_min_ = {0.0f, 0.0f, 0.0f};
            local_bound_max_ = {0.0f, 0.0f, 0.0f};
        }

        inline void updateLocalBounds(const vec3f& point) {
            if (!has_local_bounds_) {
                local_bound_min_ = point;
                local_bound_max_ = point;
                has_local_bounds_ = true;
                return;
            }

            local_bound_min_.x = std::min(local_bound_min_.x, point.x);
            local_bound_min_.y = std::min(local_bound_min_.y, point.y);
            local_bound_min_.z = std::min(local_bound_min_.z, point.z);
            local_bound_max_.x = std::max(local_bound_max_.x, point.x);
            local_bound_max_.y = std::max(local_bound_max_.y, point.y);
            local_bound_max_.z = std::max(local_bound_max_.z, point.z);
        }

        inline std::array<vec3f, 8> getWorldBoundsCorners() const {
            auto corners = axis_gizmo::buildBoundsCorners(local_bound_min_, local_bound_max_);
            for (auto& corner : corners) {
                corner = axis_gizmo::transformPoint(axis_state_.model_matrix, corner);
            }
            return corners;
        }

        inline void renderAxis(RenderMeshShader & shader) {
            if (!line_layout_initialized_) {
                mesh_detail::ColorLineVertex::init(line_layout_);
                line_layout_initialized_ = true;
            }

            if (!shader.ensureLineProgram()) {
                return;
            }

            std::vector<mesh_detail::ColorLineVertex> axis_vertices;
            axis_gizmo::appendAxisColorVertices(axis_vertices, axis_state_);
            if (axis_vertices.empty()) {
                return;
            }

            if (bgfx::getAvailTransientVertexBuffer(static_cast<uint32_t>(axis_vertices.size()),
                                                    line_layout_) < axis_vertices.size()) {
                return;
            }

            bgfx::TransientVertexBuffer tvb;
            bgfx::allocTransientVertexBuffer(&tvb,
                                             static_cast<uint32_t>(axis_vertices.size()),
                                             line_layout_);
            std::memcpy(tvb.data, axis_vertices.data(),
                        axis_vertices.size() * sizeof(mesh_detail::ColorLineVertex));

            bgfx::setTransform(identity_mtx_);
            bgfx::setVertexBuffer(0, &tvb);
            bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                           BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS |
                           BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
            bgfx::submit(shader.overlay_view_id_, shader.line_program_);
        }

        bgfx::VertexLayout layout_{};
        bgfx::VertexLayout line_layout_{};
        bool layout_initialized_ = false;
        bool line_layout_initialized_ = false;
        mesh_detail::MeshHandle mesh_{};
        axis_gizmo::GizmoState axis_state_{};
        bool has_local_bounds_ = false;
        vec3f local_bound_min_{};
        vec3f local_bound_max_{};
        std::array<float, 4> base_color_ = {0.82f, 0.82f, 0.82f, 1.0f};
        float identity_mtx_[16]{};
    };

    class AsyncVoxelLoader {
       public:
        using PosNormalVertex = mesh_detail::PosNormalVertex;
        using MeshData = mesh_detail::AsyncVoxelMeshData;

        AsyncVoxelLoader() = default;
        ~AsyncVoxelLoader() { cancel(); }

        inline void start(const std::string& filename,
                          float voxel_size = 0.5f,
                          double isolevel = 0.5,
                          bool smooth_normals = true) {
            cancel();
            running_.store(true);
            ready_.store(false);
            progress_.store(0.0f);
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                status_ = "Reading STL...";
            }
            thread_ = std::thread(&AsyncVoxelLoader::doWork, this,
                                  filename, voxel_size, isolevel, smooth_normals);
        }

        inline void cancel() {
            cancel_.store(true);
            if (thread_.joinable()) {
                thread_.join();
            }
            cancel_.store(false);
        }

        inline bool isRunning() const { return running_.load(); }
        inline bool isReady() const { return ready_.load(); }
        inline float getProgress() const { return progress_.load(); }

        inline std::string getStatus() const {
            std::lock_guard<std::mutex> lock(status_mutex_);
            return status_;
        }

        bool tryTakeResult(MeshData& out_rebuild_mesh, sinriv::kigstudio::voxel::VoxelGrid& out_voxel_data) {
            if (!ready_.load()) return false;
            if (thread_.joinable()) {
                thread_.join();
            }
            {
                std::lock_guard<std::mutex> lock(result_mutex_);
                out_rebuild_mesh = std::move(result_mesh);
                out_voxel_data = std::move(result_voxel);
            }
            running_.store(false);
            ready_.store(false);
            progress_.store(0.0f);
            return true;
        }

       private:
        void setStatus(const std::string& s) {
            std::lock_guard<std::mutex> lock(status_mutex_);
            status_ = s;
        }

        void setProgress(float p) {
            progress_.store(p);
        }

        void finishCancel() {
            running_.store(false);
            ready_.store(false);
            progress_.store(0.0f);
            setStatus("Cancelled");
        }

        void doWork(std::string filename, float voxel_size, double isolevel, bool smooth_normals) {
            using namespace sinriv::kigstudio::voxel;
            using Triangle = triangle_bvh<float>::triangle;

            // Phase 1: Read STL
            setProgress(0.05f);
            triangle_bvh<float> bvh;
            size_t tri_count = 0;
            for (auto [tri, n] : readSTL(filename)) {
                (void)n;
                bvh.insert(tri);
                ++tri_count;
                if (tri_count % 1000 == 0) {
                    setProgress(0.05f + 0.08f * std::min(1.0f,
                        static_cast<float>(tri_count) / 50000.0f));
                }
                if (cancel_.load()) {
                    finishCancel();
                    return;
                }
            }

            // Phase 2: Voxelize
            setStatus("Voxelizing...");
            setProgress(0.15f);

            VoxelGrid voxel_data;
            voxel_data.voxel_size.x = voxel_size;
            voxel_data.voxel_size.y = voxel_size;
            voxel_data.voxel_size.z = voxel_size;
            voxel_data.global_position.x = bvh.global_boundBox_min.x;
            voxel_data.global_position.y = bvh.global_boundBox_min.y;
            voxel_data.global_position.z = bvh.global_boundBox_min.z;

            float minx = floor(bvh.global_boundBox_min.x / voxel_size) * voxel_size;
            float miny = floor(bvh.global_boundBox_min.y / voxel_size) * voxel_size;
            float minz = floor(bvh.global_boundBox_min.z / voxel_size) * voxel_size;
            float maxx = ceil(bvh.global_boundBox_max.x / voxel_size) * voxel_size;
            float maxy = ceil(bvh.global_boundBox_max.y / voxel_size) * voxel_size;
            float maxz = ceil(bvh.global_boundBox_max.z / voxel_size) * voxel_size;
            int num_block_y = static_cast<int>(ceil((maxy - miny) / voxel_size));
            int num_block_z = static_cast<int>(ceil((maxz - minz) / voxel_size));
            size_t total_rays = static_cast<size_t>(num_block_y) * num_block_z;
            if (total_rays == 0) total_rays = 1;

            std::mutex locker;
            std::atomic<size_t> callback_count{0};

            bvh.getSolidByFace(
                voxel_size, voxel_size, voxel_size,
                triangle_bvh<float>::voxel_face_X,
                [&](auto start, auto end) {
                    int start_x = static_cast<int>(std::round(
                        (start.x - voxel_data.global_position.x) / voxel_size));
                    int start_y = static_cast<int>(std::round(
                        (start.y - voxel_data.global_position.y) / voxel_size));
                    int start_z = static_cast<int>(std::round(
                        (start.z - voxel_data.global_position.z) / voxel_size));
                    int end_x = static_cast<int>(std::round(
                        (end.x - voxel_data.global_position.x) / voxel_size));
                    int end_y = static_cast<int>(std::round(
                        (end.y - voxel_data.global_position.y) / voxel_size));
                    int end_z = static_cast<int>(std::round(
                        (end.z - voxel_data.global_position.z) / voxel_size));

                    locker.lock();
                    for (int i = start_x; i <= end_x; ++i) {
                        for (int j = start_y; j <= end_y; ++j) {
                            for (int k = start_z; k <= end_z; ++k) {
                                if (i >= 0 && j >= 0 && k >= 0) {
                                    voxel_data.insert({i, j, k});
                                }
                            }
                        }
                    }
                    locker.unlock();

                    size_t cnt = callback_count.fetch_add(1, std::memory_order_relaxed) + 1;
                    if (cnt % 100 == 0) {
                        float p = 0.15f + 0.35f * std::min(1.0f,
                            static_cast<float>(cnt) / static_cast<float>(total_rays * 2));
                        setProgress(p);
                    }
                }
            );

            if (cancel_.load()) {
                finishCancel();
                return;
            }
            setProgress(0.50f);

            // Phase 3: Generate mesh
            setStatus("Generating mesh...");
            int num_triangles = 0;
            auto generator = generateMesh(voxel_data, isolevel, num_triangles, smooth_normals);

            MeshData data;
            size_t processed_tris = 0;
            size_t estimated_tris = voxel_data.num_chunk() * 200;
            if (estimated_tris == 0) estimated_tris = 1;

            for (auto [tri, n] : generator) {
                const uint32_t base = static_cast<uint32_t>(data.vertices.size());
                data.vertices.push_back(
                    {std::get<0>(tri).x, std::get<0>(tri).y, std::get<0>(tri).z,
                     n.x, n.y, n.z});
                data.vertices.push_back(
                    {std::get<1>(tri).x, std::get<1>(tri).y, std::get<1>(tri).z,
                     n.x, n.y, n.z});
                data.vertices.push_back(
                    {std::get<2>(tri).x, std::get<2>(tri).y, std::get<2>(tri).z,
                     n.x, n.y, n.z});
                data.indices.push_back(base);
                data.indices.push_back(base + 1);
                data.indices.push_back(base + 2);

                ++processed_tris;
                if (processed_tris % 1000 == 0) {
                    float p = 0.50f + 0.40f * std::min(1.0f,
                        static_cast<float>(processed_tris) / static_cast<float>(estimated_tris));
                    setProgress(p);
                }
                if (cancel_.load()) {
                    finishCancel();
                    return;
                }
            }

            setStatus("Uploading...");
            setProgress(0.95f);

            {
                std::lock_guard<std::mutex> lock(result_mutex_);
                result_mesh = std::move(data);
                result_voxel = std::move(voxel_data);
            }

            setStatus("Done");
            setProgress(1.0f);
            running_.store(false);
            ready_.store(true);
        }

        std::atomic<bool> running_{false};
        std::atomic<bool> ready_{false};
        std::atomic<bool> cancel_{false};
        std::atomic<float> progress_{0.0f};
        mutable std::mutex status_mutex_;
        std::string status_;
        mutable std::mutex result_mutex_;
        MeshData result_mesh;
        sinriv::kigstudio::voxel::VoxelGrid result_voxel;
        std::thread thread_;
    };
}
