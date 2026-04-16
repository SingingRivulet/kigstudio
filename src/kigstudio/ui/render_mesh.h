#pragma once
#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <bx/math.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
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

        inline bgfx::ShaderHandle loadShader(const std::string& path) {
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

    class RenderMesh {
       public:
        using AxisHandle = axis_gizmo::AxisHandle;
        using vec3f = sinriv::kigstudio::vec3<float>;
        using mat4f = sinriv::kigstudio::mat::matrix<float>;

        explicit RenderMesh(bgfx::ViewId view_id = 0,
                            bgfx::ViewId overlay_view_id = 0,
                            std::string shader_dir = "../../shader/base/")
            : view_id_(view_id),
              overlay_view_id_(overlay_view_id),
              shader_dir_(std::move(shader_dir)) {}

        ~RenderMesh() {
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

        inline bool empty() const {
            return !bgfx::isValid(mesh_.vbh) || !bgfx::isValid(mesh_.ibh) ||
                   mesh_.index_count == 0;
        }

        inline void release() {
            mesh_.destroy();
            destroyPrograms();
            destroyUniforms();
        }

        inline void clear() {
            mesh_.destroy();
            resetBounds();
        }

        void renderGBuffer(const float* transform) {
            axis_state_.model_matrix = mat4f(transform);
            if (empty() || !ensureGBufferProgram()) {
                return;
            }

            bgfx::setTransform(transform);
            bgfx::setVertexBuffer(0, mesh_.vbh);
            bgfx::setIndexBuffer(mesh_.ibh);
            ensureUniforms();
            bgfx::setUniform(u_base_color_, base_color_.data());
            bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                           BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS |
                           BGFX_STATE_CULL_CCW | BGFX_STATE_MSAA);
            bgfx::submit(view_id_, gbuffer_program_);
        }

        void renderOverlay() {
            if (!empty() && showAxis) {
                renderAxis();
            }
        }

        void render(const float* transform) {
            renderGBuffer(transform);
            renderOverlay();
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

        inline void renderAxis() {
            if (!ensureLineProgram()) {
                return;
            }

            if (!line_layout_initialized_) {
                mesh_detail::ColorLineVertex::init(line_layout_);
                line_layout_initialized_ = true;
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
            bgfx::submit(overlay_view_id_, line_program_);
        }

        inline void destroyPrograms() {
            if (bgfx::isValid(gbuffer_program_)) {
                bgfx::destroy(gbuffer_program_);
                gbuffer_program_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(line_program_)) {
                bgfx::destroy(line_program_);
                line_program_ = BGFX_INVALID_HANDLE;
            }
        }

        inline void destroyUniforms() {
            if (bgfx::isValid(u_base_color_)) {
                bgfx::destroy(u_base_color_);
                u_base_color_ = BGFX_INVALID_HANDLE;
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
            if (!layout_initialized_) {
                mesh_detail::PosNormalVertex::init(layout_);
                layout_initialized_ = true;
            }
            ensureUniforms();
            bx::mtxIdentity(identity_mtx_);

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
            if (!line_layout_initialized_) {
                mesh_detail::ColorLineVertex::init(line_layout_);
                line_layout_initialized_ = true;
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

        bgfx::ViewId view_id_ = 0;
        bgfx::ViewId overlay_view_id_ = 0;
        std::string shader_dir_ = "../../shader/base/";
        bgfx::VertexLayout layout_{};
        bgfx::VertexLayout line_layout_{};
        bool layout_initialized_ = false;
        bool line_layout_initialized_ = false;
        mesh_detail::MeshHandle mesh_{};
        bgfx::ProgramHandle gbuffer_program_ = BGFX_INVALID_HANDLE;
        bgfx::ProgramHandle line_program_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_base_color_ = BGFX_INVALID_HANDLE;
        axis_gizmo::GizmoState axis_state_{};
        bool has_local_bounds_ = false;
        vec3f local_bound_min_{};
        vec3f local_bound_max_{};
        std::array<float, 4> base_color_ = {0.82f, 0.82f, 0.82f, 1.0f};
        float identity_mtx_[16]{};
    };
}
