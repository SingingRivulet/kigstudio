#pragma once
#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <bx/math.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <cstring>
#include <cstdio>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "kigstudio/ui/render_axis_gizmo.h"
#include "kigstudio/voxel/collision.h"

namespace sinriv::ui::render {
    namespace detail {
        struct CollisionLineVertex {
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
            return bgfx::createShader(bgfx::copy(data.data(), static_cast<uint32_t>(data.size())));
        }
    }

    class RenderCollisionShader {
    public:
        bgfx::ViewId view_id_ = 0;
        bgfx::ViewId overlay_view_id_ = 0;
        std::string shader_dir_ = "../../shader/base/";
        bgfx::ProgramHandle program_ = BGFX_INVALID_HANDLE;

        inline explicit RenderCollisionShader(bgfx::ViewId view_id = 0,
                                 bgfx::ViewId overlay_view_id = 0,
                                 std::string shader_dir = "../../shader/base/")
            : view_id_(view_id),
              overlay_view_id_(overlay_view_id),
              shader_dir_(std::move(shader_dir)) {}

        inline bool ensureProgram() {
            if (bgfx::isValid(program_)) {
                return true;
            }

            bgfx::ShaderHandle vs = detail::loadShader(shader_dir_ + "vs_color_line.bin");
            bgfx::ShaderHandle fs = detail::loadShader(shader_dir_ + "fs_color_line.bin");
            if (!bgfx::isValid(vs) || !bgfx::isValid(fs)) {
                if (bgfx::isValid(vs)) {
                    bgfx::destroy(vs);
                }
                if (bgfx::isValid(fs)) {
                    bgfx::destroy(fs);
                }
                std::cerr << "RenderCollision shader load failed from " << shader_dir_ << std::endl;
                return false;
            }

            program_ = bgfx::createProgram(vs, fs, true);
            return bgfx::isValid(program_);
        }
        inline void destroyProgram() {
            if (bgfx::isValid(program_)) {
                bgfx::destroy(program_);
                program_ = BGFX_INVALID_HANDLE;
                std::cout << "RenderCollision shader destroyed" << std::endl;
            }
        }
        inline void release() {
            destroyProgram();
        }
    };

    class RenderCollision {
    public:
        using vec3f = sinriv::kigstudio::vec3<float>;
        using mat4f = sinriv::kigstudio::mat::matrix<float>;
        using CollisionGroup = sinriv::kigstudio::voxel::collision::CollisionGroup;
        using GeometryInstance = sinriv::kigstudio::voxel::collision::GeometryInstance;
        using Sphere = sinriv::kigstudio::voxel::collision::Sphere;
        using Cylinder = sinriv::kigstudio::voxel::collision::Cylinder;
        using Capsule = sinriv::kigstudio::voxel::collision::Capsule;
        using OBB = sinriv::kigstudio::voxel::collision::OBB;
        using AxisHandle = axis_gizmo::AxisHandle;

        inline explicit RenderCollision()
            : vertex_count_per_circle_(32) {}

        inline ~RenderCollision() {
            release();
        }

        inline void release() {}

        inline void setCircleSegments(uint16_t segments) {
            vertex_count_per_circle_ = std::max<uint16_t>(segments, 8);
        }

        inline uint16_t getCircleSegments() const {
            return vertex_count_per_circle_;
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

        inline void render(const CollisionGroup& geo_group, RenderCollisionShader & shader){
            float identity[16];
            bx::mtxIdentity(identity);
            render(geo_group, identity, identity, shader);
        }

        inline void render(const CollisionGroup& geo_group,
                           const float* model_transform,
                           const float* model_transform_2,
                           RenderCollisionShader & shader,
                           const mat4f* cpu_model_matrix = nullptr){
            if (!layout_initialized_) {
                detail::CollisionLineVertex::init(layout_);
                layout_initialized_ = true;
            }
            if (!shader.ensureProgram()) {
                return;
            }
            const mat4f model_matrix =
                cpu_model_matrix ? *cpu_model_matrix : mat4f(model_transform);
            updateBounds(geo_group, model_matrix);
            render_collision(geo_group, model_transform, shader);
            
            if (has_world_bounds_) {
                axis_state_.axis_length = axis_gizmo::estimateAxisLengthFromBounds(
                    world_bound_min_, world_bound_max_);
            }
            // axis_state_.model_matrix = geo_group.transform.getBgfxMatrix() * mat4f(model_transform);
            {
                sinriv::kigstudio::voxel::collision::Quaternion tmp_rotation;
                tmp_rotation.x = -geo_group.transform.rotation_.x;
                tmp_rotation.y = -geo_group.transform.rotation_.y;
                tmp_rotation.z = -geo_group.transform.rotation_.z;
                tmp_rotation.w = geo_group.transform.rotation_.w;
                //bgfx渲染需要使用共轭的四元数，原因未知
                vec3f tmp_position;
                tmp_position.x = geo_group.transform.position_.x;
                tmp_position.y = geo_group.transform.position_.y;
                tmp_position.z = -geo_group.transform.position_.z; //bgfx需要反转z轴，原因未知
                auto mtx = sinriv::kigstudio::voxel::collision::composeMatrix(tmp_position, tmp_rotation, geo_group.transform.scale_); 
                axis_state_.model_matrix = model_matrix * mtx;
            }

            if (showAxis) {
                renderAxis(shader);
            }
        }
        //计算物体在屏幕上覆盖的矩形区域（x1,y1,x2,y2），用于提前过滤鼠标事件
        inline std::tuple<int, int, int, int> getScreenBoundBox() {
            if (!has_world_bounds_) {
                return {0, 0, 0, 0};
            }
            return axis_gizmo::projectBoundsToScreen(
                axis_gizmo::buildBoundsCorners(world_bound_min_, world_bound_max_),
                axis_state_);
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
            has_world_bounds_ = false;
            world_bound_min_ = {0.0f, 0.0f, 0.0f};
            world_bound_max_ = {0.0f, 0.0f, 0.0f};
        }

        inline void expandBounds(const vec3f& point) {
            if (!has_world_bounds_) {
                world_bound_min_ = point;
                world_bound_max_ = point;
                has_world_bounds_ = true;
                return;
            }

            world_bound_min_.x = std::min(world_bound_min_.x, point.x);
            world_bound_min_.y = std::min(world_bound_min_.y, point.y);
            world_bound_min_.z = std::min(world_bound_min_.z, point.z);
            world_bound_max_.x = std::max(world_bound_max_.x, point.x);
            world_bound_max_.y = std::max(world_bound_max_.y, point.y);
            world_bound_max_.z = std::max(world_bound_max_.z, point.z);
        }

        inline void expandSphereBounds(const Sphere& sphere, const mat4f& world_matrix) {
            const float r = sphere.radius;
            for (const vec3f& offset : std::array<vec3f, 8>{
                     vec3f{-r, -r, -r}, vec3f{r, -r, -r}, vec3f{r, r, -r}, vec3f{-r, r, -r},
                     vec3f{-r, -r, r}, vec3f{r, -r, r}, vec3f{r, r, r}, vec3f{-r, r, r}}) {
                expandBounds(sinriv::kigstudio::voxel::collision::transformPoint(
                    world_matrix, sphere.center + offset));
            }
        }

        inline void expandCylinderBounds(const Cylinder& cylinder, const mat4f& world_matrix) {
            const vec3f axis = cylinder.end - cylinder.start;
            const float axis_length = axis.length();
            const vec3f axis_dir =
                axis_length > 1e-6f ? axis / axis_length : vec3f(0.0f, 0.0f, 1.0f);
            const vec3f helper = std::fabs(axis_dir.z) < 0.99f ? vec3f(0.0f, 0.0f, 1.0f)
                                                                : vec3f(0.0f, 1.0f, 0.0f);
            const vec3f tangent = axis_dir.cross(helper).normalize();
            const vec3f bitangent = axis_dir.cross(tangent).normalize();
            const float r = cylinder.radius;
            for (const vec3f& center : {cylinder.start, cylinder.end}) {
                for (const vec3f& offset : std::array<vec3f, 4>{
                         tangent * r, tangent * -r, bitangent * r, bitangent * -r}) {
                    expandBounds(sinriv::kigstudio::voxel::collision::transformPoint(
                        world_matrix, center + offset));
                }
            }
        }

        inline void expandCapsuleBounds(const Capsule& capsule, const mat4f& world_matrix) {
            expandSphereBounds({capsule.start, capsule.radius}, world_matrix);
            expandSphereBounds({capsule.end, capsule.radius}, world_matrix);
            expandCylinderBounds({capsule.start, capsule.end, capsule.radius}, world_matrix);
        }

        inline void expandOBBBounds(const OBB& obb, const mat4f& world_matrix) {
            const vec3f x = obb.axis_x * obb.half_extent.x;
            const vec3f y = obb.axis_y * obb.half_extent.y;
            const vec3f z = obb.axis_z * obb.half_extent.z;
            for (const vec3f& point : std::array<vec3f, 8>{
                     obb.center - x - y - z, obb.center + x - y - z, obb.center + x + y - z,
                     obb.center - x + y - z, obb.center - x - y + z, obb.center + x - y + z,
                     obb.center + x + y + z, obb.center - x + y + z}) {
                expandBounds(sinriv::kigstudio::voxel::collision::transformPoint(world_matrix,
                                                                                  point));
            }
        }

        inline void updateBounds(const CollisionGroup& geo_group,
                                 const mat4f& model_matrix) {
            resetBounds();
            const mat4f group_matrix = geo_group.transform.getBgfxMatrix() * model_matrix;
            axis_state_.model_matrix = group_matrix;

            for (const auto& geometry : geo_group.geometries()) {
                const mat4f world_matrix = geometry.transform.getBgfxMatrix() * group_matrix;
                std::visit(
                    [&](const auto& shape) {
                        using ShapeType = std::decay_t<decltype(shape)>;
                        if constexpr (std::is_same_v<ShapeType, Sphere>) {
                            expandSphereBounds(shape, world_matrix);
                        } else if constexpr (std::is_same_v<ShapeType, Cylinder>) {
                            expandCylinderBounds(shape, world_matrix);
                        } else if constexpr (std::is_same_v<ShapeType, Capsule>) {
                            expandCapsuleBounds(shape, world_matrix);
                        } else if constexpr (std::is_same_v<ShapeType, OBB>) {
                            expandOBBBounds(shape, world_matrix);
                        }
                    },
                    geometry.geometry);
            }
        }

        inline void renderAxis(RenderCollisionShader & shader) {
            std::vector<detail::CollisionLineVertex> axis_vertices;
            axis_gizmo::appendAxisColorVertices(axis_vertices, axis_state_);
            if (axis_vertices.empty()) {
                return;
            }

            if (bgfx::getAvailTransientVertexBuffer(static_cast<uint32_t>(axis_vertices.size()),
                                                    layout_) < axis_vertices.size()) {
                return;
            }

            bgfx::TransientVertexBuffer tvb;
            bgfx::allocTransientVertexBuffer(&tvb,
                                             static_cast<uint32_t>(axis_vertices.size()),
                                             layout_);
            std::memcpy(tvb.data, axis_vertices.data(),
                        axis_vertices.size() * sizeof(detail::CollisionLineVertex));

            bx::mtxIdentity(identity_mtx_);
            bgfx::setTransform(identity_mtx_);
            bgfx::setVertexBuffer(0, &tvb);
            bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                           BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS |
                           BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
            bgfx::submit(shader.overlay_view_id_, shader.program_);
        }

        inline void appendLine(std::vector<detail::CollisionLineVertex>& vertices,
                               const vec3f& a,
                               const vec3f& b,
                               uint32_t color = 0xffffffff) const {
            vertices.push_back({a.x, a.y, a.z, color});
            vertices.push_back({b.x, b.y, b.z, color});
        }

        inline void appendCircle(std::vector<detail::CollisionLineVertex>& vertices,
                                 const mat4f& world_matrix,
                                 const vec3f& center,
                                 const vec3f& axis_u,
                                 const vec3f& axis_v,
                                 float radius) const {
            if (radius <= 0.0f) {
                return;
            }

            const float pi = 3.14159265358979323846f;
            for (uint16_t i = 0; i < vertex_count_per_circle_; ++i) {
                const float angle0 = (2.0f * pi * i) / vertex_count_per_circle_;
                const float angle1 = (2.0f * pi * (i + 1)) / vertex_count_per_circle_;

                const vec3f local0 = center + axis_u * (std::cos(angle0) * radius) +
                                     axis_v * (std::sin(angle0) * radius);
                const vec3f local1 = center + axis_u * (std::cos(angle1) * radius) +
                                     axis_v * (std::sin(angle1) * radius);

                appendLine(vertices,
                           sinriv::kigstudio::voxel::collision::transformPoint(world_matrix, local0),
                           sinriv::kigstudio::voxel::collision::transformPoint(world_matrix, local1));
            }
        }

        inline void appendSphere(std::vector<detail::CollisionLineVertex>& vertices,
                                 const Sphere& sphere,
                                 const mat4f& world_matrix) const {
            appendCircle(vertices, world_matrix, sphere.center, {1.0f, 0.0f, 0.0f},
                         {0.0f, 1.0f, 0.0f}, sphere.radius);
            appendCircle(vertices, world_matrix, sphere.center, {1.0f, 0.0f, 0.0f},
                         {0.0f, 0.0f, 1.0f}, sphere.radius);
            appendCircle(vertices, world_matrix, sphere.center, {0.0f, 1.0f, 0.0f},
                         {0.0f, 0.0f, 1.0f}, sphere.radius);
        }

        inline void appendCylinder(std::vector<detail::CollisionLineVertex>& vertices,
                                   const Cylinder& cylinder,
                                   const mat4f& world_matrix) const {
            const vec3f axis = cylinder.end - cylinder.start;
            const float axis_length = axis.length();
            const vec3f axis_dir = axis_length > 1e-6f ? axis / axis_length : vec3f(0.0f, 0.0f, 1.0f);
            const vec3f helper = std::fabs(axis_dir.z) < 0.99f ? vec3f(0.0f, 0.0f, 1.0f) : vec3f(0.0f, 1.0f, 0.0f);
            const vec3f tangent = axis_dir.cross(helper).normalize();
            const vec3f bitangent = axis_dir.cross(tangent).normalize();

            appendCircle(vertices, world_matrix, cylinder.start, tangent, bitangent, cylinder.radius);
            appendCircle(vertices, world_matrix, cylinder.end, tangent, bitangent, cylinder.radius);

            for (uint16_t i = 0; i < 4; ++i) {
                const float angle = (2.0f * 3.14159265358979323846f * i) / 4.0f;
                const vec3f offset = tangent * (std::cos(angle) * cylinder.radius) +
                                     bitangent * (std::sin(angle) * cylinder.radius);
                appendLine(vertices,
                           sinriv::kigstudio::voxel::collision::transformPoint(world_matrix, cylinder.start + offset),
                           sinriv::kigstudio::voxel::collision::transformPoint(world_matrix, cylinder.end + offset));
            }
        }

        inline void appendCapsule(std::vector<detail::CollisionLineVertex>& vertices,
                                  const Capsule& capsule,
                                  const mat4f& world_matrix) const {
            const Cylinder body{capsule.start, capsule.end, capsule.radius};
            appendCylinder(vertices, body, world_matrix);

            const vec3f axis = capsule.end - capsule.start;
            const float axis_length = axis.length();
            const vec3f axis_dir = axis_length > 1e-6f ? axis / axis_length : vec3f(0.0f, 0.0f, 1.0f);
            const vec3f helper = std::fabs(axis_dir.z) < 0.99f ? vec3f(0.0f, 0.0f, 1.0f) : vec3f(0.0f, 1.0f, 0.0f);
            const vec3f tangent = axis_dir.cross(helper).normalize();
            const vec3f bitangent = axis_dir.cross(tangent).normalize();

            appendCircle(vertices, world_matrix, capsule.start, tangent, axis_dir, capsule.radius);
            appendCircle(vertices, world_matrix, capsule.start, bitangent, axis_dir, capsule.radius);
            appendCircle(vertices, world_matrix, capsule.end, tangent, axis_dir, capsule.radius);
            appendCircle(vertices, world_matrix, capsule.end, bitangent, axis_dir, capsule.radius);
        }

        inline void appendOBB(std::vector<detail::CollisionLineVertex>& vertices,
                              const OBB& obb,
                              const mat4f& world_matrix) const {
            const vec3f x = obb.axis_x * obb.half_extent.x;
            const vec3f y = obb.axis_y * obb.half_extent.y;
            const vec3f z = obb.axis_z * obb.half_extent.z;

            const vec3f corners[8] = {
                obb.center - x - y - z,
                obb.center + x - y - z,
                obb.center + x + y - z,
                obb.center - x + y - z,
                obb.center - x - y + z,
                obb.center + x - y + z,
                obb.center + x + y + z,
                obb.center - x + y + z,
            };

            static const uint8_t edges[12][2] = {
                {0, 1}, {1, 2}, {2, 3}, {3, 0},
                {4, 5}, {5, 6}, {6, 7}, {7, 4},
                {0, 4}, {1, 5}, {2, 6}, {3, 7},
            };

            for (const auto& edge : edges) {
                appendLine(vertices,
                           sinriv::kigstudio::voxel::collision::transformPoint(world_matrix, corners[edge[0]]),
                           sinriv::kigstudio::voxel::collision::transformPoint(world_matrix, corners[edge[1]]));
            }
        }

        inline void appendGeometry(std::vector<detail::CollisionLineVertex>& vertices,
                                   const GeometryInstance& geometry,
                                   const mat4f& group_matrix) const {
            const mat4f world_matrix = geometry.transform.getMatrix() * group_matrix;
            std::visit([&](const auto& shape) {
                using ShapeType = std::decay_t<decltype(shape)>;
                if constexpr (std::is_same_v<ShapeType, Sphere>) {
                    appendSphere(vertices, shape, world_matrix);
                } else if constexpr (std::is_same_v<ShapeType, Cylinder>) {
                    appendCylinder(vertices, shape, world_matrix);
                } else if constexpr (std::is_same_v<ShapeType, Capsule>) {
                    appendCapsule(vertices, shape, world_matrix);
                } else if constexpr (std::is_same_v<ShapeType, OBB>) {
                    appendOBB(vertices, shape, world_matrix);
                }
            }, geometry.geometry);
        }

        void render_collision(const CollisionGroup& geo_group,
                              const float* model_transform,
                              RenderCollisionShader & shader){
            std::vector<detail::CollisionLineVertex> vertices;
            vertices.reserve(geo_group.geometries().size() * vertex_count_per_circle_ * 12);

            const mat4f group_matrix = geo_group.getMatrix();
            for (const auto& geometry : geo_group.geometries()) {
                appendGeometry(vertices, geometry, group_matrix);
            }

            if (vertices.empty()) {
                return;
            }

            if (bgfx::getAvailTransientVertexBuffer(static_cast<uint32_t>(vertices.size()), layout_) < vertices.size()) {
                return;
            }

            bgfx::TransientVertexBuffer tvb;
            bgfx::allocTransientVertexBuffer(&tvb, static_cast<uint32_t>(vertices.size()), layout_);
            std::memcpy(tvb.data, vertices.data(), vertices.size() * sizeof(detail::CollisionLineVertex));

            bgfx::setTransform(model_transform);
            bgfx::setVertexBuffer(0, &tvb);
            bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                           BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS |
                           BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
            bgfx::submit(shader.overlay_view_id_, shader.program_);
        }

        uint16_t vertex_count_per_circle_ = 32;
        bgfx::VertexLayout layout_{};
        bool layout_initialized_ = false;
        axis_gizmo::GizmoState axis_state_{};
        bool has_world_bounds_ = false;
        vec3f world_bound_min_{};
        vec3f world_bound_max_{};
        float identity_mtx_[16]{};
    };

}
