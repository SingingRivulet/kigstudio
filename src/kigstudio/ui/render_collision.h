#pragma once
#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <bx/math.h>
#include <iostream>
#include <cstring>
#include <cstdio>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "kigstudio/voxel/collision.h"

namespace sinriv::ui::render {
    namespace detail {
        struct CollisionLineVertex {
            float x, y, z;
            float nx, ny, nz;

            static inline void init(bgfx::VertexLayout& layout) {
                layout.begin()
                    .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
                    .add(bgfx::Attrib::Normal, 3, bgfx::AttribType::Float)
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

        explicit RenderCollision(bgfx::ViewId view_id = 0,
                                 std::string shader_dir = "../../shader/base/")
            : view_id_(view_id),
              vertex_count_per_circle_(32),
              shader_dir_(std::move(shader_dir)) {}

        ~RenderCollision() {
            release();
        }

        inline void release() {
            if (bgfx::isValid(program_)) {
                bgfx::destroy(program_);
                program_ = BGFX_INVALID_HANDLE;
            }
        }

        inline void setViewId(bgfx::ViewId view_id) {
            view_id_ = view_id;
        }

        inline bgfx::ViewId getViewId() const {
            return view_id_;
        }

        inline void setCircleSegments(uint16_t segments) {
            vertex_count_per_circle_ = std::max<uint16_t>(segments, 8);
        }

        inline uint16_t getCircleSegments() const {
            return vertex_count_per_circle_;
        }

        inline void setShaderDirectory(const std::string& shader_dir) {
            shader_dir_ = shader_dir;
            if (bgfx::isValid(program_)) {
                bgfx::destroy(program_);
                program_ = BGFX_INVALID_HANDLE;
            }
        }

        void render(const CollisionGroup& geo_group){
            if (!ensureProgram()) {
                return;
            }
            render_collision(geo_group);
        }
    private:
        inline bool ensureProgram() {
            if (bgfx::isValid(program_)) {
                return true;
            }

            if (!layout_initialized_) {
                detail::CollisionLineVertex::init(layout_);
                layout_initialized_ = true;
            }

            bgfx::ShaderHandle vs = detail::loadShader(shader_dir_ + "vs_mesh_base.bin");
            bgfx::ShaderHandle fs = detail::loadShader(shader_dir_ + "fs_mesh_base.bin");
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

        inline void appendLine(std::vector<detail::CollisionLineVertex>& vertices,
                               const vec3f& a,
                               const vec3f& b) const {
            vec3f normal = b - a;
            const float len = normal.length();
            if (len > 1e-6f) {
                normal /= len;
            } else {
                normal = {0.0f, 0.0f, 1.0f};
            }

            vertices.push_back({a.x, a.y, a.z, normal.x, normal.y, normal.z});
            vertices.push_back({b.x, b.y, b.z, normal.x, normal.y, normal.z});
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

        void render_collision(const CollisionGroup& geo_group){
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

            float identity[16];
            bx::mtxIdentity(identity);
            bgfx::setTransform(identity);
            bgfx::setVertexBuffer(0, &tvb);
            bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                           BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS |
                           BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
            bgfx::submit(view_id_, program_);
        }

        bgfx::ViewId view_id_ = 0;
        uint16_t vertex_count_per_circle_ = 32;
        std::string shader_dir_ = "../../shader/base/";
        bgfx::VertexLayout layout_{};
        bool layout_initialized_ = false;
        bgfx::ProgramHandle program_ = BGFX_INVALID_HANDLE;
    };

}
