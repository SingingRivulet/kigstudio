#pragma once

#include <bgfx/bgfx.h>
#include <bx/math.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <type_traits>
#include <vector>

#include "kigstudio/voxel/collision.h"

namespace sinriv::ui::render {
    namespace deferred_detail {
        using vec3f = sinriv::kigstudio::voxel::collision::vec3f;
        using mat4f = sinriv::kigstudio::voxel::collision::mat4f;

        struct ScreenVertex {
            float x, y, z;
            float u, v;

            static inline void init(bgfx::VertexLayout& layout) {
                layout.begin()
                    .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
                    .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
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

        inline vec3f transformVector(const mat4f& matrix, const vec3f& value) {
            return {
                value.x * matrix[0][0] + value.y * matrix[1][0] + value.z * matrix[2][0],
                value.x * matrix[0][1] + value.y * matrix[1][1] + value.z * matrix[2][1],
                value.x * matrix[0][2] + value.y * matrix[1][2] + value.z * matrix[2][2],
            };
        }

        inline float safeLength(const vec3f& value) {
            return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
        }
    }

    class RenderDeferred {
       public:
        using CollisionGroup = sinriv::kigstudio::voxel::collision::CollisionGroup;
        using GeometryInstance = sinriv::kigstudio::voxel::collision::GeometryInstance;
        using Sphere = sinriv::kigstudio::voxel::collision::Sphere;
        using Cylinder = sinriv::kigstudio::voxel::collision::Cylinder;
        using Capsule = sinriv::kigstudio::voxel::collision::Capsule;
        using OBB = sinriv::kigstudio::voxel::collision::OBB;
        using vec3f = deferred_detail::vec3f;
        using mat4f = deferred_detail::mat4f;

        explicit RenderDeferred(bgfx::ViewId gbuffer_view_id = 0,
                                bgfx::ViewId lighting_view_id = 1,
                                std::string shader_dir = "../../shader/base/")
            : gbuffer_view_id_(gbuffer_view_id),
              lighting_view_id_(lighting_view_id),
              shader_dir_(std::move(shader_dir)) {}

        ~RenderDeferred() { release(); }

        inline bgfx::ViewId getGBufferViewId() const { return gbuffer_view_id_; }
        inline bgfx::ViewId getLightingViewId() const { return lighting_view_id_; }

        inline void setViewIds(bgfx::ViewId gbuffer_view_id,
                               bgfx::ViewId lighting_view_id) {
            gbuffer_view_id_ = gbuffer_view_id;
            lighting_view_id_ = lighting_view_id;
        }

        inline void setShaderDirectory(const std::string& shader_dir) {
            shader_dir_ = shader_dir;
            destroyPrograms();
        }

        inline void setViewportSize(uint16_t width, uint16_t height) {
            width_ = std::max<uint16_t>(width, 1);
            height_ = std::max<uint16_t>(height, 1);
        }

        inline void setLightDirection(float x, float y, float z) {
            light_dir_[0] = x;
            light_dir_[1] = y;
            light_dir_[2] = z;
        }

        inline void clearCollisionTint() {
            sphere_count_ = 0;
            cylinder_count_ = 0;
            capsule_count_ = 0;
            obb_count_ = 0;
        }

        inline void setCollisionGroup(const CollisionGroup& group) {
            clearCollisionTint();

            const mat4f group_matrix = group.getMatrix();
            for (const auto& geometry : group.geometries()) {
                const mat4f local_matrix = geometry.transform.getMatrix() * group_matrix;
                std::visit(
                    [&](const auto& shape) {
                        using ShapeType = std::decay_t<decltype(shape)>;
                        if constexpr (std::is_same_v<ShapeType, Sphere>) {
                            appendSphere(shape, local_matrix);
                        } else if constexpr (std::is_same_v<ShapeType, Cylinder>) {
                            appendCylinder(shape, local_matrix);
                        } else if constexpr (std::is_same_v<ShapeType, Capsule>) {
                            appendCapsule(shape, local_matrix);
                        } else if constexpr (std::is_same_v<ShapeType, OBB>) {
                            appendOBB(shape, local_matrix);
                        }
                    },
                    geometry.geometry);
            }
        }

        inline void prepareFrame() {
            if (!ensureFrameBuffer()) {
                return;
            }

            bgfx::setViewName(gbuffer_view_id_, "GBuffer");
            bgfx::setViewFrameBuffer(gbuffer_view_id_, gbuffer_);
            bgfx::setViewRect(gbuffer_view_id_, 0, 0, width_, height_);
            bgfx::setViewClear(gbuffer_view_id_,
                               BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                               0x00000000, 1.0f, 0);
            bgfx::touch(gbuffer_view_id_);

            float view[16];
            float proj[16];
            bx::mtxIdentity(view);
            bx::mtxOrtho(proj, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 100.0f, 0.0f,
                         bgfx::getCaps()->homogeneousDepth);

            bgfx::setViewName(lighting_view_id_, "DeferredLighting");
            bgfx::setViewFrameBuffer(lighting_view_id_, BGFX_INVALID_HANDLE);
            bgfx::setViewRect(lighting_view_id_, 0, 0, width_, height_);
            bgfx::setViewTransform(lighting_view_id_, view, proj);
            bgfx::setViewClear(lighting_view_id_,
                               BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                               0x303030ff, 1.0f, 0);
            bgfx::touch(lighting_view_id_);
        }

        inline void render() {
            if (!ensureFrameBuffer() || !ensureProgram()) {
                return;
            }

            if (bgfx::getAvailTransientVertexBuffer(4, screen_layout_) < 4 ||
                bgfx::getAvailTransientIndexBuffer(6) < 6) {
                return;
            }

            static constexpr deferred_detail::ScreenVertex kQuadVertices[4] = {
                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                {1.0f, 0.0f, 0.0f, 1.0f, 0.0f},
                {1.0f, 1.0f, 0.0f, 1.0f, 1.0f},
                {0.0f, 1.0f, 0.0f, 0.0f, 1.0f},
            };
            static constexpr uint16_t kQuadIndices[6] = {0, 1, 2, 0, 2, 3};

            bgfx::TransientVertexBuffer tvb;
            bgfx::TransientIndexBuffer tib;
            bgfx::allocTransientVertexBuffer(&tvb, 4, screen_layout_);
            bgfx::allocTransientIndexBuffer(&tib, 6);
            std::memcpy(tvb.data, kQuadVertices, sizeof(kQuadVertices));
            std::memcpy(tib.data, kQuadIndices, sizeof(kQuadIndices));

            bgfx::setTransform(identity_mtx_);
            bgfx::setVertexBuffer(0, &tvb);
            bgfx::setIndexBuffer(&tib);
            bgfx::setTexture(0, s_albedo_, albedo_texture_);
            bgfx::setTexture(1, s_normal_, normal_texture_);
            bgfx::setTexture(2, s_world_pos_, world_pos_texture_);
            bgfx::setUniform(u_light_dir_, light_dir_.data());
            bgfx::setUniform(u_collision_counts_, collision_counts_.data());
            if (sphere_count_ > 0) {
                bgfx::setUniform(u_collision_spheres_, sphere_data_.data(),
                                 static_cast<uint16_t>(sphere_count_));
            }
            if (cylinder_count_ > 0) {
                bgfx::setUniform(u_collision_cylinders_a_, cylinder_start_radius_.data(),
                                 static_cast<uint16_t>(cylinder_count_));
                bgfx::setUniform(u_collision_cylinders_b_, cylinder_end_.data(),
                                 static_cast<uint16_t>(cylinder_count_));
            }
            if (capsule_count_ > 0) {
                bgfx::setUniform(u_collision_capsules_a_, capsule_start_radius_.data(),
                                 static_cast<uint16_t>(capsule_count_));
                bgfx::setUniform(u_collision_capsules_b_, capsule_end_.data(),
                                 static_cast<uint16_t>(capsule_count_));
            }
            if (obb_count_ > 0) {
                bgfx::setUniform(u_collision_obb_center_, obb_center_.data(),
                                 static_cast<uint16_t>(obb_count_));
                bgfx::setUniform(u_collision_obb_axis_x_, obb_axis_x_.data(),
                                 static_cast<uint16_t>(obb_count_));
                bgfx::setUniform(u_collision_obb_axis_y_, obb_axis_y_.data(),
                                 static_cast<uint16_t>(obb_count_));
                bgfx::setUniform(u_collision_obb_axis_z_, obb_axis_z_.data(),
                                 static_cast<uint16_t>(obb_count_));
            }
            bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                           BGFX_STATE_MSAA);
            bgfx::submit(lighting_view_id_, combine_program_);
        }

        inline void release() {
            destroyPrograms();
            destroyFrameBuffer();
            if (bgfx::isValid(s_albedo_)) {
                bgfx::destroy(s_albedo_);
                s_albedo_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(s_normal_)) {
                bgfx::destroy(s_normal_);
                s_normal_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(s_world_pos_)) {
                bgfx::destroy(s_world_pos_);
                s_world_pos_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(u_light_dir_)) {
                bgfx::destroy(u_light_dir_);
                u_light_dir_ = BGFX_INVALID_HANDLE;
            }
            destroyCollisionUniforms();
        }

       private:
        static constexpr std::size_t kMaxCollisionShapes = 16;

        inline void destroyCollisionUniforms() {
            if (bgfx::isValid(u_collision_counts_)) {
                bgfx::destroy(u_collision_counts_);
                u_collision_counts_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(u_collision_spheres_)) {
                bgfx::destroy(u_collision_spheres_);
                u_collision_spheres_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(u_collision_cylinders_a_)) {
                bgfx::destroy(u_collision_cylinders_a_);
                u_collision_cylinders_a_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(u_collision_cylinders_b_)) {
                bgfx::destroy(u_collision_cylinders_b_);
                u_collision_cylinders_b_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(u_collision_capsules_a_)) {
                bgfx::destroy(u_collision_capsules_a_);
                u_collision_capsules_a_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(u_collision_capsules_b_)) {
                bgfx::destroy(u_collision_capsules_b_);
                u_collision_capsules_b_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(u_collision_obb_center_)) {
                bgfx::destroy(u_collision_obb_center_);
                u_collision_obb_center_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(u_collision_obb_axis_x_)) {
                bgfx::destroy(u_collision_obb_axis_x_);
                u_collision_obb_axis_x_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(u_collision_obb_axis_y_)) {
                bgfx::destroy(u_collision_obb_axis_y_);
                u_collision_obb_axis_y_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(u_collision_obb_axis_z_)) {
                bgfx::destroy(u_collision_obb_axis_z_);
                u_collision_obb_axis_z_ = BGFX_INVALID_HANDLE;
            }
        }

        inline void destroyPrograms() {
            if (bgfx::isValid(combine_program_)) {
                bgfx::destroy(combine_program_);
                combine_program_ = BGFX_INVALID_HANDLE;
            }
        }

        inline void destroyFrameBuffer() {
            if (bgfx::isValid(gbuffer_)) {
                bgfx::destroy(gbuffer_);
                gbuffer_ = BGFX_INVALID_HANDLE;
            }
            albedo_texture_ = BGFX_INVALID_HANDLE;
            normal_texture_ = BGFX_INVALID_HANDLE;
            world_pos_texture_ = BGFX_INVALID_HANDLE;
            depth_texture_ = BGFX_INVALID_HANDLE;
        }

        inline float extractMaxScale(const mat4f& matrix) const {
            const vec3f axis_x{matrix[0][0], matrix[0][1], matrix[0][2]};
            const vec3f axis_y{matrix[1][0], matrix[1][1], matrix[1][2]};
            const vec3f axis_z{matrix[2][0], matrix[2][1], matrix[2][2]};
            return std::max({deferred_detail::safeLength(axis_x),
                             deferred_detail::safeLength(axis_y),
                             deferred_detail::safeLength(axis_z),
                             1e-6f});
        }

        inline void appendSphere(const Sphere& sphere, const mat4f& world_matrix) {
            if (sphere_count_ >= kMaxCollisionShapes) {
                return;
            }
            const vec3f center =
                sinriv::kigstudio::voxel::collision::transformPoint(world_matrix, sphere.center);
            sphere_data_[sphere_count_] = {center.x, center.y, center.z,
                                           sphere.radius * extractMaxScale(world_matrix)};
            ++sphere_count_;
            collision_counts_[0] = static_cast<float>(sphere_count_);
        }

        inline void appendCylinder(const Cylinder& cylinder, const mat4f& world_matrix) {
            if (cylinder_count_ >= kMaxCollisionShapes) {
                return;
            }
            const vec3f start =
                sinriv::kigstudio::voxel::collision::transformPoint(world_matrix, cylinder.start);
            const vec3f end =
                sinriv::kigstudio::voxel::collision::transformPoint(world_matrix, cylinder.end);
            cylinder_start_radius_[cylinder_count_] = {
                start.x,
                start.y,
                start.z,
                cylinder.radius * extractMaxScale(world_matrix),
            };
            cylinder_end_[cylinder_count_] = {end.x, end.y, end.z, 0.0f};
            ++cylinder_count_;
            collision_counts_[1] = static_cast<float>(cylinder_count_);
        }

        inline void appendCapsule(const Capsule& capsule, const mat4f& world_matrix) {
            if (capsule_count_ >= kMaxCollisionShapes) {
                return;
            }
            const vec3f start =
                sinriv::kigstudio::voxel::collision::transformPoint(world_matrix, capsule.start);
            const vec3f end =
                sinriv::kigstudio::voxel::collision::transformPoint(world_matrix, capsule.end);
            capsule_start_radius_[capsule_count_] = {
                start.x,
                start.y,
                start.z,
                capsule.radius * extractMaxScale(world_matrix),
            };
            capsule_end_[capsule_count_] = {end.x, end.y, end.z, 0.0f};
            ++capsule_count_;
            collision_counts_[2] = static_cast<float>(capsule_count_);
        }

        inline void appendOBB(const OBB& obb, const mat4f& world_matrix) {
            if (obb_count_ >= kMaxCollisionShapes) {
                return;
            }

            const vec3f center =
                sinriv::kigstudio::voxel::collision::transformPoint(world_matrix, obb.center);
            const vec3f axis_x =
                deferred_detail::transformVector(world_matrix, obb.axis_x * obb.half_extent.x);
            const vec3f axis_y =
                deferred_detail::transformVector(world_matrix, obb.axis_y * obb.half_extent.y);
            const vec3f axis_z =
                deferred_detail::transformVector(world_matrix, obb.axis_z * obb.half_extent.z);

            obb_center_[obb_count_] = {center.x, center.y, center.z, 0.0f};
            obb_axis_x_[obb_count_] = {axis_x.x, axis_x.y, axis_x.z, 0.0f};
            obb_axis_y_[obb_count_] = {axis_y.x, axis_y.y, axis_y.z, 0.0f};
            obb_axis_z_[obb_count_] = {axis_z.x, axis_z.y, axis_z.z, 0.0f};
            ++obb_count_;
            collision_counts_[3] = static_cast<float>(obb_count_);
        }

        inline bool ensureFrameBuffer() {
            if (!screen_layout_initialized_) {
                deferred_detail::ScreenVertex::init(screen_layout_);
                screen_layout_initialized_ = true;
                bx::mtxIdentity(identity_mtx_);
            }

            if (bgfx::isValid(gbuffer_) && width_ == fb_width_ && height_ == fb_height_) {
                return true;
            }

            destroyFrameBuffer();

            constexpr uint64_t kSamplerFlags =
                BGFX_TEXTURE_RT | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP;
            albedo_texture_ = bgfx::createTexture2D(
                width_, height_, false, 1, bgfx::TextureFormat::BGRA8, kSamplerFlags);
            normal_texture_ = bgfx::createTexture2D(
                width_, height_, false, 1, bgfx::TextureFormat::BGRA8, kSamplerFlags);
            world_pos_texture_ = bgfx::createTexture2D(
                width_, height_, false, 1, bgfx::TextureFormat::RGBA32F, kSamplerFlags);
            depth_texture_ = bgfx::createTexture2D(
                width_, height_, false, 1, bgfx::TextureFormat::D32F, kSamplerFlags);

            bgfx::TextureHandle attachments[] = {
                albedo_texture_,
                normal_texture_,
                world_pos_texture_,
                depth_texture_,
            };
            gbuffer_ = bgfx::createFrameBuffer(
                static_cast<uint8_t>(BX_COUNTOF(attachments)), attachments, true);
            fb_width_ = width_;
            fb_height_ = height_;
            return bgfx::isValid(gbuffer_);
        }

        inline bool ensureProgram() {
            if (bgfx::isValid(combine_program_)) {
                return true;
            }

            if (!bgfx::isValid(s_albedo_)) {
                s_albedo_ = bgfx::createUniform("s_albedo", bgfx::UniformType::Sampler);
            }
            if (!bgfx::isValid(s_normal_)) {
                s_normal_ = bgfx::createUniform("s_normal", bgfx::UniformType::Sampler);
            }
            if (!bgfx::isValid(s_world_pos_)) {
                s_world_pos_ = bgfx::createUniform("s_worldPos", bgfx::UniformType::Sampler);
            }
            if (!bgfx::isValid(u_light_dir_)) {
                u_light_dir_ = bgfx::createUniform("u_lightDir", bgfx::UniformType::Vec4);
            }
            if (!bgfx::isValid(u_collision_counts_)) {
                u_collision_counts_ =
                    bgfx::createUniform("u_collisionCounts", bgfx::UniformType::Vec4);
            }
            if (!bgfx::isValid(u_collision_spheres_)) {
                u_collision_spheres_ = bgfx::createUniform(
                    "u_collisionSpheres", bgfx::UniformType::Vec4,
                    static_cast<uint16_t>(kMaxCollisionShapes));
            }
            if (!bgfx::isValid(u_collision_cylinders_a_)) {
                u_collision_cylinders_a_ = bgfx::createUniform(
                    "u_collisionCylindersA", bgfx::UniformType::Vec4,
                    static_cast<uint16_t>(kMaxCollisionShapes));
            }
            if (!bgfx::isValid(u_collision_cylinders_b_)) {
                u_collision_cylinders_b_ = bgfx::createUniform(
                    "u_collisionCylindersB", bgfx::UniformType::Vec4,
                    static_cast<uint16_t>(kMaxCollisionShapes));
            }
            if (!bgfx::isValid(u_collision_capsules_a_)) {
                u_collision_capsules_a_ = bgfx::createUniform(
                    "u_collisionCapsulesA", bgfx::UniformType::Vec4,
                    static_cast<uint16_t>(kMaxCollisionShapes));
            }
            if (!bgfx::isValid(u_collision_capsules_b_)) {
                u_collision_capsules_b_ = bgfx::createUniform(
                    "u_collisionCapsulesB", bgfx::UniformType::Vec4,
                    static_cast<uint16_t>(kMaxCollisionShapes));
            }
            if (!bgfx::isValid(u_collision_obb_center_)) {
                u_collision_obb_center_ = bgfx::createUniform(
                    "u_collisionObbCenter", bgfx::UniformType::Vec4,
                    static_cast<uint16_t>(kMaxCollisionShapes));
            }
            if (!bgfx::isValid(u_collision_obb_axis_x_)) {
                u_collision_obb_axis_x_ = bgfx::createUniform(
                    "u_collisionObbAxisX", bgfx::UniformType::Vec4,
                    static_cast<uint16_t>(kMaxCollisionShapes));
            }
            if (!bgfx::isValid(u_collision_obb_axis_y_)) {
                u_collision_obb_axis_y_ = bgfx::createUniform(
                    "u_collisionObbAxisY", bgfx::UniformType::Vec4,
                    static_cast<uint16_t>(kMaxCollisionShapes));
            }
            if (!bgfx::isValid(u_collision_obb_axis_z_)) {
                u_collision_obb_axis_z_ = bgfx::createUniform(
                    "u_collisionObbAxisZ", bgfx::UniformType::Vec4,
                    static_cast<uint16_t>(kMaxCollisionShapes));
            }

            bgfx::ShaderHandle vs =
                deferred_detail::loadShader(shader_dir_ + "vs_screen_quad.bin");
            bgfx::ShaderHandle fs =
                deferred_detail::loadShader(shader_dir_ + "fs_deferred_combine.bin");
            if (!bgfx::isValid(vs) || !bgfx::isValid(fs)) {
                if (bgfx::isValid(vs)) {
                    bgfx::destroy(vs);
                }
                if (bgfx::isValid(fs)) {
                    bgfx::destroy(fs);
                }
                std::cerr << "RenderDeferred shader load failed from " << shader_dir_
                          << std::endl;
                return false;
            }

            combine_program_ = bgfx::createProgram(vs, fs, true);
            return bgfx::isValid(combine_program_);
        }

        bgfx::ViewId gbuffer_view_id_ = 0;
        bgfx::ViewId lighting_view_id_ = 1;
        std::string shader_dir_ = "../../shader/base/";
        uint16_t width_ = 1;
        uint16_t height_ = 1;
        uint16_t fb_width_ = 0;
        uint16_t fb_height_ = 0;
        bgfx::VertexLayout screen_layout_{};
        bool screen_layout_initialized_ = false;
        bgfx::FrameBufferHandle gbuffer_ = BGFX_INVALID_HANDLE;
        bgfx::TextureHandle albedo_texture_ = BGFX_INVALID_HANDLE;
        bgfx::TextureHandle normal_texture_ = BGFX_INVALID_HANDLE;
        bgfx::TextureHandle world_pos_texture_ = BGFX_INVALID_HANDLE;
        bgfx::TextureHandle depth_texture_ = BGFX_INVALID_HANDLE;
        bgfx::ProgramHandle combine_program_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle s_albedo_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle s_normal_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle s_world_pos_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_light_dir_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_collision_counts_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_collision_spheres_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_collision_cylinders_a_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_collision_cylinders_b_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_collision_capsules_a_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_collision_capsules_b_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_collision_obb_center_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_collision_obb_axis_x_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_collision_obb_axis_y_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_collision_obb_axis_z_ = BGFX_INVALID_HANDLE;
        std::array<float, 4> light_dir_ = {0.3f, 0.5f, 0.8f, 0.0f};
        std::array<float, 4> collision_counts_ = {0.0f, 0.0f, 0.0f, 0.0f};
        std::array<std::array<float, 4>, kMaxCollisionShapes> sphere_data_{};
        std::array<std::array<float, 4>, kMaxCollisionShapes> cylinder_start_radius_{};
        std::array<std::array<float, 4>, kMaxCollisionShapes> cylinder_end_{};
        std::array<std::array<float, 4>, kMaxCollisionShapes> capsule_start_radius_{};
        std::array<std::array<float, 4>, kMaxCollisionShapes> capsule_end_{};
        std::array<std::array<float, 4>, kMaxCollisionShapes> obb_center_{};
        std::array<std::array<float, 4>, kMaxCollisionShapes> obb_axis_x_{};
        std::array<std::array<float, 4>, kMaxCollisionShapes> obb_axis_y_{};
        std::array<std::array<float, 4>, kMaxCollisionShapes> obb_axis_z_{};
        std::size_t sphere_count_ = 0;
        std::size_t cylinder_count_ = 0;
        std::size_t capsule_count_ = 0;
        std::size_t obb_count_ = 0;
        float identity_mtx_[16]{};
    };
}
