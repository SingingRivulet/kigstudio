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

        bgfx::ShaderHandle loadShader(const std::string& path);

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
        using Box = sinriv::kigstudio::voxel::collision::Box;
        using vec3f = deferred_detail::vec3f;
        using mat4f = deferred_detail::mat4f;

        explicit RenderDeferred(bgfx::ViewId gbuffer_view_id = 0,
                                bgfx::ViewId lighting_view_id = 1,
                                bgfx::ViewId collision_view_id = 2,
                                std::string shader_dir = "../../shader/base/")
            : gbuffer_view_id_(gbuffer_view_id),
              lighting_view_id_(lighting_view_id),
              collision_view_id_(collision_view_id),
              shader_dir_(std::move(shader_dir)) {}

        ~RenderDeferred() { release(); }

        inline bgfx::ViewId getGBufferViewId() const { return gbuffer_view_id_; }
        inline bgfx::ViewId getLightingViewId() const { return lighting_view_id_; }
        inline bgfx::ViewId getCollisionViewId() const { return collision_view_id_; }

        inline void setViewIds(bgfx::ViewId gbuffer_view_id,
                               bgfx::ViewId lighting_view_id,
                               bgfx::ViewId collision_view_id) {
            gbuffer_view_id_ = gbuffer_view_id;
            lighting_view_id_ = lighting_view_id;
            collision_view_id_ = collision_view_id;
        }

        inline void setShaderDirectory(const std::string& shader_dir) {
            shader_dir_ = shader_dir;
            destroyPrograms();
        }

        inline void setViewportSize(uint16_t width, uint16_t height) {
            width_ = std::max<uint16_t>(width, 1);
            height_ = std::max<uint16_t>(height, 1);
        }

        inline void setSpaceDivVisible(bool visible) {
            space_div_mix[0] = visible ? 1.0f : 0.0f;
        }
        inline void setSpaceDiv(float A, float B, float C, float D) {
            space_div[0] = A;
            space_div[1] = B;
            space_div[2] = C;
            space_div[3] = D;
        }

        inline void setLightDirection(float x, float y, float z) {
            light_dir_[0] = x;
            light_dir_[1] = y;
            light_dir_[2] = z;
        }

        inline void clearCollisionTint() {
            collision_items_.clear();
        }

        void setCollisionGroup(const CollisionGroup& group);
        void prepareFrame();
        void render();
        void release();

        struct CollisionItem {
            uint32_t type; // 0=sphere, 1=cylinder, 2=capsule, 3=box
            std::array<std::array<float, 4>, 4> data{};
            vec3f half_extent = {};
        };

        void destroyCollisionUniforms();
        void destroyPrograms();
        void destroyFrameBuffer();
        float extractMaxScale(const mat4f& matrix) const;

        void appendSphere(const Sphere& sphere, const mat4f& world_matrix);
        void appendCylinder(const Cylinder& cylinder, const mat4f& world_matrix);
        void appendCapsule(const Capsule& capsule, const mat4f& world_matrix) ;
        void appendBox(const Box& box, const mat4f& world_matrix);
        bool ensureFrameBuffer();
        bool ensureProgram();

        bgfx::ViewId gbuffer_view_id_ = 0;
        bgfx::ViewId collision_view_id_ = 1;
        bgfx::ViewId lighting_view_id_ = 2;
        std::string shader_dir_ = "../../shader/base/";
        uint16_t width_ = 1;
        uint16_t height_ = 1;
        uint16_t fb_width_ = 0;
        uint16_t fb_height_ = 0;
        bgfx::VertexLayout screen_layout_{};
        bool screen_layout_initialized_ = false;
        bgfx::FrameBufferHandle gbuffer_ = BGFX_INVALID_HANDLE;
        bgfx::FrameBufferHandle collision_fb_ = BGFX_INVALID_HANDLE;
        bgfx::TextureHandle albedo_texture_ = BGFX_INVALID_HANDLE;
        bgfx::TextureHandle collision_body_texture_ = BGFX_INVALID_HANDLE;
        bgfx::TextureHandle normal_texture_ = BGFX_INVALID_HANDLE;
        bgfx::TextureHandle world_pos_texture_ = BGFX_INVALID_HANDLE;
        bgfx::TextureHandle readback_ = BGFX_INVALID_HANDLE;
        bgfx::TextureHandle depth_texture_ = BGFX_INVALID_HANDLE;
        bgfx::ProgramHandle combine_program_ = BGFX_INVALID_HANDLE;
        bgfx::ProgramHandle collision_program_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle s_albedo_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle s_normal_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle s_world_pos_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle s_collision_status_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_light_dir_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_shape_type_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_shape_data_0_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_shape_data_1_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_shape_data_2_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_shape_data_3_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_space_div_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_space_div_mix_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_mouse_pos_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_mouse_highlight_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_pos_hightlight_counts_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_pos_hightlight_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_pos_hightlight_color_ = BGFX_INVALID_HANDLE;
        std::array<float, 4> space_div = {1.0f, 0.0f, 0.0f, 0.0f};
        std::array<float, 4> space_div_mix = {1.0f, 0.0f, 0.0f, 0.0f};
        std::array<float, 4> light_dir_ = {0.3f, 0.5f, 0.8f, 0.0f};
        std::array<float, 4> mouse_pos_ = {0.f, 0.f, 0.f, 0.f};
        std::array<float, 4> mouse_highlight_ = {0.f, 0.f, 0.f, 0.f};
        std::array<float, 4> mouse_ori_ = {0.f, 0.f, 0.f, 0.f};
        std::array<float, 4> mouse_dir_ = {0.f, 0.f, 0.f, 0.f};
        std::array<float, 2> screen_mouse_pos_ = {0.f, 0.f};
        
        int pos_hightlight_counts = 0;
        std::array<float, 4> pos_hightlight_counts_gpu_ = {0.0f, 0.0f, 0.0f, 0.0f};
        std::array<std::array<float, 4>, 16> pos_hightlight_{};
        std::array<std::array<float, 4>, 16> pos_hightlight_color_{};

        std::vector<CollisionItem> collision_items_;
        float identity_mtx_[16]{};
        float readback_buffer[2 * 2 * 4]; // 用于从GPU读取碰撞信息
    };
}
