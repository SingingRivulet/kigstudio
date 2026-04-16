#pragma once

#include <bgfx/bgfx.h>
#include <bx/math.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace sinriv::ui::render {
    namespace deferred_detail {
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
    }

    class RenderDeferred {
       public:
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
            bgfx::setUniform(u_light_dir_, light_dir_.data());
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
            if (bgfx::isValid(u_light_dir_)) {
                bgfx::destroy(u_light_dir_);
                u_light_dir_ = BGFX_INVALID_HANDLE;
            }
        }

       private:
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
            depth_texture_ = BGFX_INVALID_HANDLE;
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
            depth_texture_ = bgfx::createTexture2D(
                width_, height_, false, 1, bgfx::TextureFormat::D32F, kSamplerFlags);

            bgfx::TextureHandle attachments[] = {
                albedo_texture_,
                normal_texture_,
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
            if (!bgfx::isValid(u_light_dir_)) {
                u_light_dir_ = bgfx::createUniform("u_lightDir", bgfx::UniformType::Vec4);
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
        bgfx::TextureHandle depth_texture_ = BGFX_INVALID_HANDLE;
        bgfx::ProgramHandle combine_program_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle s_albedo_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle s_normal_ = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle u_light_dir_ = BGFX_INVALID_HANDLE;
        std::array<float, 4> light_dir_ = {0.3f, 0.5f, 0.8f, 0.0f};
        float identity_mtx_[16]{};
    };
}
