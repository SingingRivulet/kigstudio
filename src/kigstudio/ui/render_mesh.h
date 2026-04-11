#pragma once
#include <bgfx/bgfx.h>
#include <bgfx/platform.h>

#include <cstdio>
#include <iostream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

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
        explicit RenderMesh(bgfx::ViewId view_id = 0,
                            std::string shader_dir = "../../shader/base/")
            : view_id_(view_id), shader_dir_(std::move(shader_dir)) {}

        ~RenderMesh() {
            release();
        }

        inline void setViewId(bgfx::ViewId view_id) { view_id_ = view_id; }
        inline bgfx::ViewId getViewId() const { return view_id_; }

        inline void setShaderDirectory(const std::string& shader_dir) {
            shader_dir_ = shader_dir;
            if (bgfx::isValid(program_)) {
                bgfx::destroy(program_);
                program_ = BGFX_INVALID_HANDLE;
            }
        }

        template <class T>
        void loadGeometry(T&& geometry) {
            if (!layout_initialized_) {
                mesh_detail::PosNormalVertex::init(layout_);
                layout_initialized_ = true;
            }

            std::vector<mesh_detail::PosNormalVertex> vertices;
            std::vector<uint32_t> indices;

            for (auto [tri, n] : geometry) {
                const uint32_t base = static_cast<uint32_t>(vertices.size());
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
            if (bgfx::isValid(program_)) {
                bgfx::destroy(program_);
                program_ = BGFX_INVALID_HANDLE;
            }
        }

        inline void clear() { mesh_.destroy(); }

        void render(const float* transform) {
            if (empty() || !ensureProgram()) {
                return;
            }

            bgfx::setTransform(transform);
            bgfx::setVertexBuffer(0, mesh_.vbh);
            bgfx::setIndexBuffer(mesh_.ibh);
            bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                           BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS |
                           BGFX_STATE_CULL_CCW | BGFX_STATE_MSAA);
            bgfx::submit(view_id_, program_);
        }

       private:
        inline bool ensureProgram() {
            if (bgfx::isValid(program_)) {
                return true;
            }
            if (!layout_initialized_) {
                mesh_detail::PosNormalVertex::init(layout_);
                layout_initialized_ = true;
            }

            bgfx::ShaderHandle vs =
                mesh_detail::loadShader(shader_dir_ + "vs_mesh_base.bin");
            bgfx::ShaderHandle fs =
                mesh_detail::loadShader(shader_dir_ + "fs_mesh_base.bin");
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

            program_ = bgfx::createProgram(vs, fs, true);
            return bgfx::isValid(program_);
        }

        bgfx::ViewId view_id_ = 0;
        std::string shader_dir_ = "../../shader/base/";
        bgfx::VertexLayout layout_{};
        bool layout_initialized_ = false;
        mesh_detail::MeshHandle mesh_{};
        bgfx::ProgramHandle program_ = BGFX_INVALID_HANDLE;
    };
}
