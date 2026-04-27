#include "render_deferred.h"

namespace sinriv::ui::render {
bgfx::ShaderHandle deferred_detail::loadShader(const std::string& path) {
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

void RenderDeferred::setCollisionGroup(const CollisionGroup& group) {
    clearCollisionTint();

    const mat4f group_matrix = group.transform.getRenderMatrix();
    for (const auto& geometry : group.geometries()) {
        const mat4f local_matrix =
            geometry.transform.getRenderMatrix() * group_matrix;
        std::visit(
            [&](const auto& shape) {
                using ShapeType = std::decay_t<decltype(shape)>;
                if constexpr (std::is_same_v<ShapeType, Sphere>) {
                    appendSphere(shape, local_matrix);
                } else if constexpr (std::is_same_v<ShapeType, Cylinder>) {
                    appendCylinder(shape, local_matrix);
                } else if constexpr (std::is_same_v<ShapeType, Capsule>) {
                    appendCapsule(shape, local_matrix);
                } else if constexpr (std::is_same_v<ShapeType, Box>) {
                    appendBox(shape, local_matrix);
                }
            },
            geometry.geometry);
    }
}

void RenderDeferred::prepareFrame() {
    if (!ensureFrameBuffer()) {
        return;
    }

    bgfx::setViewName(gbuffer_view_id_, "GBuffer");
    bgfx::setViewFrameBuffer(gbuffer_view_id_, gbuffer_);
    bgfx::setViewRect(gbuffer_view_id_, 0, 0, width_, height_);
    bgfx::setViewClear(gbuffer_view_id_, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                       0x00000000, 1.0f, 0);
    bgfx::touch(gbuffer_view_id_);

    float view[16];
    float proj[16];
    bx::mtxIdentity(view);
    bx::mtxOrtho(proj, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 100.0f, 0.0f,
                 bgfx::getCaps()->homogeneousDepth);

    bgfx::setViewName(collision_view_id_, "CollisionMask");
    bgfx::setViewFrameBuffer(collision_view_id_, collision_fb_);
    bgfx::setViewRect(collision_view_id_, 0, 0, width_, height_);
    bgfx::setViewTransform(collision_view_id_, view, proj);
    bgfx::setViewClear(collision_view_id_, BGFX_CLEAR_COLOR,
                       0x00000000);  // mask = 0
    bgfx::touch(collision_view_id_);

    bgfx::setViewName(lighting_view_id_, "DeferredLighting");
    bgfx::setViewFrameBuffer(lighting_view_id_, BGFX_INVALID_HANDLE);
    bgfx::setViewRect(lighting_view_id_, 0, 0, width_, height_);
    bgfx::setViewTransform(lighting_view_id_, view, proj);
    bgfx::setViewClear(lighting_view_id_, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                       0x303030ff, 1.0f, 0);
    bgfx::touch(lighting_view_id_);
}

void RenderDeferred::render() {
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

    // ===== Collision Pass =====
    for (const auto& item : collision_items_) {
        bgfx::setTransform(identity_mtx_);
        bgfx::setVertexBuffer(0, &tvb);
        bgfx::setIndexBuffer(&tib);
        bgfx::setTexture(0, s_world_pos_, world_pos_texture_);

        float type_vec[4] = {static_cast<float>(item.type), 0.0f, 0.0f, 0.0f};
        if (item.type == 3) {
            type_vec[1] = item.half_extent.x;
            type_vec[2] = item.half_extent.y;
            type_vec[3] = item.half_extent.z;
        }
        bgfx::setUniform(u_shape_type_, type_vec);
        bgfx::setUniform(u_shape_data_0_, item.data[0].data());
        if (item.type >= 1) {
            bgfx::setUniform(u_shape_data_1_, item.data[1].data());
        }
        if (item.type == 3) {
            bgfx::setUniform(u_shape_data_2_, item.data[2].data());
            bgfx::setUniform(u_shape_data_3_, item.data[3].data());
        }

        bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                       BGFX_STATE_MSAA | BGFX_STATE_BLEND_ADD);
        bgfx::submit(collision_view_id_, collision_program_);
    }

    // ===== Lighting Pass =====
    bgfx::setTransform(identity_mtx_);
    bgfx::setVertexBuffer(0, &tvb);
    bgfx::setIndexBuffer(&tib);
    bgfx::setTexture(0, s_albedo_, albedo_texture_);
    bgfx::setTexture(1, s_normal_, normal_texture_);
    bgfx::setTexture(2, s_world_pos_, world_pos_texture_);
    bgfx::setTexture(3, s_collision_status_, collision_body_texture_);
    bgfx::setUniform(u_light_dir_, light_dir_.data());
    bgfx::setUniform(u_space_div_, space_div.data());
    bgfx::setUniform(u_space_div_mix_, space_div_mix.data());
    bgfx::setUniform(u_mouse_pos_, mouse_pos_.data());
    bgfx::setUniform(u_mouse_highlight_, mouse_highlight_.data());
    bgfx::setUniform(u_pos_hightlight_counts_,
                     pos_hightlight_counts_gpu_.data());
    if (pos_hightlight_counts > 0) {
        bgfx::setUniform(u_pos_hightlight_, pos_hightlight_.data(),
                         static_cast<uint16_t>(pos_hightlight_counts));
        bgfx::setUniform(u_pos_hightlight_color_, pos_hightlight_color_.data(),
                         static_cast<uint16_t>(pos_hightlight_counts));
    }

    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_MSAA);
    bgfx::submit(lighting_view_id_, combine_program_);
    auto mouse_x = screen_mouse_pos_[0];
    auto mouse_y = screen_mouse_pos_[1];
    if (mouse_x >= 0 && mouse_x < width_ - 2 && mouse_y >= 0 &&
        mouse_y < height_ - 2) {
        bgfx::blit(lighting_view_id_, readback_, 0, 0, world_pos_texture_,
                   mouse_x, mouse_y, 2, 2);
        bgfx::readTexture(readback_, readback_buffer);
        if (readback_buffer[3] > 0.5f) {
            mouse_highlight_[0] = 1.0;
            mouse_highlight_[1] = 1.0;
            mouse_highlight_[2] = 1.0;
            mouse_highlight_[3] = 1.0;

            mouse_pos_[0] = readback_buffer[0];
            mouse_pos_[1] = readback_buffer[1];
            mouse_pos_[2] = readback_buffer[2];
        } else {
            mouse_highlight_[0] = 0.0;
            mouse_highlight_[1] = 0.0;
            mouse_highlight_[2] = 0.0;
            mouse_highlight_[3] = 0.0;
        }
    }
}

void RenderDeferred::release() {
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
    if (bgfx::isValid(s_collision_status_)) {
        bgfx::destroy(s_collision_status_);
        s_collision_status_ = BGFX_INVALID_HANDLE;
    }
    destroyCollisionUniforms();
}

void RenderDeferred::destroyCollisionUniforms() {
    if (bgfx::isValid(u_shape_type_)) {
        bgfx::destroy(u_shape_type_);
        u_shape_type_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(u_shape_data_0_)) {
        bgfx::destroy(u_shape_data_0_);
        u_shape_data_0_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(u_shape_data_1_)) {
        bgfx::destroy(u_shape_data_1_);
        u_shape_data_1_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(u_shape_data_2_)) {
        bgfx::destroy(u_shape_data_2_);
        u_shape_data_2_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(u_shape_data_3_)) {
        bgfx::destroy(u_shape_data_3_);
        u_shape_data_3_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(u_space_div_)) {
        bgfx::destroy(u_space_div_);
        u_space_div_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(u_space_div_mix_)) {
        bgfx::destroy(u_space_div_mix_);
        u_space_div_mix_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(u_mouse_pos_)) {
        bgfx::destroy(u_mouse_pos_);
        u_mouse_pos_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(u_mouse_highlight_)) {
        bgfx::destroy(u_mouse_highlight_);
        u_mouse_highlight_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(u_pos_hightlight_counts_)) {
        bgfx::destroy(u_pos_hightlight_counts_);
        u_pos_hightlight_counts_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(u_pos_hightlight_)) {
        bgfx::destroy(u_pos_hightlight_);
        u_pos_hightlight_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(u_pos_hightlight_color_)) {
        bgfx::destroy(u_pos_hightlight_color_);
        u_pos_hightlight_color_ = BGFX_INVALID_HANDLE;
    }
}

void RenderDeferred::destroyPrograms() {
    if (bgfx::isValid(combine_program_)) {
        bgfx::destroy(combine_program_);
        combine_program_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(collision_program_)) {
        bgfx::destroy(collision_program_);
        collision_program_ = BGFX_INVALID_HANDLE;
    }
}

void RenderDeferred::destroyFrameBuffer() {
    if (bgfx::isValid(gbuffer_)) {
        bgfx::destroy(gbuffer_);
        gbuffer_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(collision_fb_)) {
        bgfx::destroy(collision_fb_);
        collision_fb_ = BGFX_INVALID_HANDLE;
    }
    albedo_texture_ = BGFX_INVALID_HANDLE;
    collision_body_texture_ = BGFX_INVALID_HANDLE;
    normal_texture_ = BGFX_INVALID_HANDLE;
    world_pos_texture_ = BGFX_INVALID_HANDLE;
    depth_texture_ = BGFX_INVALID_HANDLE;
}

float RenderDeferred::extractMaxScale(const mat4f& matrix) const {
    const vec3f axis_x{matrix[0][0], matrix[0][1], matrix[0][2]};
    const vec3f axis_y{matrix[1][0], matrix[1][1], matrix[1][2]};
    const vec3f axis_z{matrix[2][0], matrix[2][1], matrix[2][2]};
    return std::max({deferred_detail::safeLength(axis_x),
                     deferred_detail::safeLength(axis_y),
                     deferred_detail::safeLength(axis_z), 1e-6f});
}

void RenderDeferred::appendSphere(const Sphere& sphere,
                                  const mat4f& world_matrix) {
    const vec3f center = sinriv::kigstudio::voxel::collision::transformPoint(
        world_matrix, sphere.center);
    CollisionItem item;
    item.type = 0;
    item.data[0] = {center.x, center.y, center.z,
                    sphere.radius * extractMaxScale(world_matrix)};
    collision_items_.push_back(item);
}

void RenderDeferred::appendCylinder(const Cylinder& cylinder,
                                    const mat4f& world_matrix) {
    const vec3f start = sinriv::kigstudio::voxel::collision::transformPoint(
        world_matrix, cylinder.start);
    const vec3f end = sinriv::kigstudio::voxel::collision::transformPoint(
        world_matrix, cylinder.end);
    CollisionItem item;
    item.type = 1;
    item.data[0] = {
        start.x,
        start.y,
        start.z,
        cylinder.radius * extractMaxScale(world_matrix),
    };
    item.data[1] = {end.x, end.y, end.z, 0.0f};
    collision_items_.push_back(item);
}

void RenderDeferred::appendCapsule(const Capsule& capsule,
                                   const mat4f& world_matrix) {
    const vec3f start = sinriv::kigstudio::voxel::collision::transformPoint(
        world_matrix, capsule.start);
    const vec3f end = sinriv::kigstudio::voxel::collision::transformPoint(
        world_matrix, capsule.end);
    CollisionItem item;
    item.type = 2;
    item.data[0] = {
        start.x,
        start.y,
        start.z,
        capsule.radius * extractMaxScale(world_matrix),
    };
    item.data[1] = {end.x, end.y, end.z, 0.0f};
    collision_items_.push_back(item);
}

void RenderDeferred::appendBox(const Box& box,
                                      const mat4f& world_matrix) {
    if (!sinriv::kigstudio::voxel::collision::isAffineInvertible(
            world_matrix)) {
        return;
    }
    mat4f inv_matrix = world_matrix;
    inv_matrix.invert();

    CollisionItem item;
    item.type = 3;
    item.data[0] = {inv_matrix[0][0], inv_matrix[0][1], inv_matrix[0][2],
                    inv_matrix[0][3]};
    item.data[1] = {inv_matrix[1][0], inv_matrix[1][1], inv_matrix[1][2],
                    inv_matrix[1][3]};
    item.data[2] = {inv_matrix[2][0], inv_matrix[2][1], inv_matrix[2][2],
                    inv_matrix[2][3]};
    item.data[3] = {inv_matrix[3][0], inv_matrix[3][1], inv_matrix[3][2],
                    inv_matrix[3][3]};
    item.half_extent = box.half_extent;
    collision_items_.push_back(item);
}

bool RenderDeferred::ensureFrameBuffer() {
    if (!screen_layout_initialized_) {
        deferred_detail::ScreenVertex::init(screen_layout_);
        screen_layout_initialized_ = true;
        bx::mtxIdentity(identity_mtx_);
    }

    if (bgfx::isValid(gbuffer_) && bgfx::isValid(collision_fb_) &&
        width_ == fb_width_ && height_ == fb_height_) {
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
    readback_ =
        bgfx::createTexture2D(2, 2, false, 1, bgfx::TextureFormat::RGBA32F,
                              BGFX_TEXTURE_READ_BACK | BGFX_TEXTURE_BLIT_DST);
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

    collision_body_texture_ = bgfx::createTexture2D(
        width_, height_, false, 1, bgfx::TextureFormat::BGRA8, kSamplerFlags);
    bgfx::TextureHandle collision_attachment[] = {collision_body_texture_};
    collision_fb_ = bgfx::createFrameBuffer(1, collision_attachment, true);

    return bgfx::isValid(gbuffer_) && bgfx::isValid(collision_fb_);
}

bool RenderDeferred::ensureProgram() {
    if (bgfx::isValid(combine_program_) && bgfx::isValid(collision_program_)) {
        return true;
    }

    if (!bgfx::isValid(s_albedo_)) {
        s_albedo_ = bgfx::createUniform("s_albedo", bgfx::UniformType::Sampler);
    }
    if (!bgfx::isValid(s_normal_)) {
        s_normal_ = bgfx::createUniform("s_normal", bgfx::UniformType::Sampler);
    }
    if (!bgfx::isValid(s_world_pos_)) {
        s_world_pos_ =
            bgfx::createUniform("s_worldPos", bgfx::UniformType::Sampler);
    }
    if (!bgfx::isValid(s_collision_status_)) {
        s_collision_status_ =
            bgfx::createUniform("s_collision", bgfx::UniformType::Sampler);
    }
    if (!bgfx::isValid(u_light_dir_)) {
        u_light_dir_ =
            bgfx::createUniform("u_lightDir", bgfx::UniformType::Vec4);
    }
    if (!bgfx::isValid(u_shape_type_)) {
        u_shape_type_ =
            bgfx::createUniform("u_shapeType", bgfx::UniformType::Vec4);
    }
    if (!bgfx::isValid(u_shape_data_0_)) {
        u_shape_data_0_ =
            bgfx::createUniform("u_shapeData0", bgfx::UniformType::Vec4);
    }
    if (!bgfx::isValid(u_shape_data_1_)) {
        u_shape_data_1_ =
            bgfx::createUniform("u_shapeData1", bgfx::UniformType::Vec4);
    }
    if (!bgfx::isValid(u_shape_data_2_)) {
        u_shape_data_2_ =
            bgfx::createUniform("u_shapeData2", bgfx::UniformType::Vec4);
    }
    if (!bgfx::isValid(u_shape_data_3_)) {
        u_shape_data_3_ =
            bgfx::createUniform("u_shapeData3", bgfx::UniformType::Vec4);
    }
    if (!bgfx::isValid(u_space_div_)) {
        u_space_div_ =
            bgfx::createUniform("u_space_div", bgfx::UniformType::Vec4);
    }
    if (!bgfx::isValid(u_space_div_mix_)) {
        u_space_div_mix_ =
            bgfx::createUniform("u_space_div_mix", bgfx::UniformType::Vec4);
    }
    if (!bgfx::isValid(u_mouse_pos_)) {
        u_mouse_pos_ =
            bgfx::createUniform("u_mousePos", bgfx::UniformType::Vec4);
    }
    if (!bgfx::isValid(u_mouse_highlight_)) {
        u_mouse_highlight_ =
            bgfx::createUniform("u_mouseHighlight", bgfx::UniformType::Vec4);
    }
    if (!bgfx::isValid(u_pos_hightlight_counts_)) {
        u_pos_hightlight_counts_ = bgfx::createUniform(
            "u_pos_hightlight_counts", bgfx::UniformType::Vec4);
    }
    if (!bgfx::isValid(u_pos_hightlight_)) {
        u_pos_hightlight_ =
            bgfx::createUniform("u_pos_hightlight", bgfx::UniformType::Vec4,
                                static_cast<uint16_t>(16));
    }
    if (!bgfx::isValid(u_pos_hightlight_color_)) {
        u_pos_hightlight_color_ = bgfx::createUniform(
            "u_pos_hightlight_color", bgfx::UniformType::Vec4,
            static_cast<uint16_t>(16));
    }

    bgfx::ShaderHandle vs =
        deferred_detail::loadShader(shader_dir_ + "vs_screen_quad.bin");
    bgfx::ShaderHandle fs_combine =
        deferred_detail::loadShader(shader_dir_ + "fs_deferred_combine.bin");
    bgfx::ShaderHandle fs_collision =
        deferred_detail::loadShader(shader_dir_ + "fs_deferred_collision.bin");

    if (!bgfx::isValid(vs) || !bgfx::isValid(fs_combine) ||
        !bgfx::isValid(fs_collision)) {
        if (bgfx::isValid(vs)) {
            bgfx::destroy(vs);
        }
        if (bgfx::isValid(fs_combine)) {
            bgfx::destroy(fs_combine);
        }
        if (bgfx::isValid(fs_collision)) {
            bgfx::destroy(fs_collision);
        }
        std::cerr << "RenderDeferred shader load failed from " << shader_dir_
                  << std::endl;
        return false;
    }

    combine_program_ = bgfx::createProgram(vs, fs_combine, true);
    collision_program_ = bgfx::createProgram(vs, fs_collision, true);

    return bgfx::isValid(combine_program_) && bgfx::isValid(collision_program_);
}

}  // namespace sinriv::ui::render