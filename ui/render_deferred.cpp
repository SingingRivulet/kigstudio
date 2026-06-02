#include "render_deferred.h"

namespace sinriv::ui::render {
namespace {
constexpr float kConeVolumeScale = 1000.0f;

template <class Vec3>
deferred_detail::VolumeVertex makeVolumeVertex(const Vec3& value) {
    return {value.x, value.y, value.z};
}

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;
};

template <class Vec3>
float dot3(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

template <class Vec3>
Vec3 cross3(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

template <class Vec3>
Vec3 normalize3(const Vec3& value, const Vec3& fallback) {
    const float len_sq = dot3(value, value);
    if (len_sq <= 1e-12f) {
        return fallback;
    }
    return value / std::sqrt(len_sq);
}

float cross2(const Vec2& a, const Vec2& b, const Vec2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

float polygonArea2(const std::vector<Vec2>& points,
                   const std::vector<size_t>& order) {
    float area = 0.0f;
    for (size_t i = 0; i < order.size(); ++i) {
        const Vec2& a = points[order[i]];
        const Vec2& b = points[order[(i + 1) % order.size()]];
        area += a.x * b.y - b.x * a.y;
    }
    return area;
}

bool pointInTriangle(const Vec2& p, const Vec2& a, const Vec2& b,
                     const Vec2& c) {
    constexpr float eps = 1e-6f;
    const float c0 = cross2(a, b, p);
    const float c1 = cross2(b, c, p);
    const float c2 = cross2(c, a, p);
    return c0 >= -eps && c1 >= -eps && c2 >= -eps;
}
}  // namespace

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

void RenderDeferred::setConcaveCone(Cone& cone) {
    clearCollisionTint();
    appendConcaveCone(cone);
}

void RenderDeferred::prepareFrame() {
    if (!ensureFrameBuffer()) {
        return;
    }

    float screen_view[16];
    float screen_proj[16];
    bx::mtxIdentity(screen_view);
    bx::mtxOrtho(screen_proj, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 100.0f, 0.0f,
                 bgfx::getCaps()->homogeneousDepth);

    bgfx::setViewName(gbuffer_view_id_, "GBuffer");
    bgfx::setViewFrameBuffer(gbuffer_view_id_, gbuffer_);
    bgfx::setViewRect(gbuffer_view_id_, 0, 0, width_, height_);
    bgfx::setViewClear(gbuffer_view_id_,
                       BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH | BGFX_CLEAR_STENCIL,
                       0x00000000, 1.0f, 0);
    bgfx::touch(gbuffer_view_id_);

    bgfx::setViewName(collision_view_id_, "CollisionVolume");
    bgfx::setViewFrameBuffer(collision_view_id_, collision_volume_fb_);
    bgfx::setViewRect(collision_view_id_, 0, 0, width_, height_);
    bgfx::setViewTransform(collision_view_id_, scene_view_, scene_proj_);
    bgfx::setViewClear(collision_view_id_, BGFX_CLEAR_COLOR | BGFX_CLEAR_STENCIL,
                       0x00000000);  // mask = 0
    bgfx::touch(collision_view_id_);

    bgfx::setViewName(collision_fill_view_id_, "CollisionMask");
    bgfx::setViewFrameBuffer(collision_fill_view_id_, collision_fb_);
    bgfx::setViewRect(collision_fill_view_id_, 0, 0, width_, height_);
    bgfx::setViewTransform(collision_fill_view_id_, screen_view, screen_proj);
    bgfx::setViewClear(collision_fill_view_id_, BGFX_CLEAR_COLOR, 0x00000000);
    bgfx::touch(collision_fill_view_id_);

    bgfx::setViewName(mesh_stencil_fill_view_id_, "MeshStencilMask");
    bgfx::setViewFrameBuffer(mesh_stencil_fill_view_id_, mesh_stencil_fb_);
    bgfx::setViewRect(mesh_stencil_fill_view_id_, 0, 0, width_, height_);
    bgfx::setViewTransform(mesh_stencil_fill_view_id_, screen_view, screen_proj);
    bgfx::setViewClear(mesh_stencil_fill_view_id_, BGFX_CLEAR_COLOR, 0x00000000);
    bgfx::touch(mesh_stencil_fill_view_id_);

    bgfx::setViewName(lighting_view_id_, "DeferredLighting");
    bgfx::setViewFrameBuffer(lighting_view_id_, BGFX_INVALID_HANDLE);
    bgfx::setViewRect(lighting_view_id_, 0, 0, width_, height_);
    bgfx::setViewTransform(lighting_view_id_, screen_view, screen_proj);
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
        if (item.type == 4) {
            if (!bgfx::isValid(volume_program_) || item.volume_vertices.empty()) {
                continue;
            }
            if (!volume_layout_initialized_) {
                deferred_detail::VolumeVertex::init(volume_layout_);
                volume_layout_initialized_ = true;
            }
            if (bgfx::getAvailTransientVertexBuffer(
                    static_cast<uint32_t>(item.volume_vertices.size()),
                    volume_layout_) < item.volume_vertices.size()) {
                continue;
            }

            bgfx::TransientVertexBuffer volume_tvb;
            bgfx::allocTransientVertexBuffer(
                &volume_tvb, static_cast<uint32_t>(item.volume_vertices.size()),
                volume_layout_);
            std::memcpy(volume_tvb.data, item.volume_vertices.data(),
                        item.volume_vertices.size() *
                            sizeof(deferred_detail::VolumeVertex));

            bgfx::setTransform(scene_model_mtx_);
            bgfx::setVertexBuffer(0, &volume_tvb);
            bgfx::setState(BGFX_STATE_DEPTH_TEST_LEQUAL | BGFX_STATE_MSAA);
            bgfx::setStencil(
                BGFX_STENCIL_TEST_ALWAYS | BGFX_STENCIL_FUNC_REF(1) |
                    BGFX_STENCIL_FUNC_RMASK(0xff) |
                    BGFX_STENCIL_OP_FAIL_S_KEEP |
                    BGFX_STENCIL_OP_FAIL_Z_INCR |
                    BGFX_STENCIL_OP_PASS_Z_KEEP,
                BGFX_STENCIL_TEST_ALWAYS | BGFX_STENCIL_FUNC_REF(1) |
                    BGFX_STENCIL_FUNC_RMASK(0xff) |
                    BGFX_STENCIL_OP_FAIL_S_KEEP |
                    BGFX_STENCIL_OP_FAIL_Z_DECR |
                    BGFX_STENCIL_OP_PASS_Z_KEEP);
            bgfx::submit(collision_view_id_, volume_program_);

            bgfx::setViewFrameBuffer(collision_fill_view_id_,
                                     collision_volume_fb_);
            bgfx::setTransform(identity_mtx_);
            bgfx::setVertexBuffer(0, &tvb);
            bgfx::setIndexBuffer(&tib);
            bgfx::setTexture(0, s_world_pos_, world_pos_texture_);
            float type_vec[4] = {5.0f, 0.0f, 0.0f, 0.0f};
            bgfx::setUniform(u_shape_type_, type_vec);
            bgfx::setUniform(u_shape_data_0_, item.data[0].data());
            bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                           BGFX_STATE_MSAA | BGFX_STATE_BLEND_ADD);
            bgfx::setStencil(
                BGFX_STENCIL_TEST_NOTEQUAL | BGFX_STENCIL_FUNC_REF(0) |
                    BGFX_STENCIL_FUNC_RMASK(0xff) |
                    BGFX_STENCIL_OP_FAIL_S_KEEP |
                    BGFX_STENCIL_OP_FAIL_Z_KEEP |
                    BGFX_STENCIL_OP_PASS_Z_KEEP);
            bgfx::submit(collision_fill_view_id_, collision_program_);
            continue;
        }

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
        bgfx::submit(collision_fill_view_id_, collision_program_);
    }

    // ===== Mesh Stencil Pass =====
    if (has_mesh_stencil_ && bgfx::isValid(mesh_stencil_program_)) {
        bgfx::setTransform(scene_model_mtx_);
        bgfx::setVertexBuffer(0, mesh_stencil_vbh_);
        bgfx::setIndexBuffer(mesh_stencil_ibh_);
        bgfx::setState(BGFX_STATE_MSAA);
        bgfx::setStencil(
            BGFX_STENCIL_TEST_ALWAYS | BGFX_STENCIL_FUNC_REF(1) |
                BGFX_STENCIL_FUNC_RMASK(0xff) |
                BGFX_STENCIL_OP_FAIL_S_KEEP |
                BGFX_STENCIL_OP_FAIL_Z_KEEP |
                BGFX_STENCIL_OP_PASS_Z_INCR,
            BGFX_STENCIL_TEST_ALWAYS | BGFX_STENCIL_FUNC_REF(1) |
                BGFX_STENCIL_FUNC_RMASK(0xff) |
                BGFX_STENCIL_OP_FAIL_S_KEEP |
                BGFX_STENCIL_OP_FAIL_Z_KEEP |
                BGFX_STENCIL_OP_PASS_Z_INCR);
        bgfx::submit(collision_view_id_, mesh_stencil_program_);

        bgfx::setTransform(identity_mtx_);
        bgfx::setVertexBuffer(0, &tvb);
        bgfx::setIndexBuffer(&tib);
        bgfx::setTexture(0, s_world_pos_, world_pos_texture_);
        float type_vec[4] = {5.0f, 0.0f, 0.0f, 0.0f};
        bgfx::setUniform(u_shape_type_, type_vec);
        bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                       BGFX_STATE_MSAA | BGFX_STATE_BLEND_ADD);
        bgfx::setStencil(
            BGFX_STENCIL_TEST_NOTEQUAL | BGFX_STENCIL_FUNC_REF(0) |
                BGFX_STENCIL_FUNC_RMASK(0xff) |
                BGFX_STENCIL_OP_FAIL_S_KEEP |
                BGFX_STENCIL_OP_FAIL_Z_KEEP |
                BGFX_STENCIL_OP_PASS_Z_KEEP);
        bgfx::submit(mesh_stencil_fill_view_id_, collision_program_);
    }

    // ===== Lighting Pass =====
    bgfx::setStencil(BGFX_STENCIL_NONE);
    bgfx::setTransform(identity_mtx_);
    bgfx::setVertexBuffer(0, &tvb);
    bgfx::setIndexBuffer(&tib);
    bgfx::setTexture(0, s_albedo_, albedo_texture_);
    bgfx::setTexture(1, s_normal_, normal_texture_);
    bgfx::setTexture(2, s_world_pos_, world_pos_texture_);
    bgfx::setTexture(3, s_collision_status_, collision_body_texture_);
    bgfx::setTexture(4, s_volume_, collision_volume_texture_);
    bgfx::setTexture(5, s_mesh_stencil_, mesh_stencil_body_texture_);
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
            mouse_highlight_[1] = mouse_highlight_range_;
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
    if (bgfx::isValid(s_volume_)) {
        bgfx::destroy(s_volume_);
        s_volume_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(s_mesh_stencil_)) {
        bgfx::destroy(s_mesh_stencil_);
        s_mesh_stencil_ = BGFX_INVALID_HANDLE;
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
    if (bgfx::isValid(volume_program_)) {
        bgfx::destroy(volume_program_);
        volume_program_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(mesh_stencil_program_)) {
        bgfx::destroy(mesh_stencil_program_);
        mesh_stencil_program_ = BGFX_INVALID_HANDLE;
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
    if (bgfx::isValid(collision_volume_fb_)) {
        bgfx::destroy(collision_volume_fb_);
        collision_volume_fb_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(mesh_stencil_fb_)) {
        bgfx::destroy(mesh_stencil_fb_);
        mesh_stencil_fb_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(albedo_texture_)) {
        bgfx::destroy(albedo_texture_);
    }
    if (bgfx::isValid(normal_texture_)) {
        bgfx::destroy(normal_texture_);
    }
    if (bgfx::isValid(world_pos_texture_)) {
        bgfx::destroy(world_pos_texture_);
    }
    if (bgfx::isValid(collision_body_texture_)) {
        bgfx::destroy(collision_body_texture_);
    }
    if (bgfx::isValid(collision_volume_texture_)) {
        bgfx::destroy(collision_volume_texture_);
    }
    if (bgfx::isValid(mesh_stencil_body_texture_)) {
        bgfx::destroy(mesh_stencil_body_texture_);
    }
    if (bgfx::isValid(depth_texture_)) {
        bgfx::destroy(depth_texture_);
    }
    albedo_texture_ = BGFX_INVALID_HANDLE;
    collision_body_texture_ = BGFX_INVALID_HANDLE;
    collision_volume_texture_ = BGFX_INVALID_HANDLE;
    mesh_stencil_body_texture_ = BGFX_INVALID_HANDLE;
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

void RenderDeferred::appendConcaveCone(Cone& cone) {
    if (cone.base_vertices.size() < 3) {
        return;
    }

    cone.triangulate();

    CollisionItem item;
    item.type = 4;
    sinriv::kigstudio::voxel::concave::vec3f apex(cone.apex.x, -cone.apex.y, cone.apex.z);
    item.data[0] = {apex.x, apex.y, apex.z, 1.0f};

    const auto& base = cone.base_vertices;
    const size_t count = base.size();
    std::vector<sinriv::kigstudio::voxel::concave::vec3f> far_vertices;
    far_vertices.reserve(count);
    for (const auto& vertex_local : base) {
        sinriv::kigstudio::voxel::concave::vec3f vertex(vertex_local.x, -vertex_local.y, vertex_local.z);
        auto far_vertex = apex + (vertex - apex) * kConeVolumeScale;
        far_vertex.y = -far_vertex.y;
        far_vertices.push_back(far_vertex);
    }

    item.volume_vertices.reserve(count * 6);
    auto pushTriangle = [&](const auto& a, const auto& b, const auto& c) {
        item.volume_vertices.push_back(makeVolumeVertex(a));
        item.volume_vertices.push_back(makeVolumeVertex(b));
        item.volume_vertices.push_back(makeVolumeVertex(c));
    };

    for (size_t i = 0; i < count; ++i) {
        const size_t next = (i + 1) % count;
        pushTriangle(cone.apex, far_vertices[i], far_vertices[next]);
    }

    for (const auto& tri : cone.base_triangles) {
        pushTriangle(far_vertices[tri[0]], far_vertices[tri[1]],
                     far_vertices[tri[2]]);
    }

    collision_items_.push_back(item);
}

bool RenderDeferred::ensureFrameBuffer() {
    if (!screen_layout_initialized_) {
        deferred_detail::ScreenVertex::init(screen_layout_);
        screen_layout_initialized_ = true;
        bx::mtxIdentity(identity_mtx_);
    }

    if (bgfx::isValid(gbuffer_) && bgfx::isValid(collision_fb_) &&
        bgfx::isValid(collision_volume_fb_) &&
        bgfx::isValid(mesh_stencil_fb_) &&
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
        width_, height_, false, 1, bgfx::TextureFormat::D24S8, kSamplerFlags);

    bgfx::TextureHandle attachments[] = {
        albedo_texture_,
        normal_texture_,
        world_pos_texture_,
        depth_texture_,
    };
    gbuffer_ = bgfx::createFrameBuffer(
        static_cast<uint8_t>(BX_COUNTOF(attachments)), attachments, false);
    fb_width_ = width_;
    fb_height_ = height_;

    collision_body_texture_ = bgfx::createTexture2D(
        width_, height_, false, 1, bgfx::TextureFormat::BGRA8, kSamplerFlags);
    bgfx::TextureHandle collision_attachment[] = {
        collision_body_texture_,
        depth_texture_,
    };
    collision_fb_ = bgfx::createFrameBuffer(
        static_cast<uint8_t>(BX_COUNTOF(collision_attachment)),
        collision_attachment, false);

    collision_volume_texture_ = bgfx::createTexture2D(
        width_, height_, false, 1, bgfx::TextureFormat::BGRA8, kSamplerFlags);
    bgfx::TextureHandle collision_volume_attachment[] = {
        collision_volume_texture_,
        depth_texture_,
    };
    collision_volume_fb_ = bgfx::createFrameBuffer(
        static_cast<uint8_t>(BX_COUNTOF(collision_volume_attachment)),
        collision_volume_attachment, false);

    mesh_stencil_body_texture_ = bgfx::createTexture2D(
        width_, height_, false, 1, bgfx::TextureFormat::BGRA8, kSamplerFlags);
    bgfx::TextureHandle mesh_stencil_attachment[] = {
        mesh_stencil_body_texture_,
        depth_texture_,
    };
    mesh_stencil_fb_ = bgfx::createFrameBuffer(
        static_cast<uint8_t>(BX_COUNTOF(mesh_stencil_attachment)),
        mesh_stencil_attachment, false);

    return bgfx::isValid(gbuffer_) && bgfx::isValid(collision_fb_) &&
           bgfx::isValid(collision_volume_fb_) &&
           bgfx::isValid(mesh_stencil_fb_);
}

bool RenderDeferred::ensureProgram() {
    if (bgfx::isValid(combine_program_) && bgfx::isValid(collision_program_) &&
        bgfx::isValid(volume_program_)) {
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
    if (!bgfx::isValid(s_volume_)) {
        s_volume_ =
            bgfx::createUniform("s_volume", bgfx::UniformType::Sampler);
    }
    if (!bgfx::isValid(s_mesh_stencil_)) {
        s_mesh_stencil_ =
            bgfx::createUniform("s_meshStencil", bgfx::UniformType::Sampler);
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
    bgfx::ShaderHandle vs_volume =
        deferred_detail::loadShader(shader_dir_ + "vs_volume_pos.bin");
    bgfx::ShaderHandle fs_volume =
        deferred_detail::loadShader(shader_dir_ + "fs_volume_mask.bin");
    bgfx::ShaderHandle vs_mesh =
        deferred_detail::loadShader(shader_dir_ + "vs_mesh_gbuffer.bin");
    bgfx::ShaderHandle fs_mesh_stencil =
        deferred_detail::loadShader(shader_dir_ + "fs_mesh_stencil.bin");

    if (!bgfx::isValid(vs) || !bgfx::isValid(fs_combine) ||
        !bgfx::isValid(fs_collision) || !bgfx::isValid(vs_volume) ||
        !bgfx::isValid(fs_volume) || !bgfx::isValid(vs_mesh) ||
        !bgfx::isValid(fs_mesh_stencil)) {
        if (bgfx::isValid(vs)) {
            bgfx::destroy(vs);
        }
        if (bgfx::isValid(fs_combine)) {
            bgfx::destroy(fs_combine);
        }
        if (bgfx::isValid(fs_collision)) {
            bgfx::destroy(fs_collision);
        }
        if (bgfx::isValid(vs_volume)) {
            bgfx::destroy(vs_volume);
        }
        if (bgfx::isValid(fs_volume)) {
            bgfx::destroy(fs_volume);
        }
        if (bgfx::isValid(vs_mesh)) {
            bgfx::destroy(vs_mesh);
        }
        if (bgfx::isValid(fs_mesh_stencil)) {
            bgfx::destroy(fs_mesh_stencil);
        }
        std::cerr << "RenderDeferred shader load failed from " << shader_dir_
                  << std::endl;
        return false;
    }

    combine_program_ = bgfx::createProgram(vs, fs_combine, true);
    collision_program_ = bgfx::createProgram(vs, fs_collision, true);
    volume_program_ = bgfx::createProgram(vs_volume, fs_volume, true);
    mesh_stencil_program_ = bgfx::createProgram(vs_mesh, fs_mesh_stencil, true);

    return bgfx::isValid(combine_program_) &&
           bgfx::isValid(collision_program_) && bgfx::isValid(volume_program_);
}

}  // namespace sinriv::ui::render
