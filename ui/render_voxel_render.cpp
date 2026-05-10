#include "render_voxel_list.h"
namespace sinriv::ui::render {
namespace {
uint32_t pack_abgr(float r, float g, float b, float a) {
    const auto pack = [](float v) -> uint32_t {
        v = std::max(0.0f, std::min(1.0f, v));
        return static_cast<uint32_t>(v * 255.0f + 0.5f);
    };
    return (pack(a) << 24) | (pack(b) << 16) | (pack(g) << 8) | pack(r);
}

bool contains_index(const std::vector<int>& indices, int value) {
    return std::find(indices.begin(), indices.end(), value) != indices.end();
}

template <class Vec3>
Vec3 extend_cone_edge(const Vec3& apex, const Vec3& vertex) {
    constexpr float kEdgeScale = 4.0f;
    return apex + (vertex - apex) * kEdgeScale;
}

bgfx::VertexLayout& concave_cone_overlay_layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;
    if (!initialized) {
        mesh_detail::ColorLineVertex::init(layout);
        initialized = true;
    }
    return layout;
}
}  // namespace

void RenderVoxelList::RenderVoxelItem::render_gbuffer(
    const float* transform,
    sinriv::ui::render::RenderMeshShader& mesh_shader) {
    if (showMesh) {
        mesh_renderer.renderGBuffer(transform, mesh_shader);
    }

    if (showVoxel) {
        voxel_renderer.renderGBuffer(transform, mesh_shader);
    }

    if (voxel_picking_enabled && !marked_voxels.empty()) {
        if (marked_voxels_dirty) {
            marked_voxels.global_position = voxel_grid_data.global_position;
            marked_voxels.voxel_size = voxel_grid_data.voxel_size;
            marked_mesh_renderer.setBaseColor(1.0f, 0.5f, 0.5f, 1.0f);
            marked_mesh_renderer.setDepthBias(0.0001f);
            int num_triangles = 0;
            auto generator = sinriv::kigstudio::voxel::generateMesh(
                marked_voxels, 0.5, num_triangles, true);
            marked_mesh_renderer.loadGeometry(generator);
            marked_voxels_dirty = false;
        }
        if (!marked_mesh_renderer.empty()) {
            marked_mesh_renderer.renderGBuffer(transform, mesh_shader);
        }
    }
}

void RenderVoxelList::RenderVoxelItem::render_overlay(
    sinriv::ui::render::RenderCollision& collision_renderer,
    const float* model_transform,
    const float* model_transform_2,
    sinriv::ui::render::RenderCollisionShader& collision_shader,
    sinriv::ui::render::RenderMeshShader& mesh_shader,
    const mat4f* cpu_model_matrix) {
    if (showMesh) {
        mesh_renderer.renderOverlay(mesh_shader);
    }
    if (showVoxel) {
        voxel_renderer.renderOverlay(mesh_shader);
    }
    if (showCollision) {
        collision_renderer.render(collision_group, model_transform,
                                  model_transform_2, collision_shader,
                                  cpu_model_matrix);
    }
    if (showCollisionBounds) {
        collision_renderer.renderBounds(collision_group, model_transform,
                                        model_transform_2, collision_shader,
                                        cpu_model_matrix);
    }
    if (showCollision && segment_mode == CONCAVE_CONE) {
        render_concave_cone_overlay(model_transform,
                                    mesh_shader);
    }
}

void RenderVoxelList::RenderVoxelItem::render_concave_cone_overlay(
    const float* model_transform,
    sinriv::ui::render::RenderMeshShader& mesh_shader) {
    const auto& verts = concave_cone.base_vertices;
    const int vertex_count = static_cast<int>(verts.size());
    if (vertex_count < 2 || !mesh_shader.ensureLineProgram()) {
        return;
    }

    bgfx::VertexLayout& layout = concave_cone_overlay_layout();

    const uint32_t face_color = pack_abgr(0.1f, 0.72f, 1.0f, 0.22f);
    const uint32_t edge_color = pack_abgr(0.0f, 0.95f, 1.0f, 0.72f);
    const uint32_t vertex_loop_color = pack_abgr(1.0f, 1.0f, 1.0f, 0.9f);
    const uint32_t highlight_color = pack_abgr(1.0f, 0.84f, 0.08f, 1.0f);

    std::vector<sinriv::kigstudio::voxel::concave::vec3f> extended_verts;
    extended_verts.reserve(static_cast<size_t>(vertex_count));
    for (const auto& vertex : verts) {
        extended_verts.push_back(
            extend_cone_edge(concave_cone.apex, vertex));
    }

    std::vector<mesh_detail::ColorLineVertex> face_vertices;
    face_vertices.reserve(static_cast<size_t>(vertex_count) * 3 +
                          static_cast<size_t>(vertex_count) * 3);
    for (int i = 0; i < vertex_count; ++i) {
        const auto& a = concave_cone.apex;
        const auto& b = extended_verts[i];
        const auto& c = extended_verts[(i + 1) % vertex_count];
        face_vertices.push_back({a.x, -a.y, a.z, face_color});
        face_vertices.push_back({b.x, -b.y, b.z, face_color});
        face_vertices.push_back({c.x, -c.y, c.z, face_color});
    }

    concave_cone.triangulate();
    for (const auto& tri : concave_cone.base_triangles) {
        const auto& v0 = extended_verts[tri[0]];
        const auto& v1 = extended_verts[tri[1]];
        const auto& v2 = extended_verts[tri[2]];
        face_vertices.push_back({v0.x, -v0.y, v0.z, face_color});
        face_vertices.push_back({v1.x, -v1.y, v1.z, face_color});
        face_vertices.push_back({v2.x, -v2.y, v2.z, face_color});
    }

    if (!face_vertices.empty() &&
        bgfx::getAvailTransientVertexBuffer(
            static_cast<uint32_t>(face_vertices.size()),
            layout) >= face_vertices.size()) {
        bgfx::TransientVertexBuffer tvb;
        bgfx::allocTransientVertexBuffer(
            &tvb, static_cast<uint32_t>(face_vertices.size()),
            layout);
        std::memcpy(tvb.data, face_vertices.data(),
                    face_vertices.size() * sizeof(mesh_detail::ColorLineVertex));

        bgfx::setTransform(model_transform);
        bgfx::setVertexBuffer(0, &tvb);
        bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                       BGFX_STATE_BLEND_ALPHA | BGFX_STATE_MSAA);
        bgfx::submit(mesh_shader.overlay_view_id_, mesh_shader.line_program_);
    }

    std::vector<mesh_detail::ColorLineVertex> line_vertices;
    line_vertices.reserve(static_cast<size_t>(vertex_count) * 6);
    auto append_line = [&](const auto& a, const auto& b, uint32_t color) {
        line_vertices.push_back({a.x, -a.y, a.z, color});
        line_vertices.push_back({b.x, -b.y, b.z, color});
    };
    for (int i = 0; i < vertex_count; ++i) {
        append_line(extended_verts[i],
                    extended_verts[(i + 1) % vertex_count], edge_color);
        append_line(verts[i], verts[(i + 1) % vertex_count],
                    vertex_loop_color);
        const bool expanded = contains_index(concave_cone_expanded_vertices, i);
        append_line(concave_cone.apex, extended_verts[i],
                    expanded ? highlight_color : edge_color);
    }

    if (line_vertices.empty() ||
        bgfx::getAvailTransientVertexBuffer(
            static_cast<uint32_t>(line_vertices.size()),
            layout) < line_vertices.size()) {
        return;
    }

    bgfx::TransientVertexBuffer tvb;
    bgfx::allocTransientVertexBuffer(
        &tvb, static_cast<uint32_t>(line_vertices.size()),
        layout);
    std::memcpy(tvb.data, line_vertices.data(),
                line_vertices.size() * sizeof(mesh_detail::ColorLineVertex));

    bgfx::setTransform(model_transform);
    bgfx::setVertexBuffer(0, &tvb);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                   BGFX_STATE_BLEND_ALPHA | BGFX_STATE_PT_LINES |
                   BGFX_STATE_MSAA);
    bgfx::submit(mesh_shader.overlay_view_id_, mesh_shader.line_program_);
}

void RenderVoxelList::RenderVoxelItem::upload_collision(
    sinriv::ui::render::RenderDeferred& render) {
    if (showCollision) {
        if (segment_mode == COLLISION) {
            render.setCollisionGroup(collision_group);
            render.setSpaceDivVisible(false);
        } else if (segment_mode == PLANE) {
            render.clearCollisionTint();
            render.setSpaceDivVisible(true);
            render.setSpaceDiv(plane.A, plane.B, plane.C, plane.D);
        } else if (segment_mode == CONCAVE_CONE) {
            render.setConcaveCone(concave_cone);
            render.setSpaceDivVisible(false);
        } else {
            // SPLIT_DISCONNECTED / NEIGHBOR: no collision overlay
            render.clearCollisionTint();
            render.setSpaceDivVisible(false);
        }
    } else {
        render.clearCollisionTint();
    }
}

void RenderVoxelList::upload_collision(
    sinriv::ui::render::RenderDeferred& render) {
    {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->upload_collision(render);
        } else {
            render.clearCollisionTint();
        }
    }
    int num = static_cast<int>(hightlight_pos.size());
    if (num > 16) {
        num = 16;
    }
    render.pos_hightlight_counts = num;
    render.pos_hightlight_counts_gpu_[0] = static_cast<float>(num);
    for (int i = 0; i < num; i++) {
        render.pos_hightlight_[i][0] = std::get<0>(hightlight_pos[i]).x;
        render.pos_hightlight_[i][1] = std::get<0>(hightlight_pos[i]).y;
        render.pos_hightlight_[i][2] = std::get<0>(hightlight_pos[i]).z;
        render.pos_hightlight_[i][3] = 1.0f;

        render.pos_hightlight_color_[i][0] = std::get<1>(hightlight_pos[i]).x;
        render.pos_hightlight_color_[i][1] = std::get<1>(hightlight_pos[i]).y;
        render.pos_hightlight_color_[i][2] = std::get<1>(hightlight_pos[i]).z;
        render.pos_hightlight_color_[i][3] = 1.0f;
    }
    hightlight_pos.clear();
}
}  // namespace sinriv::ui::render
