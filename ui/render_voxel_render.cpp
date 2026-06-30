#include "render_voxel_list.h"
#include <bx/math.h>
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

void append_marker_circle(std::vector<mesh_detail::ColorLineVertex>& vertices,
                          const sinriv::kigstudio::voxel::vec3f& center,
                          const sinriv::kigstudio::voxel::vec3f& axis_u,
                          const sinriv::kigstudio::voxel::vec3f& axis_v,
                          float radius,
                          uint32_t color) {
    constexpr int kSegments = 48;
    constexpr float kPi = 3.14159265358979323846f;
    for (int i = 0; i < kSegments; ++i) {
        const float a0 = 2.0f * kPi * static_cast<float>(i) /
                         static_cast<float>(kSegments);
        const float a1 = 2.0f * kPi * static_cast<float>(i + 1) /
                         static_cast<float>(kSegments);
        const auto p0 = center + axis_u * (std::cos(a0) * radius) +
                        axis_v * (std::sin(a0) * radius);
        const auto p1 = center + axis_u * (std::cos(a1) * radius) +
                        axis_v * (std::sin(a1) * radius);
        vertices.push_back({p0.x, -p0.y, p0.z, color});
        vertices.push_back({p1.x, -p1.y, p1.z, color});
    }
}

sinriv::kigstudio::voxel::vec3f transform_point(
    const mat4f& matrix,
    const sinriv::kigstudio::voxel::vec3f& point) {
    sinriv::kigstudio::mat::vec4<float> transformed =
        sinriv::kigstudio::mat::vec4<float>(point.x, point.y, point.z, 1.0f) *
        matrix;
    auto v3 = transformed.toVec3();
    return {v3.x, v3.y, v3.z};
}
}  // namespace

void RenderVoxelList::RenderVoxelItem::render_gbuffer(
    const float* transform,
    sinriv::ui::render::RenderMeshShader& mesh_shader) {
    mesh_renderer.cull_backface = !mesh_only;
    exported_mesh_renderer.cull_backface = !mesh_only;
    if (showMesh) {
        mesh_renderer.renderGBuffer(transform, mesh_shader);
    }
    if (showOriginMesh && !origin_mesh_renderer.empty()) {
        origin_mesh_renderer.cull_backface = false;
        origin_mesh_renderer.renderGBuffer(transform, mesh_shader);
    }
    if (showExportedMesh && !cached_mesh.empty()) {
        if (!cached_mesh_dirty) {
            exported_mesh_renderer.loadGeometry(cached_mesh);
            cached_mesh_dirty = true;
        }
        exported_mesh_renderer.renderGBuffer(transform, mesh_shader);
    }

    if (showVoxel) {
        voxel_renderer.renderGBuffer(transform, mesh_shader);
    }

    if (showVoxel && !marked_voxels.empty()) {
        if (marked_voxels_dirty) {
            marked_voxels.global_position = voxel_grid_data.global_position;
            marked_voxels.voxel_size = voxel_grid_data.voxel_size;
            marked_mesh_renderer.setBaseColor(1.0f, 0.5f, 0.5f, 1.0f);
            marked_mesh_renderer.setDepthBias(0.07f);
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

    if (segment_mode == SDF_NODE_SPLIT && showCollision &&
        sdf_split_target_id >= 0 && manager) {
        auto target_it = manager->items.find(sdf_split_target_id);
        if (target_it != manager->items.end() &&
            !target_it->second->mesh_renderer.empty()) {
            mat4f split_transform = sdf_split_transform_matrix();
            split_transform.transpose();
            float split_transform_bgfx[16];
            sinriv::kigstudio::mat::toBGFXMat(split_transform,
                                              split_transform_bgfx);
            float combined_transform[16];
            bx::mtxMul(combined_transform, split_transform_bgfx, transform);

            auto old_color = target_it->second->mesh_renderer.getBaseColor();
            target_it->second->mesh_renderer.setBaseColor(1.0f, 0.6f, 0.6f,
                                                          1.0f);
            target_it->second->mesh_renderer.renderGBuffer(
                combined_transform, mesh_shader, true);
            target_it->second->mesh_renderer.setBaseColor(
                old_color[0], old_color[1], old_color[2], old_color[3]);
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
    if (showVoxelChunkBounds && !voxel_grid_data.chunks.empty()) {
        if (mesh_shader.ensureLineProgram()) {
            bgfx::VertexLayout& layout = concave_cone_overlay_layout();
            const uint32_t chunk_color = pack_abgr(0.0f, 1.0f, 1.0f, 1.0f);
            std::vector<mesh_detail::ColorLineVertex> vertices;
            vertices.reserve(voxel_grid_data.chunks.size() * 24);
            for (const auto& [key, chunk] : voxel_grid_data.chunks) {
                (void)chunk;
                int cx, cy, cz;
                sinriv::kigstudio::voxel::unpackChunkKey(key, cx, cy, cz);
                float minx = voxel_grid_data.global_position.x +
                             cx * sinriv::kigstudio::voxel::Chunk::SIZE *
                                 voxel_grid_data.voxel_size.x;
                float miny = voxel_grid_data.global_position.y +
                             cy * sinriv::kigstudio::voxel::Chunk::SIZE *
                                 voxel_grid_data.voxel_size.y;
                float minz = voxel_grid_data.global_position.z +
                             cz * sinriv::kigstudio::voxel::Chunk::SIZE *
                                 voxel_grid_data.voxel_size.z;
                float maxx = minx + sinriv::kigstudio::voxel::Chunk::SIZE *
                                          voxel_grid_data.voxel_size.x;
                float maxy = miny + sinriv::kigstudio::voxel::Chunk::SIZE *
                                          voxel_grid_data.voxel_size.y;
                float maxz = minz + sinriv::kigstudio::voxel::Chunk::SIZE *
                                          voxel_grid_data.voxel_size.z;
                float corners[8][3] = {
                    {minx, miny, minz}, {maxx, miny, minz},
                    {maxx, maxy, minz}, {minx, maxy, minz},
                    {minx, miny, maxz}, {maxx, miny, maxz},
                    {maxx, maxy, maxz}, {minx, maxy, maxz},
                };
                int edges[12][2] = {
                    {0, 1}, {1, 2}, {2, 3}, {3, 0},
                    {4, 5}, {5, 6}, {6, 7}, {7, 4},
                    {0, 4}, {1, 5}, {2, 6}, {3, 7},
                };
                for (auto& e : edges) {
                    vertices.push_back(
                        {corners[e[0]][0], -corners[e[0]][1],
                         corners[e[0]][2], chunk_color});
                    vertices.push_back(
                        {corners[e[1]][0], -corners[e[1]][1],
                         corners[e[1]][2], chunk_color});
                }
            }
            if (!vertices.empty() &&
                bgfx::getAvailTransientVertexBuffer(
                    static_cast<uint32_t>(vertices.size()),
                    layout) >= vertices.size()) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb, static_cast<uint32_t>(vertices.size()),
                    layout);
                std::memcpy(
                    tvb.data, vertices.data(),
                    vertices.size() * sizeof(mesh_detail::ColorLineVertex));
                bgfx::setTransform(model_transform);
                bgfx::setVertexBuffer(0, &tvb);
                bgfx::setState(BGFX_STATE_WRITE_RGB |
                               BGFX_STATE_WRITE_A |
                               BGFX_STATE_PT_LINES |
                               BGFX_STATE_MSAA);
                bgfx::submit(mesh_shader.overlay_view_id_,
                             mesh_shader.line_program_);
            }
        }
    }
    if (showCollision && segment_mode == COLLISION) {
        collision_renderer.render(collision_group, model_transform,
                                  model_transform_2, collision_shader,
                                  cpu_model_matrix);
    }
    if (showCollisionBounds &&
        segment_mode == SDF_NODE_SPLIT) {
        if (sdf_split_target_id >= 0 && manager) {
            auto target_it = manager->items.find(sdf_split_target_id);
            if (target_it != manager->items.end() &&
                !target_it->second->mesh_renderer.empty() &&
                mesh_shader.ensureLineProgram()) {
                auto [min_local, max_local] =
                    target_it->second->mesh_renderer.getLocalBounds();
                bgfx::VertexLayout& layout = concave_cone_overlay_layout();
                const uint32_t bounds_color = pack_abgr(0.0f, 1.0f, 0.0f, 1.0f);
                std::vector<mesh_detail::ColorLineVertex> vertices;
                vertices.reserve(24);
                sinriv::kigstudio::voxel::vec3f corners[8] = {
                    {min_local.x, min_local.y, min_local.z},
                    {max_local.x, min_local.y, min_local.z},
                    {max_local.x, max_local.y, min_local.z},
                    {min_local.x, max_local.y, min_local.z},
                    {min_local.x, min_local.y, max_local.z},
                    {max_local.x, min_local.y, max_local.z},
                    {max_local.x, max_local.y, max_local.z},
                    {min_local.x, max_local.y, max_local.z},
                };
                mat4f split_transform = sdf_split_transform_matrix();
                // split_transform.transpose();
                for (auto& corner : corners) {
                    corner = transform_point(split_transform, corner);
                }
                int edges[12][2] = {
                    {0, 1}, {1, 2}, {2, 3}, {3, 0},
                    {4, 5}, {5, 6}, {6, 7}, {7, 4},
                    {0, 4}, {1, 5}, {2, 6}, {3, 7},
                };
                for (auto& e : edges) {
                    auto& a = corners[e[0]];
                    auto& b = corners[e[1]];
                    vertices.push_back(
                        {a.x, -a.y, a.z, bounds_color});
                    vertices.push_back(
                        {b.x, -b.y, b.z, bounds_color});
                }
                if (!vertices.empty() &&
                    bgfx::getAvailTransientVertexBuffer(
                        static_cast<uint32_t>(vertices.size()),
                        layout) >= vertices.size()) {
                    bgfx::TransientVertexBuffer tvb;
                    bgfx::allocTransientVertexBuffer(
                        &tvb, static_cast<uint32_t>(vertices.size()), layout);
                    std::memcpy(
                        tvb.data, vertices.data(),
                        vertices.size() * sizeof(mesh_detail::ColorLineVertex));
                    bgfx::setTransform(model_transform);
                    bgfx::setVertexBuffer(0, &tvb);
                    bgfx::setState(BGFX_STATE_WRITE_RGB |
                                   BGFX_STATE_WRITE_A |
                                   BGFX_STATE_PT_LINES |
                                   BGFX_STATE_MSAA);
                    bgfx::submit(mesh_shader.overlay_view_id_,
                                 mesh_shader.line_program_);
                }
            }
        }
    }
    if (showCollisionBounds && segment_mode == COLLISION) {
        collision_renderer.renderBounds(collision_group, model_transform,
                                        model_transform_2, collision_shader,
                                        cpu_model_matrix);
    }
    if (showCollision && segment_mode == CONCAVE_CONE) {
        render_concave_cone_overlay(model_transform,
                                    mesh_shader);
    }
    if (segment_mode == CHAIN && !skeleton_lines.empty()) {
        if (mesh_shader.ensureLineProgram()) {
            bgfx::VertexLayout& layout = concave_cone_overlay_layout();
            const uint32_t line_color = pack_abgr(1.0f, 0.84f, 0.08f, 1.0f);
            std::vector<mesh_detail::ColorLineVertex> vertices;
            vertices.reserve(skeleton_lines.size() * 2);
            for (const auto& line : skeleton_lines) {
                const auto& a = line.first;
                const auto& b = line.second;
                vertices.push_back({a.x, -a.y, a.z, line_color});
                vertices.push_back({b.x, -b.y, b.z, line_color});
            }
            if (!vertices.empty() &&
                bgfx::getAvailTransientVertexBuffer(
                    static_cast<uint32_t>(vertices.size()),
                    layout) >= vertices.size()) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb, static_cast<uint32_t>(vertices.size()),
                    layout);
                std::memcpy(tvb.data, vertices.data(),
                            vertices.size() *
                                sizeof(mesh_detail::ColorLineVertex));
                bgfx::setTransform(model_transform);
                bgfx::setVertexBuffer(0, &tvb);
                bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                               BGFX_STATE_WRITE_Z |
                               BGFX_STATE_DEPTH_TEST_LESS |
                               BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
                bgfx::submit(mesh_shader.overlay_view_id_,
                             mesh_shader.line_program_);
            }
        }
    }
    if (segment_mode == CHAIN && !picked_skeleton_points.empty()) {
        if (mesh_shader.ensureLineProgram()) {
            bgfx::VertexLayout& layout = concave_cone_overlay_layout();
            const uint32_t marker_color = pack_abgr(1.0f, 0.18f, 0.08f, 1.0f);
            const float radius =
                std::max({voxel_grid_data.voxel_size.x,
                          voxel_grid_data.voxel_size.y,
                          voxel_grid_data.voxel_size.z}) *
                2.0f;
            std::vector<mesh_detail::ColorLineVertex> vertices;
            vertices.reserve(picked_skeleton_points.size() * 48 * 6);
            for (const auto& picked : picked_skeleton_points) {
                const auto& p = picked.position;
                append_marker_circle(vertices, p, {1.0f, 0.0f, 0.0f},
                                     {0.0f, 1.0f, 0.0f}, radius,
                                     marker_color);
                append_marker_circle(vertices, p, {1.0f, 0.0f, 0.0f},
                                     {0.0f, 0.0f, 1.0f}, radius,
                                     marker_color);
                append_marker_circle(vertices, p, {0.0f, 1.0f, 0.0f},
                                     {0.0f, 0.0f, 1.0f}, radius,
                                     marker_color);
            }
            if (!vertices.empty() &&
                bgfx::getAvailTransientVertexBuffer(
                    static_cast<uint32_t>(vertices.size()),
                    layout) >= vertices.size()) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb, static_cast<uint32_t>(vertices.size()),
                    layout);
                std::memcpy(tvb.data, vertices.data(),
                            vertices.size() *
                                sizeof(mesh_detail::ColorLineVertex));
                bgfx::setTransform(model_transform);
                bgfx::setVertexBuffer(0, &tvb);
                bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                               BGFX_STATE_WRITE_Z |
                               BGFX_STATE_DEPTH_TEST_LESS |
                               BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
                bgfx::submit(mesh_shader.overlay_view_id_,
                             mesh_shader.line_program_);
            }

            // joint wireframes
            if (joint_wireframe_dirty) {
                rebuild_joint_wireframe();
            }
            if (!joint_wireframe_vertices.empty() &&
                bgfx::getAvailTransientVertexBuffer(
                    static_cast<uint32_t>(joint_wireframe_vertices.size()),
                    layout) >= joint_wireframe_vertices.size()) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb,
                    static_cast<uint32_t>(joint_wireframe_vertices.size()),
                    layout);
                std::memcpy(tvb.data, joint_wireframe_vertices.data(),
                            joint_wireframe_vertices.size() *
                                sizeof(mesh_detail::ColorLineVertex));
                bgfx::setTransform(model_transform);
                bgfx::setVertexBuffer(0, &tvb);
                bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                               BGFX_STATE_WRITE_Z |
                               BGFX_STATE_DEPTH_TEST_LESS |
                               BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
                bgfx::submit(mesh_shader.overlay_view_id_,
                             mesh_shader.line_program_);
            }
        }
    }
    if (showSilhouetteCenter &&
        stl_load_mode == static_cast<int>(StlLoadMode::SILHOUETTE)) {
        if (mesh_shader.ensureLineProgram()) {
            bgfx::VertexLayout& layout = concave_cone_overlay_layout();
            const uint32_t center_color = pack_abgr(1.0f, 0.84f, 0.08f, 1.0f);
            const float radius = 2.0f;
            std::vector<mesh_detail::ColorLineVertex> vertices;
            vertices.reserve(48 * 6 +
                             (inner_wall_radius > 0.0f ? 48 * 6 : 0));
            append_marker_circle(vertices, silhouette_center,
                                 {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
                                 radius, center_color);
            append_marker_circle(vertices, silhouette_center,
                                 {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
                                 radius, center_color);
            append_marker_circle(vertices, silhouette_center,
                                 {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
                                 radius, center_color);
            if (inner_wall_radius > 0.0f) {
                const uint32_t wall_color =
                    pack_abgr(0.0f, 0.95f, 1.0f, 0.72f);
                append_marker_circle(vertices, silhouette_center,
                                     {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
                                     inner_wall_radius, wall_color);
                append_marker_circle(vertices, silhouette_center,
                                     {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
                                     inner_wall_radius, wall_color);
                append_marker_circle(vertices, silhouette_center,
                                     {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
                                     inner_wall_radius, wall_color);
            }
            if (!vertices.empty() &&
                bgfx::getAvailTransientVertexBuffer(
                    static_cast<uint32_t>(vertices.size()),
                    layout) >= vertices.size()) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb, static_cast<uint32_t>(vertices.size()),
                    layout);
                std::memcpy(tvb.data, vertices.data(),
                            vertices.size() *
                                sizeof(mesh_detail::ColorLineVertex));
                bgfx::setTransform(model_transform);
                bgfx::setVertexBuffer(0, &tvb);
                bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                               BGFX_STATE_WRITE_Z |
                               BGFX_STATE_DEPTH_TEST_LESS |
                               BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
                bgfx::submit(mesh_shader.overlay_view_id_,
                             mesh_shader.line_program_);
            }
        }
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
    if (segment_mode == SDF_NODE_SPLIT) {
        render.clearCollisionTint();
        render.setSpaceDivVisible(false);
        if (sdf_split_target_id >= 0 && manager) {
            auto target_it = manager->items.find(sdf_split_target_id);
            if (target_it != manager->items.end()) {
                const auto& mh = target_it->second->mesh_renderer.getMeshHandle();
                float split_transform_bgfx[16];
                mat4f split_transform = sdf_split_transform_matrix();
                split_transform.transpose();
                sinriv::kigstudio::mat::toBGFXMat(split_transform,
                                                  split_transform_bgfx);
                render.submitMeshStencil(mh.vbh, mh.ibh, mh.index_count,
                                         split_transform_bgfx);
            } else {
                render.submitMeshStencil(
                    BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE, 0);
            }
        } else {
            render.submitMeshStencil(
                BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE, 0);
        }
    } else if (showCollision) {
        render.submitMeshStencil(
            BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE, 0);
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
        render.pos_hightlight_[i][3] = std::get<2>(hightlight_pos[i]);

        render.pos_hightlight_color_[i][0] = std::get<1>(hightlight_pos[i]).x;
        render.pos_hightlight_color_[i][1] = std::get<1>(hightlight_pos[i]).y;
        render.pos_hightlight_color_[i][2] = std::get<1>(hightlight_pos[i]).z;
        render.pos_hightlight_color_[i][3] = 1.0f;
    }
    hightlight_pos.clear();
}
}  // namespace sinriv::ui::render
