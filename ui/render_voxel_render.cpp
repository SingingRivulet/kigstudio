#include "render_voxel_list.h"
namespace sinriv::ui::render {

void RenderVoxelList::RenderVoxelItem::render_gbuffer(
    const float* transform,
    sinriv::ui::render::RenderMeshShader& mesh_shader) {
    if (showMesh) {
        mesh_renderer.renderGBuffer(transform, mesh_shader);
    }

    if (showVoxel) {
        voxel_renderer.renderGBuffer(transform, mesh_shader);
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
}

void RenderVoxelList::RenderVoxelItem::upload_collision(
    sinriv::ui::render::RenderDeferred& render) {
    if (showCollision) {
        if (segment_mode == COLLISION) {
            render.setCollisionGroup(collision_group);
            render.setSpaceDivVisible(false);
        } else {
            render.clearCollisionTint();
            render.setSpaceDivVisible(true);
            render.setSpaceDiv(plane.A, plane.B, plane.C, plane.D);
        }
    } else {
        render.clearCollisionTint();
    }
}

}  // namespace sinriv::ui::render