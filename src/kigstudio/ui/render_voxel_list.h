#pragma once
#include <iostream>
#include <map>
#include <string>

#include "kigstudio/ui/render_collision.h"
#include "kigstudio/ui/render_deferred.h"
#include "kigstudio/ui/render_mesh.h"
#include "kigstudio/ui/render_voxel.h"
#include "kigstudio/utils/plane.h"
#include "kigstudio/voxel/voxelizer_svo.h"

namespace sinriv::ui::render {

using mat4f = sinriv::kigstudio::mat::matrix<float>;
class RenderVoxelList {
    // 用于显示一系列窗口
    // 每个子对象由以下部分构成：
    // * 一个mesh
    // * 一个体素
    // * 一个碰撞体，一个空间分割平面（二者只能启动一个）
    // * 两个输出结果（被分割为两半）
    int current_id = 0;
    std::mutex locker;

   public:
    class RenderVoxelItem {
       public:
        int id = -1;
        RenderVoxelList* manager = nullptr;
        RenderVoxelItem() = default;
        ~RenderVoxelItem() = default;
        enum segment_mode {
            COLLISION,
            PLANE,
        } segment_mode = COLLISION;

        sinriv::ui::render::RenderMesh mesh_renderer;
        sinriv::ui::render::RenderVoxel voxel_renderer;
        sinriv::kigstudio::voxel::VoxelGrid voxel_grid_data;

        sinriv::kigstudio::voxel::collision::CollisionGroup collision_group;
        kigstudio::Plane<float> plane;

        inline void render_gbuffer(const float* transform) {
            if (showMesh) {
                mesh_renderer.renderGBuffer(transform);
            }

            if (showVoxel) {
                voxel_renderer.renderGBuffer(transform);
            }
        }
        inline void render_overlay(
            sinriv::ui::render::RenderCollision& collision_renderer,
            const float* model_transform,
            const float* model_transform_2,
            const mat4f* cpu_model_matrix = nullptr) {
            if (showMesh) {
                mesh_renderer.renderOverlay();
            }
            if (showVoxel) {
                voxel_renderer.renderOverlay();
            }
            if (showCollision) {
                collision_renderer.render(collision_group, model_transform,
                                          model_transform_2, cpu_model_matrix);
            }
        }
        inline void upload_collision(
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

        inline auto do_segment() {
            // 执行分割，并在manager中创建两个，然后返回
            if (segment_mode == COLLISION) {
                return std::move(voxel_grid_data.segment(collision_group));
            } else if (segment_mode == PLANE) {
                return std::move(voxel_grid_data.segment(plane));
            }
        }

        int ref_count = 0;

        bool showMesh = true;
        bool showVoxel = true;
        bool showCollision = true;
    };
    RenderVoxelList() = default;
    ~RenderVoxelList() = default;

    std::map<int, std::unique_ptr<RenderVoxelItem>> items;

    int render_id = 0;

    inline void render_gbuffer(const float* transform) {
        locker.lock();
        if (render_id >= 0 && render_id < items.size()) {
            items[render_id]->render_gbuffer(transform);
        }
        locker.unlock();
    }

    inline void render_overlay(
        sinriv::ui::render::RenderCollision& collision_renderer,
        const float* model_transform,
        const float* model_transform_2,
        const mat4f* cpu_model_matrix = nullptr) {
        locker.lock();
        if (render_id >= 0 && render_id < items.size()) {
            items[render_id]->render_overlay(collision_renderer,
                                             model_transform, model_transform_2,
                                             cpu_model_matrix);
        }
        locker.unlock();
    }

    inline void upload_collision(sinriv::ui::render::RenderDeferred& render) {
        locker.lock();
        if (render_id >= 0 && render_id < items.size()) {
            items[render_id]->upload_collision(render);
        } else {
            render.clearCollisionTint();
        }
        locker.unlock();
    }

    inline std::tuple<RenderVoxelItem*, RenderVoxelItem*> do_segment(
        int index) {
        // 执行分割，并在manager中创建两个，然后返回
        auto it = items.find(index);
        if (it != items.end()) {
            auto res = it->second->do_segment();
            auto item1 = std::make_unique<RenderVoxelItem>();
            auto item2 = std::make_unique<RenderVoxelItem>();
            item1->manager = this;
            item2->manager = this;
            item1->id = current_id++;
            item2->id = current_id++;
            item1->voxel_grid_data = std::get<0>(res);
            item2->voxel_grid_data = std::get<1>(res);
            auto item1_ptr = item1.get();
            auto item2_ptr = item2.get();
            locker.lock();
            items[item1->id] = std::move(item1);
            items[item2->id] = std::move(item2);
            locker.unlock();
            return std::make_tuple(item1_ptr, item2_ptr);
        }
        return std::make_tuple(nullptr, nullptr);
    }
};

}  // namespace sinriv::ui::render