#pragma once
#include <atomic>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <thread>

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
    std::atomic<int> current_id = 0;
    std::mutex locker;

    float queue_progress = 0;
    bool queue_running = false;
    std::string queue_status;

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

        inline void render_gbuffer(
            const float* transform,
            sinriv::ui::render::RenderMeshShader& mesh_shader) {
            if (showMesh) {
                mesh_renderer.renderGBuffer(transform, mesh_shader);
            }

            if (showVoxel) {
                voxel_renderer.renderGBuffer(transform, mesh_shader);
            }
        }
        inline void render_overlay(
            sinriv::ui::render::RenderCollision& collision_renderer,
            const float* model_transform,
            const float* model_transform_2,
            sinriv::ui::render::RenderCollisionShader& collision_shader,
            sinriv::ui::render::RenderMeshShader& mesh_shader,
            const mat4f* cpu_model_matrix = nullptr) {
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

        std::atomic<int> ref_count = 1;
        std::atomic<int> write_count = 0;

        bool showMesh = true;
        bool showVoxel = true;
        bool showCollision = true;
    };
    inline RenderVoxelList() {}
    inline ~RenderVoxelList() { release(); }

    std::map<int, std::unique_ptr<RenderVoxelItem>> items;

    int render_id = 0;

    inline void render_gbuffer(
        const float* transform,
        sinriv::ui::render::RenderMeshShader& mesh_shader) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->render_gbuffer(transform, mesh_shader);
        }
    }

    inline void render_overlay(
        sinriv::ui::render::RenderCollision& collision_renderer,
        const float* model_transform,
        const float* model_transform_2,
        sinriv::ui::render::RenderCollisionShader& collision_shader,
        sinriv::ui::render::RenderMeshShader& mesh_shader,
        const mat4f* cpu_model_matrix = nullptr) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->render_overlay(collision_renderer, model_transform,
                                       model_transform_2, collision_shader,
                                       mesh_shader, cpu_model_matrix);
        }
    }

    inline void upload_collision(sinriv::ui::render::RenderDeferred& render) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->upload_collision(render);
        } else {
            render.clearCollisionTint();
        }
    }

    inline void setViewportSize(int width, int height) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->mesh_renderer.setViewportSize(width, height);
            it->second->voxel_renderer.setViewportSize(width, height);
        }
    }

    inline void setViewProjection(const float* view, const float* proj) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->mesh_renderer.setViewProjection(view, proj);
            it->second->voxel_renderer.setViewProjection(view, proj);
        }
    }

    inline void setModelMatrix(const mat4f& model_matrix) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->mesh_renderer.setModelMatrix(model_matrix);
            it->second->voxel_renderer.setModelMatrix(model_matrix);
        }
    }

    inline void setMeshAxisVisible(bool visible) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->mesh_renderer.showAxis = visible;
        }
    }

    inline void setVoxelAxisVisible(bool visible) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->voxel_renderer.showAxis = visible;
        }
    }

    inline void setMeshVisible(bool visible) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->showMesh = visible;
        }
    }
    inline void setVoxelsVisible(bool visible) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->showVoxel = visible;
        }
    }
    inline void setCollisionVisible(bool visible) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->showCollision = visible;
        }
    }

    inline RenderVoxelItem* create_item() {
        auto item = std::make_unique<RenderVoxelItem>();
        item->manager = this;
        item->id = current_id++;
        auto item_ptr = item.get();
        {
            std::lock_guard<std::mutex> lock(locker);
            items[item->id] = std::move(item);
        }
        return item_ptr;
    }

    inline std::tuple<RenderVoxelItem*, RenderVoxelItem*> do_segment(
        int index) {
        // 执行分割，并在manager中创建两个，然后返回
        locker.lock();
        auto it = items.find(index);
        if (it != items.end()) {
            if (it->second->write_count != 0) {
                locker.unlock();
                return std::make_tuple(nullptr, nullptr);
            }
            it->second->ref_count++;
            it->second->write_count++;
            locker.unlock();
            auto res = it->second->do_segment();
            it->second->ref_count--;
            it->second->write_count--;
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
            {
                std::lock_guard<std::mutex> lock(locker);
                items[item1->id] = std::move(item1);
                items[item2->id] = std::move(item2);
            }
            return std::make_tuple(item1_ptr, item2_ptr);
        } else {
            locker.unlock();
        }
        return std::make_tuple(nullptr, nullptr);
    }

    inline void load_stl(std::string filename,
                         float voxel_size = 0.5f,
                         double isolevel = 0.5,
                         bool smooth_normals = true) {
        using namespace sinriv::kigstudio::voxel;
        using MeshData = mesh_detail::AsyncVoxelMeshData;
        using Triangle = triangle_bvh<float>::triangle;

        // Phase 1: Read STL
        queue_progress = 0.05f;
        triangle_bvh<float> bvh;
        size_t tri_count = 0;
        for (auto [tri, n] : sinriv::kigstudio::voxel::readSTL(filename)) {
            (void)n;
            bvh.insert(tri);
            ++tri_count;
            if (tri_count % 1000 == 0) {
                queue_progress =
                    (0.05f +
                     0.08f * std::min(1.0f, static_cast<float>(tri_count) /
                                                50000.0f));
            }
        }

        // Phase 2: Voxelize
        queue_status = "Voxelizing...";
        queue_progress = 0.15f;

        VoxelGrid voxel_data;
        voxel_data.voxel_size.x = voxel_size;
        voxel_data.voxel_size.y = voxel_size;
        voxel_data.voxel_size.z = voxel_size;
        voxel_data.global_position.x = bvh.global_boundBox_min.x;
        voxel_data.global_position.y = bvh.global_boundBox_min.y;
        voxel_data.global_position.z = bvh.global_boundBox_min.z;

        float minx = floor(bvh.global_boundBox_min.x / voxel_size) * voxel_size;
        float miny = floor(bvh.global_boundBox_min.y / voxel_size) * voxel_size;
        float minz = floor(bvh.global_boundBox_min.z / voxel_size) * voxel_size;
        float maxx = ceil(bvh.global_boundBox_max.x / voxel_size) * voxel_size;
        float maxy = ceil(bvh.global_boundBox_max.y / voxel_size) * voxel_size;
        float maxz = ceil(bvh.global_boundBox_max.z / voxel_size) * voxel_size;
        int num_block_y = static_cast<int>(ceil((maxy - miny) / voxel_size));
        int num_block_z = static_cast<int>(ceil((maxz - minz) / voxel_size));
        size_t total_rays = static_cast<size_t>(num_block_y) * num_block_z;
        if (total_rays == 0)
            total_rays = 1;

        std::mutex bvh_locker;
        std::atomic<size_t> callback_count{0};

        bvh.getSolidByFace(
            voxel_size, voxel_size, voxel_size,
            triangle_bvh<float>::voxel_face_X, [&](auto start, auto end) {
                int start_x = static_cast<int>(std::round(
                    (start.x - voxel_data.global_position.x) / voxel_size));
                int start_y = static_cast<int>(std::round(
                    (start.y - voxel_data.global_position.y) / voxel_size));
                int start_z = static_cast<int>(std::round(
                    (start.z - voxel_data.global_position.z) / voxel_size));
                int end_x = static_cast<int>(std::round(
                    (end.x - voxel_data.global_position.x) / voxel_size));
                int end_y = static_cast<int>(std::round(
                    (end.y - voxel_data.global_position.y) / voxel_size));
                int end_z = static_cast<int>(std::round(
                    (end.z - voxel_data.global_position.z) / voxel_size));

                bvh_locker.lock();
                for (int i = start_x; i <= end_x; ++i) {
                    for (int j = start_y; j <= end_y; ++j) {
                        for (int k = start_z; k <= end_z; ++k) {
                            if (i >= 0 && j >= 0 && k >= 0) {
                                voxel_data.insert({i, j, k});
                            }
                        }
                    }
                }
                bvh_locker.unlock();

                size_t cnt =
                    callback_count.fetch_add(1, std::memory_order_relaxed) + 1;
                if (cnt % 100 == 0) {
                    float p =
                        0.15f + 0.35f * std::min(1.0f, static_cast<float>(cnt) /
                                                           static_cast<float>(
                                                               total_rays * 2));
                    queue_progress = p;
                }
            });

        queue_progress = 0.50f;

        // Phase 3: Generate mesh
        queue_status = "Generating mesh...";
        int num_triangles = 0;
        auto generator =
            generateMesh(voxel_data, isolevel, num_triangles, smooth_normals);
        MeshData data;
        size_t processed_tris = 0;
        size_t estimated_tris = voxel_data.num_chunk() * 200;
        if (estimated_tris == 0)
            estimated_tris = 1;

        for (auto [tri, n] : generator) {
            const uint32_t base = static_cast<uint32_t>(data.vertices.size());
            data.vertices.push_back({std::get<0>(tri).x, std::get<0>(tri).y,
                                     std::get<0>(tri).z, n.x, n.y, n.z});
            data.vertices.push_back({std::get<1>(tri).x, std::get<1>(tri).y,
                                     std::get<1>(tri).z, n.x, n.y, n.z});
            data.vertices.push_back({std::get<2>(tri).x, std::get<2>(tri).y,
                                     std::get<2>(tri).z, n.x, n.y, n.z});
            data.indices.push_back(base);
            data.indices.push_back(base + 1);
            data.indices.push_back(base + 2);

            ++processed_tris;
            if (processed_tris % 1000 == 0) {
                float p =
                    0.50f +
                    0.40f *
                        std::min(1.0f, static_cast<float>(processed_tris) /
                                           static_cast<float>(estimated_tris));
                queue_progress = p;
            }
        }

        queue_status = "Uploading...";
        queue_progress = 0.95f;

        {
            // std::lock_guard<std::mutex> lock(result_mutex_);
            // result_mesh = std::move(data);
            // result_voxel = std::move(voxel_data);

            auto item = std::make_unique<RenderVoxelItem>();
            item->manager = this;
            item->id = current_id++;
            item->mesh_renderer.loadGeometry(
                sinriv::kigstudio::voxel::readSTL(filename));
            item->voxel_renderer.loadGeometry(data);
            item->voxel_grid_data = std::move(voxel_data);
            {
                std::lock_guard<std::mutex> lock(locker);
                items[item->id] = std::move(item);
            }
        }

        queue_status = "Done";
        queue_progress = 1.0f;
    }

    inline void process_queue_result() {
        // 回收ref_count和write_count为0的item
        std::lock_guard<std::mutex> lock(locker);
        std::vector<int> to_remove;
        for (auto& it : items) {
            if (it.second->ref_count == 0 && it.second->write_count == 0) {
                to_remove.push_back(it.first);
            }
        }
        for (auto& it : to_remove) {
            items.erase(it);
        }
        // 重新选一个可用的render_id
        if (items.find(render_id) == items.end()) {
            auto it = items.begin();
            if (it != items.end()) {
                render_id = it->first;
            }
        }
    }

    inline void setRenderId(int id) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(id);
        if (it != items.end()) {
            render_id = id;
        }
    }

    enum QueueTaskType {
        TASK_STOP = 1,
        TASK_REMOVE_ITEM = 2,
        TASK_LOAD_STL = 3,
        TASK_SEGMENT = 4
    };
    struct QueueTask {
        QueueTaskType type;
        int index;
        std::string file_path;
    };
    std::queue<QueueTask> queue;
    std::mutex queue_mutex;

    std::thread queue_thread_;

    inline void queue_thread() {
        std::cout << "Queue thread started" << std::endl;
        while (true) {
            if (queue.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            queue_mutex.lock();
            auto task = queue.front();
            queue.pop();
            queue_mutex.unlock();

            switch (task.type) {
                case TASK_STOP:
                    std::cout << "Stop queue" << std::endl;
                    return;
                // case TASK_REMOVE_ITEM:
                //     // 移除item
                //     break;
                case TASK_LOAD_STL:
                    // 加载stl文件
                    std::cout << "Load stl file: " << task.file_path
                              << std::endl;
                    queue_running = true;
                    load_stl(task.file_path);
                    queue_running = false;
                    break;
                case TASK_SEGMENT:
                    // 分割
                    do_segment(task.index);
                    break;
            }
        }
    }

    inline void start_thread() {
        // 启动进程
        queue_thread_ = std::thread(&RenderVoxelList::queue_thread, this);
    }

    inline void stop_thread() {
        if (queue_thread_.joinable()) {
            std::cout << "Waiting for queue thread to stop" << std::endl;
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                QueueTask task;
                task.type = TASK_STOP;
                queue.push(task);
            }
            queue_thread_.join();
            std::cout << "Queue thread stopped" << std::endl;
        }
    }

    inline auto get_num_items() {
        std::lock_guard<std::mutex> lock(locker);
        auto res = items.size();
        return res;
    }

    inline void queue_load_stl(const std::string& file_path) {
        // 将加载任务加入队列
        std::lock_guard<std::mutex> lock(queue_mutex);
        QueueTask task;
        task.type = TASK_LOAD_STL;
        task.file_path = file_path;
        queue.push(task);
    }

    inline void queue_do_segment(int index) {
        // 将分割任务加入队列
        std::lock_guard<std::mutex> lock(queue_mutex);
        QueueTask task;
        task.type = TASK_SEGMENT;
        task.index = index;
        queue.push(task);
    }

    inline void queue_do_segment() {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            queue_do_segment(it->second->id);
        }
    }

    inline void queue_remove_item(int index) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(index);
        if (it != items.end()) {
            it->second->ref_count--;
        }
    }

    inline bool isQueueRunning() { return queue_running; }

    inline std::string getQueueStatus() { return queue_status; }

    inline float getQueueProgress() { return queue_progress; }

    inline void release() {
        stop_thread();
        items.clear();
    }
};

}  // namespace sinriv::ui::render