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

    bool update_nav_node_status = true;

   public:
    class RenderVoxelItem {
       public:
        int id = -1;
        int children[2] = {-1, -1};
        int nav_node_position[2] = {0, 0}; // 在分割演示图中的位置
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

        void render_gbuffer(const float* transform,
                            sinriv::ui::render::RenderMeshShader& mesh_shader);
        void render_overlay(
            sinriv::ui::render::RenderCollision& collision_renderer,
            const float* model_transform,
            const float* model_transform_2,
            sinriv::ui::render::RenderCollisionShader& collision_shader,
            sinriv::ui::render::RenderMeshShader& mesh_shader,
            const mat4f* cpu_model_matrix = nullptr);
        void upload_collision(sinriv::ui::render::RenderDeferred& render);

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

    // 渲染
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
        collision_renderer.showAxis = this->showCollisionAxis;
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->render_overlay(collision_renderer, model_transform,
                                       model_transform_2, collision_shader,
                                       mesh_shader, cpu_model_matrix);
        }
    }

    // ui
    int window_width;
    int window_height;
    int menu_height=0;
    
    bool showMesh = true;
    bool showVoxels = false;
    bool showCollision = true;

    bool showMeshAxis = false;
    bool showVoxelAxis = false;
    bool showCollisionAxis = false;

    void render_ui();
    void render_nav_map();

    inline void upload_collision(sinriv::ui::render::RenderDeferred& render) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->upload_collision(render);
        } else {
            render.clearCollisionTint();
        }
    }

    // 摄像机

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

    // 交互

    inline RenderVoxelItem* create_item() {
        auto item = std::make_unique<RenderVoxelItem>();
        item->manager = this;
        item->id = current_id++;
        auto item_ptr = item.get();
        {
            std::lock_guard<std::mutex> lock(locker);
            items[item->id] = std::move(item);
            update_nav_node_status = true;
        }
        return item_ptr;
    }

    std::tuple<RenderVoxelItem*, RenderVoxelItem*> do_segment(int index);

    void load_stl(std::string filename,
                         float voxel_size = 0.5f,
                         double isolevel = 0.5,
                         bool smooth_normals = true);

    // 后台队列

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
            update_nav_node_status = true;
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
        this->setRenderId(id);
    }
    
    inline void setRenderId_unsafe(int id) {
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
    int queue_num = 0;

    std::thread queue_thread_;

    inline void queue_thread() {
        std::cout << "Queue thread started" << std::endl;
        while (true) {
            queue_mutex.lock();
            this->queue_num = queue.size();
            if (queue.empty()) {
                queue_mutex.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
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
                    queue_running = true;
                    do_segment(task.index);
                    queue_running = false;
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

    std::vector<int> find_roots();
    void update_nav_node_position();

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
        this->queue_num = queue.size();
    }

    inline void queue_do_segment(int index) {
        // 将分割任务加入队列
        std::lock_guard<std::mutex> lock(queue_mutex);
        QueueTask task;
        task.type = TASK_SEGMENT;
        task.index = index;
        queue.push(task);
        this->queue_num = queue.size();
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