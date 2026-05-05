#pragma once
#include <atomic>
#include <cJSON.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#include "ui/render_deferred.h"
#include "kigstudio/ui/render_collision.h"
#include "kigstudio/ui/render_mesh.h"
#include "kigstudio/ui/render_voxel.h"
#include "kigstudio/utils/plane.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/voxel/voxelizer_svo.h"
#include "kigstudio/voxel/concave.h"

namespace sinriv::ui::render {

#ifdef _WIN32
inline std::wstring utf8_to_wstring(const std::string& utf8) {
    if (utf8.empty()) return {};
    int len = MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, nullptr, 0);
    if (len <= 1) return {};
    std::wstring w(len - 1, 0);
    MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, &w[0], len);
    return w;
}
inline std::string wstring_to_utf8(const std::wstring& w) {
    if (w.empty()) return {};
    int len = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (len <= 1) return {};
    std::string s(len - 1, 0);
    WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, &s[0], len, nullptr, nullptr);
    return s;
}
inline std::filesystem::path utf8_path(const std::string& utf8) {
    return std::filesystem::path(utf8_to_wstring(utf8));
}
inline std::string path_to_utf8(const std::filesystem::path& p) {
    return wstring_to_utf8(p.wstring());
}
#else
inline std::filesystem::path utf8_path(const std::string& utf8) {
    return std::filesystem::path(utf8);
}
inline std::string path_to_utf8(const std::filesystem::path& p) {
    return p.string();
}
#endif

using mat4f = sinriv::kigstudio::mat::matrix<float>;

// Forward declaration
class RenderVoxelList;

struct CollisionEditorSnapshot {
    sinriv::kigstudio::voxel::collision::CollisionGroup collision_group;
    sinriv::kigstudio::Plane<float> plane;
    sinriv::kigstudio::voxel::concave::Cone concave_cone;
    std::vector<int> concave_cone_expanded_vertices;
    int segment_mode;
    std::string description;
};

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
        int nav_node_position[2] = {0, 0};  // 在分割演示图中的位置
        std::string err_info;
        RenderVoxelList* manager = nullptr;
        RenderVoxelItem() = default;
        ~RenderVoxelItem() {
            if (bgfx::isValid(thumbnail_tex)) {
                bgfx::destroy(thumbnail_tex);
            }
        }
        enum SegmentMode {
            COLLISION = 0,
            PLANE = 1,
            CONCAVE_CONE = 2
        } segment_mode = COLLISION;

        sinriv::ui::render::RenderMesh mesh_renderer;
        sinriv::ui::render::RenderVoxel voxel_renderer;
        sinriv::kigstudio::voxel::VoxelGrid voxel_grid_data;

        sinriv::kigstudio::voxel::collision::CollisionGroup collision_group;
        kigstudio::Plane<float> plane;
        kigstudio::voxel::concave::Cone concave_cone;
        std::vector<int> concave_cone_expanded_vertices;

        void render_gbuffer(const float* transform,
                            sinriv::ui::render::RenderMeshShader& mesh_shader);
        void render_overlay(
            sinriv::ui::render::RenderCollision& collision_renderer,
            const float* model_transform,
            const float* model_transform_2,
            sinriv::ui::render::RenderCollisionShader& collision_shader,
            sinriv::ui::render::RenderMeshShader& mesh_shader,
            const mat4f* cpu_model_matrix = nullptr);
        void render_concave_cone_overlay(const float* model_transform,
                                         sinriv::ui::render::RenderMeshShader& mesh_shader);
        void upload_collision(sinriv::ui::render::RenderDeferred& render);

        inline auto do_segment() {
            // 执行分割，并在manager中创建两个，然后返回
            if (segment_mode == COLLISION) {
                return std::move(voxel_grid_data.segment(collision_group));
            } else if (segment_mode == PLANE) {
                return std::move(voxel_grid_data.segment(plane));
            } else if (segment_mode == CONCAVE_CONE) {
                return std::move(voxel_grid_data.segment(concave_cone));
            } else {
                throw std::runtime_error("未知的分割模式");
            }
        }

        std::atomic<int> ref_count = 1;
        std::atomic<int> write_count = 0;

        bool queue_release = false;

        bool showMesh = true;
        bool showVoxel = true;
        bool showCollision = true;
        bool showCollisionBounds = false;

        bgfx::TextureHandle thumbnail_tex = BGFX_INVALID_HANDLE;
        bool thumbnail_dirty = true;
        
        std::string stl_path;
        std::string voxel_path;
        float stl_voxel_size = 1.0f;

        // undo/redo stacks for collision editor
        std::vector<CollisionEditorSnapshot> undo_stack;
        std::vector<CollisionEditorSnapshot> redo_stack;

        bool dirty = false;
    };
    inline RenderVoxelList() {}
    inline ~RenderVoxelList() { release(); }

    std::map<int, std::unique_ptr<RenderVoxelItem>> items;

    // 缩略图生成
    struct ThumbnailTask {
        int item_id = -1;
        enum Stage { RENDER, WAIT, DONE } stage = RENDER;
        int wait_frames = 0;
    };
    std::queue<ThumbnailTask> thumbnail_queue;
    std::set<int> thumbnail_mesh_pending;
    std::map<int, mesh_detail::AsyncVoxelMeshData> thumbnail_mesh_results;
    std::mutex thumbnail_mesh_mutex;

    bgfx::FrameBufferHandle thumb_fb_ = BGFX_INVALID_HANDLE;
    bgfx::TextureHandle thumb_color_tex_ = BGFX_INVALID_HANDLE;
    bgfx::TextureHandle thumb_depth_tex_ = BGFX_INVALID_HANDLE;
    std::unique_ptr<RenderMeshShader> thumb_shader_;

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
    int menu_height = 0;

    bool showMesh = true;
    bool showVoxels = true;
    bool showCollision = true;

    bool showMeshAxis = false;
    bool showVoxelAxis = false;
    bool showCollisionAxis = false;
    bool showCollisionBounds = true;

    sinriv::kigstudio::voxel::collision::vec3f mouse_world_pos = {0, 0, 0};
    bool mouse_world_pos_valid = false;
    bool mouse_world_pos_picked = false;

    void render_ui();
    void render_collision_node_editor();
    void render_plane_editor(RenderVoxelItem& item);
    void render_collision_body_editor(RenderVoxelItem& item);
    void render_concave_cone_editor(RenderVoxelItem& item);
    void render_nav_map();
    void render_file_loader();
    void render_reload_stl_dialog();
    void render_save_dialog();
    void render_load_dialog();

    bool show_edit_segment_plane = false;
    bool show_file_loader = false;
    bool show_reload_stl_dialog = false;
    int reload_stl_item_id = -1;
    float reload_stl_voxel_size = 1.0f;
    bool show_save_dialog = false;
    bool show_save_as_dialog = false;
    bool show_load_dialog = false;

    std::string project_path;

    std::string last_save_error;
    std::string last_load_error;

    // undo/redo
    struct PendingUndo {
        int item_id;
        CollisionEditorSnapshot snapshot;
    };
    std::optional<PendingUndo> pending_undo;
    static constexpr size_t kMaxUndoSize = 50;

    inline CollisionEditorSnapshot capture_snapshot(const RenderVoxelItem& item) const {
        return {
            item.collision_group,
            item.plane,
            item.concave_cone,
            item.concave_cone_expanded_vertices,
            item.segment_mode
        };
    }

    inline void apply_snapshot(RenderVoxelItem& item, const CollisionEditorSnapshot& snapshot) {
        item.collision_group = snapshot.collision_group;
        item.plane = snapshot.plane;
        item.concave_cone = snapshot.concave_cone;
        item.concave_cone_expanded_vertices = snapshot.concave_cone_expanded_vertices;
        item.segment_mode = static_cast<RenderVoxelItem::SegmentMode>(snapshot.segment_mode);
    }

    inline void begin_edit(int item_id) {
        if (pending_undo.has_value() && pending_undo->item_id == item_id) {
            return; // already have pending undo for this item
        }
        // discard pending for different item
        pending_undo.reset();
        auto it = items.find(item_id);
        if (it != items.end()) {
            pending_undo = PendingUndo{item_id, capture_snapshot(*it->second)};
        }
    }

    inline void end_edit(int item_id, const std::string& desc = "Edit") {
        if (!pending_undo.has_value() || pending_undo->item_id != item_id) {
            return;
        }
        auto it = items.find(item_id);
        if (it != items.end()) {
            it->second->undo_stack.push_back(pending_undo->snapshot);
            it->second->undo_stack.back().description = desc;
            it->second->redo_stack.clear();
            it->second->dirty = true;
            if (it->second->undo_stack.size() > kMaxUndoSize) {
                it->second->undo_stack.erase(it->second->undo_stack.begin());
            }
        }
        pending_undo.reset();
    }

    inline void push_undo_now(int item_id, const std::optional<CollisionEditorSnapshot>& before = std::nullopt, const std::string& desc = "") {
        auto it = items.find(item_id);
        if (it == items.end()) return;
        it->second->undo_stack.push_back(before.value_or(capture_snapshot(*it->second)));
        it->second->undo_stack.back().description = desc;
        it->second->redo_stack.clear();
        it->second->dirty = true;
        if (it->second->undo_stack.size() > kMaxUndoSize) {
            it->second->undo_stack.erase(it->second->undo_stack.begin());
        }
        pending_undo.reset();
    }

    inline bool undo(int item_id) {
        auto it = items.find(item_id);
        if (it == items.end() || it->second->undo_stack.empty()) return false;
        // push current state to redo stack
        it->second->redo_stack.push_back(capture_snapshot(*it->second));
        // apply undo snapshot
        apply_snapshot(*it->second, it->second->undo_stack.back());
        it->second->undo_stack.pop_back();
        it->second->dirty = true;
        return true;
    }

    inline bool redo(int item_id) {
        auto it = items.find(item_id);
        if (it == items.end() || it->second->redo_stack.empty()) return false;
        // push current state to undo stack
        it->second->undo_stack.push_back(capture_snapshot(*it->second));
        // apply redo snapshot
        apply_snapshot(*it->second, it->second->redo_stack.back());
        it->second->redo_stack.pop_back();
        it->second->dirty = true;
        return true;
    }

    inline bool can_undo(int item_id) const {
        auto it = items.find(item_id);
        return it != items.end() && !it->second->undo_stack.empty();
    }

    inline bool can_redo(int item_id) const {
        auto it = items.find(item_id);
        return it != items.end() && !it->second->redo_stack.empty();
    }

    inline bool has_dirty_items() const {
        for (const auto& [id, item] : items) {
            if (item->dirty) return true;
        }
        return false;
    }

    inline void clear_all_dirty() {
        for (auto& [id, item] : items) {
            item->dirty = false;
        }
    }

    bool show_history_window = false;
    void render_history_window();

    bool show_log_window = false;

    std::vector<std::tuple<sinriv::kigstudio::voxel::collision::vec3f,
                           sinriv::kigstudio::voxel::collision::vec3f>>
        hightlight_pos;

    void upload_collision(sinriv::ui::render::RenderDeferred& render);

    inline void update_mouse() {}

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
    inline void setCollisionBoundsVisible(bool visible) {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->showCollisionBounds = visible;
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
                  bool smooth_normals = true,
                  int target_item_id = -1);

    // 后台队列

    std::vector<std::unique_ptr<RenderVoxelItem>> pending_deletion;
    std::mutex pending_deletion_mutex;

    void process_queue_result();

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
        TASK_SEGMENT = 4,
        TASK_GENERATE_THUMBNAIL_MESH = 5,
        TASK_RELOAD_STL = 6,
    };
    struct QueueTask {
        QueueTaskType type;
        int index;
        std::string file_path;
        float voxel_size;
    };
    std::queue<QueueTask> queue;
    std::mutex queue_mutex;
    int queue_num = 0;

    std::thread queue_thread_;

    void queue_thread();

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

    inline void queue_load_stl(const std::string& file_path, float voxel_size) {
        // 将加载任务加入队列
        std::lock_guard<std::mutex> lock(queue_mutex);
        QueueTask task;
        task.type = TASK_LOAD_STL;
        task.file_path = file_path;
        task.voxel_size = voxel_size;
        queue.push(task);
        this->queue_num = static_cast<int>(queue.size());
    }

    inline void queue_reload_stl(int item_id, float voxel_size,
                                  const std::string& stl_path) {
        if (stl_path.empty()) return;
        std::lock_guard<std::mutex> lock(queue_mutex);
        QueueTask task;
        task.type = TASK_RELOAD_STL;
        task.index = item_id;
        task.file_path = stl_path;
        task.voxel_size = voxel_size;
        queue.push(task);
        this->queue_num = static_cast<int>(queue.size());
    }

    inline void queue_do_segment(int index) {
        // 将分割任务加入队列
        std::lock_guard<std::mutex> lock(queue_mutex);
        QueueTask task;
        task.type = TASK_SEGMENT;
        task.index = index;
        queue.push(task);
        this->queue_num = static_cast<int>(queue.size());
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
        destroyThumbnailResources();
        items.clear();
        pending_deletion.clear();
    }

    void processThumbnails();
    void ensureThumbnailResources();
    void destroyThumbnailResources();

    // ===== Project Serialization =====
    inline cJSON* item_to_json(const RenderVoxelItem& item) const {
        cJSON* obj = cJSON_CreateObject();
        cJSON_AddNumberToObject(obj, "id", item.id);
        cJSON* children = cJSON_CreateArray();
        cJSON_AddItemToArray(children, cJSON_CreateNumber(item.children[0]));
        cJSON_AddItemToArray(children, cJSON_CreateNumber(item.children[1]));
        cJSON_AddItemToObject(obj, "children", children);
        cJSON* nav_pos = cJSON_CreateArray();
        cJSON_AddItemToArray(nav_pos, cJSON_CreateNumber(item.nav_node_position[0]));
        cJSON_AddItemToArray(nav_pos, cJSON_CreateNumber(item.nav_node_position[1]));
        cJSON_AddItemToObject(obj, "nav_node_position", nav_pos);
        cJSON_AddStringToObject(obj, "segment_mode",
            item.segment_mode == RenderVoxelItem::COLLISION ? "collision" :
            item.segment_mode == RenderVoxelItem::PLANE ? "plane" : "concave_cone");
        cJSON_AddBoolToObject(obj, "show_mesh", item.showMesh);
        cJSON_AddBoolToObject(obj, "show_voxel", item.showVoxel);
        cJSON_AddBoolToObject(obj, "show_collision", item.showCollision);
        cJSON_AddBoolToObject(obj, "show_collision_bounds", item.showCollisionBounds);
        cJSON_AddStringToObject(obj, "stl_path", item.stl_path.c_str());
        cJSON_AddStringToObject(obj, "voxel_path", item.voxel_path.c_str());
        cJSON_AddNumberToObject(obj, "stl_voxel_size", item.stl_voxel_size);
        cJSON_AddStringToObject(obj, "err_info", item.err_info.c_str());
        cJSON_AddItemToObject(obj, "collision_group",
            sinriv::kigstudio::to_json(item.collision_group));
        cJSON_AddItemToObject(obj, "plane",
            sinriv::kigstudio::to_json(item.plane));
        cJSON_AddItemToObject(obj, "concave_cone",
            sinriv::kigstudio::voxel::concave::to_json(item.concave_cone));
        cJSON* expanded = cJSON_CreateArray();
        for (int v : item.concave_cone_expanded_vertices) {
            cJSON_AddItemToArray(expanded, cJSON_CreateNumber(v));
        }
        cJSON_AddItemToObject(obj, "concave_cone_expanded_vertices", expanded);
        cJSON_AddItemToObject(obj, "voxel_global_position",
            sinriv::kigstudio::to_json(item.voxel_grid_data.global_position));
        cJSON_AddItemToObject(obj, "voxel_size",
            sinriv::kigstudio::to_json(item.voxel_grid_data.voxel_size));
        return obj;
    }

    inline std::unique_ptr<RenderVoxelItem> item_from_json(const cJSON* obj) {
        auto item = std::make_unique<RenderVoxelItem>();
        item->manager = this;
        item->id = cJSON_GetObjectItem(obj, "id")->valueint;
        const cJSON* children = cJSON_GetObjectItem(obj, "children");
        item->children[0] = cJSON_GetArrayItem(children, 0)->valueint;
        item->children[1] = cJSON_GetArrayItem(children, 1)->valueint;
        const cJSON* nav_pos = cJSON_GetObjectItem(obj, "nav_node_position");
        item->nav_node_position[0] = cJSON_GetArrayItem(nav_pos, 0)->valueint;
        item->nav_node_position[1] = cJSON_GetArrayItem(nav_pos, 1)->valueint;
        const char* mode_str = cJSON_GetObjectItem(obj, "segment_mode")->valuestring;
        if (strcmp(mode_str, "collision") == 0) {
            item->segment_mode = RenderVoxelItem::COLLISION;
        } else if (strcmp(mode_str, "plane") == 0) {
            item->segment_mode = RenderVoxelItem::PLANE;
        } else {
            item->segment_mode = RenderVoxelItem::CONCAVE_CONE;
        }
        item->showMesh = cJSON_IsTrue(cJSON_GetObjectItem(obj, "show_mesh"));
        item->showVoxel = cJSON_IsTrue(cJSON_GetObjectItem(obj, "show_voxel"));
        item->showCollision = cJSON_IsTrue(cJSON_GetObjectItem(obj, "show_collision"));
        item->showCollisionBounds = cJSON_IsTrue(cJSON_GetObjectItem(obj, "show_collision_bounds"));
        item->stl_path = cJSON_GetObjectItem(obj, "stl_path")->valuestring;
        item->voxel_path = cJSON_GetObjectItem(obj, "voxel_path")->valuestring;
        const cJSON* stl_voxel_size_json = cJSON_GetObjectItem(obj, "stl_voxel_size");
        if (stl_voxel_size_json) {
            item->stl_voxel_size = static_cast<float>(stl_voxel_size_json->valuedouble);
        }
        item->err_info = cJSON_GetObjectItem(obj, "err_info")->valuestring;
        item->collision_group = sinriv::kigstudio::from_json_collision_group(
            cJSON_GetObjectItem(obj, "collision_group"));
        item->plane = sinriv::kigstudio::from_json_plane(
            cJSON_GetObjectItem(obj, "plane"));
        item->concave_cone = sinriv::kigstudio::voxel::concave::from_json_cone(
            cJSON_GetObjectItem(obj, "concave_cone"));
        const cJSON* expanded = cJSON_GetObjectItem(obj, "concave_cone_expanded_vertices");
        int expanded_count = cJSON_GetArraySize(expanded);
        for (int i = 0; i < expanded_count; ++i) {
            item->concave_cone_expanded_vertices.push_back(
                cJSON_GetArrayItem(expanded, i)->valueint);
        }
        item->voxel_grid_data.global_position = sinriv::kigstudio::vec3_from_json<sinriv::kigstudio::vec3<float>>(
            cJSON_GetObjectItem(obj, "voxel_global_position"));
        item->voxel_grid_data.voxel_size = sinriv::kigstudio::vec3_from_json<sinriv::kigstudio::vec3<float>>(
            cJSON_GetObjectItem(obj, "voxel_size"));
        return item;
    }

    inline bool save_current_project() {
        if (project_path.empty()) {
            last_save_error = "no project path set";
            return false;
        }
        return save_project(project_path);
    }

    inline bool save_project(const std::string& folder) {
        last_save_error.clear();
        std::filesystem::path dir = utf8_path(folder);
        try {
            std::filesystem::create_directories(dir / "voxels");
        } catch (const std::exception& e) {
            last_save_error = std::string("create_directories failed: ") + e.what();
            return false;
        }

        std::lock_guard<std::mutex> lock(locker);
        cJSON* root = cJSON_CreateObject();
        if (!root) {
            last_save_error = "cJSON_CreateObject failed";
            return false;
        }
        cJSON_AddNumberToObject(root, "version", 1);
        cJSON_AddNumberToObject(root, "current_id", current_id.load());
        cJSON* arr = cJSON_CreateArray();
        for (const auto& [id, item] : items) {
            cJSON* item_json = item_to_json(*item);
            if (!item_json) {
                last_save_error = "item_to_json failed for item id=" + std::to_string(id);
                cJSON_Delete(root);
                return false;
            }
            cJSON_AddItemToArray(arr, item_json);
            std::filesystem::path voxel_path = dir / "voxels" / (std::to_string(id) + ".vxgrid");
            if (!std::filesystem::exists(dir / "voxels")) {
                last_save_error = "voxels directory not found: " + path_to_utf8(dir / "voxels");
                cJSON_Delete(root);
                return false;
            }
            std::string voxel_error;
            if (!sinriv::kigstudio::save(voxel_path, item->voxel_grid_data, &voxel_error)) {
                last_save_error = "save voxel failed: " + path_to_utf8(voxel_path) + " (" + voxel_error + ")";
                cJSON_Delete(root);
                return false;
            }
        }
        cJSON_AddItemToObject(root, "items", arr);

        std::filesystem::path json_path = dir / "project.json";
        char* json_str = cJSON_Print(root);
        if (!json_str) {
            last_save_error = "cJSON_Print failed";
            cJSON_Delete(root);
            return false;
        }
#ifdef _WIN32
        std::ofstream ofs(json_path.wstring().c_str());
#else
        std::ofstream ofs(json_path.c_str());
#endif
        if (!ofs) {
            last_save_error = "failed to open project.json for writing: " + path_to_utf8(json_path);
            cJSON_free(json_str);
            cJSON_Delete(root);
            return false;
        }
        const char utf8_bom[] = "\xEF\xBB\xBF";
        ofs.write(utf8_bom, 3);
        ofs << json_str;
        bool ok = ofs.good();
        if (!ok) {
            last_save_error = "failed to write project.json";
        }
        cJSON_free(json_str);
        cJSON_Delete(root);
        if (ok) clear_all_dirty();
        return ok;
    }

    inline bool load_project(const std::string& folder) {
        last_load_error.clear();
        release();
        start_thread();
        current_id = 0;

        std::filesystem::path dir = utf8_path(folder);
        std::filesystem::path json_path = dir / "project.json";
#ifdef _WIN32
        std::ifstream ifs(json_path.wstring().c_str());
#else
        std::ifstream ifs(json_path.c_str());
#endif
        if (!ifs) {
            last_load_error = "failed to open project.json: " + path_to_utf8(json_path);
            return false;
        }
        std::string json_str((std::istreambuf_iterator<char>(ifs)),
                              std::istreambuf_iterator<char>());
        if (json_str.size() >= 3 &&
            static_cast<unsigned char>(json_str[0]) == 0xEF &&
            static_cast<unsigned char>(json_str[1]) == 0xBB &&
            static_cast<unsigned char>(json_str[2]) == 0xBF) {
            json_str.erase(0, 3);
        }
        cJSON* root = cJSON_Parse(json_str.c_str());
        if (!root) {
            last_load_error = "cJSON_Parse failed";
            return false;
        }

        cJSON* version_obj = cJSON_GetObjectItem(root, "version");
        if (!version_obj) {
            last_load_error = "missing 'version' field";
            cJSON_Delete(root);
            return false;
        }
        int version = version_obj->valueint;
        if (version != 1) {
            last_load_error = "unsupported version: " + std::to_string(version);
            cJSON_Delete(root);
            return false;
        }

        cJSON* current_id_obj = cJSON_GetObjectItem(root, "current_id");
        if (!current_id_obj) {
            last_load_error = "missing 'current_id' field";
            cJSON_Delete(root);
            return false;
        }
        current_id = current_id_obj->valueint;

        cJSON* arr = cJSON_GetObjectItem(root, "items");
        if (!arr) {
            last_load_error = "missing 'items' field";
            cJSON_Delete(root);
            return false;
        }
        int count = cJSON_GetArraySize(arr);
        {
            std::lock_guard<std::mutex> lock(locker);
            for (int i = 0; i < count; ++i) {
                const cJSON* item_obj = cJSON_GetArrayItem(arr, i);
                auto item = item_from_json(item_obj);
                int id = item->id;
                std::filesystem::path voxel_path = dir / "voxels" / (std::to_string(id) + ".vxgrid");
                if (!sinriv::kigstudio::load(voxel_path, item->voxel_grid_data)) {
                    last_load_error = "load voxel failed: " + voxel_path.string();
                    cJSON_Delete(root);
                    return false;
                }
                if (!item->stl_path.empty()) {
                    try {
                        item->mesh_renderer.loadSTL(item->stl_path);
                    } catch (const std::exception& e) {
                        std::cout << "Failed to load STL mesh for item " << id
                                  << ": " << e.what() << std::endl;
                    }
                }
                items[id] = std::move(item);
            }
        }

        cJSON_Delete(root);
        if (!items.empty()) {
            render_id = items.begin()->first;
        }
        project_path = folder;
        update_nav_node_status = true;
        clear_all_dirty();
        return true;
    }
};

}  // namespace sinriv::ui::render
