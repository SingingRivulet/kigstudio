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
#include "kigstudio/utils/KDTree.h"
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
        int root_id = -1;
        std::vector<int> children = {-1, -1};
        int nav_node_position[2] = {0, 0};  // 在分割演示图中的位置
        std::string err_info;
        RenderVoxelList* manager = nullptr;
        RenderVoxelItem() = default;
        ~RenderVoxelItem() {
            if (bgfx::isValid(thumbnail_tex)) {
                bgfx::destroy(thumbnail_tex);
            }
        }
        // TODO: 需要新增分离每个独立的mesh的功能(利用VoxelGrid::splitDisconnected)
        enum SegmentMode {
            COLLISION = 0,
            PLANE = 1,
            CONCAVE_CONE = 2
        } segment_mode = COLLISION;

        sinriv::ui::render::RenderMesh mesh_renderer;
        sinriv::ui::render::RenderVoxel voxel_renderer;
        sinriv::kigstudio::voxel::VoxelGrid voxel_grid_data;
        kdtree::KDTree mesh_kd_tree; // 三角形顶点的kd树，用于实现自动吸附

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

        // TODO: 需要新增复制碰撞到另一个item上的功能

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
        std::atomic<int> write_count = 0; // TODO:禁止编辑write_count不为0的item

        bool queue_release = false;

        bool showMesh = true;
        bool showVoxel = true;
        bool showCollision = true;
        bool showCollisionBounds = false;

        bool auto_segment_update = true; // TODO:关闭时，不会执行递归更新，并且手动更新需要二次确认

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
    bool mouse_world_pos_picked_auto_snapping = false;// 自动吸附
    void update_mouse_pos(RenderDeferred & renderer);

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

    CollisionEditorSnapshot capture_snapshot(const RenderVoxelItem& item) const;
    void apply_snapshot(RenderVoxelItem& item, const CollisionEditorSnapshot& snapshot);
    void begin_edit(int item_id);
    void end_edit(int item_id, const std::string& desc = "Edit");
    void push_undo_now(int item_id, const std::optional<CollisionEditorSnapshot>& before = std::nullopt, const std::string& desc = "");
    bool undo(int item_id);
    bool redo(int item_id);
    bool can_undo(int item_id) const;
    bool can_redo(int item_id) const;
    bool has_dirty_items() const;
    void clear_all_dirty();

    bool show_history_window = false;
    void render_history_window();

    bool show_log_window = false;
    void render_log_window();

    std::vector<std::string> queue_log;
    std::mutex queue_log_mutex;
    inline void append_queue_log(const std::string& msg) {
        std::lock_guard<std::mutex> lock(queue_log_mutex);
        queue_log.push_back(msg);
        if (queue_log.size() > 1000) {
            queue_log.erase(queue_log.begin(), queue_log.begin() + (queue_log.size() - 1000));
        }
    }

    std::vector<std::tuple<sinriv::kigstudio::voxel::collision::vec3f,
                           sinriv::kigstudio::voxel::collision::vec3f>>
        hightlight_pos;

    void upload_collision(sinriv::ui::render::RenderDeferred& render);

    inline void update_mouse() {}

    // 摄像机
    void setViewportSize(int width, int height);
    void setViewProjection(const float* view, const float* proj);
    void setModelMatrix(const mat4f& model_matrix);
    void setMeshAxisVisible(bool visible);
    void setVoxelAxisVisible(bool visible);
    void setMeshVisible(bool visible);
    void setVoxelsVisible(bool visible);
    void setCollisionVisible(bool visible);
    void setCollisionBoundsVisible(bool visible);

    // 交互
    RenderVoxelItem* create_item();

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

    void setRenderId(int id);
    void setRenderId_unsafe(int id);

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

    void start_thread();
    void stop_thread();

    std::vector<int> find_roots();
    void update_nav_node_position();

    size_t get_num_items();
    void queue_load_stl(const std::string& file_path, float voxel_size);
    void queue_reload_stl(int item_id, float voxel_size,
                           const std::string& stl_path);
    void queue_do_segment(int index);
    void queue_do_segment();
    void queue_remove_item(int index);
    bool isQueueRunning();
    std::string getQueueStatus();
    float getQueueProgress();
    void release();

    void processThumbnails();
    void ensureThumbnailResources();
    void destroyThumbnailResources();

    // ===== Project Serialization =====
    cJSON* item_to_json(const RenderVoxelItem& item) const;
    std::unique_ptr<RenderVoxelItem> item_from_json(const cJSON* obj);
    bool save_current_project();
    bool save_project(const std::string& folder);
    bool load_project(const std::string& folder);
};

}  // namespace sinriv::ui::render
