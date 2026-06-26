#pragma once
#include <cJSON.h>
#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#include "kigstudio/sdf/sdf_shape.h"
#include "kigstudio/ui/render_collision.h"
#include "kigstudio/ui/render_mesh.h"
#include "kigstudio/ui/render_voxel.h"
#include "kigstudio/utils/KDTree.h"
#include "kigstudio/utils/locale.h"
#include "kigstudio/utils/plane.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/voxel/concave.h"
#include "kigstudio/voxel/voxel.h"
#include "kigstudio/voxel/voxel_EDT.h"
#include "kigstudio/voxel/voxelizer_svo.h"
#include "ui/render_deferred.h"

namespace sinriv::ui::render {

using namespace locale;

#ifdef _WIN32
inline std::wstring utf8_to_wstring(const std::string& utf8) {
    if (utf8.empty())
        return {};
    int len = MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, nullptr, 0);
    if (len <= 1)
        return {};
    std::wstring w(len - 1, 0);
    MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, &w[0], len);
    return w;
}
inline std::string wstring_to_utf8(const std::wstring& w) {
    if (w.empty())
        return {};
    int len = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0,
                                  nullptr, nullptr);
    if (len <= 1)
        return {};
    std::string s(len - 1, 0);
    WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, &s[0], len, nullptr,
                        nullptr);
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

std::string tinyfd_path_to_utf8(const char* path);
std::string localize_id(const char* key, int id);

using mat4f = sinriv::kigstudio::mat::matrix<float>;
using CollisionGroup = sinriv::kigstudio::voxel::collision::CollisionGroup;
using GeometryInstance = sinriv::kigstudio::voxel::collision::GeometryInstance;
using Sphere = sinriv::kigstudio::voxel::collision::Sphere;
using Cylinder = sinriv::kigstudio::voxel::collision::Cylinder;
using Capsule = sinriv::kigstudio::voxel::collision::Capsule;
using Box = sinriv::kigstudio::voxel::collision::Box;
using Transform = sinriv::kigstudio::voxel::collision::Transform;
using vec3f = sinriv::kigstudio::voxel::collision::vec3f;
using Plane = sinriv::kigstudio::Plane<float>;

// Forward declaration
class RenderVoxelList;

struct SkeletonPointPick {
    sinriv::kigstudio::voxel::vec3f position;
    int order = 0;

    // Joint parameters for this picked point
    bool use_custom_direction = false;
    sinriv::kigstudio::voxel::vec3f custom_direction_end = {0, 0, 0};
    float socket_cone_offset = 5.f;
    float socket_cone_angle = 0.5f;
    float socket_cone_radius = 4.f;
    float head_cone_offset = 10.f;
    float head_cone_radius = 3.5f;
    float socket_support_offset = 2.f;
    float socket_support_radius = 5.f;
    float head_support_offset = 2.f;
    float head_support_radius = 5.f;
    float male_cylinder_offset = 3.f;
    float male_cylinder_radius = 1.5f;
    float female_gap = 0.3f;
    float slot_extra = 0.5f;
    float socket_fillet_radius = 5.f;
    float socket_fillet_height = 8.f;
    float socket_fillet_offset = 0.f;
    float head_fillet_height = 3.f;
    float rotation_angle = 0.f;
};

struct EditResult {
    bool activated = false;
    bool deactivated_after_edit = false;
    bool value_changed = false;  // 按钮点击等立即变化
};

EditResult edit_float_stepper(const char* label,
                              float& value,
                              float step = 1.0f);
EditResult edit_vec3_stepper(const char* label,
                             vec3f& value,
                             float step = 0.5f,
                             bool normalize = false);
EditResult edit_local_position_stepper(const char* label,
                                       vec3f& value,
                                       float step = 0.5f,
                                       bool normalize = false,
                                       bool show_label = true);
EditResult edit_transform_controls(Transform& transform);
const char* geometry_type_name(const GeometryInstance& instance);
EditResult edit_geometry_shape(GeometryInstance& instance);
void add_collision_geometry(CollisionGroup& group, int type_index);

enum class StlLoadMode : int {
    DEFAULT = 0,
    SILHOUETTE = 1,
    SURFACE_ONLY = 2,
    MESH_ONLY = 3,
    CONVEX_HULL = 4,
    COUNT
};

struct CollisionEditorSnapshot {
    sinriv::kigstudio::voxel::collision::CollisionGroup collision_group;
    sinriv::kigstudio::Plane<float> plane;
    sinriv::kigstudio::voxel::concave::Cone concave_cone;
    std::vector<int> concave_cone_expanded_vertices;
    int segment_mode;
    std::string description;
    int sdf_split_target_id = -1;
    vec3f sdf_split_translation = {0.0f, 0.0f, 0.0f};
    vec3f sdf_split_rotation = {0.0f, 0.0f, 0.0f};
    vec3f sdf_split_scale = {1.0f, 1.0f, 1.0f};

    // Chain mode state
    int chain_min_radius = 1;
    bool use_cgal_skeleton = true;
    std::vector<SkeletonPointPick> picked_skeleton_points;
    std::vector<std::pair<sinriv::kigstudio::voxel::vec3f,
                          sinriv::kigstudio::voxel::vec3f>>
        skeleton_lines;

    // STL source state
    std::string stl_path;
    int stl_load_mode = 0;
    bool load_as_sdf = false;
    bool use_precise_voxelization = false;
    bool mesh_only = false;
    int source_type = 0;
    int source_node_id = -1;
    int node_source_data_type = 0;
    int node_source_sdf_subdivisions = 2;
    bool node_source_sdf_simplify = false;
    float node_source_sdf_simplify_ratio = 0.1f;
    vec3f silhouette_center = {0.0f, 0.0f, 0.0f};
    bool show_silhouette_center = false;
};

struct MarkedVoxelsSnapshot {
    sinriv::kigstudio::voxel::VoxelGrid marked_voxels;
    std::string description;
};

class RenderVoxelList {
    /*
     * 用于显示一系列窗口
     * 每个子对象由以下部分构成：
     * 一个mesh
     * 一个体素
     * 一个碰撞体，一个空间分割平面（二者只能启动一个）
     * 两个输出结果（被分割为两半）
     */

    std::atomic<int> current_id = 0;
    std::mutex locker;

    std::atomic<float> queue_progress = 0;
    std::atomic<bool> queue_running = false;
    std::atomic<bool> queue_should_continue = true;
    std::string queue_status;
    std::mutex queue_status_mtx;

    bool update_nav_node_status = true;

    // 力导向布局参数
    bool nav_layout_force_directed = true;
    bool nav_layout_initialized = false;
    float nav_layout_repulsion = 16000.0f;  // 同 root 节点间斥力系数
    float nav_layout_repulsion_cross_root =
        24000.0f;                            // 不同 root 节点间斥力系数
    float nav_layout_spring = 0.02f;         // 弹簧系数
    float nav_layout_ideal_length = 120.0f;  // 理想边长
    float nav_layout_center_pull = 0.02f;    // 中心引力系数
    float nav_layout_right_pull = 0.002f;    // 父节点右侧虚拟点引力系数
    float nav_layout_right_offset = 80.0f;   // 父节点右侧虚拟点水平偏移
    float nav_layout_damping = 0.92f;        // 速度阻尼
    float nav_layout_dt = 0.5f;              // 时间步长
    float nav_layout_max_speed = 20.0f;      // 单帧最大速度
    float nav_layout_velocity_threshold =
        0.1f;  // 速度低于此值直接归零，防止微幅抖动
    int nav_layout_velocity_threshold_start_frame = 120;
    int nav_layout_velocity_threshold_frame = 0;

   public:
    class RenderVoxelItem {
        /*
         * TODO:
         * 1.新增功能：生成工作流
         *   允许指定多个入口节点和输出节点，生成一个工作流图
         *   处理流程是：当一个节点所有输入都有数据时，该节点就会执行，向连接节点输出数据
         *   当所有输出节点都有数据时，整个工作流结束
         *   处理完后进行剪枝，成为真正的工作流
         *   生成时，以节点为src的节点视为快照节点，不会使用已有的缓存
         */
       public:
        int id = -1;
        int root_id = -1;
        std::vector<int> children;
        int nav_node_position[2] = {0,
                                    0};  // 在分割演示图中的位置（序列化用快照）
        float nav_layout_pos[2] = {0.0f, 0.0f};  // 力导向浮点位置
        float nav_layout_vel[2] = {0.0f, 0.0f};  // 力导向速度
        bool nav_layout_pinned = false;          // 用户拖动后固定
        bool nav_layout_pos_set = false;         // 是否已有有效初始位置
        std::string err_info;
        std::string title;         // 节点标题（显示在节点编辑器上）
        std::string comment_text;  // 节点注释文本
        RenderVoxelList* manager = nullptr;
        RenderVoxelItem() : ref_count(1), write_count(0) {}
        ~RenderVoxelItem() {
            if (bgfx::isValid(thumbnail_tex)) {
                bgfx::destroy(thumbnail_tex);
            }
        }
        enum SegmentMode {
            COLLISION = 0,
            PLANE = 1,
            CONCAVE_CONE = 2,
            SPLIT_DISCONNECTED = 3,
            NEIGHBOR = 4,
            FILL_INTERIOR = 5,
            CHAIN = 6,
            SDF_NODE_SPLIT = 7
        } segment_mode = COLLISION;

        sinriv::ui::render::RenderMesh origin_mesh_renderer;
        sinriv::ui::render::RenderMesh mesh_renderer;
        sinriv::ui::render::RenderMesh exported_mesh_renderer;
        sinriv::ui::render::RenderVoxel voxel_renderer;
        sinriv::kigstudio::voxel::VoxelGrid voxel_grid_data;
        kdtree::KDTree mesh_kd_tree;  // 三角形顶点的kd树，用于实现自动吸附

        std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> sdf_data;

        std::vector<std::tuple<sinriv::kigstudio::voxel::Triangle,
                               sinriv::kigstudio::voxel::vec3f>>
            cached_mesh;
        bool cached_mesh_dirty = true;

        sinriv::kigstudio::voxel::collision::CollisionGroup collision_group;
        kigstudio::Plane<float> plane;
        kigstudio::voxel::concave::Cone concave_cone;
        std::vector<int> concave_cone_expanded_vertices;

        int sdf_split_target_id = -1;
        vec3f sdf_split_translation = {0.0f, 0.0f, 0.0f};
        vec3f sdf_split_rotation = {0.0f, 0.0f, 0.0f};
        vec3f sdf_split_scale = {1.0f, 1.0f, 1.0f};
        int chain_min_radius = 1;
        struct SurfaceSkeletonCacheEntry {
            sinriv::kigstudio::Vec3i surface_voxel;
            SkeletonPointPick skeleton;
        };
        std::vector<std::pair<sinriv::kigstudio::voxel::vec3f,
                              sinriv::kigstudio::voxel::vec3f>>
            skeleton_lines;
        std::vector<SurfaceSkeletonCacheEntry> surface_skeleton_cache;
        std::vector<SkeletonPointPick> picked_skeleton_points;
        std::vector<SkeletonPointPick> skeleton_order_cache;

        std::vector<mesh_detail::ColorLineVertex> joint_wireframe_vertices;
        bool joint_wireframe_dirty = true;
        void rebuild_joint_wireframe();

        inline void sort_picked_skeleton_points() {
            std::sort(
                picked_skeleton_points.begin(), picked_skeleton_points.end(),
                [](const SkeletonPointPick& a, const SkeletonPointPick& b) {
                    if (a.order != b.order)
                        return a.order < b.order;
                    if (a.position.x != b.position.x)
                        return a.position.x < b.position.x;
                    if (a.position.y != b.position.y)
                        return a.position.y < b.position.y;
                    return a.position.z < b.position.z;
                });
        }

        inline void move_picked_skeleton_point(size_t index, int delta) {
            if (index >= picked_skeleton_points.size() ||
                skeleton_order_cache.empty()) {
                return;
            }

            int next_order = picked_skeleton_points[index].order + delta;
            next_order = std::max(
                0, std::min(next_order,
                            static_cast<int>(skeleton_order_cache.size()) - 1));
            auto& target = picked_skeleton_points[index];
            const auto& source = skeleton_order_cache[next_order];
            target.position = source.position;
            target.order = source.order;
            sort_picked_skeleton_points();
        }

        void render_gbuffer(const float* transform,
                            sinriv::ui::render::RenderMeshShader& mesh_shader);
        void render_overlay(
            sinriv::ui::render::RenderCollision& collision_renderer,
            const float* model_transform,
            const float* model_transform_2,
            sinriv::ui::render::RenderCollisionShader& collision_shader,
            sinriv::ui::render::RenderMeshShader& mesh_shader,
            const mat4f* cpu_model_matrix = nullptr);
        void render_concave_cone_overlay(
            const float* model_transform,
            sinriv::ui::render::RenderMeshShader& mesh_shader);
        void upload_collision(sinriv::ui::render::RenderDeferred& render);

        inline void copy_segment_config_to(RenderVoxelItem& target) const {
            target.segment_mode = segment_mode;
            target.sdf_split_target_id = sdf_split_target_id;
            target.sdf_split_translation = sdf_split_translation;
            target.sdf_split_rotation = sdf_split_rotation;
            target.sdf_split_scale = sdf_split_scale;
            target.collision_group = collision_group;
            target.plane = plane;
            target.concave_cone = concave_cone;
            target.concave_cone_expanded_vertices =
                concave_cone_expanded_vertices;
            target.chain_min_radius = chain_min_radius;
        }

        std::pair<sinriv::kigstudio::voxel::VoxelGrid,
                  sinriv::kigstudio::sdf::SDFBasePtr>
        do_segment_chain() const;

        mat4f sdf_split_transform_matrix() const;

        inline mat4f sdf_split_inverse_transform_matrix() const {
            mat4f inv = sdf_split_transform_matrix();
            inv.invert();
            return inv;
        }

        inline sinriv::kigstudio::sdf::SDFBasePtr transformed_sdf_split_target(
            const sinriv::kigstudio::sdf::SDFBasePtr& target_sdf) const {
            if (!target_sdf) {
                return nullptr;
            }
            return std::make_shared<
                sinriv::kigstudio::sdf::SDF_AffineTransform>(
                sdf_split_inverse_transform_matrix(), target_sdf);
        }

        std::vector<std::tuple<sinriv::kigstudio::voxel::VoxelGrid,
                               sinriv::kigstudio::sdf::SDFBasePtr>>
        do_segment();

        std::atomic<int> ref_count = 1;
        std::atomic<int> write_count = 0;

        bool queue_release = false;

        bool showOriginMesh = false;
        bool showMesh = true;
        bool showExportedMesh = true;
        bool showVoxel = true;
        bool showCollision = true;
        bool showCollisionBounds = false;
        bool showVoxelChunkBounds = false;

        bool auto_segment_update = true;

        // 体素刷选相关
        bool voxel_picking_enabled = false;
        bool surface_cache_ready = false;
        bool surface_cache_computing = false;
        float surface_cache_progress = 0.0f;
        float voxel_pick_range = 3.0f;
        int neighbor_max_distance = 3;
        sinriv::kigstudio::voxel::VoxelGrid surface_voxels;
        sinriv::kigstudio::voxel::VoxelGrid marked_voxels;
        sinriv::ui::render::RenderMesh marked_mesh_renderer;
        bool marked_voxels_dirty = true;

        bgfx::TextureHandle thumbnail_tex = BGFX_INVALID_HANDLE;
        bool thumbnail_dirty = true;

        std::string stl_path;
        std::vector<sinriv::kigstudio::voxel::Triangle> source_triangles;
        bool use_cgal_skeleton = true;
        std::string voxel_path;
        float stl_voxel_size = 1.0f;
        int stl_load_mode = 0;
        bool load_as_sdf = false;
        bool use_precise_voxelization = true;
        bool mesh_only = false;
        int source_type = 0;
        int source_node_id = -1;
        int node_source_data_type = 0;
        int node_source_sdf_subdivisions = 2;
        bool node_source_sdf_simplify = false;
        float node_source_sdf_simplify_ratio = 0.1f;
        vec3f silhouette_center = {0.0f, 0.0f, 0.0f};
        bool showSilhouetteCenter = false;

        // undo/redo stacks for collision editor
        std::vector<CollisionEditorSnapshot> undo_stack;
        std::vector<CollisionEditorSnapshot> redo_stack;

        // undo/redo stacks for marked voxels
        std::vector<MarkedVoxelsSnapshot> marked_undo_stack;
        std::vector<MarkedVoxelsSnapshot> marked_redo_stack;

        bool dirty = false;

        inline void markVoxelChunkDirty(int wx,
                                        int wy,
                                        int wz,
                                        float expand = 0.0f) {
            using namespace sinriv::kigstudio::voxel;
            int cx = wx >> 5, cy = wy >> 5, cz = wz >> 5;
            int lx = wx & 31, ly = wy & 31, lz = wz & 31;
            voxel_renderer.updateChunk(
                voxel_grid_data, packChunkKey(cx, cy, cz), 0.5, true, expand);
            if (lx == 0)
                voxel_renderer.updateChunk(voxel_grid_data,
                                           packChunkKey(cx - 1, cy, cz), 0.5,
                                           true, expand);
            if (lx == 31)
                voxel_renderer.updateChunk(voxel_grid_data,
                                           packChunkKey(cx + 1, cy, cz), 0.5,
                                           true, expand);
            if (ly == 0)
                voxel_renderer.updateChunk(voxel_grid_data,
                                           packChunkKey(cx, cy - 1, cz), 0.5,
                                           true, expand);
            if (ly == 31)
                voxel_renderer.updateChunk(voxel_grid_data,
                                           packChunkKey(cx, cy + 1, cz), 0.5,
                                           true, expand);
            if (lz == 0)
                voxel_renderer.updateChunk(voxel_grid_data,
                                           packChunkKey(cx, cy, cz - 1), 0.5,
                                           true, expand);
            if (lz == 31)
                voxel_renderer.updateChunk(voxel_grid_data,
                                           packChunkKey(cx, cy, cz + 1), 0.5,
                                           true, expand);
        }
    };
    inline RenderVoxelList() { current_model_matrix.setIdentity(); }
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
    float item_status_height = 0;

    bool showOriginMesh = false;
    bool showMesh = true;
    bool showExportedMesh = true;
    bool showVoxels = true;
    bool showCollision = true;

    bool showMeshAxis = false;
    bool showVoxelAxis = false;
    bool showCollisionAxis = false;
    bool showCollisionBounds = true;
    bool showVoxelChunkBounds = false;

    sinriv::kigstudio::voxel::collision::vec3f mouse_world_pos = {0, 0, 0};
    bool mouse_world_pos_valid = false;
    bool mouse_world_pos_picked = false;
    bool mouse_world_pos_picked_auto_snapping = false;  // 自动吸附
    bool disable_camera_on_pick = false;
    float mouse_highlight_range = 3.0f;
    void update_mouse_pos(RenderDeferred& renderer);
    void pick_skeleton_point_from_mouse();

    void render_ui();
    int object_editor_tab = 0;
    int last_object_editor_tab = -1;
    void render_object_editor();
    void render_object_editor_toolbar(RenderVoxelItem& item);
    void copy_node_config(const RenderVoxelItem& item);
    void paste_node_config(RenderVoxelItem& item);
    void render_file_status_tab(RenderVoxelItem& item);
    void render_object_editor_collision_tab_content(RenderVoxelItem& item);
    void render_object_editor_chain_mode(RenderVoxelItem& item);
    void render_object_editor_sdf_node_split_mode(RenderVoxelItem& item);
    void render_object_editor_voxel_tab_content(RenderVoxelItem& item);
    void render_object_editor_comment_tab_content(RenderVoxelItem& item);
    void render_plane_editor(RenderVoxelItem& item);
    void render_collision_body_editor(RenderVoxelItem& item);
    void render_concave_cone_editor(RenderVoxelItem& item);
    void render_nav_map();
    void render_file_loader();
    void render_flow_viewer();

    void render_save_dialog();
    void render_load_dialog();
    void render_import_vxgrid_dialog();

    bool show_edit_segment_plane = false;
    bool show_file_loader = false;
    bool show_import_vxgrid_dialog = false;

    bool show_save_dialog = false;
    bool show_save_as_dialog = false;
    bool show_load_dialog = false;
    bool show_flow_viewer = false;

    // 工作流端点：节点ID + 文件路径
    struct FlowEntry {
        int node_id = -1;
        std::string file_path;
    };

    // 工作流查看器状态
    std::vector<FlowEntry> flow_inputs;
    std::vector<FlowEntry> flow_outputs;
    std::vector<int> flow_cached_order;
    bool flow_needs_recompute = true;

    bool show_delete_confirm = false;
    int pending_delete_item_id = -1;
    bool show_manual_update_confirm = false;

    std::string project_path;

    std::string last_save_error;
    std::string last_load_error;

    size_t memory_current = 0;
    size_t memory_peak = 0;
    float fps = 0;

    // undo/redo
    struct PendingUndo {
        int item_id;
        CollisionEditorSnapshot snapshot;
    };
    std::optional<PendingUndo> pending_undo;
    static constexpr size_t kMaxUndoSize = 50;

    CollisionEditorSnapshot capture_snapshot(const RenderVoxelItem& item) const;
    void apply_snapshot(RenderVoxelItem& item,
                        const CollisionEditorSnapshot& snapshot);

    // 将碰撞编辑器配置序列化为 JSON（用于剪贴板复制/粘贴）
    cJSON* snapshot_to_json(const CollisionEditorSnapshot& snapshot) const;
    std::optional<CollisionEditorSnapshot> snapshot_from_json(
        const cJSON* obj) const;

    void begin_edit(int item_id);
    void end_edit(int item_id, const std::string& desc = "Edit");
    void push_undo_now(
        int item_id,
        const std::optional<CollisionEditorSnapshot>& before = std::nullopt,
        const std::string& desc = "");
    bool undo(int item_id);
    bool redo(int item_id);
    bool can_undo(int item_id) const;
    bool can_redo(int item_id) const;

    // marked voxels undo/redo
    struct PendingMarkedUndo {
        int item_id;
        MarkedVoxelsSnapshot snapshot;
    };
    std::optional<PendingMarkedUndo> pending_marked_undo;
    void begin_marked_edit(int item_id);
    void end_marked_edit(int item_id, const std::string& desc = "Brush");
    void push_marked_undo_now(int item_id, const std::string& desc);
    void undo_marked(int item_id);
    void redo_marked(int item_id);
    bool can_undo_marked(int item_id) const;
    bool can_redo_marked(int item_id) const;
    bool has_dirty_items() const;
    void clear_all_dirty();

    bool show_history_window = false;
    void render_history_window();

    bool show_log_window = false;
    void render_log_window();

    // STL export dialog state (shared between single and batch export)
    int export_stl_mode = 0;  // 0 = Standard, 1 = Smooth SDF
    bool export_stl_simplify = false;
    float export_stl_simplify_ratio = 0.1f;
    int export_stl_subdivisions = 2;
    bool pending_open_export_stl_all_dialog = false;

    struct Icons {
        bgfx::TextureHandle hexagon = BGFX_INVALID_HANDLE;
        bgfx::TextureHandle circles = BGFX_INVALID_HANDLE;
        bgfx::TextureHandle circles_white = BGFX_INVALID_HANDLE;
    } icons;

    void initIcons();
    void destroyIcons();

    struct Debug {
        struct VoxelPickTiming {
            double world_to_voxel_ms = 0.0;
            double iterate_surface_ms = 0.0;
            double mark_voxels_ms = 0.0;
            double total_ms = 0.0;
        };
        bool show_voxel_pick_debug = false;
        std::vector<VoxelPickTiming> voxel_pick_timings;
        size_t max_voxel_pick_timings = 100;
    } debug;
    void render_debug_voxel_pick_window();

    std::vector<std::string> queue_log;
    std::string queue_log_text;
    std::vector<char> queue_log_buffer;
    std::mutex queue_log_mutex;
    inline void append_queue_log(const std::string& msg) {
        std::lock_guard<std::mutex> lock(queue_log_mutex);
        queue_log.push_back(msg);
        if (!queue_log_text.empty())
            queue_log_text += '\n';
        queue_log_text += msg;
        if (queue_log.size() > 1000) {
            queue_log.erase(queue_log.begin(),
                            queue_log.begin() + (queue_log.size() - 1000));
            queue_log_text.clear();
            for (size_t i = 0; i < queue_log.size(); ++i) {
                if (i > 0)
                    queue_log_text += '\n';
                queue_log_text += queue_log[i];
            }
        }
    }
    inline void append_queue_logf(const char* key, ...) {
        std::string fmt = get_locale_string(key);
        char buf[1024];
        va_list args;
        va_start(args, key);
        vsnprintf(buf, sizeof(buf), fmt.c_str(), args);
        va_end(args);
        append_queue_log(buf);
    }

    std::vector<std::tuple<sinriv::kigstudio::voxel::collision::vec3f,
                           sinriv::kigstudio::voxel::collision::vec3f,
                           float>>
        hightlight_pos;

    void upload_collision(sinriv::ui::render::RenderDeferred& render);

    inline void update_mouse() {}

    // 摄像机
    void setViewportSize(int width, int height);
    void setViewProjection(const float* view, const float* proj);
    void setModelMatrix(const mat4f& model_matrix);

    mat4f current_model_matrix;
    void setMeshAxisVisible(bool visible);
    void setVoxelAxisVisible(bool visible);
    void setOriginMeshVisible(bool visible);
    void setMeshVisible(bool visible);
    void setExportedMeshVisible(bool visible);
    void setVoxelsVisible(bool visible);
    void setCollisionVisible(bool visible);
    void setCollisionBoundsVisible(bool visible);
    void setVoxelChunkBoundsVisible(bool visible);

    // 交互
    RenderVoxelItem* create_item();

    std::vector<RenderVoxelItem*> do_segment(int index);
    void extract_skeleton(int index);

    void load_stl(std::string filename,
                  float voxel_size = 0.5f,
                  double isolevel = 0.5,
                  bool smooth_normals = true,
                  int target_item_id = -1,
                  int load_mode = 0,
                  bool load_as_sdf = false,
                  bool use_precise_voxelization = true);
    void load_from_node(int target_item_id,
                        int source_node_id,
                        int node_source_data_type,
                        int node_source_sdf_subdivisions,
                        bool node_source_sdf_simplify,
                        float node_source_sdf_simplify_ratio,
                        int load_mode = 0,
                        bool load_as_sdf = false,
                        bool use_precise_voxelization = true);

    // Cache helpers for node sources
    std::filesystem::path get_cache_dir(const std::string& subdir) const;
    std::filesystem::path get_mesh_cache_path(int node_id) const;
    std::filesystem::path get_sdf_cache_path(int node_id) const;
    std::filesystem::path get_voxel_cache_path(int node_id) const;
    // 执行工作流：加载输入文件→按模板处理→导出输出文件
    void execute_flow();

    std::vector<int> get_process_flow(const std::vector<int>& inputs,
                                      const std::vector<int>& outputs)
        const;  // TODO:用于实现工作流的辅助函数，返回依次被调用的节点id列表

    // 后台队列
    std::vector<std::unique_ptr<RenderVoxelItem>> pending_deletion;
    std::mutex pending_deletion_mutex;

    void process_queue_result();

    void setRenderId(int id);
    void setRenderId_unsafe(int id);

    void brush_marked_voxels(const sinriv::kigstudio::voxel::vec3f& world_pos,
                             float range,
                             bool remove);

    enum QueueTaskType {
        TASK_STOP = 1,
        TASK_REMOVE_ITEM = 2,
        TASK_LOAD_STL = 3,
        TASK_SEGMENT = 4,
        TASK_GENERATE_THUMBNAIL_MESH = 5,
        TASK_RELOAD_STL = 6,
        TASK_CHECK_NON_MANIFOLD = 7,
        TASK_EXTRACT_SKELETON = 8,
        TASK_EXPORT_STL = 9,
        TASK_EXPORT_STL_ALL = 10,
        TASK_EXECUTE_FLOW = 11,
    };
    struct QueueTask {
        QueueTaskType type;
        int index;
        std::string file_path;
        float voxel_size;
        int export_mode = 0;
        bool export_simplify = false;
        float export_simplify_ratio = 0.1f;
        int load_mode = 0;
        bool load_as_sdf = false;
        bool use_precise_voxelization = true;
        int subdivisions = 3;
        bool save_to_file = true;
        int source_node_id = -1;
        int node_source_data_type = 0;
        int node_source_sdf_subdivisions = 2;
        bool node_source_sdf_simplify = false;
        float node_source_sdf_simplify_ratio = 0.1f;
        // 工作流执行
        std::vector<FlowEntry> flow_input_entries;
        std::vector<FlowEntry> flow_output_entries;
    };
    std::queue<QueueTask> queue;
    std::mutex queue_mutex;
    int queue_num = 0;

    std::thread queue_thread_;

    void queue_thread();

    void start_thread();
    void stop_thread();

    std::vector<int> find_roots();
    bool is_descendant_of(int child_id, int ancestor_id);
    bool would_form_source_cycle(int from_id, int to_id);
    void update_nav_node_position();

    size_t get_num_items();
    void queue_load_stl(const std::string& file_path,
                        float voxel_size,
                        int load_mode = 0,
                        bool load_as_sdf = false,
                        bool use_precise_voxelization = true);
    void queue_reload_stl(int item_id,
                          float voxel_size,
                          const std::string& stl_path,
                          int load_mode = 0,
                          bool load_as_sdf = false,
                          bool use_precise_voxelization = true,
                          int source_node_id = -1,
                          int node_source_data_type = 0,
                          int node_source_sdf_subdivisions = 2,
                          bool node_source_sdf_simplify = false,
                          float node_source_sdf_simplify_ratio = 0.1f);
    void queue_do_segment(int index);
    void queue_do_segment();
    void queue_do_segment_unsafe();
    void queue_remove_item(int index);
    void queue_check_non_manifold(int index);
    void queue_extract_skeleton(int index);
    void queue_export_stl(int item_id,
                          const std::string& file_path,
                          int mode,
                          bool simplify,
                          float ratio,
                          int subdivisions,
                          bool save_to_file = true);
    void queue_export_stl_all(const std::string& export_dir,
                              int mode,
                              bool simplify,
                              float ratio,
                              int subdivisions,
                              bool save_to_file = true);
    bool isQueueRunning();
    std::string getQueueStatus();
    void setQueueStatus(const std::string& status);
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
