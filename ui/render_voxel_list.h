#pragma once
#include <cJSON.h>
#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
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

struct CollisionEditorSnapshot {
    sinriv::kigstudio::voxel::collision::CollisionGroup collision_group;
    sinriv::kigstudio::Plane<float> plane;
    sinriv::kigstudio::voxel::concave::Cone concave_cone;
    std::vector<int> concave_cone_expanded_vertices;
    int segment_mode;
    std::string description;

    // Chain mode state
    int chain_min_radius = 1;
    bool use_cgal_skeleton = true;
    std::vector<SkeletonPointPick> picked_skeleton_points;
    std::vector<std::pair<sinriv::kigstudio::voxel::vec3f,
                          sinriv::kigstudio::voxel::vec3f>>
        skeleton_lines;
};

struct MarkedVoxelsSnapshot {
    sinriv::kigstudio::voxel::VoxelGrid marked_voxels;
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

    std::atomic<float> queue_progress = 0;
    std::atomic<bool> queue_running = false;
    std::atomic<bool> queue_should_continue = true;
    std::string queue_status;
    std::mutex queue_status_mtx;

    bool update_nav_node_status = true;

   public:
    class RenderVoxelItem {
       public:
        int id = -1;
        int root_id = -1;
        std::vector<int> children;
        int nav_node_position[2] = {0, 0};  // 在分割演示图中的位置
        std::string err_info;
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
            CHAIN = 6
        } segment_mode = COLLISION;

        sinriv::ui::render::RenderMesh mesh_renderer;
        sinriv::ui::render::RenderVoxel voxel_renderer;
        sinriv::kigstudio::voxel::VoxelGrid voxel_grid_data;
        kdtree::KDTree mesh_kd_tree;  // 三角形顶点的kd树，用于实现自动吸附

        std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> sdf_data;

        sinriv::kigstudio::voxel::collision::CollisionGroup collision_group;
        kigstudio::Plane<float> plane;
        kigstudio::voxel::concave::Cone concave_cone;
        std::vector<int> concave_cone_expanded_vertices;

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

        inline std::vector<std::tuple<sinriv::kigstudio::voxel::VoxelGrid,
                                      sinriv::kigstudio::sdf::SDFBasePtr>>
        do_segment() {
            if (segment_mode == COLLISION) {
                auto res = voxel_grid_data.segment(collision_group);
                return {{std::move(std::get<0>(res)), nullptr},
                        {std::move(std::get<1>(res)), nullptr}};
            } else if (segment_mode == PLANE) {
                auto res = voxel_grid_data.segment(plane);
                sinriv::kigstudio::sdf::SDFBasePtr left_sdf = nullptr;
                sinriv::kigstudio::sdf::SDFBasePtr right_sdf = nullptr;
                if (sdf_data) {
                    // Create SDF for both split sides
                    auto plane_sdf =
                        std::shared_ptr<sinriv::kigstudio::sdf::SDF_Plane>(
                            new sinriv::kigstudio::sdf::SDF_Plane(
                                plane.A, plane.B, plane.C, plane.D));
                    auto plane_sdf_neg =
                        std::shared_ptr<sinriv::kigstudio::sdf::SDF_Plane>(
                            new sinriv::kigstudio::sdf::SDF_Plane(
                                -plane.A, -plane.B, -plane.C, -plane.D));
                    left_sdf = sinriv::kigstudio::sdf::sdf_intersection(
                        sdf_data, plane_sdf);
                    right_sdf = sinriv::kigstudio::sdf::sdf_intersection(
                        sdf_data, plane_sdf_neg);
                }
                return {{std::move(std::get<0>(res)), std::move(right_sdf)},
                        {std::move(std::get<1>(res)), std::move(left_sdf)}};
            } else if (segment_mode == CONCAVE_CONE) {
                auto res = voxel_grid_data.segment(concave_cone);
                sinriv::kigstudio::sdf::SDFBasePtr left_sdf = nullptr;
                sinriv::kigstudio::sdf::SDFBasePtr right_sdf = nullptr;
                if (sdf_data) {
                    auto cone_sdf =
                        sinriv::kigstudio::sdf::to_sdf(concave_cone);
                    left_sdf = sinriv::kigstudio::sdf::sdf_subtraction(
                        sdf_data, cone_sdf);
                    right_sdf = sinriv::kigstudio::sdf::sdf_intersection(
                        sdf_data, cone_sdf);
                }
                return {{std::move(std::get<0>(res)), std::move(right_sdf)},
                        {std::move(std::get<1>(res)), std::move(left_sdf)}};
            } else if (segment_mode == SPLIT_DISCONNECTED) {
                auto splits = voxel_grid_data.splitDisconnected(true);
                std::vector<std::tuple<sinriv::kigstudio::voxel::VoxelGrid,
                                       sinriv::kigstudio::sdf::SDFBasePtr>>
                    result;
                result.reserve(splits.size());
                for (auto& grid : splits) {
                    result.emplace_back(std::move(grid), nullptr);
                }
                return result;
            } else if (segment_mode == NEIGHBOR) {
                std::vector<sinriv::kigstudio::Vec3i> seeds;
                for (const auto& v : marked_voxels) {
                    seeds.push_back(v);
                }
                auto res = voxel_grid_data.bfsSplit(
                    seeds, neighbor_max_distance, true);
                return {{std::move(std::get<0>(res)), nullptr},
                        {std::move(std::get<1>(res)), nullptr}};
            } else if (segment_mode == FILL_INTERIOR) {
                auto filled = voxel_grid_data.fillInterior(true);
                return {{std::move(filled), nullptr}};
            } else if (segment_mode == CHAIN) {
                return {do_segment_chain()};
            } else {
                throw std::runtime_error("Unknown method");
            }
        }

        std::atomic<int> ref_count = 1;
        std::atomic<int> write_count = 0;

        bool queue_release = false;

        bool showMesh = true;
        bool showVoxel = true;
        bool showCollision = true;
        bool showCollisionBounds = false;

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
    void render_object_editor_collision_tab_content(RenderVoxelItem& item);
    void render_object_editor_chain_mode(RenderVoxelItem& item);
    void render_object_editor_voxel_tab_content(RenderVoxelItem& item);
    void render_plane_editor(RenderVoxelItem& item);
    void render_collision_body_editor(RenderVoxelItem& item);
    void render_concave_cone_editor(RenderVoxelItem& item);
    void render_nav_map();
    void render_file_loader();
    void render_reload_stl_dialog();
    void render_save_dialog();
    void render_load_dialog();
    void render_import_vxgrid_dialog();

    bool show_edit_segment_plane = false;
    bool show_file_loader = false;
    bool show_import_vxgrid_dialog = false;
    bool show_reload_stl_dialog = false;
    int reload_stl_item_id = -1;
    float reload_stl_voxel_size = 1.0f;
    bool load_as_sdf = false;
    bool show_save_dialog = false;
    bool show_save_as_dialog = false;
    bool show_load_dialog = false;

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
    bool export_stl_simplify = true;
    float export_stl_simplify_ratio = 0.1f;
    int export_stl_subdivisions = 3;
    bool pending_open_export_stl_all_dialog = false;

    struct Icons {
        bgfx::TextureHandle hexagon = BGFX_INVALID_HANDLE;
        bgfx::TextureHandle circles = BGFX_INVALID_HANDLE;
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

    std::vector<RenderVoxelItem*> do_segment(int index);
    void extract_skeleton(int index);

    void load_stl(std::string filename,
                  float voxel_size = 0.5f,
                  double isolevel = 0.5,
                  bool smooth_normals = true,
                  int target_item_id = -1,
                  bool load_as_sdf = false);

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
    };
    struct QueueTask {
        QueueTaskType type;
        int index;
        std::string file_path;
        float voxel_size;
        int export_mode = 0;
        bool export_simplify = false;
        float export_simplify_ratio = 0.1f;
        bool load_as_sdf = false;
        int subdivisions = 3;
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
    void queue_load_stl(const std::string& file_path,
                        float voxel_size,
                        bool load_as_sdf = false);
    void queue_reload_stl(int item_id,
                          float voxel_size,
                          const std::string& stl_path,
                          bool load_as_sdf = false);
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
                          int subdivisions);
    void queue_export_stl_all(const std::string& export_dir,
                              int mode,
                              bool simplify,
                              float ratio,
                              int subdivisions);
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
