#pragma once
#include <cJSON.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <deque>
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
#include <memory>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#include "kigstudio/sdf/sdf_mesh.h"
#include "kigstudio/sdf/sdf_shape.h"
#include "kigstudio/ui/render_collision.h"
#include "kigstudio/ui/render_mesh.h"
#include "kigstudio/ui/render_voxel.h"
#include "kigstudio/utils/KDTree.h"
#include "kigstudio/utils/locale.h"
#include "kigstudio/utils/plane.h"
#include "kigstudio/utils/process.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/voxel/concave.h"
#include "kigstudio/voxel/voxel.h"
#include "kigstudio/voxel/voxel_EDT.h"
#include "kigstudio/voxel/voxelizer_svo.h"
#include "kigstudio/voxel/triangle_bvh.h"
#include "ui/cross_section_editor.h"
#include "ui/render_deferred.h"

#include "kigstudio/agent/agent_server.h"

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

// 附加件类型
enum class AddonType : int {
    HAIR = 0,  // 毛发
    COUNT
};

// 发束生成类型
enum class HairStrandGenType : int {
    NORMAL = 0,           // 截面放样（现有行为）
    CANDIED_HAWTHORN = 1, // 糖葫芦：圆柱 + 椭球 + 关节 + 桃形尖端
    BRAID = 2,            // 麻花辫：核心 + 编织股 + 关节 + 桃形尖端
};

// 一根发束
struct HairStrand {
    std::string name;
    /// Stable unique identifier (12 hex chars), generated at creation.
    /// Used as the key in addon_renderers map and for active strand tracking.
    std::string uuid;
    // 引导曲线上的拾取点（世界坐标）
    std::vector<sinriv::kigstudio::voxel::vec3f> guide_points;

    // Special hidden guide points at strand start/end.
    // These participate in lofting and width-vector processing but are NOT
    // editable in the UI, NOT shown in export images, and NOT visible in the
    // orthogonal editor. They ARE rendered in the 3D model editor (gray color).
    std::vector<sinriv::kigstudio::voxel::vec3f> hidden_guide_points_start;
    std::vector<sinriv::kigstudio::voxel::vec3f> hidden_guide_points_end;

    // UI 折叠状态
    bool expanded = true;
    // 显示/隐藏此发束（仅影响显示，不影响碰撞生成）
    bool visible = true;

    // 宽度编辑：每个宽度点对应引导曲线上的一个位置
    // 只保存相对于引导曲线的数据（曲线id + 方向向量），不保存绝对坐标，
    // 这样编辑引导曲线时绿线会自动更新
    // curve_id: 整数部分=贝塞尔段索引，小数部分=段内参数t（0~1）
    struct WidthPoint {
        float curve_id = 0.0f;       // 曲线上的位置（int=段索引, frac=段内t）
        float scale = 1.0f;          // 宽度缩放（原始距离 = 绿线长度）
        sinriv::kigstudio::voxel::vec3f direction{};  // 从曲线指向底模的方向（单位向量）
        // Per-point independent section override (non-empty vertices = custom section)
        SectionEditorState section_state;
    };
    std::vector<WidthPoint> width_points;
    bool width_editing_active = false;

    // Cross-section path (2D closed polygon) and independent undo/redo state
    SectionEditorState section_state;

    // Section rotation around the guide curve tangent (-180 to 180 degrees)
    float section_rotation = 0.0f;

    // 细分精度：引导曲线贝塞尔插值每段采样数
    int guide_samples_per_segment = 32;
    // 细分精度：截面贝塞尔（Catmull-Rom 平滑）每条边细分数
    int section_subdiv = 8;

    // Alpha wrap repair parameters (per-strand, shown inline below section
    // precision). When the loft mesh is not boolean-ready, alpha_wrap
    // is applied with these values before rendering.
    float repair_alpha = 1.0f;
    float repair_offset = 0.01f;

    // Set to true when the loft mesh needs repair but alpha_wrap fails.
    // Reset on every rebuild; shown as a warning indicator in the UI.
    bool repair_failed = false;

    // Hair root edit mode: enable this strand's root point visualization.
    // When enabled, the common hair root point is prepended to
    // hidden_guide_points_start during lofting.
    bool hair_root_enabled = true;

    // When true, lofting injects a synthetic short width vector at the
    // strand start (curve_id 0, covering the hidden gray root region),
    // with length = RenderVoxelItem::hair_root_vector_length and the same
    // direction as the first user width vector. No-op when width_points
    // is empty (empty mesh).
    bool hair_root_generate = false;

    // When true, WidthPoint::curve_id references the full guide curve
    // (hidden_guide_points_start + guide_points + hidden_guide_points_end).
    // When false, curve_id references visible guide_points only (legacy).
    // Set to true after one-time migration in build_hair_strand_mesh.
    bool width_curve_id_v2 = false;

    // Dirty flag: set to true when any data affecting the loft mesh changes
    bool mesh_dirty = true;

    // Strand generation type (default NORMAL for backward compatibility)
    HairStrandGenType gen_type = HairStrandGenType::NORMAL;

    // ---- 糖葫芦 parameters ----
    float candy_cylinder_radius = 1.5f;       // Core cylinder radius
    float candy_ellipsoid_spacing = 8.0f;     // Distance between ellipsoids
    float candy_ellipsoid_radius_a = 3.0f;    // Short axis radius (perpendicular to curve)
    float candy_ellipsoid_radius_b = 5.0f;    // Long axis radius (along curve tangent)
    bool  candy_use_joints = false;           // Simplified joint rings

    // ---- 麻花辫 parameters ----
    float braid_core_radius = 1.0f;           // Center core cylinder radius
    float braid_strand_radius = 0.7f;         // Each outer strand radius
    float braid_braid_radius = 2.5f;          // Distance of outer strands from center
    float braid_twist_pitch = 30.0f;          // Length for one full 360° twist
    int   braid_strand_count = 3;             // Number of outer strands (2-6)
    bool  braid_use_joints = false;           // Simplified joint rings

    // ---- Tip (shared by both special types) ----
    float special_tip_length = 4.0f;          // Teardrop cone length beyond sphere
    float special_tip_radius = 2.0f;          // Teardrop sphere radius

    // ---- Tessellation quality (shared by both special types) ----
    int special_quality = 16;                 // Polygon segments (4-64), controls
                                              // cylinder/ellipsoid/cone/joint smoothness
};

/// Generate a random 12-char hex string for strand UUIDs.
/// Thread-local mt19937 seeded from steady_clock XOR thread::id hash.
/// Collision probability is negligible for the expected strand count per file.
inline std::string generate_uuid() {
    thread_local std::mt19937 rng(
        static_cast<unsigned>(
            std::chrono::steady_clock::now()
                .time_since_epoch()
                .count()) ^
        std::hash<std::thread::id>{}(std::this_thread::get_id()));
    std::uniform_int_distribution<int> dist(0, 15);
    const char hex[] = "0123456789abcdef";
    std::string id;
    id.reserve(12);
    for (int i = 0; i < 12; ++i)
        id += hex[dist(rng)];
    return id;
}

/// Orthographic projection edit mode state for hair strand editing.
/// Holds projection settings, rendered textures, overlay image state,
/// and interaction flags for the 2D ortho view editor.

// Dedicated bgfx view IDs for ortho projection off-screen renders
// (must not conflict with main pipeline 0-5, thumbnails 100-101)
constexpr bgfx::ViewId kOrthoViewView = 200;
constexpr bgfx::ViewId kOrthoCoordView = 201;
constexpr bgfx::ViewId kOrthoOverlayView = 202;
constexpr bgfx::ViewId kOrthoBlitView = 203;

/// Per-view overlay state persisted per-node for each of the six standard
/// orthographic views.  GPU handles are NOT stored here — they are
/// recreated from the image file on restore.
struct OrthoOverlayState {
    std::string image_path;
    int img_width = 0;
    int img_height = 0;
    bool enabled = false;
    float offset_x = 0.0f;
    float offset_y = 0.0f;
    float scale_x = 1.0f;
    float scale_y = 1.0f;
    float blend_ratio = 0.5f;
    bool locked = false;
};

struct OrthoProjectionState {
    bool active = false;              // master flag for edit mode

    // Setup window
    vec3f projection_dir = {0, 1, 0}; // default: top-down view
    float viewport_size = 100.0f;     // world-space side length
    bool viewport_size_defaulted = false;  // true after auto-calculation
    int vector_mode = 0;              // 0 = six-view, 1 = pick point on model
    int six_view_index = 0;           // 0-5: Front/Back/Left/Right/Top/Bottom
    bool is_picking_point = false;    // waiting for 3D click for point-pick mode

    // Edit window
    bool edit_window_open = false;
    int render_resolution = 2048;
    bool render_dirty = true;
    bool coord_map_ready = false;

    // CPU-side ortho camera params (set by perform_ortho_render)
    vec3f _cam_right = {1, 0, 0};
    vec3f _cam_up = {0, 1, 0};
    vec3f _cam_pos = {0, 0, 0};
    vec3f _center = {0, 0, 0};

    // CPU-side base model triangles for raycasting
    std::vector<sinriv::kigstudio::voxel::Triangle> _base_triangles;
    size_t _base_triangle_count = 0;   // for detecting base-model changes

    // GPU off-screen render for view image (multi-frame state machine)
    // High-res framebuffer (user-chosen resolution, used for AI export)
    bgfx::FrameBufferHandle view_fb = BGFX_INVALID_HANDLE;
    bgfx::TextureHandle view_tex = BGFX_INVALID_HANDLE;
    bgfx::TextureHandle view_depth_tex = BGFX_INVALID_HANDLE;
    bool view_tex_ready = false;
    int ortho_render_stage = 0;  // 0=IDLE, 1=RENDER, 2=WAIT, 3=DONE
    int ortho_wait_frames = 0;
    int ortho_base_item_id = -1;

    // Reference image overlay
    std::string overlay_image_path;
    bgfx::TextureHandle overlay_tex = BGFX_INVALID_HANDLE;
    int overlay_img_width = 0;
    int overlay_img_height = 0;
    bool overlay_enabled = false;       // checkbox to activate drag/scale
    ImVec2 overlay_offset = {0, 0};     // pan offset in screen pixels
    float overlay_scale_x = 1.0f;       // zoom scale X in screen pixels
    float overlay_scale_y = 1.0f;       // zoom scale Y in screen pixels
    float blend_ratio = 0.5f;           // blend slider
    float canvas_display_size = 600.0f; // display size when overlay was placed (for scaling)

    // Strand preview toggles
    bool show_guide_curves = true;
    bool show_width_vectors = true;

    // Export guide curves overlay on output image
    bool export_show_guide_curves = false;
    bool export_color_code_strands = true;

    // Depth-based colouring mode
    // 0=Contour, 1=Depth, 2=Lighting
    int ortho_render_mode = 0;

    // Interaction state
    bool overlay_locked = false;         // lock overlay from drag/resize
    bool is_dragging_overlay = false;
    ImVec2 drag_start_mouse;
    ImVec2 drag_start_offset;
    int resize_corner = -1;            // -1=none, 0=TL, 1=TR, 2=BL, 3=BR
    ImVec2 resize_start_mouse;
    ImVec2 resize_start_offset;
    float resize_start_scale_x = 1.0f;
    float resize_start_scale_y = 1.0f;
    bool is_hovering_model = false;     // mouse over valid mesh area
    bool mouse_in_image = false;       // mouse currently inside the render canvas
    vec3f hovered_world_pos = {0, 0, 0};
    int hovered_px = 0, hovered_py = 0; // API 2D render-pixel coords

    // ---- AI / external tool integration ----
    bool api_render_dirty = false;      // new render → trigger GPU readback for API cache
    bool ai_export_pending = false;     // readback pipeline is active
    // GPU readback state for saving the render to a PNG file on disk
    bgfx::TextureHandle ai_readback_tex = BGFX_INVALID_HANDLE;
    std::vector<uint8_t> ai_readback_buffer;
    bool ai_readback_pending = false;
    int ai_export_stage = 0;            // 0=idle, 1=wait-readback, 2=wait-frames-then-save
    int ai_readback_frame_wait = 0;     // countdown frames until GPU readback is ready
};

// Six standard view directions based on the semantic coordinate frame.
// Front/Back follow hair_front_reference; Top/Bottom follow
// hair_north_pole; Left/Right are derived via cross product.
inline vec3f six_view_direction(int index,
                                const sinriv::kigstudio::voxel::vec3f& front_ref,
                                const sinriv::kigstudio::voxel::vec3f& north_pole) {
    vec3f F = {front_ref.x, front_ref.y, front_ref.z};
    vec3f N = {north_pole.x, north_pole.y, north_pole.z};
    float fl = std::sqrt(F.x*F.x + F.y*F.y + F.z*F.z);
    float nl = std::sqrt(N.x*N.x + N.y*N.y + N.z*N.z);
    if (fl > 1e-8f) { F.x /= fl; F.y /= fl; F.z /= fl; }
    if (nl > 1e-8f) { N.x /= nl; N.y /= nl; N.z /= nl; }
    // Right = normalize(cross(N, F)) — with F toward the nose and N up,
    // this points toward the model's RIGHT side in a RH system.
    // (The variable retains the name 'R' from the original code.)
    vec3f R = {N.y * F.z - N.z * F.y,
               N.z * F.x - N.x * F.z,
               N.x * F.y - N.y * F.x};
    float rl = std::sqrt(R.x*R.x + R.y*R.y + R.z*R.z);
    if (rl > 1e-8f) { R.x /= rl; R.y /= rl; R.z /= rl; }
    // Direction convention: from center toward camera position (outward).
    // Camera is placed at center + projection_dir * 1000.
    // Front: camera in front of face → -F
    // Back:  camera behind head      → +F
    // Left:  camera on left side     → +R
    // Right: camera on right side    → -R
    // Top:   camera above head       → -N
    // Bottom:camera below head       → +N
    switch (index) {
        case 0: return {-F.x, -F.y, -F.z};  // Front
        case 1: return F;             // Back
        case 2: return R;             // Left
        case 3: return {-R.x, -R.y, -R.z};  // Right
        case 4: return {-N.x, -N.y, -N.z};  // Top
        case 5: return N;             // Bottom
        default: return {-N.x, -N.y, -N.z};
    }
}

/// Per-position angle configuration for semantic-coordinate ray casting.
/// Maps a semantic (X, Y) position to spherical angles that define the
/// direction from which a ray is cast toward the center point.
struct HairAngleEntry {
    float theta = 0.0f;  // azimuth (deg): 0=front, +90=right
    float phi = 45.0f;   // polar from horizontal (deg): 0=level, +90=up
};

// Result of sampling the guide curve at a given curve_id
struct GuideCurveSample {
    sinriv::kigstudio::voxel::vec3f position{};
    sinriv::kigstudio::voxel::vec3f tangent{};
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
                             bool normalize = false,
                             bool same_line = false);
EditResult edit_local_position_stepper(const char* label,
                                       vec3f& value,
                                       float step = 0.5f,
                                       bool normalize = false,
                                       bool show_label = true);
EditResult edit_transform_controls(Transform& transform);
// 计算截面自动旋转角度：给定引导曲线上一点 point、切线 tangent 与中心点
// center，返回使中心点位于曲线方向与截面正下方所构成平面上的旋转角
// （度，[-180,180]）。退化情况（向量与切线平行）返回 false。
bool compute_auto_section_rotation(const vec3f& point,
                                   const vec3f& tangent,
                                   const vec3f& center,
                                   float& out_angle_deg);
// Bezier curve utilities (defined in render_voxel_render.cpp)
sinriv::kigstudio::voxel::vec3f bezier_eval(
    const sinriv::kigstudio::voxel::vec3f& p0,
    const sinriv::kigstudio::voxel::vec3f& p1,
    const sinriv::kigstudio::voxel::vec3f& p2,
    const sinriv::kigstudio::voxel::vec3f& p3, float t);
std::vector<sinriv::kigstudio::voxel::vec3f> sample_bezier_guide_curve(
    const std::vector<sinriv::kigstudio::voxel::vec3f>& guide_points,
    int samples_per_segment = 32);

// Draw guide curves on an RGBA pixel buffer using ortho camera projection.
// Called from the API server to overlay guide curves on exported images.
// line_thickness: width of guide curve lines in pixels (default 1).
// font_size: pixel height for strand name labels (0 = no labels).
void draw_guide_curves_on_buffer(
    std::vector<uint8_t>& rgba, int w, int h,
    const OrthoProjectionState& ortho_state,
    const std::vector<HairStrand>& hair_strands,
    bool color_code,
    int line_thickness = 1,
    float font_size = 0.0f);

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

enum class SilhouetteShapeMode : int {
    ICOSAHEDRON = 0,     // subdivided icosahedron (classic)
    DELAUNAY_SPHERE = 1, // Delaunay triangulation of input vertices on sphere
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
    sinriv::kigstudio::sdf::SDFPrecision voxel_precision =
        sinriv::kigstudio::sdf::SDFPrecision::Fast;
    sinriv::kigstudio::sdf::SDFPrecision sdf_precision_cache =
        sinriv::kigstudio::sdf::SDFPrecision::Precise;
    bool mesh_only = false;
    int source_type = 0;
    int source_node_id = -1;
    int node_source_data_type = 0;
    int node_source_sdf_subdivisions = 2;
    bool node_source_sdf_simplify = false;
    float node_source_sdf_simplify_ratio = 0.1f;
    vec3f silhouette_center = {0.0f, 0.0f, 0.0f};
    bool show_silhouette_center = false;
    // 附加件中心点（所有发束共享）
    vec3f addon_center_point = {0.0f, 0.0f, 0.0f};
    bool show_addon_center = false;
    // 发根编辑共享状态
    bool auto_hair_root = false;
    vec3f common_hair_root_point = {0.0f, 0.0f, 0.0f};
    float hair_root_center_offset = 0.0f;
    // 发根处生成的短宽度向量长度（用于"生成发根"放样）
    float hair_root_vector_length = 0.1f;
    // 发际线平面（用于纺锤宽度生成）
    bool hairline_plane_enabled = false;
    bool hairline_plane_use_y = true;
    float hairline_plane_y = 0.0f;
    vec3f hairline_plane_points[3] = {};
    float hairline_spindle_scale = 0.8f;
    SilhouetteShapeMode silhouette_shape_mode = SilhouetteShapeMode::DELAUNAY_SPHERE;
    int silhouette_subdivision = 4;
    int silhouette_edge_subdiv = 0;
    float inner_wall_radius = 0.0f;
    float simplify_ratio = -1.0f;  // negative = disabled
    int repair_mode = 0;
    float alpha_wrap_alpha = 1.0f;
    float alpha_wrap_offset = 0.01f;
    int subdivide_level = 1;

    // Hair strand state (guide curves, width points)
    std::vector<HairStrand> hair_strands;

    // Addon (hair) boolean mode flags
    bool addon_reveal = false;
    bool addon_split = false;
    bool addon_sdf_boolean = true;
    bool addon_sdf_split = true;

    // Semantic coordinate angle config
    std::map<std::pair<float, float>, HairAngleEntry> hair_angle_config;
    sinriv::kigstudio::voxel::vec3f hair_north_pole = {0.0f, 1.0f, 0.0f};
    sinriv::kigstudio::voxel::vec3f hair_front_reference = {0.0f, 0.0f, 1.0f};
    int addon_base_node_id = -1;
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
    std::mutex locker;
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
            SDF_NODE_SPLIT = 7,
            SUBDIVIDE_MESH = 8, // 细分三角形网格，输出更密的 mesh
            REPAIR_MESH = 9, // 这是处理mesh的专用模式，不会输出体素和SDF
            SILHOUETTE = 10, // 锥化：由输入网格生成封闭轮廓网格，不输出体素和SDF
        } segment_mode = COLLISION;

        enum RepairMeshMode {
            ALPHA_WRAP = 0,
            FILL_HOLES = 1,
            STITCH_BORDERS = 2,
            MERGE_DUPLICATE_VERTICES = 3,
            ORIENT_VOLUME = 4
        } repair_mode = FILL_HOLES;
        float alpha_wrap_alpha = 1.0f;
        float alpha_wrap_offset = 0.01f;

        int subdivide_level = 1;  // 1 = 最粗，数值越大网格越密

        sinriv::ui::render::RenderMesh origin_mesh_renderer;
        sinriv::ui::render::RenderMesh mesh_renderer;
        sinriv::ui::render::RenderMesh exported_mesh_renderer;
        // 附加件渲染器（如毛发预览）：按发束 UUID 索引，支持增量更新。
        // RenderMesh 持有 bgfx 句柄且不可拷贝/移动，用 unique_ptr 管理。
        std::unordered_map<std::string,
                           std::unique_ptr<sinriv::ui::render::RenderMesh>>
            addon_renderers;

        // 附加件模式：选中的底模节点ID（-1表示未选择）
        int addon_base_node_id = -1;
        // 附加件类型（0=毛发）
        int addon_type = 0;
        // 附加件碰撞选项
        bool addon_reveal = false;  // 显露：SDF减去底模
        bool addon_split = false;   // 拆分：每根发束独立节点
        // 显露时勾选=用SDF减底模，未勾选=用几何布尔减底模
        bool addon_sdf_boolean = true;
        // 拆分时勾选=发束之间用SDF相减，未勾选=用几何布尔相减
        bool addon_sdf_split = true;
        // 附加件中心点（所有发束共享），用于发根汇聚与反翘控制
        vec3f addon_center_point = {0.0f, 0.0f, 0.0f};
        bool show_addon_center = false;  // 是否显示/启用中心点
        // 发根编辑模式（显示紫色发根圈）
        bool hair_root_edit_active = false;
        // 自动发根引导点：从头顶方向向底模投射的共享隐藏引导点
        bool auto_hair_root = false;
        vec3f common_hair_root_point = {0.0f, 0.0f, 0.0f};
        float hair_root_center_offset = 0.0f;  // 发根点向中心点移动的距离
        // 发根处生成的短宽度向量长度（"生成发根"放样用）
        float hair_root_vector_length = 0.1f;
        // Ortho occlusion cache: avoid recomputing per-strand visibility every frame
        size_t _ortho_occlusion_hash = 0;
        std::vector<bool> _ortho_strand_occluded;
        std::vector<std::vector<bool>> _ortho_point_occluded;
        // Per-six-view ortho overlay state (saved to JSON per-node)
        OrthoOverlayState ortho_overlay[6];
        // Ortho editor global settings (persisted per-node)
        float ortho_viewport_size = 0.0f;      // 0 = auto-calculate on next open
        int ortho_render_resolution = 0;       // 0 = use default (2048)
        // 发际线平面（用于纺锤宽度生成）
        bool hairline_plane_enabled = false;
        bool hairline_plane_use_y = true;  // true=Y水平面, false=三点平面
        float hairline_plane_y = 0.0f;
        vec3f hairline_plane_points[3] = {};
        // Hairline spindle scale factor: nearest-neighbor distance at hairline
        // intersection is multiplied by this before applying as width.
        float hairline_spindle_scale = 0.8f;
        // 应用发际线纺锤宽度：根据引导线与发际线的交点
        // 自动生成宽度向量，在两段收束形成纺锤形
        void apply_hairline_spindle();
        // 毛发数据
        std::vector<HairStrand> hair_strands;

        /// Find a strand by UUID (O(n) linear search). Returns nullptr if not found.
        HairStrand* find_strand_by_uuid(const std::string& id) {
            for (auto& s : hair_strands)
                if (s.uuid == id) return &s;
            return nullptr;
        }
        const HairStrand* find_strand_by_uuid(const std::string& id) const {
            for (const auto& s : hair_strands)
                if (s.uuid == id) return &s;
            return nullptr;
        }

        /// Rename a strand: assign a new UUID and move the addon_renderers entry.
        /// Active-tracking fields are updated if they reference the old UUID.
        void rename_strand(const std::string& old_uuid,
                           const std::string& new_uuid) {
            auto* s = find_strand_by_uuid(old_uuid);
            if (!s) return;
            s->uuid = new_uuid;
            auto it = addon_renderers.find(old_uuid);
            if (it != addon_renderers.end()) {
                auto node = addon_renderers.extract(it);
                node.key() = new_uuid;
                addon_renderers.insert(std::move(node));
            }
            if (active_guide_draw_strand == old_uuid)
                active_guide_draw_strand = new_uuid;
            if (active_width_edit_strand == old_uuid)
                active_width_edit_strand = new_uuid;
            if (active_section_edit_strand == old_uuid)
                active_section_edit_strand = new_uuid;
            if (active_perpoint_section_edit_strand == old_uuid)
                active_perpoint_section_edit_strand = new_uuid;
        }

        // 当前正在绘制引导曲线的发束 UUID（空=无）
        std::string active_guide_draw_strand;   // empty = none
        bool guide_curve_drawing_active = false;
        // 当前正在编辑宽度的发束 UUID（空=无）
        std::string active_width_edit_strand;   // empty = none
        bool width_editing_active = false;
        // Width point index highlighted in 3D viewport (hovered in width editor UI)
        int hovered_width_point_index = -1;
        // Strand UUID highlighted red when hovered in addon editor strand list
        std::string hovered_strand_uuid;  // empty = none
        // Guide point highlighted red when hovered in guide curve editor
        std::string hovered_guide_point_strand_uuid;  // empty = none
        int hovered_guide_point_index = -1;
        // 当前正在编辑截面的发束 UUID（空=无）
        std::string active_section_edit_strand;  // empty = none
        // Per-point section editor state
        std::string active_perpoint_section_edit_strand;  // empty = none
        int active_perpoint_section_edit_width_idx = -1;
        bool perpoint_section_editing_active = false;
        // 发际线三点拾取状态
        bool hairline_point_picking_active = false;
        int hairline_picking_point_index = 0;  // 0, 1, or 2
        sinriv::ui::render::RenderVoxel voxel_renderer;
        sinriv::kigstudio::voxel::VoxelGrid voxel_grid_data;
        kdtree::KDTree mesh_kd_tree;  // 三角形顶点的kd树，用于实现自动吸附

        std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> sdf_data;

        std::vector<std::tuple<sinriv::kigstudio::voxel::Triangle,
                               sinriv::kigstudio::voxel::vec3f>>
            cached_mesh;
        bool cached_mesh_dirty = true;
        bool exported_mesh_synced = false;  // renderer sync flag (separate from cached_mesh_dirty)

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

        // 宽度编辑：在引导曲线上查找离 world_pos 最近的点并添加 WidthPoint
        void add_width_point_at(int strand_idx,
                                const sinriv::kigstudio::voxel::vec3f& world_pos);

        // 在引导曲线上查找 curve_id 对应的世界坐标位置与切线方向
        GuideCurveSample sample_guide_curve_at(int strand_idx,
                                               float curve_id) const;

        // Rebuild addon meshes from hair strand data (called from render_gbuffer)
        void update_addon_meshes();

        // Build loft triangles for a single strand (for SDF construction)
        std::vector<sinriv::kigstudio::voxel::triangle_bvh<float>::triangle>
        build_strand_loft_triangles(int strand_idx) const;

        // Build SDF from all hair strands (union of per-strand SDF_Mesh)
        std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> build_hair_sdf() const;

        // Compute world-space bounding box of all hair strand loft meshes.
        // Returns {min, max}. If no valid strands, both are {0,0,0}.
        std::pair<sinriv::kigstudio::voxel::vec3f,
                  sinriv::kigstudio::voxel::vec3f>
        compute_hair_bounds() const;

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

        // 返回每个子节点的 (体素网格, SDF, 几何三角形)。
        // 第三个元素非空表示该子节点走纯几何路径（直接渲染三角形网格），
        // 此时 SDF 为 nullptr；为空则走原 SDF 路径。
        std::vector<std::tuple<sinriv::kigstudio::voxel::VoxelGrid,
                               sinriv::kigstudio::sdf::SDFBasePtr,
                               std::vector<sinriv::kigstudio::voxel::
                                               triangle_bvh<float>::triangle>>>
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
        bool showAddonMesh = true;
        bool showOriginMeshAddon = true;  // 附加件编辑器独立勾选框

        bool auto_segment_update = true;
        bool collision_edit_active = false;  // guard for begin_edit/end_edit

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
        sinriv::kigstudio::sdf::SDFPrecision voxel_precision = sinriv::kigstudio::sdf::SDFPrecision::Fast;
        sinriv::kigstudio::sdf::SDFPrecision sdf_precision_cache =
            sinriv::kigstudio::sdf::SDFPrecision::Precise;
        bool mesh_only = false;
        int source_type = 0;
        int source_node_id = -1;
        int node_source_data_type = 0;
        int node_source_sdf_subdivisions = 2;
        bool node_source_sdf_simplify = false;
        float node_source_sdf_simplify_ratio = 0.1f;
        vec3f silhouette_center = {0.0f, 0.0f, 0.0f};
        bool showSilhouetteCenter = false;
        SilhouetteShapeMode silhouette_shape_mode = SilhouetteShapeMode::DELAUNAY_SPHERE;
        int silhouette_subdivision = 4;
        int silhouette_edge_subdiv = 0;
        float inner_wall_radius = 0.0f;
        float simplify_ratio = -1.0f;

        // undo/redo stacks for collision editor
        std::vector<CollisionEditorSnapshot> undo_stack;
        std::vector<CollisionEditorSnapshot> redo_stack;

        // undo/redo stacks for marked voxels
        std::vector<MarkedVoxelsSnapshot> marked_undo_stack;
        std::vector<MarkedVoxelsSnapshot> marked_redo_stack;

        bool dirty = false;

        /// Semantic angle config: maps (X,Y) → (theta, phi) for ray casting.
        /// Must be set via setAngleConfig before semantic point addition.
        std::map<std::pair<float, float>, HairAngleEntry> hair_angle_config;
        /// North pole direction for the spherical coordinate frame.
        /// Default is world +Y. Defines the phi=+90° direction in spherical_to_dir().
        sinriv::kigstudio::voxel::vec3f hair_north_pole = {0.0f, 1.0f, 0.0f};
        /// Front reference direction. Together with north_pole, defines the
        /// sagittal (nose) plane. Projected onto the equatorial plane to derive
        /// the theta=0° (front) direction. Default is world +Z.
        sinriv::kigstudio::voxel::vec3f hair_front_reference = {0.0f, 0.0f, 1.0f};
        /// Node id whose mesh was used to build hair_bvh (-1 if none).
        int hair_bvh_base_node_id = -1;
        /// BVH tree for ray-casting against the base model.
        /// Built when angle config is set and rebuilt when the base model changes.
        std::unique_ptr<sinriv::kigstudio::voxel::triangle_bvh<float>> hair_bvh;
        // Angle config editor state (ephemeral, not serialized)
        static constexpr int kAngleConfigSentinel = -999;
        int angle_config_editing_x = kAngleConfigSentinel;
        int angle_config_editing_y = kAngleConfigSentinel;
        float angle_config_preview_theta = 0;  // live theta for cyan highlight
        float angle_config_preview_phi = 0;    // live phi for cyan highlight

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
    ~RenderVoxelList();

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

    // Ortho view off-screen render shader
    std::unique_ptr<RenderMeshShader> ortho_shader_;

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
    bool showAddonMesh = true;

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
    void render_object_editor_repair_mode(RenderVoxelItem& item);
    void render_object_editor_subdivide_mode(RenderVoxelItem& item);
    void render_object_editor_silhouette_mode(RenderVoxelItem& item);
    void render_object_editor_voxel_tab_content(RenderVoxelItem& item);
    void render_object_editor_comment_tab_content(RenderVoxelItem& item);
    void render_object_editor_addons();
    void render_guide_curve_window();
    void render_width_editor_window();
    void render_cross_section_editor();
    void render_perpoint_section_editor();
    void render_ortho_setup_window();
    void render_ortho_edit_window();
    void perform_ortho_render(RenderVoxelItem& item, RenderVoxelItem& base_item);
    void destroy_ortho_resources();
    void process_ortho_render();
    void process_ai_export();
    bool show_addon_window = false;
    bool show_guide_curve_window = false;
    bool show_width_editor_window = false;
    bool show_cross_section_editor_window = false;
    bool show_perpoint_section_editor_window = false;
    bool show_hairline_plane_window = false;
    bool show_angle_config_window = false;
    bool show_hair_root_window = false;
    bool show_ortho_setup_window = false;
    bool show_ortho_edit_window = false;

    // Orthographic projection edit mode state (all settings, textures, interaction)
    OrthoProjectionState ortho_state;

    // Pointer to the main Agent API server (owned by ui.hpp).
    // Ortho render/overlay/blend data is pushed here each frame.
    sinriv::kigstudio::agent::AgentServer* agent_server_ptr = nullptr;
    void update_api_server_caches();
    // CPU-side copy of overlay for API blending (kept alongside GPU texture)
    std::vector<uint8_t> overlay_cpu_rgba_;
    int overlay_cpu_w_ = 0, overlay_cpu_h_ = 0;
    // Per-point section conflict confirmation dialogs
    bool show_perpoint_confirm_global_open = false;
    bool show_perpoint_confirm_global_apply = false;
    int pending_global_section_strand = -1;
    // Strand rename popup state
    std::string pending_rename_strand_uuid;
    char rename_buffer[256] = {};
    void render_plane_editor(RenderVoxelItem& item);
    void render_collision_body_editor(RenderVoxelItem& item);
    void render_hairline_plane_window();
    void render_angle_config_window();
    void render_hair_root_window();
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

    // ---- 最近打开的文件/工程 ----
    struct RecentEntry {
        std::string path;
        int64_t timestamp = 0;  // unix 时间戳
    };
    std::vector<RecentEntry> recent_files;
    std::vector<RecentEntry> recent_projects;
    bool recent_state_loaded = false;
    static constexpr size_t kMaxRecentEntries = 10;

    std::filesystem::path get_state_dir() const;
    std::filesystem::path get_state_file_path() const;
    void load_recent_state();
    void save_recent_state() const;
    void add_recent_file(const std::string& path);
    void add_recent_project(const std::string& path);
    void render_recent_files_menu();

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

    // MMD / PMX extraction dialog state
    bool show_extract_mmd_dialog = false;
    bool pending_open_extract_mmd_dialog = false;
    std::string extract_mmd_pmx_path;
    int extract_mmd_mode = 0;  // 0 = bones, 1 = materials
    std::vector<std::pair<std::string, bool>> extract_mmd_items;
    float extract_mmd_threshold = 0.5f;
    bool extract_mmd_case_sensitive = false;
    bool extract_mmd_listing = false;
    bool extract_mmd_extracting = false;
    sinriv::kigstudio::Process extract_mmd_process;
    std::string extract_mmd_json_in;
    std::string extract_mmd_json_out;
    std::string extract_mmd_status_msg;
    std::string extract_mmd_result_stl;
    void render_extract_mmd_dialog();
    void extract_mmd_start_list();
    void extract_mmd_poll();
    void extract_mmd_finish_list(const std::string& json_text);
    void extract_mmd_start_extract();
    void extract_mmd_finish_extract(const std::string& json_text);

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

    // ---- Toast / 屏幕底部消息框 ----
    struct ToastMessage {
        std::string text;
        std::chrono::steady_clock::time_point start_time;
        float duration_ms = 1000.0f;
    };
    std::deque<ToastMessage> toast_queue;
    std::mutex toast_mutex;
    static constexpr size_t kMaxToastQueue = 10;
    static constexpr float kToastFadeInRatio = 0.1f;       // 前 10% 时间淡入
    static constexpr float kToastFadeOutStartRatio = 0.7f; // 后 30% 时间淡出

    // 显示一条 toast 消息（线程安全，可从后台线程调用）
    void show_toast(const std::string& msg, float duration_ms = 1000.0f);

    // 格式化并显示 toast（locale key + 可变参数）
    inline void show_toastf(float duration_ms, const char* key, ...) {
        std::string fmt = get_locale_string(key);
        char buf[1024];
        va_list args;
        va_start(args, key);
        vsnprintf(buf, sizeof(buf), fmt.c_str(), args);
        va_end(args);
        show_toast(buf, duration_ms);
    }

    // 渲染 toast（在 UI 线程中调用）
    void render_toast();

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
    void setAddonMeshVisible(bool visible);
    void setCollisionVisible(bool visible);
    void setCollisionBoundsVisible(bool visible);
    void setVoxelChunkBoundsVisible(bool visible);

    // 交互
    RenderVoxelItem* create_item();

    std::vector<RenderVoxelItem*> do_segment(int index);
    std::vector<sinriv::kigstudio::voxel::triangle_bvh<float>::triangle>
    do_repair_mesh(const RenderVoxelItem& item);
    std::vector<sinriv::kigstudio::voxel::triangle_bvh<float>::triangle>
    do_subdivide_mesh(const RenderVoxelItem& item);
    std::vector<sinriv::kigstudio::voxel::triangle_bvh<float>::triangle>
    do_silhouette_mesh(const RenderVoxelItem& item);
    void extract_skeleton(int index);

    void load_stl(std::string filename,
                  float voxel_size = 0.5f,
                  double isolevel = 0.5,
                  bool smooth_normals = true,
                  int target_item_id = -1,
                  int load_mode = 0,
                  bool load_as_sdf = false,
                  sinriv::kigstudio::sdf::SDFPrecision voxel_precision = sinriv::kigstudio::sdf::SDFPrecision::Fast);
    void load_from_node(int target_item_id,
                        int source_node_id,
                        int node_source_data_type,
                        int node_source_sdf_subdivisions,
                        bool node_source_sdf_simplify,
                        float node_source_sdf_simplify_ratio,
                        int load_mode = 0,
                        bool load_as_sdf = false,
                        sinriv::kigstudio::sdf::SDFPrecision voxel_precision = sinriv::kigstudio::sdf::SDFPrecision::Fast);

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
        sinriv::kigstudio::sdf::SDFPrecision voxel_precision = sinriv::kigstudio::sdf::SDFPrecision::Fast;
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
                        sinriv::kigstudio::sdf::SDFPrecision voxel_precision = sinriv::kigstudio::sdf::SDFPrecision::Fast);
    void queue_reload_stl(int item_id,
                          float voxel_size,
                          const std::string& stl_path,
                          int load_mode = 0,
                          bool load_as_sdf = false,
                          sinriv::kigstudio::sdf::SDFPrecision voxel_precision = sinriv::kigstudio::sdf::SDFPrecision::Fast,
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
