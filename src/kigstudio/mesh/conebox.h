#pragma once
#include <CGAL/AABB_segment_primitive_2.h>
#include <CGAL/AABB_traits_2.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_triangulation_decomposition_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <cJSON.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "kigstudio/utils/vec2.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/voxel/voxel2mesh.h"

namespace sinriv::kigstudio::mesh::conebox {

using vec3f = sinriv::kigstudio::vec3<float>;
using vec2f = sinriv::kigstudio::vec2<float>;
using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point2 = CGAL::Point_2<Kernel>;
using Segment2 = CGAL::Segment_2<Kernel>;
using Polygon_2 = CGAL::Polygon_2<Kernel>;
using Polygon_with_holes_2 = CGAL::Polygon_with_holes_2<Kernel>;
using SegmentPrimitive =
    CGAL::AABB_segment_primitive_2<Kernel, std::vector<Segment2>::iterator>;
using AABBTree = CGAL::AABB_tree<CGAL::AABB_traits_2<Kernel, SegmentPrimitive>>;
using Clipper = CGAL::Polygon_triangulation_decomposition_2<Kernel>;

struct Triangle_status {
    Triangle triangle;
    Triangle pos_in_face[6];  // 每个顶点投影到立方体上的位置，z轴是深度
    void compute_projection(
        const vec3f&
            center);  // 输入立方体中心，计算每个顶点在立方体上的投影位置
};
// 透视逆变换
vec3f perspective_inverse(const vec3f& pos, const vec3f& c, int face_index);

bool ray_triangle_intersect(const vec3f& origin,
                            const vec3f& direction,
                            const vec3f& v0,
                            const vec3f& v1,
                            const vec3f& v2,
                            vec3f& out_point);

struct Triangle_group {
    std::vector<Triangle_status> triangles;
    std::array<std::vector<size_t>, 6> face_triangle_indices;
    std::array<std::vector<Segment2>, 6> face_segments;
    std::array<std::vector<size_t>, 6> face_segment_triangle_ids;
    std::array<AABBTree, 6> face_trees;
    vec3f center;

    void add_triangle(const Triangle_status& status);
    void build_face_trees();
    std::tuple<std::vector<std::array<Point2, 3>>,
               std::vector<std::vector<Point2>>>
    compute_visible_triangulation(size_t triangle_id, int face_id) const;
    std::vector<Triangle> compute_visible_mesh_from_outside() const;
    std::vector<Triangle> compute_visible_mesh_with_cone_sides() const;
};

// Standalone face type (3 vertex indices) shared by shape generators and builder.
struct SubFace { int a, b, c; };

// Abstract interface for generating the enclosing shape's vertices and faces.
// Different implementations provide different sampling strategies
// (icosahedron, cube-sphere, UV-sphere, custom mesh, etc.).
struct ISilhouetteShapeGenerator {
	virtual ~ISilhouetteShapeGenerator() = default;
	// Populate out_verts (world-space positions on the enclosing surface),
	// out_faces (triangles as index triples into out_verts), and
	// out_min_distances (per-vertex minimum distance from center, same size as out_verts).
	//   value <= 0: vertex behaves normally (discard if ray misses)
	//   value > 0:  vertex always "hit", position clamped to >= this distance
	virtual void generate(float radius, const vec3f& center,
	                      std::vector<vec3f>& out_verts,
	                      std::vector<SubFace>& out_faces,
	                      std::vector<float>& out_min_distances) = 0;
};

// Default shape generator: subdivided icosahedron.
class IcosahedronGenerator : public ISilhouetteShapeGenerator {
public:
	explicit IcosahedronGenerator(int subdivision_level = 4)
	    : subdivision_level_(std::max(1, subdivision_level)) {}
	void generate(float radius, const vec3f& center,
	              std::vector<vec3f>& out_verts,
	              std::vector<SubFace>& out_faces,
	              std::vector<float>& out_min_distances) override;

private:
	int subdivision_level_;
};

// Shape generator from user-provided vertices via CGAL Delaunay triangulation
// on sphere. Each sample point is projected onto the enclosing sphere;
// faces are computed by CGAL's spherical Delaunay triangulation.
// out_min_distances = original distance of each sample point from center
// (so the output surface is clamped to at least this distance).
class DelaunaySphereGenerator : public ISilhouetteShapeGenerator {
public:
	// sample_points: world-space vertices whose directions from the
	// (future) center define the ray directions. Duplicates that project
	// to the same sphere position are merged (farthest distance kept).
	explicit DelaunaySphereGenerator(
	    std::vector<vec3f> sample_points)
	    : sample_points_(std::move(sample_points)) {}

	void generate(float radius, const vec3f& center,
	              std::vector<vec3f>& out_verts,
	              std::vector<SubFace>& out_faces,
	              std::vector<float>& out_min_distances) override;

private:
	std::vector<vec3f> sample_points_;
};

// Internal builder for the icosphere-silhouette algorithm.
// The free function build_closed_mesh_from_triangles_silhouette delegates to this.
class IcosphereSilhouetteBuilder {
public:
	IcosphereSilhouetteBuilder(
	    const std::vector<Triangle>& input_triangles,
	    const vec3f& center,
	    std::unique_ptr<ISilhouetteShapeGenerator> shape_generator,
	    const std::function<bool()>& should_continue,
	    const std::function<void(float, const std::string&)>& progress,
	    float inner_wall_radius,
	    float simplify_ratio);

	// Run the full pipeline. This is the only public method besides the ctor.
	std::vector<Triangle> build();

private:
	// --- Parameters (set in ctor, unchanged after) ---
	const std::vector<Triangle>& input_triangles_;
	vec3f center_;
	std::function<bool()> should_continue_;
	std::function<void(float, const std::string&)> progress_;
	std::unique_ptr<ISilhouetteShapeGenerator> shape_generator_;
	float inner_wall_radius_;
	float simplify_ratio_;

	// --- Derived constants ---
	float radius_ = 0.0f;
	float ray_len_ = 0.0f;

	// --- Cancellation throttling ---
	int cancel_step_ = 0;

	// --- BVH (bvh_indices_ before bvh_: bvh_ stores int* into it) ---
	using BVH = sinriv::kigstudio::dbvt3d<float, int>;
	std::vector<int> bvh_indices_;
	std::vector<BVH::AABB*> bvh_aabbs_;
	BVH bvh_;

	// --- Icosphere / shape geometry ---
	std::vector<vec3f> sub_verts_;
	std::vector<SubFace> sub_faces_;
	std::vector<float> vertex_min_distances_;  // per-vertex min distance from center (<=0 = no minimum, >0 = clamped)

	// --- Vertex neighbor graph ---
	std::vector<std::vector<int>> vert_neighbors_;

	// --- Ray-cast results ---
	std::vector<bool> hit_;
	std::vector<vec3f> hit_pos_;

	// --- Inner wall ---
	bool has_inner_wall_ = false;
	std::vector<vec3f> inner_pos_;

	// --- Output mesh state ---
	struct SurfFace { int va, vb, vc; };
	std::vector<SurfFace> surf_faces_;

	struct EdgeKey2 {
		int a, b;
		EdgeKey2(int va, int vb) : a(std::min(va, vb)), b(std::max(va, vb)) {}
		bool operator<(const EdgeKey2& o) const {
			return a != o.a ? a < o.a : b < o.b;
		}
	};
	std::vector<EdgeKey2> boundary_edges_;
	std::vector<Triangle> result_;

	struct Vec3fCmp {
		bool operator()(const vec3f& a, const vec3f& b) const {
			if (a.x != b.x) return a.x < b.x;
			if (a.y != b.y) return a.y < b.y;
			return a.z < b.z;
		}
	};

	// --- Helper methods ---
	void report_progress(float t, const std::string& text);
	bool check_cancel();

	// --- Phase methods ---
	void build_bvh();
	void compute_radius();
	void generate_shape();
	void build_neighbor_graph();
	void cast_rays();
	void verify_hits();
	void compute_inner_wall();
	void classify_faces_and_collect_boundaries();
	void generate_inner_wall_faces();
	void build_side_triangles();
	void stitch_and_fill();
	void simplify_mesh();
	void cleanup_bvh();
};

// 输入三角形数组，利用 cone-box 算法生成封闭 mesh
// auto_center=true 时自动从包围盒计算中心，否则使用 manual_center
std::vector<Triangle> build_closed_mesh_from_triangles(
    const std::vector<Triangle>& triangles,
    bool auto_center = true,
    const vec3f& manual_center = vec3f{0.0f, 0.0f, 0.0f});

// 3D 轮廓边算法：正二十面体射线投射版本。
// 从 center 向外通过细分的二十面体顶点发射射线，找最远碰撞点。
// 所有顶点命中的面保留为表面，边界边连接 center 形成侧面。
// should_continue: 返回 false 时提前终止；
// progress(t, step) 报告 0~1 的进度以及当前步骤的本地化描述。
// subdivision_level: 正二十面体细分等级，每条边分割为 N 段，总面数 = 20 * N^2。
//   默认 4（320 面），范围 1~10。
// simplify_ratio: CGAL 边折叠简化比率（0~1），负数禁用。默认 -1.f。
std::vector<Triangle> build_closed_mesh_from_triangles_silhouette(
    const std::vector<Triangle>& triangles,
    const vec3f& center,
    const std::function<bool()>& should_continue = nullptr,
    const std::function<void(float, const std::string&)>& progress = nullptr,
    int subdivision_level = 4,
    float inner_wall_radius = 0.f,
    float simplify_ratio = -1.f);

// Delaunay 球面版本：将输入三角形的顶点投影到包围球上，
// 用 CGAL 3D Delaunay 凸包做球面三角剖分，每个顶点以其原始
// 到 center 的距离作为最小距离约束（始终命中，位置限制不小于此距离）。
std::vector<Triangle> build_closed_mesh_from_triangles_silhouette_delaunay(
    const std::vector<Triangle>& input_triangles,
    const vec3f& center,
    const std::function<bool()>& should_continue = nullptr,
    const std::function<void(float, const std::string&)>& progress = nullptr,
    float inner_wall_radius = 0.f,
    float simplify_ratio = -1.f);

// 旧版本：锥体裁剪 + 边界边提取实现，保留用于参考。
std::vector<Triangle> build_closed_mesh_from_triangles_silhouette_old(
    const std::vector<Triangle>& triangles,
    const vec3f& center,
    const std::function<bool()>& should_continue = nullptr,
    const std::function<void(float, const std::string&)>& progress = nullptr);

}  // namespace sinriv::kigstudio::mesh::conebox