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

// 输入三角形数组，利用 cone-box 算法生成封闭 mesh
// auto_center=true 时自动从包围盒计算中心，否则使用 manual_center
std::vector<Triangle> build_closed_mesh_from_triangles(
    const std::vector<Triangle>& triangles,
    bool auto_center = true,
    const vec3f& manual_center = vec3f{0.0f, 0.0f, 0.0f});

// 3D 轮廓边算法：从外部看向 center，保留最远可见面 + 侧面
// should_continue: 返回 false 时提前终止；
// progress(t, step) 报告 0~1 的进度以及当前步骤的本地化描述。
std::vector<Triangle> build_closed_mesh_from_triangles_silhouette(
    const std::vector<Triangle>& triangles,
    const vec3f& center,
    const std::function<bool()>& should_continue = nullptr,
    const std::function<void(float, const std::string&)>& progress = nullptr);

}  // namespace sinriv::kigstudio::mesh::conebox