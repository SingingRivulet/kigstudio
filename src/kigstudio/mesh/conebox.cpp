#include "conebox.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <tuple>

#include "kigstudio/utils/dbvt3d.h"
#include "kigstudio/utils/locale.h"

namespace sinriv::kigstudio::mesh::conebox {

// #define CONEBOX_DEBUG 0

// #if CONEBOX_DEBUG
#define CB_DBG(x) std::cout << "[conebox] " << x << std::endl
// #else
// #define CB_DBG(x)
// #endif

void Triangle_status::compute_projection(const vec3f& center) {
    const auto& [v0, v1, v2] = triangle;

    const float half_size = 0.5f;
    const float min_x = center.x - half_size;
    const float max_x = center.x + half_size;
    const float min_y = center.y - half_size;
    const float max_y = center.y + half_size;
    const float min_z = center.z - half_size;
    const float max_z = center.z + half_size;

    auto intersect_face = [&](const vec3f& vertex, int face) {
        const vec3f dir = center - vertex;
        const float len = dir.length();
        if (len <= 1e-8f)
            return vec3f{0.0f, 0.0f, -1.0f};

        float t = 0.0f;
        vec3f point;
        switch (face) {
            case 0:  // +X face
                if (std::abs(dir.x) < 1e-8f)
                    return vec3f{0.0f, 0.0f, -1.0f};
                t = (max_x - vertex.x) / dir.x;
                point = vertex + dir * t;
                if (t <= 0.0f || point.y < min_y || point.y > max_y ||
                    point.z < min_z || point.z > max_z)
                    return vec3f{0.0f, 0.0f, -1.0f};
                return vec3f{point.y, point.z, t * len};
            case 1:  // -X face
                if (std::abs(dir.x) < 1e-8f)
                    return vec3f{0.0f, 0.0f, -1.0f};
                t = (min_x - vertex.x) / dir.x;
                point = vertex + dir * t;
                if (t <= 0.0f || point.y < min_y || point.y > max_y ||
                    point.z < min_z || point.z > max_z)
                    return vec3f{0.0f, 0.0f, -1.0f};
                return vec3f{point.y, point.z, t * len};
            case 2:  // +Y face
                if (std::abs(dir.y) < 1e-8f)
                    return vec3f{0.0f, 0.0f, -1.0f};
                t = (max_y - vertex.y) / dir.y;
                point = vertex + dir * t;
                if (t <= 0.0f || point.x < min_x || point.x > max_x ||
                    point.z < min_z || point.z > max_z)
                    return vec3f{0.0f, 0.0f, -1.0f};
                return vec3f{point.x, point.z, t * len};
            case 3:  // -Y face
                if (std::abs(dir.y) < 1e-8f)
                    return vec3f{0.0f, 0.0f, -1.0f};
                t = (min_y - vertex.y) / dir.y;
                point = vertex + dir * t;
                if (t <= 0.0f || point.x < min_x || point.x > max_x ||
                    point.z < min_z || point.z > max_z)
                    return vec3f{0.0f, 0.0f, -1.0f};
                return vec3f{point.x, point.z, t * len};
            case 4:  // +Z face
                if (std::abs(dir.z) < 1e-8f)
                    return vec3f{0.0f, 0.0f, -1.0f};
                t = (max_z - vertex.z) / dir.z;
                point = vertex + dir * t;
                if (t <= 0.0f || point.x < min_x || point.x > max_x ||
                    point.y < min_y || point.y > max_y)
                    return vec3f{0.0f, 0.0f, -1.0f};
                return vec3f{point.x, point.y, t * len};
            case 5:  // -Z face
                if (std::abs(dir.z) < 1e-8f)
                    return vec3f{0.0f, 0.0f, -1.0f};
                t = (min_z - vertex.z) / dir.z;
                point = vertex + dir * t;
                if (t <= 0.0f || point.x < min_x || point.x > max_x ||
                    point.y < min_y || point.y > max_y)
                    return vec3f{0.0f, 0.0f, -1.0f};
                return vec3f{point.x, point.y, t * len};
            default:
                return vec3f{0.0f, 0.0f, -1.0f};
        }
    };

    pos_in_face[0] = std::make_tuple(
        intersect_face(v0, 0), intersect_face(v1, 0), intersect_face(v2, 0));
    pos_in_face[1] = std::make_tuple(
        intersect_face(v0, 1), intersect_face(v1, 1), intersect_face(v2, 1));
    pos_in_face[2] = std::make_tuple(
        intersect_face(v0, 2), intersect_face(v1, 2), intersect_face(v2, 2));
    pos_in_face[3] = std::make_tuple(
        intersect_face(v0, 3), intersect_face(v1, 3), intersect_face(v2, 3));
    pos_in_face[4] = std::make_tuple(
        intersect_face(v0, 4), intersect_face(v1, 4), intersect_face(v2, 4));
    pos_in_face[5] = std::make_tuple(
        intersect_face(v0, 5), intersect_face(v1, 5), intersect_face(v2, 5));
}

vec3f perspective_inverse(const vec3f& pos, const vec3f& c, int face_index) {
    const float half_size = 0.5f;
    const float u = pos.x;
    const float v = pos.y;
    const float depth = pos.z;

    vec3f surface_point;
    switch (face_index) {
        case 0:  // +X face
            surface_point = vec3f(c.x + half_size, u, v);
            break;
        case 1:  // -X face
            surface_point = vec3f(c.x - half_size, u, v);
            break;
        case 2:  // +Y face
            surface_point = vec3f(u, c.y + half_size, v);
            break;
        case 3:  // -Y face
            surface_point = vec3f(u, c.y - half_size, v);
            break;
        case 4:  // +Z face
            surface_point = vec3f(u, v, c.z + half_size);
            break;
        case 5:  // -Z face
            surface_point = vec3f(u, v, c.z - half_size);
            break;
        default:
            return surface_point;
    }

    vec3f delta = surface_point - c;
    float s = delta.length();

    if (s < 1e-8f)
        return surface_point;

    return c + delta * ((s + depth) / s);
}

void Triangle_group::add_triangle(const Triangle_status& status) {
    triangles.push_back(status);
    const size_t index = triangles.size() - 1;

    for (int face = 0; face < 6; ++face) {
        const auto& [p0, p1, p2] = triangles[index].pos_in_face[face];
        const bool valid0 = p0.z > 0.0f;
        const bool valid1 = p1.z > 0.0f;
        const bool valid2 = p2.z > 0.0f;

        if (!(valid0 || valid1 || valid2))
            continue;

        face_triangle_indices[face].push_back(index);

        std::vector<Point2> projected_points;
        projected_points.reserve(3);
        if (valid0)
            projected_points.emplace_back(p0.x, p0.y);
        if (valid1)
            projected_points.emplace_back(p1.x, p1.y);
        if (valid2)
            projected_points.emplace_back(p2.x, p2.y);

        switch (projected_points.size()) {
            case 0:
                break;
            case 1:
                // 不生成零长度线段，避免 CGAL AABB 树退化
                break;
            case 2:
                face_segments[face].emplace_back(projected_points[0],
                                                 projected_points[1]);
                face_segment_triangle_ids[face].push_back(index);
                break;
            case 3:
                face_segments[face].emplace_back(projected_points[0],
                                                 projected_points[1]);
                face_segment_triangle_ids[face].push_back(index);
                face_segments[face].emplace_back(projected_points[1],
                                                 projected_points[2]);
                face_segment_triangle_ids[face].push_back(index);
                face_segments[face].emplace_back(projected_points[2],
                                                 projected_points[0]);
                face_segment_triangle_ids[face].push_back(index);
                break;
        }
    }
}

static float average_projected_depth(const Triangle_status& status, int face) {
    const auto& [p0, p1, p2] = status.pos_in_face[face];
    float sum = 0.0f;
    int count = 0;
    const vec3f points[3] = {p0, p1, p2};
    for (const auto& p : points) {
        if (p.z > 0.0f) {
            sum += p.z;
            ++count;
        }
    }
    return count > 0 ? sum / count : 0.0f;
}

static bool point2_near(const Point2& a, const Point2& b) {
    const double dx = CGAL::to_double(a.x() - b.x());
    const double dy = CGAL::to_double(a.y() - b.y());
    return dx * dx + dy * dy < 1e-12;  // ~1e-6 距离
}

static Point2 snap_point2(const Point2& p) {
    constexpr double SCALE = 1e5;  // 1e-5 网格
    const double x = CGAL::to_double(p.x());
    const double y = CGAL::to_double(p.y());
    Point2 sp(std::round(x * SCALE) / SCALE, std::round(y * SCALE) / SCALE);
    return sp;
}

static Polygon_2 cleanup_polygon(const Polygon_2& poly) {
    if (poly.size() < 3)
        return Polygon_2();

    std::vector<Point2> verts;
    verts.reserve(poly.size());
    for (auto it = poly.vertices_begin(); it != poly.vertices_end(); ++it) {
        Point2 sp = snap_point2(*it);
        if (verts.empty() || !point2_near(sp, verts.back()))
            verts.push_back(sp);
    }
    while (verts.size() > 1 && point2_near(verts.front(), verts.back()))
        verts.pop_back();
    if (verts.size() < 3)
        return Polygon_2();

    // 移除共线中间点
    std::vector<Point2> cleaned;
    cleaned.reserve(verts.size());
    for (size_t i = 0; i < verts.size(); ++i) {
        const auto& prev = verts[(i + verts.size() - 1) % verts.size()];
        const auto& curr = verts[i];
        const auto& next = verts[(i + 1) % verts.size()];
        const double cross =
            CGAL::to_double((curr.x() - prev.x()) * (next.y() - prev.y()) -
                            (curr.y() - prev.y()) * (next.x() - prev.x()));
        if (std::abs(cross) > 1e-12)
            cleaned.push_back(curr);
    }
    if (cleaned.size() < 3)
        return Polygon_2();

    CB_DBG("cleanup_polygon: " << poly.size() << " -> " << cleaned.size());
    return Polygon_2(cleaned.begin(), cleaned.end());
}

static Polygon_with_holes_2 cleanup_polygon_with_holes(
    const Polygon_with_holes_2& pwh) {
    Polygon_2 outer = cleanup_polygon(pwh.outer_boundary());
    if (outer.size() < 3)
        return Polygon_with_holes_2();

    Polygon_with_holes_2 result(outer);
    int hole_count = 0;
    for (auto hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
        Polygon_2 h = cleanup_polygon(*hit);
        if (h.size() >= 3) {
            result.add_hole(h);
            ++hole_count;
        }
    }
    CB_DBG("cleanup_pwh: outer=" << outer.size() << " holes=" << hole_count);
    return result;
}

static Polygon_2 make_face_polygon(const Triangle_status& status, int face) {
    const auto& [p0, p1, p2] = status.pos_in_face[face];
    CB_DBG("make_face_polygon face=" << face << " raw=" << p0.x << "," << p0.y
                                     << "," << p0.z << " " << p1.x << ","
                                     << p1.y << "," << p1.z << " " << p2.x
                                     << "," << p2.y << "," << p2.z);
    std::vector<Point2> raw;
    raw.reserve(3);
    if (p0.z > 0.0f)
        raw.emplace_back(p0.x, p0.y);
    if (p1.z > 0.0f)
        raw.emplace_back(p1.x, p1.y);
    if (p2.z > 0.0f)
        raw.emplace_back(p2.x, p2.y);

    if (raw.size() < 3) {
        CB_DBG("  -> rejected: only " << raw.size() << " valid verts");
        return Polygon_2();
    }

    // Snap + 去重：移除过近的投影点，避免 CGAL 退化
    std::vector<Point2> vertices;
    vertices.reserve(raw.size());
    for (const auto& p : raw) {
        Point2 sp = snap_point2(p);
        bool dup = false;
        for (const auto& q : vertices) {
            if (point2_near(sp, q)) {
                dup = true;
                break;
            }
        }
        if (!dup)
            vertices.push_back(sp);
    }
    if (vertices.size() < 3) {
        CB_DBG("  -> rejected: dedup left " << vertices.size());
        return Polygon_2();
    }

    // 检查是否有实际面积（不共线）
    bool has_area = false;
    for (size_t i = 0; i < vertices.size(); ++i) {
        const auto& a = vertices[i];
        const auto& b = vertices[(i + 1) % vertices.size()];
        const auto& c = vertices[(i + 2) % vertices.size()];
        const double cross = CGAL::to_double((b.x() - a.x()) * (c.y() - a.y()) -
                                             (b.y() - a.y()) * (c.x() - a.x()));
        if (std::abs(cross) > 1e-12) {
            has_area = true;
            break;
        }
    }
    if (!has_area) {
        CB_DBG("  -> rejected: no area");
        return Polygon_2();
    }

    Polygon_2 polygon(vertices.begin(), vertices.end());
    if (!polygon.is_counterclockwise_oriented())
        polygon.reverse_orientation();

    CB_DBG("  -> ok size=" << polygon.size());
    return polygon;
}

static bool polygon_overlaps_triangle(const Polygon_2& a, const Polygon_2& b) {
    if (a.is_empty() || b.is_empty())
        return false;

    // AABB 快速排除
    auto aabb = [](const Polygon_2& p, double& min_x, double& max_x,
                   double& min_y, double& max_y) {
        min_x = max_x = CGAL::to_double(p[0].x());
        min_y = max_y = CGAL::to_double(p[0].y());
        for (size_t i = 1; i < p.size(); ++i) {
            double x = CGAL::to_double(p[i].x());
            double y = CGAL::to_double(p[i].y());
            if (x < min_x)
                min_x = x;
            if (x > max_x)
                max_x = x;
            if (y < min_y)
                min_y = y;
            if (y > max_y)
                max_y = y;
        }
    };

    double a_min_x, a_max_x, a_min_y, a_max_y;
    double b_min_x, b_max_x, b_min_y, b_max_y;
    aabb(a, a_min_x, a_max_x, a_min_y, a_max_y);
    aabb(b, b_min_x, b_max_x, b_min_y, b_max_y);
    if (a_max_x < b_min_x || a_min_x > b_max_x || a_max_y < b_min_y ||
        a_min_y > b_max_y)
        return false;

    CB_DBG("polygon_overlaps: aabb ok, checking edges...");

    // 边相交检测（Segment2 do_intersect 是纯 predicate，EPICK 安全）
    for (auto e1 = a.edges_begin(); e1 != a.edges_end(); ++e1) {
        for (auto e2 = b.edges_begin(); e2 != b.edges_end(); ++e2) {
            if (CGAL::do_intersect(*e1, *e2)) {
                CB_DBG("  -> edge intersect true");
                return true;
            }
        }
    }

    // 顶点严格在另一多边形内部
    for (auto it = a.vertices_begin(); it != a.vertices_end(); ++it) {
        if (b.bounded_side(*it) == CGAL::ON_BOUNDED_SIDE) {
            CB_DBG("  -> vertex inside true");
            return true;
        }
    }
    for (auto it = b.vertices_begin(); it != b.vertices_end(); ++it) {
        if (a.bounded_side(*it) == CGAL::ON_BOUNDED_SIDE) {
            CB_DBG("  -> vertex inside true");
            return true;
        }
    }
    CB_DBG("  -> false");
    return false;
}

std::tuple<std::vector<std::array<Point2, 3>>, std::vector<std::vector<Point2>>>
Triangle_group::compute_visible_triangulation(size_t triangle_id,
                                              int face_id) const {
    std::vector<std::array<Point2, 3>> out_triangles;
    std::vector<std::vector<Point2>> polygon_vertices;

    if (triangle_id >= triangles.size() || face_id < 0 || face_id >= 6)
        return {out_triangles, polygon_vertices};

    CB_DBG("=== compute_visible_triangulation tri=" << triangle_id
                                                    << " face=" << face_id);

    const Triangle_status& target_status = triangles[triangle_id];
    Polygon_2 target_polygon = make_face_polygon(target_status, face_id);
    if (target_polygon.size() < 3) {
        CB_DBG("  target_polygon empty, skip");
        return {out_triangles, polygon_vertices};
    }

    const float target_depth = average_projected_depth(target_status, face_id);
    CB_DBG("  target_depth=" << target_depth);

    std::unordered_set<size_t> candidate_ids;
    candidate_ids.reserve(16);

    if (!face_segments[face_id].empty()) {
        std::vector<typename AABBTree::Primitive_id> primitive_ids;
        primitive_ids.reserve(16);

        for (auto edge_it = target_polygon.edges_begin();
             edge_it != target_polygon.edges_end(); ++edge_it) {
            primitive_ids.clear();
            face_trees[face_id].all_intersected_primitives(
                *edge_it, std::back_inserter(primitive_ids));
            for (const auto& primitive_id : primitive_ids) {
                const size_t segment_index = static_cast<size_t>(
                    primitive_id - face_segments[face_id].begin());
                if (segment_index < face_segment_triangle_ids[face_id].size()) {
                    candidate_ids.insert(
                        face_segment_triangle_ids[face_id][segment_index]);
                }
            }
        }
    }

    CB_DBG("  aabb candidates=" << candidate_ids.size());

    for (size_t other_id : face_triangle_indices[face_id]) {
        if (other_id == triangle_id)
            continue;
        if (candidate_ids.find(other_id) != candidate_ids.end())
            continue;

        const Polygon_2 other_polygon =
            make_face_polygon(triangles[other_id], face_id);
        if (other_polygon.is_empty())
            continue;
        if (polygon_overlaps_triangle(target_polygon, other_polygon))
            candidate_ids.insert(other_id);
    }

    CB_DBG("  total candidates=" << candidate_ids.size());

    struct Occluder {
        size_t id;
        float depth;
        Polygon_2 polygon;
    };
    std::vector<Occluder> occluders;
    occluders.reserve(candidate_ids.size());

    for (size_t other_id : candidate_ids) {
        if (other_id == triangle_id)
            continue;
        const auto& other_status = triangles[other_id];
        Polygon_2 other_polygon = make_face_polygon(other_status, face_id);
        if (other_polygon.size() < 3)
            continue;

        const float other_depth =
            average_projected_depth(other_status, face_id);
        if (other_depth <= target_depth)
            continue;

        occluders.push_back(
            Occluder{other_id, other_depth, std::move(other_polygon)});
    }

    CB_DBG("  occluders=" << occluders.size());

    std::sort(
        occluders.begin(), occluders.end(),
        [](const Occluder& a, const Occluder& b) { return a.depth < b.depth; });

    std::vector<Polygon_with_holes_2> current_regions;
    current_regions.emplace_back(target_polygon);
    std::vector<Polygon_with_holes_2> next_regions;

    auto polygon_covers = [](const Polygon_2& outer,
                             const Polygon_2& inner) -> bool {
        for (auto it = inner.vertices_begin(); it != inner.vertices_end();
             ++it) {
            if (outer.bounded_side(*it) == CGAL::ON_UNBOUNDED_SIDE)
                return false;
        }
        return true;
    };

    auto has_strictly_inside_vertex = [](const Polygon_2& container,
                                         const Polygon_2& contained) -> bool {
        for (auto it = contained.vertices_begin();
             it != contained.vertices_end(); ++it) {
            if (container.bounded_side(*it) == CGAL::ON_BOUNDED_SIDE)
                return true;
        }
        return false;
    };

    int occluder_idx = 0;
    for (const auto& occluder : occluders) {
        CB_DBG("  applying occluder "
               << occluder_idx << " id=" << occluder.id << " depth="
               << occluder.depth << " poly_size=" << occluder.polygon.size());
        next_regions.clear();
        int region_idx = 0;
        for (const auto& region : current_regions) {
            CB_DBG("    diff with region " << region_idx << " outer="
                                           << region.outer_boundary().size());

            const auto& reg_poly = region.outer_boundary();

            // 如果遮挡物完全覆盖当前区域，直接清空该区域
            if (polygon_covers(occluder.polygon, reg_poly)) {
                CB_DBG("    -> occluder fully covers region, skip");
                ++region_idx;
                continue;
            }

            // 如果两者只有边界接触（没有顶点严格在对方内部），
            // difference 结果不变，跳过以避免 CGAL 退化
            bool region_in_occluder =
                has_strictly_inside_vertex(occluder.polygon, reg_poly);
            bool occluder_in_region =
                has_strictly_inside_vertex(reg_poly, occluder.polygon);
            if (!region_in_occluder && !occluder_in_region) {
                CB_DBG("    -> edge contact only, skip diff");
                next_regions.push_back(region);
                ++region_idx;
                continue;
            }

            CGAL::difference(region, occluder.polygon,
                             std::back_inserter(next_regions));
            ++region_idx;
        }
        // 清理 difference 产生的退化顶点，再进入下一轮/Clipper
        current_regions.clear();
        for (auto& r : next_regions) {
            auto cleaned = cleanup_polygon_with_holes(r);
            if (cleaned.outer_boundary().size() >= 3)
                current_regions.push_back(std::move(cleaned));
        }
        CB_DBG("    after cleanup regions=" << current_regions.size());
        if (current_regions.empty())
            break;
        ++occluder_idx;
    }

    CB_DBG("  final regions=" << current_regions.size());

    Clipper clipper;
    std::vector<Polygon_2> triangles_out;
    for (const auto& region : current_regions) {
        auto cleaned = cleanup_polygon_with_holes(region);
        if (cleaned.outer_boundary().size() < 3)
            continue;

        std::vector<Point2> outer_vertices;
        outer_vertices.reserve(cleaned.outer_boundary().size());
        for (auto vit = cleaned.outer_boundary().vertices_begin();
             vit != cleaned.outer_boundary().vertices_end(); ++vit) {
            outer_vertices.push_back(*vit);
        }
        if (!outer_vertices.empty())
            polygon_vertices.push_back(std::move(outer_vertices));

        CB_DBG("  clipper region outer=" << cleaned.outer_boundary().size());
        clipper(cleaned, std::back_inserter(triangles_out));
    }

    for (const auto& triangle : triangles_out) {
        if (triangle.size() != 3)
            continue;
        std::array<Point2, 3> tri_points;
        auto vit = triangle.vertices_begin();
        for (int i = 0; i < 3; ++i, ++vit)
            tri_points[i] = *vit;
        out_triangles.push_back(tri_points);
    }

    return {out_triangles, polygon_vertices};
}

static bool triangle_has_valid_projection_on_face(const Triangle_status& status,
                                                  int face) {
    const auto& [p0, p1, p2] = status.pos_in_face[face];
    return (p0.z > 0.0f) || (p1.z > 0.0f) || (p2.z > 0.0f);
}

static vec3f face_plane_point(const Point2& point,
                              int face,
                              const vec3f& center) {
    const float u = static_cast<float>(point.x());
    const float v = static_cast<float>(point.y());
    const float half_size = 0.5f;
    switch (face) {
        case 0:
            return vec3f(center.x + half_size, u, v);
        case 1:
            return vec3f(center.x - half_size, u, v);
        case 2:
            return vec3f(u, center.y + half_size, v);
        case 3:
            return vec3f(u, center.y - half_size, v);
        case 4:
            return vec3f(u, v, center.z + half_size);
        case 5:
            return vec3f(u, v, center.z - half_size);
    }
    return center;
}

static bool ray_triangle_intersect(const vec3f& origin,
                                   const vec3f& direction,
                                   const vec3f& v0,
                                   const vec3f& v1,
                                   const vec3f& v2,
                                   vec3f& out_point) {
    const float EPSILON = 1e-6f;
    const vec3f edge1 = v1 - v0;
    const vec3f edge2 = v2 - v0;
    const vec3f pvec = direction.cross(edge2);
    const float det = edge1.dot(pvec);
    if (std::abs(det) < EPSILON)
        return false;
    const float inv_det = 1.0f / det;
    const vec3f tvec = origin - v0;
    const float u = tvec.dot(pvec) * inv_det;
    if (u < 0.0f || u > 1.0f)
        return false;
    const vec3f qvec = tvec.cross(edge1);
    const float v = direction.dot(qvec) * inv_det;
    if (v < 0.0f || u + v > 1.0f)
        return false;
    const float t = edge2.dot(qvec) * inv_det;
    if (t < 0.0f)
        return false;
    out_point = origin + direction * t;
    return true;
}

// 判断点 p 是否在封闭 mesh 内部（三条射线投票，避免单条射线恰好过边/顶点）
static bool is_inside_mesh(const vec3f& p,
                           const std::vector<Triangle>& tris,
                           float unique_eps = 1e-4f) {
    const vec3f dirs[3] = {vec3f(1.0f, 0.0f, 0.0f), vec3f(0.0f, 1.0f, 0.0f),
                           vec3f(0.0f, 0.0f, 1.0f)};
    int inside_votes = 0;

    for (int axis = 0; axis < 3; ++axis) {
        const vec3f& dir = dirs[axis];
        std::vector<float> ts;
        ts.reserve(tris.size());
        for (const auto& tri : tris) {
            vec3f hit;
            if (ray_triangle_intersect(p, dir, std::get<0>(tri),
                                       std::get<1>(tri), std::get<2>(tri),
                                       hit)) {
                float t = (hit - p).dot(dir);
                if (t > unique_eps) {
                    ts.push_back(t);
                }
            }
        }
        std::sort(ts.begin(), ts.end());
        int intersect_count = 0;
        for (size_t i = 0; i < ts.size(); ++i) {
            if (i == 0 || std::abs(ts[i] - ts[i - 1]) > unique_eps) {
                ++intersect_count;
            }
        }
        if (intersect_count % 2 == 1) {
            ++inside_votes;
        }
    }
    // 至少 2 轴认定为 inside 才算内部（与 sdf_mesh.cpp 一致）
    return inside_votes >= 2;
}

// 使用 BVH 加速的点在封闭 mesh 内部判断（三条射线投票）
static bool is_inside_mesh_bvh(const vec3f& p,
                               const sinriv::kigstudio::dbvt3d<float, int>& bvh,
                               const std::vector<Triangle>& tris,
                               float max_dist,
                               float unique_eps = 1e-4f) {
    const vec3f dirs[3] = {vec3f(1.0f, 0.0f, 0.0f), vec3f(0.0f, 1.0f, 0.0f),
                           vec3f(0.0f, 0.0f, 1.0f)};
    int inside_votes = 0;

    for (int axis = 0; axis < 3; ++axis) {
        const vec3f& dir = dirs[axis];
        std::vector<float> ts;
        ts.reserve(tris.size() / 4);

        bvh.rayTest(
            sinriv::kigstudio::ray<float>(p, p + dir * max_dist),
            [&](const sinriv::kigstudio::dbvt3d<float, int>::AABB* node) {
                int idx = *node->data;
                if (idx < 0 || idx >= static_cast<int>(tris.size()))
                    return;
                const auto& tri = tris[idx];
                vec3f hit;
                if (ray_triangle_intersect(p, dir, std::get<0>(tri),
                                           std::get<1>(tri), std::get<2>(tri),
                                           hit)) {
                    float t = (hit - p).dot(dir);
                    if (t > unique_eps) {
                        ts.push_back(t);
                    }
                }
            });

        std::sort(ts.begin(), ts.end());
        int intersect_count = 0;
        for (size_t i = 0; i < ts.size(); ++i) {
            if (i == 0 || std::abs(ts[i] - ts[i - 1]) > unique_eps) {
                ++intersect_count;
            }
        }
        if (intersect_count % 2 == 1) {
            ++inside_votes;
        }
    }
    return inside_votes >= 2;
}

struct TriangleHash {
    // 使用空间网格量化，避免浮点误差导致相同顶点哈希不同
    static constexpr float EPS = 1e-5f;

    static int64_t quantize(float v) {
        return static_cast<int64_t>(std::llround(v / EPS));
    }

    static size_t hash_vec3(const vec3f& p) {
        size_t h = std::hash<int64_t>{}(quantize(p.x));
        h ^= std::hash<int64_t>{}(quantize(p.y)) + 0x9e3779b97f4a7c15ULL +
             (h << 6) + (h >> 2);
        h ^= std::hash<int64_t>{}(quantize(p.z)) + 0x9e3779b97f4a7c15ULL +
             (h << 6) + (h >> 2);
        return h;
    }

    size_t operator()(const Triangle& tri) const noexcept {
        const size_t h0 = hash_vec3(std::get<0>(tri));
        const size_t h1 = hash_vec3(std::get<1>(tri));
        const size_t h2 = hash_vec3(std::get<2>(tri));
        size_t h = h0;
        h ^= h1 + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        h ^= h2 + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        return h;
    }
};

struct TriangleEqual {
    // 按距离判定：对应顶点距离小于 EPS 则认为三角形相同
    static constexpr float EPS = 1e-5f;
    static constexpr float EPS_SQ = EPS * EPS;

    static bool near(const vec3f& a, const vec3f& b) {
        return (a - b).length2() < EPS_SQ;
    }

    bool operator()(const Triangle& a, const Triangle& b) const noexcept {
        return near(std::get<0>(a), std::get<0>(b)) &&
               near(std::get<1>(a), std::get<1>(b)) &&
               near(std::get<2>(a), std::get<2>(b));
    }
};

// ------------------------------------------------------------------
// 精确 silhouette 裁剪：锥体切割辅助函数
// ------------------------------------------------------------------

struct ConePlanes {
    vec3f n01, n12, n20;
    float s01, s12, s20;
};

static ConePlanes build_cone_planes(const Triangle& tri, const vec3f& center) {
    const auto& v0 = std::get<0>(tri);
    const auto& v1 = std::get<1>(tri);
    const auto& v2 = std::get<2>(tri);
    ConePlanes cp;
    cp.n01 = (v1 - center).cross(v0 - center);
    cp.n12 = (v2 - center).cross(v1 - center);
    cp.n20 = (v0 - center).cross(v2 - center);
    cp.s01 = cp.n01.dot(v2 - center);
    cp.s12 = cp.n12.dot(v0 - center);
    cp.s20 = cp.n20.dot(v1 - center);
    return cp;
}

static void tri_aabb(const Triangle& tri, vec3f& out_min, vec3f& out_max) {
    const auto& v0 = std::get<0>(tri);
    const auto& v1 = std::get<1>(tri);
    const auto& v2 = std::get<2>(tri);
    out_min.x = std::min(v0.x, std::min(v1.x, v2.x));
    out_min.y = std::min(v0.y, std::min(v1.y, v2.y));
    out_min.z = std::min(v0.z, std::min(v1.z, v2.z));
    out_max.x = std::max(v0.x, std::max(v1.x, v2.x));
    out_max.y = std::max(v0.y, std::max(v1.y, v2.y));
    out_max.z = std::max(v0.z, std::max(v1.z, v2.z));
}

static void cone_aabb(const Triangle& tri,
                      const vec3f& center,
                      vec3f& out_min,
                      vec3f& out_max) {
    const auto& v0 = std::get<0>(tri);
    const auto& v1 = std::get<1>(tri);
    const auto& v2 = std::get<2>(tri);
    out_min.x = std::min(center.x, std::min(v0.x, std::min(v1.x, v2.x)));
    out_min.y = std::min(center.y, std::min(v0.y, std::min(v1.y, v2.y)));
    out_min.z = std::min(center.z, std::min(v0.z, std::min(v1.z, v2.z)));
    out_max.x = std::max(center.x, std::max(v0.x, std::max(v1.x, v2.x)));
    out_max.y = std::max(center.y, std::max(v0.y, std::max(v1.y, v2.y)));
    out_max.z = std::max(center.z, std::max(v0.z, std::max(v1.z, v2.z)));
}

static vec3f tri_centroid(const Triangle& tri) {
    const auto& v0 = std::get<0>(tri);
    const auto& v1 = std::get<1>(tri);
    const auto& v2 = std::get<2>(tri);
    return (v0 + v1 + v2) * (1.0f / 3.0f);
}

static std::vector<vec3f> clip_by_plane(const std::vector<vec3f>& poly,
                                        const vec3f& center,
                                        const vec3f& normal,
                                        float sign) {
    std::vector<vec3f> out;
    if (poly.empty())
        return out;

    if (std::abs(sign) < 1e-12f)
        sign = 1.0f;

    const float EPS = 1e-5f;

    for (size_t i = 0; i < poly.size(); ++i) {
        const vec3f& curr = poly[i];
        const vec3f& next = poly[(i + 1) % poly.size()];

        float dc = normal.dot(curr - center) * sign;
        float dn = normal.dot(next - center) * sign;

        bool curr_out = dc <= EPS;
        bool next_out = dn <= EPS;

        if (curr_out && next_out) {
            out.push_back(next);
        } else if (curr_out && !next_out) {
            float t = dc / (dc - dn);
            out.push_back(curr + (next - curr) * t);
        } else if (!curr_out && next_out) {
            float t = dc / (dc - dn);
            out.push_back(curr + (next - curr) * t);
            out.push_back(next);
        }
    }

    return out;
}

static std::vector<Triangle> clip_triangle_by_cone(const Triangle& tri,
                                                   const ConePlanes& cone,
                                                   const vec3f& center) {
    const auto& v0 = std::get<0>(tri);
    const auto& v1 = std::get<1>(tri);
    const auto& v2 = std::get<2>(tri);

    std::vector<vec3f> poly = {v0, v1, v2};

    poly = clip_by_plane(poly, center, cone.n01, cone.s01);
    if (poly.size() < 3)
        return {};

    poly = clip_by_plane(poly, center, cone.n12, cone.s12);
    if (poly.size() < 3)
        return {};

    poly = clip_by_plane(poly, center, cone.n20, cone.s20);
    if (poly.size() < 3)
        return {};

    // 去重相邻顶点
    std::vector<vec3f> cleaned;
    const float DUP_EPS_SQ = 1e-10f;
    for (const auto& p : poly) {
        if (cleaned.empty() || (p - cleaned.back()).length2() > DUP_EPS_SQ) {
            cleaned.push_back(p);
        }
    }
    if (cleaned.size() >= 3 &&
        (cleaned.front() - cleaned.back()).length2() <= DUP_EPS_SQ) {
        cleaned.pop_back();
    }
    if (cleaned.size() < 3)
        return {};

    // 扇形分解
    std::vector<Triangle> result;
    for (size_t i = 1; i + 1 < cleaned.size(); ++i) {
        result.emplace_back(
            std::make_tuple(cleaned[0], cleaned[i], cleaned[i + 1]));
    }
    return result;
}

// ------------------------------------------------------------------

std::vector<Triangle> Triangle_group::compute_visible_mesh_from_outside()
    const {
    std::vector<Triangle> visible_mesh;
    std::unordered_set<Triangle, TriangleHash, TriangleEqual> seen;

    for (size_t triangle_id = 0; triangle_id < triangles.size();
         ++triangle_id) {
        const Triangle_status& status = triangles[triangle_id];
        for (int face = 0; face < 6; ++face) {
            if (!triangle_has_valid_projection_on_face(status, face))
                continue;
            const auto [visible_triangles, original_polygons] =
                compute_visible_triangulation(triangle_id, face);
            for (const auto& tri2d : visible_triangles) {
                Triangle tri3d;
                bool valid = true;
                const auto& [v0, v1, v2] = status.triangle;
                for (int vi = 0; vi < 3; ++vi) {
                    const vec3f face_point =
                        face_plane_point(tri2d[vi], face, center);
                    const vec3f dir = face_point - center;
                    if (dir.length() < 1e-8f) {
                        valid = false;
                        break;
                    }
                    vec3f hit;
                    if (!ray_triangle_intersect(center, dir, v0, v1, v2, hit)) {
                        valid = false;
                        break;
                    }
                    if (vi == 0)
                        std::get<0>(tri3d) = hit;
                    else if (vi == 1)
                        std::get<1>(tri3d) = hit;
                    else
                        std::get<2>(tri3d) = hit;
                }
                if (!valid)
                    continue;
                if (seen.insert(tri3d).second)
                    visible_mesh.push_back(tri3d);
            }
        }
    }

    return visible_mesh;
}

std::vector<Triangle> Triangle_group::compute_visible_mesh_with_cone_sides()
    const {
    auto visible_mesh = compute_visible_mesh_from_outside();
    std::unordered_set<Triangle, TriangleHash, TriangleEqual> seen;

    // 初始化seen集合，防止重复添加已有的三角形
    for (const auto& tri : visible_mesh) {
        seen.insert(tri);
    }

    // 遍历所有6个面，生成侧面
    for (int face = 0; face < 6; ++face) {
        // 收集该面的所有可见边界
        for (size_t tid : face_triangle_indices[face]) {
            auto [visible_tris, polygons] =
                compute_visible_triangulation(tid, face);

            // 对于每个可见的多边形
            for (const auto& polygon : polygons) {
                // 遍历多边形的每条边
                for (size_t i = 0; i < polygon.size(); ++i) {
                    const auto& p1_2d = polygon[i];
                    const auto& p2_2d = polygon[(i + 1) % polygon.size()];

                    // 转换到3D坐标
                    vec3f v1 = face_plane_point(p1_2d, face, center);
                    vec3f v2 = face_plane_point(p2_2d, face, center);

                    // 计算两个顶点到中心的方向向量
                    vec3f dir1 = v1 - center;
                    vec3f dir2 = v2 - center;
                    float len1 = dir1.length();
                    float len2 = dir2.length();

                    // 如果任意一个点太靠近中心，跳过
                    if (len1 < 1e-8f || len2 < 1e-8f)
                        continue;

                    // 标准化方向向量
                    dir1 = dir1 / len1;
                    dir2 = dir2 / len2;

                    // 计算叉积判断是否共线
                    vec3f cross = dir1.cross(dir2);
                    float cross_len = cross.length();

                    // 如果不共线（叉积足够大），生成侧面三角形
                    if (cross_len >= 1e-6f) {
                        // 生成从边界到中心的三角形
                        Triangle tri{v1, v2, center};
                        if (seen.insert(tri).second) {
                            visible_mesh.push_back(tri);
                        }
                    }
                    // 如果共线，不生成侧面，防止非流形边
                }
            }
        }
    }

    return visible_mesh;
}

void Triangle_group::build_face_trees() {
    for (int face = 0; face < 6; ++face) {
        if (!face_segments[face].empty()) {
            face_trees[face].rebuild(face_segments[face].begin(),
                                     face_segments[face].end());
        }
    }
}

std::vector<Triangle> build_closed_mesh_from_triangles(
    const std::vector<Triangle>& input_triangles,
    bool auto_center,
    const vec3f& manual_center) {
    if (input_triangles.empty())
        return {};

    CB_DBG("build_closed_mesh: input=" << input_triangles.size());

    // 计算输入三角形的包围盒（用于缩放）
    vec3f min_p = std::get<0>(input_triangles[0]);
    vec3f max_p = min_p;
    for (const auto& tri : input_triangles) {
        const vec3f* verts[3] = {&std::get<0>(tri), &std::get<1>(tri),
                                 &std::get<2>(tri)};
        for (const auto* v : verts) {
            min_p.x = std::min(min_p.x, v->x);
            min_p.y = std::min(min_p.y, v->y);
            min_p.z = std::min(min_p.z, v->z);
            max_p.x = std::max(max_p.x, v->x);
            max_p.y = std::max(max_p.y, v->y);
            max_p.z = std::max(max_p.z, v->z);
        }
    }

    vec3f center = auto_center ? (min_p + max_p) * 0.5f : manual_center;
    const float half_x =
        std::max(std::abs(max_p.x - center.x), std::abs(center.x - min_p.x));
    const float half_y =
        std::max(std::abs(max_p.y - center.y), std::abs(center.y - min_p.y));
    const float half_z =
        std::max(std::abs(max_p.z - center.z), std::abs(center.z - min_p.z));
    const float min_half = std::min({half_x, half_y, half_z});

    CB_DBG("  bbox min=" << min_p.x << "," << min_p.y << "," << min_p.z
                         << " max=" << max_p.x << "," << max_p.y << ","
                         << max_p.z);
    CB_DBG("  center=" << center.x << "," << center.y << "," << center.z
                       << " min_half=" << min_half);

    // 若模型太贴近中心，则统一缩放使其超出算法使用的 0.5 单位立方体
    constexpr float target_half = 0.6f;
    const float scale = (min_half > 1e-8f && min_half < target_half)
                            ? target_half / min_half
                            : 1.0f;
    const float inv_scale = 1.0f / scale;

    CB_DBG("  scale=" << scale);

    const auto to_local = [&](const vec3f& v) { return (v - center) * scale; };
    const auto to_world = [&](const vec3f& v) {
        return v * inv_scale + center;
    };

    Triangle_group group;
    group.center = vec3f{0.0f, 0.0f, 0.0f};

    for (const auto& tri : input_triangles) {
        Triangle_status status;
        status.triangle = std::make_tuple(to_local(std::get<0>(tri)),
                                          to_local(std::get<1>(tri)),
                                          to_local(std::get<2>(tri)));
        status.compute_projection(group.center);
        group.add_triangle(status);
    }
    group.build_face_trees();

    CB_DBG("  group triangles=" << group.triangles.size());

    const std::vector<Triangle> local_mesh =
        group.compute_visible_mesh_with_cone_sides();

    CB_DBG("  local_mesh size=" << local_mesh.size());

    std::vector<Triangle> result;
    result.reserve(local_mesh.size());
    for (const auto& tri : local_mesh) {
        result.emplace_back(std::make_tuple(to_world(std::get<0>(tri)),
                                            to_world(std::get<1>(tri)),
                                            to_world(std::get<2>(tri))));
    }
    return result;
}

// =====================================================================
// Icosphere silhouette algorithm (new implementation).
//
// Replaces the old cone-clipping + boundary-edge-extraction approach
// with a subdivided icosahedron: rays are cast from center outward
// through each vertex; vertices that hit the input mesh move to the
// surface, vertices that miss are discarded.  Boundary edges (one face
// kept, one discarded) are sealed with triangles to center.
// =====================================================================
std::vector<Triangle> build_closed_mesh_from_triangles_silhouette(
    const std::vector<Triangle>& input_triangles,
    const vec3f& center,
    const std::function<bool()>& should_continue,
    const std::function<void(float, const std::string&)>& progress,
    int subdivision_level,
    float inner_wall_radius)
{
    if (input_triangles.empty())
        return {};
    
    CB_DBG("inner_wall_radius=" << inner_wall_radius);

    auto report_progress = [&](float t, const std::string& text) {
        if (progress) progress(t, text);
    };

    report_progress(0.0f, "Building BVH...");

    // ---- 1. Build BVH over input triangles ----
    using BVH = sinriv::kigstudio::dbvt3d<float, int>;
    BVH bvh;
    std::vector<BVH::AABB*> bvh_aabbs;
    std::vector<int> bvh_indices;
    bvh_aabbs.reserve(input_triangles.size());
    bvh_indices.reserve(input_triangles.size());

    for (int i = 0; i < static_cast<int>(input_triangles.size()); ++i) {
        const auto& tri = input_triangles[i];
        const auto& v0 = std::get<0>(tri);
        const auto& v1 = std::get<1>(tri);
        const auto& v2 = std::get<2>(tri);
        vec3f mn(v0.x, v0.y, v0.z), mx(mn);
        auto expand = [&](const vec3f& v) {
            mn.x = std::min(mn.x, v.x); mn.y = std::min(mn.y, v.y); mn.z = std::min(mn.z, v.z);
            mx.x = std::max(mx.x, v.x); mx.y = std::max(mx.y, v.y); mx.z = std::max(mx.z, v.z);
        };
        expand(v1); expand(v2);
        bvh_indices.push_back(i);
        bvh_aabbs.push_back(
            bvh.add(sinriv::kigstudio::vec3<float>(mn.x, mn.y, mn.z),
                    sinriv::kigstudio::vec3<float>(mx.x, mx.y, mx.z),
                    &bvh_indices.back()));
    }

    // ---- 2. Compute enclosing radius ----
    float max_dist2 = 0.0f;
    for (const auto& tri : input_triangles) {
        for (const auto& v : {std::get<0>(tri), std::get<1>(tri), std::get<2>(tri)}) {
            float d2 = (v - center).length2();
            if (d2 > max_dist2) max_dist2 = d2;
        }
    }
    const float radius = std::sqrt(max_dist2) * 2.0f + 1.0f;
    const float ray_len = radius * 3.0f;  // ray extends well beyond model

    // ---- 3. Create subdivided icosahedron ----
    const float phi = (1.0f + std::sqrt(5.0f)) / 2.0f;
    const float inv_norm = 1.0f / std::sqrt(1.0f + phi * phi);

    // 12 icosahedron vertices (normalized to radius)
    auto icosa_v = [&](float x, float y, float z) -> vec3f {
        float len = std::sqrt(x*x + y*y + z*z);
        float s = radius / len;
        return center + vec3f(x * s, y * s, z * s);
    };
    std::vector<vec3f> base_verts = {
        icosa_v(0, -1, -phi), icosa_v(0, -1, phi),
        icosa_v(0,  1, -phi), icosa_v(0,  1, phi),
        icosa_v(-1, -phi, 0), icosa_v(-1, phi, 0),
        icosa_v( 1, -phi, 0), icosa_v( 1, phi, 0),
        icosa_v(-phi, 0, -1), icosa_v(-phi, 0, 1),
        icosa_v( phi, 0, -1), icosa_v( phi, 0, 1),
    };

    // Auto-detect icosahedron edges from vertex distances.
    // In a regular icosahedron, the shortest inter-vertex distance
    // is the edge length; all 30 edges have this length.
    float edge_len2 = 1e30f;
    for (int i = 0; i < 12; ++i)
        for (int j = i + 1; j < 12; ++j) {
            float d2 = (base_verts[i] - base_verts[j]).length2();
            if (d2 < edge_len2) edge_len2 = d2;
        }
    edge_len2 *= 1.01f;  // slight tolerance

    // Build neighbor lists from edges
    std::vector<std::vector<int>> neighbors(12);
    for (int i = 0; i < 12; ++i)
        for (int j = i + 1; j < 12; ++j)
            if ((base_verts[i] - base_verts[j]).length2() < edge_len2) {
                neighbors[i].push_back(j);
                neighbors[j].push_back(i);
            }

    // Find all triangles where all 3 edge pairs exist.
    // Ensure CCW outward winding for each face.
    std::vector<std::array<int, 3>> base_faces_vec;
    for (int i = 0; i < 12; ++i) {
        for (int j : neighbors[i]) {
            if (j <= i) continue;
            for (int k : neighbors[j]) {
                if (k <= j) continue;
                // Check edge i-k exists
                bool has_ik = false;
                for (int n : neighbors[k])
                    if (n == i) { has_ik = true; break; }
                if (!has_ik) continue;

                // (i,j,k) is a face. Ensure outward normal.
                vec3f n = (base_verts[j] - base_verts[i])
                              .cross(base_verts[k] - base_verts[i]);
                vec3f cen = (base_verts[i] + base_verts[j] +
                             base_verts[k]) * (1.0f / 3.0f);
                if (n.dot(cen - center) < 0.0f)
                    base_faces_vec.push_back({i, k, j});
                else
                    base_faces_vec.push_back({i, j, k});
            }
        }
    }
    const int NF = (int)base_faces_vec.size();
    CB_DBG("  auto-detected " << NF << " icosahedron faces (expect 20)");

    // Clamp subdivision level to minimum 1 (no upper limit)
    const int SUBDIV = std::max(1, subdivision_level);

    // Build subdivided vertices and faces.
    // Use a map from snapped world position → vertex index for dedup.
    const float SNAP = 1e-4f;
    auto snap = [](float v) { return std::round(v / SNAP) * SNAP; };
    auto snap_vec = [&](const vec3f& v) -> vec3f { return {snap(v.x), snap(v.y), snap(v.z)}; };

    std::vector<vec3f> sub_verts;     // world-space vertices
    struct Vec3fCmp { bool operator()(const vec3f& a, const vec3f& b) const {
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.z < b.z; }};
    std::map<vec3f, int, Vec3fCmp> vert_map;  // snapped pos → index
    auto get_or_add_vert = [&](const vec3f& v) -> int {
        vec3f sv = snap_vec(v);
        auto it = vert_map.find(sv);
        if (it != vert_map.end()) return it->second;
        int idx = static_cast<int>(sub_verts.size());
        sub_verts.push_back(v);
        vert_map[sv] = idx;
        return idx;
    };

    // Subdivide each base face
    struct SubFace { int a, b, c; };
    std::vector<SubFace> sub_faces;

    for (int f = 0; f < NF; ++f) {
        int i0 = base_faces_vec[f][0], i1 = base_faces_vec[f][1], i2 = base_faces_vec[f][2];
        const vec3f& A = base_verts[i0];
        const vec3f& B = base_verts[i1];
        const vec3f& C = base_verts[i2];

        // Grid: row i, col j → point (i,j) where 0≤i≤SUBDIV, 0≤j≤SUBDIV-i
        // Position = slerp from A along AB and AC vectors, projected to sphere
        // Store grid-point vertex indices
        std::vector<std::vector<int>> grid(SUBDIV + 1);
        for (int i = 0; i <= SUBDIV; ++i) {
            grid[i].resize(SUBDIV - i + 1);
            for (int j = 0; j <= SUBDIV - i; ++j) {
                float bi = (float)i / SUBDIV;  // fraction along AB
                float bj = (float)j / SUBDIV;  // fraction along AC
                // Point = A + bi*(B-A) + bj*(C-A)
                vec3f pt = A + (B - A) * bi + (C - A) * bj;
                // Project to sphere
                vec3f dir = pt - center;
                float len = std::max(dir.length(), 1e-8f);
                vec3f sph = center + dir * (radius / len);
                grid[i][j] = get_or_add_vert(sph);
            }
        }

        // Generate sub-triangles within this face
        for (int i = 0; i < SUBDIV; ++i) {
            for (int j = 0; j < SUBDIV - i; ++j) {
                // "Up" triangle: (i,j), (i+1,j), (i,j+1)
                sub_faces.push_back({grid[i][j], grid[i+1][j], grid[i][j+1]});
                // "Down" triangle: (i+1,j), (i,j+1), (i+1,j+1) — if valid
                if (i + j + 1 < SUBDIV) {
                    sub_faces.push_back({grid[i+1][j], grid[i][j+1], grid[i+1][j+1]});
                }
            }
        }
    }

    CB_DBG("  icosahedron: " << sub_verts.size() << " verts, "
           << sub_faces.size() << " faces");

    // ---- DEBUG: output raw icosahedron (change to #if 1 to enable) ----
#if 0
    {
        std::vector<Triangle> debug_out;
        for (const auto& sf : sub_faces) {
            vec3f p0 = sub_verts[sf.a], p1 = sub_verts[sf.b],
                  p2 = sub_verts[sf.c];
            vec3f n = (p1 - p0).cross(p2 - p0);
            vec3f cen = (p0 + p1 + p2) * (1.0f / 3.0f);
            if (n.dot(cen - center) < 0.0f) std::swap(p1, p2);
            debug_out.emplace_back(std::make_tuple(p0, p1, p2));
        }
        for (auto* aabb : bvh_aabbs) {
            bvh.remove(aabb); bvh.delAABB(aabb);
        }
        CB_DBG("  DEBUG: returning raw icosahedron (" << debug_out.size()
               << " faces)");
        return debug_out;
    }
#endif

    report_progress(0.1f, "Casting rays...");

    // ---- 4. Build vertex neighbor graph ----
    std::vector<std::vector<int>> vert_neighbors(sub_verts.size());
    for (const auto& sf : sub_faces) {
        vert_neighbors[sf.a].push_back(sf.b);
        vert_neighbors[sf.a].push_back(sf.c);
        vert_neighbors[sf.b].push_back(sf.a);
        vert_neighbors[sf.b].push_back(sf.c);
        vert_neighbors[sf.c].push_back(sf.a);
        vert_neighbors[sf.c].push_back(sf.b);
    }
    // Deduplicate and sort neighbors by angle around vertex
    for (int vi = 0; vi < static_cast<int>(sub_verts.size()); ++vi) {
        auto& nb = vert_neighbors[vi];
        std::sort(nb.begin(), nb.end());
        nb.erase(std::unique(nb.begin(), nb.end()), nb.end());
        // Sort by angle around the radial direction
        vec3f radial = sub_verts[vi] - center;
        float rlen = radial.length();
        if (rlen < 1e-8f) continue;
        radial = radial * (1.0f / rlen);
        // Pick a tangent vector
        vec3f tangent = (std::abs(radial.x) < 0.9f)
                            ? vec3f(1, 0, 0)
                            : vec3f(0, 1, 0);
        tangent = (tangent - radial * radial.dot(tangent));
        tangent = tangent * (1.0f / std::max(tangent.length(), 1e-8f));
        vec3f bitangent = radial.cross(tangent);
        std::sort(nb.begin(), nb.end(),
                  [&](int a, int b) {
                      vec3f da = (sub_verts[a] - sub_verts[vi]);
                      vec3f db = (sub_verts[b] - sub_verts[vi]);
                      float ang_a = std::atan2(da.dot(bitangent), da.dot(tangent));
                      float ang_b = std::atan2(db.dot(bitangent), db.dot(tangent));
                      return ang_a < ang_b;
                  });
    }

    // ---- 5. Ray cast with supersampling ----
    // For each vertex A, also cast rays through sample points between
    // adjacent neighbor directions at 1/3 of the angular offset.
    // Any hit → vertex is "hit".  Multiple hits → average distance.
    auto cast_ray = [&](const vec3f& ray_origin, const vec3f& ray_dir_n,
                        float& out_dist, vec3f& out_pos) -> bool {
        float farthest = -1.0f;
        bvh.rayTest(
            sinriv::kigstudio::ray<float>(ray_origin,
                                          ray_origin + ray_dir_n * ray_len),
            [&](const BVH::AABB* node) {
                int idx = *node->data;
                if (idx < 0 || idx >= static_cast<int>(input_triangles.size()))
                    return;
                const auto& tri = input_triangles[idx];
                vec3f hp;
                if (ray_triangle_intersect(ray_origin, ray_dir_n,
                                           std::get<0>(tri), std::get<1>(tri),
                                           std::get<2>(tri), hp)) {
                    float t = (hp - ray_origin).length();
                    if (t > farthest) {
                        farthest = t;
                        out_pos = hp;
                    }
                }
            });
        if (farthest > 0.0f) {
            out_dist = farthest;
            return true;
        }
        return false;
    };

    std::vector<bool> hit(sub_verts.size(), false);
    std::vector<vec3f> hit_pos(sub_verts.size());

    for (int vi = 0; vi < static_cast<int>(sub_verts.size()); ++vi) {
        if (should_continue && !should_continue()) break;
        const vec3f& vt = sub_verts[vi];
        vec3f main_dir = vt - center;
        float main_len = main_dir.length();
        if (main_len < 1e-8f) continue;
        vec3f main_dir_n = main_dir * (1.0f / main_len);

        // Cast main ray through vertex
        float dist_sum = 0.0f;
        int hit_count = 0;
        float dummy_dist;
        vec3f dummy_pos;
        if (cast_ray(center, main_dir_n, dummy_dist, dummy_pos)) {
            dist_sum += dummy_dist;
            ++hit_count;
        }

        // Cast sample rays between adjacent neighbor pairs
        const auto& nb = vert_neighbors[vi];
        const int nb_count = static_cast<int>(nb.size());
        for (int ni = 0; ni < nb_count && nb_count >= 2; ++ni) {
            int n_next = (ni + 1) % nb_count;
            // Sample point: A + (AB_i + AB_{i+1})/6, projected to sphere
            vec3f sample_pt =
                vt + ((sub_verts[nb[ni]] - vt) +
                       (sub_verts[nb[n_next]] - vt)) * (1.0f / 6.0f);
            vec3f sample_dir = sample_pt - center;
            float sample_len = sample_dir.length();
            if (sample_len < 1e-8f) continue;
            sample_dir = sample_dir * (1.0f / sample_len);

            if (cast_ray(center, sample_dir, dummy_dist, dummy_pos)) {
                dist_sum += dummy_dist;
                ++hit_count;
            }
        }

        if (hit_count > 0) {
            hit[vi] = true;
            float avg_dist = dist_sum / static_cast<float>(hit_count);
            hit_pos[vi] = center + main_dir_n * avg_dist;
        }
    }

    // ---- 4b. Verify hits are on radial direction ----
    {
        int off_radial = 0;
        float max_deviation = 0;
        for (int vi = 0; vi < static_cast<int>(sub_verts.size()); ++vi) {
            if (!hit[vi]) continue;
            vec3f orig_dir = sub_verts[vi] - center;
            float orig_len = orig_dir.length();
            if (orig_len < 1e-8f) continue;
            orig_dir = orig_dir * (1.0f / orig_len);
            vec3f hit_dir = hit_pos[vi] - center;
            float hit_len = hit_dir.length();
            if (hit_len < 1e-8f) continue;
            hit_dir = hit_dir * (1.0f / hit_len);
            // Angle between original direction and hit direction
            float dot = orig_dir.dot(hit_dir);
            if (dot > 1.0f) dot = 1.0f;
            if (dot < -1.0f) dot = -1.0f;
            float angle = std::acos(dot);
            if (angle > 1e-4f) {  // > 0.0057 degrees
                ++off_radial;
                if (angle > max_deviation) max_deviation = angle;
            }
        }
        if (off_radial > 0) {
            CB_DBG("  WARNING: " << off_radial << " hits off radial, max angle="
                   << (max_deviation * 180.0f / 3.14159f) << " deg");
        } else {
            CB_DBG("  all hits on radial direction (OK)");
        }
    }

    report_progress(0.6f, "Building output mesh...");

    // ---- 4c. Inner wall: compute inner vertex positions ----
    const bool has_inner_wall = (inner_wall_radius > 0.0f);
    std::vector<vec3f> inner_pos;
    if (has_inner_wall) {
        inner_pos.resize(sub_verts.size());
        for (int vi = 0; vi < static_cast<int>(sub_verts.size()); ++vi) {
            if (!hit[vi]) continue;
            float dist = (hit_pos[vi] - center).length();
            if (dist <= inner_wall_radius) {
                throw std::runtime_error(
                    "Vertex distance " + std::to_string(dist) +
                    " <= inner_wall_radius " +
                    std::to_string(inner_wall_radius));
            }
            vec3f dir = (hit_pos[vi] - center) * (1.0f / dist);
            inner_pos[vi] = center + dir * inner_wall_radius;
        }
    }

    // ---- 5. Classify faces & collect boundary edges ----
    struct EdgeKey2 {
        int a, b;
        EdgeKey2(int va, int vb) : a(std::min(va, vb)), b(std::max(va, vb)) {}
        bool operator<(const EdgeKey2& o) const {
            return a != o.a ? a < o.a : b < o.b;
        }
    };

    // 5a. Initial classification: face candidates if all 3 vertices hit
    std::vector<bool> face_kept(sub_faces.size(), false);
    for (size_t fi = 0; fi < sub_faces.size(); ++fi) {
        const auto& sf = sub_faces[fi];
        if (hit[sf.a] && hit[sf.b] && hit[sf.c])
            face_kept[fi] = true;
    }

    // 5b. Generate surface faces; track output vertex order
    struct SurfFace { int va, vb, vc; };
    std::vector<SurfFace> surf_faces;
    surf_faces.reserve(sub_faces.size());
    std::vector<Triangle> result;
    result.reserve(sub_faces.size() * (has_inner_wall ? 2 : 1) + 128);

    for (size_t fi = 0; fi < sub_faces.size(); ++fi) {
        if (!face_kept[fi]) continue;
        const auto& sf = sub_faces[fi];
        int va = sf.a, vb = sf.b, vc = sf.c;
        vec3f p0 = hit_pos[va], p1 = hit_pos[vb], p2 = hit_pos[vc];

        vec3f n = (p1 - p0).cross(p2 - p0);
        if (n.length2() < 1e-12f) { face_kept[fi] = false; continue; }

        vec3f cen = (p0 + p1 + p2) * (1.0f / 3.0f);
        if (n.dot(cen - center) < 0.0f) {
            std::swap(p1, p2);
            std::swap(vb, vc);
        }
        result.emplace_back(std::make_tuple(p0, p1, p2));
        surf_faces.push_back({va, vb, vc});
    }

    // 5c. Edge counting from OUTPUT vertex order
    std::map<EdgeKey2, int> edge_face_count;
    for (const auto& sf : surf_faces) {
        edge_face_count[EdgeKey2(sf.va, sf.vb)]++;
        edge_face_count[EdgeKey2(sf.vb, sf.vc)]++;
        edge_face_count[EdgeKey2(sf.vc, sf.va)]++;
    }

    // 5d. Boundary edges: exactly one adjacent kept face
    std::vector<EdgeKey2> boundary_edges;
    for (const auto& [ek, cnt] : edge_face_count) {
        if (cnt == 1) boundary_edges.push_back(ek);
    }

    // ---- 6. Inner wall faces (if enabled) ----
    if (has_inner_wall) {
        for (const auto& sf : surf_faces) {
            vec3f p0 = inner_pos[sf.va];
            vec3f p1 = inner_pos[sf.vb];
            vec3f p2 = inner_pos[sf.vc];

            vec3f n = (p1 - p0).cross(p2 - p0);
            if (n.length2() < 1e-12f) continue;

            // Inner wall normal should point toward center
            vec3f cen = (p0 + p1 + p2) * (1.0f / 3.0f);
            if (n.dot(cen - center) > 0.0f) {
                std::swap(p1, p2);
            }
            result.emplace_back(std::make_tuple(p0, p1, p2));
        }
    }

    // ---- 7. Build sides: triangles to center or quads to inner wall ----
    result.reserve(result.size() + boundary_edges.size() * (has_inner_wall ? 2 : 1));

    for (const auto& ek : boundary_edges) {
        bool found = false;
        for (const auto& sf : surf_faces) {
            const int fe[3][2] = {{sf.va, sf.vb}, {sf.vb, sf.vc}, {sf.vc, sf.va}};
            for (int e = 0; e < 3; ++e) {
                EdgeKey2 ekey(fe[e][0], fe[e][1]);
                if (ekey.a == ek.a && ekey.b == ek.b) {
                    vec3f outer_a = hit_pos[fe[e][0]];
                    vec3f outer_b = hit_pos[fe[e][1]];

                    if (has_inner_wall) {
                        // Side quad: outer edge + inner edge → 2 triangles
                        // Current side direction is (outer_b, outer_a, center)
                        // Replace center with inner positions:
                        //   (outer_b, outer_a, inner_a)
                        //   (inner_b, outer_b, inner_a)
                        vec3f inner_a = inner_pos[fe[e][0]];
                        vec3f inner_b = inner_pos[fe[e][1]];
                        result.emplace_back(
                            std::make_tuple(outer_b, outer_a, inner_a));
                        result.emplace_back(
                            std::make_tuple(inner_b, outer_b, inner_a));
                    } else {
                        result.emplace_back(
                            std::make_tuple(outer_b, outer_a, center));
                    }
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
    }

    // Cleanup BVH
    for (auto* aabb : bvh_aabbs) {
        bvh.remove(aabb);
        bvh.delAABB(aabb);
    }

    report_progress(1.0f, "Done.");
    CB_DBG("  silhouette result: " << result.size() << " triangles ("
           << surf_faces.size() << " surface"
           << (has_inner_wall ? " + " + std::to_string(surf_faces.size()) + " inner" : "")
           << " + " << boundary_edges.size()
           << (has_inner_wall ? " quads (x2)" : " sides") << ")");

    return result;
}

// =====================================================================
// Old cone-clipping implementation (kept for reference)
// =====================================================================
std::vector<Triangle> build_closed_mesh_from_triangles_silhouette_old(
    const std::vector<Triangle>& /*input_triangles*/,
    const vec3f& /*center*/,
    const std::function<bool()>& /*should_continue*/,
    const std::function<void(float, const std::string&)>& /*progress*/) {
    // Old implementation replaced by icosahedron version.
    // Kept as a stub for reference.
    return {};
}

}  // namespace sinriv::kigstudio::mesh::conebox
