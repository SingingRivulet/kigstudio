#include "conebox.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <tuple>

#include "kigstudio/utils/dbvt3d.h"

namespace sinriv::kigstudio::mesh::conebox {

#define CONEBOX_DEBUG 0

#if CONEBOX_DEBUG
    #define CB_DBG(x) std::cout << "[conebox] " << x << std::endl
#else
    #define CB_DBG(x)
#endif

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
    Point2 sp(std::round(x * SCALE) / SCALE,
              std::round(y * SCALE) / SCALE);
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
        const double cross = CGAL::to_double(
            (curr.x() - prev.x()) * (next.y() - prev.y()) -
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
    CB_DBG("make_face_polygon face=" << face
           << " raw=" << p0.x << "," << p0.y << "," << p0.z
           << " " << p1.x << "," << p1.y << "," << p1.z
           << " " << p2.x << "," << p2.y << "," << p2.z);
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
        const double cross =
            CGAL::to_double((b.x() - a.x()) * (c.y() - a.y()) -
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

static bool polygon_overlaps_triangle(const Polygon_2& a,
                                      const Polygon_2& b) {
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
            if (x < min_x) min_x = x;
            if (x > max_x) max_x = x;
            if (y < min_y) min_y = y;
            if (y > max_y) max_y = y;
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
                                         const Polygon_2& contained)
        -> bool {
        for (auto it = contained.vertices_begin();
             it != contained.vertices_end(); ++it) {
            if (container.bounded_side(*it) == CGAL::ON_BOUNDED_SIDE)
                return true;
        }
        return false;
    };

    int occluder_idx = 0;
    for (const auto& occluder : occluders) {
        CB_DBG("  applying occluder " << occluder_idx
               << " id=" << occluder.id << " depth=" << occluder.depth
               << " poly_size=" << occluder.polygon.size());
        next_regions.clear();
        int region_idx = 0;
        for (const auto& region : current_regions) {
            CB_DBG("    diff with region " << region_idx
                   << " outer=" << region.outer_boundary().size());

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
    const vec3f dirs[3] = {
        vec3f(1.0f, 0.0f, 0.0f),
        vec3f(0.0f, 1.0f, 0.0f),
        vec3f(0.0f, 0.0f, 1.0f)
    };
    int inside_votes = 0;

    for (int axis = 0; axis < 3; ++axis) {
        const vec3f& dir = dirs[axis];
        std::vector<float> ts;
        ts.reserve(tris.size());
        for (const auto& tri : tris) {
            vec3f hit;
            if (ray_triangle_intersect(p, dir,
                                       std::get<0>(tri), std::get<1>(tri),
                                       std::get<2>(tri), hit)) {
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

static ConePlanes build_cone_planes(const Triangle& tri,
                                    const vec3f& center) {
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

static void cone_aabb(const Triangle& tri, const vec3f& center,
                      vec3f& out_min, vec3f& out_max) {
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
    if (poly.empty()) return out;

    if (std::abs(sign) < 1e-12f) sign = 1.0f;

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

static std::vector<Triangle> clip_triangle_by_cone(
    const Triangle& tri, const ConePlanes& cone, const vec3f& center) {
    const auto& v0 = std::get<0>(tri);
    const auto& v1 = std::get<1>(tri);
    const auto& v2 = std::get<2>(tri);

    std::vector<vec3f> poly = {v0, v1, v2};

    poly = clip_by_plane(poly, center, cone.n01, cone.s01);
    if (poly.size() < 3) return {};

    poly = clip_by_plane(poly, center, cone.n12, cone.s12);
    if (poly.size() < 3) return {};

    poly = clip_by_plane(poly, center, cone.n20, cone.s20);
    if (poly.size() < 3) return {};

    // 去重相邻顶点
    std::vector<vec3f> cleaned;
    const float DUP_EPS_SQ = 1e-10f;
    for (const auto& p : poly) {
        if (cleaned.empty() ||
            (p - cleaned.back()).length2() > DUP_EPS_SQ) {
            cleaned.push_back(p);
        }
    }
    if (cleaned.size() >= 3 &&
        (cleaned.front() - cleaned.back()).length2() <= DUP_EPS_SQ) {
        cleaned.pop_back();
    }
    if (cleaned.size() < 3) return {};

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
           << " max=" << max_p.x << "," << max_p.y << "," << max_p.z);
    CB_DBG("  center=" << center.x << "," << center.y << "," << center.z
           << " min_half=" << min_half);

    // 若模型太贴近中心，则统一缩放使其超出算法使用的 0.5 单位立方体
    constexpr float target_half = 0.6f;
    const float scale =
        (min_half > 1e-8f && min_half < target_half) ? target_half / min_half
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
        status.triangle =
            std::make_tuple(to_local(std::get<0>(tri)),
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
        result.emplace_back(std::make_tuple(
            to_world(std::get<0>(tri)),
            to_world(std::get<1>(tri)),
            to_world(std::get<2>(tri))));
    }
    return result;
}

std::vector<Triangle> build_closed_mesh_from_triangles_silhouette(
    const std::vector<Triangle>& input_triangles,
    const vec3f& center,
    const std::function<bool()>& should_continue,
    const std::function<void(float)>& progress) {
    if (input_triangles.empty())
        return {};

    // =====================================================================
    // Phase 0: 精确锥体裁剪 — 用每个三角形与 center 构成的三面锥去切割
    //          被它遮挡的更远三角形，保留锥体外部（未遮挡区域）
    // =====================================================================
    std::vector<Triangle> fragments = input_triangles;
    std::vector<int> frag_src(input_triangles.size());
    std::iota(frag_src.begin(), frag_src.end(), 0);
    std::vector<float> src_depths;
    src_depths.reserve(input_triangles.size());
    for (const auto& tri : input_triangles) {
        src_depths.push_back((tri_centroid(tri) - center).length());
    }

    // 按原始三角形重心深度升序排序（近的优先处理）
    std::vector<int> order(input_triangles.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return src_depths[a] < src_depths[b];
    });

    std::vector<float> frag_depths = src_depths;

    // 检测 center 是否在 mesh 内部：沿 +X 方向发射射线，数交点
    bool center_inside = false;
    {
        vec3f ray_dir(1.0f, 0.0f, 0.0f);
        float ray_max = 0.0f;
        for (const auto& tri : input_triangles) {
            for (const auto& v : {std::get<0>(tri), std::get<1>(tri),
                                  std::get<2>(tri)}) {
                ray_max = std::max(ray_max, std::abs(v.x - center.x));
            }
        }
        ray_max += 1.0f;
        std::vector<float> ts;
        for (const auto& tri : input_triangles) {
            vec3f hit;
            if (ray_triangle_intersect(center, ray_dir,
                                       std::get<0>(tri), std::get<1>(tri),
                                       std::get<2>(tri), hit)) {
                float t = (hit - center).dot(ray_dir);
                if (t > 1e-6f && t < ray_max) {
                    ts.push_back(t);
                }
            }
        }
        std::sort(ts.begin(), ts.end());
        int intersect_count = 0;
        for (size_t i = 0; i < ts.size(); ++i) {
            if (i == 0 || std::abs(ts[i] - ts[i - 1]) > 1e-4f) {
                ++intersect_count;
            }
        }
        center_inside = (intersect_count % 2 == 1);
    }

    // 碎片数量上限，防止内存爆炸
    const size_t MAX_FRAGMENTS =
        std::max(input_triangles.size() * 20, size_t(10000));

    // 若 center 在内部，不需要锥体裁剪（所有面都可见，无遮挡）
    if (!center_inside) {
        for (size_t loop_idx = 0; loop_idx < order.size(); ++loop_idx) {
            int src_idx = order[loop_idx];
            if (should_continue && !should_continue())
                break;
            if (progress && loop_idx % 10 == 0) {
                progress(static_cast<float>(loop_idx) /
                         static_cast<float>(order.size()) * 0.5f);
            }
            if (fragments.size() > MAX_FRAGMENTS)
                break;

        const auto& src_tri = input_triangles[src_idx];
        ConePlanes cone = build_cone_planes(src_tri, center);

        // 跳过退化锥体（三角形与 center 近似共面）
        if (std::abs(cone.s01) < 1e-8f && std::abs(cone.s12) < 1e-8f &&
            std::abs(cone.s20) < 1e-8f) {
            continue;
        }

        vec3f cone_min, cone_max;
        cone_aabb(src_tri, center, cone_min, cone_max);

        std::vector<Triangle> new_fragments;
        std::vector<int> new_frag_src;
        std::vector<float> new_frag_depths;
        new_fragments.reserve(fragments.size());
        new_frag_src.reserve(fragments.size());
        new_frag_depths.reserve(fragments.size());

        for (size_t j = 0; j < fragments.size(); ++j) {
            // 不切割自己
            if (frag_src[j] == src_idx) {
                new_fragments.push_back(fragments[j]);
                new_frag_src.push_back(frag_src[j]);
                new_frag_depths.push_back(frag_depths[j]);
                continue;
            }

            // AABB 快速排斥
            vec3f fmin, fmax;
            tri_aabb(fragments[j], fmin, fmax);
            if (fmax.x < cone_min.x || fmin.x > cone_max.x ||
                fmax.y < cone_min.y || fmin.y > cone_max.y ||
                fmax.z < cone_min.z || fmin.z > cone_max.z) {
                new_fragments.push_back(fragments[j]);
                new_frag_src.push_back(frag_src[j]);
                new_frag_depths.push_back(frag_depths[j]);
                continue;
            }

            // 碎片来自比 src 更近的原始三角形，不切割
            if (frag_depths[j] < src_depths[src_idx] - 1e-4f) {
                new_fragments.push_back(fragments[j]);
                new_frag_src.push_back(frag_src[j]);
                new_frag_depths.push_back(frag_depths[j]);
                continue;
            }

            // 精确裁剪：保留锥体外部；若外部为空则保留原片
            //（让 silhouette 步骤处理完全遮挡的情况）
            auto clipped = clip_triangle_by_cone(fragments[j], cone, center);
            if (clipped.empty()) {
                new_fragments.push_back(fragments[j]);
                new_frag_src.push_back(frag_src[j]);
                new_frag_depths.push_back(frag_depths[j]);
            } else {
                for (const auto& ct : clipped) {
                    new_fragments.push_back(ct);
                    new_frag_src.push_back(frag_src[j]);
                    new_frag_depths.push_back(
                        (tri_centroid(ct) - center).length());
                }
            }
        }

        fragments = std::move(new_fragments);
        frag_src = std::move(new_frag_src);
        frag_depths = std::move(new_frag_depths);
    }
    }  // if (!center_inside)

    // =====================================================================
    // Phase 1+: 基于裁剪后的碎片运行 silhouette 算法
    // =====================================================================

    // 1. 构建 DBVT 加速射线查询
    using BVH = sinriv::kigstudio::dbvt3d<float, int>;
    BVH bvh;
    std::vector<BVH::AABB*> aabbs;
    std::vector<int> indices;
    aabbs.reserve(fragments.size());
    indices.reserve(fragments.size());

    for (int i = 0; i < static_cast<int>(fragments.size()); ++i) {
        if (should_continue && !should_continue())
            break;
        if (progress && i % 100 == 0) {
            progress(0.5f + static_cast<float>(i) /
                                static_cast<float>(fragments.size()) * 0.5f);
        }
        const auto& tri = fragments[i];
        const auto& v0 = std::get<0>(tri);
        const auto& v1 = std::get<1>(tri);
        const auto& v2 = std::get<2>(tri);
        vec3f mn(v0.x, v0.y, v0.z);
        vec3f mx(v0.x, v0.y, v0.z);
        mn.x = std::min(mn.x, std::min(v1.x, v2.x));
        mn.y = std::min(mn.y, std::min(v1.y, v2.y));
        mn.z = std::min(mn.z, std::min(v1.z, v2.z));
        mx.x = std::max(mx.x, std::max(v1.x, v2.x));
        mx.y = std::max(mx.y, std::max(v1.y, v2.y));
        mx.z = std::max(mx.z, std::max(v1.z, v2.z));
        indices.push_back(i);
        aabbs.push_back(bvh.add(mn, mx, &indices.back()));
    }

    // 2. 计算 mesh 最大范围，用于确定射线长度
    float max_dist = 0.0f;
    for (const auto& tri : fragments) {
        for (const auto& v :
             {std::get<0>(tri), std::get<1>(tri), std::get<2>(tri)}) {
            max_dist = std::max(max_dist, (v - center).length());
        }
    }
    max_dist = std::max(max_dist, 1.0f) * 3.0f;

    // 3. 对每个碎片发射射线，找最远交点
    std::vector<int> visible_flags(fragments.size(), 0);

    for (int i = 0; i < static_cast<int>(fragments.size()); ++i) {
        const auto& tri = fragments[i];
        const auto& v0 = std::get<0>(tri);
        const auto& v1 = std::get<1>(tri);
        const auto& v2 = std::get<2>(tri);
        const vec3f centroid = (v0 + v1 + v2) * (1.0f / 3.0f);
        const vec3f dir = centroid - center;
        const float dir_len = dir.length();
        if (dir_len < 1e-8f)
            continue;
        const vec3f dir_n = dir * (1.0f / dir_len);

        float farthest_t = -1.0f;
        int farthest_idx = -1;

        bvh.rayTest(
            sinriv::kigstudio::ray<float>(center, center + dir_n * max_dist),
            [&](const BVH::AABB* node) {
                int idx = *node->data;
                if (idx < 0 || idx >= static_cast<int>(fragments.size()))
                    return;
                const auto& t2 = fragments[idx];
                vec3f hit;
                if (ray_triangle_intersect(center, dir_n,
                                           std::get<0>(t2), std::get<1>(t2),
                                           std::get<2>(t2), hit)) {
                    float t = (hit - center).length();
                    if (t > farthest_t) {
                        farthest_t = t;
                        farthest_idx = idx;
                    }
                }
            });

        if (farthest_idx >= 0) {
            visible_flags[farthest_idx] = 1;
        }
    }

    // 收集可见碎片
    std::vector<Triangle> visible_tris;
    for (int i = 0; i < static_cast<int>(fragments.size()); ++i) {
        if (visible_flags[i]) {
            visible_tris.push_back(fragments[i]);
        }
    }

    if (visible_tris.empty()) {
        for (auto* aabb : aabbs) {
            bvh.remove(aabb);
            bvh.delAABB(aabb);
        }
        return {};
    }

    // 4. 提取边界边（只被一个可见三角形共享的边）
    // 对顶点做 snap，避免浮点误差导致切割后的相邻碎片产生重叠边界边
    struct EdgeKey {
        static float snap(float v) {
            const float SNAP_EPS = 1e-4f;
            return std::round(v / SNAP_EPS) * SNAP_EPS;
        }
        static vec3f snap_vec(const vec3f& v) {
            return vec3f(snap(v.x), snap(v.y), snap(v.z));
        }
        vec3f a, b;
        EdgeKey(const vec3f& v0, const vec3f& v1) {
            vec3f s0 = snap_vec(v0);
            vec3f s1 = snap_vec(v1);
            if (s0.x < s1.x || (s0.x == s1.x && s0.y < s1.y) ||
                (s0.x == s1.x && s0.y == s1.y && s0.z < s1.z)) {
                a = s0;
                b = s1;
            } else {
                a = s1;
                b = s0;
            }
        }
        bool operator<(const EdgeKey& o) const {
            if (a.x != o.a.x) return a.x < o.a.x;
            if (a.y != o.a.y) return a.y < o.a.y;
            if (a.z != o.a.z) return a.z < o.a.z;
            if (b.x != o.b.x) return b.x < o.b.x;
            if (b.y != o.b.y) return b.y < o.b.y;
            return b.z < o.b.z;
        }
    };

    std::map<EdgeKey, int> edge_count;
    for (const auto& tri : visible_tris) {
        const auto& v0 = std::get<0>(tri);
        const auto& v1 = std::get<1>(tri);
        const auto& v2 = std::get<2>(tri);
        edge_count[EdgeKey(v0, v1)]++;
        edge_count[EdgeKey(v1, v2)]++;
        edge_count[EdgeKey(v2, v0)]++;
    }

    std::vector<std::pair<vec3f, vec3f>> boundary_edges;
    for (const auto& kv : edge_count) {
        if (kv.second == 1) {
            boundary_edges.push_back({kv.first.a, kv.first.b});
        }
    }

    // 5. 调整可见面法线方向：使法线朝外（支持凹体，使用三条射线投票）
    for (auto& tri : visible_tris) {
        auto& v0 = std::get<0>(tri);
        auto& v1 = std::get<1>(tri);
        auto& v2 = std::get<2>(tri);
        vec3f n = (v1 - v0).cross(v2 - v0);
        float n_len = n.length();
        if (n_len < 1e-12f)
            continue;
        vec3f n_unit = n * (1.0f / n_len);
        vec3f centroid = (v0 + v1 + v2) * (1.0f / 3.0f);
        vec3f p_test = centroid + n_unit * 1e-3f;
        if (is_inside_mesh(p_test, input_triangles)) {
            std::swap(v1, v2);
        }
    }

    // 预计算可见面法线（用于侧面朝向判断）
    std::vector<vec3f> visible_normals;
    visible_normals.reserve(visible_tris.size());
    for (const auto& tri : visible_tris) {
        const auto& v0 = std::get<0>(tri);
        const auto& v1 = std::get<1>(tri);
        const auto& v2 = std::get<2>(tri);
        visible_normals.push_back((v1 - v0).cross(v2 - v0));
    }

    // 6. 生成侧面并调整法线：侧面法线应与邻接可见面法线同向（都朝外）
    std::vector<Triangle> result;
    result.reserve(visible_tris.size() + boundary_edges.size());
    result.insert(result.end(), visible_tris.begin(), visible_tris.end());

    const float EDGE_EPS_SQ = 1e-10f;
    auto edge_eq = [&](const vec3f& a, const vec3f& b) {
        return (a - b).length2() < EDGE_EPS_SQ;
    };

    for (const auto& e : boundary_edges) {
        vec3f n_F;
        bool found = false;
        for (size_t j = 0; j < visible_tris.size(); ++j) {
            const auto& tri = visible_tris[j];
            const auto& v0 = std::get<0>(tri);
            const auto& v1 = std::get<1>(tri);
            const auto& v2 = std::get<2>(tri);
            if ((edge_eq(v0, e.first) && edge_eq(v1, e.second)) ||
                (edge_eq(v1, e.first) && edge_eq(v2, e.second)) ||
                (edge_eq(v2, e.first) && edge_eq(v0, e.second)) ||
                (edge_eq(v0, e.second) && edge_eq(v1, e.first)) ||
                (edge_eq(v1, e.second) && edge_eq(v2, e.first)) ||
                (edge_eq(v2, e.second) && edge_eq(v0, e.first))) {
                n_F = visible_normals[j];
                found = true;
                break;
            }
        }

        vec3f a = e.first, b = e.second;
        vec3f n_T = (b - a).cross(center - a);
        if (found && n_T.dot(n_F) < 0.0f) {
            std::swap(a, b);
        }
        result.emplace_back(std::make_tuple(a, b, center));
    }

    // 清理 DBVT
    for (auto* aabb : aabbs) {
        bvh.remove(aabb);
        bvh.delAABB(aabb);
    }
    return result;
}

}  // namespace sinriv::kigstudio::mesh::conebox