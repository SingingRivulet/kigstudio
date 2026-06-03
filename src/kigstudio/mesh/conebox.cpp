#include "conebox.h"
#include <algorithm>
#include <cmath>
#include <queue>
#include <tuple>

namespace sinriv::kigstudio::mesh::conebox {

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
                face_segments[face].emplace_back(projected_points[0],
                                                 projected_points[0]);
                face_segment_triangle_ids[face].push_back(index);
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

static Polygon_2 make_face_polygon(const Triangle_status& status, int face) {
    const auto& [p0, p1, p2] = status.pos_in_face[face];
    std::vector<Point2> vertices;
    vertices.reserve(3);
    if (p0.z > 0.0f)
        vertices.emplace_back(p0.x, p0.y);
    if (p1.z > 0.0f)
        vertices.emplace_back(p1.x, p1.y);
    if (p2.z > 0.0f)
        vertices.emplace_back(p2.x, p2.y);

    if (vertices.size() < 3)
        return Polygon_2();

    Polygon_2 polygon(vertices.begin(), vertices.end());
    if (!polygon.is_counterclockwise_oriented())
        polygon.reverse_orientation();

    return polygon;
}

static bool polygon_overlaps_triangle(const Polygon_2& polygon,
                                      const Polygon_2& other) {
    if (polygon.is_empty() || other.is_empty())
        return false;
    if (CGAL::do_intersect(polygon, other))
        return true;

    for (auto it = polygon.vertices_begin(); it != polygon.vertices_end();
         ++it) {
        if (other.bounded_side(*it) != CGAL::ON_UNBOUNDED_SIDE)
            return true;
    }
    for (auto it = other.vertices_begin(); it != other.vertices_end(); ++it) {
        if (polygon.bounded_side(*it) != CGAL::ON_UNBOUNDED_SIDE)
            return true;
    }
    return false;
}

std::tuple<std::vector<std::array<Point2, 3>>, std::vector<std::vector<Point2>>>
Triangle_group::compute_visible_triangulation(size_t triangle_id,
                                              int face_id) const {
    std::vector<std::array<Point2, 3>> out_triangles;
    std::vector<std::vector<Point2>> polygon_vertices;

    if (triangle_id >= triangles.size() || face_id < 0 || face_id >= 6)
        return {out_triangles, polygon_vertices};

    const Triangle_status& target_status = triangles[triangle_id];
    Polygon_2 target_polygon = make_face_polygon(target_status, face_id);
    if (target_polygon.size() < 3)
        return {out_triangles, polygon_vertices};

    const float target_depth = average_projected_depth(target_status, face_id);
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

    std::sort(
        occluders.begin(), occluders.end(),
        [](const Occluder& a, const Occluder& b) { return a.depth < b.depth; });

    std::vector<Polygon_with_holes_2> current_regions;
    current_regions.emplace_back(target_polygon);
    std::vector<Polygon_with_holes_2> next_regions;

    for (const auto& occluder : occluders) {
        next_regions.clear();
        for (const auto& region : current_regions) {
            CGAL::difference(region, occluder.polygon,
                             std::back_inserter(next_regions));
        }
        current_regions.swap(next_regions);
        if (current_regions.empty())
            break;
    }

    Clipper clipper;
    std::vector<Polygon_2> triangles_out;
    for (const auto& region : current_regions) {
        std::vector<Point2> outer_vertices;
        outer_vertices.reserve(region.outer_boundary().size());
        for (auto vit = region.outer_boundary().vertices_begin();
             vit != region.outer_boundary().vertices_end(); ++vit) {
            outer_vertices.push_back(*vit);
        }
        if (!outer_vertices.empty())
            polygon_vertices.push_back(std::move(outer_vertices));

        clipper(region, std::back_inserter(triangles_out));
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

struct TriangleHash {
    static size_t hash_vec3(const vec3f& p) {
        size_t h = std::hash<float>()(p.x);
        h ^= std::hash<float>()(p.y) + 0x9e3779b97f4a7c15ULL + (h << 6) +
             (h >> 2);
        h ^= std::hash<float>()(p.z) + 0x9e3779b97f4a7c15ULL + (h << 6) +
             (h >> 2);
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

std::vector<Triangle> Triangle_group::compute_visible_mesh_from_outside()
    const {
    std::vector<Triangle> visible_mesh;
    std::unordered_set<Triangle, TriangleHash> seen;

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

void Triangle_group::build_face_trees() {
    for (int face = 0; face < 6; ++face) {
        if (!face_segments[face].empty()) {
            face_trees[face].rebuild(face_segments[face].begin(),
                                     face_segments[face].end());
        }
    }
}

}  // namespace sinriv::kigstudio::mesh::conebox