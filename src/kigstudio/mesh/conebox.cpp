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

#include "kigstudio/cgal/mesh_repair.h"
#include "kigstudio/cgal/mesh_simplification.h"
#include "kigstudio/utils/dbvt3d.h"
#include "kigstudio/utils/locale.h"

#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Simple_cartesian.h>

namespace sinriv::kigstudio::mesh::conebox {

// #define CONEBOX_DEBUG 0

// #if CONEBOX_DEBUG
#define CB_DBG(x) std::cout << "[conebox] " << x << std::endl
// #else
// #define CB_DBG(x)
// #endif


bool ray_triangle_intersect(const vec3f& origin,
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
    float inner_wall_radius,
    float simplify_ratio) {
    if (input_triangles.empty())
        return {};

    CB_DBG("inner_wall_radius=" << inner_wall_radius);

    auto report_progress = [&](float t, const std::string& text) {
        if (progress)
            progress(t, text);
    };

    // Check cancellation roughly every N iterations in hot loops
    int cancel_step = 0;
    auto check_cancel = [&]() -> bool {
        if (++cancel_step < 500)
            return false;
        cancel_step = 0;
        return should_continue && !should_continue();
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
        if (check_cancel())
            return {};
        const auto& tri = input_triangles[i];
        const auto& v0 = std::get<0>(tri);
        const auto& v1 = std::get<1>(tri);
        const auto& v2 = std::get<2>(tri);
        vec3f mn(v0.x, v0.y, v0.z), mx(mn);
        auto expand = [&](const vec3f& v) {
            mn.x = std::min(mn.x, v.x);
            mn.y = std::min(mn.y, v.y);
            mn.z = std::min(mn.z, v.z);
            mx.x = std::max(mx.x, v.x);
            mx.y = std::max(mx.y, v.y);
            mx.z = std::max(mx.z, v.z);
        };
        expand(v1);
        expand(v2);
        bvh_indices.push_back(i);
        bvh_aabbs.push_back(
            bvh.add(sinriv::kigstudio::vec3<float>(mn.x, mn.y, mn.z),
                    sinriv::kigstudio::vec3<float>(mx.x, mx.y, mx.z),
                    &bvh_indices.back()));
    }

    // ---- 2. Compute enclosing radius ----
    float max_dist2 = 0.0f;
    for (size_t ri = 0; ri < input_triangles.size(); ++ri) {
        if (check_cancel())
            return {};
        const auto& tri = input_triangles[ri];
        for (const auto& v :
             {std::get<0>(tri), std::get<1>(tri), std::get<2>(tri)}) {
            float d2 = (v - center).length2();
            if (d2 > max_dist2)
                max_dist2 = d2;
        }
    }
    const float radius = std::sqrt(max_dist2) * 2.0f + 1.0f;
    const float ray_len = radius * 3.0f;  // ray extends well beyond model

    // ---- 3. Create subdivided icosahedron ----
    const float phi = (1.0f + std::sqrt(5.0f)) / 2.0f;
    const float inv_norm = 1.0f / std::sqrt(1.0f + phi * phi);

    // 12 icosahedron vertices (normalized to radius)
    auto icosa_v = [&](float x, float y, float z) -> vec3f {
        float len = std::sqrt(x * x + y * y + z * z);
        float s = radius / len;
        return center + vec3f(x * s, y * s, z * s);
    };
    std::vector<vec3f> base_verts = {
        icosa_v(0, -1, -phi), icosa_v(0, -1, phi),  icosa_v(0, 1, -phi),
        icosa_v(0, 1, phi),   icosa_v(-1, -phi, 0), icosa_v(-1, phi, 0),
        icosa_v(1, -phi, 0),  icosa_v(1, phi, 0),   icosa_v(-phi, 0, -1),
        icosa_v(-phi, 0, 1),  icosa_v(phi, 0, -1),  icosa_v(phi, 0, 1),
    };

    // Auto-detect icosahedron edges from vertex distances.
    // In a regular icosahedron, the shortest inter-vertex distance
    // is the edge length; all 30 edges have this length.
    float edge_len2 = 1e30f;
    for (int i = 0; i < 12; ++i)
        for (int j = i + 1; j < 12; ++j) {
            float d2 = (base_verts[i] - base_verts[j]).length2();
            if (d2 < edge_len2)
                edge_len2 = d2;
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
            if (j <= i)
                continue;
            for (int k : neighbors[j]) {
                if (k <= j)
                    continue;
                // Check edge i-k exists
                bool has_ik = false;
                for (int n : neighbors[k])
                    if (n == i) {
                        has_ik = true;
                        break;
                    }
                if (!has_ik)
                    continue;

                // (i,j,k) is a face. Ensure outward normal.
                vec3f n = (base_verts[j] - base_verts[i])
                              .cross(base_verts[k] - base_verts[i]);
                vec3f cen = (base_verts[i] + base_verts[j] + base_verts[k]) *
                            (1.0f / 3.0f);
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
    auto snap_vec = [&](const vec3f& v) -> vec3f {
        return {snap(v.x), snap(v.y), snap(v.z)};
    };

    std::vector<vec3f> sub_verts;  // world-space vertices
    struct Vec3fCmp {
        bool operator()(const vec3f& a, const vec3f& b) const {
            if (a.x != b.x)
                return a.x < b.x;
            if (a.y != b.y)
                return a.y < b.y;
            return a.z < b.z;
        }
    };
    std::map<vec3f, int, Vec3fCmp> vert_map;  // snapped pos → index
    auto get_or_add_vert = [&](const vec3f& v) -> int {
        vec3f sv = snap_vec(v);
        auto it = vert_map.find(sv);
        if (it != vert_map.end())
            return it->second;
        int idx = static_cast<int>(sub_verts.size());
        sub_verts.push_back(v);
        vert_map[sv] = idx;
        return idx;
    };

    // Subdivide each base face
    struct SubFace {
        int a, b, c;
    };
    std::vector<SubFace> sub_faces;

    for (int f = 0; f < NF; ++f) {
        int i0 = base_faces_vec[f][0], i1 = base_faces_vec[f][1],
            i2 = base_faces_vec[f][2];
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
                sub_faces.push_back(
                    {grid[i][j], grid[i + 1][j], grid[i][j + 1]});
                // "Down" triangle: (i+1,j), (i,j+1), (i+1,j+1) — if valid
                if (i + j + 1 < SUBDIV) {
                    sub_faces.push_back(
                        {grid[i + 1][j], grid[i][j + 1], grid[i + 1][j + 1]});
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
        if (rlen < 1e-8f)
            continue;
        radial = radial * (1.0f / rlen);
        // Pick a tangent vector
        vec3f tangent =
            (std::abs(radial.x) < 0.9f) ? vec3f(1, 0, 0) : vec3f(0, 1, 0);
        tangent = (tangent - radial * radial.dot(tangent));
        tangent = tangent * (1.0f / std::max(tangent.length(), 1e-8f));
        vec3f bitangent = radial.cross(tangent);
        std::sort(nb.begin(), nb.end(), [&](int a, int b) {
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
        if (should_continue && !should_continue())
            break;
        const vec3f& vt = sub_verts[vi];
        vec3f main_dir = vt - center;
        float main_len = main_dir.length();
        if (main_len < 1e-8f)
            continue;
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
                vt + ((sub_verts[nb[ni]] - vt) + (sub_verts[nb[n_next]] - vt)) *
                         (1.0f / 6.0f);
            vec3f sample_dir = sample_pt - center;
            float sample_len = sample_dir.length();
            if (sample_len < 1e-8f)
                continue;
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
            if (!hit[vi])
                continue;
            vec3f orig_dir = sub_verts[vi] - center;
            float orig_len = orig_dir.length();
            if (orig_len < 1e-8f)
                continue;
            orig_dir = orig_dir * (1.0f / orig_len);
            vec3f hit_dir = hit_pos[vi] - center;
            float hit_len = hit_dir.length();
            if (hit_len < 1e-8f)
                continue;
            hit_dir = hit_dir * (1.0f / hit_len);
            // Angle between original direction and hit direction
            float dot = orig_dir.dot(hit_dir);
            if (dot > 1.0f)
                dot = 1.0f;
            if (dot < -1.0f)
                dot = -1.0f;
            float angle = std::acos(dot);
            if (angle > 1e-4f) {  // > 0.0057 degrees
                ++off_radial;
                if (angle > max_deviation)
                    max_deviation = angle;
            }
        }
        if (off_radial > 0) {
            CB_DBG("  WARNING: " << off_radial << " hits off radial, max angle="
                                 << (max_deviation * 180.0f / 3.14159f)
                                 << " deg");
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
            if (!hit[vi])
                continue;
            float dist = (hit_pos[vi] - center).length();
            if (dist <= inner_wall_radius) {
                throw std::runtime_error("Vertex distance " +
                                         std::to_string(dist) +
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
        if (check_cancel())
            return {};
        const auto& sf = sub_faces[fi];
        if (hit[sf.a] && hit[sf.b] && hit[sf.c])
            face_kept[fi] = true;
    }

    // 5b. Generate surface faces; track output vertex order
    struct SurfFace {
        int va, vb, vc;
    };
    std::vector<SurfFace> surf_faces;
    surf_faces.reserve(sub_faces.size());
    std::vector<Triangle> result;
    result.reserve(sub_faces.size() * (has_inner_wall ? 2 : 1) + 128);

    for (size_t fi = 0; fi < sub_faces.size(); ++fi) {
        if (!face_kept[fi])
            continue;
        const auto& sf = sub_faces[fi];
        int va = sf.a, vb = sf.b, vc = sf.c;
        vec3f p0 = hit_pos[va], p1 = hit_pos[vb], p2 = hit_pos[vc];

        vec3f n = (p1 - p0).cross(p2 - p0);
        if (n.length2() < 1e-12f) {
            face_kept[fi] = false;
            continue;
        }

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
        if (cnt == 1)
            boundary_edges.push_back(ek);
    }

    // ---- 6. Inner wall faces (if enabled) ----
    if (has_inner_wall) {
        for (const auto& sf : surf_faces) {
            vec3f p0 = inner_pos[sf.va];
            vec3f p1 = inner_pos[sf.vb];
            vec3f p2 = inner_pos[sf.vc];

            vec3f n = (p1 - p0).cross(p2 - p0);
            if (n.length2() < 1e-12f)
                continue;

            // Inner wall normal should point toward center
            vec3f cen = (p0 + p1 + p2) * (1.0f / 3.0f);
            if (n.dot(cen - center) > 0.0f) {
                std::swap(p1, p2);
            }
            result.emplace_back(std::make_tuple(p0, p1, p2));
        }
    }

    // ---- 7. Build sides: triangles to center or quads to inner wall ----
    result.reserve(result.size() +
                   boundary_edges.size() * (has_inner_wall ? 2 : 1));

    for (const auto& ek : boundary_edges) {
        bool found = false;
        for (const auto& sf : surf_faces) {
            const int fe[3][2] = {
                {sf.va, sf.vb}, {sf.vb, sf.vc}, {sf.vc, sf.va}};
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
            if (found)
                break;
        }
    }

    // ---- Repair: stitch boundary edges then fill remaining holes ----
    // Convert result to MeshData (triangle + normal) for CGAL repair ops.
    report_progress(0.95f, "Stitching borders...");
    sinriv::kigstudio::cgal::MeshData mesh_in;
    mesh_in.reserve(result.size());
    for (const auto& tri : result) {
        vec3f n = (std::get<1>(tri) - std::get<0>(tri))
                      .cross(std::get<2>(tri) - std::get<0>(tri));
        float nl = n.length();
        if (nl < 1e-12f)
            continue;
        mesh_in.emplace_back(tri, n * (1.0f / nl));
    }

    sinriv::kigstudio::cgal::stitch_borders_async async_stitch(mesh_in, 0.001);
    while (!async_stitch.done()) {
        if (should_continue && !should_continue()) {
            async_stitch.terminal();
            return {};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    auto stitched = async_stitch.get_result();

    report_progress(0.97f, "Filling holes...");
    sinriv::kigstudio::cgal::fill_holes_async async_fill(stitched);
    while (!async_fill.done()) {
        if (should_continue && !should_continue()) {
            async_fill.terminal();
            return {};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    auto filled = async_fill.get_result();

    result.clear();
    result.reserve(filled.size());
    for (const auto& [tri, n] : filled)
        result.push_back(tri);

    // Cleanup BVH
    for (auto* aabb : bvh_aabbs) {
        bvh.remove(aabb);
        bvh.delAABB(aabb);
    }

    report_progress(1.0f, "Done.");
    CB_DBG("  silhouette result: "
           << result.size() << " triangles (" << surf_faces.size() << " surface"
           << (has_inner_wall
                   ? " + " + std::to_string(surf_faces.size()) + " inner"
                   : "")
           << " + " << boundary_edges.size()
           << (has_inner_wall ? " quads (x2)" : " sides") << ")");

    // ---- 7. CGAL edge-collapse simplification ----
    // Uses the existing simplifyMesh() which performs vertex welding,
    // edge collapse (ratio-based), and coplanar face merging.
    // simplify_ratio < 0 → skip simplification entirely.
    if (simplify_ratio >= 0.0f) {
        report_progress(0.99f, "Simplifying mesh...");
        float ratio = std::max(0.01f, std::min(1.0f, simplify_ratio));
        std::vector<std::tuple<Triangle, vec3f>> mesh_in;
        mesh_in.reserve(result.size());
        for (const auto& tri : result) {
            vec3f n = (std::get<1>(tri) - std::get<0>(tri))
                          .cross(std::get<2>(tri) - std::get<0>(tri));
            float nl = n.length();
            if (nl < 1e-12f)
                continue;
            mesh_in.emplace_back(tri, n * (1.0f / nl));
        }

        report_progress(0.99f, "Simplifying mesh processing...");

        sinriv::kigstudio::cgal::simplifyMesh_async async_simplify(mesh_in,
                                                                   ratio);
        while (!async_simplify.done()) {
            if (should_continue && !should_continue()) {
                async_simplify.terminal();
                return {};
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        auto simplified = async_simplify.get_result();

        result.clear();
        result.reserve(simplified.size());
        for (const auto& [tri, n] : simplified)
            result.push_back(tri);

        CB_DBG("  simplified: " << result.size()
                                << " triangles (ratio=" << ratio << ")");
    }

    return result;
}

}  // namespace sinriv::kigstudio::mesh::conebox
