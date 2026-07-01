#include "conebox.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <thread>
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
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Simple_cartesian.h>

namespace sinriv::kigstudio::mesh::conebox {

// #define CONEBOX_DEBUG 0

// #if CONEBOX_DEBUG
#define CB_DBG(x) std::cout << "[conebox] " << x << std::endl
// #else
// #define CB_DBG(x)
// #endif

// =====================================================================
// IcosahedronGenerator implementation
// =====================================================================

void IcosahedronGenerator::generate(float radius, const vec3f& center,
                                    std::vector<vec3f>& out_verts,
                                    std::vector<SubFace>& out_faces,
                                    std::vector<float>& out_min_distances) {
	const float phi = (1.0f + std::sqrt(5.0f)) / 2.0f;

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

	// Auto-detect icosahedron edges from vertex distances
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
				bool has_ik = false;
				for (int n : neighbors[k])
					if (n == i) {
						has_ik = true;
						break;
					}
				if (!has_ik)
					continue;

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

	const int SUBDIV = subdivision_level_;

	// Build subdivided vertices and faces.
	// Use a map from snapped world position -> vertex index for dedup.
	const float SNAP = 1e-4f;
	auto snap = [](float v) { return std::round(v / SNAP) * SNAP; };
	auto snap_vec = [&](const vec3f& v) -> vec3f {
		return {snap(v.x), snap(v.y), snap(v.z)};
	};

	struct Vec3fCmp {
		bool operator()(const vec3f& a, const vec3f& b) const {
			if (a.x != b.x) return a.x < b.x;
			if (a.y != b.y) return a.y < b.y;
			return a.z < b.z;
		}
	};
	std::map<vec3f, int, Vec3fCmp> vert_map;
	out_verts.clear();
	auto get_or_add_vert = [&](const vec3f& v) -> int {
		vec3f sv = snap_vec(v);
		auto it = vert_map.find(sv);
		if (it != vert_map.end())
			return it->second;
		int idx = static_cast<int>(out_verts.size());
		out_verts.push_back(v);
		vert_map[sv] = idx;
		return idx;
	};

	// Subdivide each base face
	out_faces.clear();
	for (int f = 0; f < NF; ++f) {
		int i0 = base_faces_vec[f][0], i1 = base_faces_vec[f][1],
		    i2 = base_faces_vec[f][2];
		const vec3f& A = base_verts[i0];
		const vec3f& B = base_verts[i1];
		const vec3f& C = base_verts[i2];

		std::vector<std::vector<int>> grid(SUBDIV + 1);
		for (int i = 0; i <= SUBDIV; ++i) {
			grid[i].resize(SUBDIV - i + 1);
			for (int j = 0; j <= SUBDIV - i; ++j) {
				float bi = (float)i / SUBDIV;
				float bj = (float)j / SUBDIV;
				vec3f pt = A + (B - A) * bi + (C - A) * bj;
				vec3f dir = pt - center;
				float len = std::max(dir.length(), 1e-8f);
				vec3f sph = center + dir * (radius / len);
				grid[i][j] = get_or_add_vert(sph);
			}
		}

		for (int i = 0; i < SUBDIV; ++i) {
			for (int j = 0; j < SUBDIV - i; ++j) {
				out_faces.push_back(
				    {grid[i][j], grid[i + 1][j], grid[i][j + 1]});
				if (i + j + 1 < SUBDIV) {
					out_faces.push_back(
					    {grid[i + 1][j], grid[i][j + 1],
					     grid[i + 1][j + 1]});
				}
			}
		}
	}

	CB_DBG("  icosahedron: " << out_verts.size() << " verts, "
	                         << out_faces.size() << " faces");
	out_min_distances.assign(out_verts.size(), -1.0f);
}

// =====================================================================
// DelaunaySphereGenerator implementation
// =====================================================================

void DelaunaySphereGenerator::generate(float radius, const vec3f& center,
                                       std::vector<vec3f>& out_verts,
                                       std::vector<SubFace>& out_faces,
                                       std::vector<float>& out_min_distances) {
	out_verts.clear();
	out_faces.clear();

	if (sample_points_.empty())
		return;

	// ---- 1. Project sample points to sphere, deduplicate ----
	const float SNAP = 1e-4f;
	auto snap = [](float v) { return std::round(v / SNAP) * SNAP; };

	struct Vec3fCmp {
		bool operator()(const vec3f& a, const vec3f& b) const {
			if (a.x != b.x) return a.x < b.x;
			if (a.y != b.y) return a.y < b.y;
			return a.z < b.z;
		}
	};
	std::map<vec3f, int, Vec3fCmp> vert_map;       // snapped sphere pos -> index
	std::map<vec3f, float, Vec3fCmp> dist_map;     // snapped sphere pos -> max original distance

	for (const auto& pt : sample_points_) {
		vec3f dir = pt - center;
		float orig_dist = dir.length();
		if (orig_dist < 1e-8f)
			continue;  // skip points at center
		vec3f dir_n = dir * (1.0f / orig_dist);
		vec3f sphere_pt = center + dir_n * radius;

		vec3f key = {snap(sphere_pt.x), snap(sphere_pt.y), snap(sphere_pt.z)};

		auto it = vert_map.find(key);
		if (it != vert_map.end()) {
			// Duplicate direction: keep the larger distance
			if (orig_dist > dist_map[key])
				dist_map[key] = orig_dist;
		} else {
			int idx = static_cast<int>(out_verts.size());
			out_verts.push_back(sphere_pt);
			vert_map[key] = idx;
			dist_map[key] = orig_dist;
		}
	}

	CB_DBG("  delaunay sphere: " << sample_points_.size()
	       << " sample points -> " << out_verts.size() << " unique verts");

	if (out_verts.size() < 3) {
		// Not enough points for triangulation
		out_min_distances.assign(out_verts.size(), -1.0f);
		return;
	}

	// ---- 2. CGAL 3D Delaunay triangulation (convex hull = spherical triangulation) ----
	using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
	using DT3 = CGAL::Delaunay_triangulation_3<Kernel>;

	DT3 dt3;

	// Build a map from snapped CGAL point -> our vertex index
	std::map<vec3f, int, Vec3fCmp> cgal_to_idx;  // snapped CGAL point -> index
	for (size_t i = 0; i < out_verts.size(); ++i) {
		const auto& v = out_verts[i];
		auto vh = dt3.insert(Kernel::Point_3(v.x, v.y, v.z));
		vec3f key = {snap(v.x), snap(v.y), snap(v.z)};
		cgal_to_idx[key] = static_cast<int>(i);
	}

	// ---- 3. Extract convex hull faces ----
	// Faces on the convex hull are those incident to the infinite vertex
	auto inf_v = dt3.infinite_vertex();
	std::vector<typename DT3::Cell_handle> inf_cells;
	dt3.incident_cells(inf_v, std::back_inserter(inf_cells));

	for (auto cell : inf_cells) {
		// Find the index of the infinite vertex in this cell
		int inf_idx = cell->index(inf_v);

		auto get_idx = [&](typename DT3::Vertex_handle vh) -> int {
			auto pt = vh->point();
			vec3f key = {snap(static_cast<float>(pt.x())),
			             snap(static_cast<float>(pt.y())),
			             snap(static_cast<float>(pt.z()))};
			return cgal_to_idx.at(key);
		};

		int a = get_idx(cell->vertex((inf_idx + 1) % 4));
		int b = get_idx(cell->vertex((inf_idx + 2) % 4));
		int c = get_idx(cell->vertex((inf_idx + 3) % 4));
		out_faces.push_back({a, b, c});
	}

	CB_DBG("  delaunay sphere: " << out_faces.size() << " faces");

	// ---- 4. Fill per-vertex min distances ----
	out_min_distances.resize(out_verts.size());
	for (size_t i = 0; i < out_verts.size(); ++i) {
		vec3f key = {snap(out_verts[i].x), snap(out_verts[i].y),
			         snap(out_verts[i].z)};
		out_min_distances[i] = dist_map.at(key);
	}
}

// =====================================================================
// IcosphereSilhouetteBuilder implementation
// =====================================================================

IcosphereSilhouetteBuilder::IcosphereSilhouetteBuilder(
    const std::vector<Triangle>& input_triangles,
    const vec3f& center,
    std::unique_ptr<ISilhouetteShapeGenerator> shape_generator,
    const std::function<bool()>& should_continue,
    const std::function<void(float, const std::string&)>& progress,
    float inner_wall_radius,
    float simplify_ratio)
    : input_triangles_(input_triangles)
    , center_(center)
    , shape_generator_(std::move(shape_generator))
    , should_continue_(should_continue)
    , progress_(progress)
    , inner_wall_radius_(inner_wall_radius)
    , simplify_ratio_(simplify_ratio)
    , has_inner_wall_(inner_wall_radius > 0.0f) {}

void IcosphereSilhouetteBuilder::report_progress(float t,
                                                  const std::string& text) {
	if (progress_)
		progress_(t, text);
}

bool IcosphereSilhouetteBuilder::check_cancel() {
	if (++cancel_step_ < 500)
		return false;
	cancel_step_ = 0;
	return should_continue_ && !should_continue_();
}

// ---- Phase 1: Build BVH over input triangles ----
void IcosphereSilhouetteBuilder::build_bvh() {
	bvh_aabbs_.reserve(input_triangles_.size());
	bvh_indices_.reserve(input_triangles_.size());

	for (int i = 0; i < static_cast<int>(input_triangles_.size()); ++i) {
		if (check_cancel())
			return;
		const auto& tri = input_triangles_[i];
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
		bvh_indices_.push_back(i);
		bvh_aabbs_.push_back(
		    bvh_.add(sinriv::kigstudio::vec3<float>(mn.x, mn.y, mn.z),
		             sinriv::kigstudio::vec3<float>(mx.x, mx.y, mx.z),
		             &bvh_indices_.back()));
	}
}

// ---- Phase 2: Compute enclosing radius ----
void IcosphereSilhouetteBuilder::compute_radius() {
	float max_dist2 = 0.0f;
	for (size_t ri = 0; ri < input_triangles_.size(); ++ri) {
		if (check_cancel())
			return;
		const auto& tri = input_triangles_[ri];
		for (const auto& v :
		     {std::get<0>(tri), std::get<1>(tri), std::get<2>(tri)}) {
			float d2 = (v - center_).length2();
			if (d2 > max_dist2)
				max_dist2 = d2;
		}
	}
	radius_ = std::sqrt(max_dist2) * 2.0f + 1.0f;
	ray_len_ = radius_ * 3.0f;  // ray extends well beyond model
}

// ---- Phase 3: Generate enclosing shape vertices and faces ----
void IcosphereSilhouetteBuilder::generate_shape() {
	shape_generator_->generate(radius_, center_, sub_verts_, sub_faces_,
	                           vertex_min_distances_);
}

// ---- Phase 4: Build vertex neighbor graph ----
void IcosphereSilhouetteBuilder::build_neighbor_graph() {
	vert_neighbors_.assign(sub_verts_.size(), {});
	for (const auto& sf : sub_faces_) {
		vert_neighbors_[sf.a].push_back(sf.b);
		vert_neighbors_[sf.a].push_back(sf.c);
		vert_neighbors_[sf.b].push_back(sf.a);
		vert_neighbors_[sf.b].push_back(sf.c);
		vert_neighbors_[sf.c].push_back(sf.a);
		vert_neighbors_[sf.c].push_back(sf.b);
	}
	// Deduplicate and sort neighbors by angle around vertex
	for (int vi = 0; vi < static_cast<int>(sub_verts_.size()); ++vi) {
		auto& nb = vert_neighbors_[vi];
		std::sort(nb.begin(), nb.end());
		nb.erase(std::unique(nb.begin(), nb.end()), nb.end());
		// Sort by angle around the radial direction
		vec3f radial = sub_verts_[vi] - center_;
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
			vec3f da = (sub_verts_[a] - sub_verts_[vi]);
			vec3f db = (sub_verts_[b] - sub_verts_[vi]);
			float ang_a = std::atan2(da.dot(bitangent), da.dot(tangent));
			float ang_b = std::atan2(db.dot(bitangent), db.dot(tangent));
			return ang_a < ang_b;
		});
	}
}

// ---- Phase 5: Ray cast with supersampling ----
void IcosphereSilhouetteBuilder::cast_rays() {
	auto cast_ray = [this](const vec3f& ray_origin, const vec3f& ray_dir_n,
	                       float& out_dist, vec3f& out_pos) -> bool {
		float farthest = -1.0f;
		bvh_.rayTest(
		    sinriv::kigstudio::ray<float>(
		        ray_origin, ray_origin + ray_dir_n * ray_len_),
		    [&](const BVH::AABB* node) {
			    int idx = *node->data;
			    if (idx < 0 ||
			        idx >= static_cast<int>(input_triangles_.size()))
				    return;
			    const auto& tri = input_triangles_[idx];
			    vec3f hp;
			    if (ray_triangle_intersect(ray_origin, ray_dir_n,
			                               std::get<0>(tri),
			                               std::get<1>(tri),
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

	hit_.assign(sub_verts_.size(), false);
	hit_pos_.resize(sub_verts_.size());

	for (int vi = 0; vi < static_cast<int>(sub_verts_.size()); ++vi) {
		if (check_cancel())
			break;
		const vec3f& vt = sub_verts_[vi];
		vec3f main_dir = vt - center_;
		float main_len = main_dir.length();
		if (main_len < 1e-8f)
			continue;
		vec3f main_dir_n = main_dir * (1.0f / main_len);

		// Cast main ray through vertex
		float dist_sum = 0.0f;
		int hit_count = 0;
		float dummy_dist;
		vec3f dummy_pos;
		if (cast_ray(center_, main_dir_n, dummy_dist, dummy_pos)) {
			dist_sum += dummy_dist;
			++hit_count;
		}

		// Cast sample rays between adjacent neighbor pairs
		const auto& nb = vert_neighbors_[vi];
		const int nb_count = static_cast<int>(nb.size());
		for (int ni = 0; ni < nb_count && nb_count >= 2; ++ni) {
			int n_next = (ni + 1) % nb_count;
			vec3f sample_pt =
			    vt +
			    ((sub_verts_[nb[ni]] - vt) + (sub_verts_[nb[n_next]] - vt)) *
			        (1.0f / 6.0f);
			vec3f sample_dir = sample_pt - center_;
			float sample_len = sample_dir.length();
			if (sample_len < 1e-8f)
				continue;
			sample_dir = sample_dir * (1.0f / sample_len);

			if (cast_ray(center_, sample_dir, dummy_dist, dummy_pos)) {
				dist_sum += dummy_dist;
				++hit_count;
			}
		}

		float min_dist = vertex_min_distances_[vi];

		if (min_dist > 0.0f) {
			// Always "hit": clamp position to >= min_dist from center
			hit_[vi] = true;
			if (hit_count > 0) {
				float avg_dist = dist_sum / static_cast<float>(hit_count);
				hit_pos_[vi] =
				    center_ + main_dir_n * std::max(avg_dist, min_dist);
			} else {
				hit_pos_[vi] = center_ + main_dir_n * min_dist;
			}
		} else if (hit_count > 0) {
			hit_[vi] = true;
			float avg_dist = dist_sum / static_cast<float>(hit_count);
			hit_pos_[vi] = center_ + main_dir_n * avg_dist;
		}
	}
}

// ---- Phase 5b: Verify hits are on radial direction ----
void IcosphereSilhouetteBuilder::verify_hits() {
	int off_radial = 0;
	float max_deviation = 0;
	for (int vi = 0; vi < static_cast<int>(sub_verts_.size()); ++vi) {
		if (!hit_[vi])
			continue;
		vec3f orig_dir = sub_verts_[vi] - center_;
		float orig_len = orig_dir.length();
		if (orig_len < 1e-8f)
			continue;
		orig_dir = orig_dir * (1.0f / orig_len);
		vec3f hit_dir = hit_pos_[vi] - center_;
		float hit_len = hit_dir.length();
		if (hit_len < 1e-8f)
			continue;
		hit_dir = hit_dir * (1.0f / hit_len);
		float dot = orig_dir.dot(hit_dir);
		if (dot > 1.0f)
			dot = 1.0f;
		if (dot < -1.0f)
			dot = -1.0f;
		float angle = std::acos(dot);
		if (angle > 1e-4f) {
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

// ---- Phase 6: Inner wall vertex positions ----
void IcosphereSilhouetteBuilder::compute_inner_wall() {
	if (!has_inner_wall_)
		return;

	inner_pos_.resize(sub_verts_.size());
	for (int vi = 0; vi < static_cast<int>(sub_verts_.size()); ++vi) {
		if (!hit_[vi])
			continue;
		float dist = (hit_pos_[vi] - center_).length();
		if (dist <= inner_wall_radius_) {
			throw std::runtime_error(
			    "Vertex distance " + std::to_string(dist) +
			    " <= inner_wall_radius " +
			    std::to_string(inner_wall_radius_));
		}
		vec3f dir = (hit_pos_[vi] - center_) * (1.0f / dist);
		inner_pos_[vi] = center_ + dir * inner_wall_radius_;
	}
}

// ---- Phase 7: Classify faces & collect boundary edges ----
void IcosphereSilhouetteBuilder::classify_faces_and_collect_boundaries() {
	// 7a. Initial classification: face candidates if all 3 vertices hit
	std::vector<bool> face_kept(sub_faces_.size(), false);
	for (size_t fi = 0; fi < sub_faces_.size(); ++fi) {
		if (check_cancel())
			return;
		const auto& sf = sub_faces_[fi];
		if (hit_[sf.a] && hit_[sf.b] && hit_[sf.c])
			face_kept[fi] = true;
	}

	// 7b. Generate surface faces; track output vertex order
	surf_faces_.clear();
	surf_faces_.reserve(sub_faces_.size());
	result_.clear();
	result_.reserve(sub_faces_.size() * (has_inner_wall_ ? 2 : 1) + 128);

	for (size_t fi = 0; fi < sub_faces_.size(); ++fi) {
		if (!face_kept[fi])
			continue;
		const auto& sf = sub_faces_[fi];
		int va = sf.a, vb = sf.b, vc = sf.c;
		vec3f p0 = hit_pos_[va], p1 = hit_pos_[vb], p2 = hit_pos_[vc];

		vec3f n = (p1 - p0).cross(p2 - p0);
		if (n.length2() < 1e-12f) {
			face_kept[fi] = false;
			continue;
		}

		vec3f cen = (p0 + p1 + p2) * (1.0f / 3.0f);
		if (n.dot(cen - center_) < 0.0f) {
			std::swap(p1, p2);
			std::swap(vb, vc);
		}
		result_.emplace_back(std::make_tuple(p0, p1, p2));
		surf_faces_.push_back({va, vb, vc});
	}

	// 7c. Edge counting from OUTPUT vertex order
	std::map<EdgeKey2, int> edge_face_count;
	for (const auto& sf : surf_faces_) {
		edge_face_count[EdgeKey2(sf.va, sf.vb)]++;
		edge_face_count[EdgeKey2(sf.vb, sf.vc)]++;
		edge_face_count[EdgeKey2(sf.vc, sf.va)]++;
	}

	// 7d. Boundary edges: exactly one adjacent kept face
	boundary_edges_.clear();
	for (const auto& [ek, cnt] : edge_face_count) {
		if (cnt == 1)
			boundary_edges_.push_back(ek);
	}
}

// ---- Phase 8: Inner wall faces (if enabled) ----
void IcosphereSilhouetteBuilder::generate_inner_wall_faces() {
	if (!has_inner_wall_)
		return;

	for (const auto& sf : surf_faces_) {
		vec3f p0 = inner_pos_[sf.va];
		vec3f p1 = inner_pos_[sf.vb];
		vec3f p2 = inner_pos_[sf.vc];

		vec3f n = (p1 - p0).cross(p2 - p0);
		if (n.length2() < 1e-12f)
			continue;

		// Inner wall normal should point toward center
		vec3f cen = (p0 + p1 + p2) * (1.0f / 3.0f);
		if (n.dot(cen - center_) > 0.0f) {
			std::swap(p1, p2);
		}
		result_.emplace_back(std::make_tuple(p0, p1, p2));
	}
}

// ---- Phase 9: Build sides ----
void IcosphereSilhouetteBuilder::build_side_triangles() {
	result_.reserve(result_.size() +
	                boundary_edges_.size() * (has_inner_wall_ ? 2 : 1));

	for (const auto& ek : boundary_edges_) {
		bool found = false;
		for (const auto& sf : surf_faces_) {
			const int fe[3][2] = {
			    {sf.va, sf.vb}, {sf.vb, sf.vc}, {sf.vc, sf.va}};
			for (int e = 0; e < 3; ++e) {
				EdgeKey2 ekey(fe[e][0], fe[e][1]);
				if (ekey.a == ek.a && ekey.b == ek.b) {
					vec3f outer_a = hit_pos_[fe[e][0]];
					vec3f outer_b = hit_pos_[fe[e][1]];

					if (has_inner_wall_) {
						vec3f inner_a = inner_pos_[fe[e][0]];
						vec3f inner_b = inner_pos_[fe[e][1]];
						result_.emplace_back(
						    std::make_tuple(outer_b, outer_a, inner_a));
						result_.emplace_back(
						    std::make_tuple(inner_b, outer_b, inner_a));
					} else {
						result_.emplace_back(
						    std::make_tuple(outer_b, outer_a, center_));
					}
					found = true;
					break;
				}
			}
			if (found)
				break;
		}
	}
}

// ---- Phase 10: Stitch borders & fill holes (CGAL async) ----
void IcosphereSilhouetteBuilder::stitch_and_fill() {
	sinriv::kigstudio::cgal::MeshData mesh_in;
	mesh_in.reserve(result_.size());
	for (const auto& tri : result_) {
		vec3f n = (std::get<1>(tri) - std::get<0>(tri))
		              .cross(std::get<2>(tri) - std::get<0>(tri));
		float nl = n.length();
		if (nl < 1e-12f)
			continue;
		mesh_in.emplace_back(tri, n * (1.0f / nl));
	}

	sinriv::kigstudio::cgal::stitch_borders_async async_stitch(mesh_in,
	                                                            0.001);
	while (!async_stitch.done()) {
		if (should_continue_ && !should_continue_()) {
			async_stitch.terminal();
			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	auto stitched = async_stitch.get_result();

	sinriv::kigstudio::cgal::fill_holes_async async_fill(stitched);
	while (!async_fill.done()) {
		if (should_continue_ && !should_continue_()) {
			async_fill.terminal();
			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	auto filled = async_fill.get_result();

	result_.clear();
	result_.reserve(filled.size());
	for (const auto& [tri, n] : filled)
		result_.push_back(tri);
}

// ---- Cleanup: free BVH nodes ----
void IcosphereSilhouetteBuilder::cleanup_bvh() {
	for (auto* aabb : bvh_aabbs_) {
		bvh_.remove(aabb);
		bvh_.delAABB(aabb);
	}
	bvh_aabbs_.clear();
}

// ---- Phase 11: CGAL edge-collapse simplification ----
void IcosphereSilhouetteBuilder::simplify_mesh() {
	if (simplify_ratio_ < 0.0f)
		return;

	float ratio = std::max(0.01f, std::min(1.0f, simplify_ratio_));
	std::vector<std::tuple<Triangle, vec3f>> mesh_in;
	mesh_in.reserve(result_.size());
	for (const auto& tri : result_) {
		vec3f n = (std::get<1>(tri) - std::get<0>(tri))
		              .cross(std::get<2>(tri) - std::get<0>(tri));
		float nl = n.length();
		if (nl < 1e-12f)
			continue;
		mesh_in.emplace_back(tri, n * (1.0f / nl));
	}

	sinriv::kigstudio::cgal::simplifyMesh_async async_simplify(mesh_in, ratio);
	while (!async_simplify.done()) {
		if (should_continue_ && !should_continue_()) {
			async_simplify.terminal();
			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	auto simplified = async_simplify.get_result();

	result_.clear();
	result_.reserve(simplified.size());
	for (const auto& [tri, n] : simplified)
		result_.push_back(tri);

	CB_DBG("  simplified: " << result_.size()
	                        << " triangles (ratio=" << ratio << ")");
}

// ---- Public: build() orchestrator ----
std::vector<Triangle> IcosphereSilhouetteBuilder::build() {
	if (input_triangles_.empty())
		return {};

	CB_DBG("inner_wall_radius=" << inner_wall_radius_);

	report_progress(0.0f, "Building BVH...");
	build_bvh();
	if (check_cancel()) { cleanup_bvh(); return {}; }

	compute_radius();
	if (check_cancel()) { cleanup_bvh(); return {}; }

	generate_shape();

	report_progress(0.1f, "Casting rays...");
	build_neighbor_graph();
	cast_rays();

	verify_hits();

	report_progress(0.6f, "Building output mesh...");
	compute_inner_wall();

	classify_faces_and_collect_boundaries();
	if (check_cancel()) { cleanup_bvh(); return {}; }

	generate_inner_wall_faces();
	build_side_triangles();

	report_progress(0.95f, "Stitching borders...");
	stitch_and_fill();
	if (check_cancel()) { cleanup_bvh(); return {}; }

	cleanup_bvh();

	report_progress(1.0f, "Done.");
	CB_DBG("  silhouette result: "
	       << result_.size() << " triangles (" << surf_faces_.size()
	       << " surface"
	       << (has_inner_wall_
	               ? " + " + std::to_string(surf_faces_.size()) + " inner"
	               : "")
	       << " + " << boundary_edges_.size()
	       << (has_inner_wall_ ? " quads (x2)" : " sides") << ")");

	report_progress(0.99f, "Simplifying mesh...");
	simplify_mesh();

	return result_;
	}

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
    auto generator = std::make_unique<IcosahedronGenerator>(subdivision_level);
    IcosphereSilhouetteBuilder builder(input_triangles, center,
                                       std::move(generator),
                                       should_continue, progress,
                                       inner_wall_radius, simplify_ratio);
    return builder.build();
}

}  // namespace sinriv::kigstudio::mesh::conebox
