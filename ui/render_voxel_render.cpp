#include "render_voxel_list.h"
#include <bx/math.h>
#include <chrono>
#include <map>
#include <unordered_map>
#include "kigstudio/cgal/mesh_repair.h"
#include "kigstudio/cgal/repair_ipc.h"
#include "kigstudio/mesh/loft.h"
namespace sinriv::ui::render {
namespace {
uint32_t pack_abgr(float r, float g, float b, float a) {
    const auto pack = [](float v) -> uint32_t {
        v = std::max(0.0f, std::min(1.0f, v));
        return static_cast<uint32_t>(v * 255.0f + 0.5f);
    };
    return (pack(a) << 24) | (pack(b) << 16) | (pack(g) << 8) | pack(r);
}

bool contains_index(const std::vector<int>& indices, int value) {
    return std::find(indices.begin(), indices.end(), value) != indices.end();
}

using vec3f = sinriv::kigstudio::voxel::vec3f;

// 评估三次贝塞尔曲线：B(t) = P0*(1-t)³ + P1*3(1-t)²t + P2*3(1-t)t² + P3*t³
inline vec3f bezier_eval(const vec3f& p0, const vec3f& p1,
                          const vec3f& p2, const vec3f& p3, float t) {
    const float u = 1.0f - t;
    const float u2 = u * u;
    const float t2 = t * t;
    return p0 * (u2 * u) +
           p1 * (3.0f * u2 * t) +
           p2 * (3.0f * u * t2) +
           p3 * (t2 * t);
}

// 将 Catmull-Rom 样条的关键点转换为多段三次贝塞尔，采样为折线
// guide_points: 用户拾取的点（曲线经过这些点）
// samples_per_segment: 每段贝塞尔的采样数
// 返回采样后的密集点集
std::vector<vec3f> sample_bezier_guide_curve(
    const std::vector<vec3f>& guide_points,
    int samples_per_segment = 32) {
    if (guide_points.size() < 2)
        return guide_points;

    std::vector<vec3f> result;
    const size_t n = guide_points.size();

    for (size_t i = 0; i + 1 < n; ++i) {
        // Catmull-Rom → cubic Bézier control points
        // 虚拟端点处理首尾
        vec3f p0 = guide_points[i];
        vec3f p3 = guide_points[i + 1];

        // 入切线：基于前一点
        vec3f p1;
        if (i == 0) {
            // 首段：反射 P₁ 得到虚拟 P₋₁
            p1 = p0 + (p3 - p0) * (1.0f / 3.0f);
        } else {
            p1 = p0 + (p3 - guide_points[i - 1]) * (1.0f / 6.0f);
        }

        // 出切线：基于后一点
        vec3f p2;
        if (i + 2 >= n) {
            // 末段：反射 Pₙ₋₂ 得到虚拟 Pₙ
            p2 = p3 - (p3 - p0) * (1.0f / 3.0f);
        } else {
            p2 = p3 - (guide_points[i + 2] - p0) * (1.0f / 6.0f);
        }

        // 采样贝塞尔段
        for (int s = 0; s < samples_per_segment; ++s) {
            float t = static_cast<float>(s) / static_cast<float>(samples_per_segment);
            result.push_back(bezier_eval(p0, p1, p2, p3, t));
        }
    }
    // 加入最后一个端点
    result.push_back(guide_points.back());

    return result;
}

// 在引导曲线上查找离 world_pos 最近的点
// 返回 {curve_id, curve_pos}，curve_id 整数部分=段索引，小数部分=段内t
struct NearestCurveResult {
    float curve_id = 0.0f;  // 整数部分=段索引, 小数部分=段内t
    vec3f curve_pos{};
};
NearestCurveResult find_nearest_on_bezier_guide(
    const std::vector<vec3f>& guide_points,
    const vec3f& world_pos,
    int samples_per_segment = 32) {
    NearestCurveResult result;
    if (guide_points.size() < 2) {
        if (!guide_points.empty()) {
            result.curve_pos = guide_points[0];
        }
        return result;
    }

    auto sampled = sample_bezier_guide_curve(guide_points, samples_per_segment);
    if (sampled.empty())
        return result;

    // 在采样点中找最近点
    float best_dist = std::numeric_limits<float>::max();
    size_t best_sample_idx = 0;
    for (size_t i = 0; i < sampled.size(); ++i) {
        float d = (sampled[i] - world_pos).length();
        if (d < best_dist) {
            best_dist = d;
            best_sample_idx = i;
        }
    }

    result.curve_pos = sampled[best_sample_idx];

    // 推算 curve_id：整数部分=段索引，小数部分=段内参数t
    const int kSamplesPerSegment = std::max(samples_per_segment, 1);
    size_t seg_idx = best_sample_idx / kSamplesPerSegment;
    if (seg_idx >= guide_points.size() - 1)
        seg_idx = guide_points.size() - 2;
    size_t sample_in_seg = best_sample_idx - seg_idx * kSamplesPerSegment;
    float t = static_cast<float>(sample_in_seg) /
              static_cast<float>(kSamplesPerSegment);
    result.curve_id = static_cast<float>(seg_idx) + t;

    return result;
}

template <class Vec3>
Vec3 extend_cone_edge(const Vec3& apex, const Vec3& vertex) {
    constexpr float kEdgeScale = 4.0f;
    return apex + (vertex - apex) * kEdgeScale;
}

bgfx::VertexLayout& concave_cone_overlay_layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;
    if (!initialized) {
        mesh_detail::ColorLineVertex::init(layout);
        initialized = true;
    }
    return layout;
}

void append_marker_circle(std::vector<mesh_detail::ColorLineVertex>& vertices,
                          const sinriv::kigstudio::voxel::vec3f& center,
                          const sinriv::kigstudio::voxel::vec3f& axis_u,
                          const sinriv::kigstudio::voxel::vec3f& axis_v,
                          float radius,
                          uint32_t color) {
    constexpr int kSegments = 48;
    constexpr float kPi = 3.14159265358979323846f;
    for (int i = 0; i < kSegments; ++i) {
        const float a0 = 2.0f * kPi * static_cast<float>(i) /
                         static_cast<float>(kSegments);
        const float a1 = 2.0f * kPi * static_cast<float>(i + 1) /
                         static_cast<float>(kSegments);
        const auto p0 = center + axis_u * (std::cos(a0) * radius) +
                        axis_v * (std::sin(a0) * radius);
        const auto p1 = center + axis_u * (std::cos(a1) * radius) +
                        axis_v * (std::sin(a1) * radius);
        vertices.push_back({p0.x, -p0.y, p0.z, color});
        vertices.push_back({p1.x, -p1.y, p1.z, color});
    }
}

sinriv::kigstudio::voxel::vec3f transform_point(
    const mat4f& matrix,
    const sinriv::kigstudio::voxel::vec3f& point) {
    sinriv::kigstudio::mat::vec4<float> transformed =
        sinriv::kigstudio::mat::vec4<float>(point.x, point.y, point.z, 1.0f) *
        matrix;
    auto v3 = transformed.toVec3();
    return {v3.x, v3.y, v3.z};
}
}  // namespace

// ============================================================================
// Hair strand loft mesh builder
// ============================================================================
namespace {

using loft_vec3f = sinriv::kigstudio::vec3<float>;
using loft_Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;

// Safe normalize: returns fallback if vector is too short
loft_vec3f safe_normalize(const loft_vec3f& v, const loft_vec3f& fallback) {
	float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	if (len < 1e-8f) return fallback;
	float inv = 1.0f / len;
	return loft_vec3f{v.x * inv, v.y * inv, v.z * inv};
}

// Compute triangle normal via cross product
loft_vec3f compute_triangle_normal(const loft_Triangle& tri) {
	const auto& v0 = std::get<0>(tri);
	const auto& v1 = std::get<1>(tri);
	const auto& v2 = std::get<2>(tri);
	float dx1 = v1.x - v0.x, dy1 = v1.y - v0.y, dz1 = v1.z - v0.z;
	float dx2 = v2.x - v0.x, dy2 = v2.y - v0.y, dz2 = v2.z - v0.z;
	float nx = dy1 * dz2 - dz1 * dy2;
	float ny = dz1 * dx2 - dx1 * dz2;
	float nz = dx1 * dy2 - dy1 * dx2;
	float len = std::sqrt(nx * nx + ny * ny + nz * nz);
	if (len < 1e-8f) return loft_vec3f{0, 0, 1};
	float inv = 1.0f / len;
	return loft_vec3f{nx * inv, ny * inv, nz * inv};
}

// Compute tangent at sample point i along the sampled curve
loft_vec3f tangent_at_sample(const std::vector<loft_vec3f>& sampled, int i) {
	int n = static_cast<int>(sampled.size());
	if (n < 2) return loft_vec3f{0, 0, 1};
	if (i <= 0)
		return safe_normalize(
		    loft_vec3f{sampled[1].x - sampled[0].x, sampled[1].y - sampled[0].y,
		               sampled[1].z - sampled[0].z},
		    loft_vec3f{0, 0, 1});
	if (i >= n - 1)
		return safe_normalize(
		    loft_vec3f{sampled[n - 1].x - sampled[n - 2].x,
		               sampled[n - 1].y - sampled[n - 2].y,
		               sampled[n - 1].z - sampled[n - 2].z},
		    loft_vec3f{0, 0, 1});
	return safe_normalize(
	    loft_vec3f{sampled[i + 1].x - sampled[i - 1].x,
	               sampled[i + 1].y - sampled[i - 1].y,
	               sampled[i + 1].z - sampled[i - 1].z},
	    loft_vec3f{0, 0, 1});
}

constexpr float kMinTipWidth = 0.02f;

// Catmull-Rom smoothing for a closed 2D polygon (matching the section editor
// preview).  Used for both global and per-point section Bézier mode.
inline void catmull_rom_smooth_closed(
    const std::vector<sinriv::kigstudio::vec2<float>>& input,
    std::vector<sinriv::kigstudio::vec2<float>>& output,
    int kSubdiv = 8) {
    const int n = static_cast<int>(input.size());
    if (n < 3) { output = input; return; }
    auto cr = [](float p0, float p1, float p2, float p3, float t) -> float {
        float t2 = t * t;
        float t3 = t2 * t;
        return 0.5f * ((2.0f * p1) + (-p0 + p2) * t +
                       (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                       (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
    };
    output.clear();
    output.reserve(n * kSubdiv);
    for (int i = 0; i < n; ++i) {
        int i0 = (i - 1 + n) % n;
        int i1 = i;
        int i2 = (i + 1) % n;
        int i3 = (i + 2) % n;
        const auto& p0 = input[i0];
        const auto& p1 = input[i1];
        const auto& p2 = input[i2];
        const auto& p3 = input[i3];
        for (int s = 0; s < kSubdiv; ++s) {
            float t = static_cast<float>(s) / static_cast<float>(kSubdiv);
            output.push_back({cr(p0.x, p1.x, p2.x, p3.x, t),
                              cr(p0.y, p1.y, p2.y, p3.y, t)});
        }
    }
}

// 封闭性（watertight）检测：量化顶点后统计每条无向边的引用次数，
// 封闭网格中每条边应恰好被两个三角形共享。
bool is_mesh_watertight(const std::vector<loft_Triangle>& tris) {
	if (tris.empty()) return false;
	std::map<std::tuple<int32_t, int32_t, int32_t>, uint32_t> vertex_ids;
	std::unordered_map<uint64_t, uint32_t> edge_count;
	auto get_id = [&vertex_ids](const loft_vec3f& p) -> uint32_t {
		auto key = std::make_tuple(
		    static_cast<int32_t>(std::lround(p.x * 1e5)),
		    static_cast<int32_t>(std::lround(p.y * 1e5)),
		    static_cast<int32_t>(std::lround(p.z * 1e5)));
		auto it = vertex_ids.find(key);
		if (it != vertex_ids.end()) return it->second;
		uint32_t id = static_cast<uint32_t>(vertex_ids.size());
		vertex_ids.emplace(key, id);
		return id;
	};
	for (const auto& tri : tris) {
		uint32_t v[3] = {get_id(std::get<0>(tri)), get_id(std::get<1>(tri)),
		                 get_id(std::get<2>(tri))};
		for (int e = 0; e < 3; ++e) {
			uint32_t lo = std::min(v[e], v[(e + 1) % 3]);
			uint32_t hi = std::max(v[e], v[(e + 1) % 3]);
			edge_count[(static_cast<uint64_t>(lo) << 32) | hi]++;
		}
	}
	for (const auto& [key, count] : edge_count) {
		(void)key;
		if (count != 2) return false;
	}
	return true;
}

sinriv::kigstudio::cgal::MeshData addon_triangles_to_mesh_data(
    const std::vector<loft_Triangle>& triangles) {
	sinriv::kigstudio::cgal::MeshData mesh;
	mesh.reserve(triangles.size());
	for (const auto& tri : triangles) {
		loft_vec3f n = (std::get<1>(tri) - std::get<0>(tri))
		                   .cross(std::get<2>(tri) - std::get<0>(tri));
		float nl = n.length();
		if (nl < 1e-12f) continue;
		mesh.emplace_back(tri, n * (1.0f / nl));
	}
	return mesh;
}

// Build loft mesh triangles from a single hair strand.
// Returns (Triangle, normal) pairs suitable for RenderMesh::loadGeometry().
std::vector<std::tuple<loft_Triangle, loft_vec3f>> build_hair_strand_mesh(
    const HairStrand& strand) {
	using namespace sinriv::kigstudio::mesh::loft;
	std::vector<std::tuple<loft_Triangle, loft_vec3f>> result;

	if (strand.guide_points.size() < 2) return result;
	if (strand.width_points.empty()) return result;
	// Use committed section if applied; otherwise fall back to vertices;
	// if neither is set, use a default unit square.
	static const std::vector<sinriv::kigstudio::vec2<float>> kDefaultSection = {
	    {-0.5f, -0.5f}, {0.5f, -0.5f}, {0.5f, 0.5f}, {-0.5f, 0.5f}};
	const auto& raw_section_path =
	    strand.section_state.committed.size() >= 3
	        ? strand.section_state.committed
	        : (strand.section_state.vertices.size() >= 3
	               ? strand.section_state.vertices
	               : kDefaultSection);

	// Optional Catmull-Rom smoothing for Bézier section mode
	std::vector<sinriv::kigstudio::vec2<float>> global_smoothed_path;
	const std::vector<sinriv::kigstudio::vec2<float>>* section_path_ptr =
	    &raw_section_path;

	if (strand.section_state.use_bezier_section &&
	    raw_section_path.size() >= 3) {
		catmull_rom_smooth_closed(raw_section_path, global_smoothed_path,
		                          std::max(strand.section_subdiv, 1));
		section_path_ptr = &global_smoothed_path;
	}

	const auto& global_section_path = *section_path_ptr;

	// Step 1: Sample guide curve as dense polyline
	auto sampled = sample_bezier_guide_curve(
	    strand.guide_points, std::max(strand.guide_samples_per_segment, 1));
	if (sampled.size() < 2) return result;

	// Convert sampled to loft_vec3f
	std::vector<loft_vec3f> guide_curve;
	guide_curve.reserve(sampled.size());
	for (const auto& p : sampled)
		guide_curve.push_back(loft_vec3f{p.x, p.y, p.z});

	// Step 2: Sort width_points by curve_id for interpolation
	auto sorted_wp = strand.width_points;
	std::sort(sorted_wp.begin(), sorted_wp.end(),
	          [](const HairStrand::WidthPoint& a,
	             const HairStrand::WidthPoint& b) {
		          return a.curve_id < b.curve_id;
	          });

	// Ensure consistent direction orientation along the guide curve.
	// If a width point's direction is >90° from the previous one (dot < 0),
	// flip it to prevent the loft mesh from twisting/knotting.
	for (size_t wi = 1; wi < sorted_wp.size(); ++wi) {
		const auto& prev_dir = sorted_wp[wi - 1].direction;
		auto& cur_dir = sorted_wp[wi].direction;
		float dot = prev_dir.x * cur_dir.x + prev_dir.y * cur_dir.y +
		            prev_dir.z * cur_dir.z;
		if (dot < 0.0f) {
			cur_dir.x = -cur_dir.x;
			cur_dir.y = -cur_dir.y;
			cur_dir.z = -cur_dir.z;
		}
	}

	const int M = static_cast<int>(guide_curve.size());
	const int N = static_cast<int>(strand.guide_points.size());

	// Pre-compute width interpolation for every sample point
	std::vector<float> scales(M);
	std::vector<loft_vec3f> directions(M);

	for (int i = 0; i < M; ++i) {
		// Map sample index to curve_id in guide_point space
		float curve_id =
		    static_cast<float>(i) / static_cast<float>(M - 1) *
		    static_cast<float>(N - 1);

		// Find bracketing width_points
		const HairStrand::WidthPoint* wp_a = nullptr;
		const HairStrand::WidthPoint* wp_b = nullptr;

		for (const auto& wp : sorted_wp) {
			if (wp.curve_id <= curve_id + 1e-6f)
				wp_a = &wp;
			if (wp.curve_id >= curve_id - 1e-6f && !wp_b)
				wp_b = &wp;
		}

		if (wp_a && wp_b) {
			if (wp_a == wp_b) {
				scales[i] = wp_a->scale;
				directions[i] =
				    loft_vec3f{wp_a->direction.x, wp_a->direction.y,
				               wp_a->direction.z};
			} else {
				// Catmull-Rom spline interpolation for smooth width transitions
				auto catmull_rom = [](float p0, float p1, float p2, float p3,
				                      float t) -> float {
					float t2 = t * t;
					float t3 = t2 * t;
					return 0.5f *
					       ((2.0f * p1) + (-p0 + p2) * t +
					        (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
					        (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
				};

				float t =
				    (curve_id - wp_a->curve_id) /
				    (wp_b->curve_id - wp_a->curve_id + 1e-10f);
				t = std::clamp(t, 0.0f, 1.0f);

				int idx_a = static_cast<int>(wp_a - sorted_wp.data());
				int idx_b = idx_a + 1;
				int wp_count = static_cast<int>(sorted_wp.size());

				// Gather 4 control points (P0, P1, P2, P3) for Catmull-Rom.
				// P1 = wp_a, P2 = wp_b. P0/P3 are neighbors or mirrored.
				auto mirror_before = [](float ref, float next) {
					return ref - (next - ref);
				};
				auto mirror_after = [](float ref, float prev) {
					return ref + (ref - prev);
				};

				// --- Scale ---
				float p0_s = (idx_a > 0)
				                 ? sorted_wp[idx_a - 1].scale
				                 : mirror_before(wp_a->scale, wp_b->scale);
				float p1_s = wp_a->scale;
				float p2_s = wp_b->scale;
				float p3_s = (idx_b < wp_count - 1)
				                 ? sorted_wp[idx_b + 1].scale
				                 : mirror_after(wp_b->scale, wp_a->scale);
				scales[i] = catmull_rom(p0_s, p1_s, p2_s, p3_s, t);

				// --- Direction (Catmull-Rom per component, then normalize) ---
				auto mirror_vec_before =
				    [](const loft_vec3f& ref, const loft_vec3f& next) {
					    return loft_vec3f{
					        ref.x - (next.x - ref.x),
					        ref.y - (next.y - ref.y),
					        ref.z - (next.z - ref.z),
					    };
				    };
				auto mirror_vec_after =
				    [](const loft_vec3f& ref, const loft_vec3f& prev) {
					    return loft_vec3f{
					        ref.x + (ref.x - prev.x),
					        ref.y + (ref.y - prev.y),
					        ref.z + (ref.z - prev.z),
					    };
				    };

				loft_vec3f p1_d{wp_a->direction.x, wp_a->direction.y,
				                wp_a->direction.z};
				loft_vec3f p2_d{wp_b->direction.x, wp_b->direction.y,
				                wp_b->direction.z};

				loft_vec3f p0_d =
				    (idx_a > 0)
				        ? loft_vec3f{sorted_wp[idx_a - 1].direction.x,
				                     sorted_wp[idx_a - 1].direction.y,
				                     sorted_wp[idx_a - 1].direction.z}
				        : mirror_vec_before(p1_d, p2_d);

				loft_vec3f p3_d =
				    (idx_b < wp_count - 1)
				        ? loft_vec3f{sorted_wp[idx_b + 1].direction.x,
				                     sorted_wp[idx_b + 1].direction.y,
				                     sorted_wp[idx_b + 1].direction.z}
				        : mirror_vec_after(p2_d, p1_d);

				float dirx =
				    catmull_rom(p0_d.x, p1_d.x, p2_d.x, p3_d.x, t);
				float diry =
				    catmull_rom(p0_d.y, p1_d.y, p2_d.y, p3_d.y, t);
				float dirz =
				    catmull_rom(p0_d.z, p1_d.z, p2_d.z, p3_d.z, t);
				directions[i] = safe_normalize(
				    loft_vec3f{dirx, diry, dirz}, loft_vec3f{0, 1, 0});
			}
		} else if (wp_a) {
			// After last width_point: use tip falloff
			float tip_t = 0.0f;
			if (!sorted_wp.empty()) {
				float last_id = sorted_wp.back().curve_id;
				float range = static_cast<float>(N - 1) - last_id;
				if (range > 1e-4f)
					tip_t = std::clamp(
					    (curve_id - last_id) / range, 0.0f, 1.0f);
			}
			scales[i] = wp_a->scale * (1.0f - tip_t) + kMinTipWidth * tip_t;
			directions[i] =
			    safe_normalize(
			        loft_vec3f{wp_a->direction.x, wp_a->direction.y,
			                   wp_a->direction.z},
			        loft_vec3f{0, 1, 0});
		} else if (wp_b) {
			// Before first width_point: use tip falloff
			float tip_t = 0.0f;
			if (!sorted_wp.empty()) {
				float first_id = sorted_wp.front().curve_id;
				if (first_id > 1e-4f)
					tip_t =
					    std::clamp(1.0f - curve_id / first_id, 0.0f, 1.0f);
			}
			scales[i] = wp_b->scale * (1.0f - tip_t) + kMinTipWidth * tip_t;
			directions[i] =
			    safe_normalize(
			        loft_vec3f{wp_b->direction.x, wp_b->direction.y,
			                   wp_b->direction.z},
			        loft_vec3f{0, 1, 0});
		} else {
			scales[i] = kMinTipWidth;
			directions[i] = loft_vec3f{0, 1, 0};
		}

		// Clamp minimum
		if (scales[i] < kMinTipWidth) scales[i] = kMinTipWidth;
	}

	// Compute section range from width points: cap at first/last width
	// point instead of converging to a tip at guide curve endpoints.
	int first_section_idx = 0;
	int last_section_idx = M - 1;
	if (!sorted_wp.empty()) {
		auto curve_id_to_sample = [&](float curve_id) -> int {
			int idx = static_cast<int>(std::round(
			    curve_id * static_cast<float>(M - 1) /
			    static_cast<float>(N - 1)));
			return std::clamp(idx, 0, M - 1);
		};
		first_section_idx =
		    curve_id_to_sample(sorted_wp.front().curve_id);
		last_section_idx =
		    curve_id_to_sample(sorted_wp.back().curve_id);
		if (first_section_idx > last_section_idx)
			std::swap(first_section_idx, last_section_idx);
		// Ensure at least 2 sections for a valid loft mesh
		if (last_section_idx - first_section_idx < 1) {
			last_section_idx =
			    std::min(first_section_idx + 1, M - 1);
			first_section_idx =
			    std::max(last_section_idx - 1, 0);
		}
	}

	// Step 3: Build LoftSections
	std::vector<LoftSection> sections;
	const float rot_rad = strand.section_rotation * 3.14159265358979f / 180.0f;
	const float cos_r = std::cos(rot_rad);
	const float sin_r = std::sin(rot_rad);

	for (int i = first_section_idx; i <= last_section_idx; ++i) {
		float scale = scales[i];

		// Compute local frame
		loft_vec3f tangent = tangent_at_sample(guide_curve, i);

		// axis_v = direction projected perpendicular to tangent
		float dot_vt = directions[i].x * tangent.x +
		               directions[i].y * tangent.y +
		               directions[i].z * tangent.z;
		loft_vec3f axis_v_raw{directions[i].x - tangent.x * dot_vt,
		                      directions[i].y - tangent.y * dot_vt,
		                      directions[i].z - tangent.z * dot_vt};
		loft_vec3f axis_v = safe_normalize(axis_v_raw, loft_vec3f{0, 1, 0});

		// axis_u = cross(tangent, axis_v)
		loft_vec3f axis_u{
		    tangent.y * axis_v.z - tangent.z * axis_v.y,
		    tangent.z * axis_v.x - tangent.x * axis_v.z,
		    tangent.x * axis_v.y - tangent.y * axis_v.x,
		};
		axis_u = safe_normalize(axis_u, loft_vec3f{1, 0, 0});

		// Re-orthogonalize axis_v = cross(axis_u, tangent)
		axis_v = loft_vec3f{
		    axis_u.y * tangent.z - axis_u.z * tangent.y,
		    axis_u.z * tangent.x - axis_u.x * tangent.z,
		    axis_u.x * tangent.y - axis_u.y * tangent.x,
		};
		axis_v = safe_normalize(axis_v, loft_vec3f{0, 1, 0});

		// Determine per-sample section path: use per-point override
		// if the nearest width point has a custom section, otherwise
		// fall back to the global section.
		const auto* per_sample_path = &global_section_path;
		std::vector<sinriv::kigstudio::vec2<float>> temp_smoothed;

		float curve_id =
		    static_cast<float>(i) / static_cast<float>(M - 1) *
		    static_cast<float>(N - 1);
		const HairStrand::WidthPoint* nearest_wp = nullptr;
		float nearest_dist = 1e10f;
		for (const auto& wp : sorted_wp) {
			float d = std::abs(wp.curve_id - curve_id);
			if (d < nearest_dist) {
				nearest_dist = d;
				nearest_wp = &wp;
			}
		}

		if (nearest_wp &&
		    nearest_wp->section_state.vertices.size() >= 3) {
			if (nearest_wp->section_state.use_bezier_section) {
				catmull_rom_smooth_closed(
				    nearest_wp->section_state.vertices,
				    temp_smoothed,
				    std::max(strand.section_subdiv, 1));
				per_sample_path = &temp_smoothed;
			} else {
				per_sample_path =
				    &nearest_wp->section_state.vertices;
			}
		}

		// Build rotated + scaled path
		std::vector<sinriv::kigstudio::vec2<float>> path;
		path.reserve(per_sample_path->size());
		for (const auto& v : *per_sample_path) {
			float rx = v.x * cos_r - v.y * sin_r;
			float ry = v.x * sin_r + v.y * cos_r;
			path.push_back({rx * scale, ry * scale});
		}

		LoftSection sec;
		sec.guide_vertex_id = static_cast<size_t>(i);
		sec.axis_u = axis_u;
		sec.axis_v = axis_v;
		sec.path = std::move(path);
		sections.push_back(std::move(sec));
	}

	if (sections.size() < 2) return result;

	// Step 4: Build loft mesh
	LoftOptions opts;
	opts.cap_first = true;
	opts.cap_last = true;
	auto loft_tris = build_loft_mesh(guide_curve, sections, opts);

	// Step 5: Wrap triangles with normals
	result.reserve(loft_tris.size());
	for (const auto& tri : loft_tris) {
		result.emplace_back(tri, compute_triangle_normal(tri));
	}

	return result;
}

}  // namespace

bool compute_auto_section_rotation(const vec3f& point, const vec3f& tangent,
                                   const vec3f& center,
                                   float& out_angle_deg) {
    const vec3f& P = point;
    const vec3f& T = tangent;
    const vec3f& C = center;

    vec3f V = {C.x - P.x, C.y - P.y, C.z - P.z};
    float v_dot_t = V.x * T.x + V.y * T.y + V.z * T.z;
    vec3f V_perp = {V.x - T.x * v_dot_t, V.y - T.y * v_dot_t,
                    V.z - T.z * v_dot_t};

    vec3f D = {0.0f, -1.0f, 0.0f};
    float d_dot_t = D.x * T.x + D.y * T.y + D.z * T.z;
    vec3f D_perp = {D.x - T.x * d_dot_t, D.y - T.y * d_dot_t,
                    D.z - T.z * d_dot_t};

    float v_perp_len =
        std::sqrt(V_perp.x * V_perp.x + V_perp.y * V_perp.y +
                  V_perp.z * V_perp.z);
    float d_perp_len =
        std::sqrt(D_perp.x * D_perp.x + D_perp.y * D_perp.y +
                  D_perp.z * D_perp.z);

    const float kEps = 0.0001f;
    if (v_perp_len <= kEps || d_perp_len <= kEps)
        return false;

    // Normalize
    V_perp.x /= v_perp_len;
    V_perp.y /= v_perp_len;
    V_perp.z /= v_perp_len;
    D_perp.x /= d_perp_len;
    D_perp.y /= d_perp_len;
    D_perp.z /= d_perp_len;

    // Cross product of V_perp and D_perp (angle from D_perp to V_perp)
    vec3f cross_vd = {V_perp.y * D_perp.z - V_perp.z * D_perp.y,
                      V_perp.z * D_perp.x - V_perp.x * D_perp.z,
                      V_perp.x * D_perp.y - V_perp.y * D_perp.x};
    float dot_val =
        D_perp.x * V_perp.x + D_perp.y * V_perp.y + D_perp.z * V_perp.z;
    float sign =
        cross_vd.x * T.x + cross_vd.y * T.y + cross_vd.z * T.z;

    float angle_rad = std::atan2(sign, dot_val);
    float angle_deg = angle_rad * 180.0f / 3.14159265358979f;

    // Clamp to [-180, 180]
    if (angle_deg > 180.0f) angle_deg -= 360.0f;
    if (angle_deg < -180.0f) angle_deg += 360.0f;

    out_angle_deg = angle_deg;
    return true;
}

void RenderVoxelList::RenderVoxelItem::add_width_point_at(
    int strand_idx, const vec3f& world_pos) {
    if (strand_idx < 0 ||
        strand_idx >= static_cast<int>(hair_strands.size()))
        return;
    auto& strand = hair_strands[strand_idx];
    if (strand.guide_points.size() < 2)
        return;

    auto nearest = find_nearest_on_bezier_guide(
        strand.guide_points, world_pos, strand.guide_samples_per_segment);

    vec3f diff = world_pos - nearest.curve_pos;
    float dist = diff.length();
    if (dist < 0.0001f)
        return;  // 太靠近曲线，跳过

    HairStrand::WidthPoint wp;
    wp.curve_id = nearest.curve_id;
    wp.direction = diff / dist;  // 单位方向向量
    wp.scale = dist;
    strand.width_points.push_back(wp);

    // 添加第一个宽度向量时，若中心点存在，自动计算截面旋转角度
    if (strand.width_points.size() == 1 && show_addon_center) {
        auto sample = sample_guide_curve_at(strand_idx, wp.curve_id);
        float angle_deg = 0.0f;
        if (compute_auto_section_rotation(sample.position, sample.tangent,
                                          addon_center_point, angle_deg)) {
            strand.section_rotation = angle_deg;
        }
    }
}

GuideCurveSample
RenderVoxelList::RenderVoxelItem::sample_guide_curve_at(
    int strand_idx, float curve_id) const {
    GuideCurveSample result;
    if (strand_idx < 0 ||
        strand_idx >= static_cast<int>(hair_strands.size()))
        return result;
    const auto& strand = hair_strands[strand_idx];
    if (strand.guide_points.size() < 2)
        return result;

    const int subdiv = std::max(strand.guide_samples_per_segment, 1);
    auto sampled = sample_bezier_guide_curve(strand.guide_points, subdiv);
    if (sampled.size() < 2)
        return result;

    // Convert curve_id to sample index in the sampled curve
    const int N = static_cast<int>(strand.guide_points.size());
    const int kSubdiv = subdiv;
    int seg_idx = static_cast<int>(curve_id);
    float t = curve_id - static_cast<float>(seg_idx);
    if (seg_idx < 0) { seg_idx = 0; t = 0.0f; }
    if (seg_idx > N - 2) { seg_idx = N - 2; t = 1.0f; }

    int sample_idx =
        seg_idx * kSubdiv +
        static_cast<int>(std::round(t * static_cast<float>(kSubdiv)));
    if (sample_idx < 0) sample_idx = 0;
    if (sample_idx >= static_cast<int>(sampled.size()))
        sample_idx = static_cast<int>(sampled.size()) - 1;

    result.position = sampled[sample_idx];

    // Convert sampled points to loft_vec3f for tangent_at_sample
    std::vector<loft_vec3f> loft_sampled;
    loft_sampled.reserve(sampled.size());
    for (const auto& p : sampled)
        loft_sampled.push_back(loft_vec3f{p.x, p.y, p.z});
    auto tang = tangent_at_sample(loft_sampled, sample_idx);
    result.tangent = {tang.x, tang.y, tang.z};

    return result;
}

void RenderVoxelList::RenderVoxelItem::update_addon_meshes() {
	addon_renderers.clear();
	for (auto& strand : hair_strands) {
		try {
			auto tris = build_hair_strand_mesh(strand);
			if (tris.empty()) continue;

			// 提交显示前检测网格是否需要修复（不封闭 或 自相交），
			// 需要时用 alpha_wrap 修复（参数为每根发束独立调节）
			std::vector<loft_Triangle> plain;
			plain.reserve(tris.size());
			for (const auto& [tri, n] : tris) {
				(void)n;
				plain.push_back(tri);
			}
			if (!is_mesh_watertight(plain) ||
			    !sinriv::kigstudio::cgal::is_boolean_ready(
			        addon_triangles_to_mesh_data(plain))) {
				// 持久 worker 进程（共享内存 IPC），按需启动，常驻复用。
				// 首帧首次调用时 spawn worker，后续帧零启动开销。
				static sinriv::kigstudio::cgal::RepairWorkerIPC
				    g_repair_worker;

				auto mesh_in = addon_triangles_to_mesh_data(plain);
				double alpha = static_cast<double>(strand.repair_alpha);
				double offset = static_cast<double>(strand.repair_offset);

				bool submitted = g_repair_worker.submit(
				    mesh_in, alpha, offset);
				if (submitted) {
					// 当前无任务运行，已提交到 worker。
					// 等待最多 1 秒取回结果。
					bool was_failed = strand.repair_failed;
					auto wrapped =
					    g_repair_worker.wait_result(1000);
					if (!wrapped.empty()) {
						strand.repair_failed = false;
						plain.clear();
						plain.reserve(wrapped.size());
						for (const auto& [tri, n] : wrapped) {
							(void)n;
							plain.push_back(tri);
						}
					} else {
						// wait_result 返回空 = 超时或失败。
						// 超时时 worker 已在内部被 kill + 重启。
						strand.repair_failed = true;
						if (!was_failed && manager) {
							manager->show_toast(
							    get_locale_string(
							        "toast.repair_timeout") +
							        " \"" + strand.name + "\"",
							    3000.0f);
						}
						std::cerr
						    << "[addon_mesh] alpha_wrap failed"
						    << " or timed out for strand \""
						    << strand.name
						    << "\", rendering original mesh.\n";
					}
				} else {
					// Worker 正忙（上一任务尚未完成）。
					// 当前发束参数已覆盖存入 pending slot，
					// wait_result 完成后会自动提交。
					// 本帧先用原始网格渲染。
				}
			} else {
				strand.repair_failed = false;
			}

			std::vector<std::tuple<loft_Triangle, loft_vec3f>> out;
			out.reserve(plain.size());
			for (const auto& tri : plain) {
				out.emplace_back(tri, compute_triangle_normal(tri));
			}
			addon_renderers.emplace_back();
			addon_renderers.back().setBaseColor(0.3f, 0.65f, 0.42f, 1.0f);
			addon_renderers.back().loadGeometry(out);
		} catch (const std::exception& e) {
			std::cerr << "[addon_mesh] build failed for strand: " << e.what()
			          << std::endl;
		}
}
}

std::vector<sinriv::kigstudio::voxel::triangle_bvh<float>::triangle>
RenderVoxelList::RenderVoxelItem::build_strand_loft_triangles(
    int strand_idx) const {
	std::vector<sinriv::kigstudio::voxel::triangle_bvh<float>::triangle> result;
	if (strand_idx < 0 ||
	    strand_idx >= static_cast<int>(hair_strands.size()))
		return result;

	const auto& strand = hair_strands[strand_idx];
	auto tris_with_normals = build_hair_strand_mesh(strand);
	if (tris_with_normals.empty()) return result;

	// 提取纯三角形用于水密性和自相交检测
	std::vector<loft_Triangle> plain;
	plain.reserve(tris_with_normals.size());
	for (const auto& [tri, n] : tris_with_normals) {
		(void)n;
		plain.push_back(tri);
	}

	// 与 update_addon_meshes() 保持一致的修复逻辑：
	// 检测网格是否需要修复（不封闭 或 自相交），
	// 需要时用 alpha_wrap 修复（参数为每根发束独立调节）
	if (!is_mesh_watertight(plain) ||
	    !sinriv::kigstudio::cgal::is_boolean_ready(
	        addon_triangles_to_mesh_data(plain))) {
		auto mesh_in = addon_triangles_to_mesh_data(plain);
		auto wrapped = sinriv::kigstudio::cgal::alpha_wrap(
		    mesh_in,
		    static_cast<double>(strand.repair_alpha),
		    static_cast<double>(strand.repair_offset));
		if (!wrapped.empty()) {
			result.reserve(wrapped.size());
			for (const auto& [tri, n] : wrapped) {
				(void)n;
				result.push_back(tri);
			}
			return result;
		}
		std::cerr << "[addon_split] alpha_wrap failed for strand \""
		          << strand.name << "\", using original mesh.\n";
	}

	// 无需修复或修复失败：使用原始网格
	result.reserve(tris_with_normals.size());
	for (const auto& [tri, n] : tris_with_normals) {
		(void)n;
		result.push_back(tri);
	}
	return result;
}

std::shared_ptr<sinriv::kigstudio::sdf::SDFBase>
RenderVoxelList::RenderVoxelItem::build_hair_sdf() const {
	using namespace sinriv::kigstudio::sdf;

	std::vector<SDFBasePtr> strand_sdfs;
	for (size_t i = 0; i < hair_strands.size(); ++i) {
		auto tris = build_strand_loft_triangles(static_cast<int>(i));
		if (tris.empty()) continue;

		auto mesh_sdf = std::make_shared<SDF_Mesh>();
		mesh_sdf->precision_mode = SDFPrecision::Precise;
		if (!mesh_sdf->loadTriangles(tris)) continue;
		strand_sdfs.push_back(mesh_sdf);
	}

	if (strand_sdfs.empty()) return nullptr;
	if (strand_sdfs.size() == 1) return strand_sdfs[0];
	return sdf_group(strand_sdfs);
}

std::pair<sinriv::kigstudio::voxel::vec3f, sinriv::kigstudio::voxel::vec3f>
RenderVoxelList::RenderVoxelItem::compute_hair_bounds() const {
	using vec3f = sinriv::kigstudio::voxel::vec3f;
	bool first = true;
	vec3f bmin{0, 0, 0}, bmax{0, 0, 0};

	for (size_t si = 0; si < hair_strands.size(); ++si) {
		auto tris = build_strand_loft_triangles(static_cast<int>(si));
		for (const auto& tri : tris) {
			for (const auto& v :
			     {std::get<0>(tri), std::get<1>(tri), std::get<2>(tri)}) {
				if (first) {
					bmin = vec3f{v.x, v.y, v.z};
					bmax = vec3f{v.x, v.y, v.z};
					first = false;
				} else {
					if (v.x < bmin.x) bmin.x = v.x;
					if (v.y < bmin.y) bmin.y = v.y;
					if (v.z < bmin.z) bmin.z = v.z;
					if (v.x > bmax.x) bmax.x = v.x;
					if (v.y > bmax.y) bmax.y = v.y;
					if (v.z > bmax.z) bmax.z = v.z;
				}
			}
		}
	}

	return {bmin, bmax};
}

void RenderVoxelList::RenderVoxelItem::render_gbuffer(
    const float* transform,
    sinriv::ui::render::RenderMeshShader& mesh_shader) {
    mesh_renderer.cull_backface = false;
    exported_mesh_renderer.cull_backface = false;
    if (showMesh) {
        mesh_renderer.renderGBuffer(transform, mesh_shader);
    }
    if (showOriginMesh && !origin_mesh_renderer.empty()) {
        origin_mesh_renderer.cull_backface = false;
        origin_mesh_renderer.renderGBuffer(transform, mesh_shader);
    }
    if (showExportedMesh && !cached_mesh.empty()) {
        if (!cached_mesh_dirty) {
            exported_mesh_renderer.loadGeometry(cached_mesh);
            cached_mesh_dirty = true;
        }
        exported_mesh_renderer.renderGBuffer(transform, mesh_shader);
    }

    // Rebuild addon meshes if any strand is dirty
    {
        bool any_dirty = false;
        for (const auto& strand : hair_strands) {
            if (strand.mesh_dirty) {
                any_dirty = true;
                break;
            }
        }
        if (any_dirty) {
            update_addon_meshes();
            for (auto& strand : hair_strands)
                strand.mesh_dirty = false;
        }
    }

    // 附加件渲染器（如毛发预览）：写入 albedo/normal/depth，与主模型正确
    // 互相遮挡，但不写 world_pos 通道，鼠标拾取可穿透它拾取下层模型
    if (!manager || manager->showAddonMesh) {
        for (auto& addon : addon_renderers) {
            addon.cull_backface = false;
            addon.renderGBufferAddon(transform, mesh_shader);
        }
    }

    if (showVoxel) {
        voxel_renderer.renderGBuffer(transform, mesh_shader);
    }

    if (showVoxel && !marked_voxels.empty()) {
        if (marked_voxels_dirty) {
            marked_voxels.global_position = voxel_grid_data.global_position;
            marked_voxels.voxel_size = voxel_grid_data.voxel_size;
            marked_mesh_renderer.setBaseColor(1.0f, 0.5f, 0.5f, 1.0f);
            marked_mesh_renderer.setDepthBias(0.07f);
            int num_triangles = 0;
            auto generator = sinriv::kigstudio::voxel::generateMesh(
                marked_voxels, 0.5, num_triangles, true);
            marked_mesh_renderer.loadGeometry(generator);
            marked_voxels_dirty = false;
        }
        if (!marked_mesh_renderer.empty()) {
            marked_mesh_renderer.renderGBuffer(transform, mesh_shader);
        }
    }

    if (segment_mode == SDF_NODE_SPLIT && showCollision &&
        sdf_split_target_id >= 0 && manager) {
        auto target_it = manager->items.find(sdf_split_target_id);
        if (target_it != manager->items.end() &&
            !target_it->second->mesh_renderer.empty()) {
            mat4f split_transform = sdf_split_transform_matrix();
            split_transform.transpose();
            float split_transform_bgfx[16];
            sinriv::kigstudio::mat::toBGFXMat(split_transform,
                                              split_transform_bgfx);
            float combined_transform[16];
            bx::mtxMul(combined_transform, split_transform_bgfx, transform);

            auto old_color = target_it->second->mesh_renderer.getBaseColor();
            target_it->second->mesh_renderer.setBaseColor(1.0f, 0.6f, 0.6f,
                                                          1.0f);
            target_it->second->mesh_renderer.renderGBuffer(
                combined_transform, mesh_shader, true);
            target_it->second->mesh_renderer.setBaseColor(
                old_color[0], old_color[1], old_color[2], old_color[3]);
        }
    }
}

void RenderVoxelList::RenderVoxelItem::render_overlay(
    sinriv::ui::render::RenderCollision& collision_renderer,
    const float* model_transform,
    const float* model_transform_2,
    sinriv::ui::render::RenderCollisionShader& collision_shader,
    sinriv::ui::render::RenderMeshShader& mesh_shader,
    const mat4f* cpu_model_matrix) {
    if (showMesh) {
        mesh_renderer.renderOverlay(mesh_shader);
    }
    if (showVoxel) {
        voxel_renderer.renderOverlay(mesh_shader);
    }
    if (showVoxelChunkBounds && !voxel_grid_data.chunks.empty()) {
        if (mesh_shader.ensureLineProgram()) {
            bgfx::VertexLayout& layout = concave_cone_overlay_layout();
            const uint32_t chunk_color = pack_abgr(0.0f, 1.0f, 1.0f, 1.0f);
            std::vector<mesh_detail::ColorLineVertex> vertices;
            vertices.reserve(voxel_grid_data.chunks.size() * 24);
            for (const auto& [key, chunk] : voxel_grid_data.chunks) {
                (void)chunk;
                int cx, cy, cz;
                sinriv::kigstudio::voxel::unpackChunkKey(key, cx, cy, cz);
                float minx = voxel_grid_data.global_position.x +
                             cx * sinriv::kigstudio::voxel::Chunk::SIZE *
                                 voxel_grid_data.voxel_size.x;
                float miny = voxel_grid_data.global_position.y +
                             cy * sinriv::kigstudio::voxel::Chunk::SIZE *
                                 voxel_grid_data.voxel_size.y;
                float minz = voxel_grid_data.global_position.z +
                             cz * sinriv::kigstudio::voxel::Chunk::SIZE *
                                 voxel_grid_data.voxel_size.z;
                float maxx = minx + sinriv::kigstudio::voxel::Chunk::SIZE *
                                          voxel_grid_data.voxel_size.x;
                float maxy = miny + sinriv::kigstudio::voxel::Chunk::SIZE *
                                          voxel_grid_data.voxel_size.y;
                float maxz = minz + sinriv::kigstudio::voxel::Chunk::SIZE *
                                          voxel_grid_data.voxel_size.z;
                float corners[8][3] = {
                    {minx, miny, minz}, {maxx, miny, minz},
                    {maxx, maxy, minz}, {minx, maxy, minz},
                    {minx, miny, maxz}, {maxx, miny, maxz},
                    {maxx, maxy, maxz}, {minx, maxy, maxz},
                };
                int edges[12][2] = {
                    {0, 1}, {1, 2}, {2, 3}, {3, 0},
                    {4, 5}, {5, 6}, {6, 7}, {7, 4},
                    {0, 4}, {1, 5}, {2, 6}, {3, 7},
                };
                for (auto& e : edges) {
                    vertices.push_back(
                        {corners[e[0]][0], -corners[e[0]][1],
                         corners[e[0]][2], chunk_color});
                    vertices.push_back(
                        {corners[e[1]][0], -corners[e[1]][1],
                         corners[e[1]][2], chunk_color});
                }
            }
            if (!vertices.empty() &&
                bgfx::getAvailTransientVertexBuffer(
                    static_cast<uint32_t>(vertices.size()),
                    layout) >= vertices.size()) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb, static_cast<uint32_t>(vertices.size()),
                    layout);
                std::memcpy(
                    tvb.data, vertices.data(),
                    vertices.size() * sizeof(mesh_detail::ColorLineVertex));
                bgfx::setTransform(model_transform);
                bgfx::setVertexBuffer(0, &tvb);
                bgfx::setState(BGFX_STATE_WRITE_RGB |
                               BGFX_STATE_WRITE_A |
                               BGFX_STATE_PT_LINES |
                               BGFX_STATE_MSAA);
                bgfx::submit(mesh_shader.overlay_view_id_,
                             mesh_shader.line_program_);
            }
        }
    }
    if (showCollision && segment_mode == COLLISION) {
        collision_renderer.render(collision_group, model_transform,
                                  model_transform_2, collision_shader,
                                  cpu_model_matrix);
    }
    if (showCollisionBounds &&
        segment_mode == SDF_NODE_SPLIT) {
        if (sdf_split_target_id >= 0 && manager) {
            auto target_it = manager->items.find(sdf_split_target_id);
            if (target_it != manager->items.end() &&
                !target_it->second->mesh_renderer.empty() &&
                mesh_shader.ensureLineProgram()) {
                auto [min_local, max_local] =
                    target_it->second->mesh_renderer.getLocalBounds();
                bgfx::VertexLayout& layout = concave_cone_overlay_layout();
                const uint32_t bounds_color = pack_abgr(0.0f, 1.0f, 0.0f, 1.0f);
                std::vector<mesh_detail::ColorLineVertex> vertices;
                vertices.reserve(24);
                sinriv::kigstudio::voxel::vec3f corners[8] = {
                    {min_local.x, min_local.y, min_local.z},
                    {max_local.x, min_local.y, min_local.z},
                    {max_local.x, max_local.y, min_local.z},
                    {min_local.x, max_local.y, min_local.z},
                    {min_local.x, min_local.y, max_local.z},
                    {max_local.x, min_local.y, max_local.z},
                    {max_local.x, max_local.y, max_local.z},
                    {min_local.x, max_local.y, max_local.z},
                };
                mat4f split_transform = sdf_split_transform_matrix();
                // split_transform.transpose();
                for (auto& corner : corners) {
                    corner = transform_point(split_transform, corner);
                }
                int edges[12][2] = {
                    {0, 1}, {1, 2}, {2, 3}, {3, 0},
                    {4, 5}, {5, 6}, {6, 7}, {7, 4},
                    {0, 4}, {1, 5}, {2, 6}, {3, 7},
                };
                for (auto& e : edges) {
                    auto& a = corners[e[0]];
                    auto& b = corners[e[1]];
                    vertices.push_back(
                        {a.x, -a.y, a.z, bounds_color});
                    vertices.push_back(
                        {b.x, -b.y, b.z, bounds_color});
                }
                if (!vertices.empty() &&
                    bgfx::getAvailTransientVertexBuffer(
                        static_cast<uint32_t>(vertices.size()),
                        layout) >= vertices.size()) {
                    bgfx::TransientVertexBuffer tvb;
                    bgfx::allocTransientVertexBuffer(
                        &tvb, static_cast<uint32_t>(vertices.size()), layout);
                    std::memcpy(
                        tvb.data, vertices.data(),
                        vertices.size() * sizeof(mesh_detail::ColorLineVertex));
                    bgfx::setTransform(model_transform);
                    bgfx::setVertexBuffer(0, &tvb);
                    bgfx::setState(BGFX_STATE_WRITE_RGB |
                                   BGFX_STATE_WRITE_A |
                                   BGFX_STATE_PT_LINES |
                                   BGFX_STATE_MSAA);
                    bgfx::submit(mesh_shader.overlay_view_id_,
                                 mesh_shader.line_program_);
                }
            }
        }
    }
    if (showCollisionBounds && segment_mode == COLLISION) {
        collision_renderer.renderBounds(collision_group, model_transform,
                                        model_transform_2, collision_shader,
                                        cpu_model_matrix);
    }
    if (showCollision && segment_mode == CONCAVE_CONE) {
        render_concave_cone_overlay(model_transform,
                                    mesh_shader);
    }
    if (segment_mode == CHAIN && !skeleton_lines.empty()) {
        if (mesh_shader.ensureLineProgram()) {
            bgfx::VertexLayout& layout = concave_cone_overlay_layout();
            const uint32_t line_color = pack_abgr(1.0f, 0.84f, 0.08f, 1.0f);
            std::vector<mesh_detail::ColorLineVertex> vertices;
            vertices.reserve(skeleton_lines.size() * 2);
            for (const auto& line : skeleton_lines) {
                const auto& a = line.first;
                const auto& b = line.second;
                vertices.push_back({a.x, -a.y, a.z, line_color});
                vertices.push_back({b.x, -b.y, b.z, line_color});
            }
            if (!vertices.empty() &&
                bgfx::getAvailTransientVertexBuffer(
                    static_cast<uint32_t>(vertices.size()),
                    layout) >= vertices.size()) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb, static_cast<uint32_t>(vertices.size()),
                    layout);
                std::memcpy(tvb.data, vertices.data(),
                            vertices.size() *
                                sizeof(mesh_detail::ColorLineVertex));
                bgfx::setTransform(model_transform);
                bgfx::setVertexBuffer(0, &tvb);
                bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                               BGFX_STATE_WRITE_Z |
                               BGFX_STATE_DEPTH_TEST_LESS |
                               BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
                bgfx::submit(mesh_shader.overlay_view_id_,
                             mesh_shader.line_program_);
            }
        }
    }
    if (segment_mode == CHAIN && !picked_skeleton_points.empty()) {
        if (mesh_shader.ensureLineProgram()) {
            bgfx::VertexLayout& layout = concave_cone_overlay_layout();
            const uint32_t marker_color = pack_abgr(1.0f, 0.18f, 0.08f, 1.0f);
            const float radius =
                std::max({voxel_grid_data.voxel_size.x,
                          voxel_grid_data.voxel_size.y,
                          voxel_grid_data.voxel_size.z}) *
                2.0f;
            std::vector<mesh_detail::ColorLineVertex> vertices;
            vertices.reserve(picked_skeleton_points.size() * 48 * 6);
            for (const auto& picked : picked_skeleton_points) {
                const auto& p = picked.position;
                append_marker_circle(vertices, p, {1.0f, 0.0f, 0.0f},
                                     {0.0f, 1.0f, 0.0f}, radius,
                                     marker_color);
                append_marker_circle(vertices, p, {1.0f, 0.0f, 0.0f},
                                     {0.0f, 0.0f, 1.0f}, radius,
                                     marker_color);
                append_marker_circle(vertices, p, {0.0f, 1.0f, 0.0f},
                                     {0.0f, 0.0f, 1.0f}, radius,
                                     marker_color);
            }
            if (!vertices.empty() &&
                bgfx::getAvailTransientVertexBuffer(
                    static_cast<uint32_t>(vertices.size()),
                    layout) >= vertices.size()) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb, static_cast<uint32_t>(vertices.size()),
                    layout);
                std::memcpy(tvb.data, vertices.data(),
                            vertices.size() *
                                sizeof(mesh_detail::ColorLineVertex));
                bgfx::setTransform(model_transform);
                bgfx::setVertexBuffer(0, &tvb);
                bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                               BGFX_STATE_WRITE_Z |
                               BGFX_STATE_DEPTH_TEST_LESS |
                               BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
                bgfx::submit(mesh_shader.overlay_view_id_,
                             mesh_shader.line_program_);
            }

            // joint wireframes
            if (joint_wireframe_dirty) {
                rebuild_joint_wireframe();
            }
            if (!joint_wireframe_vertices.empty() &&
                bgfx::getAvailTransientVertexBuffer(
                    static_cast<uint32_t>(joint_wireframe_vertices.size()),
                    layout) >= joint_wireframe_vertices.size()) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb,
                    static_cast<uint32_t>(joint_wireframe_vertices.size()),
                    layout);
                std::memcpy(tvb.data, joint_wireframe_vertices.data(),
                            joint_wireframe_vertices.size() *
                                sizeof(mesh_detail::ColorLineVertex));
                bgfx::setTransform(model_transform);
                bgfx::setVertexBuffer(0, &tvb);
                bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                               BGFX_STATE_WRITE_Z |
                               BGFX_STATE_DEPTH_TEST_LESS |
                               BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
                bgfx::submit(mesh_shader.overlay_view_id_,
                             mesh_shader.line_program_);
            }
        }
    }
    // 毛发引导曲线渲染
    if (!hair_strands.empty()) {
        bool has_any_guide_points = false;
        for (const auto& strand : hair_strands) {
            if (strand.guide_points.size() >= 2) {
                has_any_guide_points = true;
                break;
            }
        }
        if (has_any_guide_points && mesh_shader.ensureLineProgram()) {
            bgfx::VertexLayout& layout = concave_cone_overlay_layout();
            const uint32_t active_line_color =
                pack_abgr(1.0f, 0.84f, 0.08f, 1.0f);   // yellow
            const uint32_t active_marker_color =
                pack_abgr(1.0f, 0.6f, 0.1f, 1.0f);     // orange-yellow
            const uint32_t idle_line_color =
                pack_abgr(1.0f, 1.0f, 1.0f, 0.7f);     // white
            const uint32_t idle_marker_color =
                pack_abgr(0.8f, 0.8f, 0.8f, 0.5f);     // grey-white
            std::vector<mesh_detail::ColorLineVertex> vertices;
            for (size_t si = 0; si < hair_strands.size(); ++si) {
                const auto& strand = hair_strands[si];
                if (strand.guide_points.size() < 2)
                    continue;
                // Active strand (drawing guide or editing width) → yellow;
                // idle strands → white
                bool is_active =
                    (guide_curve_drawing_active &&
                     active_guide_draw_strand == static_cast<int>(si)) ||
                    (width_editing_active &&
                     active_width_edit_strand == static_cast<int>(si));
                uint32_t line_color =
                    is_active ? active_line_color : idle_line_color;
                uint32_t marker_color =
                    is_active ? active_marker_color : idle_marker_color;
                // 贝塞尔插值采样 → 平滑曲线折线
                auto sampled = sample_bezier_guide_curve(
                    strand.guide_points,
                    std::max(strand.guide_samples_per_segment, 1));
                for (size_t pi = 0; pi + 1 < sampled.size(); ++pi) {
                    const auto& a = sampled[pi];
                    const auto& b = sampled[pi + 1];
                    vertices.push_back({a.x, -a.y, a.z, line_color});
                    vertices.push_back({b.x, -b.y, b.z, line_color});
                }
                const float marker_size = 1.5f;
                for (const auto& p : strand.guide_points) {
                    vertices.push_back({p.x - marker_size, -p.y, p.z, marker_color});
                    vertices.push_back({p.x + marker_size, -p.y, p.z, marker_color});
                    vertices.push_back({p.x, -(p.y - marker_size), p.z, marker_color});
                    vertices.push_back({p.x, -(p.y + marker_size), p.z, marker_color});
                    vertices.push_back({p.x, -p.y, p.z - marker_size, marker_color});
                    vertices.push_back({p.x, -p.y, p.z + marker_size, marker_color});
                }
            }
            if (!vertices.empty() &&
                bgfx::getAvailTransientVertexBuffer(
                    static_cast<uint32_t>(vertices.size()),
                    layout) >= vertices.size()) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb, static_cast<uint32_t>(vertices.size()),
                    layout);
                std::memcpy(tvb.data, vertices.data(),
                            vertices.size() *
                                sizeof(mesh_detail::ColorLineVertex));
                bgfx::setTransform(model_transform);
                bgfx::setVertexBuffer(0, &tvb);
                bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                               BGFX_STATE_WRITE_Z |
                               BGFX_STATE_DEPTH_TEST_LESS |
                               BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
                bgfx::submit(mesh_shader.overlay_view_id_,
                             mesh_shader.line_program_);
            }
        }
    }

    // 宽度编辑连接线（绿色：点击位置 → 曲线上最近点）
    {
        bool has_width_lines = false;
        for (const auto& strand : hair_strands) {
            if (!strand.width_points.empty()) {
                has_width_lines = true;
                break;
            }
        }
        if (has_width_lines && mesh_shader.ensureLineProgram()) {
            bgfx::VertexLayout& layout = concave_cone_overlay_layout();
            const uint32_t green_color = pack_abgr(0.2f, 0.9f, 0.3f, 1.0f);
            const uint32_t cyan_color = pack_abgr(0.0f, 1.0f, 1.0f, 1.0f);
            std::vector<mesh_detail::ColorLineVertex> vertices;
            int si = 0;
            for (const auto& strand : hair_strands) {
                bool is_active_strand =
                    (si == active_width_edit_strand);
                int wi = 0;
                for (const auto& wp : strand.width_points) {
                    // 越界检查：curve_id 超出有效范围则不显示
                    if (strand.guide_points.size() < 2)
                        continue;
                    if (wp.curve_id < 0.0f)
                        continue;
                    float max_id =
                        static_cast<float>(strand.guide_points.size() - 1);
                    if (wp.curve_id > max_id)
                        continue;

                    // Highlight the hovered row in cyan (0,1,1)
                    uint32_t color = green_color;
                    if (is_active_strand &&
                        wi == hovered_width_point_index) {
                        color = cyan_color;
                    }

                    // 从 curve_id 重建贝塞尔曲线上的点
                    size_t seg_idx =
                        static_cast<size_t>(wp.curve_id);
                    if (seg_idx >= strand.guide_points.size() - 1)
                        seg_idx = strand.guide_points.size() - 2;
                    float t =
                        wp.curve_id - static_cast<float>(seg_idx);

                    const auto& gpts = strand.guide_points;
                    size_t n = gpts.size();
                    vec3f p0 = gpts[seg_idx];
                    vec3f p3 = gpts[seg_idx + 1];
                    vec3f p1, p2;
                    if (seg_idx == 0) {
                        p1 = p0 + (p3 - p0) * (1.0f / 3.0f);
                    } else {
                        p1 = p0 + (p3 - gpts[seg_idx - 1]) *
                                      (1.0f / 6.0f);
                    }
                    if (seg_idx + 2 >= n) {
                        p2 = p3 - (p3 - p0) * (1.0f / 3.0f);
                    } else {
                        p2 = p3 - (gpts[seg_idx + 2] - p0) *
                                      (1.0f / 6.0f);
                    }
                    vec3f curve_pos =
                        bezier_eval(p0, p1, p2, p3, t);
                    vec3f end_pos =
                        curve_pos + wp.direction * wp.scale;
                    vertices.push_back({end_pos.x, -end_pos.y,
                                        end_pos.z, color});
                    vertices.push_back({curve_pos.x, -curve_pos.y,
                                        curve_pos.z, color});
                    ++wi;
                }
                ++si;
            }
            if (!vertices.empty() &&
                bgfx::getAvailTransientVertexBuffer(
                    static_cast<uint32_t>(vertices.size()),
                    layout) >= vertices.size()) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb, static_cast<uint32_t>(vertices.size()),
                    layout);
                std::memcpy(tvb.data, vertices.data(),
                            vertices.size() *
                                sizeof(mesh_detail::ColorLineVertex));
                bgfx::setTransform(model_transform);
                bgfx::setVertexBuffer(0, &tvb);
                bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                               BGFX_STATE_WRITE_Z |
                               BGFX_STATE_DEPTH_TEST_LESS |
                               BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
                bgfx::submit(mesh_shader.overlay_view_id_,
                             mesh_shader.line_program_);
            }
        }
    }

    if (showSilhouetteCenter && mesh_only && segment_mode == SILHOUETTE) {
        if (mesh_shader.ensureLineProgram()) {
            bgfx::VertexLayout& layout = concave_cone_overlay_layout();
            const uint32_t center_color = pack_abgr(1.0f, 0.84f, 0.08f, 1.0f);
            const float radius = 2.0f;
            std::vector<mesh_detail::ColorLineVertex> vertices;
            vertices.reserve(48 * 6 +
                             (inner_wall_radius > 0.0f ? 48 * 6 : 0));
            append_marker_circle(vertices, silhouette_center,
                                 {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
                                 radius, center_color);
            append_marker_circle(vertices, silhouette_center,
                                 {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
                                 radius, center_color);
            append_marker_circle(vertices, silhouette_center,
                                 {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
                                 radius, center_color);
            if (inner_wall_radius > 0.0f) {
                const uint32_t wall_color =
                    pack_abgr(0.0f, 0.95f, 1.0f, 0.72f);
                append_marker_circle(vertices, silhouette_center,
                                     {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
                                     inner_wall_radius, wall_color);
                append_marker_circle(vertices, silhouette_center,
                                     {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
                                     inner_wall_radius, wall_color);
                append_marker_circle(vertices, silhouette_center,
                                     {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
                                     inner_wall_radius, wall_color);
            }
            if (!vertices.empty() &&
                bgfx::getAvailTransientVertexBuffer(
                    static_cast<uint32_t>(vertices.size()),
                    layout) >= vertices.size()) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb, static_cast<uint32_t>(vertices.size()),
                    layout);
                std::memcpy(tvb.data, vertices.data(),
                            vertices.size() *
                                sizeof(mesh_detail::ColorLineVertex));
                bgfx::setTransform(model_transform);
                bgfx::setVertexBuffer(0, &tvb);
                bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                               BGFX_STATE_WRITE_Z |
                               BGFX_STATE_DEPTH_TEST_LESS |
                               BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
                bgfx::submit(mesh_shader.overlay_view_id_,
                             mesh_shader.line_program_);
            }
        }
    }

    // Addon center point rendering (three yellow circles, same style as silhouette)
    if (show_addon_center && source_type == 2) {
        if (mesh_shader.ensureLineProgram()) {
            bgfx::VertexLayout& layout = concave_cone_overlay_layout();
            const uint32_t center_color = pack_abgr(1.0f, 0.84f, 0.08f, 1.0f);
            const float radius = 2.0f;
            std::vector<mesh_detail::ColorLineVertex> vertices;
            vertices.reserve(48 * 3);
            append_marker_circle(vertices, addon_center_point,
                                 {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
                                 radius, center_color);
            append_marker_circle(vertices, addon_center_point,
                                 {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
                                 radius, center_color);
            append_marker_circle(vertices, addon_center_point,
                                 {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
                                 radius, center_color);
            if (!vertices.empty() &&
                bgfx::getAvailTransientVertexBuffer(
                    static_cast<uint32_t>(vertices.size()),
                    layout) >= vertices.size()) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb, static_cast<uint32_t>(vertices.size()),
                    layout);
                std::memcpy(tvb.data, vertices.data(),
                            vertices.size() *
                                sizeof(mesh_detail::ColorLineVertex));
                bgfx::setTransform(model_transform);
                bgfx::setVertexBuffer(0, &tvb);
                bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                               BGFX_STATE_WRITE_Z |
                               BGFX_STATE_DEPTH_TEST_LESS |
                               BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
                bgfx::submit(mesh_shader.overlay_view_id_,
                             mesh_shader.line_program_);
            }
        }
    }
}

void RenderVoxelList::RenderVoxelItem::render_concave_cone_overlay(
    const float* model_transform,
    sinriv::ui::render::RenderMeshShader& mesh_shader) {
    const auto& verts = concave_cone.base_vertices;
    const int vertex_count = static_cast<int>(verts.size());
    if (vertex_count < 2 || !mesh_shader.ensureLineProgram()) {
        return;
    }

    bgfx::VertexLayout& layout = concave_cone_overlay_layout();

    const uint32_t face_color = pack_abgr(0.1f, 0.72f, 1.0f, 0.22f);
    const uint32_t edge_color = pack_abgr(0.0f, 0.95f, 1.0f, 0.72f);
    const uint32_t vertex_loop_color = pack_abgr(1.0f, 1.0f, 1.0f, 0.9f);
    const uint32_t highlight_color = pack_abgr(1.0f, 0.84f, 0.08f, 1.0f);

    std::vector<sinriv::kigstudio::voxel::concave::vec3f> extended_verts;
    extended_verts.reserve(static_cast<size_t>(vertex_count));
    for (const auto& vertex : verts) {
        extended_verts.push_back(
            extend_cone_edge(concave_cone.apex, vertex));
    }

    std::vector<mesh_detail::ColorLineVertex> face_vertices;
    face_vertices.reserve(static_cast<size_t>(vertex_count) * 3 +
                          static_cast<size_t>(vertex_count) * 3);
    for (int i = 0; i < vertex_count; ++i) {
        const auto& a = concave_cone.apex;
        const auto& b = extended_verts[i];
        const auto& c = extended_verts[(i + 1) % vertex_count];
        face_vertices.push_back({a.x, -a.y, a.z, face_color});
        face_vertices.push_back({b.x, -b.y, b.z, face_color});
        face_vertices.push_back({c.x, -c.y, c.z, face_color});
    }

    concave_cone.triangulate();
    for (const auto& tri : concave_cone.base_triangles) {
        const auto& v0 = extended_verts[tri[0]];
        const auto& v1 = extended_verts[tri[1]];
        const auto& v2 = extended_verts[tri[2]];
        face_vertices.push_back({v0.x, -v0.y, v0.z, face_color});
        face_vertices.push_back({v1.x, -v1.y, v1.z, face_color});
        face_vertices.push_back({v2.x, -v2.y, v2.z, face_color});
    }

    if (!face_vertices.empty() &&
        bgfx::getAvailTransientVertexBuffer(
            static_cast<uint32_t>(face_vertices.size()),
            layout) >= face_vertices.size()) {
        bgfx::TransientVertexBuffer tvb;
        bgfx::allocTransientVertexBuffer(
            &tvb, static_cast<uint32_t>(face_vertices.size()),
            layout);
        std::memcpy(tvb.data, face_vertices.data(),
                    face_vertices.size() * sizeof(mesh_detail::ColorLineVertex));

        bgfx::setTransform(model_transform);
        bgfx::setVertexBuffer(0, &tvb);
        bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                       BGFX_STATE_BLEND_ALPHA | BGFX_STATE_MSAA);
        bgfx::submit(mesh_shader.overlay_view_id_, mesh_shader.line_program_);
    }

    std::vector<mesh_detail::ColorLineVertex> line_vertices;
    line_vertices.reserve(static_cast<size_t>(vertex_count) * 6);
    auto append_line = [&](const auto& a, const auto& b, uint32_t color) {
        line_vertices.push_back({a.x, -a.y, a.z, color});
        line_vertices.push_back({b.x, -b.y, b.z, color});
    };
    for (int i = 0; i < vertex_count; ++i) {
        append_line(extended_verts[i],
                    extended_verts[(i + 1) % vertex_count], edge_color);
        append_line(verts[i], verts[(i + 1) % vertex_count],
                    vertex_loop_color);
        const bool expanded = contains_index(concave_cone_expanded_vertices, i);
        append_line(concave_cone.apex, extended_verts[i],
                    expanded ? highlight_color : edge_color);
    }

    if (line_vertices.empty() ||
        bgfx::getAvailTransientVertexBuffer(
            static_cast<uint32_t>(line_vertices.size()),
            layout) < line_vertices.size()) {
        return;
    }

    bgfx::TransientVertexBuffer tvb;
    bgfx::allocTransientVertexBuffer(
        &tvb, static_cast<uint32_t>(line_vertices.size()),
        layout);
    std::memcpy(tvb.data, line_vertices.data(),
                line_vertices.size() * sizeof(mesh_detail::ColorLineVertex));

    bgfx::setTransform(model_transform);
    bgfx::setVertexBuffer(0, &tvb);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                   BGFX_STATE_BLEND_ALPHA | BGFX_STATE_PT_LINES |
                   BGFX_STATE_MSAA);
    bgfx::submit(mesh_shader.overlay_view_id_, mesh_shader.line_program_);
}

void RenderVoxelList::RenderVoxelItem::upload_collision(
    sinriv::ui::render::RenderDeferred& render) {
    if (segment_mode == SDF_NODE_SPLIT) {
        render.clearCollisionTint();
        render.setSpaceDivVisible(false);
        if (sdf_split_target_id >= 0 && manager) {
            auto target_it = manager->items.find(sdf_split_target_id);
            if (target_it != manager->items.end()) {
                const auto& mh = target_it->second->mesh_renderer.getMeshHandle();
                float split_transform_bgfx[16];
                mat4f split_transform = sdf_split_transform_matrix();
                split_transform.transpose();
                sinriv::kigstudio::mat::toBGFXMat(split_transform,
                                                  split_transform_bgfx);
                render.submitMeshStencil(mh.vbh, mh.ibh, mh.index_count,
                                         split_transform_bgfx);
            } else {
                render.submitMeshStencil(
                    BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE, 0);
            }
        } else {
            render.submitMeshStencil(
                BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE, 0);
        }
    } else if (showCollision) {
        render.submitMeshStencil(
            BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE, 0);
        if (segment_mode == COLLISION) {
            render.setCollisionGroup(collision_group);
            render.setSpaceDivVisible(false);
        } else if (segment_mode == PLANE) {
            render.clearCollisionTint();
            render.setSpaceDivVisible(true);
            render.setSpaceDiv(plane.A, plane.B, plane.C, plane.D);
        } else if (segment_mode == CONCAVE_CONE) {
            render.setConcaveCone(concave_cone);
            render.setSpaceDivVisible(false);
        } else {
            // SPLIT_DISCONNECTED / NEIGHBOR: no collision overlay
            render.clearCollisionTint();
            render.setSpaceDivVisible(false);
        }
    } else {
        render.clearCollisionTint();
    }
}

void RenderVoxelList::upload_collision(
    sinriv::ui::render::RenderDeferred& render) {
    {
        std::lock_guard<std::mutex> lock(locker);
        auto it = items.find(render_id);
        if (it != items.end()) {
            it->second->upload_collision(render);
        } else {
            render.clearCollisionTint();
        }
    }
    int num = static_cast<int>(hightlight_pos.size());
    if (num > 16) {
        num = 16;
    }
    render.pos_hightlight_counts = num;
    render.pos_hightlight_counts_gpu_[0] = static_cast<float>(num);
    for (int i = 0; i < num; i++) {
        render.pos_hightlight_[i][0] = std::get<0>(hightlight_pos[i]).x;
        render.pos_hightlight_[i][1] = std::get<0>(hightlight_pos[i]).y;
        render.pos_hightlight_[i][2] = std::get<0>(hightlight_pos[i]).z;
        render.pos_hightlight_[i][3] = std::get<2>(hightlight_pos[i]);

        render.pos_hightlight_color_[i][0] = std::get<1>(hightlight_pos[i]).x;
        render.pos_hightlight_color_[i][1] = std::get<1>(hightlight_pos[i]).y;
        render.pos_hightlight_color_[i][2] = std::get<1>(hightlight_pos[i]).z;
        render.pos_hightlight_color_[i][3] = 1.0f;
    }
    hightlight_pos.clear();
}
}  // namespace sinriv::ui::render
