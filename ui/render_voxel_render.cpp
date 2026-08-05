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

}  // end anonymous namespace

// 评估三次贝塞尔曲线：B(t) = P0*(1-t)³ + P1*3(1-t)²t + P2*3(1-t)t² + P3*t³
vec3f bezier_eval(const vec3f& p0, const vec3f& p1,
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
    int samples_per_segment) {
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

namespace {  // resume anonymous namespace

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

// Resample a closed 2D polygon to a fixed number of evenly-spaced vertices
// along the perimeter. Used to ensure all loft sections have matching
// path sizes when per-point section overrides differ from the global section.
inline void resample_closed_polygon(
    const std::vector<sinriv::kigstudio::vec2<float>>& input,
    std::vector<sinriv::kigstudio::vec2<float>>& output,
    int target_count) {
    const int n = static_cast<int>(input.size());
    if (n < 2 || target_count < 3) { output = input; return; }

    // Compute cumulative arc lengths along the perimeter
    std::vector<float> arc_len(n + 1);
    arc_len[0] = 0.0f;
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        float dx = input[j].x - input[i].x;
        float dy = input[j].y - input[i].y;
        arc_len[i + 1] = arc_len[i] + std::sqrt(dx * dx + dy * dy);
    }
    float total_len = arc_len[n];
    if (total_len < 1e-10f) { output = input; return; }

    output.clear();
    output.reserve(target_count);
    for (int k = 0; k < target_count; ++k) {
        float t = static_cast<float>(k) / static_cast<float>(target_count);
        float target_dist = t * total_len;

        // Find which segment contains this distance
        int seg = 0;
        while (seg < n && arc_len[seg + 1] < target_dist) ++seg;
        if (seg >= n) seg = n - 1;

        float seg_start = arc_len[seg];
        float seg_len = arc_len[seg + 1] - seg_start;
        float local_t = (seg_len > 1e-10f) ?
            (target_dist - seg_start) / seg_len : 0.0f;
        local_t = std::clamp(local_t, 0.0f, 1.0f);

        int next = (seg + 1) % n;
        float x = input[seg].x + local_t * (input[next].x - input[seg].x);
        float y = input[seg].y + local_t * (input[next].y - input[seg].y);
        output.push_back({x, y});
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

// ---- Special strand primitive builders (CGAL union approach) ----

// Generate 2D circular path with `segments` vertices on a circle of given radius
inline std::vector<sinriv::kigstudio::vec2<float>> build_circular_path(
    float radius, int segments) {
    std::vector<sinriv::kigstudio::vec2<float>> path;
    path.reserve(segments);
    for (int i = 0; i < segments; ++i) {
        float angle = 2.0f * 3.14159265358979f * static_cast<float>(i) /
                      static_cast<float>(segments);
        path.push_back({radius * std::cos(angle), radius * std::sin(angle)});
    }
    return path;
}

// Compute local frame (tangent, axis_u, axis_v) at sample index `i`
// along a sampled guide curve. axis_v is initially the world Y axis projected
// perpendicular to tangent; axis_u = cross(tangent, axis_v).
inline void local_frame_at_sample(
    const std::vector<loft_vec3f>& guide_curve, int i,
    loft_vec3f& out_tangent, loft_vec3f& out_axis_u, loft_vec3f& out_axis_v) {
    out_tangent = tangent_at_sample(guide_curve, i);
    // project world Y onto tangent plane
    float dot_yt = out_tangent.y;  // world Y = (0,1,0)
    loft_vec3f v_raw{ -out_tangent.x * dot_yt, 1.0f - out_tangent.y * dot_yt,
                      -out_tangent.z * dot_yt };
    out_axis_v = safe_normalize(v_raw, loft_vec3f{0, 1, 0});
    // axis_u = cross(tangent, axis_v)
    out_axis_u = loft_vec3f{
        out_tangent.y * out_axis_v.z - out_tangent.z * out_axis_v.y,
        out_tangent.z * out_axis_v.x - out_tangent.x * out_axis_v.z,
        out_tangent.x * out_axis_v.y - out_tangent.y * out_axis_v.x,
    };
    out_axis_u = safe_normalize(out_axis_u, loft_vec3f{1, 0, 0});
    // re-orthogonalize axis_v
    out_axis_v = loft_vec3f{
        out_axis_u.y * out_tangent.z - out_axis_u.z * out_tangent.y,
        out_axis_u.z * out_tangent.x - out_axis_u.x * out_tangent.z,
        out_axis_u.x * out_tangent.y - out_axis_u.y * out_tangent.x,
    };
    out_axis_v = safe_normalize(out_axis_v, loft_vec3f{0, 1, 0});
}

// Map 2D point to 3D using local frame: origin + u*axis_u + v*axis_v
inline loft_vec3f map_to_3d(const sinriv::kigstudio::vec2<float>& p,
                            const loft_vec3f& origin,
                            const loft_vec3f& axis_u,
                            const loft_vec3f& axis_v) {
    return loft_vec3f{
        origin.x + p.x * axis_u.x + p.y * axis_v.x,
        origin.y + p.x * axis_u.y + p.y * axis_v.y,
        origin.z + p.x * axis_u.z + p.y * axis_v.z,
    };
}

// Build a watertight cylinder along a sampled curve with circular cross-section
using MeshData = sinriv::kigstudio::cgal::MeshData;
using cgal_Triangle = sinriv::kigstudio::cgal::Triangle;
using cgal_vec3f = sinriv::kigstudio::cgal::vec3f;

MeshData build_cylinder_mesh(
    const std::vector<loft_vec3f>& sampled_curve,
    float radius, int circle_segments = 16) {
    MeshData result;
    if (sampled_curve.size() < 2 || radius < 1e-6f) return result;

    const int M = static_cast<int>(sampled_curve.size());
    const int C = circle_segments;

    // Pre-compute 3D ring vertices for each sample point
    auto circle_2d = build_circular_path(radius, C);
    std::vector<std::vector<loft_vec3f>> rings(M);
    for (int i = 0; i < M; ++i) {
        loft_vec3f tangent, axis_u, axis_v;
        local_frame_at_sample(sampled_curve, i, tangent, axis_u, axis_v);
        rings[i].reserve(C);
        for (const auto& p : circle_2d) {
            rings[i].push_back(map_to_3d(p, sampled_curve[i], axis_u, axis_v));
        }
    }

    // Tube surface: connect adjacent rings with quads (2 triangles each)
    for (int i = 0; i < M - 1; ++i) {
        for (int j = 0; j < C; ++j) {
            int j2 = (j + 1) % C;
            const auto& a = rings[i][j];
            const auto& b = rings[i][j2];
            const auto& c = rings[i + 1][j];
            const auto& d = rings[i + 1][j2];
            cgal_Triangle t1{a, b, c};
            cgal_Triangle t2{c, b, d};
            loft_vec3f n1 = (b - a).cross(c - a); n1 = n1 * (1.0f / (n1.length() + 1e-12f));
            loft_vec3f n2 = (b - d).cross(c - d); n2 = n2 * (1.0f / (n2.length() + 1e-12f));
            result.emplace_back(t1, cgal_vec3f{n1.x, n1.y, n1.z});
            result.emplace_back(t2, cgal_vec3f{n2.x, n2.y, n2.z});
        }
    }

    // Cap first end (triangle fan)
    {
        const auto& center = sampled_curve.front();
        loft_vec3f tangent, axis_u, axis_v;
        local_frame_at_sample(sampled_curve, 0, tangent, axis_u, axis_v);
        loft_vec3f normal = tangent * -1.0f;  // outward normal points backward
        for (int j = 0; j < C; ++j) {
            int j2 = (j + 1) % C;
            cgal_Triangle tri{center, rings[0][j2], rings[0][j]};
            result.emplace_back(tri, cgal_vec3f{normal.x, normal.y, normal.z});
        }
    }
    // Cap last end
    {
        const auto& center = sampled_curve.back();
        loft_vec3f tangent, axis_u, axis_v;
        local_frame_at_sample(sampled_curve, M - 1, tangent, axis_u, axis_v);
        for (int j = 0; j < C; ++j) {
            int j2 = (j + 1) % C;
            cgal_Triangle tri{center, rings[M - 1][j], rings[M - 1][j2]};
            result.emplace_back(tri, cgal_vec3f{tangent.x, tangent.y, tangent.z});
        }
    }

    return result;
}

// Build a watertight ellipsoid mesh at `center`, with long axis `radius_b`
// aligned to `tangent` and short axes `radius_a` perpendicular to it.
MeshData build_ellipsoid_mesh(
    const loft_vec3f& center,
    const loft_vec3f& tangent,
    const loft_vec3f& axis_u,
    const loft_vec3f& axis_v,
    float radius_a, float radius_b,
    int lat_segments = 8, int long_segments = 16) {
    MeshData result;
    const int L = std::max(lat_segments, 3);
    const int S = std::max(long_segments, 4);

    // Generate unit sphere vertices via lat/long, scaled by (ra, ra, rb)
    // Latitude: 0=bottom pole (-Y in local), L=top pole (+Y in local)
    // Longitude: around the Y axis
    std::vector<std::vector<loft_vec3f>> rings(L + 1);
    for (int i = 0; i <= L; ++i) {
        float phi = -3.14159265358979f / 2.0f +
                    static_cast<float>(i) * 3.14159265358979f /
                        static_cast<float>(L);
        float cos_phi = std::cos(phi);
        float sin_phi = std::sin(phi);
        // In local space: Y corresponds to tangent direction
        // X -> axis_u, Z -> axis_v, Y -> tangent
        float local_y = sin_phi * radius_b;  // scaled by long radius
        float r = cos_phi * radius_a;         // scaled by short radius
        rings[i].reserve(S);
        for (int j = 0; j < S; ++j) {
            float theta = static_cast<float>(j) * 2.0f * 3.14159265358979f /
                          static_cast<float>(S);
            float local_x = r * std::cos(theta);
            float local_z = r * std::sin(theta);
            // Map to world: center + local_x*axis_u + local_y*tangent + local_z*axis_v
            rings[i].push_back(loft_vec3f{
                center.x + local_x * axis_u.x + local_y * tangent.x +
                    local_z * axis_v.x,
                center.y + local_x * axis_u.y + local_y * tangent.y +
                    local_z * axis_v.y,
                center.z + local_x * axis_u.z + local_y * tangent.z +
                    local_z * axis_v.z,
            });
        }
    }

    // Tube surface between latitude rings
    for (int i = 0; i < L; ++i) {
        for (int j = 0; j < S; ++j) {
            int j2 = (j + 1) % S;
            const auto& a = rings[i][j];
            const auto& b = rings[i][j2];
            const auto& c = rings[i + 1][j];
            const auto& d = rings[i + 1][j2];
            cgal_Triangle t1{a, b, c};
            cgal_Triangle t2{c, b, d};
            loft_vec3f n1 = (b - a).cross(c - a); n1 = n1 * (1.0f / (n1.length() + 1e-12f));
            loft_vec3f n2 = (b - d).cross(c - d); n2 = n2 * (1.0f / (n2.length() + 1e-12f));
            result.emplace_back(t1, cgal_vec3f{n1.x, n1.y, n1.z});
            result.emplace_back(t2, cgal_vec3f{n2.x, n2.y, n2.z});
        }
    }

    // Cap bottom pole (i=0)
    {
        loft_vec3f bottom = loft_vec3f{
            center.x - radius_b * tangent.x,
            center.y - radius_b * tangent.y,
            center.z - radius_b * tangent.z};
        loft_vec3f n_bot = tangent * -1.0f;
        for (int j = 0; j < S; ++j) {
            int j2 = (j + 1) % S;
            cgal_Triangle tri{bottom, rings[0][j2], rings[0][j]};
            result.emplace_back(tri, cgal_vec3f{n_bot.x, n_bot.y, n_bot.z});
        }
    }
    // Cap top pole (i=L)
    {
        loft_vec3f top = loft_vec3f{
            center.x + radius_b * tangent.x,
            center.y + radius_b * tangent.y,
            center.z + radius_b * tangent.z};
        for (int j = 0; j < S; ++j) {
            int j2 = (j + 1) % S;
            cgal_Triangle tri{top, rings[L][j], rings[L][j2]};
            result.emplace_back(tri, cgal_vec3f{tangent.x, tangent.y, tangent.z});
        }
    }

    return result;
}

// Build a closed cone mesh: base circle at `base_center` with `base_radius`,
// apex at `apex`, in the plane perpendicular to the cone axis.
MeshData build_cone_mesh(const loft_vec3f& apex,
                         const loft_vec3f& base_center,
                         float base_radius,
                         const loft_vec3f& axis_u,
                         const loft_vec3f& axis_v,
                         int circle_segments = 16) {
    MeshData result;
    const int C = circle_segments;
    auto circle_2d = build_circular_path(base_radius, C);
    std::vector<loft_vec3f> base_ring(C);
    for (int j = 0; j < C; ++j) {
        base_ring[j] = map_to_3d(circle_2d[j], base_center, axis_u, axis_v);
    }

    // Side surface: triangles from apex to base ring
    loft_vec3f axis = safe_normalize(apex - base_center, loft_vec3f{0, 1, 0});
    for (int j = 0; j < C; ++j) {
        int j2 = (j + 1) % C;
        cgal_Triangle tri{apex, base_ring[j], base_ring[j2]};
        loft_vec3f e1 = base_ring[j] - apex;
        loft_vec3f e2 = base_ring[j2] - apex;
        loft_vec3f n = e1.cross(e2); n = n * (1.0f / (n.length() + 1e-12f));
        // Ensure outward: normal should point away from the axis
        if (n.x * axis.x + n.y * axis.y + n.z * axis.z > 0) {
            n = n * -1.0f;
        }
        result.emplace_back(tri, cgal_vec3f{n.x, n.y, n.z});
    }

    // Base cap (triangle fan)
    for (int j = 0; j < C; ++j) {
        int j2 = (j + 1) % C;
        cgal_Triangle tri{base_center, base_ring[j2], base_ring[j]};
        loft_vec3f n = axis * -1.0f;
        result.emplace_back(tri, cgal_vec3f{n.x, n.y, n.z});
    }

    return result;
}

// Build a closed sphere mesh
MeshData build_sphere_mesh(const loft_vec3f& center, float radius,
                           int lat_segments = 8, int long_segments = 16) {
    // Use arbitrary frame for the sphere
    loft_vec3f tangent{0, 1, 0}, axis_u{1, 0, 0}, axis_v{0, 0, 1};
    return build_ellipsoid_mesh(center, tangent, axis_u, axis_v,
                                radius, radius, lat_segments, long_segments);
}

// Build teardrop tip: sphere at last guide point, cone extending forward
std::vector<MeshData> build_teardrop_primitives(
    const loft_vec3f& tip_center,
    const loft_vec3f& tangent,
    float sphere_radius, float cone_length,
    int circle_segments = 16) {
    std::vector<MeshData> out;
    loft_vec3f axis_u, axis_v;

    // Build local frame at tip
    float dot_yt = tangent.y;
    loft_vec3f v_raw{-tangent.x * dot_yt, 1.0f - tangent.y * dot_yt,
                     -tangent.z * dot_yt};
    axis_v = safe_normalize(v_raw, loft_vec3f{0, 1, 0});
    axis_u = loft_vec3f{
        tangent.y * axis_v.z - tangent.z * axis_v.y,
        tangent.z * axis_v.x - tangent.x * axis_v.z,
        tangent.x * axis_v.y - tangent.y * axis_v.x,
    };
    axis_u = safe_normalize(axis_u, loft_vec3f{1, 0, 0});

    // Sphere: centered at tip_center + tangent * sphere_radius (so back end
    // sits at tip_center)
    loft_vec3f sphere_center{
        tip_center.x + tangent.x * sphere_radius,
        tip_center.y + tangent.y * sphere_radius,
        tip_center.z + tangent.z * sphere_radius,
    };
    int lat_seg = std::max(3, circle_segments / 2);
    out.push_back(build_ellipsoid_mesh(sphere_center, tangent, axis_u, axis_v,
                                       sphere_radius, sphere_radius, lat_seg, circle_segments));

    // Cone: from sphere equator to tip
    loft_vec3f cone_apex{
        sphere_center.x + tangent.x * cone_length,
        sphere_center.y + tangent.y * cone_length,
        sphere_center.z + tangent.z * cone_length,
    };
    out.push_back(build_cone_mesh(cone_apex, sphere_center, sphere_radius,
                                  axis_u, axis_v, circle_segments));

    return out;
}

// Build joint ring: a thickened annulus ring (simplified socket joint)
MeshData build_joint_ring_mesh(
    const loft_vec3f& center,
    const loft_vec3f& tangent,
    const loft_vec3f& axis_u,
    const loft_vec3f& axis_v,
    float inner_radius, float outer_radius, float thickness,
    int circle_segments = 16) {
    MeshData result;
    const int C = circle_segments;

    auto inner_2d = build_circular_path(inner_radius, C);
    auto outer_2d = build_circular_path(outer_radius, C);

    // Front and back face centers
    loft_vec3f half_t{ tangent.x * thickness * 0.5f, tangent.y * thickness * 0.5f,
                       tangent.z * thickness * 0.5f };
    loft_vec3f front_center{ center.x + half_t.x, center.y + half_t.y,
                             center.z + half_t.z };
    loft_vec3f back_center{ center.x - half_t.x, center.y - half_t.y,
                            center.z - half_t.z };

    // Generate rings
    std::vector<loft_vec3f> inner_front(C), outer_front(C);
    std::vector<loft_vec3f> inner_back(C), outer_back(C);
    for (int j = 0; j < C; ++j) {
        inner_front[j] = map_to_3d(inner_2d[j], front_center, axis_u, axis_v);
        outer_front[j] = map_to_3d(outer_2d[j], front_center, axis_u, axis_v);
        inner_back[j] = map_to_3d(inner_2d[j], back_center, axis_u, axis_v);
        outer_back[j] = map_to_3d(outer_2d[j], back_center, axis_u, axis_v);
    }

    // Outer tube surface
    for (int j = 0; j < C; ++j) {
        int j2 = (j + 1) % C;
        const auto& a = outer_front[j];  const auto& b = outer_front[j2];
        const auto& c = outer_back[j];   const auto& d = outer_back[j2];
        cgal_Triangle t1{a, b, c}, t2{c, b, d};
        loft_vec3f n1 = (b - a).cross(c - a); n1 = n1 * (1.0f / (n1.length() + 1e-12f));
        loft_vec3f n2 = (b - d).cross(c - d); n2 = n2 * (1.0f / (n2.length() + 1e-12f));
        result.emplace_back(t1, cgal_vec3f{n1.x, n1.y, n1.z});
        result.emplace_back(t2, cgal_vec3f{n2.x, n2.y, n2.z});
    }

    // Inner tube surface (normals point inward)
    for (int j = 0; j < C; ++j) {
        int j2 = (j + 1) % C;
        const auto& a = inner_front[j];  const auto& b = inner_back[j];
        const auto& c = inner_front[j2]; const auto& d = inner_back[j2];
        cgal_Triangle t1{a, b, c}, t2{c, b, d};
        loft_vec3f n1 = (b - a).cross(c - a); n1 = n1 * (1.0f / (n1.length() + 1e-12f));
        loft_vec3f n2 = (b - d).cross(c - d); n2 = n2 * (1.0f / (n2.length() + 1e-12f));
        result.emplace_back(t1, cgal_vec3f{n1.x, n1.y, n1.z});
        result.emplace_back(t2, cgal_vec3f{n2.x, n2.y, n2.z});
    }

    // Front annulus cap (outer to inner)
    for (int j = 0; j < C; ++j) {
        int j2 = (j + 1) % C;
        cgal_Triangle t1{outer_front[j], outer_front[j2], inner_front[j]};
        cgal_Triangle t2{inner_front[j], outer_front[j2], inner_front[j2]};
        result.emplace_back(t1, cgal_vec3f{tangent.x, tangent.y, tangent.z});
        result.emplace_back(t2, cgal_vec3f{tangent.x, tangent.y, tangent.z});
    }
    // Back annulus cap
    {
        loft_vec3f n_back{ -tangent.x, -tangent.y, -tangent.z };
        for (int j = 0; j < C; ++j) {
            int j2 = (j + 1) % C;
            cgal_Triangle t1{outer_back[j], inner_back[j], outer_back[j2]};
            cgal_Triangle t2{inner_back[j], inner_back[j2], outer_back[j2]};
            result.emplace_back(t1, cgal_vec3f{n_back.x, n_back.y, n_back.z});
            result.emplace_back(t2, cgal_vec3f{n_back.x, n_back.y, n_back.z});
        }
    }

    return result;
}

// Incrementally union all primitives via CGAL, with fallback to concatenation
MeshData union_all_primitives(std::vector<MeshData>& primitives) {
    // Remove empties
    primitives.erase(
        std::remove_if(primitives.begin(), primitives.end(),
                       [](const MeshData& m) { return m.empty(); }),
        primitives.end());
    if (primitives.empty()) return {};

    MeshData result = std::move(primitives[0]);
    for (size_t i = 1; i < primitives.size(); ++i) {
        MeshData merged =
            sinriv::kigstudio::cgal::mesh_union(result, primitives[i]);
        if (!merged.empty()) {
            result = std::move(merged);
        } else {
            // Fallback: just concatenate
            result.insert(result.end(), primitives[i].begin(),
                          primitives[i].end());
        }
    }
    return result;
}

// ---- Special strand type builders ----

std::vector<std::tuple<loft_Triangle, loft_vec3f>>
build_candied_hawthorn_mesh(const HairStrand& strand) {
    using namespace sinriv::kigstudio::mesh::loft;
    std::vector<std::tuple<loft_Triangle, loft_vec3f>> empty_result;

    if (strand.guide_points.size() < 2) return empty_result;

    auto sampled = sample_bezier_guide_curve(
        strand.guide_points, std::max(strand.guide_samples_per_segment, 1));
    if (sampled.size() < 2) return empty_result;

    // Convert sampled to loft_vec3f
    std::vector<loft_vec3f> guide_curve;
    guide_curve.reserve(sampled.size());
    for (const auto& p : sampled)
        guide_curve.push_back(loft_vec3f{p.x, p.y, p.z});

    std::vector<MeshData> primitives;

    int Q = std::clamp(strand.special_quality, 4, 64);
    int Q_lat = std::max(4, Q / 2);  // latitude uses half resolution

    // 1. Core cylinder along the full guide curve
    primitives.push_back(
        build_cylinder_mesh(guide_curve, strand.candy_cylinder_radius, Q));

    // 2. Ellipsoids at regular intervals
    // Compute total arc length
    float total_arc = 0.0f;
    for (size_t i = 1; i < guide_curve.size(); ++i) {
        total_arc +=
            (guide_curve[i] - guide_curve[i - 1]).length();
    }
    if (strand.candy_ellipsoid_spacing > 0.01f && total_arc > 0.01f) {
        float dist = strand.candy_ellipsoid_spacing;
        while (dist < total_arc - 0.01f) {
            // Find the sample point closest to `dist` along the curve
            float accum = 0.0f;
            int best_i = 0;
            for (size_t i = 1; i < guide_curve.size(); ++i) {
                float seg =
                    (guide_curve[i] - guide_curve[i - 1]).length();
                if (accum + seg >= dist) {
                    float local_t =
                        (dist - accum) / (seg + 1e-10f);
                    best_i = (local_t < 0.5f) ? static_cast<int>(i - 1)
                                              : static_cast<int>(i);
                    break;
                }
                accum += seg;
            }
            if (best_i >= static_cast<int>(guide_curve.size()))
                best_i = static_cast<int>(guide_curve.size()) - 1;

            loft_vec3f tangent, axis_u, axis_v;
            local_frame_at_sample(guide_curve, best_i, tangent, axis_u, axis_v);

            primitives.push_back(build_ellipsoid_mesh(
                guide_curve[best_i], tangent, axis_u, axis_v,
                strand.candy_ellipsoid_radius_a,
                strand.candy_ellipsoid_radius_b,
                Q_lat, Q));

            // Optional joint ring at far side of ellipsoid
            if (strand.candy_use_joints) {
                loft_vec3f joint_center{
                    guide_curve[best_i].x +
                        tangent.x * strand.candy_ellipsoid_radius_b,
                    guide_curve[best_i].y +
                        tangent.y * strand.candy_ellipsoid_radius_b,
                    guide_curve[best_i].z +
                        tangent.z * strand.candy_ellipsoid_radius_b,
                };
                float jr = strand.candy_ellipsoid_radius_a;
                primitives.push_back(build_joint_ring_mesh(
                    joint_center, tangent, axis_u, axis_v,
                    jr * 0.85f, jr * 1.15f, jr * 0.3f, Q));
            }

            dist += strand.candy_ellipsoid_spacing;
        }
    }

    // 3. Teardrop tip at the last sample point
    {
        loft_vec3f tangent, axis_u, axis_v;
        local_frame_at_sample(guide_curve, static_cast<int>(guide_curve.size()) - 1,
                              tangent, axis_u, axis_v);
        auto tip_parts = build_teardrop_primitives(
            guide_curve.back(), tangent, strand.special_tip_radius,
            strand.special_tip_length, Q);
        for (auto& p : tip_parts)
            primitives.push_back(std::move(p));
    }

    // Concatenate all primitives (skip CGAL boolean union — the
    // overlapping cylinders/ellipsoids cause cascading union failures
    // that degrade the mesh; concatenation renders correctly for both
    // braid and candied hawthorn visual styles).
    MeshData merged;
    size_t total_faces = 0;
    for (auto& p : primitives) total_faces += p.size();
    merged.reserve(total_faces);
    for (auto& p : primitives)
        merged.insert(merged.end(), p.begin(), p.end());
    if (merged.empty()) return empty_result;

    // Convert MeshData to return type (they are the same underlying types)
    std::vector<std::tuple<loft_Triangle, loft_vec3f>> result;
    result.reserve(merged.size());
    for (auto& [tri, n] : merged)
        result.emplace_back(std::move(tri), std::move(n));
    return result;
}

std::vector<std::tuple<loft_Triangle, loft_vec3f>>
build_braid_mesh(const HairStrand& strand) {
    using namespace sinriv::kigstudio::mesh::loft;
    std::vector<std::tuple<loft_Triangle, loft_vec3f>> empty_result;

    if (strand.guide_points.size() < 2) return empty_result;

    auto sampled = sample_bezier_guide_curve(
        strand.guide_points, std::max(strand.guide_samples_per_segment, 1));
    if (sampled.size() < 2) return empty_result;

    std::vector<loft_vec3f> guide_curve;
    guide_curve.reserve(sampled.size());
    for (const auto& p : sampled)
        guide_curve.push_back(loft_vec3f{p.x, p.y, p.z});

    const int M = static_cast<int>(guide_curve.size());

    // Pre-compute arc length and local frames
    std::vector<float> accum_arc(M);
    accum_arc[0] = 0.0f;
    for (int i = 1; i < M; ++i) {
        accum_arc[i] = accum_arc[i - 1] +
                       (guide_curve[i] - guide_curve[i - 1]).length();
    }
    float total_arc = accum_arc.back();

    std::vector<loft_vec3f> tangents(M), axis_us(M), axis_vs(M);
    for (int i = 0; i < M; ++i) {
        local_frame_at_sample(guide_curve, i, tangents[i], axis_us[i],
                              axis_vs[i]);
    }

    std::vector<MeshData> primitives;

    int Q = std::clamp(strand.special_quality, 4, 64);

    // 1. Core cylinder
    primitives.push_back(
        build_cylinder_mesh(guide_curve, strand.braid_core_radius, Q));

    // 2. Braid strands: true 3-strand braid topology.
    //    Each strand oscillates between 3 "lanes" (left/center/right) via a
    //    trapezoidal wave, never completing a full rotation around the axis.
    //    The three strands are the same curve shifted by 2h along t.
    //
    //    Model: σ₁σ₂ alternating braid (standard 3-strand).
    //    p(t): horizontal lane position  (period 6, trapezoidal with dwell)
    //    q(t): over/under depth          (period 3)
    //    h     = pitch / 6               (cross-step length)
    //    a     = braid_braid_radius      (lane spacing)
    //    b     = strand_radius * 1.5     (depth, ≥radius avoids interpenetration)

    // --- p(t): trapezoidal horizontal wave, period 6 ---
    auto braid_p = [](float t_mod6) -> float {
        // t_mod6 in [0, 6)
        if (t_mod6 < 1.0f)      // left→center: -(1+cos πt)/2
            return -(1.0f + std::cos(3.14159265358979f * t_mod6)) * 0.5f;
        else if (t_mod6 < 2.0f) // center→right: (1-cos π(t-1))/2
            return (1.0f - std::cos(3.14159265358979f * (t_mod6 - 1.0f))) * 0.5f;
        else if (t_mod6 < 3.0f) // right dwell: 1
            return 1.0f;
        else if (t_mod6 < 4.0f) // right→center: (1+cos π(t-3))/2
            return (1.0f + std::cos(3.14159265358979f * (t_mod6 - 3.0f))) * 0.5f;
        else if (t_mod6 < 5.0f) // center→left: -(1-cos π(t-4))/2
            return -(1.0f - std::cos(3.14159265358979f * (t_mod6 - 4.0f))) * 0.5f;
        else                     // left dwell: -1
            return -1.0f;
    };

    // --- q(t): over/under depth, period 3 ---
    auto braid_q = [](float t_mod3) -> float {
        // t_mod3 in [0, 3)
        if (t_mod3 < 1.0f)      // crossing over: +sin πt
            return std::sin(3.14159265358979f * t_mod3);
        else if (t_mod3 < 2.0f) // being crossed under: -sin π(t-1)
            return -std::sin(3.14159265358979f * (t_mod3 - 1.0f));
        else                     // rest (at edge, no crossing): 0
            return 0.0f;
    };

    int count = std::clamp(strand.braid_strand_count, 2, 6);
    float pitch = std::max(strand.braid_twist_pitch, 0.01f);
    float h_step = pitch / 6.0f;  // cross-step length
    float a = strand.braid_braid_radius;  // lane spacing
    float b = strand.braid_strand_radius * 1.5f;  // over/under depth

    for (int s = 0; s < count; ++s) {
        // For the mathematically correct 3-strand model, strands are
        // the same curve shifted by 2h. For other counts, spread
        // evenly across the 6-step cycle.
        float t_shift;
        if (count == 3) {
            t_shift = static_cast<float>(s) * 2.0f;  // shifts: 0, 2, 4
        } else {
            t_shift = static_cast<float>(s) * 6.0f / static_cast<float>(count);
        }

        std::vector<loft_vec3f> braid_curve(M);
        for (int i = 0; i < M; ++i) {
            float arc = accum_arc[i];
            float t = arc / h_step - t_shift;

            // Wrap t into [0, 6) and [0, 3) for p and q
            float t6 = std::fmod(t, 6.0f);
            if (t6 < 0.0f) t6 += 6.0f;
            float t3 = std::fmod(t, 3.0f);
            if (t3 < 0.0f) t3 += 3.0f;

            float px = a * braid_p(t6);  // horizontal lane offset
            float py = b * braid_q(t3);  // over/under depth

            braid_curve[i] = loft_vec3f{
                guide_curve[i].x +
                    px * axis_us[i].x + py * axis_vs[i].x,
                guide_curve[i].y +
                    px * axis_us[i].y + py * axis_vs[i].y,
                guide_curve[i].z +
                    px * axis_us[i].z + py * axis_vs[i].z,
            };
        }
        primitives.push_back(
            build_cylinder_mesh(braid_curve, strand.braid_strand_radius, Q));
    }

    // 3. Joints at uniform intervals along the core
    if (strand.braid_use_joints && total_arc > 0.01f) {
        float joint_spacing = strand.braid_twist_pitch / 4.0f;
        if (joint_spacing < 0.1f) joint_spacing = strand.braid_twist_pitch;
        float dist = joint_spacing;
        while (dist < total_arc - 0.01f) {
            // Find nearest sample index
            int best_i = 0;
            float best_diff = 1e10f;
            for (int i = 0; i < M; ++i) {
                float diff = std::abs(accum_arc[i] - dist);
                if (diff < best_diff) { best_diff = diff; best_i = i; }
            }
            float jr = strand.braid_braid_radius + strand.braid_strand_radius;
            primitives.push_back(build_joint_ring_mesh(
                guide_curve[best_i], tangents[best_i], axis_us[best_i],
                axis_vs[best_i], jr * 0.85f, jr * 1.15f, jr * 0.3f, Q));
            dist += joint_spacing;
        }
    }

    // 4. Teardrop tip
    {
        auto tip_parts = build_teardrop_primitives(
            guide_curve.back(), tangents.back(),
            strand.special_tip_radius, strand.special_tip_length, Q);
        for (auto& p : tip_parts)
            primitives.push_back(std::move(p));
    }

    // Concatenate all primitives (same rationale as candied hawthorn)
    MeshData merged;
    size_t total_faces = 0;
    for (auto& p : primitives) total_faces += p.size();
    merged.reserve(total_faces);
    for (auto& p : primitives)
        merged.insert(merged.end(), p.begin(), p.end());
    if (merged.empty()) return empty_result;

    std::vector<std::tuple<loft_Triangle, loft_vec3f>> result;
    result.reserve(merged.size());
    for (auto& [tri, n] : merged)
        result.emplace_back(std::move(tri), std::move(n));
    return result;
}

// Build loft mesh triangles from a single hair strand.
// Returns (Triangle, normal) pairs suitable for RenderMesh::loadGeometry().
std::vector<std::tuple<loft_Triangle, loft_vec3f>> build_hair_strand_mesh(
    const HairStrand& strand) {
	using namespace sinriv::kigstudio::mesh::loft;
	std::vector<std::tuple<loft_Triangle, loft_vec3f>> result;

	// Merge hidden guide points with visible guide points for lofting
	std::vector<sinriv::kigstudio::voxel::vec3f> all_guide_points;
	all_guide_points.reserve(strand.hidden_guide_points_start.size() +
	                         strand.guide_points.size() +
	                         strand.hidden_guide_points_end.size());
	all_guide_points.insert(all_guide_points.end(),
	                        strand.hidden_guide_points_start.begin(),
	                        strand.hidden_guide_points_start.end());
	all_guide_points.insert(all_guide_points.end(),
	                        strand.guide_points.begin(),
	                        strand.guide_points.end());
	all_guide_points.insert(all_guide_points.end(),
	                        strand.hidden_guide_points_end.begin(),
	                        strand.hidden_guide_points_end.end());

	if (all_guide_points.size() < 2) return result;

	// Dispatch to special strand type builders (parameter-driven, no width_points needed)
	if (strand.gen_type == HairStrandGenType::CANDIED_HAWTHORN)
		return build_candied_hawthorn_mesh(strand);
	if (strand.gen_type == HairStrandGenType::BRAID)
		return build_braid_mesh(strand);

	// --- NORMAL path below ---
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
	    all_guide_points, std::max(strand.guide_samples_per_segment, 1));
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

		// Resample per-point override to match global section vertex
		// count, preventing "Loft sections must have matching path
		// sizes" errors when per-point sections differ from the global.
		std::vector<sinriv::kigstudio::vec2<float>> temp_resampled;
		if (per_sample_path != &global_section_path &&
		    per_sample_path->size() != global_section_path.size()) {
			resample_closed_polygon(*per_sample_path, temp_resampled,
				static_cast<int>(global_section_path.size()));
			per_sample_path = &temp_resampled;
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

void RenderVoxelList::RenderVoxelItem::apply_hairline_spindle() {
    if (hair_strands.empty()) return;

    // ---- Step 1: Compute hairline plane equation ----
    // Plane: normal · point = d
    vec3f plane_normal;
    float plane_d;

    if (hairline_plane_use_y) {
        // Horizontal plane: Y = hairline_plane_y
        plane_normal = {0.0f, 1.0f, 0.0f};
        plane_d = hairline_plane_y;
    } else {
        // Three-point plane
        const vec3f& p0 = hairline_plane_points[0];
        const vec3f& p1 = hairline_plane_points[1];
        const vec3f& p2 = hairline_plane_points[2];
        vec3f edge1 = {p1.x - p0.x, p1.y - p0.y, p1.z - p0.z};
        vec3f edge2 = {p2.x - p0.x, p2.y - p0.y, p2.z - p0.z};
        plane_normal = {
            edge1.y * edge2.z - edge1.z * edge2.y,
            edge1.z * edge2.x - edge1.x * edge2.z,
            edge1.x * edge2.y - edge1.y * edge2.x,
        };
        float len = std::sqrt(plane_normal.x * plane_normal.x +
                              plane_normal.y * plane_normal.y +
                              plane_normal.z * plane_normal.z);
        if (len < 1e-8f) return;  // degenerate
        plane_normal = {plane_normal.x / len, plane_normal.y / len,
                        plane_normal.z / len};
        plane_d = plane_normal.x * p0.x + plane_normal.y * p0.y +
                  plane_normal.z * p0.z;
    }

    // Helper: signed distance from point to plane
    auto signed_dist = [&](const vec3f& p) -> float {
        return p.x * plane_normal.x + p.y * plane_normal.y +
               p.z * plane_normal.z - plane_d;
    };

    // ---- Step 2: Find hairline-plane intersection for each strand ----
    struct StrandIntersection {
        int strand_idx;
        float curve_id;  // position on guide curve
        vec3f world_pos; // intersection point in world space
    };
    std::vector<StrandIntersection> intersections;

    for (int si = 0; si < static_cast<int>(hair_strands.size()); ++si) {
        const auto& strand = hair_strands[si];
        if (strand.guide_points.size() < 2) continue;

        const int subdiv = std::max(strand.guide_samples_per_segment, 1);
        auto sampled = sample_bezier_guide_curve(strand.guide_points, subdiv);
        if (sampled.size() < 2) continue;

        const int N = static_cast<int>(strand.guide_points.size());
        const int M = static_cast<int>(sampled.size());

        // Count crossings of the guide curve sample polyline with the plane
        struct Crossing {
            float curve_id;
            vec3f world_pos;
        };
        std::vector<Crossing> crossings;

        for (int i = 0; i < M - 1; ++i) {
            float d1 = signed_dist(sampled[i]);
            float d2 = signed_dist(sampled[i + 1]);

            // Sign change (or exact hit at d1==0)
            if (d1 * d2 > 0.0f) continue;  // same side, no crossing
            if (std::abs(d1) < 1e-7f && std::abs(d2) < 1e-7f) continue;  // grazing

            // Exact hit at sample i
            if (std::abs(d1) < 1e-7f) {
                float curve_id = static_cast<float>(i) /
                                 static_cast<float>(M - 1) *
                                 static_cast<float>(N - 1);
                crossings.push_back({curve_id, sampled[i]});
                continue;
            }

            // Interpolate crossing between sample i and i+1
            float t = -d1 / (d2 - d1);
            t = std::clamp(t, 0.0f, 1.0f);

            vec3f pos = {
                sampled[i].x + t * (sampled[i + 1].x - sampled[i].x),
                sampled[i].y + t * (sampled[i + 1].y - sampled[i].y),
                sampled[i].z + t * (sampled[i + 1].z - sampled[i].z),
            };

            float cid_i = static_cast<float>(i) /
                          static_cast<float>(M - 1) *
                          static_cast<float>(N - 1);
            float cid_i1 = static_cast<float>(i + 1) /
                           static_cast<float>(M - 1) *
                           static_cast<float>(N - 1);
            float curve_id = cid_i + t * (cid_i1 - cid_i);

            crossings.push_back({curve_id, pos});
        }

        // Only process strands with exactly one intersection
        if (crossings.size() != 1) continue;

        intersections.push_back(
            {si, crossings[0].curve_id, crossings[0].world_pos});
    }

    if (intersections.empty()) return;

    // ---- Step 3: Generate spindle width points for each valid strand ----
    for (size_t ii = 0; ii < intersections.size(); ++ii) {
        const auto& inter = intersections[ii];
        auto& strand = hair_strands[inter.strand_idx];

        // Compute nearest-neighbor distance (width at hairline)
        float nearest = std::numeric_limits<float>::max();
        for (size_t jj = 0; jj < intersections.size(); ++jj) {
            if (jj == ii) continue;
            const auto& other = intersections[jj];
            float dx = inter.world_pos.x - other.world_pos.x;
            float dy = inter.world_pos.y - other.world_pos.y;
            float dz = inter.world_pos.z - other.world_pos.z;
            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist < nearest) nearest = dist;
        }
        if (nearest >= std::numeric_limits<float>::max()) nearest = 0.5f;

        const int N = static_cast<int>(strand.guide_points.size());
        if (N < 2) continue;

        // Determine width direction:
        // - If center point is enabled: direction from curve toward center
        // - Otherwise: world-down (0, -1, 0)
        auto sample = sample_guide_curve_at(inter.strand_idx, inter.curve_id);
        vec3f dir;
        if (show_addon_center) {
            dir = {
                addon_center_point.x - sample.position.x,
                addon_center_point.y - sample.position.y,
                addon_center_point.z - sample.position.z,
            };
            float dlen = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
            if (dlen < 1e-6f) {
                dir = {0.0f, -1.0f, 0.0f};
            } else {
                dir = {dir.x / dlen, dir.y / dlen, dir.z / dlen};
            }
        } else {
            dir = {0.0f, -1.0f, 0.0f};
        }

        // Clear existing width points and replace with spindle
        strand.width_points.clear();

        // Spindle: three width points creating a taper from start to hairline
        // to end
        //   - Start (curve_id=0): tiny scale
        //   - Hairline (inter.curve_id): full neighbor-distance scale
        //   - End (curve_id=N-1): tiny scale
        HairStrand::WidthPoint wp_start;
        wp_start.curve_id = 0.0f;
        wp_start.scale = kMinTipWidth;
        wp_start.direction = dir;

        HairStrand::WidthPoint wp_hairline;
        wp_hairline.curve_id = inter.curve_id;
        wp_hairline.scale = nearest * hairline_spindle_scale;
        wp_hairline.direction = dir;

        HairStrand::WidthPoint wp_end;
        wp_end.curve_id = static_cast<float>(N - 1);
        wp_end.scale = kMinTipWidth;
        wp_end.direction = dir;

        strand.width_points.push_back(wp_start);
        strand.width_points.push_back(wp_hairline);
        strand.width_points.push_back(wp_end);

        strand.mesh_dirty = true;
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
	// Collect UUIDs of currently visible strands
	std::unordered_set<std::string> needed;
	for (auto& strand : hair_strands) {
		if (strand.visible) needed.insert(strand.uuid);
	}

	// Remove entries for strands that were deleted or became invisible
	for (auto it = addon_renderers.begin(); it != addon_renderers.end(); ) {
		if (needed.find(it->first) == needed.end()) {
			it = addon_renderers.erase(it);
		} else {
			++it;
		}
	}

	// Build or rebuild only dirty strands
	for (auto& strand : hair_strands) {
		if (!strand.visible || !strand.mesh_dirty) continue;
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
			auto renderer = std::make_unique<sinriv::ui::render::RenderMesh>();
			renderer->setBaseColor(0.3f, 0.65f, 0.42f, 1.0f);
			renderer->loadGeometry(out);
			addon_renderers[strand.uuid] = std::move(renderer);
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
    bool show_origin = showOriginMesh;
    if (source_type == 2) {
        show_origin = showOriginMesh || showOriginMeshAddon;
    }
    if (show_origin && !origin_mesh_renderer.empty()) {
        origin_mesh_renderer.cull_backface = false;
        origin_mesh_renderer.renderGBuffer(transform, mesh_shader);
    }
    if (showExportedMesh && !cached_mesh.empty()) {
        if (!exported_mesh_synced) {
            exported_mesh_renderer.loadGeometry(cached_mesh);
            exported_mesh_synced = true;
        }
        exported_mesh_renderer.renderGBuffer(transform, mesh_shader);
    }

    // Rebuild addon meshes if any strand is dirty, or clear when
    // Rebuild addon meshes selectively (only dirty strands are rebuilt)
    if (!hair_strands.empty() || !addon_renderers.empty()) {
        update_addon_meshes();
        for (auto& strand : hair_strands)
            strand.mesh_dirty = false;
    }

    // 附加件渲染器（如毛发预览）：写入 albedo/normal/depth，与主模型正确
    // 互相遮挡，但不写 world_pos 通道，鼠标拾取可穿透它拾取下层模型
    if (!manager || showAddonMesh) {
        for (auto& [uuid, addon] : addon_renderers) {
            addon->cull_backface = false;
            addon->renderGBufferAddon(transform, mesh_shader);
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
    // Compute bounding sphere radius from base mesh (used by guide point
    // crosshair markers, center circles, arrow, triangle, sphere wireframe).
    // For addon nodes (source_type==2), look up the base node's mesh via
    // addon_base_node_id since the addon itself doesn't own the mesh triangles.
    float sphere_r = 1.0f;
    if (source_type == 2 && manager) {
        float max_dist2 = 0.0f;
        auto check_vertex = [&](const sinriv::kigstudio::voxel::vec3f& v) {
            float d2 = (v - addon_center_point).length2();
            if (d2 > max_dist2) max_dist2 = d2;
        };
        auto base_it = manager->items.find(addon_base_node_id);
        if (base_it != manager->items.end()) {
            const auto& base = *base_it->second;
            if (!base.source_triangles.empty()) {
                for (const auto& tri : base.source_triangles) {
                    check_vertex(std::get<0>(tri));
                    check_vertex(std::get<1>(tri));
                    check_vertex(std::get<2>(tri));
                }
            } else if (!base.cached_mesh.empty()) {
                for (const auto& tri_n : base.cached_mesh) {
                    const auto& tri = std::get<0>(tri_n);
                    check_vertex(std::get<0>(tri));
                    check_vertex(std::get<1>(tri));
                    check_vertex(std::get<2>(tri));
                }
            }
        }
        if (max_dist2 > 0.0f)
            sphere_r = std::sqrt(max_dist2) * 1.05f;  // 5% margin
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
                     active_guide_draw_strand == hair_strands[si].uuid) ||
                    (width_editing_active &&
                     active_width_edit_strand == hair_strands[si].uuid);
                uint32_t line_color =
                    is_active ? active_line_color : idle_line_color;
                uint32_t marker_color =
                    is_active ? active_marker_color : idle_marker_color;
                // --- Unified guide curve rendering ---
                // When hidden guide points exist, sample from the same
                // all_guide_points that build_hair_strand_mesh uses.
                // Each line segment is colored gray (hidden) or
                // white/yellow (visible) based on which control points
                // it spans.  No double-draw, no sharp angles.
                bool has_hidden =
                    !strand.hidden_guide_points_start.empty() ||
                    !strand.hidden_guide_points_end.empty();

                const int sps =
                    std::max(strand.guide_samples_per_segment, 1);
                const float marker_size = sphere_r * 0.03f;

                if (has_hidden) {
                    // Build all_guide_points exactly as loft does
                    std::vector<sinriv::kigstudio::voxel::vec3f> all_pts;
                    all_pts.reserve(
                        strand.hidden_guide_points_start.size() +
                        strand.guide_points.size() +
                        strand.hidden_guide_points_end.size());
                    all_pts.insert(all_pts.end(),
                                   strand.hidden_guide_points_start.begin(),
                                   strand.hidden_guide_points_start.end());
                    all_pts.insert(all_pts.end(),
                                   strand.guide_points.begin(),
                                   strand.guide_points.end());
                    all_pts.insert(all_pts.end(),
                                   strand.hidden_guide_points_end.begin(),
                                   strand.hidden_guide_points_end.end());

                    if (all_pts.size() < 2) continue;

                    auto sampled = sample_bezier_guide_curve(all_pts, sps);
                    const int H =
                        static_cast<int>(strand.hidden_guide_points_start.size());
                    const int V =
                        static_cast<int>(strand.guide_points.size());

                    const uint32_t hidden_line_col =
                        pack_abgr(0.45f, 0.45f, 0.45f, 0.5f);
                    const uint32_t hidden_marker_col =
                        pack_abgr(0.5f, 0.5f, 0.5f, 0.7f);

                    // Segment s (connecting all_pts[s]→all_pts[s+1]) is
                    // visible-white only when both endpoints are
                    // guide_points, i.e. H <= s < H+V-1.
                    // Everything else is gray.
                    for (size_t k = 0; k + 1 < sampled.size(); ++k) {
                        int seg = static_cast<int>(k) / sps;
                        bool vis = (seg >= H && seg < H + V - 1);
                        uint32_t col =
                            vis ? line_color : hidden_line_col;
                        const auto& a = sampled[k];
                        const auto& b = sampled[k + 1];
                        vertices.push_back(
                            {a.x, -a.y, a.z, col});
                        vertices.push_back(
                            {b.x, -b.y, b.z, col});
                    }

                    // Visible guide points → normal markers
                    for (const auto& p : strand.guide_points) {
                        vertices.push_back({p.x - marker_size, -p.y, p.z,
                                            marker_color});
                        vertices.push_back({p.x + marker_size, -p.y, p.z,
                                            marker_color});
                        vertices.push_back({p.x,
                                            -(p.y - marker_size), p.z,
                                            marker_color});
                        vertices.push_back({p.x,
                                            -(p.y + marker_size), p.z,
                                            marker_color});
                        vertices.push_back({p.x, -p.y,
                                            p.z - marker_size,
                                            marker_color});
                        vertices.push_back({p.x, -p.y,
                                            p.z + marker_size,
                                            marker_color});
                    }

                    // Hidden guide points → gray cross markers
                    for (const auto& p :
                         strand.hidden_guide_points_start) {
                        vertices.push_back({p.x - marker_size, -p.y, p.z,
                                            hidden_marker_col});
                        vertices.push_back({p.x + marker_size, -p.y, p.z,
                                            hidden_marker_col});
                        vertices.push_back({p.x,
                                            -(p.y - marker_size), p.z,
                                            hidden_marker_col});
                        vertices.push_back({p.x,
                                            -(p.y + marker_size), p.z,
                                            hidden_marker_col});
                        vertices.push_back({p.x, -p.y,
                                            p.z - marker_size,
                                            hidden_marker_col});
                        vertices.push_back({p.x, -p.y,
                                            p.z + marker_size,
                                            hidden_marker_col});
                    }
                    for (const auto& p :
                         strand.hidden_guide_points_end) {
                        vertices.push_back({p.x - marker_size, -p.y, p.z,
                                            hidden_marker_col});
                        vertices.push_back({p.x + marker_size, -p.y, p.z,
                                            hidden_marker_col});
                        vertices.push_back({p.x,
                                            -(p.y - marker_size), p.z,
                                            hidden_marker_col});
                        vertices.push_back({p.x,
                                            -(p.y + marker_size), p.z,
                                            hidden_marker_col});
                        vertices.push_back({p.x, -p.y,
                                            p.z - marker_size,
                                            hidden_marker_col});
                        vertices.push_back({p.x, -p.y,
                                            p.z + marker_size,
                                            hidden_marker_col});
                    }
                } else {
                    // No hidden points — simple path: sample guide_points
                    // directly
                    auto sampled = sample_bezier_guide_curve(
                        strand.guide_points, sps);
                    for (size_t k = 0; k + 1 < sampled.size(); ++k) {
                        const auto& a = sampled[k];
                        const auto& b = sampled[k + 1];
                        vertices.push_back(
                            {a.x, -a.y, a.z, line_color});
                        vertices.push_back(
                            {b.x, -b.y, b.z, line_color});
                    }
                    for (const auto& p : strand.guide_points) {
                        vertices.push_back({p.x - marker_size, -p.y, p.z,
                                            marker_color});
                        vertices.push_back({p.x + marker_size, -p.y, p.z,
                                            marker_color});
                        vertices.push_back({p.x,
                                            -(p.y - marker_size), p.z,
                                            marker_color});
                        vertices.push_back({p.x,
                                            -(p.y + marker_size), p.z,
                                            marker_color});
                        vertices.push_back({p.x, -p.y,
                                            p.z - marker_size,
                                            marker_color});
                        vertices.push_back({p.x, -p.y,
                                            p.z + marker_size,
                                            marker_color});
                    }
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
                    (hair_strands[si].uuid == active_width_edit_strand);
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
            const float radius = sphere_r * 0.05f;
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

    // Ortho projection vector preview (shown when setup window is open)
    if (manager && manager->show_ortho_setup_window &&
        show_addon_center && mesh_shader.ensureLineProgram()) {
        bgfx::VertexLayout& layout = concave_cone_overlay_layout();
        const uint32_t arrow_color = pack_abgr(0.2f, 0.6f, 1.0f, 1.0f);  // blue
        vec3f dir = manager->ortho_state.projection_dir;
        float vp_half = manager->ortho_state.viewport_size * 0.5f;
        vec3f arrow_end = {
            addon_center_point.x + dir.x * vp_half,
            addon_center_point.y + dir.y * vp_half,
            addon_center_point.z + dir.z * vp_half
        };
        std::vector<mesh_detail::ColorLineVertex> vertices;
        // Main direction line
        vertices.push_back({addon_center_point.x, -addon_center_point.y,
                            addon_center_point.z, arrow_color});
        vertices.push_back({arrow_end.x, -arrow_end.y, arrow_end.z, arrow_color});
        // Small cross at arrow end
        float cs = vp_half * 0.05f;
        vertices.push_back({arrow_end.x - cs, -arrow_end.y, arrow_end.z, arrow_color});
        vertices.push_back({arrow_end.x + cs, -arrow_end.y, arrow_end.z, arrow_color});
        vertices.push_back({arrow_end.x, -(arrow_end.y - cs), arrow_end.z, arrow_color});
        vertices.push_back({arrow_end.x, -(arrow_end.y + cs), arrow_end.z, arrow_color});
        vertices.push_back({arrow_end.x, -arrow_end.y, arrow_end.z - cs, arrow_color});
        vertices.push_back({arrow_end.x, -arrow_end.y, arrow_end.z + cs, arrow_color});

        if (!vertices.empty() &&
            bgfx::getAvailTransientVertexBuffer(
                static_cast<uint32_t>(vertices.size()), layout) >=
                vertices.size()) {
            bgfx::TransientVertexBuffer tvb;
            bgfx::allocTransientVertexBuffer(
                &tvb, static_cast<uint32_t>(vertices.size()), layout);
            std::memcpy(tvb.data, vertices.data(),
                        vertices.size() * sizeof(mesh_detail::ColorLineVertex));
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

    // 语义坐标框架调试可视化：棕色北极箭头 + 矢状面三角
    // 只要显示了中心点就渲染坐标系参考框架
    if (show_addon_center && mesh_shader.ensureLineProgram()) {
        bgfx::VertexLayout& layout = concave_cone_overlay_layout();

        // 构建局部球形坐标系（与 agent_handlers.h 中 spherical_to_dir 相同的数学）
        vec3f N = hair_north_pole.normalize();  // 北极方向 (phi=+90°)
        vec3f F = hair_front_reference.normalize();  // 前参考方向
        float f_dot_n = F.dot(N);
        vec3f V = F - N * f_dot_n;  // 投影到赤道面 → theta=0° 方向
        float v_len2 = V.length2();

        // 退化情况：front_reference 与 north_pole 几乎平行 → 回退试探
        if (v_len2 < 1e-10f) {
            vec3f A = (std::abs(N.z) < 0.99f)
                          ? vec3f(0.0f, 0.0f, 1.0f)
                          : vec3f(1.0f, 0.0f, 0.0f);
            V = A - N * A.dot(N);
            v_len2 = V.length2();
        }
        V = V / std::sqrt(v_len2);  // theta=0° 前方向（鼻尖侧）
        // U = cross(N, V) → theta=+90° 右方向
        vec3f U{N.y * V.z - N.z * V.y, N.z * V.x - N.x * V.z,
                N.x * V.y - N.y * V.x};

        const vec3f& center = addon_center_point;
        const float arrow_len = sphere_r * 1.2f;
        const float head_size = sphere_r * 0.15f;
        const float tri_leg = sphere_r;

        const uint32_t arrow_color =
            pack_abgr(0.55f, 0.27f, 0.07f, 1.0f);  // 棕色
        const uint32_t tri_color =
            pack_abgr(0.7f, 0.45f, 0.15f, 1.0f);  // 浅棕色

        std::vector<mesh_detail::ColorLineVertex> vertices;
        vertices.reserve(20);

        // --- 北极箭头 ---
        vec3f arrow_tip{center.x + N.x * arrow_len,
                        center.y + N.y * arrow_len,
                        center.z + N.z * arrow_len};
        // 箭杆
        vertices.push_back({center.x, -center.y, center.z, arrow_color});
        vertices.push_back(
            {arrow_tip.x, -arrow_tip.y, arrow_tip.z, arrow_color});

        // 箭头（使用 V 方向作为箭头宽度的参考）
        vec3f head_base{arrow_tip.x - N.x * head_size,
                        arrow_tip.y - N.y * head_size,
                        arrow_tip.z - N.z * head_size};
        float hw = head_size * 0.5f;
        vec3f head_l{head_base.x + V.x * hw, head_base.y + V.y * hw,
                     head_base.z + V.z * hw};
        vec3f head_r{head_base.x - V.x * hw, head_base.y - V.y * hw,
                     head_base.z - V.z * hw};

        vertices.push_back(
            {arrow_tip.x, -arrow_tip.y, arrow_tip.z, arrow_color});
        vertices.push_back({head_l.x, -head_l.y, head_l.z, arrow_color});
        vertices.push_back(
            {arrow_tip.x, -arrow_tip.y, arrow_tip.z, arrow_color});
        vertices.push_back({head_r.x, -head_r.y, head_r.z, arrow_color});

        // --- 矢状面等腰直角三角形（直角在中心点）---
        // 直角边1：沿 N（北极）
        vec3f tri_up{center.x + N.x * tri_leg, center.y + N.y * tri_leg,
                     center.z + N.z * tri_leg};
        // 直角边2：沿 V（前/鼻尖方向）
        vec3f tri_front{center.x + V.x * tri_leg,
                        center.y + V.y * tri_leg,
                        center.z + V.z * tri_leg};

        vertices.push_back({center.x, -center.y, center.z, tri_color});
        vertices.push_back({tri_up.x, -tri_up.y, tri_up.z, tri_color});
        vertices.push_back({center.x, -center.y, center.z, tri_color});
        vertices.push_back(
            {tri_front.x, -tri_front.y, tri_front.z, tri_color});
        // 斜边
        vertices.push_back({tri_up.x, -tri_up.y, tri_up.z, tri_color});
        vertices.push_back(
            {tri_front.x, -tri_front.y, tri_front.z, tri_color});

        if (!vertices.empty() &&
            bgfx::getAvailTransientVertexBuffer(
                static_cast<uint32_t>(vertices.size()), layout) >=
                vertices.size()) {
            bgfx::TransientVertexBuffer tvb;
            bgfx::allocTransientVertexBuffer(
                &tvb, static_cast<uint32_t>(vertices.size()), layout);
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
    // --- Coordinate system sphere for angle config editor ---
    // Renders only the defined meridians/parallels from hair_angle_config,
    // with three-line cross markers at each configured key point.
    if (manager && manager->show_angle_config_window && show_addon_center &&
        mesh_shader.ensureLineProgram()) {

        // Lazy-build BVH if we have angle config entries but no BVH yet.
        // The BVH is normally built when editing the angle config via UI
        // or via the HTTP API, but older sessions or edge cases may leave
        // it null.  We rebuild from the base node's mesh here so that
        // crosshair markers can raycast to the model surface.
        if (!hair_bvh && !hair_angle_config.empty() &&
            addon_base_node_id >= 0) {
            auto base_it = manager->items.find(addon_base_node_id);
            if (base_it != manager->items.end()) {
                auto& base = *base_it->second;
                std::vector<sinriv::kigstudio::voxel::Triangle> tris;
                if (!base.source_triangles.empty()) {
                    tris = base.source_triangles;
                } else if (!base.cached_mesh.empty()) {
                    tris.reserve(base.cached_mesh.size());
                    for (const auto& [tri, _] : base.cached_mesh)
                        tris.push_back(tri);
                }
                if (!tris.empty()) {
                    auto bvh = std::make_unique<
                        sinriv::kigstudio::voxel::triangle_bvh<float>>();
                    for (const auto& tri : tris)
                        bvh->insert(tri);
                    hair_bvh = std::move(bvh);
                    hair_bvh_base_node_id = addon_base_node_id;
                }
            }
        }

        // Build local spherical frame (same math as agent_handlers.h spherical_to_dir)
        vec3f N = hair_north_pole.normalize();
        vec3f F = hair_front_reference.normalize();
        float f_dot_n = F.dot(N);
        vec3f V = F - N * f_dot_n;
        float v_len2 = V.length2();
        if (v_len2 < 1e-10f) {
            vec3f A = (std::abs(N.z) < 0.99f)
                          ? vec3f(0.0f, 0.0f, 1.0f)
                          : vec3f(1.0f, 0.0f, 0.0f);
            V = A - N * A.dot(N);
            v_len2 = V.length2();
        }
        V = V / std::sqrt(v_len2);
        vec3f U = cross(N, V);

        // Convert spherical (theta_deg, phi_deg) -> world-space point on sphere
        auto sphere_point = [&](float theta_deg, float phi_deg, float radius) {
            constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
            float t = theta_deg * kDegToRad;
            float p = phi_deg * kDegToRad;
            float cos_p = std::cos(p), sin_p = std::sin(p);
            float sin_t = std::sin(t), cos_t = std::cos(t);
            vec3f dir = U * (sin_t * cos_p) + N * sin_p + V * (cos_t * cos_p);
            return addon_center_point + dir * radius;
        };

        bgfx::VertexLayout& layout = concave_cone_overlay_layout();
        const uint32_t blue_color = pack_abgr(0.15f, 0.35f, 0.85f, 0.45f);
        const uint32_t cyan_color = pack_abgr(0.0f, 0.85f, 0.85f, 0.9f);
        const uint32_t key_color = pack_abgr(0.0f, 1.0f, 0.6f, 0.85f);
        std::vector<mesh_detail::ColorLineVertex> vertices;

        constexpr int kCircleSegs = 48;
        constexpr int kMeridianSegs = 24;

        // Collect unique theta from X-axis entries (Y=0, control azimuth)
        // and unique phi from Y-axis entries (X=0, control elevation).
        // Diagonal entries are blends and don't produce independent grid lines.
        std::set<float> used_thetas;  // from Y=0 entries (X-axis)
        std::set<float> used_phis;    // from X=0 entries (Y-axis)
        for (const auto& [xy, entry] : hair_angle_config) {
            if (xy.second == 0.0f)   // Y=0 → X-axis → controls theta
                used_thetas.insert(entry.theta);
            if (xy.first == 0.0f)    // X=0 → Y-axis → controls phi
                used_phis.insert(entry.phi);
        }

        // ---- Meridians: only from X-axis (Y=0) entries (effective thetas) ----
        for (float theta : used_thetas) {
            for (int i = 0; i < kMeridianSegs; ++i) {
                float p0 = -90.0f + 180.0f * i / kMeridianSegs;
                float p1 = -90.0f + 180.0f * (i + 1) / kMeridianSegs;
                vec3f q0 = sphere_point(theta, p0, sphere_r);
                vec3f q1 = sphere_point(theta, p1, sphere_r);
                vertices.push_back({q0.x, -q0.y, q0.z, blue_color});
                vertices.push_back({q1.x, -q1.y, q1.z, blue_color});
            }
        }

        // ---- Parallels: only from Y-axis (X=0) entries (effective phis) ----
        for (float phi : used_phis) {
            for (int i = 0; i < kCircleSegs; ++i) {
                float t0 = 360.0f * i / kCircleSegs;
                float t1 = 360.0f * (i + 1) / kCircleSegs;
                vec3f p0 = sphere_point(t0, phi, sphere_r);
                vec3f p1 = sphere_point(t1, phi, sphere_r);
                vertices.push_back({p0.x, -p0.y, p0.z, blue_color});
                vertices.push_back({p1.x, -p1.y, p1.z, blue_color});
            }
        }

        // ---- Key point markers: at axis entries (Y=0 or X=0) ----
        // For each key point, try to raycast from the sphere surface
        // toward the center to find the intersection with the base model.
        // If the ray hits the base model, display the crosshair at the
        // hit point; otherwise fall back to the sphere surface position.
        const float marker_size = sphere_r * 0.03f;
        for (const auto& [xy, entry] : hair_angle_config) {
            if (xy.second != 0.0f && xy.first != 0.0f) continue;  // skip diagonals
            vec3f sphere_pt = sphere_point(entry.theta, entry.phi, sphere_r);

            // Try raycast to base model surface
            vec3f display_pt = sphere_pt;  // default: sphere surface
            if (hair_bvh) {
                vec3f dir = (sphere_pt - addon_center_point).normalize();
                sinriv::kigstudio::ray<float> r;
                r.begin = sphere_pt;          // from sphere surface
                r.end = addon_center_point;    // toward center
                float closest_dist = std::numeric_limits<float>::max();
                vec3f hit_pt;
                bool hit = false;
                hair_bvh->rayTest(r, [&](auto, const vec3f& pos) {
                    float dist = (pos - r.begin).length();
                    if (dist < closest_dist) {
                        closest_dist = dist;
                        hit_pt = pos;
                        hit = true;
                    }
                });
                if (hit) {
                    display_pt = hit_pt;
                }
            }

            vec3f p = display_pt;
            // X-axis line
            vertices.push_back({p.x - marker_size, -p.y, p.z, key_color});
            vertices.push_back({p.x + marker_size, -p.y, p.z, key_color});
            // Y-axis line (note: Y is flipped in rendering)
            vertices.push_back({p.x, -(p.y - marker_size), p.z, key_color});
            vertices.push_back({p.x, -(p.y + marker_size), p.z, key_color});
            // Z-axis line
            vertices.push_back({p.x, -p.y, p.z - marker_size, key_color});
            vertices.push_back({p.x, -p.y, p.z + marker_size, key_color});
        }

        // Submit sphere lines
        if (!vertices.empty()) {
            uint32_t count = static_cast<uint32_t>(vertices.size());
            if (bgfx::getAvailTransientVertexBuffer(count, layout) >= count) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(&tvb, count, layout);
                std::memcpy(tvb.data, vertices.data(),
                            count * sizeof(mesh_detail::ColorLineVertex));
                bgfx::setTransform(model_transform);
                bgfx::setVertexBuffer(0, &tvb);
                bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                               BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS |
                               BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
                bgfx::submit(mesh_shader.overlay_view_id_,
                             mesh_shader.line_program_);
            }
        }

        // ---- Highlight: ray line + meridian + parallel for edited point ----
        int edit_x = angle_config_editing_x;
        if (edit_x != RenderVoxelItem::kAngleConfigSentinel) {
            float hl_theta = angle_config_preview_theta;
            float hl_phi   = angle_config_preview_phi;
            std::vector<mesh_detail::ColorLineVertex> hl;

            // Ray line from center through direction to sphere surface
            {
                // Direction from center to sphere surface point
                vec3f sphere_surf = sphere_point(hl_theta, hl_phi, sphere_r);
                vec3f dir = (sphere_surf - addon_center_point).normalize();
                // Extend slightly beyond the sphere for visibility
                vec3f ray_end = addon_center_point + dir * sphere_r * 1.02f;
                const uint32_t ray_color = pack_abgr(1.0f, 0.9f, 0.2f, 0.9f);
                hl.push_back({addon_center_point.x, -addon_center_point.y,
                              addon_center_point.z, ray_color});
                hl.push_back({ray_end.x, -ray_end.y, ray_end.z, ray_color});
            }

            // Meridian at hl_theta (from phi=-90 to phi=+90)
            for (int i = 0; i < kMeridianSegs; ++i) {
                float p0 = -90.0f + 180.0f * i / kMeridianSegs;
                float p1 = -90.0f + 180.0f * (i + 1) / kMeridianSegs;
                vec3f q0 = sphere_point(hl_theta, p0, sphere_r);
                vec3f q1 = sphere_point(hl_theta, p1, sphere_r);
                hl.push_back({q0.x, -q0.y, q0.z, cyan_color});
                hl.push_back({q1.x, -q1.y, q1.z, cyan_color});
            }

            // Parallel at hl_phi (full circle)
            for (int i = 0; i < kCircleSegs; ++i) {
                float t0 = 360.0f * i / kCircleSegs;
                float t1 = 360.0f * (i + 1) / kCircleSegs;
                vec3f r0 = sphere_point(t0, hl_phi, sphere_r);
                vec3f r1 = sphere_point(t1, hl_phi, sphere_r);
                hl.push_back({r0.x, -r0.y, r0.z, cyan_color});
                hl.push_back({r1.x, -r1.y, r1.z, cyan_color});
            }

            if (!hl.empty()) {
                uint32_t hl_count = static_cast<uint32_t>(hl.size());
                if (bgfx::getAvailTransientVertexBuffer(hl_count, layout) >= hl_count) {
                    bgfx::TransientVertexBuffer tvb;
                    bgfx::allocTransientVertexBuffer(&tvb, hl_count, layout);
                    std::memcpy(tvb.data, hl.data(),
                                hl_count * sizeof(mesh_detail::ColorLineVertex));
                    bgfx::setTransform(model_transform);
                    bgfx::setVertexBuffer(0, &tvb);
                    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                                   BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS |
                                   BGFX_STATE_PT_LINES | BGFX_STATE_MSAA);
                    bgfx::submit(mesh_shader.overlay_view_id_,
                                 mesh_shader.line_program_);
                }
            }
        }
    }

    // 发际线平面蓝色线框三角形预览
    if (manager && manager->show_hairline_plane_window &&
        hairline_plane_enabled && mesh_shader.ensureLineProgram()) {
        bgfx::VertexLayout& layout = concave_cone_overlay_layout();
        const uint32_t blue_color = pack_abgr(0.0f, 0.5f, 1.0f, 1.0f);
        std::vector<mesh_detail::ColorLineVertex> vertices;

        vec3f tri[3];
        if (hairline_plane_use_y) {
            // Y 水平面模式：边长 1 的正三角形，位于 XZ 平面 Y=hairline_plane_y，
            // 中心在原点。
            float s = 0.5f;            // 半边长
            float h = 0.86602540378f;  // sqrt(3)/2
            tri[0] = {-s, hairline_plane_y, -h / 3.0f};
            tri[1] = { s, hairline_plane_y, -h / 3.0f};
            tri[2] = {0.0f, hairline_plane_y, h * 2.0f / 3.0f};
        } else {
            // 三点平面模式：使用实际三点
            tri[0] = hairline_plane_points[0];
            tri[1] = hairline_plane_points[1];
            tri[2] = hairline_plane_points[2];
        }

        // 三条边（注意 Y 轴翻转）
        for (int e = 0; e < 3; ++e) {
            int n = (e + 1) % 3;
            vertices.push_back({tri[e].x, -tri[e].y, tri[e].z, blue_color});
            vertices.push_back({tri[n].x, -tri[n].y, tri[n].z, blue_color});
        }

        // 顶点标记（小十字）
        const float m = 0.15f;
        const uint32_t vert_color = pack_abgr(0.0f, 0.75f, 1.0f, 1.0f);
        for (int v = 0; v < 3; ++v) {
            vertices.push_back({tri[v].x - m, -tri[v].y, tri[v].z, vert_color});
            vertices.push_back({tri[v].x + m, -tri[v].y, tri[v].z, vert_color});
            vertices.push_back({tri[v].x, -(tri[v].y - m), tri[v].z, vert_color});
            vertices.push_back({tri[v].x, -(tri[v].y + m), tri[v].z, vert_color});
            vertices.push_back({tri[v].x, -tri[v].y, tri[v].z - m, vert_color});
            vertices.push_back({tri[v].x, -tri[v].y, tri[v].z + m, vert_color});
        }

        if (!vertices.empty() &&
            bgfx::getAvailTransientVertexBuffer(
                static_cast<uint32_t>(vertices.size()),
                layout) >= vertices.size()) {
            bgfx::TransientVertexBuffer tvb;
            bgfx::allocTransientVertexBuffer(
                &tvb, static_cast<uint32_t>(vertices.size()), layout);
            std::memcpy(tvb.data, vertices.data(),
                        vertices.size() * sizeof(mesh_detail::ColorLineVertex));
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
