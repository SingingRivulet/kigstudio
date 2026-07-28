// Reproduction test: CGAL geometric boolean difference between two
// overlapping loft hair strands (mirrors the addon "geo split" path in
// RenderVoxelList::RenderVoxelItem::do_segment()).
#include "test_common.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <map>
#include <tuple>
#include <vector>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/measure.h>

#include "kigstudio/cgal/mesh_repair.h"
#include "kigstudio/mesh/loft.h"

using namespace sinriv::kigstudio::mesh::loft;
namespace kcgal = sinriv::kigstudio::cgal;
namespace PMP = CGAL::Polygon_mesh_processing;

namespace {

constexpr float PI = 3.14159265358979323846f;

void expect(bool condition, const char* message) {
	if (!condition) {
		std::cerr << "FAILED: " << message << std::endl;
		std::exit(EXIT_FAILURE);
	}
}

std::vector<vec2f> circle_path(float radius, int n) {
	std::vector<vec2f> path;
	path.reserve(n);
	for (int i = 0; i < n; ++i) {
		const float a = PI * 2.0f * static_cast<float>(i) /
		                static_cast<float>(n);
		path.push_back({std::cos(a) * radius, std::sin(a) * radius});
	}
	return path;
}

// A straight strand along X, tapered, similar to a hair strand.
std::vector<Triangle> build_strand(float y_offset, float z_offset) {
	std::vector<vec3f> guide;
	const int guide_count = 16;
	for (int i = 0; i < guide_count; ++i) {
		const float t = static_cast<float>(i) / (guide_count - 1);
		guide.push_back({t * 6.0f, y_offset, z_offset});
	}

	std::vector<LoftSection> sections;
	for (int i = 0; i < guide_count; ++i) {
		const float t = static_cast<float>(i) / (guide_count - 1);
		const float radius = 1.0f * (1.0f - t) + 0.1f * t;
		LoftSection sec;
		sec.guide_vertex_id = static_cast<size_t>(i);
		sec.axis_u = {0.0f, 1.0f, 0.0f};
		sec.axis_v = {0.0f, 0.0f, 1.0f};
		sec.path = circle_path(radius, 8);
		sections.push_back(std::move(sec));
	}

	LoftOptions options;
	options.cap_first = true;
	options.cap_last = true;
	options.orient_faces = true;
	return build_loft_mesh(guide, sections, options);
}

// An S-curved strand, like test_loft's; both roots share the same start.
std::vector<Triangle> build_curved_strand(float y_amp, float z_offset) {
	std::vector<vec3f> guide;
	const int guide_count = 25;
	for (int i = 0; i < guide_count; ++i) {
		const float t = static_cast<float>(i) / (guide_count - 1);
		const float x = (t - 0.5f) * 14.0f;
		const float y = std::sin((t - 0.5f) * PI * 2.0f) * y_amp;
		const float z = t * 2.0f + z_offset;
		guide.push_back({x, y, z});
	}

	auto tangent_at = [&](size_t i) {
		vec3f t{0, 0, 1};
		if (i == 0)
			t = guide[1] - guide[0];
		else if (i + 1 == guide.size())
			t = guide[i] - guide[i - 1];
		else
			t = guide[i + 1] - guide[i - 1];
		const float len = t.length();
		return len > 1e-8f ? t * (1.0f / len) : t;
	};

	std::vector<LoftSection> sections;
	for (size_t i = 0; i < guide.size(); ++i) {
		const float t = static_cast<float>(i) / (guide.size() - 1);
		const float radius = 1.2f * (1.0f - t) + 0.15f * t;
		const vec3f tangent = tangent_at(i);
		vec3f axis_u(0.0f, 0.0f, 1.0f);
		vec3f axis_v = tangent.cross(axis_u);
		if (axis_v.length2() < 1e-8f)
			axis_v = vec3f(0.0f, 1.0f, 0.0f);
		axis_v = axis_v * (1.0f / axis_v.length());

		LoftSection sec;
		sec.guide_vertex_id = i;
		sec.axis_u = axis_u;
		sec.axis_v = axis_v;
		sec.path = circle_path(radius, 8);
		sections.push_back(std::move(sec));
	}

	LoftOptions options;
	options.cap_first = true;
	options.cap_last = true;
	options.orient_faces = true;
	return build_loft_mesh(guide, sections, options);
}

kcgal::MeshData to_mesh_data(const std::vector<Triangle>& tris) {
	kcgal::MeshData md;
	md.reserve(tris.size());
	for (const auto& t : tris)
		md.emplace_back(t, kcgal::vec3f{});
	return md;
}

// Signed volume of a closed triangle soup (divergence theorem).
double soup_volume(const kcgal::MeshData& md) {
	double vol = 0.0;
	for (const auto& [tri, n] : md) {
		const auto& a = std::get<0>(tri);
		const auto& b = std::get<1>(tri);
		const auto& c = std::get<2>(tri);
		vol += static_cast<double>(a.x) *
		       (static_cast<double>(b.y) * c.z -
		        static_cast<double>(b.z) * c.y);
		vol += static_cast<double>(a.y) *
		       (static_cast<double>(b.z) * c.x -
		        static_cast<double>(b.x) * c.z);
		vol += static_cast<double>(a.z) *
		       (static_cast<double>(b.x) * c.y -
		        static_cast<double>(b.y) * c.x);
	}
	return vol / 6.0;
}

// Point-in-mesh test using CGAL (exact kernel, same as mesh_repair.cpp).
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Surface_mesh = CGAL::Surface_mesh<Point_3>;

Surface_mesh to_surface_mesh(const kcgal::MeshData& mesh) {
	Surface_mesh sm;
	std::map<std::tuple<int64_t, int64_t, int64_t>,
	         Surface_mesh::Vertex_index>
	    vmap;
	auto get_v = [&](const vec3f& p) {
		auto key = std::make_tuple(
		    static_cast<int64_t>(std::round(p.x * 1e6)),
		    static_cast<int64_t>(std::round(p.y * 1e6)),
		    static_cast<int64_t>(std::round(p.z * 1e6)));
		auto it = vmap.find(key);
		if (it != vmap.end()) return it->second;
		auto vi = sm.add_vertex(Point_3(p.x, p.y, p.z));
		vmap.emplace(key, vi);
		return vi;
	};
	for (const auto& [tri, n] : mesh) {
		auto va = get_v(std::get<0>(tri));
		auto vb = get_v(std::get<1>(tri));
		auto vc = get_v(std::get<2>(tri));
		if (va == vb || vb == vc || vc == va) continue;
		sm.add_face(va, vb, vc);
	}
	return sm;
}

bool is_inside(const kcgal::MeshData& md, float x, float y, float z) {
	Surface_mesh sm = to_surface_mesh(md);
	CGAL::Side_of_triangle_mesh<Surface_mesh, Kernel> inside(sm);
	return inside(Point_3(x, y, z)) == CGAL::ON_BOUNDED_SIDE;
}

}  // namespace

// ===========================================================================
// Replicate build_hair_strand_mesh() from ui/render_voxel_render.cpp for the
// exact strand data of the user's broken project (测试发束/project.json).
// ===========================================================================
namespace {

vec3f bezier_eval(const vec3f& p0, const vec3f& p1, const vec3f& p2,
                  const vec3f& p3, float t) {
	const float u = 1.0f - t;
	const float u2 = u * u;
	const float t2 = t * t;
	return p0 * (u2 * u) + p1 * (3.0f * u2 * t) + p2 * (3.0f * u * t2) +
	       p3 * (t2 * t);
}

std::vector<vec3f> sample_bezier_guide_curve(
    const std::vector<vec3f>& guide_points, int samples_per_segment = 32) {
	if (guide_points.size() < 2) return guide_points;
	std::vector<vec3f> result;
	const size_t n = guide_points.size();
	for (size_t i = 0; i + 1 < n; ++i) {
		vec3f p0 = guide_points[i];
		vec3f p3 = guide_points[i + 1];
		vec3f p1;
		if (i == 0)
			p1 = p0 + (p3 - p0) * (1.0f / 3.0f);
		else
			p1 = p0 + (p3 - guide_points[i - 1]) * (1.0f / 6.0f);
		vec3f p2;
		if (i + 2 >= n)
			p2 = p3 - (p3 - p0) * (1.0f / 3.0f);
		else
			p2 = p3 - (guide_points[i + 2] - p0) * (1.0f / 6.0f);
		for (int s = 0; s < samples_per_segment; ++s) {
			float t = static_cast<float>(s) /
			          static_cast<float>(samples_per_segment);
			result.push_back(bezier_eval(p0, p1, p2, p3, t));
		}
	}
	result.push_back(guide_points.back());
	return result;
}

struct WidthPoint {
	float curve_id = 0.0f;
	float scale = 1.0f;
	vec3f direction{0, 1, 0};
};

vec3f safe_normalize(const vec3f& v, const vec3f& fallback) {
	const float len = v.length();
	return (len < 1e-8f) ? fallback : v * (1.0f / len);
}

constexpr float kMinTipWidth = 0.02f;

std::vector<Triangle> build_user_strand(const std::vector<vec3f>& guide_points,
                                        std::vector<WidthPoint> width_points) {
	static const std::vector<vec2f> kDefaultSection = {
	    {-0.5f, -0.5f}, {0.5f, -0.5f}, {0.5f, 0.5f}, {-0.5f, 0.5f}};

	auto sampled = sample_bezier_guide_curve(guide_points, 32);
	std::vector<vec3f> guide_curve = sampled;

	std::sort(width_points.begin(), width_points.end(),
	          [](const WidthPoint& a, const WidthPoint& b) {
		          return a.curve_id < b.curve_id;
	          });
	for (size_t wi = 1; wi < width_points.size(); ++wi) {
		if (width_points[wi - 1].direction.dot(width_points[wi].direction) <
		    0.0f) {
			auto& d = width_points[wi].direction;
			d = {-d.x, -d.y, -d.z};
		}
	}

	const int M = static_cast<int>(guide_curve.size());
	const int N = static_cast<int>(guide_points.size());

	std::vector<float> scales(M);
	std::vector<vec3f> directions(M);

	auto catmull_rom = [](float p0, float p1, float p2, float p3, float t) {
		float t2 = t * t;
		float t3 = t2 * t;
		return 0.5f * ((2.0f * p1) + (-p0 + p2) * t +
		               (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
		               (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
	};

	for (int i = 0; i < M; ++i) {
		float curve_id =
		    static_cast<float>(i) / static_cast<float>(M - 1) *
		    static_cast<float>(N - 1);
		const WidthPoint* wp_a = nullptr;
		const WidthPoint* wp_b = nullptr;
		for (const auto& wp : width_points) {
			if (wp.curve_id <= curve_id + 1e-6f) wp_a = &wp;
			if (wp.curve_id >= curve_id - 1e-6f && !wp_b) wp_b = &wp;
		}
		if (wp_a && wp_b) {
			if (wp_a == wp_b) {
				scales[i] = wp_a->scale;
				directions[i] = wp_a->direction;
			} else {
				float t = (curve_id - wp_a->curve_id) /
				          (wp_b->curve_id - wp_a->curve_id + 1e-10f);
				t = std::clamp(t, 0.0f, 1.0f);
				int idx_a = static_cast<int>(wp_a - width_points.data());
				int idx_b = idx_a + 1;
				int wp_count = static_cast<int>(width_points.size());
				auto mirror_before = [](float ref, float next) {
					return ref - (next - ref);
				};
				auto mirror_after = [](float ref, float prev) {
					return ref + (ref - prev);
				};
				float p0_s = (idx_a > 0)
				                 ? width_points[idx_a - 1].scale
				                 : mirror_before(wp_a->scale, wp_b->scale);
				float p3_s = (idx_b < wp_count - 1)
				                 ? width_points[idx_b + 1].scale
				                 : mirror_after(wp_b->scale, wp_a->scale);
				scales[i] =
				    catmull_rom(p0_s, wp_a->scale, wp_b->scale, p3_s, t);
				auto mirror_vec_before = [](const vec3f& ref,
				                            const vec3f& next) {
					return ref - (next - ref);
				};
				auto mirror_vec_after = [](const vec3f& ref,
				                           const vec3f& prev) {
					return ref + (ref - prev);
				};
				vec3f p0_d = (idx_a > 0)
				                 ? width_points[idx_a - 1].direction
				                 : mirror_vec_before(wp_a->direction,
				                                     wp_b->direction);
				vec3f p3_d = (idx_b < wp_count - 1)
				                 ? width_points[idx_b + 1].direction
				                 : mirror_vec_after(wp_b->direction,
				                                    wp_a->direction);
				vec3f dir{catmull_rom(p0_d.x, wp_a->direction.x,
				                      wp_b->direction.x, p3_d.x, t),
				          catmull_rom(p0_d.y, wp_a->direction.y,
				                      wp_b->direction.y, p3_d.y, t),
				          catmull_rom(p0_d.z, wp_a->direction.z,
				                      wp_b->direction.z, p3_d.z, t)};
				directions[i] = safe_normalize(dir, {0, 1, 0});
			}
		} else if (wp_a) {
			float tip_t = 0.0f;
			float last_id = width_points.back().curve_id;
			float range = static_cast<float>(N - 1) - last_id;
			if (range > 1e-4f)
				tip_t = std::clamp((curve_id - last_id) / range, 0.0f, 1.0f);
			scales[i] = wp_a->scale * (1.0f - tip_t) + kMinTipWidth * tip_t;
			directions[i] = safe_normalize(wp_a->direction, {0, 1, 0});
		} else if (wp_b) {
			float tip_t = 0.0f;
			float first_id = width_points.front().curve_id;
			if (first_id > 1e-4f)
				tip_t = std::clamp(1.0f - curve_id / first_id, 0.0f, 1.0f);
			scales[i] = wp_b->scale * (1.0f - tip_t) + kMinTipWidth * tip_t;
			directions[i] = safe_normalize(wp_b->direction, {0, 1, 0});
		} else {
			scales[i] = kMinTipWidth;
			directions[i] = {0, 1, 0};
		}
		if (scales[i] < kMinTipWidth) scales[i] = kMinTipWidth;
	}

	int first_section_idx = 0;
	int last_section_idx = M - 1;
	auto curve_id_to_sample = [&](float curve_id) {
		int idx = static_cast<int>(std::round(
		    curve_id * static_cast<float>(M - 1) /
		    static_cast<float>(N - 1)));
		return std::clamp(idx, 0, M - 1);
	};
	first_section_idx = curve_id_to_sample(width_points.front().curve_id);
	last_section_idx = curve_id_to_sample(width_points.back().curve_id);
	if (first_section_idx > last_section_idx)
		std::swap(first_section_idx, last_section_idx);
	if (last_section_idx - first_section_idx < 1) {
		last_section_idx = std::min(first_section_idx + 1, M - 1);
		first_section_idx = std::max(last_section_idx - 1, 0);
	}

	std::vector<LoftSection> sections;
	for (int i = first_section_idx; i <= last_section_idx; ++i) {
		float scale = scales[i];
		vec3f tangent;
		{
			int n = M;
			if (i <= 0)
				tangent = safe_normalize(guide_curve[1] - guide_curve[0],
				                         {0, 0, 1});
			else if (i >= n - 1)
				tangent = safe_normalize(guide_curve[n - 1] -
				                             guide_curve[n - 2],
				                         {0, 0, 1});
			else
				tangent = safe_normalize(guide_curve[i + 1] -
				                             guide_curve[i - 1],
				                         {0, 0, 1});
		}
		float dot_vt = directions[i].dot(tangent);
		vec3f axis_v_raw = directions[i] - tangent * dot_vt;
		vec3f axis_v = safe_normalize(axis_v_raw, {0, 1, 0});
		vec3f axis_u = tangent.cross(axis_v);
		axis_u = safe_normalize(axis_u, {1, 0, 0});
		axis_v = axis_u.cross(tangent);
		axis_v = safe_normalize(axis_v, {0, 1, 0});

		std::vector<vec2f> path;
		for (const auto& v : kDefaultSection)
			path.push_back({v.x * scale, v.y * scale});

		LoftSection sec;
		sec.guide_vertex_id = static_cast<size_t>(i);
		sec.axis_u = axis_u;
		sec.axis_v = axis_v;
		sec.path = std::move(path);
		sections.push_back(std::move(sec));
	}

	LoftOptions opts;
	opts.cap_first = true;
	opts.cap_last = true;
	opts.orient_faces = true;
	return build_loft_mesh(guide_curve, sections, opts);
}

void test_user_project_strands() {
	// 测试发束/project.json — Strand 1
	std::vector<vec3f> guide1 = {
	    {0.65498411655426025f, -4.8227801322937f, -8.09713554382324f},
	    {1.6156212091445923f, -6.8678231239318848f, -6.3845677375793457f},
	    {1.054129958152771f, -9.0931272506713867f, -2.5109031200408936f},
	    {-0.54123234748840332f, -9.72588539123535f, -0.737947106361389f}};
	std::vector<WidthPoint> wp1 = {
	    {0.21875f, 1.0483307838439941f,
	     {-0.90517562627792358f, -0.40647503733634949f, 0.12423828989267349f}},
	    {0.78125f, 1.5696210861206055f,
	     {-0.969917356967926f, -0.19115851819515228f, 0.150727242231369f}},
	    {1.40625f, 1.9884053468704224f,
	     {-0.98135673999786377f, -0.064100898802280426f,
	      -0.18119068443775177f}},
	    {2.53125f, 2.4231464862823486f,
	     {-0.65985792875289917f, 0.24603506922721863f, -0.709967851638794f}},
	    {3.0f, 1.5823965072631836f,
	     {-0.820413112640381f, 0.328847199678421f, -0.46774116158485413f}}};

	// Strand 2
	std::vector<vec3f> guide2 = {
	    {-0.8343624472618103f, -5.09644603729248f, -8.24989700317383f},
	    {-0.5432390570640564f, -5.9937429428100586f, -7.4449334144592285f},
	    {-0.78664529323577881f, -7.3086719512939453f, -5.94278621673584f},
	    {-1.5040532350540161f, -8.13704013824463f, -4.9636297225952148f},
	    {-2.8551065921783447f, -8.7553033828735352f, -2.3809115886688232f},
	    {-3.605219841003418f, -8.6959056854248047f, -0.99652403593063354f}};
	std::vector<WidthPoint> wp2 = {
	    {0.3125f, 1.0821206569671631f,
	     {-0.9406965970993042f, -0.25382137298583984f, 0.22508831322193146f}},
	    {1.84375f, 1.7200000286102295f,
	     {-0.980253279209137f, 0.15751715004444122f, -0.11954854428768158f}},
	    {3.1875f, 1.9199999570846558f,
	     {-0.88910925388336182f, 0.20400747656822205f, -0.4097142219543457f}},
	    {3.78125f, 1.2693217992782593f,
	     {-0.86016374826431274f, 0.31193175911903381f, -0.403505802154541f}}};

	auto md1 = to_mesh_data(build_user_strand(guide1, wp1));
	auto md2 = to_mesh_data(build_user_strand(guide2, wp2));
	std::cout << "user strand1: " << md1.size() << " tris, volume "
	          << soup_volume(md1) << "\n";
	std::cout << "user strand2: " << md2.size() << " tris, volume "
	          << soup_volume(md2) << "\n";
	expect(!md1.empty() && !md2.empty(), "user strands should be non-empty");

	// 找出一个同时位于两根发束内部的点（取 strand2 的一个顶点，
	// 若其在 strand1 内部即为重叠证据）
	float px = 0, py = 0, pz = 0;
	bool found_overlap = false;
	for (const auto& [tri, n] : md2) {
		const auto& a = std::get<0>(tri);
		if (is_inside(md1, a.x, a.y, a.z)) {
			px = a.x;
			py = a.y;
			pz = a.z;
			found_overlap = true;
			break;
		}
	}
	// 顶点判据不够时做三角形级相交检测
	if (!found_overlap) {
		auto sm1 = to_surface_mesh(md1);
		auto sm2 = to_surface_mesh(md2);
		const bool tri_intersect =
		    PMP::do_intersect(sm1, sm2);
		std::cout << "do_intersect(strand1, strand2): " << tri_intersect
		          << "\n";
		if (tri_intersect) {
			// 用 strand1 内部取样点验证它是否在 strand2 内部
			for (const auto& [tri, n] : md1) {
				const auto& a = std::get<0>(tri);
				const auto& b = std::get<1>(tri);
				const auto& c = std::get<2>(tri);
				vec3f mid{(a.x + b.x + c.x) / 3.0f,
				          (a.y + b.y + c.y) / 3.0f,
				          (a.z + b.z + c.z) / 3.0f};
				if (is_inside(md2, mid.x, mid.y, mid.z)) {
					px = mid.x;
					py = mid.y;
					pz = mid.z;
					found_overlap = true;
					break;
				}
			}
		}
	}
	std::cout << "overlap found: " << found_overlap << " at (" << px << ", "
	          << py << ", " << pz << ")\n";

	// 默认方形截面下发束不相交（用户实际截面数据未存入工程）。
	// 将 strand2 平移到与 strand1 强制重叠，验证真实发束几何上的布尔。
	{
		auto md2_shifted = md2;
		for (auto& [tri, n] : md2_shifted) {
			for (auto* v : {&std::get<0>(tri), &std::get<1>(tri),
			                &std::get<2>(tri)}) {
				v->x += 1.0f;
				v->z += 1.0f;
			}
		}
		auto sm1 = to_surface_mesh(md1);
		auto sm2s = to_surface_mesh(md2_shifted);
		std::cout << "do_intersect(strand1, strand2 shifted): "
		          << PMP::do_intersect(sm1, sm2s) << "\n";
		auto diff2 = kcgal::mesh_difference(md2_shifted, md1);
		if (diff2.empty()) {
			std::cerr << "FAILED: mesh_difference on shifted user strands "
			             "returned empty (boolean failed -> caller keeps "
			             "un-clipped mesh)\n";
			std::exit(EXIT_FAILURE);
		}
		std::cout << "shifted strand2-strand1: " << diff2.size()
		          << " tris, volume " << soup_volume(diff2)
		          << " (original volume " << soup_volume(md2_shifted)
		          << ")\n";
	}
	if (!found_overlap) {
		std::cout << "user strands do not overlap; skipping difference "
		             "check\n";
		return;
	}

	// do_segment 中 i=1, j=0: strand2 - strand1
	auto diff = kcgal::mesh_difference(md2, md1);
	if (diff.empty()) {
		std::cerr << "FAILED: mesh_difference on user strands returned "
		             "empty (boolean failed -> caller keeps un-clipped "
		             "mesh)\n";
		std::exit(EXIT_FAILURE);
	}
	std::cout << "user strand2-strand1: " << diff.size() << " tris, volume "
	          << soup_volume(diff) << "\n";
	expect(!is_inside(diff, px, py, pz),
	       "overlap point should be removed from strand2-strand1");
}

}  // namespace

int main() {
	setup_test_environment();

	// Strand A at y=0, strand B at y=1.2: two tubes of radius ~1.0
	// overlapping along their whole length.
	auto tris_a = build_strand(0.0f, 0.0f);
	auto tris_b = build_strand(1.2f, 0.0f);
	expect(!tris_a.empty() && !tris_b.empty(), "strands should be non-empty");

	auto md_a = to_mesh_data(tris_a);
	auto md_b = to_mesh_data(tris_b);

	std::cout << "A: " << tris_a.size() << " tris, volume "
	          << soup_volume(md_a) << "\n";
	std::cout << "B: " << tris_b.size() << " tris, volume "
	          << soup_volume(md_b) << "\n";

	auto diff = kcgal::mesh_difference(md_a, md_b);
	if (diff.empty()) {
		std::cerr << "FAILED: mesh_difference returned empty "
		             "(boolean failed, caller falls back to the "
		             "un-clipped mesh)\n";
		return EXIT_FAILURE;
	}

	std::cout << "A-B: " << diff.size() << " tris, volume "
	          << soup_volume(diff) << "\n";

	// Decisive containment checks:
	// - a point in the overlap zone (inside both A and B) must NOT be
	//   inside A-B;
	// - a point inside A but far from B must stay inside A-B.
	expect(is_inside(md_a, 0.5f, 0.6f, 0.0f), "p1 should be inside A");
	expect(is_inside(md_b, 0.5f, 0.6f, 0.0f), "p1 should be inside B");
	expect(!is_inside(diff, 0.5f, 0.6f, 0.0f),
	       "overlap point should be removed from A-B");
	expect(is_inside(diff, 0.5f, -0.5f, 0.0f),
	       "non-overlap point of A should remain in A-B");

	// Curved strands with coincident roots (realistic hair setup).
	auto cur_a = to_mesh_data(build_curved_strand(3.0f, 0.0f));
	auto cur_b = to_mesh_data(build_curved_strand(-3.0f, 0.3f));
	auto cur_diff = kcgal::mesh_difference(cur_a, cur_b);
	if (cur_diff.empty()) {
		std::cerr << "FAILED: mesh_difference on curved strands returned "
		             "empty\n";
		return EXIT_FAILURE;
	}
	std::cout << "curved A-B: " << cur_diff.size() << " tris, volume "
	          << soup_volume(cur_diff) << " (A volume "
	          << soup_volume(cur_a) << ")\n";
	// Roots coincide at x=-7: a point between the two root centers must be
	// removed from A-B.
	expect(!is_inside(cur_diff, -6.5f, 0.0f, 0.4f),
	       "curved root overlap point should be removed from A-B");

	test_user_project_strands();

	std::cout << "test_cgal_boolean passed\n";
	return EXIT_SUCCESS;
}
