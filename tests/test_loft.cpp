#include "test_common.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <tuple>
#include <vector>

#include "kigstudio/mesh/loft.h"

using namespace sinriv::kigstudio::mesh::loft;

namespace {

constexpr float PI = 3.14159265358979323846f;

void expect(bool condition, const char* message) {
	if (!condition) {
		std::cerr << "FAILED: " << message << std::endl;
		std::exit(EXIT_FAILURE);
	}
}

bool all_vertices_finite(const std::vector<Triangle>& mesh) {
	for (const auto& tri : mesh) {
		for (const auto& v :
		     {std::get<0>(tri), std::get<1>(tri), std::get<2>(tri)}) {
			if (!std::isfinite(v.x) || !std::isfinite(v.y) ||
			    !std::isfinite(v.z))
				return false;
		}
	}
	return true;
}

struct VecCmp {
	bool operator()(const vec3f& a, const vec3f& b) const {
		if (a.x != b.x) return a.x < b.x;
		if (a.y != b.y) return a.y < b.y;
		return a.z < b.z;
	}
};

bool is_closed_triangle_mesh(const std::vector<Triangle>& mesh) {
	using Edge = std::pair<vec3f, vec3f>;
	struct EdgeCmp {
		bool operator()(const Edge& a, const Edge& b) const {
			VecCmp cmp;
			if (cmp(a.first, b.first)) return true;
			if (cmp(b.first, a.first)) return false;
			return cmp(a.second, b.second);
		}
	};
	auto edge = [](vec3f a, vec3f b) {
		VecCmp cmp;
		if (cmp(b, a))
			std::swap(a, b);
		return Edge{a, b};
	};

	std::map<Edge, int, EdgeCmp> edge_count;
	for (const auto& tri : mesh) {
		const vec3f a = std::get<0>(tri);
		const vec3f b = std::get<1>(tri);
		const vec3f c = std::get<2>(tri);
		edge_count[edge(a, b)]++;
		edge_count[edge(b, c)]++;
		edge_count[edge(c, a)]++;
	}
	for (const auto& [e, count] : edge_count)
		if (count != 2)
			return false;
	return !edge_count.empty();
}

std::vector<vec2f> flattened_hex(float radius, float flatten) {
	std::vector<vec2f> path;
	path.reserve(6);
	for (int i = 0; i < 6; ++i) {
		const float a = PI * 2.0f * static_cast<float>(i) / 6.0f;
		path.push_back({std::cos(a) * radius,
		                std::sin(a) * radius * flatten});
	}
	return path;
}

vec3f normalized(const vec3f& v) {
	const float len = v.length();
	return len > 1e-8f ? v * (1.0f / len) : vec3f(1.0f, 0.0f, 0.0f);
}

vec3f guide_tangent(const std::vector<vec3f>& guide, size_t i) {
	if (i == 0)
		return normalized(guide[1] - guide[0]);
	if (i + 1 == guide.size())
		return normalized(guide[i] - guide[i - 1]);
	return normalized(guide[i + 1] - guide[i - 1]);
}

LoftSection make_section(const std::vector<vec3f>& guide,
                         size_t guide_id,
                         float radius,
                         float flatten) {
	const vec3f tangent = guide_tangent(guide, guide_id);
	const vec3f axis_u(0.0f, 0.0f, 1.0f);
	vec3f axis_v = tangent.cross(axis_u);
	if (axis_v.length2() < 1e-8f)
		axis_v = vec3f(0.0f, 1.0f, 0.0f);
	axis_v = normalized(axis_v);

	LoftSection section;
	section.guide_vertex_id = guide_id;
	section.axis_u = axis_u;
	section.axis_v = axis_v;
	section.path = flattened_hex(radius, flatten);
	return section;
}

void write_ascii_stl(const std::vector<Triangle>& mesh,
                     const std::string& path) {
	std::ofstream out(path);
	expect(out.good(), "should open loft STL output file");
	out << "solid loft_s_curve_hex\n";
	for (const auto& tri : mesh) {
		const vec3f a = std::get<0>(tri);
		const vec3f b = std::get<1>(tri);
		const vec3f c = std::get<2>(tri);
		vec3f n = (b - a).cross(c - a);
		const float len = n.length();
		if (len > 1e-8f)
			n = n * (1.0f / len);
		out << "  facet normal " << n.x << " " << n.y << " " << n.z << "\n";
		out << "    outer loop\n";
		out << "      vertex " << a.x << " " << a.y << " " << a.z << "\n";
		out << "      vertex " << b.x << " " << b.y << " " << b.z << "\n";
		out << "      vertex " << c.x << " " << c.y << " " << c.z << "\n";
		out << "    endloop\n";
		out << "  endfacet\n";
	}
	out << "endsolid loft_s_curve_hex\n";
}

void test_s_curve_flattening_hex_loft() {
	std::vector<vec3f> guide;
	const int guide_count = 25;
	guide.reserve(guide_count);
	for (int i = 0; i < guide_count; ++i) {
		const float t = static_cast<float>(i) / static_cast<float>(guide_count - 1);
		const float x = (t - 0.5f) * 14.0f;
		const float y = std::sin((t - 0.5f) * PI * 2.0f) * 3.0f;
		const float z = t * 2.0f;
		guide.push_back({x, y, z});
	}

	std::vector<LoftSection> sections;
	sections.push_back(make_section(guide, 0, 1.2f, 1.0f));
	sections.push_back(make_section(guide, 8, 0.85f, 0.75f));
	sections.push_back(make_section(guide, 16, 0.48f, 0.45f));
	sections.push_back(make_section(guide, 24, 0.18f, 0.22f));

	LoftOptions options;
	options.cap_first = true;
	options.cap_last = true;
	options.orient_faces = true;

	const auto mesh = build_loft_mesh(guide, sections, options);
	expect(mesh.size() == 300,
	       "S-curve hex loft should have expected triangle count");
	expect(all_vertices_finite(mesh),
	       "S-curve hex loft vertices should be finite");
	expect(is_closed_triangle_mesh(mesh),
	       "S-curve hex loft should be closed");

	write_ascii_stl(mesh, "test_loft_s_curve_hex.stl");
}

}  // namespace

int main() {
	setup_test_environment();
	test_s_curve_flattening_hex_loft();
	std::cout << "test_loft passed; wrote test_loft_s_curve_hex.stl"
	          << std::endl;
	return 0;
}
