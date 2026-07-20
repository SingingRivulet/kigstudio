#include "loft.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sinriv::kigstudio::mesh::loft {
namespace {

constexpr float EPS = 1e-8f;

vec3f normalize_or_throw(const vec3f& v, const char* name) {
	const float len = v.length();
	if (len < EPS)
		throw std::invalid_argument(std::string("Invalid loft ") + name);
	return v * (1.0f / len);
}

vec3f safe_normalize(const vec3f& v, const vec3f& fallback) {
	const float len = v.length();
	return (len < EPS) ? fallback : v * (1.0f / len);
}

vec3f tangent_at(const std::vector<vec3f>& guide, int index) {
	const int n = static_cast<int>(guide.size());
	if (n < 2)
		return {0.0f, 0.0f, 1.0f};
	if (index <= 0)
		return safe_normalize(guide[1] - guide[0], {0.0f, 0.0f, 1.0f});
	if (index >= n - 1)
		return safe_normalize(guide[n - 1] - guide[n - 2],
		                      {0.0f, 0.0f, 1.0f});
	return safe_normalize(guide[index + 1] - guide[index - 1],
	                      {0.0f, 0.0f, 1.0f});
}

vec3f lerp_vec3(const vec3f& a, const vec3f& b, float t) {
	return a + (b - a) * t;
}

vec2f lerp_vec2(const vec2f& a, const vec2f& b, float t) {
	return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}

float path_area(const std::vector<vec2f>& path) {
	float area = 0.0f;
	const int n = static_cast<int>(path.size());
	for (int i = 0; i < n; ++i) {
		const int j = (i + 1) % n;
		area += path[i].x * path[j].y - path[j].x * path[i].y;
	}
	return area * 0.5f;
}

Triangle make_oriented_triangle(vec3f a, vec3f b, vec3f c,
                                const vec3f& desired_normal) {
	const vec3f n = (b - a).cross(c - a);
	if (n.dot(desired_normal) < 0.0f)
		std::swap(b, c);
	return std::make_tuple(a, b, c);
}

struct PreparedSection {
	size_t id = 0;
	vec3f u;
	vec3f v;
	std::vector<vec2f> path;
};

struct Ring {
	size_t guide_id = 0;
	vec3f center;
	vec3f u;
	vec3f v;
	std::vector<vec3f> vertices;
};

std::vector<float> cumulative_lengths(const std::vector<vec3f>& guide) {
	std::vector<float> lengths(guide.size(), 0.0f);
	for (size_t i = 1; i < guide.size(); ++i)
		lengths[i] = lengths[i - 1] + (guide[i] - guide[i - 1]).length();
	return lengths;
}

void validate_inputs(const std::vector<vec3f>& guide,
                     const std::vector<LoftSection>& sections) {
	if (guide.size() < 2)
		throw std::invalid_argument("Loft guide_curve needs at least 2 points");
	if (sections.size() < 2)
		throw std::invalid_argument("Loft needs at least 2 sections");
	if (sections.front().path.size() < 3)
		throw std::invalid_argument("Loft section path needs at least 3 points");

	const size_t count = sections.front().path.size();
	for (const auto& section : sections) {
		if (section.guide_vertex_id >= guide.size())
			throw std::invalid_argument("Loft section guide_vertex_id is out of range");
		if (section.path.size() != count)
			throw std::invalid_argument("Loft sections must have matching path sizes");
	}
}

std::vector<PreparedSection> prepare_sections(
    const std::vector<LoftSection>& sections) {
	std::vector<PreparedSection> prepared;
	prepared.reserve(sections.size());
	for (const auto& section : sections) {
		PreparedSection out;
		out.id = section.guide_vertex_id;
		out.u = normalize_or_throw(section.axis_u, "axis_u");
		out.v = section.axis_v - out.u * out.u.dot(section.axis_v);
		out.v = normalize_or_throw(out.v, "axis_v");
		out.path = section.path;
		prepared.push_back(std::move(out));
	}

	std::sort(prepared.begin(), prepared.end(),
	          [](const PreparedSection& a, const PreparedSection& b) {
		          return a.id < b.id;
	          });
	for (size_t i = 1; i < prepared.size(); ++i)
		if (prepared[i - 1].id == prepared[i].id)
			throw std::invalid_argument("Loft sections must use unique guide ids");
	return prepared;
}

Ring make_ring(const std::vector<vec3f>& guide,
               const std::vector<float>& arc_lengths,
               const PreparedSection& a,
               const PreparedSection& b,
               size_t guide_id) {
	const float span = arc_lengths[b.id] - arc_lengths[a.id];
	const float t = (span <= EPS) ? 0.0f :
	    (arc_lengths[guide_id] - arc_lengths[a.id]) / span;

	Ring ring;
	ring.guide_id = guide_id;
	ring.center = guide[guide_id];
	ring.u = safe_normalize(lerp_vec3(a.u, b.u, t), a.u);
	ring.v = lerp_vec3(a.v, b.v, t);
	ring.v = ring.v - ring.u * ring.u.dot(ring.v);
	ring.v = safe_normalize(ring.v, a.v);

	const size_t count = a.path.size();
	ring.vertices.reserve(count);
	for (size_t i = 0; i < count; ++i) {
		const vec2f p = lerp_vec2(a.path[i], b.path[i], t);
		ring.vertices.push_back(ring.center + ring.u * p.x + ring.v * p.y);
	}
	return ring;
}

void add_cap(std::vector<Triangle>& mesh,
             const Ring& ring,
             const std::vector<vec2f>& path,
             const vec3f& tangent,
             bool first,
             bool orient_faces) {
	vec3f cap_center = {0.0f, 0.0f, 0.0f};
	for (const auto& v : ring.vertices)
		cap_center += v;
	cap_center = cap_center * (1.0f / static_cast<float>(ring.vertices.size()));

	vec3f desired = first ? -tangent : tangent;
	if (path_area(path) < 0.0f)
		desired = -desired;

	const int n = static_cast<int>(ring.vertices.size());
	for (int i = 0; i < n; ++i) {
		const int j = (i + 1) % n;
		if (orient_faces)
			mesh.push_back(make_oriented_triangle(cap_center, ring.vertices[i],
			                                      ring.vertices[j], desired));
		else
			mesh.emplace_back(std::make_tuple(cap_center, ring.vertices[i],
			                                  ring.vertices[j]));
	}
}

}  // namespace

std::vector<Triangle> build_loft_mesh(
    const std::vector<vec3f>& guide_curve,
    const std::vector<LoftSection>& sections,
    const LoftOptions& options,
    const std::function<bool()>& should_continue) {
	validate_inputs(guide_curve, sections);

	auto prepared = prepare_sections(sections);
	const auto arc_lengths = cumulative_lengths(guide_curve);
	const size_t point_count = prepared.front().path.size();

	std::vector<Ring> rings;
	for (size_t si = 0; si + 1 < prepared.size(); ++si) {
		const auto& a = prepared[si];
		const auto& b = prepared[si + 1];
		if (a.id >= b.id)
			throw std::invalid_argument("Loft sections must span increasing guide ids");
		for (size_t gid = a.id; gid <= b.id; ++gid) {
			if (should_continue && !should_continue())
				return {};
			if (!rings.empty() && rings.back().guide_id == gid)
				continue;
			rings.push_back(make_ring(guide_curve, arc_lengths, a, b, gid));
		}
	}

	std::vector<Triangle> mesh;
	if (rings.size() < 2)
		return mesh;
	mesh.reserve((rings.size() - 1) * point_count * 2 + point_count * 2);

	for (size_t ri = 0; ri + 1 < rings.size(); ++ri) {
		if (should_continue && !should_continue())
			return {};
		const Ring& a = rings[ri];
		const Ring& b = rings[ri + 1];
		for (size_t pi = 0; pi < point_count; ++pi) {
			const size_t pn = (pi + 1) % point_count;
			const vec3f a0 = a.vertices[pi];
			const vec3f a1 = a.vertices[pn];
			const vec3f b0 = b.vertices[pi];
			const vec3f b1 = b.vertices[pn];

			const vec3f radial =
			    ((a0 + a1 + b0 + b1) * 0.25f) -
			    ((a.center + b.center) * 0.5f);
			if (options.orient_faces) {
				mesh.push_back(make_oriented_triangle(a0, b0, b1, radial));
				mesh.push_back(make_oriented_triangle(a0, b1, a1, radial));
			} else {
				mesh.emplace_back(std::make_tuple(a0, b0, b1));
				mesh.emplace_back(std::make_tuple(a0, b1, a1));
			}
		}
	}

	if (options.cap_first) {
		const Ring& ring = rings.front();
		add_cap(mesh, ring, prepared.front().path,
		        tangent_at(guide_curve, static_cast<int>(ring.guide_id)), true,
		        options.orient_faces);
	}
	if (options.cap_last) {
		const Ring& ring = rings.back();
		add_cap(mesh, ring, prepared.back().path,
		        tangent_at(guide_curve, static_cast<int>(ring.guide_id)), false,
		        options.orient_faces);
	}

	return mesh;
}

}  // namespace sinriv::kigstudio::mesh::loft
