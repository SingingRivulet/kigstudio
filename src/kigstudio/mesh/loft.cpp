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

vec3f lerp_vec3(const vec3f& a, const vec3f& b, float t) {
	return a + (b - a) * t;
}

vec2f lerp_vec2(const vec2f& a, const vec2f& b, float t) {
	return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
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
             bool first) {
	vec3f cap_center = {0.0f, 0.0f, 0.0f};
	for (const auto& v : ring.vertices)
		cap_center += v;
	cap_center = cap_center * (1.0f / static_cast<float>(ring.vertices.size()));

	// Use fixed winding that produces halfedges opposite to the side-
	// triangle boundary halfedges, guaranteeing a watertight mesh.
	//
	// Side triangles (fixed winding):
	//   T1 = (a0, b1, b0)  → ring-b boundary edge: b.pn → b.pi
	//   T2 = (a0, a1, b1)  → ring-a boundary edge: a.pi → a.pn
	//
	// First cap (ring a):  (C, a.pn, a.pi)  → edge a.pn → a.pi  ← opposes T2 ✓
	// Last  cap (ring b):  (C, b.pi, b.pn)  → edge b.pi → b.pn  ← opposes T1 ✓
	const int n = static_cast<int>(ring.vertices.size());
	for (int i = 0; i < n; ++i) {
		const int j = (i + 1) % n;
		// Side triangles: T1=(a0,b1,b0) gives ring-b edge b.pn→b.pi
		//                T2=(a0,a1,b1) gives ring-a edge a.pi→a.pn
		// First cap (ring a): (C,a.pn,a.pi) ← opposes T2 ✓
		// Last  cap (ring b): (C,b.pi,b.pn) ← opposes T1 ✓
		if (first)
			mesh.emplace_back(std::make_tuple(cap_center,
			                                  ring.vertices[j],
			                                  ring.vertices[i]));
		else
			mesh.emplace_back(std::make_tuple(cap_center,
			                                  ring.vertices[i],
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

			// Fixed winding — (a0,b1,b0) + (a0,a1,b1).
			// Guarantees watertight halfedge pairings across all quads
			// regardless of section shape, with outward-facing normals.
			mesh.emplace_back(std::make_tuple(a0, b1, b0));
			mesh.emplace_back(std::make_tuple(a0, a1, b1));
		}
	}

	if (options.cap_first) {
		add_cap(mesh, rings.front(), true);
	}
	if (options.cap_last) {
		add_cap(mesh, rings.back(), false);
	}

	// Compute signed volume to ensure outward-facing normals.
	// The fixed winding (a0,b1,b0)+(a0,a1,b1) produces outward normals
	// for right-handed (u,v,tangent) systems. Left-handed systems (e.g.
	// u = tangent × v) produce inward normals → flip all triangles.
	{
		float vol = 0.0f;
		for (const auto& tri : mesh) {
			const auto& a = std::get<0>(tri);
			const auto& b = std::get<1>(tri);
			const auto& c = std::get<2>(tri);
			vol += a.x * (b.y * c.z - b.z * c.y) +
			       a.y * (b.z * c.x - b.x * c.z) +
			       a.z * (b.x * c.y - b.y * c.x);
		}
		if (vol < 0.0f) {
			for (auto& tri : mesh)
				std::swap(std::get<1>(tri), std::get<2>(tri));
		}
	}

	return mesh;
}

}  // namespace sinriv::kigstudio::mesh::loft
