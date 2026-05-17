#include "kigstudio/cgal/poisson_utils.h"

#include <cstdlib>
#include <map>
#include <tuple>
#include <cmath>
#include <limits>

namespace sinriv::kigstudio::cgal {

std::vector<std::pair<vec3f, vec3f>>
samplePointCloudFromTriangles(const std::vector<Triangle>& triangles) {
    if (triangles.empty()) {
        return {};
    }

    // Seed rand for reproducible jitter
    std::srand(42);

    // 1. Filter degenerate triangles and compute face normals
    struct Face {
        vec3f a, b, c;
        vec3f normal;
        float area;
        vec3f centroid;
    };
    std::vector<Face> faces;
    faces.reserve(triangles.size());

    vec3f bbox_min(
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());
    vec3f bbox_max(
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max());

    double total_volume = 0.0;
    for (const auto& tri : triangles) {
        auto a = std::get<0>(tri);
        auto b = std::get<1>(tri);
        auto c = std::get<2>(tri);

        // Update bbox
        for (const auto& p : {a, b, c}) {
            bbox_min.x = std::min(bbox_min.x, p.x);
            bbox_min.y = std::min(bbox_min.y, p.y);
            bbox_min.z = std::min(bbox_min.z, p.z);
            bbox_max.x = std::max(bbox_max.x, p.x);
            bbox_max.y = std::max(bbox_max.y, p.y);
            bbox_max.z = std::max(bbox_max.z, p.z);
        }

        vec3f ab = b - a;
        vec3f ac = c - a;
        vec3f normal = ab.cross(ac);
        float area = normal.length() * 0.5f;
        if (area < 1e-6f) {
            continue; // skip degenerate triangle
        }

        normal = normal.normalize();
        vec3f centroid = (a + b + c) * (1.0f / 3.0f);
        faces.push_back({a, b, c, normal, area, centroid});

        // Signed volume contribution of tetrahedron (origin, a, b, c)
        total_volume += static_cast<double>(a.dot(b.cross(c))) / 6.0;
    }

    if (faces.empty()) {
        return {};
    }

    // Compute jitter magnitude relative to model scale
    float dx = bbox_max.x - bbox_min.x;
    float dy = bbox_max.y - bbox_min.y;
    float dz = bbox_max.z - bbox_min.z;
    float diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
    // Jitter: 0.01% of diagonal (~0.015 for a 150-unit model)
    float jitter = diagonal * 1e-4f;
    if (jitter < 1e-4f) {
        jitter = 1e-4f;
    }

    // 2. Unify normal orientation using volume sign heuristic
    if (total_volume < 0.0) {
        for (auto& f : faces) {
            f.normal = f.normal * -1.0f;
        }
    }

    // 3. Sample points with deduplication
    auto quantize = [](const vec3f& v) -> std::tuple<int64_t, int64_t, int64_t> {
        int64_t qx = static_cast<int64_t>(std::round(v.x * 1e5f));
        int64_t qy = static_cast<int64_t>(std::round(v.y * 1e5f));
        int64_t qz = static_cast<int64_t>(std::round(v.z * 1e5f));
        return {qx, qy, qz};
    };

    std::map<std::tuple<int64_t, int64_t, int64_t>, std::pair<vec3f, vec3f>> unique_points;

    for (const auto& f : faces) {
        // Vertices (average normal if shared)
        for (const auto& p : {f.a, f.b, f.c}) {
            auto key = quantize(p);
            auto it = unique_points.find(key);
            if (it == unique_points.end()) {
                unique_points[key] = {p, f.normal};
            } else {
                it->second.second = (it->second.second + f.normal).normalize();
            }
        }

        // Edge midpoints
        for (const auto& p : {(f.a + f.b) * 0.5f,
                               (f.b + f.c) * 0.5f,
                               (f.c + f.a) * 0.5f}) {
            unique_points[quantize(p)] = {p, f.normal};
        }

        // Interior samples with barycentric coordinates
        const int num_samples = 3;
        for (int s = 0; s < num_samples; ++s) {
            float u = static_cast<float>(s + 1) / (num_samples + 1);
            float v = (1.0f - u) * 0.5f;
            float w = 1.0f - u - v;

            vec3f p = f.a * u + f.b * v + f.c * w;

            // Scale-dependent jitter to break exact coplanarity
            p.x += (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * jitter;
            p.y += (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * jitter;
            p.z += (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * jitter;

            unique_points[quantize(p)] = {p, f.normal};
        }

        // Centroid with jitter
        vec3f centroid = f.centroid;
        centroid.x += (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * jitter;
        centroid.y += (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * jitter;
        centroid.z += (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * jitter;
        unique_points[quantize(centroid)] = {centroid, f.normal};
    }

    std::vector<std::pair<vec3f, vec3f>> result;
    result.reserve(unique_points.size());
    for (const auto& kv : unique_points) {
        result.push_back(kv.second);
    }
    return result;
}

} // namespace sinriv::kigstudio::cgal
