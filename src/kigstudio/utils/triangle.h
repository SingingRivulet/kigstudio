#pragma once

#include <cmath>
#include <concepts>

namespace sinriv::kigstudio {

/// Concept constraining 3D vector types usable with ray_triangle_intersect.
/// The type must expose .x/.y/.z as float-compatible scalars, support
/// component-wise +/-, scalar *, dot product, and cross product.
template <typename V>
concept Vec3Like = requires(const V v, const V u, float s) {
    { v.x } -> std::convertible_to<float>;
    { v.y } -> std::convertible_to<float>;
    { v.z } -> std::convertible_to<float>;
    { v + u } -> std::same_as<V>;
    { v - u } -> std::same_as<V>;
    { v * s } -> std::same_as<V>;
    { v.dot(u) } -> std::convertible_to<float>;
    { v.cross(u) } -> std::same_as<V>;
};

/// Triangle defined by three vertices.
template <Vec3Like V>
struct triangle {
    V v0, v1, v2;

    triangle() = default;
    triangle(const V& a, const V& b, const V& c) : v0(a), v1(b), v2(c) {}
};

// ============================================================================
// Möller–Trumbore ray-triangle intersection (scalar t output).
//
// Returns true and sets `t` (distance along the ray from origin) when the
// ray hits the triangle (v0, v1, v2).  `eps` controls both the
// parallel-check threshold and the minimum t considered valid.
// ============================================================================
template <Vec3Like V>
bool ray_triangle_intersect(const V& ray_origin,
                            const V& ray_dir,
                            const V& v0, const V& v1, const V& v2,
                            float& t,
                            float eps = 1e-8f) {
    V e1   = v1 - v0;
    V e2   = v2 - v0;
    V pvec = ray_dir.cross(e2);
    float det = e1.dot(pvec);
    if (std::abs(det) < eps)
        return false;

    float inv_det = 1.0f / det;
    V tvec = ray_origin - v0;
    float u = tvec.dot(pvec) * inv_det;
    if (u < 0.0f || u > 1.0f)
        return false;

    V qvec = tvec.cross(e1);
    float v = ray_dir.dot(qvec) * inv_det;
    if (v < 0.0f || u + v > 1.0f)
        return false;

    t = e2.dot(qvec) * inv_det;
    return t > eps;
}

// ============================================================================
// Convenience overload — outputs the hit *point* instead of a scalar t.
// ============================================================================
template <Vec3Like V>
bool ray_triangle_intersect(const V& ray_origin,
                            const V& ray_dir,
                            const V& v0, const V& v1, const V& v2,
                            V& out_point,
                            float eps = 1e-8f) {
    float t;
    if (!ray_triangle_intersect(ray_origin, ray_dir, v0, v1, v2, t, eps))
        return false;
    out_point = ray_origin + ray_dir * t;
    return true;
}

}  // namespace sinriv::kigstudio
