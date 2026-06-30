#include "kigstudio/sdf/sdf_SweepBezier.h"
#include <cstring>

namespace sinriv::kigstudio::sdf {

// ===================================================================
// Construction
// ===================================================================

SDF_SweepBezier::SDF_SweepBezier(const Vec3f& p0, const Vec3f& p1,
                                 const Vec3f& p2, const Vec3f& p3,
                                 float maj0, float min0,
                                 float maj1, float min1)
    : P0(p0), P1(p1), P2(p2), P3(p3)
    , major_start(maj0), minor_start(min0)
    , major_end  (maj1), minor_end  (min1) {}

// ===================================================================
// Cubic Bézier evaluation
// ===================================================================

Vec3f SDF_SweepBezier::bezier_eval(float t) const {
    const float u  = 1.0f - t;
    const float u2 = u * u;
    const float t2 = t * t;

    // B0·u³ + B1·3u²t + B2·3ut² + B3·t³
    return P0 * (u2 * u)
         + P1 * (3.0f * u2 * t)
         + P2 * (3.0f * u  * t2)
         + P3 * (t2 * t);
}

Vec3f SDF_SweepBezier::bezier_derivative(float t) const {
    const float u = 1.0f - t;

    // 3u²(P1−P0) + 6ut(P2−P1) + 3t²(P3−P2)
    return (P1 - P0) * (3.0f * u * u)
         + (P2 - P1) * (6.0f * u * t)
         + (P3 - P2) * (3.0f * t * t);
}

// ===================================================================
// Rotation Minimising Frame (RMF) — Rodrigues-rotation propagation
// ===================================================================

void SDF_SweepBezier::build_rmf() const {
    rmf_frames.clear();
    rmf_params.clear();
    rmf_arc_lengths.clear();
    total_arc_len = 0.0f;

    const int n = rmf_samples;
    if (n < 2) return;

    // ---- 1.  Sample positions and unit tangents ----
    std::vector<Vec3f> pts(n);
    std::vector<Vec3f> T(n);

    for (int i = 0; i < n; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(n - 1);
        rmf_params.push_back(t);
        pts[i] = bezier_eval(t);

        Vec3f d = bezier_derivative(t);
        const float len = d.length();
        T[i] = (len > 1e-8f) ? d / len : Vec3f(0, 0, 1);
    }

    // ---- 2.  Arc-length LUT ----
    rmf_arc_lengths.resize(n);
    rmf_arc_lengths[0] = 0.0f;
    float running = 0.0f;
    for (int i = 1; i < n; ++i) {
        const Vec3f delta = pts[i] - pts[i - 1];
        running += delta.length();
        rmf_arc_lengths[i] = running;
    }
    total_arc_len = running;

    // ---- 3.  Initial normal ----
    Vec3f N0;
    // User-specified initial normal (non-zero → use after orthogonalisation).
    const float n0_len2 = dot(initial_normal, initial_normal);
    if (n0_len2 > 1e-12f) {
        N0 = initial_normal;
    } else {
        // Auto-pick: a reference axis not parallel to T0.
        if (std::fabs(T[0].x) < 0.9f)
            N0 = Vec3f(1, 0, 0);
        else
            N0 = Vec3f(0, 1, 0);
    }

    // Gram–Schmidt to make N0 ⟂ T0.
    N0 = N0 - T[0] * dot(N0, T[0]);
    const float nl = N0.length();
    N0 = (nl > 1e-8f) ? N0 / nl : Vec3f(1, 0, 0);

    std::vector<Vec3f> N(n);
    N[0] = N0;

    // ---- 4.  Propagate normals via rotation around T_{i-1} × T_i ----
    for (int i = 1; i < n; ++i) {
        const Vec3f axis = cross(T[i - 1], T[i]);
        const float  alen = axis.length();

        if (alen < 1e-8f) {
            N[i] = N[i - 1];
        } else {
            const Vec3f ax = axis / alen;
            float cos_a = dot(T[i - 1], T[i]);
            cos_a = std::max(-1.0f, std::min(1.0f, cos_a));
            const float angle = std::acos(cos_a);

            const float ca = std::cos(angle);
            const float sa = std::sin(angle);
            N[i] = N[i - 1] * ca
                 + cross(ax, N[i - 1]) * sa
                 + ax * dot(ax, N[i - 1]) * (1.0f - ca);
        }
    }

    // ---- 5.  Assemble frames (x_axis = N, y_axis = B, z_axis = T) ----
    rmf_frames.resize(n);
    for (int i = 0; i < n; ++i) {
        Frame& f = rmf_frames[i];
        f.origin = pts[i];
        f.z_axis = T[i];
        f.x_axis = N[i];
        f.y_axis = cross(T[i], N[i]);

        const float yl = f.y_axis.length();
        if (yl > 1e-8f) {
            f.y_axis = f.y_axis / yl;
            f.x_axis = cross(f.y_axis, f.z_axis);
        }
    }

    rmf_built = true;
}

// ===================================================================
// Arc-length ↔ parameter t  (binary search in LUT)
// ===================================================================

float SDF_SweepBezier::t_to_arc_length(float t) const {
    if (!rmf_built) build_rmf();

    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return total_arc_len;

    // Binary search for the interval containing t.
    int lo = 0, hi = rmf_samples - 1;
    while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;
        if (rmf_params[mid] <= t)
            lo = mid;
        else
            hi = mid;
    }

    // Linear interpolation inside [lo, hi].
    const float dt = rmf_params[hi] - rmf_params[lo];
    if (dt < 1e-8f) return rmf_arc_lengths[lo];
    const float frac = (t - rmf_params[lo]) / dt;
    return rmf_arc_lengths[lo] + frac * (rmf_arc_lengths[hi] - rmf_arc_lengths[lo]);
}

float SDF_SweepBezier::arc_length_to_t(float s) const {
    if (!rmf_built) build_rmf();

    if (s <= 0.0f) return 0.0f;
    if (s >= total_arc_len) return 1.0f;

    // Binary search for the interval containing s.
    int lo = 0, hi = rmf_samples - 1;
    while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;
        if (rmf_arc_lengths[mid] <= s)
            lo = mid;
        else
            hi = mid;
    }

    // Linear interpolation inside [lo, hi].
    const float ds = rmf_arc_lengths[hi] - rmf_arc_lengths[lo];
    if (ds < 1e-8f) return rmf_params[lo];
    const float frac = (s - rmf_arc_lengths[lo]) / ds;
    return rmf_params[lo] + frac * (rmf_params[hi] - rmf_params[lo]);
}

// ===================================================================
// Retrieve RMF frame at parameter t, with twist applied
// ===================================================================

Frame SDF_SweepBezier::get_frame(float t) const {
    if (!rmf_built) build_rmf();

    t = std::max(0.0f, std::min(1.0f, t));

    // Nearest-neighbour lookup.
    const float fidx = t * static_cast<float>(rmf_samples - 1);
    int idx = static_cast<int>(fidx + 0.5f);
    idx = std::max(0, std::min(rmf_samples - 1, idx));

    Frame f = rmf_frames[idx];

    // ---- Apply twist: rotate (x_axis, y_axis) around z_axis ----
    const float s = rmf_arc_lengths[idx];
    const float theta = twist_start + twist_rate * s;

    if (std::fabs(theta) > 1e-8f) {
        const float ca = std::cos(theta);
        const float sa = std::sin(theta);

        const Vec3f new_x = f.x_axis * ca + f.y_axis * sa;
        const Vec3f new_y = f.y_axis * ca - f.x_axis * sa;

        f.x_axis = new_x;
        f.y_axis = new_y;
    }

    return f;
}

// ===================================================================
// Exact 2-D ellipse SDF  (Inigo Quilez, MIT licence)
// ===================================================================

float SDF_SweepBezier::sd_ellipse(float u, float v, float a, float b) {
    // Degenerate cases
    if (a < 1e-8f && b < 1e-8f) return std::sqrt(u * u + v * v);
    if (a < 1e-8f) return std::fabs(v) - b;
    if (b < 1e-8f) return std::fabs(u) - a;

    // Circle fast-path
    if (std::fabs(a - b) < 1e-6f) {
        return std::sqrt(u * u + v * v) - a;
    }

    u = std::fabs(u);
    v = std::fabs(v);

    // Work in the half-quadrant where u/a ≥ v/b by swapping if needed.
    bool swapped = false;
    if (u / a < v / b) {
        std::swap(u, v);
        std::swap(a, b);
        swapped = true;
    }

    const float uu = u / a;
    const float vv = v / b;

    // Point at origin → deepest interior.
    if (uu < 1e-8f && vv < 1e-8f) return -std::min(a, b);

    // Newton's method for the closest point on the ellipse.
    // Parameterisation:  E(θ) = (a·cos θ,  b·sin θ).
    // The closest point to (u,v) satisfies:
    //   f(θ) = a·u·sin θ − b·v·cos θ + (a²−b²)·sin θ·cos θ = 0
    float theta = std::atan2(vv, uu);

    const float a2 = a * a;
    const float b2 = b * b;
    const float a2b2 = a2 - b2;

    for (int iter = 0; iter < 8; ++iter) {
        const float ct = std::cos(theta);
        const float st = std::sin(theta);

        const float f  = a * u * st - b * v * ct + a2b2 * st * ct;
        const float fp = a * u * ct + b * v * st + a2b2 * (ct * ct - st * st);

        if (std::fabs(fp) < 1e-12f) break;
        const float step = f / fp;
        theta -= step;
        if (std::fabs(step) < 1e-8f) break;
    }

    // Clamp to [0, π/2] — the ellipse is symmetric.
    theta = std::max(0.0f, std::min(3.141592653589793f * 0.5f, theta));

    const float ex = a * std::cos(theta);
    const float ey = b * std::sin(theta);

    const float dx = u - ex;
    const float dy = v - ey;
    const float dist = std::sqrt(dx * dx + dy * dy);

    const float inside_test = uu * uu + vv * vv - 1.0f;

    (void)swapped;
    return inside_test < 0.0f ? -dist : dist;
}

// ===================================================================
// SDF evaluation
// ===================================================================

float SDF_SweepBezier::get(const Vec3f& p) const {
    if (!rmf_built) build_rmf();

    float min_dist = std::numeric_limits<float>::max();

    for (int i = 0; i < sweep_samples; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(sweep_samples - 1);

        const Frame  f = get_frame(t);
        const Vec3f  local = f.worldToLocal(p);
        const float  u = local.x;
        const float  v = local.y;
        const float  w = local.z;

        // Linearly interpolate ellipse radii.
        const float a = major_start + (major_end - major_start) * t;
        const float b = minor_start + (minor_end - minor_start) * t;

        if (a <= 0.0f && b <= 0.0f) {
            const float d = std::sqrt(u * u + v * v + w * w);
            if (d < min_dist) min_dist = d;
            continue;
        }

        const float e = sd_ellipse(u, v,
                                    a > 0.0f ? a : 0.0f,
                                    b > 0.0f ? b : 0.0f);

        float dist;
        if (e <= 0.0f) {
            dist = std::fabs(w);
        } else {
            dist = std::sqrt(e * e + w * w);
        }

        if (dist < min_dist) min_dist = dist;
    }

    return min_dist;
}

// ===================================================================
// Voxel-grid evaluation
// ===================================================================

void SDF_SweepBezier::get(const Vec3f&      begin,
                          const Vec3f&      voxelSize,
                          const Vec3i&      voxelCount,
                          std::vector<float>& out) const {
    if (voxelCount.x <= 0 || voxelCount.y <= 0 || voxelCount.z <= 0) {
        out.clear();
        return;
    }

    if (!rmf_built) build_rmf();

    const size_t total = static_cast<size_t>(voxelCount.x) *
                         static_cast<size_t>(voxelCount.y) *
                         static_cast<size_t>(voxelCount.z);
    out.resize(total);

    size_t idx = 0;
    for (int z = 0; z < voxelCount.z; ++z) {
        const float wz = begin.z + static_cast<float>(z) * voxelSize.z;
        for (int y = 0; y < voxelCount.y; ++y) {
            const float wy = begin.y + static_cast<float>(y) * voxelSize.y;
            for (int x = 0; x < voxelCount.x; ++x) {
                const float wx = begin.x + static_cast<float>(x) * voxelSize.x;
                out[idx++] = get(Vec3f(wx, wy, wz));
            }
        }
    }
}

// ===================================================================
// Debug / info
// ===================================================================

std::string SDF_SweepBezier::getInfo(int indent) const {
    const std::string prefix(static_cast<size_t>(indent) * 2, ' ');
    return prefix + "SDF_SweepBezier("
           + "P0=" + std::to_string(P0.x) + "," + std::to_string(P0.y) + "," + std::to_string(P0.z)
           + " P1=" + std::to_string(P1.x) + "," + std::to_string(P1.y) + "," + std::to_string(P1.z)
           + " P2=" + std::to_string(P2.x) + "," + std::to_string(P2.y) + "," + std::to_string(P2.z)
           + " P3=" + std::to_string(P3.x) + "," + std::to_string(P3.y) + "," + std::to_string(P3.z)
           + " maj0=" + std::to_string(major_start)
           + " min0=" + std::to_string(minor_start)
           + " maj1=" + std::to_string(major_end)
           + " min1=" + std::to_string(minor_end)
           + " twist_start=" + std::to_string(twist_start)
           + " twist_rate=" + std::to_string(twist_rate)
           + " N0=" + std::to_string(initial_normal.x) + "," + std::to_string(initial_normal.y) + "," + std::to_string(initial_normal.z)
           + " arc_len=" + std::to_string(total_arc_len)
           + ")";
}

// ===================================================================
// JSON serialisation
// ===================================================================

cJSON* SDF_SweepBezier::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "sweep_bezier");

    cJSON_AddItemToObject(obj, "P0", sinriv::kigstudio::to_json(P0));
    cJSON_AddItemToObject(obj, "P1", sinriv::kigstudio::to_json(P1));
    cJSON_AddItemToObject(obj, "P2", sinriv::kigstudio::to_json(P2));
    cJSON_AddItemToObject(obj, "P3", sinriv::kigstudio::to_json(P3));

    cJSON_AddNumberToObject(obj, "major_start", major_start);
    cJSON_AddNumberToObject(obj, "minor_start", minor_start);
    cJSON_AddNumberToObject(obj, "major_end",   major_end);
    cJSON_AddNumberToObject(obj, "minor_end",   minor_end);

    cJSON_AddNumberToObject(obj, "twist_start", twist_start);
    cJSON_AddNumberToObject(obj, "twist_rate",  twist_rate);

    cJSON_AddItemToObject(obj, "initial_normal",
                          sinriv::kigstudio::to_json(initial_normal));

    cJSON_AddNumberToObject(obj, "sweep_samples", sweep_samples);
    cJSON_AddNumberToObject(obj, "rmf_samples",   rmf_samples);

    return obj;
}

void SDF_SweepBezier::fromJSON(const cJSON* json) {
    if (!json) return;

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, json) {
        if (!child->string) continue;

        // ---- vec3 fields ----
        if (cJSON_IsObject(child)) {
            if (strcmp(child->string, "P0") == 0)
                P0 = sinriv::kigstudio::vec3_from_json<Vec3f>(child);
            else if (strcmp(child->string, "P1") == 0)
                P1 = sinriv::kigstudio::vec3_from_json<Vec3f>(child);
            else if (strcmp(child->string, "P2") == 0)
                P2 = sinriv::kigstudio::vec3_from_json<Vec3f>(child);
            else if (strcmp(child->string, "P3") == 0)
                P3 = sinriv::kigstudio::vec3_from_json<Vec3f>(child);
            else if (strcmp(child->string, "initial_normal") == 0)
                initial_normal = sinriv::kigstudio::vec3_from_json<Vec3f>(child);
        }

        // ---- scalar fields ----
        if (cJSON_IsNumber(child)) {
            const double v = cJSON_GetNumberValue(child);
            if (strcmp(child->string, "major_start") == 0)
                major_start = static_cast<float>(v);
            else if (strcmp(child->string, "minor_start") == 0)
                minor_start = static_cast<float>(v);
            else if (strcmp(child->string, "major_end") == 0)
                major_end   = static_cast<float>(v);
            else if (strcmp(child->string, "minor_end") == 0)
                minor_end   = static_cast<float>(v);
            else if (strcmp(child->string, "twist_start") == 0)
                twist_start = static_cast<float>(v);
            else if (strcmp(child->string, "twist_rate") == 0)
                twist_rate  = static_cast<float>(v);
            else if (strcmp(child->string, "sweep_samples") == 0)
                sweep_samples = static_cast<int>(v);
            else if (strcmp(child->string, "rmf_samples") == 0)
                rmf_samples = static_cast<int>(v);
        }
    }

    rmf_built = false;
}

// ===================================================================
// Type registration
// ===================================================================

static bool _register_sweep_bezier = []() {
    sdf_register_type("sweep_bezier",
        [](const cJSON* json) -> std::shared_ptr<SDFBase> {
            auto obj = std::make_shared<SDF_SweepBezier>();
            obj->fromJSON(json);
            return obj;
        });
    return true;
}();

}  // namespace sinriv::kigstudio::sdf
