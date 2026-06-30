#pragma once
#include <cJSON.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "kigstudio/sdf/sdf_shape.h"
#include "kigstudio/utils/vec3.h"

namespace sinriv::kigstudio::sdf {

/// SDF_SweepBezier — Sweep an elliptical cross-section along a cubic Bézier curve.
///
/// The cross-section is an ellipse whose major and minor radii can vary linearly
/// from the start (t=0) to the end (t=1) of the curve.  The local frame along
/// the curve is built with a Rotation Minimising Frame (RMF) so that the ellipse
/// orientation stays smooth even where curvature vanishes.
///
/// ## Twist (optional)
///
/// On top of the RMF, an additional rotation θ(s) is applied around the tangent
/// axis, where s is the cumulative arc-length along the curve:
///
///     θ(s) = twist_start + twist_rate × s
///
/// - `twist_start` — rotation angle (radians) at the start of the curve (s = 0).
/// - `twist_rate`  — radians per unit arc-length; total twist across the whole
///                    curve equals `twist_rate × total_arc_length`.
///
/// When both are 0 (the default) the cross-section is twist-free.
///
/// ## Initial normal
///
/// By default (`initial_normal` is the zero vector) the RMF picks an automatic
/// reference normal at t = 0.  Pass a non-zero `initial_normal` to lock the
/// cross-section orientation at the start — the vector will be orthogonalised
/// against the start tangent before RMF propagation.
///
/// End caps are flat elliptical disks at t = 0 and t = 1.
///
/// JSON type: "sweep_bezier"
struct SDF_SweepBezier final : public SDFBase {
    // ---- Bézier control points ----
    Vec3f P0{0, 0, 0};
    Vec3f P1{0, 1, 1};
    Vec3f P2{1, 0, 2};
    Vec3f P3{1, 1, 3};

    // ---- Ellipse radii at t = 0 ----
    float major_start = 1.0f;
    float minor_start = 1.0f;

    // ---- Ellipse radii at t = 1 ----
    float major_end   = 0.0f;
    float minor_end   = 0.0f;

    // ---- Twist (arc-length-parameterised) ----
    float twist_start = 0.0f;  ///< rotation angle at s = 0  (radians)
    float twist_rate  = 0.0f;  ///< radians per unit arc-length

    // ---- Initial normal (zero vector → auto-compute) ----
    Vec3f initial_normal{0, 0, 0};

    // ---- Quality knobs ----
    int sweep_samples = 64;   ///< t-samples for the sweep minisation pass
    int rmf_samples   = 256;  ///< samples used to build the RMF lookup table

    // ---- Precomputed data (mutable for lazy build) ----
    mutable std::vector<Frame>  rmf_frames;       ///< RMF frame at each sample
    mutable std::vector<float>  rmf_params;       ///< t-parameter for each sample
    mutable std::vector<float>  rmf_arc_lengths;  ///< cumulative arc-length s at each sample
    mutable float               total_arc_len = 0.0f;
    mutable bool                rmf_built = false;

    // ---- Construction ----
    SDF_SweepBezier() = default;

    SDF_SweepBezier(const Vec3f& p0, const Vec3f& p1,
                    const Vec3f& p2, const Vec3f& p3,
                    float maj0, float min0,
                    float maj1, float min1);

    // ---- Bézier helpers ----
    Vec3f bezier_eval(float t)       const;
    Vec3f bezier_derivative(float t) const;

    // ---- RMF ----
    void  build_rmf() const;

    /// Return the RMF frame at parameter t, with twist already applied.
    Frame get_frame(float t) const;

    // ---- Arc-length helpers (require built RMF) ----
    float t_to_arc_length(float t) const;   ///< parameter t → cumulative arc-length s
    float arc_length_to_t(float s) const;   ///< arc-length s → parameter t

    // ---- 2-D ellipse SDF ----
    static float sd_ellipse(float u, float v, float a, float b);

    // ---- SDFBase interface ----
    float get(const Vec3f& p) const override;

    void get(const Vec3f&      begin,
             const Vec3f&      voxelSize,
             const Vec3i&      voxelCount,
             std::vector<float>& out) const override;

    std::string getInfo(int indent = 0) const override;
    cJSON*      toJSON()               const override;
    void        fromJSON(const cJSON* json)  override;
};

}  // namespace sinriv::kigstudio::sdf
