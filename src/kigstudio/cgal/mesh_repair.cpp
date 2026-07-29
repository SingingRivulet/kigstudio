#include "kigstudio/cgal/mesh_repair.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/IO/polygon_soup_io.h>

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <cstdio>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

namespace PMP = CGAL::Polygon_mesh_processing;

namespace sinriv::kigstudio::cgal {

// ===========================================================================
// Internal helpers — MeshData ↔ Surface_mesh conversion
// ===========================================================================

namespace {

struct VKey {
    int64_t x, y, z;
    bool operator==(const VKey& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};
struct VKeyHash {
    size_t operator()(const VKey& k) const {
        return std::hash<int64_t>()(k.x
            ^ (k.y * 0x9e3779b97f4a7c15ULL)
            ^ (k.z * 0x9e3779b97f4a7c16ULL));
    }
};

/// Convert MeshData → CGAL Surface_mesh (welding vertices).
Surface_mesh to_surface_mesh(const MeshData& mesh) {
    Surface_mesh sm;
    std::unordered_map<VKey, Surface_mesh::Vertex_index, VKeyHash> vmap;

    auto get_v = [&](const vec3f& p) -> Surface_mesh::Vertex_index {
        VKey key{
            static_cast<int64_t>(std::round(p.x * 1e6)),
            static_cast<int64_t>(std::round(p.y * 1e6)),
            static_cast<int64_t>(std::round(p.z * 1e6))
        };
        auto it = vmap.find(key);
        if (it != vmap.end()) return it->second;
        auto vi = sm.add_vertex(Point_3(p.x, p.y, p.z));
        vmap.emplace(key, vi);
        return vi;
    };

    for (const auto& [tri, n] : mesh) {
        auto a = std::get<0>(tri);
        auto b = std::get<1>(tri);
        auto c = std::get<2>(tri);
        auto va = get_v(a), vb = get_v(b), vc = get_v(c);
        if (va == vb || vb == vc || vc == va) continue;
        sm.add_face(va, vb, vc);
    }
    return sm;
}

/// Convert CGAL Surface_mesh → MeshData.
/// Templated on the mesh type so both EPECK and EPICK meshes can be
/// converted (CGAL::to_double is the identity for EPICK's double FT).
template <typename SMesh>
MeshData from_surface_mesh(const SMesh& sm) {
    MeshData result;
    result.reserve(sm.number_of_faces());
    for (auto f : sm.faces()) {
        auto hd = sm.halfedge(f);
        auto p0 = sm.point(sm.source(hd));
        auto p1 = sm.point(sm.target(hd));
        auto p2 = sm.point(sm.target(sm.next(hd)));
        vec3f a(static_cast<float>(CGAL::to_double(p0.x())),
                static_cast<float>(CGAL::to_double(p0.y())),
                static_cast<float>(CGAL::to_double(p0.z())));
        vec3f b(static_cast<float>(CGAL::to_double(p1.x())),
                static_cast<float>(CGAL::to_double(p1.y())),
                static_cast<float>(CGAL::to_double(p1.z())));
        vec3f c(static_cast<float>(CGAL::to_double(p2.x())),
                static_cast<float>(CGAL::to_double(p2.y())),
                static_cast<float>(CGAL::to_double(p2.z())));
        vec3f n = (b - a).cross(c - a).normalize();
        result.emplace_back(std::make_tuple(a, b, c), n);
    }
    return result;
}

} // anonymous namespace

// ===========================================================================
// 1. Alpha Wrap
// ===========================================================================

MeshData alpha_wrap(const MeshData& mesh, double alpha, double offset) {
    // alpha_wrap_3 内部静态断言要求 FT 是浮点类型（显式禁用精确核），
    // 因此这里单独使用 EPICK（精确谓词 + 非精确构造）。
    using AW_Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using AW_Point = AW_Kernel::Point_3;
    using AW_Mesh = CGAL::Surface_mesh<AW_Point>;

    if (mesh.empty()) return {};

    // Collect all points from the triangle soup
    std::vector<AW_Point> points;
    std::vector<std::vector<std::size_t>> faces;
    points.reserve(mesh.size() * 3);
    faces.reserve(mesh.size());

    // Use a local vertex map to build indexed faces
    std::unordered_map<VKey, std::size_t, VKeyHash> vmap;

    for (const auto& [tri, n] : mesh) {
        std::vector<std::size_t> face;
        for (const auto& v : {std::get<0>(tri), std::get<1>(tri), std::get<2>(tri)}) {
            VKey key{
                static_cast<int64_t>(std::round(v.x * 1e6)),
                static_cast<int64_t>(std::round(v.y * 1e6)),
                static_cast<int64_t>(std::round(v.z * 1e6))
            };
            auto it = vmap.find(key);
            if (it != vmap.end()) {
                face.push_back(it->second);
            } else {
                std::size_t idx = points.size();
                points.emplace_back(v.x, v.y, v.z);
                vmap.emplace(key, idx);
                face.push_back(idx);
            }
        }
        if (face[0] != face[1] && face[1] != face[2] && face[2] != face[0])
            faces.push_back(std::move(face));
    }

    if (points.size() < 3 || faces.empty()) return {};

    try {
        AW_Mesh out;
        CGAL::alpha_wrap_3(points, faces, alpha, offset, out);
        if (out.number_of_faces() == 0) return {};
        return from_surface_mesh(out);
    } catch (const std::exception& e) {
        std::cerr << "[alpha_wrap] " << e.what() << "\n";
        return {};
    }
}

// ===========================================================================
// 2. Fill Holes
// ===========================================================================

MeshData fill_holes(const MeshData& mesh) {
    if (mesh.empty()) return {};

    Surface_mesh sm = to_surface_mesh(mesh);
    if (sm.number_of_faces() == 0) return {};

    try {
        // Collect all boundary halfedges
        std::vector<Surface_mesh::Halfedge_index> borders;
        for (auto h : sm.halfedges()) {
            if (sm.is_border(h))
                borders.push_back(h);
        }

        // Deduplicate to one representative per boundary cycle
        std::unordered_set<Surface_mesh::Halfedge_index> seen;
        std::vector<Surface_mesh::Halfedge_index> cycles;
        for (auto h : borders) {
            auto n = sm.next(h);
            // Walk to find the canonical border halfedge
            auto cur = h;
            while (!sm.is_border(sm.prev(cur)))
                cur = sm.prev(cur);
            if (seen.insert(cur).second)
                cycles.push_back(cur);
        }

        unsigned filled = 0;
        for (auto h : cycles) {
            if (!sm.is_border(h)) continue;
            try {
                PMP::triangulate_refine_and_fair_hole(sm, h,
                    CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, sm)));
                ++filled;
            } catch (...) {
                // Skip holes that can't be filled
            }
        }

        std::cerr << "[fill_holes] Filled " << filled << " / " << cycles.size()
                  << " holes.\n";
        return from_surface_mesh(sm);
    } catch (const std::exception& e) {
        std::cerr << "[fill_holes] " << e.what() << "\n";
        return {};
    }
}

// ===========================================================================
// 3. Stitch Borders
// ===========================================================================

MeshData stitch_borders(const MeshData& mesh, double max_dist) {
    if (mesh.empty()) return {};

    Surface_mesh sm = to_surface_mesh(mesh);
    if (sm.number_of_faces() == 0) return {};

    try {
        std::size_t count = PMP::stitch_borders(sm,
            CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, sm)));
        std::cerr << "[stitch_borders] Stitched " << count << " border vertices.\n";

        // Remove isolated vertices after stitching
        sm.collect_garbage();
        return from_surface_mesh(sm);
    } catch (const std::exception& e) {
        std::cerr << "[stitch_borders] " << e.what() << "\n";
        return {};
    }
}

// ===========================================================================
// 4. Merge Duplicated Vertices
// ===========================================================================

MeshData merge_duplicate_vertices(const MeshData& mesh, double tol) {
    if (mesh.empty()) return {};

    // Simply re-index through the existing to_surface_mesh() which already
    // welds vertices by quantising to 1e-6. For larger tolerances we adjust
    // the quantisation scale.
    double scale = 1.0 / std::max(tol, 1e-9);

    Surface_mesh sm;
    std::unordered_map<VKey, Surface_mesh::Vertex_index, VKeyHash> vmap;

    auto get_v = [&](const vec3f& p) -> Surface_mesh::Vertex_index {
        VKey key{
            static_cast<int64_t>(std::round(p.x * scale)),
            static_cast<int64_t>(std::round(p.y * scale)),
            static_cast<int64_t>(std::round(p.z * scale))
        };
        auto it = vmap.find(key);
        if (it != vmap.end()) return it->second;
        auto vi = sm.add_vertex(Point_3(p.x, p.y, p.z));
        vmap.emplace(key, vi);
        return vi;
    };

    for (const auto& [tri, n] : mesh) {
        auto a = std::get<0>(tri), b = std::get<1>(tri), c = std::get<2>(tri);
        auto va = get_v(a), vb = get_v(b), vc = get_v(c);
        if (va == vb || vb == vc || vc == va) continue;
        sm.add_face(va, vb, vc);
    }

    sm.collect_garbage();

    std::cerr << "[merge_vertices] " << sm.number_of_vertices()
              << " vertices, " << sm.number_of_faces() << " faces (tol="
              << tol << ").\n";
    return from_surface_mesh(sm);
}

// ===========================================================================
// Helpers for robust boolean operations — auto-repair non-closed or
// self-intersecting meshes when the first attempt fails, then retry.
// ===========================================================================

namespace {

/// Check whether a MeshData converts to a closed Surface_mesh.
bool is_mesh_closed(const MeshData& md) {
    if (md.empty()) return false;
    Surface_mesh sm = to_surface_mesh(md);
    if (sm.number_of_faces() == 0) return false;
    return CGAL::is_closed(sm);
}

/// Try to repair a mesh so it becomes closed (watertight).
/// Pipeline: merge vertices → fill holes → orient volume.
/// If the result is still not closed, falls back to alpha_wrap which
/// guarantees a watertight (but approximate) output.
/// Returns empty on total failure.
MeshData repair_for_boolean(const MeshData& mesh) {
    // Step 0: merge near-duplicate vertices with a relaxed tolerance
    MeshData current = merge_duplicate_vertices(mesh, 1e-4);
    if (current.empty()) current = mesh;

    // Step 1: fill holes (boundary cycles → triangulated patches)
    MeshData filled = fill_holes(current);
    if (!filled.empty()) current = std::move(filled);

    // Step 2: orient all faces to bound a volume
    MeshData oriented = orient_volume(current);
    if (!oriented.empty()) current = std::move(oriented);

    // Step 3: verify the result is actually closed
    if (is_mesh_closed(current)) {
        std::cerr << "[repair] Mesh is now closed.\n";
        return current;
    }

    // Step 4: alpha_wrap as last resort — guarantees watertight output
    std::cerr << "[repair] Mesh still not closed, "
              << "trying alpha_wrap as fallback...\n";
    MeshData wrapped = alpha_wrap(mesh, 10.0, 0.1);
    if (!wrapped.empty() && is_mesh_closed(wrapped)) {
        std::cerr << "[repair] alpha_wrap produced closed mesh.\n";
        return wrapped;
    }

    std::cerr << "[repair] All repair strategies failed.\n";
    return {};
}

/// Core boolean-difference implementation (no repair, no early-empty check).
/// Takes copies because CGAL modifies the input meshes during corefinement.
MeshData mesh_difference_impl(Surface_mesh sm_a, Surface_mesh sm_b) {
    try {
        Surface_mesh out;
        bool valid = PMP::corefine_and_compute_difference(
            sm_a, sm_b, out,
            CGAL::parameters::vertex_point_map(
                get(CGAL::vertex_point, sm_a)),
            CGAL::parameters::vertex_point_map(
                get(CGAL::vertex_point, sm_b)),
            CGAL::parameters::vertex_point_map(
                get(CGAL::vertex_point, out)));

        if (!valid || out.number_of_faces() == 0) {
            return {};
        }
        return from_surface_mesh(out);
    } catch (const std::exception& e) {
        std::cerr << "[mesh_difference] " << e.what() << "\n";
        return {};
    }
}

/// Core boolean-union implementation (no repair, no early-empty check).
/// Takes copies because CGAL modifies the input meshes during corefinement.
MeshData mesh_union_impl(Surface_mesh sm_a, Surface_mesh sm_b) {
    try {
        Surface_mesh out;
        bool valid = PMP::corefine_and_compute_union(
            sm_a, sm_b, out,
            CGAL::parameters::vertex_point_map(
                get(CGAL::vertex_point, sm_a)),
            CGAL::parameters::vertex_point_map(
                get(CGAL::vertex_point, sm_b)),
            CGAL::parameters::vertex_point_map(
                get(CGAL::vertex_point, out)));

        if (!valid || out.number_of_faces() == 0) {
            return {};
        }
        return from_surface_mesh(out);
    } catch (const std::exception& e) {
        std::cerr << "[mesh_union] " << e.what() << "\n";
        return {};
    }
}

/// Full repair-and-retry wrapper for boolean difference.
/// On failure, repairs meshes, validates they became closed, then retries.
/// If the first repair still fails (e.g. self-intersections), tries alpha_wrap
/// as a second-level fallback which guarantees watertight output.
/// Returns empty if everything fails (caller should fall back gracefully).
MeshData mesh_difference_robust(const MeshData& mesh_a,
                                const MeshData& mesh_b,
                                bool a_closed,
                                bool b_closed) {
    // --- Level 1: fill_holes + orient_volume for non-closed meshes ---
    {
        MeshData repaired_a_data =
            !a_closed ? repair_for_boolean(mesh_a) : mesh_a;
        MeshData repaired_b_data =
            !b_closed ? repair_for_boolean(mesh_b) : mesh_b;

        if (!repaired_a_data.empty() && !repaired_b_data.empty()) {
            Surface_mesh sm_a2 = to_surface_mesh(repaired_a_data);
            Surface_mesh sm_b2 = to_surface_mesh(repaired_b_data);
            if (sm_a2.number_of_faces() > 0 &&
                sm_b2.number_of_faces() > 0 &&
                CGAL::is_closed(sm_a2) && CGAL::is_closed(sm_b2)) {
                std::cerr << "[mesh_difference] Retrying with repaired "
                          << "meshes...\n";
                auto result =
                    mesh_difference_impl(std::move(sm_a2), std::move(sm_b2));
                if (!result.empty()) {
                    std::cerr << "[mesh_difference] Repair succeeded: "
                              << result.size() << " faces.\n";
                    return result;
                }
                std::cerr << "[mesh_difference] Repaired meshes still "
                          << "fail boolean (possible self-intersections).\n";
            }
        }
    }

    // --- Level 2: alpha_wrap fallback (guarantees closed, non-self-intersecting) ---
    std::cerr << "[mesh_difference] Trying alpha_wrap fallback...\n";
    {
        MeshData wrapped_a = alpha_wrap(mesh_a, 10.0, 0.1);
        MeshData wrapped_b = alpha_wrap(mesh_b, 10.0, 0.1);

        if (!wrapped_a.empty() && !wrapped_b.empty()) {
            Surface_mesh sm_a3 = to_surface_mesh(wrapped_a);
            Surface_mesh sm_b3 = to_surface_mesh(wrapped_b);
            if (sm_a3.number_of_faces() > 0 &&
                sm_b3.number_of_faces() > 0 &&
                CGAL::is_closed(sm_a3) && CGAL::is_closed(sm_b3)) {
                std::cerr << "[mesh_difference] Retrying with alpha_wrap "
                          << "meshes...\n";
                auto result =
                    mesh_difference_impl(std::move(sm_a3), std::move(sm_b3));
                if (!result.empty()) {
                    std::cerr << "[mesh_difference] alpha_wrap succeeded: "
                              << result.size() << " faces.\n";
                    return result;
                }
            }
        }
    }

    std::cerr << "[mesh_difference] All repair strategies failed.\n";
    return {};
}

/// Full repair-and-retry wrapper for boolean union.
MeshData mesh_union_robust(const MeshData& mesh_a,
                           const MeshData& mesh_b,
                           bool a_closed,
                           bool b_closed) {
    // --- Level 1: fill_holes + orient_volume ---
    {
        MeshData repaired_a_data =
            !a_closed ? repair_for_boolean(mesh_a) : mesh_a;
        MeshData repaired_b_data =
            !b_closed ? repair_for_boolean(mesh_b) : mesh_b;

        if (!repaired_a_data.empty() && !repaired_b_data.empty()) {
            Surface_mesh sm_a2 = to_surface_mesh(repaired_a_data);
            Surface_mesh sm_b2 = to_surface_mesh(repaired_b_data);
            if (sm_a2.number_of_faces() > 0 &&
                sm_b2.number_of_faces() > 0 &&
                CGAL::is_closed(sm_a2) && CGAL::is_closed(sm_b2)) {
                std::cerr << "[mesh_union] Retrying with repaired meshes...\n";
                auto result =
                    mesh_union_impl(std::move(sm_a2), std::move(sm_b2));
                if (!result.empty()) {
                    std::cerr << "[mesh_union] Repair succeeded: "
                              << result.size() << " faces.\n";
                    return result;
                }
                std::cerr << "[mesh_union] Repaired meshes still fail "
                          << "boolean (possible self-intersections).\n";
            }
        }
    }

    // --- Level 2: alpha_wrap fallback ---
    std::cerr << "[mesh_union] Trying alpha_wrap fallback...\n";
    {
        MeshData wrapped_a = alpha_wrap(mesh_a, 10.0, 0.1);
        MeshData wrapped_b = alpha_wrap(mesh_b, 10.0, 0.1);

        if (!wrapped_a.empty() && !wrapped_b.empty()) {
            Surface_mesh sm_a3 = to_surface_mesh(wrapped_a);
            Surface_mesh sm_b3 = to_surface_mesh(wrapped_b);
            if (sm_a3.number_of_faces() > 0 &&
                sm_b3.number_of_faces() > 0 &&
                CGAL::is_closed(sm_a3) && CGAL::is_closed(sm_b3)) {
                std::cerr << "[mesh_union] Retrying with alpha_wrap "
                          << "meshes...\n";
                auto result =
                    mesh_union_impl(std::move(sm_a3), std::move(sm_b3));
                if (!result.empty()) {
                    std::cerr << "[mesh_union] alpha_wrap succeeded: "
                              << result.size() << " faces.\n";
                    return result;
                }
            }
        }
    }

    std::cerr << "[mesh_union] All repair strategies failed.\n";
    return {};
}

}  // anonymous namespace

// ===========================================================================
// 5. Boolean Union
// ===========================================================================

MeshData mesh_union(const MeshData& mesh_a, const MeshData& mesh_b) {
    if (mesh_a.empty() || mesh_b.empty()) return {};

    Surface_mesh sm_a = to_surface_mesh(mesh_a);
    Surface_mesh sm_b = to_surface_mesh(mesh_b);

    if (sm_a.number_of_faces() == 0 || sm_b.number_of_faces() == 0)
        return {};

    const bool a_closed = CGAL::is_closed(sm_a);
    const bool b_closed = CGAL::is_closed(sm_b);

    if (!a_closed) {
        std::cerr << "[mesh_union] Warning: mesh A is not closed ("
                  << sm_a.number_of_faces() << " faces).\n";
    }
    if (!b_closed) {
        std::cerr << "[mesh_union] Warning: mesh B is not closed ("
                  << sm_b.number_of_faces() << " faces).\n";
    }

    // First attempt: use the original meshes directly
    auto result = mesh_union_impl(std::move(sm_a), std::move(sm_b));
    if (!result.empty()) {
        std::cerr << "[mesh_union] Result: " << result.size() << " faces.\n";
        return result;
    }

    // Direct boolean failed — try repair + retry regardless of cause
    // (non-closed meshes, self-intersections, etc.)
    std::cerr << "[mesh_union] Direct boolean failed, "
              << "attempting repair + retry...\n";
    return mesh_union_robust(mesh_a, mesh_b, a_closed, b_closed);
}

// ===========================================================================
// 5b. Boolean Difference (A - B)
// ===========================================================================

MeshData mesh_difference(const MeshData& mesh_a, const MeshData& mesh_b) {
    if (mesh_a.empty() || mesh_b.empty()) return {};

    Surface_mesh sm_a = to_surface_mesh(mesh_a);
    Surface_mesh sm_b = to_surface_mesh(mesh_b);

    if (sm_a.number_of_faces() == 0 || sm_b.number_of_faces() == 0)
        return {};

    const bool a_closed = CGAL::is_closed(sm_a);
    const bool b_closed = CGAL::is_closed(sm_b);

    if (!a_closed) {
        std::cerr << "[mesh_difference] Warning: mesh A is not closed ("
                  << sm_a.number_of_faces() << " faces).\n";
    }
    if (!b_closed) {
        std::cerr << "[mesh_difference] Warning: mesh B is not closed ("
                  << sm_b.number_of_faces() << " faces).\n";
    }

    // First attempt: use the original meshes directly
    auto result = mesh_difference_impl(std::move(sm_a), std::move(sm_b));
    if (!result.empty()) {
        std::cerr << "[mesh_difference] Result: " << result.size()
                  << " faces.\n";
        return result;
    }

    // Direct boolean failed — try repair + retry regardless of cause
    // (non-closed meshes, self-intersections, etc.)
    std::cerr << "[mesh_difference] Direct boolean failed, "
              << "attempting repair + retry...\n";
    return mesh_difference_robust(mesh_a, mesh_b, a_closed, b_closed);
}

// ===========================================================================
// 6. Orient to Bound a Volume
// ===========================================================================

MeshData orient_volume(const MeshData& mesh) {
    if (mesh.empty()) return {};

    Surface_mesh sm = to_surface_mesh(mesh);
    if (sm.number_of_faces() == 0) return {};

    try {
        if (!CGAL::is_closed(sm)) {
            std::cerr << "[orient_volume] Warning: mesh is not closed; "
                      << "orient_to_bound_a_volume requires a closed mesh. "
                      << "Skipping.\n";
            return {};  // Skip to avoid CGAL assertion failure (abort) on non-closed input
        }

        PMP::orient_to_bound_a_volume(sm,
            CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, sm)));

        std::cerr << "[orient_volume] Oriented " << sm.number_of_faces()
                  << " faces.\n";
        return from_surface_mesh(sm);
    } catch (const std::exception& e) {
        std::cerr << "[orient_volume] " << e.what() << "\n";
        return {};
    }
}

// ===========================================================================
// 7. Boolean readiness check (closed + no self-intersections)
// ===========================================================================

bool is_boolean_ready(const MeshData& mesh) {
    if (mesh.empty()) return false;

    Surface_mesh sm = to_surface_mesh(mesh);
    if (sm.number_of_faces() == 0) return false;

    if (!CGAL::is_closed(sm)) return false;

    // does_self_intersect works with any kernel; performance is
    // acceptable for strand meshes (hundreds to low-thousands of tris).
    if (PMP::does_self_intersect(sm)) return false;

    return true;
}

// ===========================================================================
// Shared async helper
// ===========================================================================

namespace {

std::string make_temp_path(const std::string& suffix) {
    static std::atomic<unsigned> counter{0};
    auto dir = std::filesystem::temp_directory_path();
    auto ts  = std::chrono::steady_clock::now().time_since_epoch().count();
    auto name = "kgs_" + std::to_string(ts) + "_"
              + std::to_string(counter.fetch_add(1)) + suffix;
    return (dir / name).string();
}

void save_mesh(const MeshData& m, const std::string& path) {
    sinriv::kigstudio::voxel::saveMeshToBinarySTL(m, path);
}

MeshData load_mesh(const std::string& path) {
    MeshData m;
    auto gen = sinriv::kigstudio::voxel::readSTL(path);
    for (auto it = gen.begin(); it != gen.end(); ++it)
        m.push_back(std::move(*it));
    return m;
}

} // anonymous namespace

// ===========================================================================
// alpha_wrap_async
// ===========================================================================

alpha_wrap_async::alpha_wrap_async(const MeshData& mesh, double alpha, double offset)
    : alpha_(alpha), offset_(offset)
{
    if (mesh.empty())
        throw std::invalid_argument("alpha_wrap_async: mesh is empty");

    tmp_in_  = make_temp_path("_aw_in.stl");
    tmp_out_ = make_temp_path("_aw_out.stl");
    save_mesh(mesh, tmp_in_);

    std::vector<std::string> args = {
        "--tools", "--alphaWrap",
        "--in",  tmp_in_,
        "--out", tmp_out_,
        "--alpha",  std::to_string(alpha_),
        "--offset", std::to_string(offset_)};

    if (!process_.start(Process::self_exe_path(), args)) {
        std::remove(tmp_in_.c_str());
        throw std::runtime_error("alpha_wrap_async: failed to start subprocess");
    }
}

alpha_wrap_async::~alpha_wrap_async() { terminal(); }
bool alpha_wrap_async::done() const { return !process_.isRunning(); }

void alpha_wrap_async::terminal() {
    if (process_.isRunning()) process_.kill();
    if (!tmp_in_.empty())  { std::remove(tmp_in_.c_str());  tmp_in_.clear(); }
    if (!tmp_out_.empty()) { std::remove(tmp_out_.c_str()); tmp_out_.clear(); }
}

MeshData alpha_wrap_async::get_result() const {
    if (!done())
        throw std::runtime_error("alpha_wrap_async::get_result(): still running");
    if (result_ready_) return result_;
    if (!std::filesystem::exists(tmp_out_)) {
        throw std::runtime_error(
            "alpha_wrap_async::get_result(): subprocess did not produce output "
            "(alpha_wrap may have failed or produced an empty mesh); "
            "try smaller alpha/offset values");
    }
    result_ = load_mesh(tmp_out_);
    result_ready_ = true;
    return result_;
}

// ===========================================================================
// fill_holes_async
// ===========================================================================

fill_holes_async::fill_holes_async(const MeshData& mesh) {
    if (mesh.empty())
        throw std::invalid_argument("fill_holes_async: mesh is empty");

    tmp_in_  = make_temp_path("_fh_in.stl");
    tmp_out_ = make_temp_path("_fh_out.stl");
    save_mesh(mesh, tmp_in_);

    std::vector<std::string> args = {
        "--tools", "--fillHoles",
        "--in",  tmp_in_,
        "--out", tmp_out_};

    if (!process_.start(Process::self_exe_path(), args)) {
        std::remove(tmp_in_.c_str());
        throw std::runtime_error("fill_holes_async: failed to start subprocess");
    }
}

fill_holes_async::~fill_holes_async() { terminal(); }
bool fill_holes_async::done() const { return !process_.isRunning(); }

void fill_holes_async::terminal() {
    if (process_.isRunning()) process_.kill();
    if (!tmp_in_.empty())  { std::remove(tmp_in_.c_str());  tmp_in_.clear(); }
    if (!tmp_out_.empty()) { std::remove(tmp_out_.c_str()); tmp_out_.clear(); }
}

MeshData fill_holes_async::get_result() const {
    if (!done())
        throw std::runtime_error("fill_holes_async::get_result(): still running");
    if (result_ready_) return result_;
    if (!std::filesystem::exists(tmp_out_)) {
        throw std::runtime_error(
            "fill_holes_async::get_result(): subprocess did not produce output");
    }
    result_ = load_mesh(tmp_out_);
    result_ready_ = true;
    return result_;
}

// ===========================================================================
// stitch_borders_async
// ===========================================================================

stitch_borders_async::stitch_borders_async(const MeshData& mesh, double max_dist)
    : max_dist_(max_dist)
{
    if (mesh.empty())
        throw std::invalid_argument("stitch_borders_async: mesh is empty");

    tmp_in_  = make_temp_path("_sb_in.stl");
    tmp_out_ = make_temp_path("_sb_out.stl");
    save_mesh(mesh, tmp_in_);

    std::vector<std::string> args = {
        "--tools", "--stitchBorders",
        "--in",     tmp_in_,
        "--out",    tmp_out_,
        "--maxDist", std::to_string(max_dist_)};

    if (!process_.start(Process::self_exe_path(), args)) {
        std::remove(tmp_in_.c_str());
        throw std::runtime_error("stitch_borders_async: failed to start subprocess");
    }
}

stitch_borders_async::~stitch_borders_async() { terminal(); }
bool stitch_borders_async::done() const { return !process_.isRunning(); }

void stitch_borders_async::terminal() {
    if (process_.isRunning()) process_.kill();
    if (!tmp_in_.empty())  { std::remove(tmp_in_.c_str());  tmp_in_.clear(); }
    if (!tmp_out_.empty()) { std::remove(tmp_out_.c_str()); tmp_out_.clear(); }
}

MeshData stitch_borders_async::get_result() const {
    if (!done())
        throw std::runtime_error("stitch_borders_async::get_result(): still running");
    if (result_ready_) return result_;
    if (!std::filesystem::exists(tmp_out_)) {
        throw std::runtime_error(
            "stitch_borders_async::get_result(): subprocess did not produce output");
    }
    result_ = load_mesh(tmp_out_);
    result_ready_ = true;
    return result_;
}

// ===========================================================================
// merge_vertices_async
// ===========================================================================

merge_vertices_async::merge_vertices_async(const MeshData& mesh, double tol)
    : tol_(tol)
{
    if (mesh.empty())
        throw std::invalid_argument("merge_vertices_async: mesh is empty");

    tmp_in_  = make_temp_path("_mv_in.stl");
    tmp_out_ = make_temp_path("_mv_out.stl");
    save_mesh(mesh, tmp_in_);

    std::vector<std::string> args = {
        "--tools", "--mergeVertices",
        "--in",  tmp_in_,
        "--out", tmp_out_,
        "--tol", std::to_string(tol_)};

    if (!process_.start(Process::self_exe_path(), args)) {
        std::remove(tmp_in_.c_str());
        throw std::runtime_error("merge_vertices_async: failed to start subprocess");
    }
}

merge_vertices_async::~merge_vertices_async() { terminal(); }
bool merge_vertices_async::done() const { return !process_.isRunning(); }

void merge_vertices_async::terminal() {
    if (process_.isRunning()) process_.kill();
    if (!tmp_in_.empty())  { std::remove(tmp_in_.c_str());  tmp_in_.clear(); }
    if (!tmp_out_.empty()) { std::remove(tmp_out_.c_str()); tmp_out_.clear(); }
}

MeshData merge_vertices_async::get_result() const {
    if (!done())
        throw std::runtime_error("merge_vertices_async::get_result(): still running");
    if (result_ready_) return result_;
    if (!std::filesystem::exists(tmp_out_)) {
        throw std::runtime_error(
            "merge_vertices_async::get_result(): subprocess did not produce output");
    }
    result_ = load_mesh(tmp_out_);
    result_ready_ = true;
    return result_;
}

// ===========================================================================
// mesh_union_async
// ===========================================================================

mesh_union_async::mesh_union_async(const MeshData& mesh_a, const MeshData& mesh_b) {
    if (mesh_a.empty() || mesh_b.empty())
        throw std::invalid_argument("mesh_union_async: one or both meshes are empty");

    tmp_a_   = make_temp_path("_mu_a.stl");
    tmp_b_   = make_temp_path("_mu_b.stl");
    tmp_out_ = make_temp_path("_mu_out.stl");

    save_mesh(mesh_a, tmp_a_);
    save_mesh(mesh_b, tmp_b_);

    std::vector<std::string> args = {
        "--tools", "--meshUnion",
        "--inA", tmp_a_,
        "--inB", tmp_b_,
        "--out", tmp_out_};

    if (!process_.start(Process::self_exe_path(), args)) {
        std::remove(tmp_a_.c_str());
        std::remove(tmp_b_.c_str());
        throw std::runtime_error("mesh_union_async: failed to start subprocess");
    }
}

mesh_union_async::~mesh_union_async() { terminal(); }
bool mesh_union_async::done() const { return !process_.isRunning(); }

void mesh_union_async::terminal() {
    if (process_.isRunning()) process_.kill();
    if (!tmp_a_.empty())   { std::remove(tmp_a_.c_str());   tmp_a_.clear(); }
    if (!tmp_b_.empty())   { std::remove(tmp_b_.c_str());   tmp_b_.clear(); }
    if (!tmp_out_.empty()) { std::remove(tmp_out_.c_str()); tmp_out_.clear(); }
}

MeshData mesh_union_async::get_result() const {
    if (!done())
        throw std::runtime_error("mesh_union_async::get_result(): still running");
    if (result_ready_) return result_;
    if (!std::filesystem::exists(tmp_out_)) {
        throw std::runtime_error(
            "mesh_union_async::get_result(): subprocess did not produce output");
    }
    result_ = load_mesh(tmp_out_);
    result_ready_ = true;
    return result_;
}

// ===========================================================================
// orient_volume_async
// ===========================================================================

orient_volume_async::orient_volume_async(const MeshData& mesh) {
    if (mesh.empty())
        throw std::invalid_argument("orient_volume_async: mesh is empty");

    tmp_in_  = make_temp_path("_ov_in.stl");
    tmp_out_ = make_temp_path("_ov_out.stl");
    save_mesh(mesh, tmp_in_);

    std::vector<std::string> args = {
        "--tools", "--orientVolume",
        "--in",  tmp_in_,
        "--out", tmp_out_};

    if (!process_.start(Process::self_exe_path(), args)) {
        std::remove(tmp_in_.c_str());
        throw std::runtime_error("orient_volume_async: failed to start subprocess");
    }
}

orient_volume_async::~orient_volume_async() { terminal(); }
bool orient_volume_async::done() const { return !process_.isRunning(); }

void orient_volume_async::terminal() {
    if (process_.isRunning()) process_.kill();
    if (!tmp_in_.empty())  { std::remove(tmp_in_.c_str());  tmp_in_.clear(); }
    if (!tmp_out_.empty()) { std::remove(tmp_out_.c_str()); tmp_out_.clear(); }
}

MeshData orient_volume_async::get_result() const {
    if (!done())
        throw std::runtime_error("orient_volume_async::get_result(): still running");
    if (result_ready_) return result_;
    if (!std::filesystem::exists(tmp_out_)) {
        throw std::runtime_error(
            "orient_volume_async::get_result(): subprocess did not produce output");
    }
    result_ = load_mesh(tmp_out_);
    result_ready_ = true;
    return result_;
}

} // namespace sinriv::kigstudio::cgal
