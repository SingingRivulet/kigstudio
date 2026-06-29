#include "kigstudio/cgal/mesh_repair.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
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

typedef CGAL::Simple_cartesian<double> Kernel;
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
MeshData from_surface_mesh(const Surface_mesh& sm) {
    MeshData result;
    result.reserve(sm.number_of_faces());
    for (auto f : sm.faces()) {
        auto hd = sm.halfedge(f);
        auto p0 = sm.point(sm.source(hd));
        auto p1 = sm.point(sm.target(hd));
        auto p2 = sm.point(sm.target(sm.next(hd)));
        vec3f a(p0.x(), p0.y(), p0.z());
        vec3f b(p1.x(), p1.y(), p1.z());
        vec3f c(p2.x(), p2.y(), p2.z());
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
    if (mesh.empty()) return {};

    // Collect all points from the triangle soup
    std::vector<Point_3> points;
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
        Surface_mesh out;
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
// 5. Boolean Union
// ===========================================================================

MeshData mesh_union(const MeshData& mesh_a, const MeshData& mesh_b) {
    if (mesh_a.empty() || mesh_b.empty()) return {};

    Surface_mesh sm_a = to_surface_mesh(mesh_a);
    Surface_mesh sm_b = to_surface_mesh(mesh_b);

    if (sm_a.number_of_faces() == 0 || sm_b.number_of_faces() == 0)
        return {};

    try {
        // Ensure meshes are closed before boolean operations
        if (!CGAL::is_closed(sm_a)) {
            std::cerr << "[mesh_union] Warning: mesh A is not closed; "
                      << "result may be incomplete.\n";
        }
        if (!CGAL::is_closed(sm_b)) {
            std::cerr << "[mesh_union] Warning: mesh B is not closed; "
                      << "result may be incomplete.\n";
        }

        Surface_mesh out;
        bool valid = PMP::corefine_and_compute_union(sm_a, sm_b, out,
            CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, sm_a)),
            CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, sm_b)),
            CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, out)));

        if (!valid || out.number_of_faces() == 0) {
            std::cerr << "[mesh_union] Union produced empty or invalid mesh.\n";
            return {};
        }

        std::cerr << "[mesh_union] Result: " << out.number_of_vertices()
                  << " vertices, " << out.number_of_faces() << " faces.\n";
        return from_surface_mesh(out);
    } catch (const std::exception& e) {
        std::cerr << "[mesh_union] " << e.what() << "\n";
        return {};
    }
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
                      << "orient_to_bound_a_volume requires a closed mesh.\n";
            // Still attempt — PMP::orient_to_bound_a_volume works on closed
            // meshes; for open meshes we proceed with a best-effort approach.
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

    std::string cmd = "\"" + Process::self_exe_path() + "\""
                    + " --tools --alphaWrap"
                    + " --in \""    + tmp_in_  + "\""
                    + " --out \""   + tmp_out_ + "\""
                    + " --alpha "   + std::to_string(alpha_)
                    + " --offset "  + std::to_string(offset_);

    if (!process_.start(cmd)) {
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

    std::string cmd = "\"" + Process::self_exe_path() + "\""
                    + " --tools --fillHoles"
                    + " --in \""  + tmp_in_  + "\""
                    + " --out \"" + tmp_out_ + "\"";

    if (!process_.start(cmd)) {
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

    std::string cmd = "\"" + Process::self_exe_path() + "\""
                    + " --tools --stitchBorders"
                    + " --in \""     + tmp_in_  + "\""
                    + " --out \""    + tmp_out_ + "\""
                    + " --maxDist "  + std::to_string(max_dist_);

    if (!process_.start(cmd)) {
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

    std::string cmd = "\"" + Process::self_exe_path() + "\""
                    + " --tools --mergeVertices"
                    + " --in \""  + tmp_in_  + "\""
                    + " --out \"" + tmp_out_ + "\""
                    + " --tol "   + std::to_string(tol_);

    if (!process_.start(cmd)) {
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

    std::string cmd = "\"" + Process::self_exe_path() + "\""
                    + " --tools --meshUnion"
                    + " --inA \""  + tmp_a_   + "\""
                    + " --inB \""  + tmp_b_   + "\""
                    + " --out \""  + tmp_out_ + "\"";

    if (!process_.start(cmd)) {
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

    std::string cmd = "\"" + Process::self_exe_path() + "\""
                    + " --tools --orientVolume"
                    + " --in \""  + tmp_in_  + "\""
                    + " --out \"" + tmp_out_ + "\"";

    if (!process_.start(cmd)) {
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
    result_ = load_mesh(tmp_out_);
    result_ready_ = true;
    return result_;
}

} // namespace sinriv::kigstudio::cgal
