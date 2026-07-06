#include "kigstudio/cgal/mesh_subdivision.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

#include <iostream>
#include <vector>
#include <unordered_map>
#include <filesystem>
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <cstdio>
#include <cmath>
#include <limits>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

namespace PMP = CGAL::Polygon_mesh_processing;

namespace sinriv::kigstudio::cgal {

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

MeshData subdivideMesh(const MeshData& mesh, double target_length,
                       unsigned int iterations)
{
    if (mesh.empty()) {
        return {};
    }
    if (target_length <= 0.0 || !std::isfinite(target_length)) {
        std::cerr << "[CGAL Subdivide] Error: target_length must be positive.\n";
        return {};
    }
    if (iterations == 0) {
        return mesh;
    }

    Surface_mesh sm = to_surface_mesh(mesh);
    if (sm.number_of_faces() == 0) {
        std::cerr << "[CGAL Subdivide] No valid faces after conversion.\n";
        return {};
    }

    // Ensure the mesh is triangulated (faces should already be triangles)
    if (!CGAL::is_triangle_mesh(sm)) {
        try {
            PMP::triangulate_faces(sm);
        } catch (const std::exception& e) {
            std::cerr << "[CGAL Subdivide] triangulate_faces failed: "
                      << e.what() << "\n";
            return {};
        }
    }

    std::cerr << "[CGAL Subdivide] Before: " << sm.number_of_vertices()
              << " vertices, " << sm.number_of_faces() << " faces.\n";

    try {
        PMP::isotropic_remeshing(
            sm.faces(),
            target_length,
            sm,
            CGAL::parameters::number_of_iterations(iterations));

        sm.collect_garbage();

        std::cerr << "[CGAL Subdivide] After: " << sm.number_of_vertices()
                  << " vertices, " << sm.number_of_faces() << " faces.\n";
    } catch (const std::exception& e) {
        std::cerr << "[CGAL Subdivide] Exception during remeshing: "
                  << e.what() << "\n";
        return {};
    }

    return from_surface_mesh(sm);
}

double subdivideMeshTargetLength(const MeshData& mesh, int level)
{
    if (level < 1) {
        std::cerr << "[CGAL Subdivide] Error: level must be >= 1.\n";
        return 0.0;
    }
    if (mesh.empty()) {
        return 0.0;
    }

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    double max_z = std::numeric_limits<double>::lowest();

    for (const auto& [tri, n] : mesh) {
        for (const auto& v : {std::get<0>(tri), std::get<1>(tri), std::get<2>(tri)}) {
            min_x = std::min(min_x, static_cast<double>(v.x));
            min_y = std::min(min_y, static_cast<double>(v.y));
            min_z = std::min(min_z, static_cast<double>(v.z));
            max_x = std::max(max_x, static_cast<double>(v.x));
            max_y = std::max(max_y, static_cast<double>(v.y));
            max_z = std::max(max_z, static_cast<double>(v.z));
        }
    }

    double dx = max_x - min_x;
    double dy = max_y - min_y;
    double dz = max_z - min_z;
    double diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);

    // level 1 -> diagonal / 4, level 2 -> diagonal / 8, ...
    double divisor = 4.0 * static_cast<double>(1ULL << (level - 1));
    double target = diagonal / divisor;

    // Clamp to a sane minimum to avoid runaway subdivision.
    constexpr double kMinTarget = 1e-6;
    return std::max(target, kMinTarget);
}

MeshData subdivideMeshByLevel(const MeshData& mesh, int level,
                              unsigned int iterations)
{
    double target_length = subdivideMeshTargetLength(mesh, level);
    if (target_length <= 0.0) {
        return {};
    }
    return subdivideMesh(mesh, target_length, iterations);
}

// ---------------------------------------------------------------------------
// subdivideMesh_async — subprocess helpers
// ---------------------------------------------------------------------------

std::string subdivideMesh_async::make_temp_path(const std::string& suffix) {
    static std::atomic<unsigned> counter{0};
    auto dir = std::filesystem::temp_directory_path();
    auto ts  = std::chrono::steady_clock::now().time_since_epoch().count();
    auto name = "kgs_" + std::to_string(ts) + "_"
              + std::to_string(counter.fetch_add(1)) + suffix;
    return (dir / name).string();
}

subdivideMesh_async::subdivideMesh_async(const MeshData& mesh,
                                         double target_length,
                                         unsigned int iterations)
    : target_length_(target_length), iterations_(iterations)
{
    if (mesh.empty())
        throw std::invalid_argument("subdivideMesh_async: mesh is empty");
    if (target_length <= 0.0 || !std::isfinite(target_length))
        throw std::invalid_argument("subdivideMesh_async: target_length must be positive");

    tmp_in_  = make_temp_path("_in.stl");
    tmp_out_ = make_temp_path("_out.stl");

    // Serialise input mesh to temporary STL
    sinriv::kigstudio::voxel::saveMeshToBinarySTL(mesh, tmp_in_);

    // Build argument list: <self> --tools --subdivideMesh --in ... --out ... --targetLength ... --iterations ...
    std::vector<std::string> args = {
        "--tools", "--subdivideMesh",
        "--in",  tmp_in_,
        "--out", tmp_out_,
        "--targetLength", std::to_string(target_length),
        "--iterations", std::to_string(iterations)};

    if (!process_.start(Process::self_exe_path(), args)) {
        std::remove(tmp_in_.c_str());
        throw std::runtime_error(
            "subdivideMesh_async: failed to start subprocess");
    }
}

subdivideMesh_async::~subdivideMesh_async() {
    terminal();
}

bool subdivideMesh_async::done() const {
    return !process_.isRunning();
}

void subdivideMesh_async::terminal() {
    if (process_.isRunning()) {
        process_.kill();
    }
    if (!tmp_in_.empty()) {
        std::remove(tmp_in_.c_str());
        tmp_in_.clear();
    }
    if (!tmp_out_.empty()) {
        std::remove(tmp_out_.c_str());
        tmp_out_.clear();
    }
}

MeshData subdivideMesh_async::get_result() const {
    if (!done()) {
        throw std::runtime_error("subdivideMesh_async::get_result(): process still running");
    }

    if (result_ready_)
        return result_;

    // Read back the remeshed STL
    auto gen = sinriv::kigstudio::voxel::readSTL(tmp_out_);
    result_.clear();
    for (auto it = gen.begin(); it != gen.end(); ++it) {
        result_.push_back(std::move(*it));
    }
    result_ready_ = true;
    return result_;
}

// ---------------------------------------------------------------------------
// subdivideMeshByLevel_async
// ---------------------------------------------------------------------------

std::string subdivideMeshByLevel_async::make_temp_path(const std::string& suffix) {
    static std::atomic<unsigned> counter{0};
    auto dir = std::filesystem::temp_directory_path();
    auto ts  = std::chrono::steady_clock::now().time_since_epoch().count();
    auto name = "kgs_" + std::to_string(ts) + "_"
              + std::to_string(counter.fetch_add(1)) + suffix;
    return (dir / name).string();
}

subdivideMeshByLevel_async::subdivideMeshByLevel_async(const MeshData& mesh,
                                                       int level,
                                                       unsigned int iterations)
    : level_(level), iterations_(iterations)
{
    if (mesh.empty())
        throw std::invalid_argument("subdivideMeshByLevel_async: mesh is empty");
    if (level < 1)
        throw std::invalid_argument("subdivideMeshByLevel_async: level must be >= 1");

    double target_length = subdivideMeshTargetLength(mesh, level);
    if (target_length <= 0.0)
        throw std::runtime_error("subdivideMeshByLevel_async: failed to compute target length");

    tmp_in_  = make_temp_path("_in.stl");
    tmp_out_ = make_temp_path("_out.stl");

    sinriv::kigstudio::voxel::saveMeshToBinarySTL(mesh, tmp_in_);

    std::vector<std::string> args = {
        "--tools", "--subdivideMesh",
        "--in",  tmp_in_,
        "--out", tmp_out_,
        "--targetLength", std::to_string(target_length),
        "--iterations", std::to_string(iterations)};

    if (!process_.start(Process::self_exe_path(), args)) {
        std::remove(tmp_in_.c_str());
        throw std::runtime_error(
            "subdivideMeshByLevel_async: failed to start subprocess");
    }
}

subdivideMeshByLevel_async::~subdivideMeshByLevel_async() {
    terminal();
}

bool subdivideMeshByLevel_async::done() const {
    return !process_.isRunning();
}

void subdivideMeshByLevel_async::terminal() {
    if (process_.isRunning()) {
        process_.kill();
    }
    if (!tmp_in_.empty()) {
        std::remove(tmp_in_.c_str());
        tmp_in_.clear();
    }
    if (!tmp_out_.empty()) {
        std::remove(tmp_out_.c_str());
        tmp_out_.clear();
    }
}

MeshData subdivideMeshByLevel_async::get_result() const {
    if (!done()) {
        throw std::runtime_error("subdivideMeshByLevel_async::get_result(): process still running");
    }

    if (result_ready_)
        return result_;

    auto gen = sinriv::kigstudio::voxel::readSTL(tmp_out_);
    result_.clear();
    for (auto it = gen.begin(); it != gen.end(); ++it) {
        result_.push_back(std::move(*it));
    }
    result_ready_ = true;
    return result_;
}

} // namespace sinriv::kigstudio::cgal
