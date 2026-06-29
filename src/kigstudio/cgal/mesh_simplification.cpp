#include "kigstudio/cgal/mesh_simplification.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_ratio_stop_predicate.h>
#include <iostream>
#include <unordered_map>
#include <filesystem>
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <cstdio>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

namespace SMS = CGAL::Surface_mesh_simplification;

namespace sinriv::kigstudio::cgal {

std::vector<std::tuple<Triangle, vec3f>>
simplifyMesh(const std::vector<std::tuple<Triangle, vec3f>>& mesh, double ratio)
{
    if (mesh.empty()) {
        return {};
    }

    ratio = std::max(0.0, std::min(1.0, ratio));

    Surface_mesh sm;

    // Weld vertices to build indexed mesh
    struct VKey {
        int64_t x, y, z;
        bool operator==(const VKey& o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };
    struct VKeyHash {
        size_t operator()(const VKey& k) const {
            // Use distinct mixing constants for each component to avoid collisions
            return std::hash<int64_t>()(k.x
                ^ (k.y * 0x9e3779b97f4a7c15ULL)
                ^ (k.z * 0x9e3779b97f4a7c16ULL));
        }
    };

    std::unordered_map<VKey, Surface_mesh::Vertex_index, VKeyHash> vertex_map;
    auto get_vertex = [&](const vec3f& p) -> Surface_mesh::Vertex_index {
        VKey key{
            static_cast<int64_t>(std::round(p.x * 1e6f)),
            static_cast<int64_t>(std::round(p.y * 1e6f)),
            static_cast<int64_t>(std::round(p.z * 1e6f))
        };
        auto it = vertex_map.find(key);
        if (it != vertex_map.end()) {
            return it->second;
        }
        Surface_mesh::Vertex_index v = sm.add_vertex(Point_3(static_cast<double>(p.x),
                                                               static_cast<double>(p.y),
                                                               static_cast<double>(p.z)));
        vertex_map.emplace(key, v);
        return v;
    };

    for (const auto& [tri, n] : mesh) {
        auto a = std::get<0>(tri);
        auto b = std::get<1>(tri);
        auto c = std::get<2>(tri);

        auto va = get_vertex(a);
        auto vb = get_vertex(b);
        auto vc = get_vertex(c);

        // Skip degenerate faces
        if (va == vb || vb == vc || vc == va) {
            continue;
        }

        Surface_mesh::Face_index f = sm.add_face(va, vb, vc);
        if (f == Surface_mesh::null_face()) {
            // Non-manifold or duplicate face; skip
            continue;
        }
    }

    if (sm.number_of_faces() == 0) {
        std::cerr << "[CGAL Simplify] No valid faces after conversion.\n";
        return {};
    }

    std::cerr << "[CGAL Simplify] Before: " << sm.number_of_vertices()
              << " vertices, " << sm.number_of_faces() << " faces.\n";

    try {
        SMS::Edge_count_ratio_stop_predicate<Surface_mesh> stop(ratio);
        int r = SMS::edge_collapse(sm, stop,
            CGAL::parameters::vertex_index_map(get(CGAL::vertex_index, sm)));

        std::cerr << "[CGAL Simplify] Collapsed " << r << " edges. After: "
                  << sm.number_of_vertices() << " vertices, "
                  << sm.number_of_faces() << " faces.\n";
    } catch (const std::exception& e) {
        std::cerr << "[CGAL Simplify] Exception during edge_collapse: " << e.what() << "\n";
        // Return un-simplified mesh on failure
        std::vector<std::tuple<Triangle, vec3f>> result;
        result.reserve(sm.number_of_faces());
        for (auto f : sm.faces()) {
            auto hd = sm.halfedge(f);
            auto p0 = sm.point(sm.source(hd));
            auto p1 = sm.point(sm.target(hd));
            auto p2 = sm.point(sm.target(sm.next(hd)));
            vec3f a(static_cast<float>(p0.x()), static_cast<float>(p0.y()), static_cast<float>(p0.z()));
            vec3f b(static_cast<float>(p1.x()), static_cast<float>(p1.y()), static_cast<float>(p1.z()));
            vec3f c(static_cast<float>(p2.x()), static_cast<float>(p2.y()), static_cast<float>(p2.z()));
            vec3f n = (b - a).cross(c - a).normalize();
            result.push_back({std::make_tuple(a, b, c), n});
        }
        return result;
    }

    std::vector<std::tuple<Triangle, vec3f>> result;
    result.reserve(sm.number_of_faces());

    for (auto f : sm.faces()) {
        auto hd = sm.halfedge(f);
        auto p0 = sm.point(sm.source(hd));
        auto p1 = sm.point(sm.target(hd));
        auto p2 = sm.point(sm.target(sm.next(hd)));

        vec3f a(static_cast<float>(p0.x()),
                static_cast<float>(p0.y()),
                static_cast<float>(p0.z()));
        vec3f b(static_cast<float>(p1.x()),
                static_cast<float>(p1.y()),
                static_cast<float>(p1.z()));
        vec3f c(static_cast<float>(p2.x()),
                static_cast<float>(p2.y()),
                static_cast<float>(p2.z()));
        vec3f n = (b - a).cross(c - a).normalize();
        result.push_back({std::make_tuple(a, b, c), n});
    }

    return result;
}

// ---------------------------------------------------------------------------
// simplifyMesh_async — subprocess helpers
// ---------------------------------------------------------------------------

std::string simplifyMesh_async::make_temp_path(const std::string& suffix) {
    static std::atomic<unsigned> counter{0};
    auto dir = std::filesystem::temp_directory_path();
    auto ts  = std::chrono::steady_clock::now().time_since_epoch().count();
    auto name = "kgs_" + std::to_string(ts) + "_"
              + std::to_string(counter.fetch_add(1)) + suffix;
    return (dir / name).string();
}

simplifyMesh_async::simplifyMesh_async(
    const std::vector<std::tuple<Triangle, vec3f>>& mesh, double ratio)
{
    if (mesh.empty())
        throw std::invalid_argument("simplifyMesh_async: mesh is empty");

    tmp_in_  = make_temp_path("_in.stl");
    tmp_out_ = make_temp_path("_out.stl");

    // Serialise input mesh to temporary STL
    sinriv::kigstudio::voxel::saveMeshToBinarySTL(mesh, tmp_in_);

    // Build the command line: "<self> --tools --simplifyMesh --in ... --out ... --ratio ..."
    std::string cmd = "\"" + Process::self_exe_path() + "\""
                    + " --tools --simplifyMesh"
                    + " --in \""  + tmp_in_  + "\""
                    + " --out \"" + tmp_out_ + "\""
                    + " --ratio " + std::to_string(ratio);

    if (!process_.start(cmd)) {
        std::remove(tmp_in_.c_str());
        throw std::runtime_error("simplifyMesh_async: failed to start subprocess: " + cmd);
    }
}

simplifyMesh_async::~simplifyMesh_async() {
    terminal();
}

bool simplifyMesh_async::done() const {
    return !process_.isRunning();
}

void simplifyMesh_async::terminal() {
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

std::vector<std::tuple<Triangle, vec3f>>
simplifyMesh_async::get_result() const {
    if (!done()) {
        throw std::runtime_error("simplifyMesh_async::get_result(): process still running");
    }

    if (result_ready_)
        return result_;

    // Read back the simplified STL
    auto gen = sinriv::kigstudio::voxel::readSTL(tmp_out_);
    result_.clear();
    for (auto it = gen.begin(); it != gen.end(); ++it) {
        result_.push_back(std::move(*it));
    }
    result_ready_ = true;
    return result_;
}

} // namespace sinriv::kigstudio::cgal
