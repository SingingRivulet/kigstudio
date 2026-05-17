#include "kigstudio/cgal/skeleton_extraction.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>
#include <CGAL/boost/graph/helpers.h>
#include <iostream>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

namespace sinriv::kigstudio::cgal {

std::vector<std::pair<vec3f, vec3f>>
extractSkeletonFromMesh(const std::vector<Triangle>& triangles)
{
    Surface_mesh mesh;
    for (const auto& tri : triangles) {
        auto a = std::get<0>(tri);
        auto b = std::get<1>(tri);
        auto c = std::get<2>(tri);

        auto va = mesh.add_vertex(Point_3(static_cast<double>(a.x),
                                          static_cast<double>(a.y),
                                          static_cast<double>(a.z)));
        auto vb = mesh.add_vertex(Point_3(static_cast<double>(b.x),
                                          static_cast<double>(b.y),
                                          static_cast<double>(b.z)));
        auto vc = mesh.add_vertex(Point_3(static_cast<double>(c.x),
                                          static_cast<double>(c.y),
                                          static_cast<double>(c.z)));
        mesh.add_face(va, vb, vc);
    }

    if (!CGAL::is_closed(mesh)) {
        std::cerr << "[CGAL Skeleton] Mesh is not closed, aborting CGAL path.\n";
        return {};
    }

    try {
        CGAL::Mean_curvature_flow_skeletonization<Surface_mesh>::Skeleton skeleton;
        CGAL::extract_mean_curvature_flow_skeleton(mesh, skeleton);

        std::vector<std::pair<vec3f, vec3f>> lines;
        auto edge_range = edges(skeleton);
        for (auto ei = edge_range.first; ei != edge_range.second; ++ei) {
            auto e = *ei;
            auto src = source(e, skeleton);
            auto tgt = target(e, skeleton);
            auto p1 = skeleton[src].point;
            auto p2 = skeleton[tgt].point;
            lines.push_back({
                vec3f(static_cast<float>(p1.x()),
                      static_cast<float>(p1.y()),
                      static_cast<float>(p1.z())),
                vec3f(static_cast<float>(p2.x()),
                      static_cast<float>(p2.y()),
                      static_cast<float>(p2.z()))
            });
        }
        return lines;
    } catch (const std::exception& e) {
        std::cerr << "[CGAL Skeleton] Exception during extraction: " << e.what() << "\n";
        return {};
    }
}

} // namespace sinriv::kigstudio::cgal
