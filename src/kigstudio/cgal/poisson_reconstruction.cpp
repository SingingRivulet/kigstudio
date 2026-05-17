#include "kigstudio/cgal/poisson_reconstruction.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/property_map.h>
#include <iostream>
#include <cmath>
#include <algorithm>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;

namespace sinriv::kigstudio::cgal {

std::vector<std::tuple<Triangle, vec3f>>
poissonReconstruct(const std::vector<std::pair<vec3f, vec3f>>& points_with_normals,
                   double spacing)
{
    if (points_with_normals.empty()) {
        return {};
    }

    typedef std::pair<Point_3, Vector_3> PointWithNormal;
    std::vector<PointWithNormal> points;
    points.reserve(points_with_normals.size());
    for (const auto& [p, n] : points_with_normals) {
        points.push_back({
            Point_3(static_cast<double>(p.x),
                    static_cast<double>(p.y),
                    static_cast<double>(p.z)),
            Vector_3(static_cast<double>(n.x),
                     static_cast<double>(n.y),
                     static_cast<double>(n.z))
        });
    }

    // Compute a reasonable spacing based on point cloud density
    Point_3 min_pt = points[0].first;
    Point_3 max_pt = points[0].first;
    for (const auto& pn : points) {
        const auto& p = pn.first;
        min_pt = Point_3(std::min(min_pt.x(), p.x()),
                         std::min(min_pt.y(), p.y()),
                         std::min(min_pt.z(), p.z()));
        max_pt = Point_3(std::max(max_pt.x(), p.x()),
                         std::max(max_pt.y(), p.y()),
                         std::max(max_pt.z(), p.z()));
    }
    double dx = max_pt.x() - min_pt.x();
    double dy = max_pt.y() - min_pt.y();
    double dz = max_pt.z() - min_pt.z();
    double diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);

    double avg_spacing = diagonal / std::cbrt(static_cast<double>(points.size()));
    if (avg_spacing < 1e-6) {
        avg_spacing = 0.1;
    }

    // Use caller-provided spacing if reasonable; otherwise fall back to avg_spacing
    double effective_spacing = spacing;
    if (effective_spacing < avg_spacing * 0.05 || effective_spacing > avg_spacing * 20.0) {
        effective_spacing = avg_spacing;
    }

    std::cerr << "[CGAL Poisson] Input points: " << points.size()
              << ", bbox diagonal: " << diagonal
              << ", avg_spacing: " << avg_spacing
              << ", effective_spacing: " << effective_spacing << "\n";

    typedef CGAL::Surface_mesh<Point_3> Surface_mesh;
    Surface_mesh output_mesh;

    bool ok = CGAL::poisson_surface_reconstruction_delaunay(
        points.begin(), points.end(),
        CGAL::First_of_pair_property_map<PointWithNormal>(),
        CGAL::Second_of_pair_property_map<PointWithNormal>(),
        output_mesh,
        effective_spacing);

    if (!ok) {
        std::cerr << "[CGAL Poisson] Reconstruction failed.\n";
        return {};
    }

    std::cerr << "[CGAL Poisson] Output faces: " << output_mesh.number_of_faces() << "\n";

    std::vector<std::tuple<Triangle, vec3f>> result;
    for (auto f : output_mesh.faces()) {
        auto hd = output_mesh.halfedge(f);
        auto p0 = output_mesh.point(output_mesh.source(hd));
        auto p1 = output_mesh.point(output_mesh.target(hd));
        auto p2 = output_mesh.point(
            output_mesh.target(output_mesh.next(hd)));

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

} // namespace sinriv::kigstudio::cgal
