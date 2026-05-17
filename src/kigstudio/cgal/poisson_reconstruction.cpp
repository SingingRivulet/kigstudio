#include "kigstudio/cgal/poisson_reconstruction.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/property_map.h>
#include <iostream>

typedef CGAL::Simple_cartesian<double> Kernel;
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

    typedef CGAL::Surface_mesh<Point_3> Surface_mesh;
    Surface_mesh output_mesh;

    bool ok = CGAL::poisson_surface_reconstruction_delaunay(
        points.begin(), points.end(),
        CGAL::First_of_pair_property_map<PointWithNormal>(),
        CGAL::Second_of_pair_property_map<PointWithNormal>(),
        output_mesh,
        spacing);

    if (!ok) {
        std::cerr << "[CGAL Poisson] Reconstruction failed.\n";
        return {};
    }

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
