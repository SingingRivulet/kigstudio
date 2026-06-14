#include "kigstudio/cgal/convex_hull.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>

#include <vector>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

namespace sinriv::kigstudio::cgal {

std::vector<Triangle> convexHull3(const std::vector<Triangle>& triangles) {
    if (triangles.empty()) {
        return {};
    }

    std::vector<Point_3> points;
    points.reserve(triangles.size() * 3);
    for (const auto& tri : triangles) {
        const auto& a = std::get<0>(tri);
        const auto& b = std::get<1>(tri);
        const auto& c = std::get<2>(tri);
        points.emplace_back(static_cast<double>(a.x),
                            static_cast<double>(a.y),
                            static_cast<double>(a.z));
        points.emplace_back(static_cast<double>(b.x),
                            static_cast<double>(b.y),
                            static_cast<double>(b.z));
        points.emplace_back(static_cast<double>(c.x),
                            static_cast<double>(c.y),
                            static_cast<double>(c.z));
    }

    Surface_mesh sm;
    CGAL::convex_hull_3(points.begin(), points.end(), sm);

    std::vector<Triangle> result;
    result.reserve(sm.number_of_faces());
    for (auto face : sm.faces()) {
        auto hc = sm.halfedge(face);
        std::vector<Point_3> face_points;
        for (auto v : sm.vertices_around_face(hc)) {
            face_points.push_back(sm.point(v));
        }
        if (face_points.size() == 3) {
            Triangle tri;
            std::get<0>(tri) = vec3f{
                static_cast<float>(face_points[0].x()),
                static_cast<float>(face_points[0].y()),
                static_cast<float>(face_points[0].z())};
            std::get<1>(tri) = vec3f{
                static_cast<float>(face_points[1].x()),
                static_cast<float>(face_points[1].y()),
                static_cast<float>(face_points[1].z())};
            std::get<2>(tri) = vec3f{
                static_cast<float>(face_points[2].x()),
                static_cast<float>(face_points[2].y()),
                static_cast<float>(face_points[2].z())};
            result.push_back(tri);
        }
    }
    return result;
}

}  // namespace sinriv::kigstudio::cgal
