#include "kigstudio/sdf/sdf_mesh.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <variant>
#include "kigstudio/utils/vec3.h"
#include "kigstudio/voxel/voxel2mesh.h"

#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive_3.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/helpers.h>

namespace sinriv::kigstudio::sdf {

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Ray_3 = Kernel::Ray_3;
using Segment_3 = Kernel::Segment_3;
using Triangle_3 = Kernel::Triangle_3;
using Mesh = CGAL::Surface_mesh<Point_3>;
using SideTester = CGAL::Side_of_triangle_mesh<Mesh, Kernel>;
using Primitive =
    CGAL::AABB_triangle_primitive_3<Kernel, std::vector<Triangle_3>::iterator>;
using Traits = CGAL::AABB_traits_3<Kernel, Primitive>;
using Tree = CGAL::AABB_tree<Traits>;
namespace PMP = CGAL::Polygon_mesh_processing;

void buildMeshFromTriangles(std::vector<Triangle_3>& triangles, Mesh& mesh) {
    std::vector<Point_3> points;
    std::vector<std::vector<std::size_t>> polygons;

    points.reserve(triangles.size() * 3);
    polygons.reserve(triangles.size());

    for (const auto& tri : triangles) {
        std::size_t base = points.size();

        points.push_back(tri.vertex(0));
        points.push_back(tri.vertex(1));
        points.push_back(tri.vertex(2));

        polygons.push_back({base + 0, base + 1, base + 2});
    }

    PMP::merge_duplicate_points_in_polygon_soup(points, polygons);
    PMP::orient_polygon_soup(points, polygons);

    triangles.clear();
    triangles.reserve(polygons.size());

    for (const auto& poly : polygons) {
        triangles.emplace_back(points[poly[0]], points[poly[1]],
                               points[poly[2]]);
    }

    mesh.clear();
    PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh);
}

struct SDF_Mesh::Impl {
    std::vector<Triangle_3> triangles;
    Mesh mesh;
    Tree tree;
    std::unique_ptr<SideTester> side_tester;

    void clear() {
        tree.clear();
        side_tester.reset();
        mesh.clear();
        triangles.clear();
    }

    void rebuild() {
        mesh.clear();
        tree.clear();
        side_tester.reset();

        if (triangles.empty())
            return;

        buildMeshFromTriangles(triangles, mesh);

        tree.insert(triangles.begin(), triangles.end());
        tree.build();
        tree.accelerate_distance_queries();

        if (CGAL::is_closed(mesh)) {
            side_tester = std::make_unique<SideTester>(mesh);
        }
    }

    bool isInside(const Vec3f& p) const {
        if (!side_tester)
            return false;

        Point_3 query(p.x, p.y, p.z);
        return (*side_tester)(query) == CGAL::ON_BOUNDED_SIDE;
    }

    void getBatch(const Vec3f& begin,
                  const Vec3f& voxelSize,
                  const Vec3i& voxelCount,
                  std::vector<float>& out) const {
        const int sx = voxelCount.x;
        const int sy = voxelCount.y;
        const int sz = voxelCount.z;
        const size_t total =
            static_cast<size_t>(sx) * static_cast<size_t>(sy) *
            static_cast<size_t>(sz);
        out.resize(total);

        if (triangles.empty()) {
            std::fill(out.begin(), out.end(), 1e6f);
            return;
        }

#pragma omp parallel for
        for (int64_t i = 0; i < static_cast<int64_t>(total); ++i) {
            int x = static_cast<int>(i % sx);
            int y = static_cast<int>((i / sx) % sy);
            int z = static_cast<int>(i / (static_cast<int64_t>(sx) * sy));

            Point_3 query(begin.x + static_cast<float>(x) * voxelSize.x,
                          begin.y + static_cast<float>(y) * voxelSize.y,
                          begin.z + static_cast<float>(z) * voxelSize.z);
            double dist = std::sqrt(tree.squared_distance(query));
            out[static_cast<size_t>(i)] = static_cast<float>(dist);
        }

        if (!side_tester || sx <= 0 || sy <= 0 || sz <= 0)
            return;

        std::vector<uint8_t> inside_votes(total, 0);
        const auto index = [sx, sy](int x, int y, int z) {
            return (static_cast<size_t>(z) * sy + y) * sx + x;
        };

        const auto bbox = tree.bbox();

        auto get_intersections = [&](const Ray_3& ray, int axis,
                                     double unique_eps) {
            std::vector<Tree::Intersection_and_primitive_id<Ray_3>::Type>
                intersections;
            tree.all_intersections(ray, std::back_inserter(intersections));

            std::vector<double> coords;
            coords.reserve(intersections.size() * 2);
            for (const auto& item : intersections) {
                if (const auto* p = std::get_if<Point_3>(&item.first)) {
                    coords.push_back(axis == 0 ? p->x()
                                     : axis == 1 ? p->y()
                                                 : p->z());
                } else if (const auto* s = std::get_if<Segment_3>(&item.first)) {
                    coords.push_back(axis == 0 ? s->source().x()
                                     : axis == 1 ? s->source().y()
                                                 : s->source().z());
                    coords.push_back(axis == 0 ? s->target().x()
                                     : axis == 1 ? s->target().y()
                                                 : s->target().z());
                }
            }

            std::sort(coords.begin(), coords.end());
            coords.erase(std::unique(coords.begin(), coords.end(),
                                     [&](double a, double b) {
                                         return std::abs(a - b) <= unique_eps;
                                     }),
                         coords.end());
            return coords;
        };

        const double ray_x =
            bbox.xmin() - std::max<double>(std::abs(voxelSize.x), 1.0) * 2.0 -
            1e-6;
        const double eps_x =
            std::max<double>(std::abs(voxelSize.x) * 1e-5, 1e-7);

#pragma omp parallel for collapse(2)
        for (int z = 0; z < sz; ++z) {
            for (int y = 0; y < sy; ++y) {
                const double wy =
                    begin.y + static_cast<double>(y) * voxelSize.y;
                const double wz =
                    begin.z + static_cast<double>(z) * voxelSize.z;

                Ray_3 ray(Point_3(ray_x, wy, wz), Kernel::Direction_3(1, 0, 0));
                std::vector<double> xs = get_intersections(ray, 0, eps_x);

                bool inside = false;
                size_t hit = 0;
                for (int x = 0; x < sx; ++x) {
                    const double wx =
                        begin.x + static_cast<double>(x) * voxelSize.x;
                    while (hit < xs.size() && xs[hit] < wx - eps_x) {
                        inside = !inside;
                        ++hit;
                    }

                    if (inside) {
                        inside_votes[index(x, y, z)]++;
                    }
                }
            }
        }

        const double ray_y =
            bbox.ymin() - std::max<double>(std::abs(voxelSize.y), 1.0) * 2.0 -
            1e-6;
        const double eps_y =
            std::max<double>(std::abs(voxelSize.y) * 1e-5, 1e-7);

#pragma omp parallel for collapse(2)
        for (int z = 0; z < sz; ++z) {
            for (int x = 0; x < sx; ++x) {
                const double wx =
                    begin.x + static_cast<double>(x) * voxelSize.x;
                const double wz =
                    begin.z + static_cast<double>(z) * voxelSize.z;

                Ray_3 ray(Point_3(wx, ray_y, wz), Kernel::Direction_3(0, 1, 0));
                std::vector<double> ys = get_intersections(ray, 1, eps_y);

                bool inside = false;
                size_t hit = 0;
                for (int y = 0; y < sy; ++y) {
                    const double wy =
                        begin.y + static_cast<double>(y) * voxelSize.y;
                    while (hit < ys.size() && ys[hit] < wy - eps_y) {
                        inside = !inside;
                        ++hit;
                    }

                    if (inside) {
                        inside_votes[index(x, y, z)]++;
                    }
                }
            }
        }

        const double ray_z =
            bbox.zmin() - std::max<double>(std::abs(voxelSize.z), 1.0) * 2.0 -
            1e-6;
        const double eps_z =
            std::max<double>(std::abs(voxelSize.z) * 1e-5, 1e-7);

#pragma omp parallel for collapse(2)
        for (int y = 0; y < sy; ++y) {
            for (int x = 0; x < sx; ++x) {
                const double wx =
                    begin.x + static_cast<double>(x) * voxelSize.x;
                const double wy =
                    begin.y + static_cast<double>(y) * voxelSize.y;

                Ray_3 ray(Point_3(wx, wy, ray_z), Kernel::Direction_3(0, 0, 1));
                std::vector<double> zs = get_intersections(ray, 2, eps_z);

                bool inside = false;
                size_t hit = 0;
                for (int z = 0; z < sz; ++z) {
                    const double wz =
                        begin.z + static_cast<double>(z) * voxelSize.z;
                    while (hit < zs.size() && zs[hit] < wz - eps_z) {
                        inside = !inside;
                        ++hit;
                    }

                    if (inside) {
                        inside_votes[index(x, y, z)]++;
                    }
                }
            }
        }

#pragma omp parallel for
        for (int64_t i = 0; i < static_cast<int64_t>(total); ++i) {
            if (inside_votes[static_cast<size_t>(i)] >= 2) {
                int x = static_cast<int>(i % sx);
                int y = static_cast<int>((i / sx) % sy);
                int z = static_cast<int>(i / (static_cast<int64_t>(sx) * sy));

                Point_3 query(begin.x + static_cast<float>(x) * voxelSize.x,
                              begin.y + static_cast<float>(y) * voxelSize.y,
                              begin.z + static_cast<float>(z) * voxelSize.z);
                if ((*side_tester)(query) == CGAL::ON_BOUNDED_SIDE) {
                    out[static_cast<size_t>(i)] = -out[static_cast<size_t>(i)];
                }
            }
        }
    }
};

SDF_Mesh::SDF_Mesh() : impl(std::make_unique<Impl>()) {}

SDF_Mesh::~SDF_Mesh() = default;

bool SDF_Mesh::loadSTL(const std::string& filename) {
    path = filename;
    impl->clear();

    for (auto [tri, n] : sinriv::kigstudio::voxel::readSTL(filename)) {
        auto& [a, b, c] = tri;
        impl->triangles.emplace_back(Point_3(a.x, a.y, a.z),
                                     Point_3(b.x, b.y, b.z),
                                     Point_3(c.x, c.y, c.z));
    }

    if (impl->triangles.empty()) {
        return false;
    }
    impl->rebuild();
    return true;
}

bool SDF_Mesh::loadTriangles(const std::vector<Triangle>& triangles) {
    path.clear();
    impl->clear();

    for (const auto& tri : triangles) {
        const auto& a = std::get<0>(tri);
        const auto& b = std::get<1>(tri);
        const auto& c = std::get<2>(tri);
        impl->triangles.emplace_back(Point_3(a.x, a.y, a.z),
                                     Point_3(b.x, b.y, b.z),
                                     Point_3(c.x, c.y, c.z));
    }

    if (impl->triangles.empty()) {
        return false;
    }
    impl->rebuild();
    return true;
}

float SDF_Mesh::get(const Vec3f& p) const {
    if (impl->triangles.empty()) {
        return 1e6f;
    }

    Point_3 query(p.x, p.y, p.z);
    double sqdist = impl->tree.squared_distance(query);
    double dist = std::sqrt(sqdist);

    if (dist < 1e-6) {
        return 0.0f;
    }

    bool inside = impl->isInside(p);
    return inside ? -static_cast<float>(dist) : static_cast<float>(dist);
}

bool SDF_Mesh::isInside(const Vec3f& p) const {
    if (impl->triangles.empty()) {
        return false;
    }
    return impl->isInside(p);
}

bool SDF_Mesh::hasInsideTester() const {
    return impl->side_tester != nullptr;
}

void SDF_Mesh::get(const Vec3f& begin,
                   const Vec3f& voxelSize,
                   const Vec3i& voxelCount,
                   std::vector<float>& out) const {
    if (voxelCount.x <= 0 || voxelCount.y <= 0 || voxelCount.z <= 0) {
        out.clear();
        return;
    }

    impl->getBatch(begin, voxelSize, voxelCount, out);
}

std::string SDF_Mesh::getInfo(int indent) const {
    std::string prefix(indent * 2, ' ');
    return prefix + "SDF_Mesh(triangles=" + std::to_string(impl->triangles.size()) + ")";
}

cJSON* SDF_Mesh::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "mesh");

    if (!path.empty()) {
        cJSON_AddStringToObject(obj, "path", path.c_str());
    } else {
        cJSON* tri_array = cJSON_CreateArray();
        for (const auto& tri : impl->triangles) {
            cJSON* t = cJSON_CreateArray();
            cJSON_AddItemToArray(t, cJSON_CreateNumber(tri.vertex(0).x()));
            cJSON_AddItemToArray(t, cJSON_CreateNumber(tri.vertex(0).y()));
            cJSON_AddItemToArray(t, cJSON_CreateNumber(tri.vertex(0).z()));
            cJSON_AddItemToArray(t, cJSON_CreateNumber(tri.vertex(1).x()));
            cJSON_AddItemToArray(t, cJSON_CreateNumber(tri.vertex(1).y()));
            cJSON_AddItemToArray(t, cJSON_CreateNumber(tri.vertex(1).z()));
            cJSON_AddItemToArray(t, cJSON_CreateNumber(tri.vertex(2).x()));
            cJSON_AddItemToArray(t, cJSON_CreateNumber(tri.vertex(2).y()));
            cJSON_AddItemToArray(t, cJSON_CreateNumber(tri.vertex(2).z()));
            cJSON_AddItemToArray(tri_array, t);
        }
        cJSON_AddItemToObject(obj, "triangles", tri_array);
    }

    return obj;
}

void SDF_Mesh::fromJSON(const cJSON* json) {
    if (!json)
        return;

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, json) {
        if (!child->string)
            continue;

        if (cJSON_IsArray(child) && strcmp(child->string, "triangles") == 0) {
            impl->clear();
            int tri_count = cJSON_GetArraySize(child);
            for (int i = 0; i < tri_count; ++i) {
                const cJSON* t = cJSON_GetArrayItem(child, i);
                if (!t || !cJSON_IsArray(t) || cJSON_GetArraySize(t) != 9)
                    continue;
                double ax = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 0));
                double ay = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 1));
                double az = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 2));
                double bx = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 3));
                double by = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 4));
                double bz = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 5));
                double cx = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 6));
                double cy = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 7));
                double cz = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 8));
                impl->triangles.emplace_back(Point_3(ax, ay, az),
                                             Point_3(bx, by, bz),
                                             Point_3(cx, cy, cz));
            }
            if (!impl->triangles.empty()) {
                impl->rebuild();
            }
            path.clear();
        } else if (cJSON_IsString(child) &&
                   strcmp(child->string, "path") == 0) {
            if (child->valuestring) {
                path = child->valuestring;
                loadSTL(path);
            }
        }
    }
}

// ============================================================
// Static registration
// ============================================================

static bool _register_mesh_type = []() {
    sdf_register_type("mesh",
                      [](const cJSON* json) -> std::shared_ptr<SDFBase> {
                          auto obj = std::make_shared<SDF_Mesh>();
                          obj->fromJSON(json);
                          return obj;
                      });
    return true;
}();

}  // namespace sinriv::kigstudio::sdf
