#include "kigstudio/sdf/sdf_mesh.h"
#include "kigstudio/voxel/voxel2mesh.h"
#include "kigstudio/utils/vec3.h"
#include <cmath>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_triangle_primitive_3.h>

namespace sinriv::kigstudio::sdf {

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Triangle_3 = Kernel::Triangle_3;
using Primitive = CGAL::AABB_triangle_primitive_3<Kernel, std::vector<Triangle_3>::iterator>;
using Traits = CGAL::AABB_traits_3<Kernel, Primitive>;
using Tree = CGAL::AABB_tree<Traits>;

struct SDF_Mesh::Impl {
    std::vector<Triangle_3> triangles;
    Tree tree;

    bool isInside(const Vec3f& p) const {
        if (triangles.empty())
            return false;

        Point_3 query(p.x, p.y, p.z);

        Kernel::Ray_3 ray_x(query, Kernel::Direction_3(1, 0, 0));
        Kernel::Ray_3 ray_y(query, Kernel::Direction_3(0, 1, 0));
        Kernel::Ray_3 ray_z(query, Kernel::Direction_3(0, 0, 1));

        std::size_t count_x = tree.number_of_intersected_primitives(ray_x);
        std::size_t count_y = tree.number_of_intersected_primitives(ray_y);
        std::size_t count_z = tree.number_of_intersected_primitives(ray_z);

        bool inside_x = (count_x % 2) == 1;
        bool inside_y = (count_y % 2) == 1;
        bool inside_z = (count_z % 2) == 1;

        int inside_count = (inside_x ? 1 : 0) + (inside_y ? 1 : 0) + (inside_z ? 1 : 0);
        return inside_count >= 2;
    }
};

SDF_Mesh::SDF_Mesh() : impl(std::make_unique<Impl>()) {}

SDF_Mesh::~SDF_Mesh() = default;

bool SDF_Mesh::loadSTL(const std::string& filename) {
    path = filename;
    impl->triangles.clear();
    impl->tree.clear();

    for (auto [tri, n] : sinriv::kigstudio::voxel::readSTL(filename)) {
        auto& [a, b, c] = tri;
        impl->triangles.emplace_back(
            Point_3(a.x, a.y, a.z),
            Point_3(b.x, b.y, b.z),
            Point_3(c.x, c.y, c.z));
    }

    if (impl->triangles.empty()) {
        return false;
    }

    impl->tree.insert(impl->triangles.begin(), impl->triangles.end());
    impl->tree.build();
    impl->tree.accelerate_distance_queries();
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

std::string SDF_Mesh::getInfo() const {
    return "SDF_Mesh(triangles=" + std::to_string(impl->triangles.size()) + ")";
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
    const cJSON* tri_array = cJSON_GetObjectItem(json, "triangles");
    if (tri_array && cJSON_IsArray(tri_array)) {
        impl->triangles.clear();
        impl->tree.clear();
        int tri_count = cJSON_GetArraySize(tri_array);
        for (int i = 0; i < tri_count; ++i) {
            cJSON* t = cJSON_GetArrayItem(tri_array, i);
            if (!t || !cJSON_IsArray(t) || cJSON_GetArraySize(t) != 9) continue;
            double ax = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 0));
            double ay = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 1));
            double az = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 2));
            double bx = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 3));
            double by = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 4));
            double bz = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 5));
            double cx = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 6));
            double cy = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 7));
            double cz = cJSON_GetNumberValue(cJSON_GetArrayItem(t, 8));
            impl->triangles.emplace_back(
                Point_3(ax, ay, az),
                Point_3(bx, by, bz),
                Point_3(cx, cy, cz));
        }
        if (!impl->triangles.empty()) {
            impl->tree.insert(impl->triangles.begin(), impl->triangles.end());
            impl->tree.build();
            impl->tree.accelerate_distance_queries();
        }
        path.clear();
    } else {
        const char* p = cJSON_GetStringValue(cJSON_GetObjectItem(json, "path"));
        if (p) {
            path = p;
            loadSTL(path);
        }
    }
}

// ============================================================
// Static registration
// ============================================================

static bool _register_mesh_type = []() {
    sdf_register_type("mesh", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDF_Mesh>();
        obj->fromJSON(json);
        return obj;
    });
    return true;
}();

}  // namespace sinriv::kigstudio::sdf
