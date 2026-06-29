#pragma once
#include <vector>
#include <tuple>
#include <string>
#include "kigstudio/voxel/voxel2mesh.h"
#include "kigstudio/utils/process.h"

namespace sinriv::kigstudio::cgal {

using vec3f = sinriv::kigstudio::vec3<float>;
using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;

/**
 * Simplify a triangle mesh using CGAL Surface Mesh Simplification (edge collapse).
 *
 * @param mesh   Input triangle mesh as vector of {triangle, normal}.
 * @param ratio  Target ratio of remaining edges (0.0 ~ 1.0). E.g. 0.1 keeps 10%.
 * @return       Simplified mesh. Empty on failure.
 */
std::vector<std::tuple<Triangle, vec3f>>
simplifyMesh(const std::vector<std::tuple<Triangle, vec3f>>& mesh, double ratio);

/**
 * Asynchronous mesh simplification — runs simplifyMesh in a subprocess.
 *
 * Internally serialises the mesh to a temporary STL file, invokes the CLI
 * tool mode of this same executable, then reads back the result.
 *
 * Usage:
 *   simplifyMesh_async async(mesh, 0.1);
 *   while (!async.done()) { ... }
 *   auto result = async.get_result();   // throws if still running
 */
class simplifyMesh_async {
public:
    simplifyMesh_async(const std::vector<std::tuple<Triangle, vec3f>>& mesh,
                       double ratio);
    ~simplifyMesh_async();

    bool done() const;
    void terminal();
    std::vector<std::tuple<Triangle, vec3f>> get_result() const;

private:
    static std::string make_temp_path(const std::string& suffix);

    Process process_;
    std::string tmp_in_;
    std::string tmp_out_;
    mutable std::vector<std::tuple<Triangle, vec3f>> result_;
    mutable bool result_ready_ = false;
};

} // namespace sinriv::kigstudio::cgal
