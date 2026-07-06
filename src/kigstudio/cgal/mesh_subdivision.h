#pragma once
#include <string>
#include <tuple>
#include <vector>
#include "kigstudio/utils/process.h"
#include "kigstudio/voxel/voxel2mesh.h"

namespace sinriv::kigstudio::cgal {

using vec3f = sinriv::kigstudio::vec3<float>;
using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;
using MeshData = std::vector<std::tuple<Triangle, vec3f>>;

/**
 * Subdivide / refine a triangle mesh using CGAL isotropic remeshing.
 *
 * This repeatedly splits long edges, collapses short edges, flips edges and
 * applies tangential relaxation to produce a denser, smoother triangle mesh
 * with approximately the requested target edge length.
 *
 * @param mesh            Input triangle mesh as vector of {triangle, normal}.
 * @param target_length   Desired target edge length. Must be > 0.
 *                        Smaller values produce finer meshes.
 * @param iterations      Number of remeshing iterations (default 3).
 * @return                Refined mesh. Empty on failure.
 */
MeshData subdivideMesh(
    const MeshData& mesh,
    double target_length,
    unsigned int iterations = 3);

/**
 * Compute a target edge length from a subdivision level.
 *
 * The level maps to a fraction of the mesh bounding-box diagonal:
 *   level 1 -> diagonal / 4
 *   level 2 -> diagonal / 8
 *   level N -> diagonal / (4 << (N-1))
 * Higher levels produce finer meshes. The result is clamped to a sane range.
 *
 * @param mesh   Input mesh used to compute the bounding box.
 * @param level  Subdivision level, must be >= 1.
 * @return       Target edge length for subdivideMesh().
 */
double subdivideMeshTargetLength(const MeshData& mesh, int level);

/**
 * Convenience subdivision by level.
 *
 * @param mesh       Input triangle mesh.
 * @param level      Subdivision level (>= 1). Larger = finer.
 * @param iterations Number of remeshing iterations (default 3).
 * @return           Refined mesh. Empty on failure.
 */
MeshData subdivideMeshByLevel(
    const MeshData& mesh,
    int level,
    unsigned int iterations = 3);

/**
 * Asynchronous mesh subdivision — runs subdivideMesh in a subprocess.
 *
 * Internally serialises the mesh to a temporary STL file, invokes the CLI
 * tool mode of this same executable, then reads back the result.
 */
class subdivideMesh_async {
   public:
    subdivideMesh_async(const MeshData& mesh,
                        double target_length,
                        unsigned int iterations = 3);
    ~subdivideMesh_async();

    bool done() const;
    void terminal();
    MeshData get_result() const;

   private:
    static std::string make_temp_path(const std::string& suffix);

    Process process_;
    std::string tmp_in_;
    std::string tmp_out_;
    mutable MeshData result_;
    mutable bool result_ready_ = false;
    double target_length_;
    unsigned int iterations_;
};

/**
 * Asynchronous level-based mesh subdivision.
 */
class subdivideMeshByLevel_async {
   public:
    subdivideMeshByLevel_async(const MeshData& mesh,
                               int level,
                               unsigned int iterations = 3);
    ~subdivideMeshByLevel_async();

    bool done() const;
    void terminal();
    MeshData get_result() const;

   private:
    static std::string make_temp_path(const std::string& suffix);

    Process process_;
    std::string tmp_in_;
    std::string tmp_out_;
    mutable MeshData result_;
    mutable bool result_ready_ = false;
    int level_;
    unsigned int iterations_;
};

}  // namespace sinriv::kigstudio::cgal
