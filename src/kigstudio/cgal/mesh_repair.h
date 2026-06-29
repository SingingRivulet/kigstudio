#pragma once
#include <vector>
#include <tuple>
#include <string>
#include "kigstudio/voxel/voxel2mesh.h"
#include "kigstudio/utils/process.h"

namespace sinriv::kigstudio::cgal {

using vec3f = sinriv::kigstudio::vec3<float>;
using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;
using MeshData = std::vector<std::tuple<Triangle, vec3f>>;

// ===========================================================================
// Synchronous API
// ===========================================================================

MeshData alpha_wrap(const MeshData& mesh, double alpha = 10.0, double offset = 0.1);
MeshData fill_holes(const MeshData& mesh);
MeshData stitch_borders(const MeshData& mesh, double max_dist = 0.001);
MeshData merge_duplicate_vertices(const MeshData& mesh, double tol = 1e-6);
MeshData mesh_union(const MeshData& mesh_a, const MeshData& mesh_b);
MeshData orient_volume(const MeshData& mesh);

// ===========================================================================
// Asynchronous API — runs each operation in a subprocess via the CLI tool.
//
// Usage pattern (identical for all six):
//   alpha_wrap_async async(mesh, 15.0, 0.05);
//   while (!async.done()) { /* poll or do other work */ }
//   auto result = async.get_result();   // throws if still running
// ===========================================================================

class alpha_wrap_async {
public:
    alpha_wrap_async(const MeshData& mesh, double alpha = 10.0, double offset = 0.1);
    ~alpha_wrap_async();
    bool done() const;
    void terminal();
    MeshData get_result() const;
private:

Process process_;
    std::string tmp_in_, tmp_out_;
    mutable MeshData result_;
    mutable bool result_ready_ = false;
    double alpha_, offset_;
};

class fill_holes_async {
public:
    explicit fill_holes_async(const MeshData& mesh);
    ~fill_holes_async();
    bool done() const;
    void terminal();
    MeshData get_result() const;
private:

Process process_;
    std::string tmp_in_, tmp_out_;
    mutable MeshData result_;
    mutable bool result_ready_ = false;
};

class stitch_borders_async {
public:
    stitch_borders_async(const MeshData& mesh, double max_dist = 0.001);
    ~stitch_borders_async();
    bool done() const;
    void terminal();
    MeshData get_result() const;
private:

Process process_;
    std::string tmp_in_, tmp_out_;
    mutable MeshData result_;
    mutable bool result_ready_ = false;
    double max_dist_;
};

class merge_vertices_async {
public:
    merge_vertices_async(const MeshData& mesh, double tol = 1e-6);
    ~merge_vertices_async();
    bool done() const;
    void terminal();
    MeshData get_result() const;
private:

Process process_;
    std::string tmp_in_, tmp_out_;
    mutable MeshData result_;
    mutable bool result_ready_ = false;
    double tol_;
};

class mesh_union_async {
public:
    mesh_union_async(const MeshData& mesh_a, const MeshData& mesh_b);
    ~mesh_union_async();
    bool done() const;
    void terminal();
    MeshData get_result() const;
private:

Process process_;
    std::string tmp_a_, tmp_b_, tmp_out_;
    mutable MeshData result_;
    mutable bool result_ready_ = false;
};

class orient_volume_async {
public:
    explicit orient_volume_async(const MeshData& mesh);
    ~orient_volume_async();
    bool done() const;
    void terminal();
    MeshData get_result() const;
private:

Process process_;
    std::string tmp_in_, tmp_out_;
    mutable MeshData result_;
    mutable bool result_ready_ = false;
};

} // namespace sinriv::kigstudio::cgal
