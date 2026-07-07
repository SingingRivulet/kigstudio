#pragma once

#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

#include "kigstudio/voxel/voxel2mesh.h"

namespace sinriv::kigstudio::io {

using vec3f = sinriv::kigstudio::vec3<float>;
using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;

/**
 * A single PMX vertex with position, normal and bone weights.
 */
struct PMXVertex {
    vec3f position;
    vec3f normal;
    // (bone_index, weight). Weights are normalized to sum to 1.
    std::vector<std::pair<int, float>> bone_weights;
};

/**
 * A PMX material. face_start and face_count are in triangle indices
 * (so face_count * 3 entries in the global face index array).
 */
struct PMXMaterial {
    std::string name_local;
    std::string name_universal;
    uint32_t face_start = 0;  // index into PMXModel::faces
    uint32_t face_count = 0;  // number of triangles
};

/**
 * A PMX bone.
 */
struct PMXBone {
    std::string name_local;
    std::string name_universal;
    vec3f position;
};

/**
 * In-memory representation of the parts of a PMX file we care about.
 */
struct PMXModel {
    std::vector<PMXVertex> vertices;
    std::vector<uint32_t> faces;  // triangle indices, size = num_triangles * 3
    std::vector<PMXMaterial> materials;
    std::vector<PMXBone> bones;

    uint32_t triangle_count() const {
        return static_cast<uint32_t>(faces.size() / 3);
    }
};

/**
 * Load a PMX 2.0/2.1 file and extract vertices, faces, materials and bones.
 *
 * @param path Path to the .pmx file.
 * @return Loaded model. Empty on failure.
 */
PMXModel load_pmx(const std::string& path);

/**
 * Convert PMX triangle indices + vertex data into the project's MeshData
 * representation (tuple of triangle + normal).
 */
std::vector<std::tuple<Triangle, vec3f>> pmx_model_to_mesh_data(
    const PMXModel& model);

/**
 * Extract triangles whose at least one vertex is influenced by any bone whose
 * name contains one of the keywords.
 *
 * @param model      Loaded PMX model.
 * @param keywords   List of bone name keywords.
 * @param threshold  Minimum bone weight for a vertex to be considered affected.
 * @param case_sensitive Whether keyword matching is case sensitive.
 * @return MeshData of extracted triangles.
 */
std::vector<std::tuple<Triangle, vec3f>> extract_by_bone_names(
    const PMXModel& model,
    const std::vector<std::string>& keywords,
    float threshold = 0.5f,
    bool case_sensitive = false);

/**
 * Extract triangles that belong to materials whose names contain one of the
 * keywords.
 *
 * @param model      Loaded PMX model.
 * @param keywords   List of material name keywords.
 * @param case_sensitive Whether keyword matching is case sensitive.
 * @return MeshData of extracted triangles.
 */
std::vector<std::tuple<Triangle, vec3f>> extract_by_material_names(
    const PMXModel& model,
    const std::vector<std::string>& keywords,
    bool case_sensitive = false);

}  // namespace sinriv::kigstudio::io
