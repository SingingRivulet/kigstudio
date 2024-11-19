#pragma once

#include <vector>
#include <tuple>
#include <fstream>
#include "kigstudio/mesh/octree.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/utils/generator.h"
namespace sinriv::kigstudio::voxel {

    using vec3f = sinriv::kigstudio::vec3<float>;
    using Triangle = std::tuple<vec3f, vec3f, vec3f>;

    Generator<Triangle> generateMesh(sinriv::kigstudio::octree::Octree& voxelData, double isolevel, int& numTriangles);

    void saveMeshToASCIISTL(const std::vector<Triangle>& meshTriangles, const std::string& filename);
    void saveMeshToBinarySTL(const std::vector<Triangle>& meshTriangles, const std::string& filename);
}
