#pragma once

#include <vector>
#include <tuple>
#include <fstream>
#include "kigstudio/voxel/octree.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/utils/generator.h"
namespace sinriv::kigstudio::voxel {

    using vec3f = sinriv::kigstudio::vec3<float>;
    using Triangle = std::tuple<vec3f, vec3f, vec3f>;

    Generator<std::tuple<Triangle,vec3f>> generateMesh(sinriv::kigstudio::octree::Octree& voxelData, double isolevel, int& numTriangles, bool computeNormals = false);

    void saveMeshToASCIISTL(const std::vector<std::tuple<Triangle,vec3f>>& meshTriangles, const std::string& filename);
    void saveMeshToBinarySTL(const std::vector<std::tuple<Triangle,vec3f>>& meshTriangles, const std::string& filename);
    Generator<std::tuple<Triangle, vec3f>> readSTL_ASCII(std::string filename);
    Generator<std::tuple<Triangle, vec3f>> readSTL_Binary(std::string filename);
    bool isBinarySTL(const std::string& filePath);
    inline Generator<std::tuple<Triangle, vec3f>> readSTL(std::string filename) {
        if (isBinarySTL(filename)) {
            return readSTL_Binary(filename);
        } else {
            return readSTL_ASCII(filename);
        }
    }

    inline vec3f calcTriangleNormal(const Triangle& triangle) {
        vec3f a, b, c;
        std::tie(a, b, c) = triangle;
        return (b - a).cross(c - a).normalize();
    }
}
