#include <vector>
#include <cassert>
#include "kigstudio/voxel/voxel2mesh.h"
int main() {
    sinriv::kigstudio::octree::Octree voxelData(256);
    for (int i = 0; i < 2;++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                voxelData.insert(sinriv::kigstudio::octree::Vec3i(i + 1, j + 1, k + 1));
            }
        }
    }


    // 测试参数
    double isolevel = 0.5;
    int numTriangles = 0;

    // 调用生成网格的函数
    std::vector<std::tuple<sinriv::kigstudio::voxel::Triangle, sinriv::kigstudio::voxel::vec3f>> mesh;

    for (auto triangles : sinriv::kigstudio::voxel::generateMesh(voxelData, isolevel, numTriangles, true)) {
        mesh.push_back(triangles);
    }

    // 验证生成的三角形数量是否正确
    std::cout << "Number of Triangles: " << numTriangles << std::endl;
    assert(numTriangles > 0 && "Mesh should contain triangles");

    // 验证输出顶点的数量是否符合预期 (每个三角形有 9 个 float 值)
    assert(mesh.size() == numTriangles && "Mesh vertices count mismatch");


    // 打印网格的部分顶点信息
    std::cout << "Sample Triangle Vertices:" << std::endl;
    for (size_t i = 0; i < mesh.size(); i += 1) {
        auto& [triangle, _] = mesh[i];

        std::cout << "Triangle:"
            << std::get<0>(triangle) << ", "
            << std::get<1>(triangle) << ", "
            << std::get<2>(triangle) << "" << std::endl;
    }

    sinriv::kigstudio::voxel::saveMeshToASCIISTL(mesh, "test_matchingcube.stl");

    std::cout << "Test Passed!" << std::endl;

    return 0;
}