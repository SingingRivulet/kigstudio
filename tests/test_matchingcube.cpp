#include <vector>
#include <cassert>
#include "kigstudio/voxel/voxel2mesh.h"
int main() {
    sinriv::kigstudio::octree::Octree voxelData(256);
    voxelData.insert(sinriv::kigstudio::octree::Vec3i(0, 0, 0));
    voxelData.insert(sinriv::kigstudio::octree::Vec3i(1, 0, 0));
    voxelData.insert(sinriv::kigstudio::octree::Vec3i(1, 1, 0));
    voxelData.insert(sinriv::kigstudio::octree::Vec3i(0, 1, 0));
    voxelData.insert(sinriv::kigstudio::octree::Vec3i(0, 0, 1));
    voxelData.insert(sinriv::kigstudio::octree::Vec3i(1, 0, 1));
    voxelData.insert(sinriv::kigstudio::octree::Vec3i(1, 1, 1));
    voxelData.insert(sinriv::kigstudio::octree::Vec3i(0, 1, 1));

    // 测试参数
    double isolevel = 0.5;
    int numTriangles = 0;

    // 调用生成网格的函数
    std::vector<sinriv::kigstudio::voxel::Triangle> mesh;
    
    for (auto triangles : sinriv::kigstudio::voxel::generateMesh(voxelData, isolevel, numTriangles)) {
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
        std::cout << "Triangle:" << std::get<0>(mesh[i]) << ", " << std::get<1>(mesh[i]) << ", " << std::get<2>(mesh[i]) << "" << std::endl;
    }

    sinriv::kigstudio::voxel::saveMeshToASCIISTL(mesh, "test.stl");

    std::cout << "Test Passed!" << std::endl;

    return 0;
}