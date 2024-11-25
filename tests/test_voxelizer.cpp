#include "kigstudio/voxel/voxelizer_svo.h"

int main() {
    sinriv::kigstudio::octree::Octree voxelData(64);
    // for(auto point:sinriv::kigstudio::voxel::draw_triangle(
    //     sinriv::kigstudio::voxel::Triangle({5,0,0},{0,5,0},{0,0,5}),
    //     1,1,1,1)){
    //         std::cout<<point<<std::endl;
    // }
    sinriv::kigstudio::voxel::draw_triangle(
        voxelData,
        sinriv::kigstudio::voxel::Triangle({ 10,0,0 }, { 0,10,0 }, { 0,0,10 }),
        1, 1, 1, 0.05);

    std::vector<sinriv::kigstudio::voxel::Triangle> mesh;

    double isolevel = 0.5;
    int numTriangles = 0;
    for (auto triangles : sinriv::kigstudio::voxel::generateMesh(voxelData, isolevel, numTriangles)) {
        mesh.push_back(triangles);
    }
    sinriv::kigstudio::voxel::saveMeshToASCIISTL(mesh, "test_voxelizer.stl");
    std::cout << "save stl success. numTriangles:" << numTriangles << std::endl;

    for (auto [vertex, normal] : sinriv::kigstudio::voxel::readSTL_ASCII("test_voxelizer.stl")) {
        std::cout << "[" << std::get<0>(vertex) << std::get<1>(vertex) << std::get<2>(vertex) << "]" << normal << std::endl;
    }
}