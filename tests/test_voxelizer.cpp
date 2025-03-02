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
        sinriv::kigstudio::voxel::vec3f(0,0,0),
        1, 1, 1, 0.05);

    std::vector<std::tuple<sinriv::kigstudio::voxel::Triangle, sinriv::kigstudio::voxel::vec3f>> mesh;

    double isolevel = 0.5;
    int numTriangles = 0;
    for (auto triangles : sinriv::kigstudio::voxel::generateMesh(voxelData, isolevel, numTriangles, true)) {
        mesh.push_back(triangles);
    }
    sinriv::kigstudio::voxel::saveMeshToASCIISTL(mesh, "test_voxelizer.stl");
    std::cout << "save stl success. numTriangles:" << numTriangles << std::endl;

    // for (auto [vertex, normal] : sinriv::kigstudio::voxel::readSTL("test_voxelizer.stl")) {
    //     std::cout << "[" << std::get<0>(vertex) << std::get<1>(vertex) << std::get<2>(vertex) << "]" << normal << std::endl;
    // }

    //创建一个测试三角形
    mesh.clear();
    sinriv::kigstudio::voxel::Triangle triangle({ 10,0,0 }, { 0,10,0 }, { 0,0,10 });
    sinriv::kigstudio::voxel::vec3f normal = sinriv::kigstudio::voxel::calcTriangleNormal(triangle);
    mesh.push_back(std::make_tuple(triangle, normal));
    sinriv::kigstudio::voxel::saveMeshToASCIISTL(mesh, "test_voxelizer_tmp.stl");

    std::cout << "test_voxelizer_tmp.stl saved." << std::endl;

    sinriv::kigstudio::octree::Octree voxelData2(512);
    sinriv::kigstudio::mat::matrix<float> mat;
    mat.setIdentity();
    sinriv::kigstudio::voxel::create_solid_mesh(voxelData2, "test_voxelizer_tmp.stl", mat, 1, 1, 1, 0.05, 100);
    mesh.clear();
    for (auto triangles : sinriv::kigstudio::voxel::generateMesh(voxelData2, isolevel, numTriangles, true)) {
        mesh.push_back(triangles);
    }
    sinriv::kigstudio::voxel::saveMeshToASCIISTL(mesh, "test_voxelizer2.stl");

    std::cout << "test_voxelizer2.stl saved." << std::endl;
    return 0;
}