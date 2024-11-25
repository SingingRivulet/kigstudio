#include "kigstudio/voxel/voxel2mesh.h"
#include "kigstudio/voxel/lut.h"
#include <set>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

namespace sinriv::kigstudio::voxel {

    Generator<Triangle> generateMesh(sinriv::kigstudio::octree::Octree& voxelData, double isolevel, int& numTriangles) {

        //static int numVectorMeshes;
        //numVectorMeshes++;
        using vec3i = sinriv::kigstudio::octree::Vec3i;
        std::set<vec3i> back_face;

        numTriangles = 0;

        int i;
        int cubeindex;
        vec3f vertlist[12];

        for (const auto& point : voxelData) {
            int x = point.x;
            int y = point.y;
            int z = point.z;

            //std::cout << "ITE [" << x << ", " << y << ", " << z << "]" << std::endl;

            // cubeindex = 0;

            // if (voxelData.find(vec3i(x, y, z)))  cubeindex |= 1;
            cubeindex = 1; //被检索的初始点一定是存在的

            // 处理正方向
            if (voxelData.find(vec3i(x + 1, y, z)))  cubeindex |= 2;
            if (voxelData.find(vec3i(x + 1, y, z + 1)))  cubeindex |= 4;
            if (voxelData.find(vec3i(x, y, z + 1)))  cubeindex |= 8;
            if (voxelData.find(vec3i(x, y + 1, z)))  cubeindex |= 16;
            if (voxelData.find(vec3i(x + 1, y + 1, z)))  cubeindex |= 32;
            if (voxelData.find(vec3i(x + 1, y + 1, z + 1)))  cubeindex |= 64;
            if (voxelData.find(vec3i(x, y + 1, z + 1)))  cubeindex |= 128;

            //处理负方向
            if (!voxelData.find(vec3i(x - 1 + 1, y - 1, z - 1))) back_face.insert(vec3i(x - 1 + 1, y - 1, z - 1));
            if (!voxelData.find(vec3i(x - 1 + 1, y - 1, z - 1 + 1)))back_face.insert(vec3i(x - 1 + 1, y - 1, z - 1 + 1));
            if (!voxelData.find(vec3i(x - 1 + 1, y - 1, z - 1 + 1))) back_face.insert(vec3i(x - 1 + 1, y - 1, z - 1 + 1));
            if (!voxelData.find(vec3i(x - 1, y - 1, z - 1 + 1))) back_face.insert(vec3i(x - 1, y - 1, z - 1 + 1));
            if (!voxelData.find(vec3i(x - 1, y - 1 + 1, z - 1))) back_face.insert(vec3i(x - 1, y - 1 + 1, z - 1));
            if (!voxelData.find(vec3i(x - 1 + 1, y - 1 + 1, z - 1))) back_face.insert(vec3i(x - 1 + 1, y - 1 + 1, z - 1));
            if (!voxelData.find(vec3i(x - 1 + 1, y - 1 + 1, z - 1 + 1))) back_face.insert(vec3i(x - 1 + 1, y - 1 + 1, z - 1 + 1));
            if (!voxelData.find(vec3i(x - 1, y - 1 + 1, z - 1 + 1))) back_face.insert(vec3i(x - 1, y - 1 + 1, z - 1 + 1));
            if (!voxelData.find(vec3i(x - 1, y - 1, z - 1))) back_face.insert(vec3i(x - 1, y - 1, z - 1));


            if (edgeTable[cubeindex] == 0)	// cube is entirely inside or outside of the geometry
                continue;

            // For each edge with an intersection, create the coords for the vertex
            if (edgeTable[cubeindex] & 1)
                vertlist[0] = vec3f((float)(x + .5), (float)(y), (float)(z));
            if (edgeTable[cubeindex] & 2)
                vertlist[1] = vec3f((float)(x + 1), (float)(y), (float)(z + .5));
            if (edgeTable[cubeindex] & 4)
                vertlist[2] = vec3f((float)(x + .5), (float)(y), (float)(z + 1));
            if (edgeTable[cubeindex] & 8)
                vertlist[3] = vec3f((float)(x), (float)(y), (float)(z + .5));
            if (edgeTable[cubeindex] & 16)
                vertlist[4] = vec3f((float)(x + .5), (float)(y + 1), (float)(z));
            if (edgeTable[cubeindex] & 32)
                vertlist[5] = vec3f((float)(x + 1), (float)(y + 1), (float)(z + .5));
            if (edgeTable[cubeindex] & 64)
                vertlist[6] = vec3f((float)(x + .5), (float)(y + 1), (float)(z + 1));
            if (edgeTable[cubeindex] & 128)
                vertlist[7] = vec3f((float)(x), (float)(y + 1), (float)(z + .5));
            if (edgeTable[cubeindex] & 256)
                vertlist[8] = vec3f((float)(x), (float)(y + .5), (float)(z));
            if (edgeTable[cubeindex] & 512)
                vertlist[9] = vec3f((float)(x + 1), (float)(y + .5), (float)(z));
            if (edgeTable[cubeindex] & 1024)
                vertlist[10] = vec3f((float)(x + 1), (float)(y + .5), (float)(z + 1));
            if (edgeTable[cubeindex] & 2048)
                vertlist[11] = vec3f((float)(x), (float)(y + .5), (float)(z + 1));

            // Create the triangle
            for (i = 0; triTable[cubeindex][i] != -1; i += 3) {

                vec3f v1 = vertlist[triTable[cubeindex][i]];
                vec3f v2 = vertlist[triTable[cubeindex][i + 1]];
                vec3f v3 = vertlist[triTable[cubeindex][i + 2]];

                co_yield (std::make_tuple(v1, v2, v3));

                numTriangles++;
            }
        }
        for (const auto& point : back_face) {
            int x = point.x;
            int y = point.y;
            int z = point.z;

            cubeindex = 0;
            if (voxelData.find(vec3i(x + 1, y, z)))  cubeindex |= 2;
            if (voxelData.find(vec3i(x + 1, y, z + 1)))  cubeindex |= 4;
            if (voxelData.find(vec3i(x, y, z + 1)))  cubeindex |= 8;
            if (voxelData.find(vec3i(x, y + 1, z)))  cubeindex |= 16;
            if (voxelData.find(vec3i(x + 1, y + 1, z)))  cubeindex |= 32;
            if (voxelData.find(vec3i(x + 1, y + 1, z + 1)))  cubeindex |= 64;
            if (voxelData.find(vec3i(x, y + 1, z + 1)))  cubeindex |= 128;

            if (edgeTable[cubeindex] == 0)	// cube is entirely inside or outside of the geometry
                continue;

            // For each edge with an intersection, create the coords for the vertex
            if (edgeTable[cubeindex] & 1)
                vertlist[0] = vec3f((float)(x + .5), (float)(y), (float)(z));
            if (edgeTable[cubeindex] & 2)
                vertlist[1] = vec3f((float)(x + 1), (float)(y), (float)(z + .5));
            if (edgeTable[cubeindex] & 4)
                vertlist[2] = vec3f((float)(x + .5), (float)(y), (float)(z + 1));
            if (edgeTable[cubeindex] & 8)
                vertlist[3] = vec3f((float)(x), (float)(y), (float)(z + .5));
            if (edgeTable[cubeindex] & 16)
                vertlist[4] = vec3f((float)(x + .5), (float)(y + 1), (float)(z));
            if (edgeTable[cubeindex] & 32)
                vertlist[5] = vec3f((float)(x + 1), (float)(y + 1), (float)(z + .5));
            if (edgeTable[cubeindex] & 64)
                vertlist[6] = vec3f((float)(x + .5), (float)(y + 1), (float)(z + 1));
            if (edgeTable[cubeindex] & 128)
                vertlist[7] = vec3f((float)(x), (float)(y + 1), (float)(z + .5));
            if (edgeTable[cubeindex] & 256)
                vertlist[8] = vec3f((float)(x), (float)(y + .5), (float)(z));
            if (edgeTable[cubeindex] & 512)
                vertlist[9] = vec3f((float)(x + 1), (float)(y + .5), (float)(z));
            if (edgeTable[cubeindex] & 1024)
                vertlist[10] = vec3f((float)(x + 1), (float)(y + .5), (float)(z + 1));
            if (edgeTable[cubeindex] & 2048)
                vertlist[11] = vec3f((float)(x), (float)(y + .5), (float)(z + 1));

            // Create the triangle
            for (i = 0; triTable[cubeindex][i] != -1; i += 3) {

                vec3f v1 = vertlist[triTable[cubeindex][i]];
                vec3f v2 = vertlist[triTable[cubeindex][i + 1]];
                vec3f v3 = vertlist[triTable[cubeindex][i + 2]];

                co_yield (std::make_tuple(v1, v2, v3));

                numTriangles++;
            }
        }
    }

    void saveMeshToASCIISTL(const std::vector<Triangle>& meshTriangles, const std::string& filename) {
        std::ofstream outFile(filename);

        if (!outFile) {
            throw std::runtime_error("Failed to open file for writing: " + filename);
        }

        outFile << "solid generated_mesh\n";

        for (const auto& triangle : meshTriangles) {
            const vec3f& v1 = std::get<0>(triangle);
            const vec3f& v2 = std::get<1>(triangle);
            const vec3f& v3 = std::get<2>(triangle);

            // 简单设置法向量为零，可以根据需要计算真实法向量
            float nx = 0, ny = 0, nz = 0;

            outFile << "  facet normal " << nx << " " << ny << " " << nz << "\n";
            outFile << "    outer loop\n";
            outFile << "      vertex " << v1.x << " " << v1.y << " " << v1.z << "\n";
            outFile << "      vertex " << v2.x << " " << v2.y << " " << v2.z << "\n";
            outFile << "      vertex " << v3.x << " " << v3.y << " " << v3.z << "\n";
            outFile << "    endloop\n";
            outFile << "  endfacet\n";
        }

        outFile << "endsolid generated_mesh\n";
        outFile.close();
    }

    void saveMeshToBinarySTL(const std::vector<Triangle>& meshTriangles, const std::string& filename) {
        std::ofstream outFile(filename, std::ios::binary);

        if (!outFile) {
            throw std::runtime_error("Failed to open file for writing: " + filename);
        }

        // 写入 80 字节的文件头
        char header[80] = "Generated by Marching Cubes";
        outFile.write(header, 80);

        // 写入三角形数量
        uint32_t numTriangles = static_cast<uint32_t>(meshTriangles.size());
        outFile.write(reinterpret_cast<const char*>(&numTriangles), sizeof(uint32_t));

        // 写入每个三角形的数据
        for (const auto& triangle : meshTriangles) {
            const vec3f& v1 = std::get<0>(triangle);
            const vec3f& v2 = std::get<1>(triangle);
            const vec3f& v3 = std::get<2>(triangle);

            // 简单设置法向量为零，可以根据需要计算真实法向量
            float normal[3] = { 0, 0, 0 };
            outFile.write(reinterpret_cast<const char*>(normal), 3 * sizeof(float));

            // 写入三角形顶点
            float vertices[9] = { v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, v3.x, v3.y, v3.z };
            outFile.write(reinterpret_cast<const char*>(vertices), 9 * sizeof(float));

            // 写入 2 字节的属性字节计数（通常为 0）
            uint16_t attributeByteCount = 0;
            outFile.write(reinterpret_cast<const char*>(&attributeByteCount), sizeof(uint16_t));
        }

        outFile.close();
    }

    Generator<std::tuple<Triangle, vec3f>> readSTL_ASCII(std::string filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cout << "Failed to open file:" << filename << std::endl;
            throw std::runtime_error("Failed to open file.");
        }

        std::string line;
        vec3f normal;

        std::vector<vec3f> vertices{};

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string token;
            iss >> token;
            if (token == "facet") {
                iss >> token; // skip "normal"
                iss >> normal.x >> normal.y >> normal.z;
            }
            else if (token == "vertex") {
                vec3f vertex;
                iss >> vertex.x >> vertex.y >> vertex.z;
                vertices.push_back(vertex);
                if (vertices.size() == 3) {
                    co_yield std::tuple<Triangle, vec3f>({ vertices[0], vertices[1], vertices[2] }, normal);
                    vertices.clear();
                }
            }
        }
    }
    Generator<std::tuple<Triangle, vec3f>> readSTL_Binary(std::string filename) {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            std::cout << "Failed to open file:" << filename << std::endl;
            throw std::runtime_error("Failed to open file.");
        }

        // 跳过文件头
        char header[80];
        file.read(header, 80);

        // 读取三角形数量
        uint32_t triangleCount;
        file.read(reinterpret_cast<char*>(&triangleCount), sizeof(triangleCount));

        // 读取每个三角形
        for (uint32_t i = 0; i < triangleCount; ++i) {
            vec3f normal;
            Triangle triangle;
            file.read(reinterpret_cast<char*>(&normal), sizeof(vec3f));
            file.read(reinterpret_cast<char*>(&std::get<0>(triangle)), sizeof(vec3f));
            file.read(reinterpret_cast<char*>(&std::get<1>(triangle)), sizeof(vec3f));
            file.read(reinterpret_cast<char*>(&std::get<2>(triangle)), sizeof(vec3f));

            // 跳过2字节的属性字节计数
            file.ignore(2);

            co_yield std::tuple<Triangle, vec3f>(
                { std::get<0>(triangle), std::get<1>(triangle), std::get<2>(triangle) },
                normal);
        }

    }
}
