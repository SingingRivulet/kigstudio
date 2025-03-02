#include "kigstudio/voxel/voxel2mesh.h"
#include "kigstudio/voxel/lut.h"
#include <set>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

namespace sinriv::kigstudio::voxel {
    static vec3f computeNormalFromVoxels(const sinriv::kigstudio::octree::Octree& voxelData, int x, int y, int z) {
        // 初始化法向量
        vec3f normal(0.0f, 0.0f, 0.0f);

        // X方向的贡献
        if (!voxelData.find({ x - 1, y, z })) normal.x -= 1.0f;  // 左侧无体素，负贡献
        if (voxelData.find({ x + 1, y, z })) normal.x += 1.0f;   // 右侧有体素，正贡献

        // Y方向的贡献
        if (!voxelData.find({ x, y - 1, z })) normal.y -= 1.0f;  // 下侧无体素，负贡献
        if (voxelData.find({ x, y + 1, z })) normal.y += 1.0f;   // 上侧有体素，正贡献

        // Z方向的贡献
        if (!voxelData.find({ x, y, z - 1 })) normal.z -= 1.0f;  // 前侧无体素，负贡献
        if (voxelData.find({ x, y, z + 1 })) normal.z += 1.0f;   // 后侧有体素，正贡献

        // 归一化法向量
        if (normal.length() > 0.0f) {
            normal = normal.normalize();
        }

        return normal;
    }

    Generator<std::tuple<Triangle, vec3f>> generateMesh(sinriv::kigstudio::octree::Octree& voxelData, double isolevel, int& numTriangles, bool computeNormals) {

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

                vec3f normal = { 0, 0, 0 };

                if (computeNormals) {
                    normal = (v2 - v1).cross(v3 - v1);
                    normal = normal.normalize();
                    auto normal_vx = computeNormalFromVoxels(voxelData, x, y, z);
                    //计算和normal的夹角
                    float angle = normal.dot(normal_vx);
                    if (angle < 0) {
                        normal = -normal;
                    }
                }

                co_yield std::tuple<Triangle, vec3f>((std::make_tuple(v1, v2, v3)), normal);

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

                vec3f normal = { 0, 0, 0 };

                if (computeNormals) {
                    normal = (v2 - v1).cross(v3 - v1);
                    normal = normal.normalize();
                    auto normal_vx = computeNormalFromVoxels(voxelData, x, y, z);
                    //计算和normal的夹角
                    float angle = normal.dot(normal_vx);
                    if (angle < 0) {
                        normal = -normal;
                    }
                }

                co_yield std::tuple<Triangle, vec3f>((std::make_tuple(v1, v2, v3)), normal);

                numTriangles++;
            }
        }
    }

    void saveMeshToASCIISTL(const std::vector<std::tuple<Triangle, vec3f>>& meshTriangles, const std::string& filename) {
        std::ofstream outFile(filename);

        if (!outFile) {
            throw std::runtime_error("Failed to open file for writing: " + filename);
        }

        outFile << "solid generated_mesh\n";

        for (const auto& [triangle, normal] : meshTriangles) {
            const vec3f& v1 = std::get<0>(triangle);
            const vec3f& v2 = std::get<1>(triangle);
            const vec3f& v3 = std::get<2>(triangle);

            outFile << "  facet normal " << normal.x << " " << normal.y << " " << normal.z << "\n";
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

    void saveMeshToBinarySTL(const std::vector<std::tuple<Triangle, vec3f>>& meshTriangles, const std::string& filename) {
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
        for (const auto& [triangle, normal_vec] : meshTriangles) {
            const vec3f& v1 = std::get<0>(triangle);
            const vec3f& v2 = std::get<1>(triangle);
            const vec3f& v3 = std::get<2>(triangle);

            float normal[3] = { normal_vec.x, normal_vec.y, normal_vec.z };

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
    bool isBinarySTL(const std::string& filePath) {
        std::ifstream file(filePath, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filePath << std::endl;
            return false; // 或者你可以抛出一个异常
        }
     
        // 读取文件的前80个字节作为头部
        std::vector<char> header(80);
        file.read(header.data(), 80);
     
        // 检查头部是否包含ASCII STL文件的标志 "solid"
        // 注意：这里我们假设"solid"后面紧跟着的字符不是ASCII可打印字符（如空格、制表符等），
        // 这在大多数情况下是合理的，但并不是一个严格的检查。
        const char* solidPrefix = "solid";
        if (std::memcmp(header.data(), solidPrefix, 5) == 0 &&
            (header[5] == ' ' || header[5] == '\t' || header[5] == '\n' || header[5] == '\r' || header[5] == '\0')) {
            // 进一步检查以确保它看起来像一个有效的ASCII STL文件头
            // 这只是一个简单的检查，实际上可能需要更复杂的逻辑来完全验证
            bool isValidAsciiHeader = true;
            for (int i = 6; i < 80; ++i) {
                char c = header[i];
                if (!(c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\0' || isprint(static_cast<unsigned char>(c)))) {
                    isValidAsciiHeader = false;
                    break;
                }
            }
            if (isValidAsciiHeader) {
                return false; // 是ASCII STL文件
            }
        }
     
        // 如果不是ASCII STL文件，我们假设它是二进制STL文件
        // 注意：这个假设可能不总是正确的，因为理论上可以存在既不是ASCII也不是标准二进制格式的STL文件
        // 但对于大多数实际应用来说，这个假设是足够的
        return true; // 假设是二进制STL文件
    }
}
