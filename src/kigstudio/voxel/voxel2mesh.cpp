#include "kigstudio/voxel/voxel2mesh.h"
#include "kigstudio/voxel/lut.h"

namespace sinriv::kigstudio::voxel {

    Generator<Triangle> generateMesh(sinriv::kigstudio::octree::Octree& voxelData, double isolevel, int& numTriangles) {

        //static int numVectorMeshes;
        //numVectorMeshes++;

        std::vector<Triangle> meshTriangles;

        //meshTriangles.clear();
        //meshTriangles.shrink_to_fit();
        numTriangles = 0;

        int i;
        int cubeindex;
        vec3f vertlist[12];

        for (const auto& point : voxelData) {
            int x = point.x;
            int y = point.y;
            int z = point.z;

            //std::cout << "ITE [" << x << ", " << y << ", " << z << "]" << std::endl;

            //cube[0] = voxelData[x + 0][y + 0][z + 0];
            //cube[1] = voxelData[x + 1][y + 0][z + 0];
            //cube[2] = voxelData[x + 1][y + 0][z + 1];
            //cube[3] = voxelData[x + 0][y + 0][z + 1];
            //cube[4] = voxelData[x + 0][y + 1][z + 0];
            //cube[5] = voxelData[x + 1][y + 1][z + 0];
            //cube[6] = voxelData[x + 1][y + 1][z + 1];
            //cube[7] = voxelData[x + 0][y + 1][z + 1];

            // cubeindex = 0;

            // if (voxelData.find(sinriv::kigstudio::octree::Vec3i(x, y, z)))  cubeindex |= 1;
            cubeindex = 1; //被检索的初始点一定是存在的

            if (voxelData.find(sinriv::kigstudio::octree::Vec3i(x + 1, y, z)))  cubeindex |= 2;
            if (voxelData.find(sinriv::kigstudio::octree::Vec3i(x + 1, y, z + 1)))  cubeindex |= 4;
            if (voxelData.find(sinriv::kigstudio::octree::Vec3i(x, y, z + 1)))  cubeindex |= 8;
            if (voxelData.find(sinriv::kigstudio::octree::Vec3i(x, y + 1, z)))  cubeindex |= 16;
            if (voxelData.find(sinriv::kigstudio::octree::Vec3i(x + 1, y + 1, z)))  cubeindex |= 32;
            if (voxelData.find(sinriv::kigstudio::octree::Vec3i(x + 1, y + 1, z + 1)))  cubeindex |= 64;
            if (voxelData.find(sinriv::kigstudio::octree::Vec3i(x, y + 1, z + 1)))  cubeindex |= 128;


            if (edgeTable[cubeindex] == 0)	// cube is entirely inside or outside of the geometry
                continue;

            // For each edge with an intersection, create the coords for the vertex
            if (edgeTable[cubeindex] & 1)
                vertlist[0] = vec3f(x + .5, y, z);
            if (edgeTable[cubeindex] & 2)
                vertlist[1] = vec3f(x + 1, y, z + .5);
            if (edgeTable[cubeindex] & 4)
                vertlist[2] = vec3f(x + .5, y, z + 1);
            if (edgeTable[cubeindex] & 8)
                vertlist[3] = vec3f(x, y, z + .5);
            if (edgeTable[cubeindex] & 16)
                vertlist[4] = vec3f(x + .5, y + 1, z);
            if (edgeTable[cubeindex] & 32)
                vertlist[5] = vec3f(x + 1, y + 1, z + .5);
            if (edgeTable[cubeindex] & 64)
                vertlist[6] = vec3f(x + .5, y + 1, z + 1);
            if (edgeTable[cubeindex] & 128)
                vertlist[7] = vec3f(x, y + 1, z + .5);
            if (edgeTable[cubeindex] & 256)
                vertlist[8] = vec3f(x, y + .5, z);
            if (edgeTable[cubeindex] & 512)
                vertlist[9] = vec3f(x + 1, y + .5, z);
            if (edgeTable[cubeindex] & 1024)
                vertlist[10] = vec3f(x + 1, y + .5, z + 1);
            if (edgeTable[cubeindex] & 2048)
                vertlist[11] = vec3f(x, y + .5, z + 1);

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

}
