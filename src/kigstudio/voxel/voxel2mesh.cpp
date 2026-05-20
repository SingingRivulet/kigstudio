#include "kigstudio/voxel/voxel2mesh.h"
#include "kigstudio/voxel/lut.h"
#include "kigstudio/voxel/voxel_EDT.h"
#include <algorithm>
#include <set>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#ifdef _WIN32
#include <windows.h>
inline std::wstring utf8_to_wstring_file(const std::string& utf8) {
    if (utf8.empty()) return {};
    int len = MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, nullptr, 0);
    if (len <= 1) return {};
    std::wstring w(len - 1, 0);
    MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, &w[0], len);
    return w;
}
#endif

namespace sinriv::kigstudio::voxel {
    template <typename T>
    static vec3f computeNormalFromVoxels(const T& voxelData, int x, int y, int z) {
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

    inline bool getVoxel(
        const VoxelGrid& grid,
        const Chunk& chunk,
        int cx,int cy,int cz,
        int lx,int ly,int lz)
    {
        if (lx>=0 && lx<32 && ly>=0 && ly<32 && lz>=0 && lz<32)
            return chunk.get(lx,ly,lz);

        return grid.contains(
            (cx<<5)+lx,
            (cy<<5)+ly,
            (cz<<5)+lz
        );
    }

    Generator<std::tuple<Triangle, vec3f>>
    generateMesh(
        sinriv::kigstudio::voxel::VoxelGrid& voxelData,
        double isolevel,
        int& numTriangles,
        bool computeNormals,
        float expand)
    {
        using Chunk = sinriv::kigstudio::voxel::Chunk;
        (void)isolevel;

        numTriangles = 0;
        vec3f vertlist[12];

        // ===== 快速访问函数 =====
        auto getVoxel = [&](int wx, int wy, int wz) -> bool {
            return voxelData.contains(wx, wy, wz);
        };

        // 记录已处理的 cell，避免相邻 chunk 在边界处重复生成三角形
        std::set<std::tuple<int, int, int>> processed_cells;

        // ===== 遍历所有 chunk =====
        for (const auto& [key, chunk] : voxelData.chunks) {

            int cx, cy, cz;
            unpackChunkKey(key, cx, cy, cz);

            int baseX = cx << 5;
            int baseY = cy << 5;
            int baseZ = cz << 5;

            // ❗关键：遍历 cell（范围扩大一圈），用 processed 集合避免跨 chunk 重复
            for (int z = -1; z < 32; z++)
            for (int y = -1; y < 32; y++)
            for (int x = -1; x < 32; x++) {

                int wx = baseX + x;
                int wy = baseY + y;
                int wz = baseZ + z;

                // 如果该 cell 已被其他 chunk 处理过，则跳过
                if (!processed_cells.insert({wx, wy, wz}).second)
                    continue;

                // ===== 8 corner =====
                bool v000 = getVoxel(wx,   wy,   wz);
                bool v100 = getVoxel(wx+1, wy,   wz);
                bool v110 = getVoxel(wx+1, wy+1, wz);
                bool v010 = getVoxel(wx,   wy+1, wz);

                bool v001 = getVoxel(wx,   wy,   wz+1);
                bool v101 = getVoxel(wx+1, wy,   wz+1);
                bool v111 = getVoxel(wx+1, wy+1, wz+1);
                bool v011 = getVoxel(wx,   wy+1, wz+1);

                int cubeindex = 0;
                cubeindex |= v000 << 0;
                cubeindex |= v100 << 1;
                cubeindex |= v110 << 2;
                cubeindex |= v010 << 3;
                cubeindex |= v001 << 4;
                cubeindex |= v101 << 5;
                cubeindex |= v111 << 6;
                cubeindex |= v011 << 7;

                // 跳过完全空或完全满
                if (cubeindex == 0 || cubeindex == 255)
                    continue;

                if (edgeTable[cubeindex] == 0)
                    continue;

                float fx = float(wx);
                float fy = float(wy);
                float fz = float(wz);

                vec3f cell_center{fx + 0.5f, fy + 0.5f, fz + 0.5f};

                // ===== 插值（简化：中点）=====
                if (edgeTable[cubeindex] & 1)
                    vertlist[0] = {fx+.5f, fy, fz};
                if (edgeTable[cubeindex] & 2)
                    vertlist[1] = {fx+1, fy+.5f, fz};
                if (edgeTable[cubeindex] & 4)
                    vertlist[2] = {fx+.5f, fy+1, fz};
                if (edgeTable[cubeindex] & 8)
                    vertlist[3] = {fx, fy+.5f, fz};

                if (edgeTable[cubeindex] & 16)
                    vertlist[4] = {fx+.5f, fy, fz+1};
                if (edgeTable[cubeindex] & 32)
                    vertlist[5] = {fx+1, fy+.5f, fz+1};
                if (edgeTable[cubeindex] & 64)
                    vertlist[6] = {fx+.5f, fy+1, fz+1};
                if (edgeTable[cubeindex] & 128)
                    vertlist[7] = {fx, fy+.5f, fz+1};

                if (edgeTable[cubeindex] & 256)
                    vertlist[8] = {fx, fy, fz+.5f};
                if (edgeTable[cubeindex] & 512)
                    vertlist[9] = {fx+1, fy, fz+.5f};
                if (edgeTable[cubeindex] & 1024)
                    vertlist[10] = {fx+1, fy+1, fz+.5f};
                if (edgeTable[cubeindex] & 2048)
                    vertlist[11] = {fx, fy+1, fz+.5f};

                // ===== triangles =====
                for (int i = 0; triTable[cubeindex][i] != -1; i += 3) {

                    vec3f v1 = vertlist[triTable[cubeindex][i]];
                    vec3f v2 = vertlist[triTable[cubeindex][i+1]];
                    vec3f v3 = vertlist[triTable[cubeindex][i+2]];

                    vec3f normal{0,0,0};

                    if (computeNormals || expand > 0.0f) {
                        normal = (v2 - v1).cross(v3 - v1).normalize();
                        normal = -normal;
                    }

                    if (expand > 0.0f) {
                        v1 += normal * expand;
                        v2 += normal * expand;
                        v3 += normal * expand;
                    }

                    co_yield {
                        Triangle(
                            v1 * voxelData.voxel_size + voxelData.global_position,
                            v3 * voxelData.voxel_size + voxelData.global_position,
                            v2 * voxelData.voxel_size + voxelData.global_position
                        ),
                        normal
                    };

                    numTriangles++;
                }
            }
        }
    }

    Generator<std::tuple<Triangle, vec3f>>
    generateSmoothMeshFromSDF(
        sinriv::kigstudio::voxel::VoxelGrid& voxelData,
        int& numTriangles,
        bool computeNormals,
        int subdivisions)
    {
        numTriangles = 0;
        subdivisions = std::max(1, subdivisions);

        DenseGrid dense = buildDenseGrid(voxelData, 2);
        if (dense.sx <= 1 || dense.sy <= 1 || dense.sz <= 1) {
            co_return;
        }

        SDFGrid sdf = buildSDF(dense);
        vec3f vertlist[12];

        auto sample = [&](float x, float y, float z) -> float {
            x = std::max(0.0f, std::min(x, static_cast<float>(sdf.sx - 1)));
            y = std::max(0.0f, std::min(y, static_cast<float>(sdf.sy - 1)));
            z = std::max(0.0f, std::min(z, static_cast<float>(sdf.sz - 1)));

            const int x0 = static_cast<int>(std::floor(x));
            const int y0 = static_cast<int>(std::floor(y));
            const int z0 = static_cast<int>(std::floor(z));
            const int x1 = std::min(x0 + 1, sdf.sx - 1);
            const int y1 = std::min(y0 + 1, sdf.sy - 1);
            const int z1 = std::min(z0 + 1, sdf.sz - 1);

            const float tx = x - static_cast<float>(x0);
            const float ty = y - static_cast<float>(y0);
            const float tz = z - static_cast<float>(z0);

            auto lerp = [](float a, float b, float t) {
                return a + (b - a) * t;
            };

            const float c00 = lerp(sdf.get(x0, y0, z0), sdf.get(x1, y0, z0), tx);
            const float c10 = lerp(sdf.get(x0, y1, z0), sdf.get(x1, y1, z0), tx);
            const float c01 = lerp(sdf.get(x0, y0, z1), sdf.get(x1, y0, z1), tx);
            const float c11 = lerp(sdf.get(x0, y1, z1), sdf.get(x1, y1, z1), tx);
            const float c0 = lerp(c00, c10, ty);
            const float c1 = lerp(c01, c11, ty);
            return lerp(c0, c1, tz);
        };

        auto samplePosition = [&](float x, float y, float z) -> vec3f {
            return {static_cast<float>(sdf.min_bound.x) + x + 0.5f,
                    static_cast<float>(sdf.min_bound.y) + y + 0.5f,
                    static_cast<float>(sdf.min_bound.z) + z + 0.5f};
        };

        auto interpolate = [](const vec3f& p1,
                              const vec3f& p2,
                              float v1,
                              float v2) -> vec3f {
            const float denom = v1 - v2;
            if (std::abs(denom) < 1e-6f) {
                return (p1 + p2) * 0.5f;
            }

            float t = v1 / denom;
            t = std::max(0.0f, std::min(1.0f, t));
            return p1 + (p2 - p1) * t;
        };

        const int hsx = (sdf.sx - 1) * subdivisions;
        const int hsy = (sdf.sy - 1) * subdivisions;
        const int hsz = (sdf.sz - 1) * subdivisions;
        const float inv_subdiv = 1.0f / static_cast<float>(subdivisions);

        for (int z = 0; z < hsz; ++z)
        for (int y = 0; y < hsy; ++y)
        for (int x = 0; x < hsx; ++x) {
            const float x0 = static_cast<float>(x) * inv_subdiv;
            const float y0 = static_cast<float>(y) * inv_subdiv;
            const float z0 = static_cast<float>(z) * inv_subdiv;
            const float x1 = static_cast<float>(x + 1) * inv_subdiv;
            const float y1 = static_cast<float>(y + 1) * inv_subdiv;
            const float z1 = static_cast<float>(z + 1) * inv_subdiv;

            float val[8] = {
                sample(x0, y0, z0),
                sample(x1, y0, z0),
                sample(x1, y1, z0),
                sample(x0, y1, z0),
                sample(x0, y0, z1),
                sample(x1, y0, z1),
                sample(x1, y1, z1),
                sample(x0, y1, z1),
            };

            int cubeindex = 0;
            cubeindex |= (val[0] < 0.0f) << 0;
            cubeindex |= (val[1] < 0.0f) << 1;
            cubeindex |= (val[2] < 0.0f) << 2;
            cubeindex |= (val[3] < 0.0f) << 3;
            cubeindex |= (val[4] < 0.0f) << 4;
            cubeindex |= (val[5] < 0.0f) << 5;
            cubeindex |= (val[6] < 0.0f) << 6;
            cubeindex |= (val[7] < 0.0f) << 7;

            if (cubeindex == 0 || cubeindex == 255 || edgeTable[cubeindex] == 0)
                continue;

            vec3f p[8] = {
                samplePosition(x0, y0, z0),
                samplePosition(x1, y0, z0),
                samplePosition(x1, y1, z0),
                samplePosition(x0, y1, z0),
                samplePosition(x0, y0, z1),
                samplePosition(x1, y0, z1),
                samplePosition(x1, y1, z1),
                samplePosition(x0, y1, z1),
            };

            if (edgeTable[cubeindex] & 1)
                vertlist[0] = interpolate(p[0], p[1], val[0], val[1]);
            if (edgeTable[cubeindex] & 2)
                vertlist[1] = interpolate(p[1], p[2], val[1], val[2]);
            if (edgeTable[cubeindex] & 4)
                vertlist[2] = interpolate(p[2], p[3], val[2], val[3]);
            if (edgeTable[cubeindex] & 8)
                vertlist[3] = interpolate(p[3], p[0], val[3], val[0]);

            if (edgeTable[cubeindex] & 16)
                vertlist[4] = interpolate(p[4], p[5], val[4], val[5]);
            if (edgeTable[cubeindex] & 32)
                vertlist[5] = interpolate(p[5], p[6], val[5], val[6]);
            if (edgeTable[cubeindex] & 64)
                vertlist[6] = interpolate(p[6], p[7], val[6], val[7]);
            if (edgeTable[cubeindex] & 128)
                vertlist[7] = interpolate(p[7], p[4], val[7], val[4]);

            if (edgeTable[cubeindex] & 256)
                vertlist[8] = interpolate(p[0], p[4], val[0], val[4]);
            if (edgeTable[cubeindex] & 512)
                vertlist[9] = interpolate(p[1], p[5], val[1], val[5]);
            if (edgeTable[cubeindex] & 1024)
                vertlist[10] = interpolate(p[2], p[6], val[2], val[6]);
            if (edgeTable[cubeindex] & 2048)
                vertlist[11] = interpolate(p[3], p[7], val[3], val[7]);

            for (int i = 0; triTable[cubeindex][i] != -1; i += 3) {
                vec3f v1 = vertlist[triTable[cubeindex][i]];
                vec3f v2 = vertlist[triTable[cubeindex][i + 1]];
                vec3f v3 = vertlist[triTable[cubeindex][i + 2]];

                vec3f normal{0, 0, 0};
                if (computeNormals) {
                    normal = -((v2 - v1).cross(v3 - v1).normalize());
                }

                co_yield {
                    Triangle(
                        v1 * voxelData.voxel_size + voxelData.global_position,
                        v3 * voxelData.voxel_size + voxelData.global_position,
                        v2 * voxelData.voxel_size + voxelData.global_position),
                    normal
                };

                numTriangles++;
            }
        }
    }

    Generator<std::tuple<Triangle, vec3f>>
    generateMeshForChunk(
        sinriv::kigstudio::voxel::VoxelGrid& voxelData,
        uint64_t chunkKey,
        double isolevel,
        int& numTriangles,
        bool computeNormals,
        float expand)
    {
        (void)isolevel;
        numTriangles = 0;
        vec3f vertlist[12];

        auto getVoxel = [&](int wx, int wy, int wz) -> bool {
            return voxelData.contains(wx, wy, wz);
        };

        auto it = voxelData.chunks.find(chunkKey);
        if (it == voxelData.chunks.end()) co_return;

        int cx, cy, cz;
        unpackChunkKey(chunkKey, cx, cy, cz);

        int baseX = cx << 5;
        int baseY = cy << 5;
        int baseZ = cz << 5;

        for (int z = -1; z < 32; z++)
        for (int y = -1; y < 32; y++)
        for (int x = -1; x < 32; x++) {

            int wx = baseX + x;
            int wy = baseY + y;
            int wz = baseZ + z;

            bool v000 = getVoxel(wx,   wy,   wz);
            bool v100 = getVoxel(wx+1, wy,   wz);
            bool v110 = getVoxel(wx+1, wy+1, wz);
            bool v010 = getVoxel(wx,   wy+1, wz);

            bool v001 = getVoxel(wx,   wy,   wz+1);
            bool v101 = getVoxel(wx+1, wy,   wz+1);
            bool v111 = getVoxel(wx+1, wy+1, wz+1);
            bool v011 = getVoxel(wx,   wy+1, wz+1);

            int cubeindex = 0;
            cubeindex |= v000 << 0;
            cubeindex |= v100 << 1;
            cubeindex |= v110 << 2;
            cubeindex |= v010 << 3;
            cubeindex |= v001 << 4;
            cubeindex |= v101 << 5;
            cubeindex |= v111 << 6;
            cubeindex |= v011 << 7;

            if (cubeindex == 0 || cubeindex == 255)
                continue;

            if (edgeTable[cubeindex] == 0)
                continue;

            float fx = float(wx);
            float fy = float(wy);
            float fz = float(wz);

            vec3f cell_center{fx + 0.5f, fy + 0.5f, fz + 0.5f};

            if (edgeTable[cubeindex] & 1)
                vertlist[0] = {fx+.5f, fy, fz};
            if (edgeTable[cubeindex] & 2)
                vertlist[1] = {fx+1, fy+.5f, fz};
            if (edgeTable[cubeindex] & 4)
                vertlist[2] = {fx+.5f, fy+1, fz};
            if (edgeTable[cubeindex] & 8)
                vertlist[3] = {fx, fy+.5f, fz};

            if (edgeTable[cubeindex] & 16)
                vertlist[4] = {fx+.5f, fy, fz+1};
            if (edgeTable[cubeindex] & 32)
                vertlist[5] = {fx+1, fy+.5f, fz+1};
            if (edgeTable[cubeindex] & 64)
                vertlist[6] = {fx+.5f, fy+1, fz+1};
            if (edgeTable[cubeindex] & 128)
                vertlist[7] = {fx, fy+.5f, fz+1};

            if (edgeTable[cubeindex] & 256)
                vertlist[8] = {fx, fy, fz+.5f};
            if (edgeTable[cubeindex] & 512)
                vertlist[9] = {fx+1, fy, fz+.5f};
            if (edgeTable[cubeindex] & 1024)
                vertlist[10] = {fx+1, fy+1, fz+.5f};
            if (edgeTable[cubeindex] & 2048)
                vertlist[11] = {fx, fy+1, fz+.5f};

            for (int i = 0; triTable[cubeindex][i] != -1; i += 3) {
                vec3f v1 = vertlist[triTable[cubeindex][i]];
                vec3f v2 = vertlist[triTable[cubeindex][i+1]];
                vec3f v3 = vertlist[triTable[cubeindex][i+2]];

                vec3f normal{0,0,0};

                if (computeNormals || expand > 0.0f) {
                    normal = (v2 - v1).cross(v3 - v1).normalize();
                    normal = -normal;
                }

                if (expand > 0.0f) {
                    v1 += normal * expand;
                    v2 += normal * expand;
                    v3 += normal * expand;
                }

                co_yield {
                    Triangle(
                        v1 * voxelData.voxel_size + voxelData.global_position,
                        v3 * voxelData.voxel_size + voxelData.global_position,
                        v2 * voxelData.voxel_size + voxelData.global_position
                    ),
                    normal
                };

                numTriangles++;
            }
        }
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

                vec3f normal = { 0.f, 0.f, 0.f };

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

                co_yield std::tuple<Triangle, vec3f>(
                    Triangle(
                        (v1*voxelData.voxel_size)+voxelData.global_position, 
                        (v2*voxelData.voxel_size)+voxelData.global_position, 
                        (v3*voxelData.voxel_size)+voxelData.global_position),
                    normal
                );

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

                co_yield std::tuple<Triangle, vec3f>(
                    Triangle(
                        (v1*voxelData.voxel_size)+voxelData.global_position, 
                        (v2*voxelData.voxel_size)+voxelData.global_position, 
                        (v3*voxelData.voxel_size)+voxelData.global_position),
                    normal
                );

                numTriangles++;
            }
        }
    }

    void saveMeshToASCIISTL(const std::vector<std::tuple<Triangle, vec3f>>& meshTriangles, const std::string& filename) {
#ifdef _WIN32
        std::ofstream outFile(utf8_to_wstring_file(filename).c_str());
#else
        std::ofstream outFile(filename);
#endif

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
#ifdef _WIN32
        std::ofstream outFile(utf8_to_wstring_file(filename).c_str(), std::ios::binary);
#else
        std::ofstream outFile(filename, std::ios::binary);
#endif

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
#ifdef _WIN32
        std::ifstream file(utf8_to_wstring_file(filename).c_str());
#else
        std::ifstream file(filename);
#endif
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
#ifdef _WIN32
        std::ifstream file(utf8_to_wstring_file(filename).c_str(), std::ios::binary);
#else
        std::ifstream file(filename, std::ios::binary);
#endif
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
#ifdef _WIN32
        std::ifstream file(utf8_to_wstring_file(filePath).c_str(), std::ios::binary);
#else
        std::ifstream file(filePath, std::ios::binary);
#endif
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
