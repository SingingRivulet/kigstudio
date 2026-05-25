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
        subdivisions = std::max(1, subdivisions);
        numTriangles = 0;

        DenseGrid dense = buildDenseGrid(voxelData, 2);
        if (dense.sx <= 1 || dense.sy <= 1 || dense.sz <= 1) {
            co_return;
        }

        SDFGrid sdf = buildSDF(dense);

        // --- helper: trilinear interpolation on the base SDF ---
        auto sampleSDF = [&](float x, float y, float z) -> float {
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

        // Upsample the SDF if subdivisions > 1
        std::unique_ptr<SDFGrid> usdf;
        SDFGrid* psdf = &sdf;
        if (subdivisions > 1) {
            usdf = std::make_unique<SDFGrid>();
            usdf->min_bound = sdf.min_bound;
            usdf->max_bound = sdf.max_bound;
            usdf->sx = (sdf.sx - 1) * subdivisions + 1;
            usdf->sy = (sdf.sy - 1) * subdivisions + 1;
            usdf->sz = (sdf.sz - 1) * subdivisions + 1;
            usdf->sdf.resize(static_cast<size_t>(usdf->sx) * usdf->sy * usdf->sz);

            const float inv_sub = 1.0f / static_cast<float>(subdivisions);
            for (int z = 0; z < usdf->sz; ++z) {
                for (int y = 0; y < usdf->sy; ++y) {
                    for (int x = 0; x < usdf->sx; ++x) {
                        float val = sampleSDF(
                            static_cast<float>(x) * inv_sub,
                            static_cast<float>(y) * inv_sub,
                            static_cast<float>(z) * inv_sub);
                        usdf->set(x, y, z, val);
                    }
                }
            }
            psdf = usdf.get();
        }

        // Surface Nets lookup tables
        static const int CUBE_CORNERS[8][3] = {
            {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0},
            {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}
        };
        static const vec3f CUBE_CORNER_VECTORS[8] = {
            {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}
        };
        static const int CUBE_EDGES[12][2] = {
            {0, 1}, {0, 2}, {0, 4}, {1, 3}, {1, 5}, {2, 3},
            {2, 6}, {3, 7}, {4, 5}, {4, 6}, {5, 7}, {6, 7}
        };

        const int sx = psdf->sx;
        const int sy = psdf->sy;
        const int sz = psdf->sz;
        const float inv_subdiv = 1.0f / static_cast<float>(subdivisions);

        // --- helper: estimate centroid of edge intersections inside a unit cube ---
        auto centroid_of_edge_intersections = [&](const float dists[8]) -> vec3f {
            int count = 0;
            vec3f sum(0.0f, 0.0f, 0.0f);
            for (int e = 0; e < 12; ++e) {
                int c1 = CUBE_EDGES[e][0];
                int c2 = CUBE_EDGES[e][1];
                float d1 = dists[c1];
                float d2 = dists[c2];
                if ((d1 < 0.0f) != (d2 < 0.0f)) {
                    count++;
                    float interp = d1 / (d1 - d2);
                    const vec3f& corner1 = CUBE_CORNER_VECTORS[c1];
                    const vec3f& corner2 = CUBE_CORNER_VECTORS[c2];
                    sum = sum + corner1 * (1.0f - interp) + corner2 * interp;
                }
            }
            if (count == 0) return vec3f(0.5f, 0.5f, 0.5f);
            return sum / static_cast<float>(count);
        };

        // --- helper: SDF gradient via bilinear interpolation of edge differences ---
        auto sdf_gradient = [&](const float dists[8], const vec3f& s) -> vec3f {
            vec3f p00(dists[0b001], dists[0b010], dists[0b100]);
            vec3f n00(dists[0b000], dists[0b000], dists[0b000]);

            vec3f p10(dists[0b101], dists[0b011], dists[0b110]);
            vec3f n10(dists[0b100], dists[0b001], dists[0b010]);

            vec3f p01(dists[0b011], dists[0b110], dists[0b101]);
            vec3f n01(dists[0b010], dists[0b100], dists[0b001]);

            vec3f p11(dists[0b111], dists[0b111], dists[0b111]);
            vec3f n11(dists[0b110], dists[0b101], dists[0b011]);

            vec3f d00 = p00 - n00;
            vec3f d10 = p10 - n10;
            vec3f d01 = p01 - n01;
            vec3f d11 = p11 - n11;

            float nx = 1.0f - s.x;
            float ny = 1.0f - s.y;
            float nz = 1.0f - s.z;

            vec3f grad;
            grad.x = (ny * nz) * d00.x + (ny * s.z) * d10.x + (s.y * nz) * d01.x + (s.y * s.z) * d11.x;
            grad.y = (nz * nx) * d00.y + (nz * s.x) * d10.y + (s.z * nx) * d01.y + (s.z * s.x) * d11.y;
            grad.z = (nx * ny) * d00.z + (nx * s.y) * d10.z + (s.x * ny) * d01.z + (s.x * s.y) * d11.z;
            return grad;
        };

        // --- Phase 1: estimate surface point and normal for every cube ---
        std::vector<vec3f> positions;
        std::vector<vec3f> normals;
        std::vector<uint32_t> stride_to_index(static_cast<size_t>(sx) * sy * sz, UINT32_MAX);

        for (int z = 0; z < sz - 1; ++z) {
            for (int y = 0; y < sy - 1; ++y) {
                for (int x = 0; x < sx - 1; ++x) {
                    float corner_dists[8];
                    int num_negative = 0;
                    for (int i = 0; i < 8; ++i) {
                        int cx = x + CUBE_CORNERS[i][0];
                        int cy = y + CUBE_CORNERS[i][1];
                        int cz = z + CUBE_CORNERS[i][2];
                        float d = psdf->get(cx, cy, cz);
                        corner_dists[i] = d;
                        if (d < 0.0f) num_negative++;
                    }

                    if (num_negative == 0 || num_negative == 8) {
                        continue;
                    }

                    vec3f c = centroid_of_edge_intersections(corner_dists);
                    vec3f pos(
                        static_cast<float>(psdf->min_bound.x) + (static_cast<float>(x) + c.x) * inv_subdiv,
                        static_cast<float>(psdf->min_bound.y) + (static_cast<float>(y) + c.y) * inv_subdiv,
                        static_cast<float>(psdf->min_bound.z) + (static_cast<float>(z) + c.z) * inv_subdiv
                    );
                    vec3f normal = sdf_gradient(corner_dists, c);

                    uint32_t idx = static_cast<uint32_t>(positions.size());
                    positions.push_back(pos);
                    normals.push_back(normal);
                    stride_to_index[psdf->index(x, y, z)] = idx;
                }
            }
        }

        if (positions.empty()) {
            co_return;
        }

        // --- Phase 2: emit quads for every crossing edge ---
        int x_stride = psdf->index(1, 0, 0) - psdf->index(0, 0, 0); // == 1
        int y_stride = psdf->index(0, 1, 0) - psdf->index(0, 0, 0); // == sx
        int z_stride = psdf->index(0, 0, 1) - psdf->index(0, 0, 0); // == sx*sy

        std::vector<uint32_t> indices;
        indices.reserve(positions.size() * 6);

        auto maybe_make_quad = [&](int p1, int p2, int axis_b_stride, int axis_c_stride) {
            float d1 = psdf->sdf[p1];
            float d2 = psdf->sdf[p2];
            bool negative_face;
            if (d1 < 0.0f && d2 >= 0.0f) {
                negative_face = false;
            } else if (d1 >= 0.0f && d2 < 0.0f) {
                negative_face = true;
            } else {
                return;
            }

            uint32_t v1 = stride_to_index[p1];
            uint32_t v2 = stride_to_index[p1 - axis_b_stride];
            uint32_t v3 = stride_to_index[p1 - axis_c_stride];
            uint32_t v4 = stride_to_index[p1 - axis_b_stride - axis_c_stride];

            if (v1 == UINT32_MAX || v2 == UINT32_MAX || v3 == UINT32_MAX || v4 == UINT32_MAX) {
                return;
            }

            const vec3f& pos1 = positions[v1];
            const vec3f& pos2 = positions[v2];
            const vec3f& pos3 = positions[v3];
            const vec3f& pos4 = positions[v4];

            float d14 = pos1.dist2(pos4);
            float d23 = pos2.dist2(pos3);

            if (d14 < d23) {
                if (negative_face) {
                    indices.push_back(v1); indices.push_back(v4); indices.push_back(v2);
                    indices.push_back(v1); indices.push_back(v3); indices.push_back(v4);
                } else {
                    indices.push_back(v1); indices.push_back(v2); indices.push_back(v4);
                    indices.push_back(v1); indices.push_back(v4); indices.push_back(v3);
                }
            } else {
                if (negative_face) {
                    indices.push_back(v2); indices.push_back(v3); indices.push_back(v4);
                    indices.push_back(v2); indices.push_back(v1); indices.push_back(v3);
                } else {
                    indices.push_back(v2); indices.push_back(v4); indices.push_back(v3);
                    indices.push_back(v2); indices.push_back(v3); indices.push_back(v1);
                }
            }
        };

        for (int z = 0; z < sz - 1; ++z) {
            for (int y = 0; y < sy - 1; ++y) {
                for (int x = 0; x < sx - 1; ++x) {
                    int p_stride = psdf->index(x, y, z);
                    if (stride_to_index[p_stride] == UINT32_MAX) continue;

                    // X axis edge
                    if (y > 0 && z > 0 && x < sx - 2) {
                        maybe_make_quad(p_stride, p_stride + x_stride, y_stride, z_stride);
                    }
                    // Y axis edge
                    if (x > 0 && z > 0 && y < sy - 2) {
                        maybe_make_quad(p_stride, p_stride + y_stride, z_stride, x_stride);
                    }
                    // Z axis edge
                    if (x > 0 && y > 0 && z < sz - 2) {
                        maybe_make_quad(p_stride, p_stride + z_stride, x_stride, y_stride);
                    }
                }
            }
        }

        // --- Phase 3: yield triangles ---
        for (size_t i = 0; i + 2 < indices.size(); i += 3) {
            uint32_t i0 = indices[i];
            uint32_t i1 = indices[i + 1];
            uint32_t i2 = indices[i + 2];

            vec3f v1 = positions[i0];
            vec3f v2 = positions[i1];
            vec3f v3 = positions[i2];

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
        using vec3i = sinriv::kigstudio::Vec3i;
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
