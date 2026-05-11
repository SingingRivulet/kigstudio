#pragma once

#include <vector>
#include <tuple>
#include <set>
#include <map>
#include <string>
#include <fstream>
#include <cmath>
#include "kigstudio/voxel/octree.h"
#include "kigstudio/voxel/triangle_bvh.h"
#include "kigstudio/utils/generator.h"
namespace sinriv::kigstudio::voxel {

    using vec3f = sinriv::kigstudio::vec3<float>;
    using Triangle = triangle_bvh<float>::triangle;

    Generator<std::tuple<Triangle,vec3f>> generateMesh(sinriv::kigstudio::voxel::VoxelGrid& voxelData, double isolevel, int& numTriangles, bool computeNormals = false, float expand = 0.0f);
    Generator<std::tuple<Triangle, vec3f>> generateMesh(sinriv::kigstudio::octree::Octree& voxelData, double isolevel, int& numTriangles, bool computeNormals = false);
    Generator<std::tuple<Triangle, vec3f>> generateMeshForChunk(sinriv::kigstudio::voxel::VoxelGrid& voxelData, uint64_t chunkKey, double isolevel, int& numTriangles, bool computeNormals = false, float expand = 0.0f);

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

    struct NonManifoldEdge {
        vec3f v0, v1;  // 边两端点坐标
        int triangle_count;  // 共享该边的三角形数量
    };

    // 检查非流形边（被超过2个三角形共享的边）
    inline std::vector<NonManifoldEdge> checkNonManifoldEdges(
        const std::vector<std::tuple<Triangle, vec3f>>& input) {
        // 1. 过滤退化三角形
        std::vector<std::tuple<Triangle, vec3f>> filtered;
        filtered.reserve(input.size());
        for (const auto& [tri, n] : input) {
            auto v1 = std::get<0>(tri);
            auto v2 = std::get<1>(tri);
            auto v3 = std::get<2>(tri);
            if ((v2 - v1).cross(v3 - v1).length() < 1e-6f) continue;
            filtered.push_back({tri, n});
        }

        // 2. 顶点焊接（1e-6 精度）
        std::vector<vec3f> welded_vertices;
        std::map<std::tuple<int, int, int>, int> vertex_map;
        auto weld = [&](const vec3f& v) -> int {
            int qx = static_cast<int>(std::round(v.x * 1e6f));
            int qy = static_cast<int>(std::round(v.y * 1e6f));
            int qz = static_cast<int>(std::round(v.z * 1e6f));
            auto key = std::make_tuple(qx, qy, qz);
            auto it = vertex_map.find(key);
            if (it != vertex_map.end()) return it->second;
            int idx = static_cast<int>(welded_vertices.size());
            welded_vertices.push_back(v);
            vertex_map[key] = idx;
            return idx;
        };

        struct IdxTri { int v[3]; vec3f n; };
        std::vector<IdxTri> idx_tris;
        idx_tris.reserve(filtered.size());
        for (const auto& [tri, n] : filtered) {
            IdxTri t{ { weld(std::get<0>(tri)), weld(std::get<1>(tri)), weld(std::get<2>(tri)) }, n };
            idx_tris.push_back(t);
        }

        // 3. 去除完全重复的三角形（规范化顶点顺序后比较）
        std::set<std::tuple<int, int, int>> seen_tris;
        std::vector<IdxTri> unique_tris;
        unique_tris.reserve(idx_tris.size());
        for (const auto& t : idx_tris) {
            int a = t.v[0], b = t.v[1], c = t.v[2];
            if (a <= b && a <= c) {
                // a 最小，保持 a-b-c
            } else if (b <= a && b <= c) {
                int tmp = a; a = b; b = c; c = tmp;
            } else {
                int tmp = a; a = c; c = b; b = tmp;
            }
            auto key = std::make_tuple(a, b, c);
            if (seen_tris.insert(key).second) {
                unique_tris.push_back(t);
            }
        }

        // 4. 建立边 -> 三角形索引列表映射
        std::map<std::pair<int, int>, std::vector<int>> edge_to_tris;
        for (int i = 0; i < static_cast<int>(unique_tris.size()); ++i) {
            for (int e = 0; e < 3; ++e) {
                int a = unique_tris[i].v[e];
                int b = unique_tris[i].v[(e + 1) % 3];
                if (a > b) std::swap(a, b);
                edge_to_tris[{a, b}].push_back(i);
            }
        }

        // 5. 收集非流形边
        std::vector<NonManifoldEdge> result;
        for (const auto& [edge, tri_list] : edge_to_tris) {
            if (tri_list.size() <= 2) continue;
            result.push_back({
                welded_vertices[edge.first],
                welded_vertices[edge.second],
                static_cast<int>(tri_list.size())
            });
        }
        return result;
    }

    // 网格清理：去除退化/重复三角形，并修复非流形边（被超过2个三角形共享的边）
    inline std::vector<std::tuple<Triangle, vec3f>> cleanMesh(
        const std::vector<std::tuple<Triangle, vec3f>>& input) {
        // 1. 过滤退化三角形
        std::vector<std::tuple<Triangle, vec3f>> filtered;
        filtered.reserve(input.size());
        for (const auto& [tri, n] : input) {
            auto v1 = std::get<0>(tri);
            auto v2 = std::get<1>(tri);
            auto v3 = std::get<2>(tri);
            if ((v2 - v1).cross(v3 - v1).length() < 1e-6f) continue;
            filtered.push_back({tri, n});
        }

        // 2. 顶点焊接（1e-6 精度）
        std::vector<vec3f> welded_vertices;
        std::map<std::tuple<int, int, int>, int> vertex_map;
        auto weld = [&](const vec3f& v) -> int {
            int qx = static_cast<int>(std::round(v.x * 1e6f));
            int qy = static_cast<int>(std::round(v.y * 1e6f));
            int qz = static_cast<int>(std::round(v.z * 1e6f));
            auto key = std::make_tuple(qx, qy, qz);
            auto it = vertex_map.find(key);
            if (it != vertex_map.end()) return it->second;
            int idx = static_cast<int>(welded_vertices.size());
            welded_vertices.push_back(v);
            vertex_map[key] = idx;
            return idx;
        };

        struct IdxTri { int v[3]; vec3f n; };
        std::vector<IdxTri> idx_tris;
        idx_tris.reserve(filtered.size());
        for (const auto& [tri, n] : filtered) {
            IdxTri t{ { weld(std::get<0>(tri)), weld(std::get<1>(tri)), weld(std::get<2>(tri)) }, n };
            idx_tris.push_back(t);
        }

        // 3. 去除完全重复的三角形（规范化顶点顺序后比较）
        std::set<std::tuple<int, int, int>> seen_tris;
        std::vector<IdxTri> unique_tris;
        unique_tris.reserve(idx_tris.size());
        for (const auto& t : idx_tris) {
            int a = t.v[0], b = t.v[1], c = t.v[2];
            // 规范化循环顺序（从最小索引开始）
            if (a <= b && a <= c) {
                // a 最小，保持 a-b-c
            } else if (b <= a && b <= c) {
                int tmp = a; a = b; b = c; c = tmp;
            } else {
                int tmp = a; a = c; c = b; b = tmp;
            }
            auto key = std::make_tuple(a, b, c);
            if (seen_tris.insert(key).second) {
                unique_tris.push_back(t);
            }
        }

        // 4. 建立边 -> 三角形索引列表映射
        std::map<std::pair<int, int>, std::vector<int>> edge_to_tris;
        for (int i = 0; i < static_cast<int>(unique_tris.size()); ++i) {
            for (int e = 0; e < 3; ++e) {
                int a = unique_tris[i].v[e];
                int b = unique_tris[i].v[(e + 1) % 3];
                if (a > b) std::swap(a, b);
                edge_to_tris[{a, b}].push_back(i);
            }
        }

        // 5. 修复非流形边：移除导致边被超过 2 个三角形共享的多余三角形
        std::set<int> tris_to_remove;
        for (const auto& [edge, tri_list] : edge_to_tris) {
            if (tri_list.size() <= 2) continue;
            // 按三角形 key 分组，完全重复的只保留一个
            std::map<std::tuple<int, int, int>, std::vector<int>> groups;
            for (int idx : tri_list) {
                int a = unique_tris[idx].v[0];
                int b = unique_tris[idx].v[1];
                int c = unique_tris[idx].v[2];
                if (a > b) std::swap(a, b);
                if (b > c) std::swap(b, c);
                if (a > b) std::swap(a, b);
                groups[{a, b, c}].push_back(idx);
            }
            std::vector<int> remaining;
            for (auto& [gkey, group] : groups) {
                remaining.push_back(group[0]); // 每组保留一个
                for (size_t i = 1; i < group.size(); ++i) {
                    tris_to_remove.insert(group[i]);
                }
            }
            // 如果去重后仍然超过 2 个，保留面积最大的两个，移除其余
            if (remaining.size() > 2) {
                // 计算每个三角形的面积（cross 长度）
                std::vector<std::pair<int, float>> area_list;
                for (int idx : remaining) {
                    const auto& t = unique_tris[idx];
                    vec3f a = welded_vertices[t.v[0]];
                    vec3f b = welded_vertices[t.v[1]];
                    vec3f c = welded_vertices[t.v[2]];
                    float area = (b - a).cross(c - a).length();
                    area_list.push_back({idx, area});
                }
                std::sort(area_list.begin(), area_list.end(),
                          [](const auto& x, const auto& y) { return x.second > y.second; });
                for (size_t i = 2; i < area_list.size(); ++i) {
                    tris_to_remove.insert(area_list[i].first);
                }
            }
        }

        // 6. 重建输出
        std::vector<std::tuple<Triangle, vec3f>> output;
        output.reserve(unique_tris.size() - tris_to_remove.size());
        for (int i = 0; i < static_cast<int>(unique_tris.size()); ++i) {
            if (tris_to_remove.find(i) == tris_to_remove.end()) {
                const auto& t = unique_tris[i];
                output.push_back({
                    Triangle(
                        welded_vertices[t.v[0]],
                        welded_vertices[t.v[1]],
                        welded_vertices[t.v[2]]),
                    t.n
                });
            }
        }
        return output;
    }
}
