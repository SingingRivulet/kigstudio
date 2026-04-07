#pragma once
#include <iomanip>
#include "kigstudio/utils/dbvt3d.h"
#include "kigstudio/utils/vec3.h"
namespace sinriv::kigstudio::voxel {

template <sinriv::kigstudio::Numeric number_t>
struct triangle_bvh {
    using triangle = std::tuple<vec3<number_t>, vec3<number_t>, vec3<number_t>>;
    struct trangle_box {
        triangle vertex;
        sinriv::kigstudio::dbvt3d<number_t, trangle_box>::AABB* boundBox;
        inline bool rayTest(const ray<number_t>& r,
                            vec3<number_t>& coll_pos) const {
            const auto& v0 = std::get<0>(vertex);
            const auto& v1 = std::get<1>(vertex);
            const auto& v2 = std::get<2>(vertex);

            // 1. 归一化方向
            vec3<number_t> dir = r.direction();
            number_t len = dir.length();
            if (len == 0)
                return false;
            dir = dir * (1.0f / len);

            // 2. 边向量
            vec3<number_t> edge1 = v1 - v0;
            vec3<number_t> edge2 = v2 - v0;

            // 3. 关键向量计算
            vec3<number_t> h = dir.cross(edge2);
            number_t a = edge1.dot(h);

            // 打印这些关键值
            // std::cout << "  [DEBUG] edge1: " << edge1 << std::endl;
            // std::cout << "  [DEBUG] edge2: " << edge2 << std::endl;
            // std::cout << "  [DEBUG] h: " << h << std::endl;
            // std::cout << "  [DEBUG] a (determinant): " << a << std::endl;

            if (std::abs(a) < 1e-8)
                return false;

            number_t f = 1.0 / a;
            vec3<number_t> s = r.begin - v0;

            // std::cout << "  [DEBUG] s (origin - v0): " << s << std::endl;
            // std::cout << "  [DEBUG] s.dot(h): " << s.dot(h) << std::endl;

            number_t u = f * s.dot(h);
            // std::cout << "  [DEBUG] u: " << u << std::endl;

            if (u < 0.0 || u > 1.0)
                return false;

            vec3<number_t> q = s.cross(edge1);
            number_t v = f * dir.dot(q);
            // std::cout << "  [DEBUG] v: " << v << std::endl;

            if (v < 0.0 || u + v > 1.0)
                return false;

            number_t t = f * edge2.dot(q);
            // std::cout << "  [DEBUG] t: " << t << std::endl;

            if (t > 1e-4) {
                coll_pos = r.begin + dir * t;
                return true;
            }
            return false;
        }
    };

    dbvt3d<number_t, trangle_box> bvh;

    std::vector<std::unique_ptr<trangle_box>> trangles{};
    vec3<number_t> global_boundBox_min{}, global_boundBox_max{};

    inline auto insert(const triangle& triangle) {
        auto ptr = std::make_unique<trangle_box>();
        ptr->vertex = triangle;

        // 计算三角形的包围盒
        typename sinriv::kigstudio::dbvt3d<number_t, trangle_box>::vec3_n
            boundBox_min,
            boundBox_max;

        auto updateBounds = [](const auto& v, auto& min, auto& max) {
            min.x = std::min(min.x, v.x);
            min.y = std::min(min.y, v.y);
            min.z = std::min(min.z, v.z);
            max.x = std::max(max.x, v.x);
            max.y = std::max(max.y, v.y);
            max.z = std::max(max.z, v.z);
        };

        boundBox_min.x = std::get<0>(triangle).x;
        boundBox_min.y = std::get<0>(triangle).y;
        boundBox_min.z = std::get<0>(triangle).z;
        boundBox_max.x = std::get<0>(triangle).x;
        boundBox_max.y = std::get<0>(triangle).y;
        boundBox_max.z = std::get<0>(triangle).z;
        updateBounds(std::get<0>(triangle), boundBox_min, boundBox_max);
        updateBounds(std::get<1>(triangle), boundBox_min, boundBox_max);
        updateBounds(std::get<2>(triangle), boundBox_min, boundBox_max);

        // 计算全局包围盒
        if (trangles.empty()) {
            global_boundBox_min = boundBox_min;
            global_boundBox_max = boundBox_max;
        } else {
            global_boundBox_min.x =
                std::min(global_boundBox_min.x, boundBox_min.x);
            global_boundBox_min.y =
                std::min(global_boundBox_min.y, boundBox_min.y);
            global_boundBox_min.z =
                std::min(global_boundBox_min.z, boundBox_min.z);
            global_boundBox_max.x =
                std::max(global_boundBox_max.x, boundBox_max.x);
            global_boundBox_max.y =
                std::max(global_boundBox_max.y, boundBox_max.y);
            global_boundBox_max.z =
                std::max(global_boundBox_max.z, boundBox_max.z);
        }

        ptr->boundBox = bvh.add(
            typename sinriv::kigstudio::dbvt3d<number_t, trangle_box>::vec3_n(
                boundBox_min),
            typename sinriv::kigstudio::dbvt3d<number_t, trangle_box>::vec3_n(
                boundBox_max),
            ptr.get());

        auto res = ptr.get();
        trangles.push_back(std::move(ptr));
        return res;
    }

    template <typename Func_t>
    inline void rayTest(const ray<number_t>& r, Func_t callback) {
        bvh.rayTest(r, [&](auto node) {
            auto n = node->data;
            // std::cout << "rayTest: " << std::fixed << std::setprecision(2)
            //           << std::get<0>(n->vertex) << std::get<1>(n->vertex)
            //           << std::get<2>(n->vertex) << " box:" <<
            //           n->boundBox->begin
            //           << n->boundBox->end;
            vec3<number_t> coll_pos;
            if (n->rayTest(r, coll_pos)) {
                callback(n, coll_pos);
                // std::cout << " (coll)";
            }
            // std::cout << std::endl;
        });
    }

    template <typename Func_t>
    inline void getSolid(const ray<number_t>& r, Func_t callback) {
        // 计算射线上位于物体内部的区间
        std::vector<std::tuple<vec3<number_t>, number_t>> coll_pos_list;
        rayTest(r, [&](auto node, auto coll_pos) {
            coll_pos_list.push_back(
                std::tuple<vec3<number_t>, number_t>(coll_pos, 0.));
        });
        if (coll_pos_list.empty()) {
            return;
        } else {
            // std::cout << "ray=" << r << std::endl;
            // std::cout << "coll_pos_list.size() = " << coll_pos_list.size()
            //           << std::endl;
        }
        for (auto& coll_pos : coll_pos_list) {
            std::get<1>(coll_pos) = (std::get<0>(coll_pos) - r.begin).length();
        }
        // 按离起点的距离排序
        std::sort(coll_pos_list.begin(), coll_pos_list.end(),
                  [](const auto& a, const auto& b) {
                      return std::get<1>(a) < std::get<1>(b);
                  });
        // 相邻两个配对
        for (size_t i = 0; i < coll_pos_list.size() - 1; i += 2) {
            callback(std::get<0>(coll_pos_list[i]),
                     std::get<0>(coll_pos_list[i + 1]));
        }
    }

    typedef enum {
        voxel_face_X = 0,  // yz面
        voxel_face_Y = 1,  // xz面
        voxel_face_Z = 2   // xy面
    } voxel_face_e;
    // 从坐标轴平面按指定间距发射一系列射线进行处理
    template <typename Func_t>
    inline void getSolidByFace(float voxelsizex,   // Voxel size on X-axis
                               float voxelsizey,   // Voxel size on Y-axis
                               float voxelsizez,   // Voxel size on Z-axis
                               voxel_face_e face,  // Face to emit rays
                               Func_t callback) {
        if (trangles.empty()) {
            return;
        }
        auto min = global_boundBox_min;
        auto max = global_boundBox_max;
        int num_block_x = ceil((max.x - min.x) / voxelsizex);
        int num_block_y = ceil((max.y - min.y) / voxelsizey);
        int num_block_z = ceil((max.z - min.z) / voxelsizez);
        if (face == voxel_face_X) {
            // std::cout << "getSolidByFace by voxel_face_X" << std::endl;
            #pragma omp for collapse(2)
            for (int i = 0; i < num_block_y; ++i) {
                for (int j = 0; j < num_block_z; ++j) {
                    auto ray_ori = vec3<number_t>(min.x, min.y + i * voxelsizey,
                                                  min.z + j * voxelsizez);
                    auto ray_end = vec3<number_t>(max.x, min.y + i * voxelsizey,
                                                  min.z + j * voxelsizez);
                    ray<number_t> ray(ray_ori, ray_end);
                    getSolid(ray, [&](auto start, auto end) {
                        callback(start, end);
                    });
                }
            }
        } else if (face == voxel_face_Y) {
            // std::cout << "getSolidByFace by voxel_face_Y" << std::endl;
            #pragma omp for collapse(2)
            for (int i = 0; i < num_block_x; ++i) {
                for (int j = 0; j < num_block_z; ++j) {
                    auto ray_ori = vec3<number_t>(min.x + i * voxelsizex, min.y,
                                                  min.z + j * voxelsizez);
                    auto ray_end = vec3<number_t>(min.x + i * voxelsizex, max.y,
                                                  min.z + j * voxelsizez);
                    ray<number_t> ray(ray_ori, ray_end);
                    getSolid(ray, [&](auto start, auto end) {
                        callback(start, end);
                    });
                }
            }
        } else if (face == voxel_face_Z) {
            // std::cout << "getSolidByFace by voxel_face_Z" << std::endl;
            #pragma omp for collapse(2)
            for (int i = 0; i < num_block_x; ++i) {
                auto ray_ori =
                    vec3<number_t>(min.x + i * voxelsizex, min.y, min.z);
                auto ray_end =
                    vec3<number_t>(min.x + i * voxelsizex, min.y, max.z);
                ray<number_t> ray(ray_ori, ray_end);
                getSolid(ray,
                         [&](auto start, auto end) { callback(start, end); });
            }
        }
    }
};

}  // namespace sinriv::kigstudio::voxel