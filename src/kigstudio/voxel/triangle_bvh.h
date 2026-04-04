#pragma once
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
            // 射线测试，并返回在三角形上碰撞的位置
            const auto& v0 = std::get<0>(vertex);
            const auto& v1 = std::get<1>(vertex);
            const auto& v2 = std::get<2>(vertex);

            // 计算三角形的两条边
            vec3<number_t> edge1 = v1 - v0;
            vec3<number_t> edge2 = v2 - v0;

            // 计算行列式
            vec3<number_t> h = r.direction.cross(edge2);
            number_t a = edge1.dot(h);

            // 射线与三角形平行
            if (a > -1e-6 && a < 1e-6)
                return false;

            number_t f = 1.0 / a;
            vec3<number_t> s = r.origin - v0;
            number_t u = f * s.dot(h);

            // 检查u是否在三角形内
            if (u < 0.0 || u > 1.0)
                return false;

            vec3<number_t> q = s.cross(edge1);
            number_t v = f * r.direction.dot(q);

            // 检查v是否在三角形内
            if (v < 0.0 || u + v > 1.0)
                return false;

            // 计算相交距离
            number_t t = f * edge2.dot(q);

            // 检查相交点是否在射线前方
            if (t > 1e-6) {
                coll_pos = r.origin + r.direction * t;
                return true;
            }

            return false;
        }
    };

    dbvt3d<number_t, trangle_box> bvh;

    std::vector<std::unique_ptr<trangle_box>> trangles;

    inline auto insert(const triangle& triangle) {
        auto ptr = std::make_unique<trangle_box>();
        ptr->vertex = triangle;

        // 计算包围盒
        typename sinriv::kigstudio::dbvt3d<number_t>::vec3_n boundBox_min,
            boundBox_max;

        auto updateBounds = [](const auto& v, auto& min, auto& max) {
            min.x = std::min(min.x, v.x);
            min.y = std::min(min.y, v.y);
            min.z = std::min(min.z, v.z);
            max.x = std::max(max.x, v.x);
            max.y = std::max(max.y, v.y);
            max.z = std::max(max.z, v.z);
        };

        updateBounds(std::get<0>(triangle), boundBox_min, boundBox_max);
        updateBounds(std::get<1>(triangle), boundBox_min, boundBox_max);
        updateBounds(std::get<2>(triangle), boundBox_min, boundBox_max);

        ptr->boundBox = bvh.add(
            typename sinriv::kigstudio::dbvt3d<number_t>::vec3_n(boundBox_min),
            typename sinriv::kigstudio::dbvt3d<number_t>::vec3_n(boundBox_max),
            ptr.get());

        auto res = ptr.get();
        trangles.push_back(std::move(ptr));
        return res;
    }

    template <typename Func_t>
    inline void rayTest(const ray<number_t>& r, Func_t callback) {
        bvh.rayTest(r, [&](auto node) {
            auto n = node->data;
            vec3<number_t> coll_pos;
            if (n->rayTest(r, coll_pos)) {
                callback(n, coll_pos);
            }
        });
    }
};

}  // namespace sinriv::kigstudio::voxel