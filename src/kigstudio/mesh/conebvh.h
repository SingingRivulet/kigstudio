#pragma once
#include <cJSON.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>
#include "kigstudio/utils/vec3.h"

namespace sinriv::kigstudio::mesh::conebvh {
/*
 * 用于检索假发的专用BVH
 * 用户需要提供一个统一的 apex 点和一个Mesh
 * 程序会将Mesh的每个三角形与 apex 形成一个四面体
 * 每个四面体会被包围在一个圆锥中，并构建BVH以支持快速查询
 * 最终输出的是类似SDF的距离查询接口，返回点到最近四面体的距离（负数表示在四面体内部）
 */

typedef ::sinriv::kigstudio::vec3<float> vec3f;
struct Cone {
    vec3f direction;  // 圆锥的轴向单位向量，指向圆锥的开口
    float angle;      // 圆锥的半顶角，单位为弧度
    float length;     // 圆锥的高度（从 apex 到底面）
    Cone() = default;
    Cone(const ::sinriv::kigstudio::vec3<float>& direction_,
         float angle_,
         float length_ = 1.0f)
        : direction(direction_), angle(angle_), length(length_) {}
};
class ConeGroup {
   public:
    vec3f apex;  // 所有圆锥共用一个顶点
    float get_distance(const Cone& cone, const vec3f& point)
        const;  // 计算点到圆锥的距离，负数表示在圆锥内部
    bool contains(const Cone& cone,
                  const vec3f& point) const;  // 判断点是否在圆锥内部
    void merge(const Cone& cone_A, const Cone& cone_B, Cone& cone_out)
        const;  // 将cone_A和cone_B合并成一个新的圆锥cone_out，cone_out包含cone_A和cone_B
};

// 静态 Cone-BVH 数据结构和查询接口（不包含构建）
class ConeBVHTree : public ConeGroup {
   public:
    struct Node {
        Cone bound;
        int left = -1;
        int right = -1;
        int first = -1;  // primitives index start (for leaf)
        int count = 0;   // primitives count (leaf if >0)
        inline bool isLeaf() const { return count > 0; }
    };

    std::vector<Node> nodes;
    std::vector<Cone> primitives;  // primitive cones referenced by leaf ranges
    std::vector<int>
        primitive_indices;  // optional leaf index mapping into primitives
    int root = -1;

    // 构建 BVH，使用统一 apex 和 cone 合并构建纯 cone 包围体
    void build(const std::vector<Cone>& cones, const vec3f& apex);

    // 外部设置树数据（构建在外部完成）
    void set_tree(std::vector<Node>&& n, std::vector<Cone>&& prims);

    // 严格点查询：遍历 tree 并精确判断 cone.contains
    void query_point_strict(const vec3f& p,
                            const std::function<void(int)>& callback) const;

    // 最近点查询：返回距离最近的 primitive 索引和距离
    bool query_nearest(const vec3f& p, int& outIndex, float& outDistance) const;

    // 最近 K 个 primitive 候选，按距离绝对值升序返回
    bool query_nearest_k(const vec3f& p,
                         int k,
                         std::vector<std::pair<int, float>>& outResults) const;
};

class TetraConeBVHTree : public ConeBVHTree {
   public:
    using triangle = std::tuple<vec3f, vec3f, vec3f>;

    std::vector<triangle> tetra_bases;

    // 通过三角形顶点构建以 apex 为公共顶点的封底圆锥 BVH
    void build(const std::vector<triangle>& bases, const vec3f& apex);

    // 返回最近的四面体索引和封底圆锥近似距离
    bool query_nearest_tetra(const vec3f& p,
                             int& outTetraIndex,
                             float& outSignedDistance) const;

    static Cone make_cone_from_triangle(const triangle& t, const vec3f& apex);
    static float point_to_triangle_distance(const vec3f& p,
                                            const vec3f& a,
                                            const vec3f& b,
                                            const vec3f& c);
    static bool point_in_tetra(const vec3f& p,
                               const vec3f& a,
                               const vec3f& b,
                               const vec3f& c,
                               const vec3f& d);
    static float signed_distance_to_tetra(const vec3f& p,
                                          const vec3f& a,
                                          const vec3f& b,
                                          const vec3f& c,
                                          const vec3f& d);

    bool query_nearest_tetra_k(
        const vec3f& p,
        int k,
        std::vector<std::pair<int, float>>& outTetraDistances) const;

    float get_distance(const vec3f& p, int k = 3)
        const;  // 计算点到最近四面体的距离，负数表示在四面体内部
};

}  // namespace sinriv::kigstudio::mesh::conebvh
