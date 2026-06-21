#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <variant>
#include <memory>
#include <vector>
#include "kigstudio/utils/mat.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/utils/vec2.h"

namespace sinriv::kigstudio::voxel::concave {

using vec3f = sinriv::kigstudio::vec3<float>;
using vec2f = sinriv::kigstudio::vec2<float>;
using mat4f = sinriv::kigstudio::mat::matrix<float>;
using vec4f = sinriv::kigstudio::mat::vec4<float>;

class Base {
    // 凹体基类
   public:
    virtual void triangulate() = 0;
    virtual bool check(std::string& error_message) const = 0;
    virtual ~Base() = default;
    virtual bool contains(const vec3f& p) const = 0;

    std::vector<PosNormalVertex> vertices;
    std::vector<uint32_t> indices;
};

class Cone : public Base {
    // 凹棱锥
    // 顶点和底面顶点路径（环形）定义了一个锥体，底面顶点路径必须按照顺序排列
    // 底面顶点路径必须至少包含3个顶点，且不能有重复顶点
    // 底面顶点路径必须在顶点的同一侧，且不能穿过顶点
    // 底面顶点路径不一定在同一个平面内，但必须是简单的（不能有自交）
    // 锥体棱长视为无限长（三角形化时设为一个很大的值），base_vertices是棱上的点，而不是棱的端点
   public:
    vec3f apex;                        // 顶点
    std::vector<vec3f> base_vertices;  // 底面顶点路径（环形）
    std::vector<std::array<uint32_t, 3>> base_triangles; // 底面三角形索引（由triangulate生成）
    void triangulate() override;       // 构建三角形
    bool
    check(  // 检测是否有效(锥体夹角必须小于180度，从顶点方向看，路径不能有交叉)
        std::string& error_message)
        const override;  // 返回是否有效，并输出错误信息
    bool contains(const vec3f& p) const override;  // 检测点是否在锥体内
};

inline cJSON* to_json(const Cone& c) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddItemToObject(obj, "apex", to_json(c.apex));
    cJSON* arr = cJSON_CreateArray();
    for (const auto& v : c.base_vertices) {
        cJSON_AddItemToArray(arr, to_json(v));
    }
    cJSON_AddItemToObject(obj, "base_vertices", arr);
    return obj;
}

inline Cone from_json_cone(const cJSON* obj) {
    Cone c;
    if (!cJSON_IsObject(obj)) {
        c.triangulate();
        return c;
    }

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, obj) {
        if (!child->string)
            continue;

        if (cJSON_IsObject(child) && strcmp(child->string, "apex") == 0) {
            c.apex = vec3_from_json<vec3f>(child);
        } else if (cJSON_IsArray(child) &&
                   strcmp(child->string, "base_vertices") == 0) {
            int count = cJSON_GetArraySize(child);
            for (int i = 0; i < count; ++i) {
                const cJSON* v = cJSON_GetArrayItem(child, i);
                if (v && cJSON_IsObject(v))
                    c.base_vertices.push_back(vec3_from_json<vec3f>(v));
            }
        }
    }
    c.triangulate();
    return c;
}

}  // namespace sinriv::kigstudio::voxel::concave

namespace sinriv::kigstudio::sdf {
    struct SDFBase;
    std::shared_ptr<SDFBase> to_sdf(
        const sinriv::kigstudio::voxel::concave::Cone& cone);
}