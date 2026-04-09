#pragma once
#include <cmath>
#include <iostream>
#include "vec3.h"

namespace sinriv::kigstudio {

template <Numeric T>
class Plane {
   private:
    // 平面参数：Ax + By + Cz + D = 0
    T A, B, C, D;

    // 原始数据（可选，用于调试或重新计算）
    vec3<T> m_point;
    vec3<T> m_normal;

   public:
    // 构造函数：使用点法式初始化平面
    inline Plane(const vec3<T>& point, const vec3<T>& normal)
        : m_point(point), m_normal(normal) {
        // 1. 法向量即为 A, B, C
        A = normal.x;
        B = normal.y;
        C = normal.z;

        // 2. 预先计算 D
        // 公式：Ax0 + By0 + Cz0 + D = 0  =>  D = -(Ax0 + By0 + Cz0)
        D = -(A * point.x + B * point.y + C * point.z);
    }

    //  判断点在平面的哪一侧
    inline bool getSide(const vec3<T>& point) const {
        // 计算 Ax + By + Cz + D
        // 这一步利用了预先计算好的 ABCD，避免了重复的减法运算
        T result = A * point.x + B * point.y + C * point.z + D;

        return result > 0;
    }

    // 辅助函数：获取计算出的 D 值（用于验证）
    inline T getD() const { return D; }

    // 辅助函数：打印平面方程
    inline void printEquation() const {
        std::cout << "平面方程: " << A << "x + " << B << "y + " << C << "z + "
                  << D << " = 0" << std::endl;
    }
};
}  // namespace sinriv::kigstudio
