#pragma once
#include <cmath>
#include <iostream>
#include "vec3.h"

namespace sinriv::kigstudio {

template <Numeric T>
class Plane {
   public:
    // 平面参数：Ax + By + Cz + D = 0
    T A, B, C, D;

    // 原始数据（可选，用于调试或重新计算）
    vec3<T> m_point;
    vec3<T> m_normal;
    // 构造函数：使用点法式初始化平面
    inline Plane(): A(0), B(0), C(0), D(0){}
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
    
    // 构造函数：使用三点式初始化平面
    inline Plane(const vec3<T>& p1, const vec3<T>& p2, const vec3<T>& p3) {
        // 计算两个向量
        vec3<T> v1 = p2 - p1;
        vec3<T> v2 = p3 - p1;

        // 计算法向量（叉积）
        m_normal = cross(v1, v2);

        // 确保法向量不为零（三点不共线）
        if (length(m_normal) < std::numeric_limits<T>::epsilon()) {
            throw std::invalid_argument("三点共线，无法确定平面");
        }

        // 归一化法向量
        m_normal = normalize(m_normal);

        // 设置点为第一个点
        m_point = p1;

        // 计算平面参数
        A = m_normal.x;
        B = m_normal.y;
        C = m_normal.z;
        D = -(A * m_point.x + B * m_point.y + C * m_point.z);
    }

    // 构造函数：直接使用平面方程参数初始化
    inline Plane(T a, T b, T c, T d) : A(a), B(b), C(c), D(d) {
        // 计算法向量
        m_normal = vec3<T>(A, B, C);

        // 归一化法向量
        T len = length(m_normal);
        if (len < std::numeric_limits<T>::epsilon()) {
            throw std::invalid_argument("法向量不能为零向量");
        }
        m_normal = m_normal / len;

        // 计算平面上的一个点
        // 选择一个非零分量来计算点坐标
        if (std::abs(A) > std::numeric_limits<T>::epsilon()) {
            m_point = vec3<T>(-D / A, 0, 0);
        } else if (std::abs(B) > std::numeric_limits<T>::epsilon()) {
            m_point = vec3<T>(0, -D / B, 0);
        } else {
            m_point = vec3<T>(0, 0, -D / C);
        }

        // 更新归一化后的ABCD
        A = m_normal.x;
        B = m_normal.y;
        C = m_normal.z;
        D = -(A * m_point.x + B * m_point.y + C * m_point.z);
    }

    // 获取平面的点法式表示
    inline std::pair<vec3<T>, vec3<T>> getPointNormalForm() const {
        return std::make_pair(m_point, m_normal);
    }

    // 获取平面的三点式表示
    inline std::tuple<vec3<T>, vec3<T>, vec3<T>> getThreePointForm() const {
        // 使用点法式计算平面上的三个点
        vec3<T> p1 = m_point;

        // 找到两个与法向量垂直的向量
        vec3<T> u;
        if (std::abs(m_normal.x) < std::numeric_limits<T>::epsilon() &&
            std::abs(m_normal.y) < std::numeric_limits<T>::epsilon()) {
            // 法向量接近z轴方向，使用x轴作为第一个方向
            u = vec3<T>(1, 0, 0);
        } else {
            // 使用z轴与法向量的叉积作为第一个方向
            u = normalize(cross(vec3<T>(0, 0, 1), m_normal));
        }

        // 第二个方向向量与法向量和第一个方向向量都垂直
        vec3<T> v = normalize(cross(m_normal, u));

        // 计算另外两个点
        vec3<T> p2 = p1 + u;
        vec3<T> p3 = p1 + v;

        return std::make_tuple(p1, p2, p3);
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
