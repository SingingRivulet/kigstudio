#pragma once
#include <math.h>
#include <iostream>
#include "vec3.h"

namespace sinriv::kigstudio::mat {

template <typename T>
class matrix {
   public:
    // 构造函数，初始化矩阵
    matrix() {
        // 遍历矩阵的每一个元素
        for (int i = 0; i < 16; i++) {
            data[i] = 0;
        }
    }

    matrix(const T* values) {
        for (int i = 0; i < 16; i++) {
            data[i] = values[i];
        }
    }

    T* operator[](int index) { return &data[index * 4]; }

    const T* operator[](int index) const { return &data[index * 4]; }

    matrix<T> operator*(const matrix<T>& other) const {
        matrix<T> result;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                result[i][j] = 0;
                for (int k = 0; k < 4; k++) {
                    result[i][j] += data[i * 4 + k] * other[k][j];
                }
            }
        }
        return result;
    }

    void operator*=(const matrix<T>& other) { *this = *this * other; }

    void setIdentity() {
        for (int i = 0; i < 16; i++) {
            data[i] = 0;
        }
        for (int i = 0; i < 4; i++) {
            data[i * 4 + i] = 1;
        }
    }

    void setZero() {
        for (int i = 0; i < 16; i++) {
            data[i] = 0;
        }
    }

    void setTranslation(const T x, const T y, const T z) {
        setIdentity();
        data[12] = x;
        data[13] = y;
        data[14] = z;
    }

    void setRotation(const T angle, const T x, const T y, const T z) {
        setZero();
        const T c = cos(angle);
        const T s = sin(angle);
        const T one_minus_c = 1 - c;
        data[0] = x * x * one_minus_c + c;
        data[1] = y * x * one_minus_c + z * s;
        data[2] = z * x * one_minus_c - y * s;
        data[4] = x * y * one_minus_c - z * s;
        data[5] = y * y * one_minus_c + c;
        data[6] = z * y * one_minus_c + x * s;
        data[8] = x * z * one_minus_c + y * s;
        data[9] = y * z * one_minus_c - x * s;
        data[10] = z * z * one_minus_c + c;
    }

    void setScale(const T x, const T y, const T z) {
        setZero();
        data[0] = x;
        data[5] = y;
        data[10] = z;
    }

    void setPerspective(const T fov,
                        const T aspect,
                        const T near,
                        const T far) {
        setZero();
        const T tan_half_fov = tan(fov / 2);
        data[0] = 1 / (aspect * tan_half_fov);
        data[5] = 1 / tan_half_fov;
        data[10] = -(far + near) / (far - near);
        data[11] = -1;
        data[14] = -2 * far * near / (far - near);
    }

    void setOrthographic(const T left,
                         const T right,
                         const T bottom,
                         const T top,
                         const T near,
                         const T far) {
        setZero();
        data[0] = 2 / (right - left);
        data[5] = 2 / (top - bottom);
        data[10] = -2 / (far - near);
        data[12] = -(right + left) / (right - left);
        data[13] = -(top + bottom) / (top - bottom);
        data[14] = -(far + near) / (far - near);
    }

    void setLookAt(const T eyeX,
                   const T eyeY,
                   const T eyeZ,
                   const T centerX,
                   const T centerY,
                   const T centerZ,
                   const T upX,
                   const T upY,
                   const T upZ) {
        setZero();
        const T fx = centerX - eyeX;
        const T fy = centerY - eyeY;
        const T fz = centerZ - eyeZ;
        const T rlf = 1 / sqrt(fx * fx + fy * fy + fz * fz);
        fx *= rlf;
        fy *= rlf;
        fz *= rlf;
        const T sx = fy * upZ - fz * upY;
        const T sy = fz * upX - fx * upZ;
        const T sz = fx * upY - fy * upX;
        const T rls = 1 / sqrt(sx * sx + sy * sy + sz * sz);
        sx *= rls;
        sy *= rls;
        sz *= rls;
        const T ux = sy * fz - sz * fy;
        const T uy = sz * fx - sx * fz;
        const T uz = sx * fy - sy * fx;
        data[0] = sx;
        data[1] = sy;
        data[2] = sz;
        data[4] = ux;
        data[5] = uy;
        data[6] = uz;
        data[8] = -fx;
        data[9] = -fy;
        data[10] = -fz;
        data[12] = -sx * eyeX - sy * eyeY - sz * eyeZ;
        data[13] = -ux * eyeX - uy * eyeY - uz * eyeZ;
        data[14] = fx * eyeX + fy * eyeY + fz * eyeZ;
    }

    void transpose() {
        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                std::swap(data[i * 4 + j], data[j * 4 + i]);
            }
        }
    }

    void invert() {
        const T a00 = data[0], a01 = data[1], a02 = data[2], a03 = data[3],
                a10 = data[4], a11 = data[5], a12 = data[6], a13 = data[7],
                a20 = data[8], a21 = data[9], a22 = data[10], a23 = data[11],
                a30 = data[12], a31 = data[13], a32 = data[14], a33 = data[15],

                b00 = a00 * a11 - a01 * a10, b01 = a00 * a12 - a02 * a10,
                b02 = a00 * a13 - a03 * a10, b03 = a01 * a12 - a02 * a11,
                b04 = a01 * a13 - a03 * a11, b05 = a02 * a13 - a03 * a12,
                b06 = a20 * a31 - a21 * a30, b07 = a20 * a32 - a22 * a30,
                b08 = a20 * a33 - a23 * a30, b09 = a21 * a32 - a22 * a31,
                b10 = a21 * a33 - a23 * a31, b11 = a22 * a33 - a23 * a32,

                det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 -
                      b04 * b07 + b05 * b06;

        if (det == 0) {
            setIdentity();
            return;
        }

        const T invDet = 1 / det;

        data[0] = (a11 * b11 - a12 * b10 + a13 * b09) * invDet;
        data[1] = (a02 * b10 - a01 * b11 - a03 * b09) * invDet;
        data[2] = (a31 * b05 - a32 * b04 + a33 * b03) * invDet;
        data[3] = (a22 * b04 - a21 * b05 - a23 * b03) * invDet;
        data[4] = (a12 * b08 - a10 * b11 - a13 * b07) * invDet;
        data[5] = (a00 * b11 - a02 * b08 + a03 * b07) * invDet;
        data[6] = (a32 * b02 - a30 * b05 - a33 * b01) * invDet;
        data[7] = (a20 * b05 - a22 * b02 + a23 * b01) * invDet;
        data[8] = (a10 * b10 - a11 * b08 + a13 * b06) * invDet;
        data[9] = (a01 * b08 - a00 * b10 - a03 * b06) * invDet;
        data[10] = (a30 * b04 - a31 * b02 + a33 * b00) * invDet;
        data[11] = (a21 * b02 - a20 * b04 - a23 * b00) * invDet;
        data[12] = (a11 * b07 - a10 * b09 - a12 * b06) * invDet;
        data[13] = (a00 * b09 - a01 * b07 + a02 * b06) * invDet;
        data[14] = (a31 * b01 - a30 * b03 - a32 * b00) * invDet;
        data[15] = (a20 * b03 - a21 * b01 + a22 * b00) * invDet;
    }

    void print() const {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                std::cout << data[i * 4 + j] << " ";
            }
            std::cout << std::endl;
        }
    }

   private:
    T data[16];
};

template <typename T>
class vec4 {
   public:
    vec4() {
        for (int i = 0; i < 4; i++) {
            data[i] = 0;
        }
    }

    vec4(const T x, const T y, const T z, const T w = 1) {
        data[0] = x;
        data[1] = y;
        data[2] = z;
        data[3] = w;
    }

    vec3<T> toVec3() const { 
        return vec3<T>(data[0]/data[3], data[1]/data[3], data[2]/data[3]); 
    }

    T& operator[](int index) { return data[index]; }

    const T& operator[](int index) const { return data[index]; }

    vec4<T> operator*(const matrix<T>& mat) const {
        vec4<T> result;
        for (int i = 0; i < 4; i++) {
            result[i] = 0;
            for (int j = 0; j < 4; j++) {
                result[i] += data[j] * mat[j][i];
            }
        }
        return result;
    }

    void operator*=(const matrix<T>& mat) { *this = *this * mat; }

    void print() const {
        for (int i = 0; i < 4; i++) {
            std::cout << data[i] << " ";
        }
        std::cout << std::endl;
    }

   private:
    T data[4];
};

}  // namespace sinriv::kigstudio::mat