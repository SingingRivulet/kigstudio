#pragma once
#include <cJSON.h>
#include <math.h>
#include <concepts>
#include <iostream>

namespace sinriv::kigstudio {
template <typename T>
concept Numeric = std::integral<T> || std::floating_point<T>;

template <typename T>
concept Vec3_c = requires(T v) {
    ((float)v.x);
    ((float)v.y);
    ((float)v.z);
};

template <Vec3_c T>
cJSON* to_json(const T& v) {
    cJSON* json = cJSON_CreateObject();
    if (!json)
        return nullptr;

    cJSON_AddNumberToObject(json, "x", v.x);
    cJSON_AddNumberToObject(json, "y", v.y);
    cJSON_AddNumberToObject(json, "z", v.z);

    return json;
}

// 反序列化函数：从cJSON对象创建Vec3_c类型
template <Vec3_c T>
T vec3_from_json(const cJSON* json) {
    T result;

    cJSON* x_item = cJSON_GetObjectItemCaseSensitive(json, "x");
    cJSON* y_item = cJSON_GetObjectItemCaseSensitive(json, "y");
    cJSON* z_item = cJSON_GetObjectItemCaseSensitive(json, "z");

    if (x_item)
        result.x = static_cast<decltype(result.x)>(x_item->valuedouble);
    if (y_item)
        result.y = static_cast<decltype(result.y)>(y_item->valuedouble);
    if (z_item)
        result.z = static_cast<decltype(result.z)>(z_item->valuedouble);

    return result;
}

template <Numeric T>
struct vec3 {
   public:
    T x;
    T y;
    T z;
    inline vec3() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }
    inline vec3(const vec3<T>& p) {
        x = p.x;
        y = p.y;
        z = p.z;
    }
    inline vec3(T xt, T yt, T zt) {
        x = xt;
        y = yt;
        z = zt;
    }
    inline void init(T xt, T yt, T zt) {
        x = xt;
        y = yt;
        z = zt;
    }
    inline friend std::ostream& operator<<(std::ostream& os, const vec3& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
    inline bool operator==(const vec3<T>& i) const {
        if (x != i.x)
            return false;
        if (y != i.y)
            return false;
        if (z != i.z)
            return false;
        return true;
    }
    inline void operator()(T xt, T yt, T zt) { init(xt, yt, zt); }
    inline vec3<T>& operator=(const vec3<T>* p) {
        x = p->x;
        y = p->y;
        z = p->z;
        return *this;
    }
    inline vec3<T>& operator=(const vec3<T>& p) {
        x = p.x;
        y = p.y;
        z = p.z;
        return *this;
    }
    inline vec3<T> operator+(const vec3<T>& p) const {
        vec3<T> b;
        b = this;
        b.x += p.x;
        b.y += p.y;
        b.z += p.z;
        return b;
    }
    inline vec3<T>& operator+=(const vec3<T>& p) {
        x += p.x;
        y += p.y;
        z += p.z;
        return *this;
    }
    inline vec3<T>& operator-=(const vec3<T>& p) {
        x -= p.x;
        y -= p.y;
        z -= p.z;
        return *this;
    }
    inline vec3<T> operator-(const vec3<T>& p) const {
        vec3<T> b;
        b = this;
        b.x -= p.x;
        b.y -= p.y;
        b.z -= p.z;
        return b;
    }
    inline vec3<T>& operator*=(T p) {
        x *= p;
        y *= p;
        z *= p;
        return *this;
    }
    inline vec3<T>& operator/=(T p) {
        x /= p;
        y /= p;
        z /= p;
        return *this;
    }
    inline vec3<T> operator*(const T& p) const {
        return vec3<T>(p * x, p * y, p * z);
    }
    inline vec3<T> operator*(const vec3<T>& p) const {
        return vec3<T>(x * p.x, y * p.y, z * p.z);
    }
    inline vec3<T> operator/(const T& p) const {
        return vec3<T>(x / p, y / p, z / p);
    }
    inline vec3<T> operator/(const vec3<T>& p) const {
        return vec3<T>(x / p.x, y / p.y, z / p.z);
    }
    inline vec3<T> operator-() const { return vec3<T>(-x, -y, -z); }
    inline vec3<T> cross(const vec3<T>& o) const {
        return vec3(y * o.z - z * o.y,  // x 分量
                    z * o.x - x * o.z,  // y 分量 (注意这里是减号)
                    x * o.y - y * o.x   // z 分量
        );
    }
    inline T length() const { return sqrt((x * x) + (y * y) + (z * z)); }
    inline T L1() const { return abs(x) + abs(y) + abs(z); }
    inline vec3<T> normalize() const { return (*this) * invnorm(); }
    inline T invnorm() const { return 1 / sqrt((x * x) + (y * y) + (z * z)); }
    inline T dist2(const vec3<T>& p) const {
        auto t = p - (*this);
        return ((t.x * t.x) + (t.y * t.y) + (t.z * t.z));
    }
    inline T dist(const vec3<T>& p) const { return sqrt(dist(p)); }
    inline T dot(const vec3<T>* p) const {
        return ((x * p->x) + (y * p->y) + (z * p->z));
    }
    inline T dot(const vec3<T>& p) const {
        return ((x * p.x) + (y * p.y) + (z * p.z));
    }
    inline void GeoHash(T length, char* str, int l) const {
        vec3<T> v;
        GeoHash(length, str, 0, l, (&v));
    }
    inline void GeoHashBin(T length, double* str, int l) const {
        vec3<T> v;
        GeoHashBin(length, str, 0, l, (&v));
    }
#define __RET           \
    if (begin == end) { \
        return;         \
    }
    inline void GeoHashBin(T length,
                           double* str,
                           int begin,
                           int end,
                           vec3<T>* origin) const {
        __RET;
        if (x > origin->x) {
            if (y > origin->y) {
                if (z > origin->z) {
                    str[begin] = 1;
                    begin++;
                    __RET
                    str[begin] = 1;
                    begin++;
                    __RET
                    str[begin] = 1;

                    origin->x += length;
                    origin->y += length;
                    origin->z += length;
                    GeoHashBin(length * 0.5f, str, begin + 1, end, origin);
                } else {
                    str[begin] = 1;
                    begin++;
                    __RET
                    str[begin] = 1;
                    begin++;
                    __RET
                    str[begin] = 0;

                    origin->x += length;
                    origin->y += length;
                    origin->z -= length;
                    GeoHashBin(length * 0.5f, str, begin + 1, end, origin);
                }
            } else {
                if (z > origin->z) {
                    str[begin] = 1;
                    begin++;
                    __RET
                    str[begin] = 0;
                    begin++;
                    __RET
                    str[begin] = 1;

                    origin->x += length;
                    origin->y -= length;
                    origin->z += length;
                    GeoHashBin(length * 0.5f, str, begin + 1, end, origin);
                } else {
                    str[begin] = 1;
                    begin++;
                    __RET
                    str[begin] = 0;
                    begin++;
                    __RET
                    str[begin] = 0;

                    origin->x += length;
                    origin->y -= length;
                    origin->z -= length;
                    GeoHashBin(length * 0.5f, str, begin + 1, end, origin);
                }
            }
        } else {
            if (y > origin->y) {
                if (z > origin->z) {
                    str[begin] = 0;
                    begin++;
                    __RET
                    str[begin] = 1;
                    begin++;
                    __RET
                    str[begin] = 1;

                    origin->x -= length;
                    origin->y += length;
                    origin->z += length;
                    GeoHashBin(length * 0.5f, str, begin + 1, end, origin);
                } else {
                    str[begin] = 0;
                    begin++;
                    __RET
                    str[begin] = 1;
                    begin++;
                    __RET
                    str[begin] = 0;

                    origin->x -= length;
                    origin->y += length;
                    origin->z -= length;
                    GeoHashBin(length * 0.5f, str, begin + 1, end, origin);
                }
            } else {
                if (z > origin->z) {
                    str[begin] = 0;
                    begin++;
                    __RET
                    str[begin] = 0;
                    begin++;
                    __RET
                    str[begin] = 1;

                    origin->x -= length;
                    origin->y -= length;
                    origin->z += length;
                    GeoHashBin(length * 0.5f, str, begin + 1, end, origin);
                } else {
                    str[begin] = 0;
                    begin++;
                    __RET
                    str[begin] = 0;
                    begin++;
                    __RET
                    str[begin] = 0;

                    origin->x -= length;
                    origin->y -= length;
                    origin->z -= length;
                    GeoHashBin(length * 0.5f, str, begin + 1, end, origin);
                }
            }
        }
    }
#undef __RET
    void GeoHash(T length,
                 char* str,
                 int begin,
                 int end,
                 vec3<T>* origin) const {
        if (begin == end) {
            str[begin] = '\n';
            return;
        }
        if (x > origin->x) {
            if (y > origin->y) {
                if (z > origin->z) {
                    str[begin] = 'a';
                    origin->x += length;
                    origin->y += length;
                    origin->z += length;
                    GeoHash(length * 0.5f, str, begin + 1, end, origin);
                } else {
                    str[begin] = 'b';
                    origin->x += length;
                    origin->y += length;
                    origin->z -= length;
                    GeoHash(length * 0.5f, str, begin + 1, end, origin);
                }
            } else {
                if (z > origin->z) {
                    str[begin] = 'c';
                    origin->x += length;
                    origin->y -= length;
                    origin->z += length;
                    GeoHash(length * 0.5f, str, begin + 1, end, origin);
                } else {
                    str[begin] = 'd';
                    origin->x += length;
                    origin->y -= length;
                    origin->z -= length;
                    GeoHash(length * 0.5f, str, begin + 1, end, origin);
                }
            }
        } else {
            if (y > origin->y) {
                if (z > origin->z) {
                    str[begin] = 'e';
                    origin->x -= length;
                    origin->y += length;
                    origin->z += length;
                    GeoHash(length * 0.5f, str, begin + 1, end, origin);
                } else {
                    str[begin] = 'f';
                    origin->x -= length;
                    origin->y += length;
                    origin->z -= length;
                    GeoHash(length * 0.5f, str, begin + 1, end, origin);
                }
            } else {
                if (z > origin->z) {
                    str[begin] = 'g';
                    origin->x -= length;
                    origin->y -= length;
                    origin->z += length;
                    GeoHash(length * 0.5f, str, begin + 1, end, origin);
                } else {
                    str[begin] = 'h';
                    origin->x -= length;
                    origin->y -= length;
                    origin->z -= length;
                    GeoHash(length * 0.5f, str, begin + 1, end, origin);
                }
            }
        }
    }
    inline bool GeoHashDecode(T length, const char* s) {
        const char* str = s;
        T ll = length;
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        while (*str) {
            switch (*str) {
                case ('a'):
                    x += ll;
                    y += ll;
                    z += ll;
                    break;
                case ('b'):
                    x += ll;
                    y += ll;
                    z -= ll;
                    break;
                case ('c'):
                    x += ll;
                    y -= ll;
                    z += ll;
                    break;
                case ('d'):
                    x += ll;
                    y -= ll;
                    z -= ll;
                    break;
                case ('e'):
                    x -= ll;
                    y += ll;
                    z += ll;
                    break;
                case ('f'):
                    x -= ll;
                    y += ll;
                    z -= ll;
                    break;
                case ('g'):
                    x -= ll;
                    y -= ll;
                    z += ll;
                    break;
                case ('h'):
                    x -= ll;
                    y -= ll;
                    z -= ll;
                    break;
                default:
                    return false;
                    break;
            }
            ll = ll * 0.5f;
            str++;
        }
        return true;
    }
};

struct Vec3i {
    int32_t x, y, z;

    inline Vec3i(int x = 0, int y = 0, int z = 0) : x(x), y(y), z(z) {}

    // 比较两个向量是否相等
    inline bool operator==(const Vec3i& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    inline bool operator!=(const Vec3i& other) const {
        return !(*this == other);
    }

    inline bool operator<(const Vec3i& other) const {
        return x < other.x || (x == other.x && y < other.y) ||
               (x == other.x && y == other.y && z < other.z);
    }
    inline bool operator<=(const Vec3i& other) const {
        return *this < other || *this == other;
    }
    inline bool operator>(const Vec3i& other) const {
        return !(*this <= other);
    }
    inline bool operator>=(const Vec3i& other) const {
        return !(*this < other);
    }
    inline Vec3i operator+(const Vec3i& other) const {
        return Vec3i(x + other.x, y + other.y, z + other.z);
    }
    inline Vec3i operator-(const Vec3i& other) const {
        return Vec3i(x - other.x, y - other.y, z - other.z);
    }
    inline Vec3i operator*(int scalar) const {
        return Vec3i(x * scalar, y * scalar, z * scalar);
    }
    inline Vec3i operator/(int scalar) const {
        return Vec3i(x / scalar, y / scalar, z / scalar);
    }
    inline Vec3i& operator+=(const Vec3i& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    inline Vec3i& operator-=(const Vec3i& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }
    inline Vec3i& operator*=(int scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }
    inline Vec3i& operator/=(int scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }
    inline friend std::ostream& operator<<(std::ostream& os, const Vec3i& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
};

template <Numeric T>
struct ray {
    vec3<T> begin, end;

    ray() : begin(vec3<T>(0, 0, 0)), end(vec3<T>(0, 0, 0)) {}
    ray(const vec3<T>& b, const vec3<T>& e) : begin(b), end(e) {}

    vec3<T> direction() const { return (end - begin).normalize(); }
    vec3<T> getMiddle() const { return (end + begin) * 0.5f; }
    float getLength() const { return (end - begin).length(); }

    inline friend std::ostream& operator<<(std::ostream& os, const ray& v) {
        os << v.begin << " -> " << v.end;
        return os;
    }
};

struct PosNormalVertex {
    vec3<float> pos;
    vec3<float> normal;
    inline PosNormalVertex() : pos(0, 0, 0), normal(0, 0, 0) {}
    inline PosNormalVertex(const vec3<float>& p, const vec3<float>& n)
        : pos(p), normal(n) {}
    inline PosNormalVertex(const vec3<float>& p) : pos(p), normal(0, 0, 0) {}
    inline PosNormalVertex(const PosNormalVertex& v)
        : pos(v.pos), normal(v.normal) {}
    inline PosNormalVertex(float x, float y, float z)
        : pos(x, y, z), normal(0, 0, 0) {}
    inline PosNormalVertex(float x,
                           float y,
                           float z,
                           float nx,
                           float ny,
                           float nz)
        : pos(x, y, z), normal(nx, ny, nz) {}
};

template <typename T>
inline auto dot(const T& a, const T& b) {
    return a.dot(b);
}
template <typename T>
inline auto cross(const T& a, const T& b) {
    return a.cross(b);
}
template <typename T>
inline auto length(const T& v) {
    return v.length();
}
template <typename T>
inline auto normalize(const T& v) {
    return v.normalize();
}
}  // namespace sinriv::kigstudio