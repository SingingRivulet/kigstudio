#pragma once
#include <math.h>
namespace sinriv::kigstudio {
    template<typename T> struct vec3 {
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
        friend std::ostream& operator<<(std::ostream& os, const vec3& v) {
            os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
            return os;
        }
        inline bool operator==(const vec3<T>& i)const {
            if (x != i.x)return false;
            if (y != i.y)return false;
            if (z != i.z)return false;
            return true;
        }
        inline void operator()(T xt, T yt, T zt) {
            init(xt, yt, zt);
        }
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
        inline vec3<T> operator+(const vec3<T>& p)const {
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
        inline vec3<T> operator-(const vec3<T>& p)const {
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
        inline vec3<T> operator*(const T& p)const {
            return vec3<T>(p * x, p * y, p * z);
        }
        inline vec3<T> operator/(const T& p)const {
            return vec3<T>(x / p, y / p, z / p);
        }
        inline vec3<T> operator-()const {
            return vec3<T>(-x, -y, -z);
        }
        inline vec3<T> cross(const vec3<T>& i)const {
            return vec3<T>(
                x * i.y - y * i.x,
                y * i.z - z * i.x,
                z * i.x - x * i.z
            );
        }
        inline T length()const {
            return sqrt((x * x) + (y * y) + (z * z));
        }
        inline vec3<T> normalize()const {
            return (*this) * invnorm();
        }
        inline T invnorm()const {
            return 1 / sqrt((x * x) + (y * y) + (z * z));
        }
        inline T dist2(const vec3<T>& p)const {
            auto t = p - (*this);
            return ((t.x * t.x) + (t.y * t.y) + (t.z * t.z));
        }
        inline T dist(const vec3<T>& p)const {
            return sqrt(dist(p));
        }
        inline T dot(const vec3<T>* p)const {
            return (
                (x * p->x) +
                (y * p->y) +
                (z * p->z)
                );
        }
        inline T dot(const vec3<T>& p)const {
            return (
                (x * p.x) +
                (y * p.y) +
                (z * p.z)
                );
        }
        inline void GeoHash(T length, char* str, int l)const {
            vec3<T> v;
            GeoHash(length, str, 0, l, (&v));
        }
        inline void GeoHashBin(T length, double* str, int l)const {
            vec3<T> v;
            GeoHashBin(length, str, 0, l, (&v));
        }
#define __RET if(begin==end){return;}
        inline void GeoHashBin(T length, double* str, int begin, int end, vec3<T>* origin)const {
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
                    }
                    else {

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
                }
                else {
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
                    }
                    else {

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
            }
            else {
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
                    }
                    else {

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
                }
                else {
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
                    }
                    else {

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
#undef  __RET
        void GeoHash(T length, char* str, int begin, int end, vec3<T>* origin)const {
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
                    }
                    else {
                        str[begin] = 'b';
                        origin->x += length;
                        origin->y += length;
                        origin->z -= length;
                        GeoHash(length * 0.5f, str, begin + 1, end, origin);
                    }
                }
                else {
                    if (z > origin->z) {
                        str[begin] = 'c';
                        origin->x += length;
                        origin->y -= length;
                        origin->z += length;
                        GeoHash(length * 0.5f, str, begin + 1, end, origin);
                    }
                    else {
                        str[begin] = 'd';
                        origin->x += length;
                        origin->y -= length;
                        origin->z -= length;
                        GeoHash(length * 0.5f, str, begin + 1, end, origin);
                    }
                }
            }
            else {
                if (y > origin->y) {
                    if (z > origin->z) {
                        str[begin] = 'e';
                        origin->x -= length;
                        origin->y += length;
                        origin->z += length;
                        GeoHash(length * 0.5f, str, begin + 1, end, origin);
                    }
                    else {
                        str[begin] = 'f';
                        origin->x -= length;
                        origin->y += length;
                        origin->z -= length;
                        GeoHash(length * 0.5f, str, begin + 1, end, origin);
                    }
                }
                else {
                    if (z > origin->z) {
                        str[begin] = 'g';
                        origin->x -= length;
                        origin->y -= length;
                        origin->z += length;
                        GeoHash(length * 0.5f, str, begin + 1, end, origin);
                    }
                    else {
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
}