#include "concave.h"
namespace sinriv::kigstudio::voxel::concave {

inline std::vector<vec3f> computeDirections(const vec3f& vertex,
                                            const std::vector<vec3f>& base) {
    std::vector<vec3f> dirs;
    dirs.reserve(base.size());

    for (auto& v : base)
        dirs.push_back(normalize(v - vertex));

    return dirs;
}

inline vec3f computeConeDir(const std::vector<vec3f>& dirs) {
    vec3f dir{0, 0, 0};

    for (auto& d : dirs)
        dir += d;

    if (length(dir) < 1e-5f) {
        // fallback：边法线累加
        for (int i = 0; i < dirs.size(); ++i) {
            vec3f d0 = dirs[i];
            vec3f d1 = dirs[(i + 1) % dirs.size()];
            dir += cross(d0, d1);
        }
    }

    if (length(dir) < 1e-6f)
        return dirs[0];

    return normalize(dir);
}

inline void buildBasis(const vec3f& dir, vec3f& right, vec3f& up) {
    vec3f tmp = fabs(dir.x) < 0.9f ? vec3f{1, 0, 0} : vec3f{0, 1, 0};
    right = normalize(cross(tmp, dir));
    up = cross(dir, right);
}

inline std::vector<vec2f> projectDirs(const std::vector<vec3f>& dirs,
                                      const vec3f& right,
                                      const vec3f& up) {
    std::vector<vec2f> proj(dirs.size());

    for (int i = 0; i < dirs.size(); ++i) {
        proj[i] = {dot(dirs[i], right), dot(dirs[i], up)};
    }
    return proj;
}

inline bool checkDuplicate(const std::vector<vec3f>& base) {
    const float eps = 1e-6f;
    for (int i = 0; i < base.size(); ++i)
        for (int j = i + 1; j < base.size(); ++j)
            if (length(base[i] - base[j]) < eps)
                return false;
    return true;
}

inline bool inSameHemisphere(const std::vector<vec3f>& dirs) {
    for (int i = 0; i < dirs.size(); ++i)
        for (int j = i + 1; j < dirs.size(); ++j)
            if (dot(dirs[i], dirs[j]) <= -1.0f + 1e-4f)
                return false;
    return true;
}
inline float cross2(vec2f a, vec2f b, vec2f c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

inline bool intersect2D(vec2f a1, vec2f a2, vec2f b1, vec2f b2) {
    float d1 = cross2(a1, a2, b1);
    float d2 = cross2(a1, a2, b2);
    float d3 = cross2(b1, b2, a1);
    float d4 = cross2(b1, b2, a2);

    return (d1 * d2 < 0 && d3 * d4 < 0);
}
inline bool checkSelfIntersect(const std::vector<vec2f>& poly) {
    int n = poly.size();

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (abs(i - j) <= 1)
                continue;
            if (i == 0 && j == n - 1)
                continue;

            if (intersect2D(poly[i], poly[(i + 1) % n], poly[j],
                            poly[(j + 1) % n]))
                return false;
        }
    }
    return true;
}
inline bool checkAngleMonotonic(const std::vector<vec2f>& proj) {
    const float PI = 3.1415926f;
    const float eps = 1e-6f;

    int sign = 0;

    for (int i = 0; i < proj.size(); ++i) {
        float a0 = atan2(proj[i].y, proj[i].x);
        float a1 =
            atan2(proj[(i + 1) % proj.size()].y, proj[(i + 1) % proj.size()].x);

        float d = a1 - a0;
        if (d > PI)
            d -= 2 * PI;
        if (d < -PI)
            d += 2 * PI;

        if (fabs(d) < eps)
            continue;

        int s = d > 0 ? 1 : -1;

        if (sign == 0)
            sign = s;
        else if (sign != s)
            return false;
    }

    return true;
}

bool Cone::check(std::string& err) const {
    if (base_vertices.size() < 3) {
        err = "too few vertices";
        return false;
    }

    if (!checkDuplicate(base_vertices)) {
        err = "duplicate vertices";
        return false;
    }

    auto dirs = computeDirections(apex, base_vertices);

    if (!inSameHemisphere(dirs)) {
        err = "span >= 180 deg";
        return false;
    }

    vec3f dir = computeConeDir(dirs);

    vec3f right, up;
    buildBasis(dir, right, up);

    auto proj = projectDirs(dirs, right, up);

    if (!checkSelfIntersect(proj)) {
        err = "self intersection";
        return false;
    }

    if (!checkAngleMonotonic(proj)) {
        err = "not star-shaped";
        return false;
    }

    return true;
}

inline bool pointInPolygon(const std::vector<vec2f>& poly, vec2f p) {
    bool inside = false;

    for (int i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        if ((poly[i].y > p.y) != (poly[j].y > p.y) &&
            p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) /
                          (poly[j].y - poly[i].y) +
                      poly[i].x)
            inside = !inside;
    }
    return inside;
}

inline bool pointInTriangle2D(const vec2f& p,
                              const vec2f& a,
                              const vec2f& b,
                              const vec2f& c) {
    constexpr float eps = 1e-6f;
    const float c0 = cross2(a, b, p);
    const float c1 = cross2(b, c, p);
    const float c2 = cross2(c, a, p);
    return c0 >= -eps && c1 >= -eps && c2 >= -eps;
}

bool Cone::contains(const vec3f& p) const {
    vec3f offset = p - apex;
    if (length(offset) < 1e-6f)
        return true;

    vec3f d = normalize(offset);

    auto dirs = computeDirections(apex, base_vertices);
    vec3f dir = computeConeDir(dirs);

    // 点必须在 apex 的 base 方向一侧；背面对称位置不应算在锥体内
    if (dot(d, dir) < 0.0f)
        return false;

    vec3f right, up;
    buildBasis(dir, right, up);

    auto proj = projectDirs(dirs, right, up);

    vec2f pp = {dot(d, right), dot(d, up)};

    return pointInPolygon(proj, pp);
}

void Cone::triangulate() {
    vertices.clear();
    indices.clear();
    base_triangles.clear();
    const float eps = 1e-6f;
    int n = (int)base_vertices.size();
    if (n < 2)
        return;

    // ---------- 1. 计算 cone_dir（用于统一法线方向） ----------
    std::vector<vec3f> dirs;
    dirs.reserve(n);
    for (auto& v : base_vertices)
        dirs.push_back(normalize(v - apex));

    vec3f cone_dir{0, 0, 0};
    for (auto& d : dirs)
        cone_dir += d;

    if (length(cone_dir) < 1e-5f)
        cone_dir = dirs[0];
    else
        cone_dir = normalize(cone_dir);

    // ---------- 2. 生成侧面三角形 ----------
    for (int i = 0; i < n; ++i) {
        const vec3f& v0 = apex;
        const vec3f& v1 = base_vertices[i];
        const vec3f& v2 = base_vertices[(i + 1) % n];

        // ---------- 计算法线 ----------
        vec3f e1 = v1 - v0;
        vec3f e2 = v2 - v0;

        vec3f normal = cross(e1, e2);
        float len = length(normal);
        if (len < eps)
            continue;

        normal /= len;

        // ---------- 统一法线方向（非常关键） ----------
        if (dot(normal, cone_dir) < 0)
            normal = -normal;

        // ---------- 写入顶点 ----------
        uint32_t baseIndex = (uint32_t)vertices.size();

        auto push = [&](const vec3f& p) {
            PosNormalVertex v;
            v.pos.x = p.x;
            v.pos.y = p.y;
            v.pos.z = p.z;
            v.normal.x = normal.x;
            v.normal.y = normal.y;
            v.normal.z = normal.z;
            vertices.push_back(v);
        };

        push(v0);
        push(v1);
        push(v2);

        // ---------- 写入索引 ----------
        indices.push_back(baseIndex + 0);
        indices.push_back(baseIndex + 1);
        indices.push_back(baseIndex + 2);
    }

    // ---------- 3. 底面封盖（耳切法三角化） ----------
    vec3f right, up;
    buildBasis(cone_dir, right, up);
    auto proj = projectDirs(dirs, right, up);

    std::vector<int> order;
    order.reserve(n);
    for (int i = 0; i < n; ++i)
        order.push_back(i);

    float area = 0.0f;
    for (int i = 0; i < n; ++i) {
        const vec2f& a = proj[order[i]];
        const vec2f& b = proj[order[(i + 1) % n]];
        area += a.x * b.y - b.x * a.y;
    }
    if (area < 0.0f)
        std::reverse(order.begin(), order.end());

    auto pushBaseTriangle = [&](int i0, int i1, int i2) {
        const vec3f& v0 = base_vertices[i0];
        const vec3f& v1 = base_vertices[i1];
        const vec3f& v2 = base_vertices[i2];

        vec3f e1 = v1 - v0;
        vec3f e2 = v2 - v0;
        vec3f normal = cross(e1, e2);
        float len = length(normal);
        if (len > eps) {
            normal /= len;
            if (dot(normal, cone_dir) > 0)
                normal = -normal;
        } else {
            normal = -cone_dir;
        }

        uint32_t baseIndex = (uint32_t)vertices.size();
        auto push = [&](const vec3f& p) {
            PosNormalVertex v;
            v.pos.x = p.x;
            v.pos.y = p.y;
            v.pos.z = p.z;
            v.normal.x = normal.x;
            v.normal.y = normal.y;
            v.normal.z = normal.z;
            vertices.push_back(v);
        };
        push(v0);
        push(v1);
        push(v2);
        indices.push_back(baseIndex + 0);
        indices.push_back(baseIndex + 1);
        indices.push_back(baseIndex + 2);
        base_triangles.push_back({static_cast<uint32_t>(i0),
                                  static_cast<uint32_t>(i1),
                                  static_cast<uint32_t>(i2)});
    };

    std::vector<int> remaining = order;
    while (remaining.size() > 3) {
        bool clipped = false;
        for (size_t i = 0; i < remaining.size(); ++i) {
            size_t prev_i = (i + remaining.size() - 1) % remaining.size();
            size_t next_i = (i + 1) % remaining.size();
            int prev = remaining[prev_i];
            int curr = remaining[i];
            int next = remaining[next_i];

            if (cross2(proj[prev], proj[curr], proj[next]) <= 1e-6f)
                continue;

            bool contains_point = false;
            for (size_t j = 0; j < remaining.size(); ++j) {
                int test = remaining[j];
                if (test == prev || test == curr || test == next)
                    continue;
                if (pointInTriangle2D(proj[test], proj[prev], proj[curr],
                                      proj[next])) {
                    contains_point = true;
                    break;
                }
            }
            if (contains_point)
                continue;

            pushBaseTriangle(prev, curr, next);
            remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            break;
        }
        if (!clipped)
            break;
    }

    if (remaining.size() == 3) {
        pushBaseTriangle(remaining[0], remaining[1], remaining[2]);
    } else if (remaining.size() > 3) {
        for (size_t i = 1; i + 1 < remaining.size(); ++i) {
            pushBaseTriangle(remaining[0], remaining[i], remaining[i + 1]);
        }
    }
}

}  // namespace sinriv::kigstudio::voxel::concave