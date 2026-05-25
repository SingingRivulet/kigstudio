#pragma once
#include <algorithm>
#include <cmath>
#include "kigstudio/utils/vec3.h"

namespace sinriv::kigstudio::sdf {

using Vec3f = sinriv::kigstudio::vec3<float>;

struct SDFBase {
    virtual float get(const Vec3f& p) const = 0;
    inline float get(float x, float y, float z) const { return get(Vec3f(x, y, z)); }
};

struct SDFGrid {
    Vec3i min_bound;
    Vec3i max_bound;

    int sx = 0;
    int sy = 0;
    int sz = 0;

    std::vector<float> sdf;

    inline int index(int x, int y, int z) const {
        return (z * sy + y) * sx + x;
    }

    inline bool inBounds(int x, int y, int z) const {
        return x >= 0 && y >= 0 && z >= 0 && x < sx && y < sy && z < sz;
    }

    inline float get(int x, int y, int z) const { return sdf[index(x, y, z)]; }

    inline void set(int x, int y, int z, float v) { sdf[index(x, y, z)] = v; }

    inline Vec3i denseToWorldVoxel(int x, int y, int z) const {
        return Vec3i(x + min_bound.x, y + min_bound.y, z + min_bound.z);
    }

    inline Vec3i worldVoxelToDense(const Vec3i& p) const {
        return Vec3i(p.x - min_bound.x, p.y - min_bound.y, p.z - min_bound.z);
    }
};
}  // namespace sinriv::kigstudio::sdf