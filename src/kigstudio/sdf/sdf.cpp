#include "kigstudio/sdf/sdf.h"
#include <cstdint>
#include <cstring>
#include <cmath>

#if defined(_MSC_VER)
#define KIGSTUDIO_SIMD_LOOP __pragma(loop(ivdep))
#else
#define KIGSTUDIO_SIMD_LOOP _Pragma("omp simd")
#endif

namespace sinriv::kigstudio::sdf {

static std::unordered_map<std::string, std::function<std::shared_ptr<SDFBase>(const cJSON*)>>& get_registry() {
    static std::unordered_map<std::string, std::function<std::shared_ptr<SDFBase>(const cJSON*)>> registry;
    return registry;
}

void sdf_register_type(
    const std::string& name,
    std::function<std::shared_ptr<SDFBase>(const cJSON*)> factory) {
    get_registry()[name] = std::move(factory);
}

std::shared_ptr<SDFBase> sdf_from_json(const cJSON* json) {
    if (!json) return nullptr;
    const cJSON* type_item = cJSON_GetObjectItem(json, "type");
    const char* type = cJSON_GetStringValue(type_item);
    if (!type) return nullptr;
    auto it = get_registry().find(type);
    if (it == get_registry().end()) return nullptr;
    return it->second(json);
}

void SDFBase::get(const Vec3f& begin,
                  const Vec3f& voxelSize,
                  const Vec3i& voxelCount,
                  std::vector<float>& out) const {
    const size_t total = static_cast<size_t>(voxelCount.x) *
                         static_cast<size_t>(voxelCount.y) *
                         static_cast<size_t>(voxelCount.z);
    out.resize(total);

    size_t i = 0;
    for (int z = 0; z < voxelCount.z; ++z) {
        const float wz = begin.z + static_cast<float>(z) * voxelSize.z;
        for (int y = 0; y < voxelCount.y; ++y) {
            const float wy = begin.y + static_cast<float>(y) * voxelSize.y;
            for (int x = 0; x < voxelCount.x; ++x) {
                const float wx = begin.x + static_cast<float>(x) * voxelSize.x;
                out[i++] = get(Vec3f(wx, wy, wz));
            }
        }
    }
}

// ============================================================
// SDF_bool
// ============================================================

void SDF_bool::get(const Vec3f& begin,
                   const Vec3f& voxelSize,
                   const Vec3i& voxelCount,
                   std::vector<float>& out) const {
    if (!left || !right) {
        SDFBase::get(begin, voxelSize, voxelCount, out);
        return;
    }

    left->get(begin, voxelSize, voxelCount, out);

    std::vector<float> rhs;
    right->get(begin, voxelSize, voxelCount, rhs);

    const size_t count = std::min(out.size(), rhs.size());
    float* out_data = out.data();
    const float* rhs_data = rhs.data();
    constexpr int64_t simd_block_size = 1024;
    const int64_t block_count =
        (static_cast<int64_t>(count) + simd_block_size - 1) / simd_block_size;

    switch (op) {
        case SDFBoolOp::Union:
#pragma omp parallel for
            for (int64_t block = 0; block < block_count; ++block) {
                const int64_t begin_i = block * simd_block_size;
                const int64_t end_i =
                    std::min<int64_t>(begin_i + simd_block_size,
                                      static_cast<int64_t>(count));
                KIGSTUDIO_SIMD_LOOP
                for (int64_t i = begin_i; i < end_i; ++i) {
                    out_data[i] = std::min(out_data[i], rhs_data[i]);
                }
            }
            break;
        case SDFBoolOp::Intersection:
#pragma omp parallel for
            for (int64_t block = 0; block < block_count; ++block) {
                const int64_t begin_i = block * simd_block_size;
                const int64_t end_i =
                    std::min<int64_t>(begin_i + simd_block_size,
                                      static_cast<int64_t>(count));
                KIGSTUDIO_SIMD_LOOP
                for (int64_t i = begin_i; i < end_i; ++i) {
                    out_data[i] = std::max(out_data[i], rhs_data[i]);
                }
            }
            break;
        case SDFBoolOp::Subtraction:
#pragma omp parallel for
            for (int64_t block = 0; block < block_count; ++block) {
                const int64_t begin_i = block * simd_block_size;
                const int64_t end_i =
                    std::min<int64_t>(begin_i + simd_block_size,
                                      static_cast<int64_t>(count));
                KIGSTUDIO_SIMD_LOOP
                for (int64_t i = begin_i; i < end_i; ++i) {
                    out_data[i] = std::max(out_data[i], -rhs_data[i]);
                }
            }
            break;
    }
}

std::string SDF_bool::getInfo() const {
    const char* op_name = "Unknown";
    switch (op) {
        case SDFBoolOp::Union: op_name = "Union"; break;
        case SDFBoolOp::Intersection: op_name = "Intersection"; break;
        case SDFBoolOp::Subtraction: op_name = "Subtraction"; break;
    }
    return std::string("SDF_bool(") + op_name + ")";
}

cJSON* SDF_bool::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "bool");
    const char* op_name = "Union";
    switch (op) {
        case SDFBoolOp::Union: op_name = "Union"; break;
        case SDFBoolOp::Intersection: op_name = "Intersection"; break;
        case SDFBoolOp::Subtraction: op_name = "Subtraction"; break;
    }
    cJSON_AddStringToObject(obj, "op", op_name);
    if (left) cJSON_AddItemToObject(obj, "left", left->toJSON());
    if (right) cJSON_AddItemToObject(obj, "right", right->toJSON());
    return obj;
}

void SDF_bool::fromJSON(const cJSON* json) {
    const char* op_name = cJSON_GetStringValue(cJSON_GetObjectItem(json, "op"));
    if (op_name) {
        if (strcmp(op_name, "Union") == 0) op = SDFBoolOp::Union;
        else if (strcmp(op_name, "Intersection") == 0) op = SDFBoolOp::Intersection;
        else if (strcmp(op_name, "Subtraction") == 0) op = SDFBoolOp::Subtraction;
    }
    const cJSON* lj = cJSON_GetObjectItem(json, "left");
    if (lj) left = sdf_from_json(lj);
    const cJSON* rj = cJSON_GetObjectItem(json, "right");
    if (rj) right = sdf_from_json(rj);
}

// ============================================================
// Factory functions
// ============================================================

std::shared_ptr<SDF_bool> sdf_union(
    std::shared_ptr<SDFBase> a, std::shared_ptr<SDFBase> b) {
    return std::make_shared<SDF_bool>(SDFBoolOp::Union,
                                       std::move(a), std::move(b));
}
std::shared_ptr<SDF_bool> sdf_intersection(
    std::shared_ptr<SDFBase> a, std::shared_ptr<SDFBase> b) {
    return std::make_shared<SDF_bool>(SDFBoolOp::Intersection,
                                       std::move(a), std::move(b));
}
std::shared_ptr<SDF_bool> sdf_subtraction(
    std::shared_ptr<SDFBase> a, std::shared_ptr<SDFBase> b) {
    return std::make_shared<SDF_bool>(SDFBoolOp::Subtraction,
                                       std::move(a), std::move(b));
}

std::shared_ptr<SDFBase> sdf_group(std::vector<std::shared_ptr<SDFBase>> children) {
    return std::make_shared<SDF_Group>(std::move(children));
}

// ============================================================
// SDF_Group
// ============================================================

float SDF_Group::get(const Vec3f& p) const {
    if (children.empty())
        return std::numeric_limits<float>::infinity();
    float result = std::numeric_limits<float>::infinity();
    for (const auto& child : children) {
        if (!child) continue;
        result = std::min(result, child->get(p));
    }
    return result;
}

void SDF_Group::get(const Vec3f& begin,
                    const Vec3f& voxelSize,
                    const Vec3i& voxelCount,
                    std::vector<float>& out) const {
    const size_t total = static_cast<size_t>(voxelCount.x) *
                         static_cast<size_t>(voxelCount.y) *
                         static_cast<size_t>(voxelCount.z);
    out.assign(total, std::numeric_limits<float>::infinity());
    if (children.empty())
        return;

    std::vector<float> child_out;
    for (const auto& child : children) {
        if (!child)
            continue;
        child->get(begin, voxelSize, voxelCount, child_out);
        const size_t count = std::min(out.size(), child_out.size());
        for (size_t i = 0; i < count; ++i) {
            out[i] = std::min(out[i], child_out[i]);
        }
    }
}

std::string SDF_Group::getInfo() const {
    return "SDF_Group(" + std::to_string(children.size()) + " children)";
}

cJSON* SDF_Group::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "group");
    cJSON* arr = cJSON_CreateArray();
    for (const auto& child : children) {
        if (child)
            cJSON_AddItemToArray(arr, child->toJSON());
    }
    cJSON_AddItemToObject(obj, "children", arr);
    return obj;
}

void SDF_Group::fromJSON(const cJSON* json) {
    children.clear();
    const cJSON* arr = cJSON_GetObjectItem(json, "children");
    if (!arr)
        return;
    int count = cJSON_GetArraySize(arr);
    children.reserve(count);
    for (int i = 0; i < count; ++i) {
        const cJSON* item = cJSON_GetArrayItem(arr, i);
        if (!item)
            continue;
        auto child = sdf_from_json(item);
        if (child)
            children.push_back(std::move(child));
    }
}

// ============================================================
// SDF_Translate
// ============================================================

std::string SDF_Translate::getInfo() const {
    return "SDF_Translate(offset=" + std::to_string(offset.x) + "," +
           std::to_string(offset.y) + "," + std::to_string(offset.z) + ")";
}

void SDF_Translate::get(const Vec3f& begin,
                         const Vec3f& voxelSize,
                         const Vec3i& voxelCount,
                         std::vector<float>& out) const {
    if (!child) {
        out.clear();
        return;
    }
    child->get(begin - offset, voxelSize, voxelCount, out);
}

cJSON* SDF_Translate::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "translate");
    cJSON* off = cJSON_CreateObject();
    cJSON_AddNumberToObject(off, "x", offset.x);
    cJSON_AddNumberToObject(off, "y", offset.y);
    cJSON_AddNumberToObject(off, "z", offset.z);
    cJSON_AddItemToObject(obj, "offset", off);
    if (child) cJSON_AddItemToObject(obj, "child", child->toJSON());
    return obj;
}

void SDF_Translate::fromJSON(const cJSON* json) {
    const cJSON* off = cJSON_GetObjectItem(json, "offset");
    if (off) {
        offset.x = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(off, "x")));
        offset.y = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(off, "y")));
        offset.z = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(off, "z")));
    }
    const cJSON* cj = cJSON_GetObjectItem(json, "child");
    if (cj) child = sdf_from_json(cj);
}

// ============================================================
// SDF_Offset
// ============================================================

std::string SDF_Offset::getInfo() const {
    return "SDF_Offset(offset=" + std::to_string(offset) + ")";
}

void SDF_Offset::get(const Vec3f& begin,
                      const Vec3f& voxelSize,
                      const Vec3i& voxelCount,
                      std::vector<float>& out) const {
    if (!child) {
        out.clear();
        return;
    }
    child->get(begin, voxelSize, voxelCount, out);
    const size_t count = out.size();
    for (size_t i = 0; i < count; ++i) {
        out[i] -= offset;
    }
}

cJSON* SDF_Offset::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "offset");
    cJSON_AddNumberToObject(obj, "offset", offset);
    if (child) cJSON_AddItemToObject(obj, "child", child->toJSON());
    return obj;
}

void SDF_Offset::fromJSON(const cJSON* json) {
    offset = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "offset")));
    const cJSON* cj = cJSON_GetObjectItem(json, "child");
    if (cj) child = sdf_from_json(cj);
}

// ============================================================
// SDF_Plane
// ============================================================

void SDF_Plane::get(const Vec3f& begin,
                    const Vec3f& voxelSize,
                    const Vec3i& voxelCount,
                    std::vector<float>& out) const {
    const size_t total = static_cast<size_t>(voxelCount.x) *
                         static_cast<size_t>(voxelCount.y) *
                         static_cast<size_t>(voxelCount.z);
    out.resize(total);

    size_t i = 0;
    for (int z = 0; z < voxelCount.z; ++z) {
        const float wz = begin.z + static_cast<float>(z) * voxelSize.z;
        for (int y = 0; y < voxelCount.y; ++y) {
            const float wy = begin.y + static_cast<float>(y) * voxelSize.y;
            for (int x = 0; x < voxelCount.x; ++x) {
                const float wx = begin.x + static_cast<float>(x) * voxelSize.x;
                out[i++] = A * wx + B * wy + C * wz + D;
            }
        }
    }
}

std::string SDF_Plane::getInfo() const {
    return "SDF_Plane(" + std::to_string(A) + "x + " + std::to_string(B) + 
           "y + " + std::to_string(C) + "z + " + std::to_string(D) + ")";
}

cJSON* SDF_Plane::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "plane");
    cJSON_AddNumberToObject(obj, "A", A);
    cJSON_AddNumberToObject(obj, "B", B);
    cJSON_AddNumberToObject(obj, "C", C);
    cJSON_AddNumberToObject(obj, "D", D);
    return obj;
}

void SDF_Plane::fromJSON(const cJSON* json) {
    A = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "A")));
    B = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "B")));
    C = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "C")));
    D = static_cast<float>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "D")));
}

// ============================================================
// SDFGrid
// ============================================================

float SDFGrid::get(const Vec3f& p) const {
    if (sx <= 0 || sy <= 0 || sz <= 0 || sdf.empty()) {
        return 1e6f;
    }

    float dx = p.x - static_cast<float>(min_bound.x);
    float dy = p.y - static_cast<float>(min_bound.y);
    float dz = p.z - static_cast<float>(min_bound.z);

    dx = std::clamp(dx, 0.0f, static_cast<float>(sx - 1));
    dy = std::clamp(dy, 0.0f, static_cast<float>(sy - 1));
    dz = std::clamp(dz, 0.0f, static_cast<float>(sz - 1));

    int x0 = static_cast<int>(std::floor(dx));
    int y0 = static_cast<int>(std::floor(dy));
    int z0 = static_cast<int>(std::floor(dz));
    int x1 = std::min(x0 + 1, sx - 1);
    int y1 = std::min(y0 + 1, sy - 1);
    int z1 = std::min(z0 + 1, sz - 1);

    float fx = dx - static_cast<float>(x0);
    float fy = dy - static_cast<float>(y0);
    float fz = dz - static_cast<float>(z0);

    float c000 = get(x0, y0, z0);
    float c100 = get(x1, y0, z0);
    float c010 = get(x0, y1, z0);
    float c110 = get(x1, y1, z0);
    float c001 = get(x0, y0, z1);
    float c101 = get(x1, y0, z1);
    float c011 = get(x0, y1, z1);
    float c111 = get(x1, y1, z1);

    float c00 = c000 * (1.0f - fx) + c100 * fx;
    float c10 = c010 * (1.0f - fx) + c110 * fx;
    float c01 = c001 * (1.0f - fx) + c101 * fx;
    float c11 = c011 * (1.0f - fx) + c111 * fx;

    float c0 = c00 * (1.0f - fy) + c10 * fy;
    float c1 = c01 * (1.0f - fy) + c11 * fy;

    return c0 * (1.0f - fz) + c1 * fz;
}

std::string SDFGrid::getInfo() const {
    return "SDFGrid(size=[" + std::to_string(sx) + "," + std::to_string(sy) +
           "," + std::to_string(sz) + "], min_bound=[" + std::to_string(min_bound.x) +
           "," + std::to_string(min_bound.y) + "," + std::to_string(min_bound.z) + "])";
}

cJSON* SDFGrid::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "grid");

    cJSON* minb = cJSON_CreateObject();
    cJSON_AddNumberToObject(minb, "x", min_bound.x);
    cJSON_AddNumberToObject(minb, "y", min_bound.y);
    cJSON_AddNumberToObject(minb, "z", min_bound.z);
    cJSON_AddItemToObject(obj, "min_bound", minb);

    cJSON* maxb = cJSON_CreateObject();
    cJSON_AddNumberToObject(maxb, "x", max_bound.x);
    cJSON_AddNumberToObject(maxb, "y", max_bound.y);
    cJSON_AddNumberToObject(maxb, "z", max_bound.z);
    cJSON_AddItemToObject(obj, "max_bound", maxb);

    cJSON_AddNumberToObject(obj, "sx", sx);
    cJSON_AddNumberToObject(obj, "sy", sy);
    cJSON_AddNumberToObject(obj, "sz", sz);

    if (!sdf.empty()) {
        cJSON* data = cJSON_CreateFloatArray(sdf.data(), static_cast<int>(sdf.size()));
        cJSON_AddItemToObject(obj, "sdf", data);
    }

    return obj;
}

void SDFGrid::fromJSON(const cJSON* json) {
    const cJSON* minb = cJSON_GetObjectItem(json, "min_bound");
    if (minb) {
        min_bound.x = static_cast<int>(cJSON_GetNumberValue(cJSON_GetObjectItem(minb, "x")));
        min_bound.y = static_cast<int>(cJSON_GetNumberValue(cJSON_GetObjectItem(minb, "y")));
        min_bound.z = static_cast<int>(cJSON_GetNumberValue(cJSON_GetObjectItem(minb, "z")));
    }
    const cJSON* maxb = cJSON_GetObjectItem(json, "max_bound");
    if (maxb) {
        max_bound.x = static_cast<int>(cJSON_GetNumberValue(cJSON_GetObjectItem(maxb, "x")));
        max_bound.y = static_cast<int>(cJSON_GetNumberValue(cJSON_GetObjectItem(maxb, "y")));
        max_bound.z = static_cast<int>(cJSON_GetNumberValue(cJSON_GetObjectItem(maxb, "z")));
    }
    sx = static_cast<int>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "sx")));
    sy = static_cast<int>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "sy")));
    sz = static_cast<int>(cJSON_GetNumberValue(cJSON_GetObjectItem(json, "sz")));

    const cJSON* data = cJSON_GetObjectItem(json, "sdf");
    if (data && cJSON_IsArray(data)) {
        int count = cJSON_GetArraySize(data);
        sdf.resize(count);
        for (int i = 0; i < count; ++i) {
            sdf[i] = static_cast<float>(cJSON_GetNumberValue(cJSON_GetArrayItem(data, i)));
        }
    }
}

// ============================================================
// Static registration
// ============================================================

static bool _register_sdf_types = []() {
    sdf_register_type("bool", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDF_bool>(SDFBoolOp::Union, nullptr, nullptr);
        obj->fromJSON(json);
        return obj;
    });
    sdf_register_type("translate", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDF_Translate>(Vec3f(0, 0, 0), nullptr);
        obj->fromJSON(json);
        return obj;
    });
    sdf_register_type("offset", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDF_Offset>(0.0f, nullptr);
        obj->fromJSON(json);
        return obj;
    });
    sdf_register_type("plane", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDF_Plane>();
        obj->fromJSON(json);
        return obj;
    });
    sdf_register_type("grid", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDFGrid>();
        obj->fromJSON(json);
        return obj;
    });
    sdf_register_type("group", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDF_Group>(std::vector<std::shared_ptr<SDFBase>>());
        obj->fromJSON(json);
        return obj;
    });
    return true;
}();

}  // namespace sinriv::kigstudio::sdf

#undef KIGSTUDIO_SIMD_LOOP
