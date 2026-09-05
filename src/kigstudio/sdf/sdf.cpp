#include "kigstudio/sdf/sdf.h"
#include "kigstudio/sdf/sdf_chunked.h"
#include "kigstudio/utils/base64.h"
#include "kigstudio/utils/compress.h"
#include <cstdint>
#include <cstdio>
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
    if (!type_item) return nullptr;
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

std::string SDF_bool::getInfo(int indent) const {
    std::string prefix(indent * 2, ' ');
    const char* op_name = "Unknown";
    switch (op) {
        case SDFBoolOp::Union: op_name = "Union"; break;
        case SDFBoolOp::Intersection: op_name = "Intersection"; break;
        case SDFBoolOp::Subtraction: op_name = "Subtraction"; break;
    }
    std::string result = prefix + "SDF_bool(" + op_name + ")";
    if (left) result += "\n" + left->getInfo(indent + 1);
    if (right) result += "\n" + right->getInfo(indent + 1);
    return result;
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
    if (!json)
        return;

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, json) {
        if (!child->string)
            continue;

        if (cJSON_IsString(child)) {
            if (strcmp(child->string, "op") == 0) {
                const char* op_name = child->valuestring;
                if (strcmp(op_name, "Union") == 0) {
                    op = SDFBoolOp::Union;
                } else if (strcmp(op_name, "Intersection") == 0) {
                    op = SDFBoolOp::Intersection;
                } else if (strcmp(op_name, "Subtraction") == 0) {
                    op = SDFBoolOp::Subtraction;
                }
            }
        } else if (cJSON_IsObject(child)) {
            if (strcmp(child->string, "left") == 0) {
                left = sdf_from_json(child);
            } else if (strcmp(child->string, "right") == 0) {
                right = sdf_from_json(child);
            }
        }
    }
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

std::string SDF_Group::getInfo(int indent) const {
    std::string prefix(indent * 2, ' ');
    std::string result = prefix + "SDF_Group(" + std::to_string(children.size()) + " children)";
    for (const auto& child : children) {
        if (child) result += "\n" + child->getInfo(indent + 1);
    }
    return result;
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

std::string SDF_Translate::getInfo(int indent) const {
    std::string prefix(indent * 2, ' ');
    std::string result = prefix + "SDF_Translate(offset=" + std::to_string(offset.x) + "," +
           std::to_string(offset.y) + "," + std::to_string(offset.z) + ")";
    if (child) result += "\n" + child->getInfo(indent + 1);
    return result;
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
    cJSON_AddItemToObject(obj, "offset",
                          sinriv::kigstudio::to_json(offset));
    if (child) cJSON_AddItemToObject(obj, "child", child->toJSON());
    return obj;
}

void SDF_Translate::fromJSON(const cJSON* json) {
    if (!json)
        return;

    const cJSON* c = nullptr;
    cJSON_ArrayForEach(c, json) {
        if (!c->string)
            continue;

        if (cJSON_IsObject(c)) {
            if (strcmp(c->string, "offset") == 0) {
                offset = sinriv::kigstudio::vec3_from_json<Vec3f>(c);
            } else if (strcmp(c->string, "child") == 0) {
                this->child = sdf_from_json(c);
            }
        }
    }
}

// ============================================================
// SDF_Offset
// ============================================================

std::string SDF_Offset::getInfo(int indent) const {
    std::string prefix(indent * 2, ' ');
    std::string result = prefix + "SDF_Offset(offset=" + std::to_string(offset) + ")";
    if (child) result += "\n" + child->getInfo(indent + 1);
    return result;
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
    auto offset_obj = cJSON_GetObjectItem(json, "offset");
    if (!offset_obj) return;
    offset = static_cast<float>(cJSON_GetNumberValue(offset_obj));
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

std::string SDF_Plane::getInfo(int indent) const {
    std::string prefix(indent * 2, ' ');
    return prefix + "SDF_Plane(" + std::to_string(A) + "x + " + std::to_string(B) + 
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
    if (!json)
        return;

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, json) {
        if (!child->string || !cJSON_IsNumber(child))
            continue;

        const double value = cJSON_GetNumberValue(child);
        if (strcmp(child->string, "A") == 0) {
            A = static_cast<float>(value);
        } else if (strcmp(child->string, "B") == 0) {
            B = static_cast<float>(value);
        } else if (strcmp(child->string, "C") == 0) {
            C = static_cast<float>(value);
        } else if (strcmp(child->string, "D") == 0) {
            D = static_cast<float>(value);
        }
    }
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

std::string SDFGrid::getInfo(int indent) const {
    std::string prefix(indent * 2, ' ');
    return prefix + "SDFGrid(size=[" + std::to_string(sx) + "," + std::to_string(sy) +
           "," + std::to_string(sz) + "], min_bound=[" + std::to_string(min_bound.x) +
           "," + std::to_string(min_bound.y) + "," + std::to_string(min_bound.z) + "])";
}

cJSON* SDFGrid::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "grid");

    cJSON_AddItemToObject(obj, "min_bound",
                          sinriv::kigstudio::to_json(min_bound));
    cJSON_AddItemToObject(obj, "max_bound",
                          sinriv::kigstudio::to_json(max_bound));

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
    if (!json)
        return;

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, json) {
        if (!child->string)
            continue;

        const char* key = child->string;

        if (cJSON_IsObject(child)) {
            if (strcmp(key, "min_bound") == 0) {
                min_bound = sinriv::kigstudio::vec3_from_json<Vec3i>(child);
            } else if (strcmp(key, "max_bound") == 0) {
                max_bound = sinriv::kigstudio::vec3_from_json<Vec3i>(child);
            }
        } else if (cJSON_IsNumber(child)) {
            if (strcmp(key, "sx") == 0) {
                sx = child->valueint;
            } else if (strcmp(key, "sy") == 0) {
                sy = child->valueint;
            } else if (strcmp(key, "sz") == 0) {
                sz = child->valueint;
            }
        } else if (cJSON_IsArray(child) && strcmp(key, "sdf") == 0) {
            int count = cJSON_GetArraySize(child);
            sdf.resize(count);
            for (int i = 0; i < count; ++i) {
                const cJSON* v = cJSON_GetArrayItem(child, i);
                if (v && cJSON_IsNumber(v))
                    sdf[i] = static_cast<float>(cJSON_GetNumberValue(v));
            }
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
    sdf_register_type("chunked_grid", [](const cJSON* json) -> std::shared_ptr<SDFBase> {
        auto obj = std::make_shared<SDFChunkedGrid>();
        obj->fromJSON(json);
        return obj;
    });
    return true;
}();

// ============ SDFChunkedGrid chunk 记录二进制编解码 ============

std::vector<uint8_t> serialize_chunk_records(const SDFChunkedGrid& grid) {
    std::vector<uint8_t> raw;
    auto append = [&](const void* p, size_t n) {
        const uint8_t* b = static_cast<const uint8_t*>(p);
        raw.insert(raw.end(), b, b + n);
    };
    for (const auto& [key, chunk] : grid.chunks) {
        const uint8_t type = chunk.type == SDFChunk::Type::Dense ? 1 : 0;
        append(&key, sizeof(key));
        append(&type, sizeof(type));
        if (type == 0) {
            append(&chunk.uniform_value, sizeof(float));
        } else {
            append(chunk.dense.get(), sizeof(float) * SDFChunk::VOXEL_COUNT);
        }
    }
    return raw;
}

bool deserialize_chunk_records(const uint8_t* data, size_t size,
                               SDFChunkedGrid& grid) {
    size_t pos = 0;
    auto read = [&](void* out, size_t n) -> bool {
        if (pos + n > size)
            return false;
        std::memcpy(out, data + pos, n);
        pos += n;
        return true;
    };
    while (pos < size) {
        uint64_t key;
        uint8_t type;
        if (!read(&key, sizeof(key)) || !read(&type, sizeof(type)))
            return false;
        SDFChunk chunk;
        if (type == 0) {
            chunk.type = SDFChunk::Type::Uniform;
            if (!read(&chunk.uniform_value, sizeof(float)))
                return false;
        } else if (type == 1) {
            chunk.type = SDFChunk::Type::Dense;
            chunk.dense = std::make_unique<float[]>(SDFChunk::VOXEL_COUNT);
            if (!read(chunk.dense.get(),
                      sizeof(float) * SDFChunk::VOXEL_COUNT))
                return false;
        } else {
            return false;
        }
        grid.chunks.emplace(key, chunk);
    }
    return true;
}

// ============ SDFChunkedGrid JSON 序列化 ============
// chunks 以 base64(zlib(二进制chunk记录)) 字符串存储；旧 JSON 数组格式可读。

cJSON* SDFChunkedGrid::toJSON() const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", "chunked_grid");
    cJSON_AddItemToObject(obj, "global_position",
                          sinriv::kigstudio::to_json(global_position));
    cJSON_AddItemToObject(obj, "voxel_size",
                          sinriv::kigstudio::to_json(voxel_size));

    std::vector<uint8_t> raw = serialize_chunk_records(*this);
    std::vector<uint8_t> compressed;
    if (sinriv::kigstudio::zlibCompress(raw, compressed)) {
        cJSON_AddStringToObject(
            obj, "chunks_b64",
            sinriv::kigstudio::base64Encode(compressed).c_str());
        cJSON_AddNumberToObject(obj, "chunks_raw_size",
                                static_cast<double>(raw.size()));
    }
    return obj;
}

void SDFChunkedGrid::fromJSON(const cJSON* json) {
    if (!json) {
        return;
    }
    chunks.clear();

    const cJSON* gp = cJSON_GetObjectItem(json, "global_position");
    if (gp) {
        global_position = sinriv::kigstudio::vec3_from_json<Vec3f>(gp);
    }
    const cJSON* vs = cJSON_GetObjectItem(json, "voxel_size");
    if (vs) {
        voxel_size = sinriv::kigstudio::vec3_from_json<Vec3f>(vs);
    }

    // 新格式：base64(zlib(二进制chunk记录))
    const cJSON* b64 = cJSON_GetObjectItem(json, "chunks_b64");
    const cJSON* raw_size_j = cJSON_GetObjectItem(json, "chunks_raw_size");
    if (b64 && cJSON_IsString(b64) && raw_size_j &&
        cJSON_IsNumber(raw_size_j)) {
        const size_t raw_size =
            static_cast<size_t>(cJSON_GetNumberValue(raw_size_j));
        std::vector<uint8_t> comp, raw;
        if (sinriv::kigstudio::base64Decode(b64->valuestring, comp) &&
            (raw_size == 0 ||
             sinriv::kigstudio::zlibDecompress(comp, raw, raw_size))) {
            deserialize_chunk_records(raw.data(), raw.size(), *this);
        }
        return;
    }

    // 旧格式：JSON 数组（保留读取兼容）
    const cJSON* arr = cJSON_GetObjectItem(json, "chunks");
    const cJSON* item = nullptr;
    cJSON_ArrayForEach(item, arr) {
        const cJSON* coord = cJSON_GetObjectItem(item, "chunk");
        if (!coord || cJSON_GetArraySize(coord) != 3) {
            continue;
        }
        const int cx = cJSON_GetArrayItem(coord, 0)->valueint;
        const int cy = cJSON_GetArrayItem(coord, 1)->valueint;
        const int cz = cJSON_GetArrayItem(coord, 2)->valueint;
        SDFChunk& chunk = chunks[voxel::packChunkKey(cx, cy, cz)];

        const cJSON* value = cJSON_GetObjectItem(item, "value");
        if (value && cJSON_IsNumber(value)) {
            chunk.type = SDFChunk::Type::Uniform;
            chunk.uniform_value =
                static_cast<float>(cJSON_GetNumberValue(value));
            continue;
        }
        const cJSON* data = cJSON_GetObjectItem(item, "data");
        if (data && cJSON_IsArray(data) &&
            cJSON_GetArraySize(data) == SDFChunk::VOXEL_COUNT) {
            chunk.type = SDFChunk::Type::Dense;
            chunk.dense = std::make_unique<float[]>(SDFChunk::VOXEL_COUNT);
            int i = 0;
            const cJSON* v = nullptr;
            cJSON_ArrayForEach(v, data) {
                chunk.dense[i++] =
                    static_cast<float>(cJSON_GetNumberValue(v));
            }
        }
    }
}

// ============ SDFChunkedGrid .sdfchk 文件序列化 ============
// v1: chunk 记录直接写在头部之后（原始）
// v2: chunk 记录整体 zlib 压缩（SDF 距离场平滑，实测可压到 ~10%）

bool save_chunked_file(const std::filesystem::path& path,
                       const SDFChunkedGrid& grid,
                       std::string* error) {
    // chunk 记录拼成 raw buffer 后整体压缩
    std::vector<uint8_t> raw = serialize_chunk_records(grid);
    std::vector<uint8_t> compressed;
    if (!sinriv::kigstudio::zlibCompress(raw, compressed)) {
        if (error)
            *error = "zlib compress failed";
        return false;
    }

#ifdef _WIN32
    FILE* fp = _wfopen(path.wstring().c_str(), L"wb");
#else
    FILE* fp = std::fopen(path.c_str(), "wb");
#endif
    if (!fp) {
        if (error)
            *error = "open file failed";
        return false;
    }
    auto write = [&](const void* data, size_t size) -> bool {
        return std::fwrite(data, 1, size, fp) == size;
    };
    const char magic[8] = {'S', 'D', 'F', 'C', 'H', 'K', '1', '\0'};
    const uint32_t version = 2;
    bool ok = write(magic, 8) && write(&version, sizeof(version));
    ok = ok && write(&grid.global_position, sizeof(grid.global_position));
    ok = ok && write(&grid.voxel_size, sizeof(grid.voxel_size));
    const uint32_t chunk_count = static_cast<uint32_t>(grid.chunks.size());
    ok = ok && write(&chunk_count, sizeof(chunk_count));
    const uint32_t comp_size = static_cast<uint32_t>(compressed.size());
    const uint32_t raw_size = static_cast<uint32_t>(raw.size());
    ok = ok && write(&comp_size, sizeof(comp_size)) &&
         write(&raw_size, sizeof(raw_size));
    ok = ok && write(compressed.data(), comp_size);
    std::fclose(fp);
    if (!ok && error)
        *error = "write file failed";
    return ok;
}

bool load_chunked_file(const std::filesystem::path& path,
                       SDFChunkedGrid& grid,
                       std::string* error) {
#ifdef _WIN32
    FILE* fp = _wfopen(path.wstring().c_str(), L"rb");
#else
    FILE* fp = std::fopen(path.c_str(), "rb");
#endif
    if (!fp) {
        if (error)
            *error = "open file failed";
        return false;
    }
    auto read_file = [&](void* data, size_t size) -> bool {
        return std::fread(data, 1, size, fp) == size;
    };
    auto fail = [&](const char* msg) {
        std::fclose(fp);
        if (error)
            *error = msg;
        return false;
    };
    char magic[8];
    uint32_t version;
    if (!read_file(magic, 8) || !read_file(&version, sizeof(version)))
        return fail("truncated header");
    if (std::strncmp(magic, "SDFCHK1", 7) != 0 ||
        (version != 1 && version != 2))
        return fail("bad magic or unsupported version");

    // 读到临时对象，失败不破坏调用方的现有数据
    SDFChunkedGrid tmp;
    if (!read_file(&tmp.global_position, sizeof(tmp.global_position)) ||
        !read_file(&tmp.voxel_size, sizeof(tmp.voxel_size)))
        return fail("truncated header");
    uint32_t chunk_count;
    if (!read_file(&chunk_count, sizeof(chunk_count)))
        return fail("truncated header");

    // v1 读取剩余文件内容；v2 先解压到 buffer
    std::vector<uint8_t> buf;
    if (version == 1) {
        uint8_t block[65536];
        size_t n;
        while ((n = std::fread(block, 1, sizeof(block), fp)) > 0)
            buf.insert(buf.end(), block, block + n);
    } else {
        uint32_t comp_size, raw_size;
        if (!read_file(&comp_size, sizeof(comp_size)) ||
            !read_file(&raw_size, sizeof(raw_size)))
            return fail("truncated header");
        std::vector<uint8_t> comp(comp_size);
        if (comp_size > 0 && !read_file(comp.data(), comp_size))
            return fail("truncated chunk data");
        if (raw_size > 0 &&
            !sinriv::kigstudio::zlibDecompress(comp, buf, raw_size))
            return fail("zlib decompress failed");
    }
    if (!deserialize_chunk_records(buf.data(), buf.size(), tmp))
        return fail("truncated chunk data");
    if (tmp.chunks.size() != chunk_count)
        return fail("chunk count mismatch");
    std::fclose(fp);
    grid = std::move(tmp);
    return true;
}

}  // namespace sinriv::kigstudio::sdf

#undef KIGSTUDIO_SIMD_LOOP
