#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "kigstudio/io/pmx_loader.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

namespace sinriv::kigstudio::io {

namespace {

// ============================================================================
// Helpers
// ============================================================================

struct PMXGlobals {
    uint8_t text_encoding = 0;        // 0=UTF16LE, 1=UTF8
    uint8_t additional_uv_count = 0;
    uint8_t vertex_index_size = 4;
    uint8_t texture_index_size = 1;
    uint8_t material_index_size = 1;
    uint8_t bone_index_size = 1;
    uint8_t morph_index_size = 1;
    uint8_t rigid_body_index_size = 1;
};

// Simple RAII file handle.
class FileHandle {
   public:
    explicit FileHandle(const std::string& path) {
        file_ = std::fopen(path.c_str(), "rb");
    }
    ~FileHandle() {
        if (file_) std::fclose(file_);
    }
    FILE* get() const { return file_; }
    bool valid() const { return file_ != nullptr; }
   private:
    FILE* file_ = nullptr;
};

// Read a little-endian integer.
template <typename T>
T read_le(FILE* f) {
    T value = 0;
    std::fread(&value, sizeof(T), 1, f);
    return value;
}

// Read a single byte.
uint8_t read_u8(FILE* f) {
    uint8_t v = 0;
    std::fread(&v, 1, 1, f);
    return v;
}

// Read a signed int32.
int32_t read_i32(FILE* f) {
    return static_cast<int32_t>(read_le<int32_t>(f));
}

// Read a float.
float read_f32(FILE* f) {
    return read_le<float>(f);
}

// Read a variable-size index based on PMX globals.
int read_index(FILE* f, uint8_t size) {
    switch (size) {
        case 1: {
            int8_t v = 0;
            std::fread(&v, 1, 1, f);
            return static_cast<int>(v);
        }
        case 2: {
            int16_t v = 0;
            std::fread(&v, 2, 1, f);
            return static_cast<int>(v);
        }
        case 4: {
            int32_t v = 0;
            std::fread(&v, 4, 1, f);
            return static_cast<int>(v);
        }
        default:
            return 0;
    }
}

// Convert UTF-16LE code unit to UTF-8 bytes.
static void utf16le_codeunit_to_utf8(uint32_t code, std::string& out) {
    if (code <= 0x7F) {
        out.push_back(static_cast<char>(code));
    } else if (code <= 0x7FF) {
        out.push_back(static_cast<char>(0xC0 | ((code >> 6) & 0x1F)));
        out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
    } else if (code <= 0xFFFF) {
        out.push_back(static_cast<char>(0xE0 | ((code >> 12) & 0x0F)));
        out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
    } else {
        out.push_back(static_cast<char>(0xF0 | ((code >> 18) & 0x07)));
        out.push_back(static_cast<char>(0x80 | ((code >> 12) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
    }
}

// Convert a UTF-16LE byte buffer to UTF-8 std::string.
std::string utf16le_to_utf8(const char* data, int len) {
    std::string out;
    out.reserve(static_cast<size_t>(len));
    const uint8_t* p = reinterpret_cast<const uint8_t*>(data);
    const uint8_t* end = p + len;
    while (p + 1 < end) {
        uint16_t unit = static_cast<uint16_t>(p[0] | (p[1] << 8));
        p += 2;
        if (unit >= 0xD800 && unit <= 0xDBFF && p + 1 < end) {
            uint16_t low = static_cast<uint16_t>(p[0] | (p[1] << 8));
            p += 2;
            uint32_t code = 0x10000 + ((unit - 0xD800) << 10) + (low - 0xDC00);
            utf16le_codeunit_to_utf8(code, out);
        } else {
            utf16le_codeunit_to_utf8(unit, out);
        }
    }
    return out;
}

// Read a PMX text block (int32 length + bytes) and convert to UTF-8.
std::string read_text(FILE* f, const PMXGlobals& g) {
    int32_t len = read_i32(f);
    if (len <= 0) return {};
    std::vector<char> buffer(len);
    std::fread(buffer.data(), 1, static_cast<size_t>(len), f);
    if (g.text_encoding == 1) {
        // UTF-8
        return std::string(buffer.data(), static_cast<size_t>(len));
    }
    // UTF-16LE
    return utf16le_to_utf8(buffer.data(), len);
}

// Read a vec3.
vec3f read_vec3(FILE* f) {
    float x = read_f32(f);
    float y = read_f32(f);
    float z = read_f32(f);
    return vec3f(x, y, z);
}

void read_vec3(FILE* f, float out[3]) {
    out[0] = read_f32(f);
    out[1] = read_f32(f);
    out[2] = read_f32(f);
}

// Read a vec2.
std::pair<float, float> read_vec2(FILE* f) {
    float x = read_f32(f);
    float y = read_f32(f);
    return {x, y};
}

// Read a vec4.
void read_vec4(FILE* f, float out[4]) {
    out[0] = read_f32(f);
    out[1] = read_f32(f);
    out[2] = read_f32(f);
    out[3] = read_f32(f);
}

// Read PMX globals section.
PMXGlobals read_globals(FILE* f) {
    PMXGlobals g;
    uint8_t count = read_u8(f);
    if (count >= 1) g.text_encoding = read_u8(f);
    if (count >= 2) g.additional_uv_count = read_u8(f);
    if (count >= 3) g.vertex_index_size = read_u8(f);
    if (count >= 4) g.texture_index_size = read_u8(f);
    if (count >= 5) g.material_index_size = read_u8(f);
    if (count >= 6) g.bone_index_size = read_u8(f);
    if (count >= 7) g.morph_index_size = read_u8(f);
    if (count >= 8) g.rigid_body_index_size = read_u8(f);
    return g;
}

// ============================================================================
// Text matching
// ============================================================================

std::string to_lower(const std::string& s) {
    std::string r = s;
    std::transform(r.begin(), r.end(), r.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return r;
}

bool name_matches_any(const std::string& name,
                      const std::vector<std::string>& keywords,
                      bool case_sensitive) {
    std::string haystack = case_sensitive ? name : to_lower(name);
    for (const auto& kw : keywords) {
        std::string needle = case_sensitive ? kw : to_lower(kw);
        if (haystack.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

}  // anonymous namespace

// ============================================================================
// Public API
// ============================================================================

PMXModel load_pmx(const std::string& path) {
    PMXModel model;
    FileHandle fh(path);
    if (!fh.valid()) {
        std::cerr << "[PMX] Failed to open: " << path << "\n";
        return {};
    }
    FILE* f = fh.get();

    // Header: "PMX " + version float
    char header[8];
    if (std::fread(header, 1, 8, f) != 8) {
        std::cerr << "[PMX] Failed to read header.\n";
        return {};
    }
    if (std::memcmp(header, "PMX ", 4) != 0) {
        std::cerr << "[PMX] Invalid PMX signature.\n";
        return {};
    }
    float version = *reinterpret_cast<float*>(header + 4);
    if (version < 2.0f || version > 2.1f) {
        std::cerr << "[PMX] Unsupported version: " << version << "\n";
        return {};
    }

    PMXGlobals g = read_globals(f);

    // Model name (local)
    std::string name_local = read_text(f, g);
    // Model name (universal)
    std::string name_universal = read_text(f, g);
    // Model comment (local)
    std::string comment_local = read_text(f, g);
    // Model comment (universal)
    std::string comment_universal = read_text(f, g);

    (void)name_local;
    (void)name_universal;
    (void)comment_local;
    (void)comment_universal;

    // -------------------------------------------------------------------------
    // Vertices
    // -------------------------------------------------------------------------
    int32_t vertex_count = read_i32(f);
    if (vertex_count < 0) {
        std::cerr << "[PMX] Invalid vertex count.\n";
        return {};
    }
    model.vertices.reserve(static_cast<size_t>(vertex_count));

    for (int i = 0; i < vertex_count; ++i) {
        PMXVertex v;
        v.position = read_vec3(f);
        v.normal = read_vec3(f);
        auto uv = read_vec2(f);
        (void)uv;

        // Additional UVs
        for (uint8_t au = 0; au < g.additional_uv_count; ++au) {
            float adduv[4];
            read_vec4(f, adduv);
        }

        uint8_t deform_type = read_u8(f);
        switch (deform_type) {
            case 0: {  // BDEF1
                int b0 = read_index(f, g.bone_index_size);
                v.bone_weights.push_back({b0, 1.0f});
                break;
            }
            case 1: {  // BDEF2
                int b0 = read_index(f, g.bone_index_size);
                int b1 = read_index(f, g.bone_index_size);
                float w0 = read_f32(f);
                v.bone_weights.push_back({b0, w0});
                v.bone_weights.push_back({b1, 1.0f - w0});
                break;
            }
            case 2: {  // BDEF4
                int b[4];
                float w[4];
                for (int j = 0; j < 4; ++j) b[j] = read_index(f, g.bone_index_size);
                for (int j = 0; j < 4; ++j) w[j] = read_f32(f);
                for (int j = 0; j < 4; ++j) {
                    if (w[j] > 0.0f) v.bone_weights.push_back({b[j], w[j]});
                }
                break;
            }
            case 3: {  // SDEF (treat as BDEF2 for extraction)
                int b0 = read_index(f, g.bone_index_size);
                int b1 = read_index(f, g.bone_index_size);
                float w0 = read_f32(f);
                v.bone_weights.push_back({b0, w0});
                v.bone_weights.push_back({b1, 1.0f - w0});
                // Skip C, R0, R1
                float tmp[3];
                read_vec3(f, tmp);
                read_vec3(f, tmp);
                read_vec3(f, tmp);
                break;
            }
            default:
                std::cerr << "[PMX] Unknown deform type " << (int)deform_type
                          << " at vertex " << i << "\n";
                return {};
        }

        float edge_scale = read_f32(f);
        (void)edge_scale;

        model.vertices.push_back(std::move(v));
    }

    // -------------------------------------------------------------------------
    // Faces
    // -------------------------------------------------------------------------
    int32_t face_index_count = read_i32(f);
    if (face_index_count < 0 || face_index_count % 3 != 0) {
        std::cerr << "[PMX] Invalid face index count: " << face_index_count << "\n";
        return {};
    }
    model.faces.reserve(static_cast<size_t>(face_index_count));
    for (int i = 0; i < face_index_count; ++i) {
        int idx = read_index(f, g.vertex_index_size);
        if (idx < 0 || static_cast<size_t>(idx) >= model.vertices.size()) {
            std::cerr << "[PMX] Face index out of range: " << idx << "\n";
            return {};
        }
        model.faces.push_back(static_cast<uint32_t>(idx));
    }

    // -------------------------------------------------------------------------
    // Textures (skip)
    // -------------------------------------------------------------------------
    int32_t texture_count = read_i32(f);
    if (texture_count < 0) {
        std::cerr << "[PMX] Invalid texture count.\n";
        return {};
    }
    for (int i = 0; i < texture_count; ++i) {
        std::string tex_path = read_text(f, g);
        (void)tex_path;
    }

    // -------------------------------------------------------------------------
    // Materials
    // -------------------------------------------------------------------------
    int32_t material_count = read_i32(f);
    if (material_count < 0) {
        std::cerr << "[PMX] Invalid material count.\n";
        return {};
    }
    model.materials.reserve(static_cast<size_t>(material_count));

    uint32_t current_face = 0;
    for (int i = 0; i < material_count; ++i) {
        PMXMaterial mat;
        mat.name_local = read_text(f, g);
        mat.name_universal = read_text(f, g);

        // Diffuse, specular, ambient
        float diffuse[4];
        read_vec4(f, diffuse);
        float specular[3];
        read_vec3(f, specular);
        float specularity = read_f32(f);
        float ambient[3];
        read_vec3(f, ambient);
        (void)specularity;
        (void)ambient;

        uint8_t draw_flags = read_u8(f);
        (void)draw_flags;

        float edge_color[4];
        read_vec4(f, edge_color);
        float edge_size = read_f32(f);
        (void)edge_size;

        // Texture indices
        read_index(f, g.texture_index_size);  // texture
        read_index(f, g.texture_index_size);  // sphere texture
        uint8_t sphere_mode = read_u8(f);
        (void)sphere_mode;

        // Toon
        uint8_t toon_flag = read_u8(f);
        if (toon_flag == 0) {
            read_index(f, g.texture_index_size);
        } else {
            read_u8(f);  // shared toon index 0-9
        }

        std::string memo = read_text(f, g);
        (void)memo;

        int32_t mat_face_count = read_i32(f);
        if (mat_face_count < 0 || mat_face_count % 3 != 0) {
            std::cerr << "[PMX] Invalid material face count.\n";
            return {};
        }

        mat.face_start = current_face;
        mat.face_count = static_cast<uint32_t>(mat_face_count / 3);
        current_face += mat.face_count;

        model.materials.push_back(std::move(mat));
    }

    if (current_face != model.triangle_count()) {
        std::cerr << "[PMX] Material face counts do not sum to total face count.\n";
        return {};
    }

    // -------------------------------------------------------------------------
    // Bones
    // -------------------------------------------------------------------------
    int32_t bone_count = read_i32(f);
    if (bone_count < 0) {
        std::cerr << "[PMX] Invalid bone count.\n";
        return {};
    }
    model.bones.reserve(static_cast<size_t>(bone_count));

    for (int i = 0; i < bone_count; ++i) {
        PMXBone bone;
        bone.name_local = read_text(f, g);
        bone.name_universal = read_text(f, g);
        bone.position = read_vec3(f);

        int parent_index = read_index(f, g.bone_index_size);
        int layer = read_i32(f);
        uint16_t flags = read_le<uint16_t>(f);
        (void)parent_index;
        (void)layer;

        // Connection flag
        bool use_tail_bone = (flags & 0x0001) != 0;
        bool rotatable = (flags & 0x0002) != 0;
        bool translatable = (flags & 0x0004) != 0;
        bool visible = (flags & 0x0008) != 0;
        bool enabled = (flags & 0x0010) != 0;
        bool ik = (flags & 0x0020) != 0;
        bool inherit_rotation = (flags & 0x0100) != 0;
        bool inherit_translation = (flags & 0x0200) != 0;
        bool fixed_axis = (flags & 0x0400) != 0;
        bool local_axis = (flags & 0x0800) != 0;
        bool physics_after_deform = (flags & 0x1000) != 0;
        bool external_parent = (flags & 0x2000) != 0;
        (void)rotatable;
        (void)translatable;
        (void)visible;
        (void)enabled;
        (void)ik;
        bool do_inherit = inherit_rotation || inherit_translation;
        (void)do_inherit;
        (void)fixed_axis;
        (void)local_axis;
        (void)physics_after_deform;
        (void)external_parent;

        if (use_tail_bone) {
            read_index(f, g.bone_index_size);
        } else {
            read_vec3(f);  // tail position
        }

        if (inherit_rotation || inherit_translation) {
            read_index(f, g.bone_index_size);
            read_f32(f);
        }

        if (fixed_axis) {
            read_vec3(f);
        }

        if (local_axis) {
            read_vec3(f);
            read_vec3(f);
        }

        if (external_parent) {
            read_i32(f);
        }

        if (ik) {
            read_index(f, g.bone_index_size);  // ik target
            int32_t iterations = read_i32(f);
            float limit = read_f32(f);
            int32_t link_count = read_i32(f);
            (void)iterations;
            (void)limit;
            for (int j = 0; j < link_count; ++j) {
                read_index(f, g.bone_index_size);
                uint8_t has_limit = read_u8(f);
                if (has_limit) {
                    read_vec3(f);
                    read_vec3(f);
                }
            }
        }

        model.bones.push_back(std::move(bone));
    }

    std::cout << "[PMX] Loaded: " << model.vertices.size() << " vertices, "
              << model.triangle_count() << " triangles, "
              << model.materials.size() << " materials, "
              << model.bones.size() << " bones.\n";

    return model;
}

std::vector<std::tuple<Triangle, vec3f>> pmx_model_to_mesh_data(
    const PMXModel& model) {
    std::vector<std::tuple<Triangle, vec3f>> result;
    result.reserve(model.triangle_count());
    for (size_t i = 0; i + 2 < model.faces.size(); i += 3) {
        uint32_t i0 = model.faces[i];
        uint32_t i1 = model.faces[i + 1];
        uint32_t i2 = model.faces[i + 2];
        if (i0 >= model.vertices.size() || i1 >= model.vertices.size() ||
            i2 >= model.vertices.size()) {
            continue;
        }
        const auto& v0 = model.vertices[i0];
        const auto& v1 = model.vertices[i1];
        const auto& v2 = model.vertices[i2];
        Triangle tri = std::make_tuple(v0.position, v1.position, v2.position);
        vec3f n = (v1.position - v0.position).cross(v2.position - v0.position);
        float nl = n.length();
        if (nl > 1e-12f) n = n * (1.0f / nl);
        result.emplace_back(tri, n);
    }
    return result;
}

std::vector<std::tuple<Triangle, vec3f>> extract_by_bone_names(
    const PMXModel& model,
    const std::vector<std::string>& keywords,
    float threshold,
    bool case_sensitive) {
    if (model.vertices.empty() || model.faces.empty() || keywords.empty()) {
        return {};
    }

    // Build set of matching bone indices.
    std::vector<bool> bone_matches(model.bones.size(), false);
    for (size_t bi = 0; bi < model.bones.size(); ++bi) {
        if (name_matches_any(model.bones[bi].name_local, keywords, case_sensitive) ||
            name_matches_any(model.bones[bi].name_universal, keywords, case_sensitive)) {
            bone_matches[bi] = true;
        }
    }

    // Mark vertices affected by matching bones above threshold.
    std::vector<bool> vertex_affected(model.vertices.size(), false);
    for (size_t vi = 0; vi < model.vertices.size(); ++vi) {
        for (const auto& [bi, w] : model.vertices[vi].bone_weights) {
            if (bi >= 0 && static_cast<size_t>(bi) < bone_matches.size() &&
                bone_matches[bi] && w >= threshold) {
                vertex_affected[vi] = true;
                break;
            }
        }
    }

    // Extract triangles with at least one affected vertex.
    std::vector<std::tuple<Triangle, vec3f>> result;
    result.reserve(model.triangle_count());
    for (size_t i = 0; i + 2 < model.faces.size(); i += 3) {
        uint32_t i0 = model.faces[i];
        uint32_t i1 = model.faces[i + 1];
        uint32_t i2 = model.faces[i + 2];
        if (vertex_affected[i0] || vertex_affected[i1] || vertex_affected[i2]) {
            const auto& v0 = model.vertices[i0];
            const auto& v1 = model.vertices[i1];
            const auto& v2 = model.vertices[i2];
            Triangle tri = std::make_tuple(v0.position, v1.position, v2.position);
            vec3f n = (v1.position - v0.position).cross(v2.position - v0.position);
            float nl = n.length();
            if (nl > 1e-12f) n = n * (1.0f / nl);
            result.emplace_back(tri, n);
        }
    }

    return result;
}

std::vector<std::tuple<Triangle, vec3f>> extract_by_material_names(
    const PMXModel& model,
    const std::vector<std::string>& keywords,
    bool case_sensitive) {
    if (model.faces.empty() || model.materials.empty() || keywords.empty()) {
        return {};
    }

    std::vector<std::tuple<Triangle, vec3f>> result;
    result.reserve(model.triangle_count());

    for (const auto& mat : model.materials) {
        if (!name_matches_any(mat.name_local, keywords, case_sensitive) &&
            !name_matches_any(mat.name_universal, keywords, case_sensitive)) {
            continue;
        }

        uint32_t start = mat.face_start * 3;
        uint32_t end = start + mat.face_count * 3;
        if (end > model.faces.size()) end = static_cast<uint32_t>(model.faces.size());

        for (uint32_t i = start; i < end; i += 3) {
            uint32_t i0 = model.faces[i];
            uint32_t i1 = model.faces[i + 1];
            uint32_t i2 = model.faces[i + 2];
            if (i0 >= model.vertices.size() || i1 >= model.vertices.size() ||
                i2 >= model.vertices.size()) {
                continue;
            }
            const auto& v0 = model.vertices[i0];
            const auto& v1 = model.vertices[i1];
            const auto& v2 = model.vertices[i2];
            Triangle tri = std::make_tuple(v0.position, v1.position, v2.position);
            vec3f n = (v1.position - v0.position).cross(v2.position - v0.position);
            float nl = n.length();
            if (nl > 1e-12f) n = n * (1.0f / nl);
            result.emplace_back(tri, n);
        }
    }

    return result;
}

}  // namespace sinriv::kigstudio::io
