#include <algorithm>
#include <memory>
#include <optional>
#include <queue>
#include <unordered_map>
#include "render_voxel_list.h"
#include "kigstudio/sdf/sdf_mesh.h"
namespace sinriv::ui::render {

namespace {

cJSON* vec2f_to_json(const vec2f& v) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(obj, "x", static_cast<double>(v.x));
    cJSON_AddNumberToObject(obj, "y", static_cast<double>(v.y));
    return obj;
}

vec2f vec2f_from_json(const cJSON* json) {
    vec2f v = {0.0f, 0.0f};
    if (!json) return v;
    cJSON* x = cJSON_GetObjectItem(json, "x");
    cJSON* y = cJSON_GetObjectItem(json, "y");
    if (x && cJSON_IsNumber(x)) v.x = static_cast<float>(x->valuedouble);
    if (y && cJSON_IsNumber(y)) v.y = static_cast<float>(y->valuedouble);
    return v;
}

cJSON* vec2f_array_to_json(const std::vector<vec2f>& points) {
    cJSON* arr = cJSON_CreateArray();
    for (const auto& p : points)
        cJSON_AddItemToArray(arr, vec2f_to_json(p));
    return arr;
}

std::vector<vec2f> vec2f_array_from_json(const cJSON* arr) {
    std::vector<vec2f> points;
    if (!arr || !cJSON_IsArray(arr)) return points;
    int count = cJSON_GetArraySize(arr);
    points.reserve(count);
    for (int i = 0; i < count; ++i)
        points.push_back(vec2f_from_json(cJSON_GetArrayItem(arr, i)));
    return points;
}

bool section_state_has_data(const SectionEditorState& state) {
    return !state.vertices.empty() || !state.committed.empty() ||
           !state.use_bezier_section ||  // true is default, save only when false
           state.normalize_mode != NormalizeMode::NORMALIZE_XY;
}

cJSON* section_state_to_json(const SectionEditorState& state) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddItemToObject(obj, "vertices",
                          vec2f_array_to_json(state.vertices));
    cJSON_AddItemToObject(obj, "committed",
                          vec2f_array_to_json(state.committed));
    cJSON_AddBoolToObject(obj, "use_bezier_section",
                          state.use_bezier_section);
    cJSON_AddNumberToObject(obj, "normalize_mode",
                            static_cast<int>(state.normalize_mode));
    return obj;
}

void section_state_from_json(const cJSON* obj, SectionEditorState& state) {
    if (!obj || !cJSON_IsObject(obj)) return;
    state.vertices =
        vec2f_array_from_json(cJSON_GetObjectItem(obj, "vertices"));
    state.committed =
        vec2f_array_from_json(cJSON_GetObjectItem(obj, "committed"));
    cJSON* bez = cJSON_GetObjectItem(obj, "use_bezier_section");
    if (bez && cJSON_IsBool(bez))
        state.use_bezier_section = cJSON_IsTrue(bez);
    cJSON* norm = cJSON_GetObjectItem(obj, "normalize_mode");
    if (norm && cJSON_IsNumber(norm))
        state.normalize_mode = static_cast<NormalizeMode>(norm->valueint);
}

cJSON* hair_strand_to_json(const HairStrand& strand) {
    cJSON* s_obj = cJSON_CreateObject();
    cJSON_AddStringToObject(s_obj, "name", strand.name.c_str());
    cJSON_AddStringToObject(s_obj, "uuid", strand.uuid.c_str());
    cJSON_AddBoolToObject(s_obj, "expanded", strand.expanded);
    cJSON_AddBoolToObject(s_obj, "visible", strand.visible);
    cJSON_AddBoolToObject(s_obj, "hair_root_enabled", strand.hair_root_enabled);
    cJSON_AddBoolToObject(s_obj, "hair_root_generate", strand.hair_root_generate);
    cJSON_AddNumberToObject(s_obj, "section_rotation",
                            static_cast<double>(strand.section_rotation));
    cJSON_AddNumberToObject(s_obj, "guide_samples_per_segment",
                            strand.guide_samples_per_segment);
    cJSON_AddNumberToObject(s_obj, "section_subdiv", strand.section_subdiv);
    if (section_state_has_data(strand.section_state)) {
        cJSON_AddItemToObject(s_obj, "section_state",
                              section_state_to_json(strand.section_state));
    }
    cJSON* pts_arr = cJSON_CreateArray();
    for (const auto& pt : strand.guide_points) {
        cJSON_AddItemToArray(pts_arr, sinriv::kigstudio::to_json(pt));
    }
    cJSON_AddItemToObject(s_obj, "guide_points", pts_arr);
    // Hidden guide points (participate in lofting, not editable)
    if (!strand.hidden_guide_points_start.empty()) {
        cJSON* hsp_start = cJSON_CreateArray();
        for (const auto& pt : strand.hidden_guide_points_start)
            cJSON_AddItemToArray(hsp_start, sinriv::kigstudio::to_json(pt));
        cJSON_AddItemToObject(s_obj, "hidden_guide_points_start", hsp_start);
    }
    if (!strand.hidden_guide_points_end.empty()) {
        cJSON* hsp_end = cJSON_CreateArray();
        for (const auto& pt : strand.hidden_guide_points_end)
            cJSON_AddItemToArray(hsp_end, sinriv::kigstudio::to_json(pt));
        cJSON_AddItemToObject(s_obj, "hidden_guide_points_end", hsp_end);
    }
    if (!strand.width_points.empty()) {
        cJSON* wp_arr = cJSON_CreateArray();
        for (const auto& wp : strand.width_points) {
            cJSON* wp_obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(wp_obj, "curve_id",
                                    static_cast<double>(wp.curve_id));
            cJSON_AddNumberToObject(wp_obj, "scale",
                                    static_cast<double>(wp.scale));
            cJSON_AddItemToObject(wp_obj, "direction",
                                  sinriv::kigstudio::to_json(wp.direction));
            if (section_state_has_data(wp.section_state)) {
                cJSON_AddItemToObject(
                    wp_obj, "section_state",
                    section_state_to_json(wp.section_state));
            }
            cJSON_AddItemToArray(wp_arr, wp_obj);
        }
        cJSON_AddItemToObject(s_obj, "width_points", wp_arr);
    }
    // Strand generation type and special type parameters
    cJSON_AddNumberToObject(s_obj, "gen_type", static_cast<int>(strand.gen_type));
    // 糖葫芦
    cJSON_AddNumberToObject(s_obj, "candy_cylinder_radius", static_cast<double>(strand.candy_cylinder_radius));
    cJSON_AddNumberToObject(s_obj, "candy_ellipsoid_spacing", static_cast<double>(strand.candy_ellipsoid_spacing));
    cJSON_AddNumberToObject(s_obj, "candy_ellipsoid_radius_a", static_cast<double>(strand.candy_ellipsoid_radius_a));
    cJSON_AddNumberToObject(s_obj, "candy_ellipsoid_radius_b", static_cast<double>(strand.candy_ellipsoid_radius_b));
    cJSON_AddBoolToObject(s_obj, "candy_use_joints", strand.candy_use_joints);
    // 麻花辫
    cJSON_AddNumberToObject(s_obj, "braid_core_radius", static_cast<double>(strand.braid_core_radius));
    cJSON_AddNumberToObject(s_obj, "braid_strand_radius", static_cast<double>(strand.braid_strand_radius));
    cJSON_AddNumberToObject(s_obj, "braid_braid_radius", static_cast<double>(strand.braid_braid_radius));
    cJSON_AddNumberToObject(s_obj, "braid_twist_pitch", static_cast<double>(strand.braid_twist_pitch));
    cJSON_AddNumberToObject(s_obj, "braid_strand_count", strand.braid_strand_count);
    cJSON_AddBoolToObject(s_obj, "braid_use_joints", strand.braid_use_joints);
    // Tip (shared)
    cJSON_AddNumberToObject(s_obj, "special_tip_length", static_cast<double>(strand.special_tip_length));
    cJSON_AddNumberToObject(s_obj, "special_tip_radius", static_cast<double>(strand.special_tip_radius));
    // Tessellation quality (shared by special types)
    cJSON_AddNumberToObject(s_obj, "special_quality", strand.special_quality);
    return s_obj;
}

HairStrand hair_strand_from_json(const cJSON* s_obj) {
    HairStrand strand;
    if (!s_obj || !cJSON_IsObject(s_obj)) return strand;
    cJSON* name_obj = cJSON_GetObjectItem(s_obj, "name");
    if (name_obj && cJSON_IsString(name_obj))
        strand.name = name_obj->valuestring;
    cJSON* uuid_obj = cJSON_GetObjectItem(s_obj, "uuid");
    if (uuid_obj && cJSON_IsString(uuid_obj) && uuid_obj->valuestring[0])
        strand.uuid = uuid_obj->valuestring;
    else
        strand.uuid = generate_uuid();  // backward compat: old project files
    cJSON* exp_obj = cJSON_GetObjectItem(s_obj, "expanded");
    if (exp_obj)
        strand.expanded = exp_obj->valueint != 0;
    cJSON* vis_obj = cJSON_GetObjectItem(s_obj, "visible");
    if (vis_obj)
        strand.visible = vis_obj->valueint != 0;
    cJSON* hre_obj = cJSON_GetObjectItem(s_obj, "hair_root_enabled");
    if (hre_obj && cJSON_IsBool(hre_obj))
        strand.hair_root_enabled = cJSON_IsTrue(hre_obj);
    cJSON* hrg_obj = cJSON_GetObjectItem(s_obj, "hair_root_generate");
    if (hrg_obj && cJSON_IsBool(hrg_obj))
        strand.hair_root_generate = cJSON_IsTrue(hrg_obj);
    cJSON* rot_obj = cJSON_GetObjectItem(s_obj, "section_rotation");
    if (rot_obj && cJSON_IsNumber(rot_obj))
        strand.section_rotation = static_cast<float>(rot_obj->valuedouble);
    cJSON* gsub_obj = cJSON_GetObjectItem(s_obj, "guide_samples_per_segment");
    if (gsub_obj && cJSON_IsNumber(gsub_obj))
        strand.guide_samples_per_segment = std::max(gsub_obj->valueint, 1);
    cJSON* ssub_obj = cJSON_GetObjectItem(s_obj, "section_subdiv");
    if (ssub_obj && cJSON_IsNumber(ssub_obj))
        strand.section_subdiv = std::max(ssub_obj->valueint, 1);
    section_state_from_json(cJSON_GetObjectItem(s_obj, "section_state"),
                            strand.section_state);
    cJSON* pts_arr = cJSON_GetObjectItem(s_obj, "guide_points");
    if (pts_arr && cJSON_IsArray(pts_arr)) {
        int pt_count = cJSON_GetArraySize(pts_arr);
        for (int pi = 0; pi < pt_count; ++pi) {
            cJSON* pt_obj = cJSON_GetArrayItem(pts_arr, pi);
            vec3f pt = sinriv::kigstudio::vec3_from_json<vec3f>(pt_obj);
            strand.guide_points.push_back(pt);
        }
    }
    // Load hidden guide points (start)
    cJSON* hsp_start = cJSON_GetObjectItem(s_obj, "hidden_guide_points_start");
    if (hsp_start && cJSON_IsArray(hsp_start)) {
        int hc = cJSON_GetArraySize(hsp_start);
        for (int hi = 0; hi < hc; ++hi) {
            cJSON* pt_obj = cJSON_GetArrayItem(hsp_start, hi);
            strand.hidden_guide_points_start.push_back(
                sinriv::kigstudio::vec3_from_json<vec3f>(pt_obj));
        }
    }
    // Load hidden guide points (end)
    cJSON* hsp_end = cJSON_GetObjectItem(s_obj, "hidden_guide_points_end");
    if (hsp_end && cJSON_IsArray(hsp_end)) {
        int hc = cJSON_GetArraySize(hsp_end);
        for (int hi = 0; hi < hc; ++hi) {
            cJSON* pt_obj = cJSON_GetArrayItem(hsp_end, hi);
            strand.hidden_guide_points_end.push_back(
                sinriv::kigstudio::vec3_from_json<vec3f>(pt_obj));
        }
    }
    cJSON* wp_arr = cJSON_GetObjectItem(s_obj, "width_points");
    if (wp_arr && cJSON_IsArray(wp_arr)) {
        int wp_count = cJSON_GetArraySize(wp_arr);
        for (int wi = 0; wi < wp_count; ++wi) {
            cJSON* wp_obj = cJSON_GetArrayItem(wp_arr, wi);
            if (!wp_obj) continue;
            HairStrand::WidthPoint wp;
            // 新格式：curve_id（浮点，整数部分=段索引，小数部分=段内t）
            cJSON* cid = cJSON_GetObjectItem(wp_obj, "curve_id");
            if (cid) {
                wp.curve_id = static_cast<float>(cid->valuedouble);
            } else {
                // 兼容旧格式：从 guide_vertex_id 转换
                cJSON* gvi = cJSON_GetObjectItem(wp_obj, "guide_vertex_id");
                if (gvi)
                    wp.curve_id = static_cast<float>(gvi->valuedouble);
            }
            cJSON* sc = cJSON_GetObjectItem(wp_obj, "scale");
            if (sc) wp.scale = static_cast<float>(sc->valuedouble);
            // direction 向量
            cJSON* dir = cJSON_GetObjectItem(wp_obj, "direction");
            if (dir) {
                wp.direction = sinriv::kigstudio::vec3_from_json<vec3f>(dir);
            } else {
                // 兼容旧格式：从 world_pos 和 curve_pos 计算 direction
                cJSON* wpos = cJSON_GetObjectItem(wp_obj, "world_pos");
                cJSON* cpos = cJSON_GetObjectItem(wp_obj, "curve_pos");
                if (wpos && cpos) {
                    vec3f world_pos =
                        sinriv::kigstudio::vec3_from_json<vec3f>(wpos);
                    vec3f curve_pos =
                        sinriv::kigstudio::vec3_from_json<vec3f>(cpos);
                    vec3f diff = world_pos - curve_pos;
                    float dist = diff.length();
                    if (dist > 0.0001f) {
                        wp.direction = diff / dist;
                        if (wp.scale == 1.0f)
                            wp.scale = dist;
                    } else {
                        wp.direction = {0.0f, 1.0f, 0.0f};
                    }
                } else {
                    wp.direction = {0.0f, 1.0f, 0.0f};
                }
            }
            section_state_from_json(
                cJSON_GetObjectItem(wp_obj, "section_state"),
                wp.section_state);
            strand.width_points.push_back(wp);
        }
    }
    // Strand generation type
    cJSON* gt = cJSON_GetObjectItem(s_obj, "gen_type");
    if (gt && cJSON_IsNumber(gt))
        strand.gen_type = static_cast<HairStrandGenType>(gt->valueint);
    // 糖葫芦
    cJSON* ccr = cJSON_GetObjectItem(s_obj, "candy_cylinder_radius");
    if (ccr && cJSON_IsNumber(ccr))
        strand.candy_cylinder_radius = static_cast<float>(ccr->valuedouble);
    cJSON* ces = cJSON_GetObjectItem(s_obj, "candy_ellipsoid_spacing");
    if (ces && cJSON_IsNumber(ces))
        strand.candy_ellipsoid_spacing = static_cast<float>(ces->valuedouble);
    cJSON* era = cJSON_GetObjectItem(s_obj, "candy_ellipsoid_radius_a");
    if (era && cJSON_IsNumber(era))
        strand.candy_ellipsoid_radius_a = static_cast<float>(era->valuedouble);
    cJSON* erb = cJSON_GetObjectItem(s_obj, "candy_ellipsoid_radius_b");
    if (erb && cJSON_IsNumber(erb))
        strand.candy_ellipsoid_radius_b = static_cast<float>(erb->valuedouble);
    cJSON* cuj = cJSON_GetObjectItem(s_obj, "candy_use_joints");
    if (cuj) strand.candy_use_joints = cuj->valueint != 0;
    // 麻花辫
    cJSON* bcr = cJSON_GetObjectItem(s_obj, "braid_core_radius");
    if (bcr && cJSON_IsNumber(bcr))
        strand.braid_core_radius = static_cast<float>(bcr->valuedouble);
    cJSON* bsr = cJSON_GetObjectItem(s_obj, "braid_strand_radius");
    if (bsr && cJSON_IsNumber(bsr))
        strand.braid_strand_radius = static_cast<float>(bsr->valuedouble);
    cJSON* bbr = cJSON_GetObjectItem(s_obj, "braid_braid_radius");
    if (bbr && cJSON_IsNumber(bbr))
        strand.braid_braid_radius = static_cast<float>(bbr->valuedouble);
    cJSON* btp = cJSON_GetObjectItem(s_obj, "braid_twist_pitch");
    if (btp && cJSON_IsNumber(btp))
        strand.braid_twist_pitch = static_cast<float>(btp->valuedouble);
    cJSON* bsc = cJSON_GetObjectItem(s_obj, "braid_strand_count");
    if (bsc && cJSON_IsNumber(bsc))
        strand.braid_strand_count = bsc->valueint;
    cJSON* buj = cJSON_GetObjectItem(s_obj, "braid_use_joints");
    if (buj) strand.braid_use_joints = buj->valueint != 0;
    // Tip (shared)
    cJSON* stl = cJSON_GetObjectItem(s_obj, "special_tip_length");
    if (stl && cJSON_IsNumber(stl))
        strand.special_tip_length = static_cast<float>(stl->valuedouble);
    cJSON* str_ = cJSON_GetObjectItem(s_obj, "special_tip_radius");
    if (str_ && cJSON_IsNumber(str_))
        strand.special_tip_radius = static_cast<float>(str_->valuedouble);
    // Tessellation quality (shared by special types)
    cJSON* sq = cJSON_GetObjectItem(s_obj, "special_quality");
    if (sq && cJSON_IsNumber(sq))
        strand.special_quality = sq->valueint;
    return strand;
}

// ---- DrillPath JSON helpers ----

cJSON* drill_path_to_json(const DrillPath& path) {
    cJSON* p_obj = cJSON_CreateObject();
    cJSON_AddStringToObject(p_obj, "uuid", path.uuid.c_str());
    cJSON_AddStringToObject(p_obj, "name", path.name.c_str());
    cJSON_AddNumberToObject(p_obj, "radius",
                            static_cast<double>(path.radius));
    cJSON_AddBoolToObject(p_obj, "visible", path.visible);
    cJSON* pts_arr = cJSON_CreateArray();
    for (const auto& pt : path.points) {
        cJSON_AddItemToArray(pts_arr, sinriv::kigstudio::to_json(pt));
    }
    cJSON_AddItemToObject(p_obj, "points", pts_arr);
    return p_obj;
}

DrillPath drill_path_from_json(const cJSON* p_obj) {
    DrillPath path;
    if (!p_obj || !cJSON_IsObject(p_obj)) return path;
    cJSON* uuid_obj = cJSON_GetObjectItem(p_obj, "uuid");
    if (uuid_obj && cJSON_IsString(uuid_obj) && uuid_obj->valuestring[0])
        path.uuid = uuid_obj->valuestring;
    else
        path.uuid = generate_uuid();
    cJSON* name_obj = cJSON_GetObjectItem(p_obj, "name");
    if (name_obj && cJSON_IsString(name_obj))
        path.name = name_obj->valuestring;
    cJSON* radius_obj = cJSON_GetObjectItem(p_obj, "radius");
    if (radius_obj && cJSON_IsNumber(radius_obj))
        path.radius = static_cast<float>(radius_obj->valuedouble);
    cJSON* vis_obj = cJSON_GetObjectItem(p_obj, "visible");
    if (vis_obj)
        path.visible = vis_obj->valueint != 0;
    cJSON* pts_arr = cJSON_GetObjectItem(p_obj, "points");
    if (pts_arr && cJSON_IsArray(pts_arr)) {
        int pt_count = cJSON_GetArraySize(pts_arr);
        for (int pi = 0; pi < pt_count; ++pi) {
            path.points.push_back(
                sinriv::kigstudio::vec3_from_json<vec3f>(
                    cJSON_GetArrayItem(pts_arr, pi)));
        }
    }
    path.mesh_dirty = true;
    return path;
}

// ---- mesh binary I/O helpers ----

using MeshTriangle = sinriv::kigstudio::voxel::Triangle;
using MeshVec3f = sinriv::kigstudio::vec3<float>;
using MeshData = std::vector<std::tuple<MeshTriangle, MeshVec3f>>;

// Save cached_mesh (triangles with normals) to binary file
static bool save_mesh_file(const std::filesystem::path& path,
                           const MeshData& mesh) {
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) return false;
    const uint32_t magic = 0x4D5348;  // "MSH"
    const uint32_t version = 1;
    const uint32_t tri_count = static_cast<uint32_t>(mesh.size());
    const uint32_t flags = 1;  // has_normals
    ofs.write(reinterpret_cast<const char*>(&magic), 4);
    ofs.write(reinterpret_cast<const char*>(&version), 4);
    ofs.write(reinterpret_cast<const char*>(&tri_count), 4);
    ofs.write(reinterpret_cast<const char*>(&flags), 4);
    for (const auto& [tri, n] : mesh) {
        const float v[12] = {
            std::get<0>(tri).x, std::get<0>(tri).y, std::get<0>(tri).z,
            std::get<1>(tri).x, std::get<1>(tri).y, std::get<1>(tri).z,
            std::get<2>(tri).x, std::get<2>(tri).y, std::get<2>(tri).z,
            n.x, n.y, n.z,
        };
        ofs.write(reinterpret_cast<const char*>(v), sizeof(v));
    }
    return ofs.good();
}

// Save source_triangles (no normals) to binary file
static bool save_mesh_file(const std::filesystem::path& path,
                           const std::vector<MeshTriangle>& tris) {
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) return false;
    const uint32_t magic = 0x4D5348;
    const uint32_t version = 1;
    const uint32_t tri_count = static_cast<uint32_t>(tris.size());
    const uint32_t flags = 0;  // no normals
    ofs.write(reinterpret_cast<const char*>(&magic), 4);
    ofs.write(reinterpret_cast<const char*>(&version), 4);
    ofs.write(reinterpret_cast<const char*>(&tri_count), 4);
    ofs.write(reinterpret_cast<const char*>(&flags), 4);
    for (const auto& tri : tris) {
        const float v[9] = {
            std::get<0>(tri).x, std::get<0>(tri).y, std::get<0>(tri).z,
            std::get<1>(tri).x, std::get<1>(tri).y, std::get<1>(tri).z,
            std::get<2>(tri).x, std::get<2>(tri).y, std::get<2>(tri).z,
        };
        ofs.write(reinterpret_cast<const char*>(v), sizeof(v));
    }
    return ofs.good();
}

// Load mesh from binary file. Returns mesh data with normals.
// If source had no normals, computes face normals.
static MeshData load_mesh_file(const std::filesystem::path& path, bool& ok) {
    ok = false;
    MeshData result;
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) return result;

    uint32_t magic = 0, version = 0, tri_count = 0, flags = 0;
    ifs.read(reinterpret_cast<char*>(&magic), 4);
    ifs.read(reinterpret_cast<char*>(&version), 4);
    ifs.read(reinterpret_cast<char*>(&tri_count), 4);
    ifs.read(reinterpret_cast<char*>(&flags), 4);

    if (magic != 0x4D5348 || version != 1 || tri_count > 100'000'000)
        return result;

    const bool has_normals = (flags & 1) != 0;
    result.reserve(tri_count);

    if (has_normals) {
        for (uint32_t i = 0; i < tri_count; ++i) {
            float v[12];
            ifs.read(reinterpret_cast<char*>(v), sizeof(v));
            if (!ifs) return result;
            MeshTriangle tri{MeshVec3f{v[0], v[1], v[2]},
                             MeshVec3f{v[3], v[4], v[5]},
                             MeshVec3f{v[6], v[7], v[8]}};
            MeshVec3f n{v[9], v[10], v[11]};
            result.emplace_back(tri, n);
        }
    } else {
        for (uint32_t i = 0; i < tri_count; ++i) {
            float v[9];
            ifs.read(reinterpret_cast<char*>(v), sizeof(v));
            if (!ifs) return result;
            MeshTriangle tri{MeshVec3f{v[0], v[1], v[2]},
                             MeshVec3f{v[3], v[4], v[5]},
                             MeshVec3f{v[6], v[7], v[8]}};
            MeshVec3f a = std::get<0>(tri);
            MeshVec3f b = std::get<1>(tri);
            MeshVec3f c = std::get<2>(tri);
            MeshVec3f n = (b - a).cross(c - a);
            float len = n.length();
            if (len > 1e-8f)
                n = n / len;
            else
                n = MeshVec3f{0, 0, 0};
            result.emplace_back(tri, n);
        }
    }
    ok = ifs.good() || ifs.eof();
    return result;
}

}  // namespace

cJSON* RenderVoxelList::item_to_json(const RenderVoxelItem& item) const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(obj, "id", item.id);
    cJSON* children = cJSON_CreateArray();
    for (int child_id : item.children) {
        cJSON_AddItemToArray(children, cJSON_CreateNumber(child_id));
    }
    cJSON_AddItemToObject(obj, "children", children);
    cJSON* nav_pos = cJSON_CreateArray();
    // 力导向下保存当前浮点位置（同步到整型快照以保持兼容）
    int save_x = this->nav_layout_force_directed
                     ? static_cast<int>(item.nav_layout_pos[0])
                     : item.nav_node_position[0];
    int save_y = this->nav_layout_force_directed
                     ? static_cast<int>(item.nav_layout_pos[1])
                     : item.nav_node_position[1];
    cJSON_AddItemToArray(nav_pos, cJSON_CreateNumber(save_x));
    cJSON_AddItemToArray(nav_pos, cJSON_CreateNumber(save_y));
    cJSON_AddItemToObject(obj, "nav_node_position", nav_pos);
    const char* mode_str;
    switch (item.segment_mode) {
        case RenderVoxelItem::COLLISION:
            mode_str = "collision";
            break;
        case RenderVoxelItem::PLANE:
            mode_str = "plane";
            break;
        case RenderVoxelItem::CONCAVE_CONE:
            mode_str = "concave_cone";
            break;
        case RenderVoxelItem::SPLIT_DISCONNECTED:
            mode_str = "split_disconnected";
            break;
        case RenderVoxelItem::NEIGHBOR:
            mode_str = "neighbor";
            break;
        case RenderVoxelItem::FILL_INTERIOR:
            mode_str = "fill_interior";
            break;
        case RenderVoxelItem::CHAIN:
            mode_str = "chain";
            break;
        case RenderVoxelItem::SDF_NODE_SPLIT:
            mode_str = "sdf_node_split";
            break;
        case RenderVoxelItem::SUBDIVIDE_MESH:
            mode_str = "subdivide_mesh";
            break;
        case RenderVoxelItem::REPAIR_MESH:
            mode_str = "repair_mesh";
            break;
        case RenderVoxelItem::SILHOUETTE:
            mode_str = "silhouette";
            break;
        default:
            mode_str = "collision";
            break;
    }
    cJSON_AddStringToObject(obj, "segment_mode", mode_str);
    cJSON_AddNumberToObject(obj, "repair_mode", item.repair_mode);
    cJSON_AddNumberToObject(obj, "sdf_split_target_id",
                            item.sdf_split_target_id);
    cJSON_AddItemToObject(
        obj, "sdf_split_translation",
        sinriv::kigstudio::to_json(item.sdf_split_translation));
    cJSON_AddItemToObject(
        obj, "sdf_split_rotation",
        sinriv::kigstudio::to_json(item.sdf_split_rotation));
    cJSON_AddItemToObject(
        obj, "sdf_split_scale",
        sinriv::kigstudio::to_json(item.sdf_split_scale));
    cJSON_AddBoolToObject(obj, "show_origin_mesh", item.showOriginMesh);
    cJSON_AddBoolToObject(obj, "show_mesh", item.showMesh);
    cJSON_AddBoolToObject(obj, "show_exported_mesh", item.showExportedMesh);
    cJSON_AddBoolToObject(obj, "show_voxel", item.showVoxel);
    cJSON_AddBoolToObject(obj, "show_collision", item.showCollision);
    cJSON_AddBoolToObject(obj, "show_collision_bounds",
                          item.showCollisionBounds);
    cJSON_AddBoolToObject(obj, "auto_segment_update",
                          item.auto_segment_update);
    cJSON_AddBoolToObject(obj, "voxel_picking_enabled",
                          item.voxel_picking_enabled);
    cJSON_AddNumberToObject(obj, "voxel_pick_range", item.voxel_pick_range);
    cJSON_AddNumberToObject(obj, "neighbor_max_distance",
                            item.neighbor_max_distance);
    cJSON_AddNumberToObject(obj, "chain_min_radius",
                            item.chain_min_radius);
    cJSON_AddBoolToObject(obj, "has_marked_voxels",
                          !item.marked_voxels.empty());
    cJSON_AddStringToObject(obj, "stl_path", item.stl_path.c_str());
    cJSON_AddStringToObject(obj, "voxel_path", item.voxel_path.c_str());
    cJSON_AddNumberToObject(obj, "stl_voxel_size", item.stl_voxel_size);
    cJSON_AddNumberToObject(obj, "stl_load_mode", item.stl_load_mode);
    cJSON_AddBoolToObject(obj, "load_as_sdf", item.load_as_sdf);
    cJSON_AddNumberToObject(obj, "voxel_precision",
                            static_cast<int>(item.voxel_precision));
    cJSON_AddBoolToObject(obj, "mesh_only", item.mesh_only);
    cJSON_AddNumberToObject(obj, "source_type", item.source_type);
    cJSON_AddNumberToObject(obj, "source_node_id", item.source_node_id);
    cJSON_AddNumberToObject(obj, "addon_base_node_id", item.addon_base_node_id);
    cJSON_AddNumberToObject(obj, "addon_type", item.addon_type);
    cJSON_AddBoolToObject(obj, "addon_reveal", item.addon_reveal);
    cJSON_AddBoolToObject(obj, "addon_split", item.addon_split);
    cJSON_AddBoolToObject(obj, "addon_sdf_boolean", item.addon_sdf_boolean);
    cJSON_AddBoolToObject(obj, "addon_sdf_split", item.addon_sdf_split);
    // hair strands
    if (!item.hair_strands.empty()) {
        cJSON* strands_arr = cJSON_CreateArray();
        for (const auto& strand : item.hair_strands) {
            cJSON_AddItemToArray(strands_arr, hair_strand_to_json(strand));
        }
        cJSON_AddItemToObject(obj, "hair_strands", strands_arr);
    }
    cJSON_AddNumberToObject(obj, "node_source_data_type",
                            item.node_source_data_type);
    cJSON_AddNumberToObject(obj, "node_source_sdf_subdivisions",
                            item.node_source_sdf_subdivisions);
    cJSON_AddBoolToObject(obj, "node_source_sdf_simplify",
                          item.node_source_sdf_simplify);
    cJSON_AddNumberToObject(obj, "node_source_sdf_simplify_ratio",
                            item.node_source_sdf_simplify_ratio);
    cJSON_AddItemToObject(
        obj, "silhouette_center",
        sinriv::kigstudio::to_json(item.silhouette_center));
    cJSON_AddBoolToObject(obj, "show_silhouette_center",
                          item.showSilhouetteCenter);
    cJSON_AddItemToObject(
        obj, "addon_center_point",
        sinriv::kigstudio::to_json(item.addon_center_point));
    cJSON_AddBoolToObject(obj, "show_addon_center",
                          item.show_addon_center);
    cJSON_AddBoolToObject(obj, "auto_hair_root", item.auto_hair_root);
    cJSON_AddItemToObject(obj, "common_hair_root_point",
                          sinriv::kigstudio::to_json(item.common_hair_root_point));
    cJSON_AddNumberToObject(obj, "hair_root_center_offset",
                            static_cast<double>(item.hair_root_center_offset));
    cJSON_AddNumberToObject(obj, "hair_root_vector_length",
                            static_cast<double>(item.hair_root_vector_length));
    cJSON_AddBoolToObject(obj, "show_connection_faces",
                          item.show_connection_faces);
    if (!item.drill_paths.empty()) {
        cJSON* drill_arr = cJSON_CreateArray();
        for (const auto& path : item.drill_paths)
            cJSON_AddItemToArray(drill_arr, drill_path_to_json(path));
        cJSON_AddItemToObject(obj, "drill_paths", drill_arr);
    }
    cJSON_AddBoolToObject(obj, "hairline_plane_enabled",
                          item.hairline_plane_enabled);
    cJSON_AddBoolToObject(obj, "hairline_plane_use_y",
                          item.hairline_plane_use_y);
    cJSON_AddNumberToObject(obj, "hairline_plane_y",
                            item.hairline_plane_y);
    cJSON_AddNumberToObject(obj, "hairline_spindle_scale",
                            item.hairline_spindle_scale);
    cJSON_AddItemToObject(
        obj, "hairline_plane_p0",
        sinriv::kigstudio::to_json(item.hairline_plane_points[0]));
    cJSON_AddItemToObject(
        obj, "hairline_plane_p1",
        sinriv::kigstudio::to_json(item.hairline_plane_points[1]));
    cJSON_AddItemToObject(
        obj, "hairline_plane_p2",
        sinriv::kigstudio::to_json(item.hairline_plane_points[2]));
    // Semantic coordinate angle config
    cJSON_AddItemToObject(
        obj, "hair_north_pole",
        sinriv::kigstudio::to_json(item.hair_north_pole));
    cJSON_AddItemToObject(
        obj, "hair_front_reference",
        sinriv::kigstudio::to_json(item.hair_front_reference));
    if (!item.hair_angle_config.empty()) {
        cJSON* ac_arr = cJSON_CreateArray();
        for (const auto& [key, entry] : item.hair_angle_config) {
            cJSON* ac_obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(ac_obj, "x", key.first);
            cJSON_AddNumberToObject(ac_obj, "y", key.second);
            cJSON_AddNumberToObject(ac_obj, "theta", entry.theta);
            cJSON_AddNumberToObject(ac_obj, "phi", entry.phi);
            cJSON_AddItemToArray(ac_arr, ac_obj);
        }
        cJSON_AddItemToObject(obj, "hair_angle_config", ac_arr);
    }
    // Ortho overlay states (per six-view, per-node)
    {
        cJSON* overlay_arr = cJSON_CreateArray();
        for (int vi = 0; vi < 6; ++vi) {
            const auto& ol = item.ortho_overlay[vi];
            if (ol.image_path.empty() && !ol.enabled) continue;  // skip unused
            cJSON* ol_obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(ol_obj, "view_index", vi);
            cJSON_AddStringToObject(ol_obj, "image_path", ol.image_path.c_str());
            cJSON_AddNumberToObject(ol_obj, "img_width", ol.img_width);
            cJSON_AddNumberToObject(ol_obj, "img_height", ol.img_height);
            cJSON_AddBoolToObject(ol_obj, "enabled", ol.enabled);
            cJSON_AddNumberToObject(ol_obj, "offset_x", ol.offset_x);
            cJSON_AddNumberToObject(ol_obj, "offset_y", ol.offset_y);
            cJSON_AddNumberToObject(ol_obj, "scale_x", ol.scale_x);
            cJSON_AddNumberToObject(ol_obj, "scale_y", ol.scale_y);
            cJSON_AddNumberToObject(ol_obj, "blend_ratio", ol.blend_ratio);
            cJSON_AddBoolToObject(ol_obj, "locked", ol.locked);
            cJSON_AddItemToArray(overlay_arr, ol_obj);
        }
        if (cJSON_GetArraySize(overlay_arr) > 0)
            cJSON_AddItemToObject(obj, "ortho_overlay", overlay_arr);
        else
            cJSON_Delete(overlay_arr);
        cJSON_AddNumberToObject(obj, "ortho_viewport_size",
                                static_cast<double>(item.ortho_viewport_size));
        cJSON_AddNumberToObject(obj, "ortho_render_resolution",
                                item.ortho_render_resolution);
    }
    cJSON_AddNumberToObject(obj, "silhouette_shape_mode",
                            static_cast<int>(item.silhouette_shape_mode));
    cJSON_AddNumberToObject(obj, "silhouette_subdivision",
                            item.silhouette_subdivision);
    cJSON_AddNumberToObject(obj, "silhouette_edge_subdiv",
                            item.silhouette_edge_subdiv);
    cJSON_AddNumberToObject(obj, "inner_wall_radius",
                            item.inner_wall_radius);
    cJSON_AddNumberToObject(obj, "simplify_ratio",
                            item.simplify_ratio);
    cJSON_AddNumberToObject(obj, "alpha_wrap_alpha",
                            item.alpha_wrap_alpha);
    cJSON_AddNumberToObject(obj, "alpha_wrap_offset",
                            item.alpha_wrap_offset);
    cJSON_AddNumberToObject(obj, "subdivide_level",
                            item.subdivide_level);
    cJSON_AddStringToObject(obj, "err_info", item.err_info.c_str());
    cJSON_AddStringToObject(obj, "title", item.title.c_str());
    cJSON_AddStringToObject(obj, "comment_text", item.comment_text.c_str());
    cJSON_AddItemToObject(obj, "collision_group",
                          sinriv::kigstudio::to_json(item.collision_group));
    cJSON_AddItemToObject(obj, "plane", sinriv::kigstudio::to_json(item.plane));
    cJSON_AddItemToObject(
        obj, "concave_cone",
        sinriv::kigstudio::voxel::concave::to_json(item.concave_cone));
    cJSON* expanded = cJSON_CreateArray();
    for (int v : item.concave_cone_expanded_vertices) {
        cJSON_AddItemToArray(expanded, cJSON_CreateNumber(v));
    }
    cJSON_AddItemToObject(obj, "concave_cone_expanded_vertices", expanded);
    cJSON_AddItemToObject(
        obj, "voxel_global_position",
        sinriv::kigstudio::to_json(item.voxel_grid_data.global_position));
    cJSON_AddItemToObject(
        obj, "voxel_size",
        sinriv::kigstudio::to_json(item.voxel_grid_data.voxel_size));
    cJSON_AddBoolToObject(obj, "use_cgal_skeleton",
                          item.use_cgal_skeleton);
    cJSON* skeleton_points = cJSON_CreateArray();
    for (const auto& sp : item.picked_skeleton_points) {
        cJSON* sp_obj = cJSON_CreateObject();
        cJSON_AddItemToObject(
            sp_obj, "position",
            sinriv::kigstudio::to_json(sp.position));
        cJSON_AddNumberToObject(sp_obj, "order", sp.order);
        cJSON_AddBoolToObject(sp_obj, "use_custom_direction",
                              sp.use_custom_direction);
        cJSON_AddItemToObject(
            sp_obj, "custom_direction_end",
            sinriv::kigstudio::to_json(sp.custom_direction_end));
        cJSON_AddNumberToObject(sp_obj, "socket_cone_offset",
                                sp.socket_cone_offset);
        cJSON_AddNumberToObject(sp_obj, "socket_cone_angle",
                                sp.socket_cone_angle);
        cJSON_AddNumberToObject(sp_obj, "socket_cone_radius",
                                sp.socket_cone_radius);
        cJSON_AddNumberToObject(sp_obj, "head_cone_offset",
                                sp.head_cone_offset);
        cJSON_AddNumberToObject(sp_obj, "head_cone_radius",
                                sp.head_cone_radius);
        cJSON_AddNumberToObject(sp_obj, "socket_support_offset",
                                sp.socket_support_offset);
        cJSON_AddNumberToObject(sp_obj, "socket_support_radius",
                                sp.socket_support_radius);
        cJSON_AddNumberToObject(sp_obj, "head_support_offset",
                                sp.head_support_offset);
        cJSON_AddNumberToObject(sp_obj, "head_support_radius",
                                sp.head_support_radius);
        cJSON_AddNumberToObject(sp_obj, "male_cylinder_offset",
                                sp.male_cylinder_offset);
        cJSON_AddNumberToObject(sp_obj, "male_cylinder_radius",
                                sp.male_cylinder_radius);
        cJSON_AddNumberToObject(sp_obj, "female_gap",
                                sp.female_gap);
        cJSON_AddNumberToObject(sp_obj, "slot_extra",
                                sp.slot_extra);
        cJSON_AddNumberToObject(sp_obj, "socket_fillet_radius",
                                sp.socket_fillet_radius);
        cJSON_AddNumberToObject(sp_obj, "socket_fillet_height",
                                sp.socket_fillet_height);
        cJSON_AddNumberToObject(sp_obj, "socket_fillet_offset",
                                sp.socket_fillet_offset);
        cJSON_AddNumberToObject(sp_obj, "head_fillet_height",
                                sp.head_fillet_height);
        cJSON_AddNumberToObject(sp_obj, "rotation_angle",
                                sp.rotation_angle);
        cJSON_AddItemToArray(skeleton_points, sp_obj);
    }
    cJSON_AddItemToObject(obj, "picked_skeleton_points",
                          skeleton_points);
    return obj;
}

std::unique_ptr<RenderVoxelList::RenderVoxelItem>
RenderVoxelList::item_from_json(const cJSON* obj) {
    if (!obj || !cJSON_IsObject(obj))
        return nullptr;

    auto item = std::make_unique<RenderVoxelItem>();
    item->manager = this;

    // 与旧实现保持一致的缺失默认值
    item->sdf_split_target_id = -1;
    item->showOriginMesh = false;
    item->showMesh = true;
    item->showExportedMesh = true;
    item->showVoxel = true;
    item->showCollision = true;
    item->showCollisionBounds = true;
    item->voxel_precision =
        sinriv::kigstudio::sdf::SDFPrecision::Fast;

    auto parse_skeleton_point = [](const cJSON* sp_obj) -> SkeletonPointPick {
        SkeletonPointPick sp;
        const cJSON* child = nullptr;
        cJSON_ArrayForEach(child, sp_obj) {
            if (!child->string)
                continue;
            if (cJSON_IsObject(child)) {
                if (strcmp(child->string, "position") == 0) {
                    sp.position = sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::voxel::vec3f>(child);
                } else if (strcmp(child->string, "custom_direction_end") == 0) {
                    sp.custom_direction_end = sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::voxel::vec3f>(child);
                }
            } else if (cJSON_IsNumber(child)) {
                const double value = cJSON_GetNumberValue(child);
                if (strcmp(child->string, "order") == 0) {
                    sp.order = static_cast<int>(value);
                } else if (strcmp(child->string, "socket_cone_offset") == 0) {
                    sp.socket_cone_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_cone_angle") == 0) {
                    sp.socket_cone_angle = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_cone_radius") == 0) {
                    sp.socket_cone_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "head_cone_offset") == 0) {
                    sp.head_cone_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "head_cone_radius") == 0) {
                    sp.head_cone_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_support_offset") == 0) {
                    sp.socket_support_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_support_radius") == 0) {
                    sp.socket_support_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "head_support_offset") == 0) {
                    sp.head_support_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "head_support_radius") == 0) {
                    sp.head_support_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "male_cylinder_offset") == 0) {
                    sp.male_cylinder_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "male_cylinder_radius") == 0) {
                    sp.male_cylinder_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "female_gap") == 0) {
                    sp.female_gap = static_cast<float>(value);
                } else if (strcmp(child->string, "slot_extra") == 0) {
                    sp.slot_extra = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_fillet_radius") == 0) {
                    sp.socket_fillet_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_fillet_height") == 0) {
                    sp.socket_fillet_height = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_fillet_offset") == 0) {
                    sp.socket_fillet_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "head_fillet_height") == 0) {
                    sp.head_fillet_height = static_cast<float>(value);
                } else if (strcmp(child->string, "rotation_angle") == 0) {
                    sp.rotation_angle = static_cast<float>(value);
                }
            } else if (cJSON_IsBool(child)) {
                if (strcmp(child->string, "use_custom_direction") == 0) {
                    sp.use_custom_direction = cJSON_IsTrue(child);
                }
            }
        }
        return sp;
    };

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, obj) {
        if (!child->string)
            continue;

        const char* key = child->string;

        if (strcmp(key, "id") == 0 && cJSON_IsNumber(child)) {
            item->id = child->valueint;
        } else if (strcmp(key, "children") == 0 && cJSON_IsArray(child)) {
            int children_count = cJSON_GetArraySize(child);
            item->children.clear();
            for (int i = 0; i < children_count; ++i) {
                const cJSON* c = cJSON_GetArrayItem(child, i);
                if (c && cJSON_IsNumber(c))
                    item->children.push_back(c->valueint);
            }
        } else if (strcmp(key, "nav_node_position") == 0 &&
                   cJSON_IsArray(child)) {
            if (cJSON_GetArraySize(child) >= 2) {
                const cJSON* x = cJSON_GetArrayItem(child, 0);
                const cJSON* y = cJSON_GetArrayItem(child, 1);
                if (x && cJSON_IsNumber(x) && y && cJSON_IsNumber(y)) {
                    item->nav_node_position[0] = x->valueint;
                    item->nav_node_position[1] = y->valueint;
                }
            }
        } else if (strcmp(key, "segment_mode") == 0 &&
                   cJSON_IsString(child)) {
            const char* mode_str = child->valuestring;
            if (strcmp(mode_str, "collision") == 0) {
                item->segment_mode = RenderVoxelItem::COLLISION;
            } else if (strcmp(mode_str, "plane") == 0) {
                item->segment_mode = RenderVoxelItem::PLANE;
            } else if (strcmp(mode_str, "concave_cone") == 0) {
                item->segment_mode = RenderVoxelItem::CONCAVE_CONE;
            } else if (strcmp(mode_str, "split_disconnected") == 0) {
                item->segment_mode = RenderVoxelItem::SPLIT_DISCONNECTED;
            } else if (strcmp(mode_str, "neighbor") == 0) {
                item->segment_mode = RenderVoxelItem::NEIGHBOR;
            } else if (strcmp(mode_str, "fill_interior") == 0) {
                item->segment_mode = RenderVoxelItem::FILL_INTERIOR;
            } else if (strcmp(mode_str, "chain") == 0) {
                item->segment_mode = RenderVoxelItem::CHAIN;
            } else if (strcmp(mode_str, "sdf_node_split") == 0) {
                item->segment_mode = RenderVoxelItem::SDF_NODE_SPLIT;
            } else if (strcmp(mode_str, "subdivide_mesh") == 0) {
                item->segment_mode = RenderVoxelItem::SUBDIVIDE_MESH;
            } else if (strcmp(mode_str, "repair_mesh") == 0) {
                item->segment_mode = RenderVoxelItem::REPAIR_MESH;
            } else if (strcmp(mode_str, "silhouette") == 0) {
                item->segment_mode = RenderVoxelItem::SILHOUETTE;
            } else {
                item->segment_mode = RenderVoxelItem::COLLISION;
            }
        } else if (cJSON_IsNumber(child)) {
            const double value = cJSON_GetNumberValue(child);
            if (strcmp(key, "sdf_split_target_id") == 0) {
                item->sdf_split_target_id = child->valueint;
            } else if (strcmp(key, "voxel_pick_range") == 0) {
                item->voxel_pick_range = static_cast<float>(value);
            } else if (strcmp(key, "neighbor_max_distance") == 0) {
                item->neighbor_max_distance = child->valueint;
            } else if (strcmp(key, "chain_min_radius") == 0) {
                item->chain_min_radius = child->valueint;
            } else if (strcmp(key, "stl_voxel_size") == 0) {
                item->stl_voxel_size = static_cast<float>(value);
            } else if (strcmp(key, "stl_load_mode") == 0) {
                item->stl_load_mode = child->valueint;
            } else if (strcmp(key, "source_type") == 0) {
                item->source_type = child->valueint;
            } else if (strcmp(key, "source_node_id") == 0) {
                item->source_node_id = child->valueint;
            } else if (strcmp(key, "addon_base_node_id") == 0) {
                item->addon_base_node_id = child->valueint;
            } else if (strcmp(key, "addon_type") == 0) {
                item->addon_type = child->valueint;
            } else if (strcmp(key, "node_source_data_type") == 0) {
                item->node_source_data_type = child->valueint;
            } else if (strcmp(key, "node_source_sdf_subdivisions") == 0) {
                item->node_source_sdf_subdivisions = child->valueint;
            } else if (strcmp(key, "silhouette_shape_mode") == 0) {
                item->silhouette_shape_mode =
                    static_cast<SilhouetteShapeMode>(child->valueint);
            } else if (strcmp(key, "silhouette_subdivision") == 0) {
                item->silhouette_subdivision = child->valueint;
            } else if (strcmp(key, "silhouette_edge_subdiv") == 0) {
                item->silhouette_edge_subdiv = child->valueint;
            } else if (strcmp(key, "inner_wall_radius") == 0) {
                item->inner_wall_radius = static_cast<float>(value);
            } else if (strcmp(key, "simplify_ratio") == 0) {
                item->simplify_ratio = static_cast<float>(value);
            } else if (strcmp(key, "repair_mode") == 0) {
                item->repair_mode =
                    static_cast<RenderVoxelItem::RepairMeshMode>(
                        child->valueint);
            } else if (strcmp(key, "alpha_wrap_alpha") == 0) {
                item->alpha_wrap_alpha = static_cast<float>(value);
            } else if (strcmp(key, "alpha_wrap_offset") == 0) {
                item->alpha_wrap_offset = static_cast<float>(value);
            } else if (strcmp(key, "subdivide_level") == 0) {
                item->subdivide_level = child->valueint;
            } else if (strcmp(key, "node_source_sdf_simplify_ratio") == 0) {
                item->node_source_sdf_simplify_ratio =
                    static_cast<float>(value);
            }
        } else if (cJSON_IsBool(child)) {
            if (strcmp(key, "show_origin_mesh") == 0) {
                item->showOriginMesh = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_mesh") == 0) {
                item->showMesh = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_exported_mesh") == 0) {
                item->showExportedMesh = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_voxel") == 0) {
                item->showVoxel = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_collision") == 0) {
                item->showCollision = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_collision_bounds") == 0) {
                item->showCollisionBounds = cJSON_IsTrue(child);
            } else if (strcmp(key, "auto_segment_update") == 0) {
                item->auto_segment_update = cJSON_IsTrue(child);
            } else if (strcmp(key, "load_as_sdf") == 0) {
                item->load_as_sdf = cJSON_IsTrue(child);
            } else if (strcmp(key, "voxel_precision") == 0) {
                if (cJSON_IsBool(child))
                    item->voxel_precision = cJSON_IsTrue(child)
                        ? sinriv::kigstudio::sdf::SDFPrecision::Precise
                        : sinriv::kigstudio::sdf::SDFPrecision::Fast;
                else if (cJSON_IsNumber(child)) {
                    int v = child->valueint;
                    if (v >= 0 && v <= 2)
                        item->voxel_precision =
                            static_cast<sinriv::kigstudio::sdf::SDFPrecision>(v);
                }
            } else if (strcmp(key, "mesh_only") == 0) {
                item->mesh_only = cJSON_IsTrue(child);
            } else if (strcmp(key, "node_source_sdf_simplify") == 0) {
                item->node_source_sdf_simplify = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_silhouette_center") == 0) {
                item->showSilhouetteCenter = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_addon_center") == 0) {
                item->show_addon_center = cJSON_IsTrue(child);
            } else if (strcmp(key, "auto_hair_root") == 0) {
                item->auto_hair_root = cJSON_IsTrue(child);
            } else if (strcmp(key, "hair_root_center_offset") == 0) {
                item->hair_root_center_offset = static_cast<float>(child->valuedouble);
            } else if (strcmp(key, "hair_root_vector_length") == 0) {
                item->hair_root_vector_length = static_cast<float>(child->valuedouble);
            } else if (strcmp(key, "show_connection_faces") == 0) {
                item->show_connection_faces = cJSON_IsTrue(child);
            } else if (strcmp(key, "drill_paths") == 0 && cJSON_IsArray(child)) {
                int dp_count = cJSON_GetArraySize(child);
                for (int di = 0; di < dp_count; ++di) {
                    item->drill_paths.push_back(drill_path_from_json(
                        cJSON_GetArrayItem(child, di)));
                }
            } else if (strcmp(key, "hairline_plane_enabled") == 0) {
                item->hairline_plane_enabled = cJSON_IsTrue(child);
            } else if (strcmp(key, "hairline_plane_use_y") == 0) {
                item->hairline_plane_use_y = cJSON_IsTrue(child);
            } else if (strcmp(key, "hairline_plane_y") == 0) {
                item->hairline_plane_y =
                    static_cast<float>(child->valuedouble);
            } else if (strcmp(key, "hairline_spindle_scale") == 0) {
                item->hairline_spindle_scale =
                    static_cast<float>(child->valuedouble);
            } else if (strcmp(key, "voxel_picking_enabled") == 0) {
                item->voxel_picking_enabled = cJSON_IsTrue(child);
            } else if (strcmp(key, "use_cgal_skeleton") == 0) {
                item->use_cgal_skeleton = cJSON_IsTrue(child);
            } else if (strcmp(key, "addon_reveal") == 0) {
                item->addon_reveal = cJSON_IsTrue(child);
            } else if (strcmp(key, "addon_split") == 0) {
                item->addon_split = cJSON_IsTrue(child);
            } else if (strcmp(key, "addon_sdf_boolean") == 0) {
                item->addon_sdf_boolean = cJSON_IsTrue(child);
            } else if (strcmp(key, "addon_sdf_split") == 0) {
                item->addon_sdf_split = cJSON_IsTrue(child);
            }
        } else if (cJSON_IsString(child)) {
            if (strcmp(key, "stl_path") == 0) {
                item->stl_path = child->valuestring;
            } else if (strcmp(key, "voxel_path") == 0) {
                item->voxel_path = child->valuestring;
            } else if (strcmp(key, "err_info") == 0) {
                item->err_info = child->valuestring;
            } else if (strcmp(key, "title") == 0) {
                item->title = child->valuestring;
            } else if (strcmp(key, "comment_text") == 0) {
                item->comment_text = child->valuestring;
            }
        } else if (cJSON_IsObject(child)) {
            if (strcmp(key, "sdf_split_translation") == 0) {
                item->sdf_split_translation =
                    sinriv::kigstudio::vec3_from_json<vec3f>(child);
            } else if (strcmp(key, "sdf_split_rotation") == 0) {
                item->sdf_split_rotation =
                    sinriv::kigstudio::vec3_from_json<vec3f>(child);
            } else if (strcmp(key, "sdf_split_scale") == 0) {
                item->sdf_split_scale =
                    sinriv::kigstudio::vec3_from_json<vec3f>(child);
            } else if (strcmp(key, "silhouette_center") == 0) {
                item->silhouette_center =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "addon_center_point") == 0) {
                item->addon_center_point =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "common_hair_root_point") == 0) {
                item->common_hair_root_point =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "hairline_plane_p0") == 0) {
                item->hairline_plane_points[0] =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "hairline_plane_p1") == 0) {
                item->hairline_plane_points[1] =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "hairline_plane_p2") == 0) {
                item->hairline_plane_points[2] =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "hair_north_pole") == 0) {
                item->hair_north_pole =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "hair_front_reference") == 0) {
                item->hair_front_reference =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "voxel_global_position") == 0) {
                item->voxel_grid_data.global_position =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "voxel_size") == 0) {
                item->voxel_grid_data.voxel_size =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "collision_group") == 0) {
                item->collision_group =
                    sinriv::kigstudio::from_json_collision_group(child);
            } else if (strcmp(key, "plane") == 0) {
                item->plane = sinriv::kigstudio::from_json_plane(child);
            } else if (strcmp(key, "concave_cone") == 0) {
                item->concave_cone =
                    sinriv::kigstudio::voxel::concave::from_json_cone(child);
            }
        } else if (cJSON_IsArray(child)) {
            if (strcmp(key, "concave_cone_expanded_vertices") == 0) {
                int expanded_count = cJSON_GetArraySize(child);
                for (int i = 0; i < expanded_count; ++i) {
                    const cJSON* v = cJSON_GetArrayItem(child, i);
                    if (v && cJSON_IsNumber(v))
                        item->concave_cone_expanded_vertices.push_back(
                            v->valueint);
                }
            } else if (strcmp(key, "picked_skeleton_points") == 0) {
                int sp_count = cJSON_GetArraySize(child);
                for (int i = 0; i < sp_count; ++i) {
                    const cJSON* sp_obj = cJSON_GetArrayItem(child, i);
                    if (sp_obj && cJSON_IsObject(sp_obj)) {
                        item->picked_skeleton_points.push_back(
                            parse_skeleton_point(sp_obj));
                    }
                }
            } else if (strcmp(key, "hair_strands") == 0) {
                item->hair_strands.clear();
                int strand_count = cJSON_GetArraySize(child);
                for (int si = 0; si < strand_count; ++si) {
                    item->hair_strands.push_back(hair_strand_from_json(
                        cJSON_GetArrayItem(child, si)));
                }
            } else if (strcmp(key, "hair_angle_config") == 0) {
                item->hair_angle_config.clear();
                int ac_n = cJSON_GetArraySize(child);
                for (int ai = 0; ai < ac_n; ++ai) {
                    cJSON* ac_obj = cJSON_GetArrayItem(child, ai);
                    if (!cJSON_IsObject(ac_obj)) continue;
                    float ax = static_cast<float>(
                        cJSON_GetObjectItem(ac_obj, "x")->valuedouble);
                    float ay = static_cast<float>(
                        cJSON_GetObjectItem(ac_obj, "y")->valuedouble);
                    HairAngleEntry ae;
                    ae.theta = static_cast<float>(
                        cJSON_GetObjectItem(ac_obj, "theta")->valuedouble);
                    ae.phi = static_cast<float>(
                        cJSON_GetObjectItem(ac_obj, "phi")->valuedouble);
                    // (0,0) is the origin anchor; theta must always be 0°
                    if (ax == 0.0f && ay == 0.0f)
                        ae.theta = 0.0f;
                    item->hair_angle_config[{ax, ay}] = ae;
                }
            }
        }
    }

    // Ortho overlay states (per six-view)
    {
        const cJSON* overlay_arr = cJSON_GetObjectItem(obj, "ortho_overlay");
        if (overlay_arr && cJSON_IsArray(overlay_arr)) {
            int count = cJSON_GetArraySize(overlay_arr);
            for (int i = 0; i < count; ++i) {
                const cJSON* ol_obj = cJSON_GetArrayItem(overlay_arr, i);
                if (!ol_obj || !cJSON_IsObject(ol_obj)) continue;
                const cJSON* vi_json = cJSON_GetObjectItem(ol_obj, "view_index");
                if (!vi_json || !cJSON_IsNumber(vi_json)) continue;
                int vi = vi_json->valueint;
                if (vi < 0 || vi >= 6) continue;
                auto& ol = item->ortho_overlay[vi];
                const cJSON* ip = cJSON_GetObjectItem(ol_obj, "image_path");
                if (ip && cJSON_IsString(ip)) ol.image_path = ip->valuestring;
                const cJSON* iw = cJSON_GetObjectItem(ol_obj, "img_width");
                if (iw && cJSON_IsNumber(iw)) ol.img_width = iw->valueint;
                const cJSON* ih = cJSON_GetObjectItem(ol_obj, "img_height");
                if (ih && cJSON_IsNumber(ih)) ol.img_height = ih->valueint;
                const cJSON* en = cJSON_GetObjectItem(ol_obj, "enabled");
                if (en && cJSON_IsBool(en)) ol.enabled = cJSON_IsTrue(en);
                const cJSON* ox = cJSON_GetObjectItem(ol_obj, "offset_x");
                if (ox && cJSON_IsNumber(ox)) ol.offset_x = static_cast<float>(ox->valuedouble);
                const cJSON* oy = cJSON_GetObjectItem(ol_obj, "offset_y");
                if (oy && cJSON_IsNumber(oy)) ol.offset_y = static_cast<float>(oy->valuedouble);
                const cJSON* scx = cJSON_GetObjectItem(ol_obj, "scale_x");
                const cJSON* scy = cJSON_GetObjectItem(ol_obj, "scale_y");
                if (scx && cJSON_IsNumber(scx)) ol.scale_x = static_cast<float>(scx->valuedouble);
                if (scy && cJSON_IsNumber(scy)) ol.scale_y = static_cast<float>(scy->valuedouble);
                // Backwards compat: old single "scale" field
                if (!scx && !scy) {
                    const cJSON* sc = cJSON_GetObjectItem(ol_obj, "scale");
                    if (sc && cJSON_IsNumber(sc))
                        ol.scale_x = ol.scale_y = static_cast<float>(sc->valuedouble);
                }
                const cJSON* br = cJSON_GetObjectItem(ol_obj, "blend_ratio");
                if (br && cJSON_IsNumber(br)) ol.blend_ratio = static_cast<float>(br->valuedouble);
                const cJSON* lk = cJSON_GetObjectItem(ol_obj, "locked");
                if (lk && cJSON_IsBool(lk)) ol.locked = cJSON_IsTrue(lk);
            }
        }
        // Ortho editor global settings
        const cJSON* ovs = cJSON_GetObjectItem(obj, "ortho_viewport_size");
        if (ovs && cJSON_IsNumber(ovs))
            item->ortho_viewport_size = static_cast<float>(ovs->valuedouble);
        const cJSON* orr = cJSON_GetObjectItem(obj, "ortho_render_resolution");
        if (orr && cJSON_IsNumber(orr))
            item->ortho_render_resolution = orr->valueint;
    }

    // nav_layout_pos 与 nav_node_position 同轴
    item->nav_layout_pos[0] = (float)item->nav_node_position[0];
    item->nav_layout_pos[1] = (float)item->nav_node_position[1];
    item->nav_layout_vel[0] = 0.0f;
    item->nav_layout_vel[1] = 0.0f;
    item->nav_layout_pinned = false;
    item->nav_layout_pos_set = true;

    return item;
}

cJSON* RenderVoxelList::snapshot_to_json(
    const CollisionEditorSnapshot& snapshot) const {
    cJSON* obj = cJSON_CreateObject();

    const char* mode_str;
    switch (static_cast<RenderVoxelItem::SegmentMode>(snapshot.segment_mode)) {
        case RenderVoxelItem::COLLISION:
            mode_str = "collision";
            break;
        case RenderVoxelItem::PLANE:
            mode_str = "plane";
            break;
        case RenderVoxelItem::CONCAVE_CONE:
            mode_str = "concave_cone";
            break;
        case RenderVoxelItem::SPLIT_DISCONNECTED:
            mode_str = "split_disconnected";
            break;
        case RenderVoxelItem::NEIGHBOR:
            mode_str = "neighbor";
            break;
        case RenderVoxelItem::FILL_INTERIOR:
            mode_str = "fill_interior";
            break;
        case RenderVoxelItem::CHAIN:
            mode_str = "chain";
            break;
        case RenderVoxelItem::SDF_NODE_SPLIT:
            mode_str = "sdf_node_split";
            break;
        case RenderVoxelItem::SUBDIVIDE_MESH:
            mode_str = "subdivide_mesh";
            break;
        case RenderVoxelItem::REPAIR_MESH:
            mode_str = "repair_mesh";
            break;
        case RenderVoxelItem::SILHOUETTE:
            mode_str = "silhouette";
            break;
        default:
            mode_str = "collision";
            break;
    }
    cJSON_AddStringToObject(obj, "segment_mode", mode_str);

    // 公共源数据配置（各模式通用）
    cJSON_AddStringToObject(obj, "stl_path", snapshot.stl_path.c_str());
    cJSON_AddNumberToObject(obj, "stl_load_mode", snapshot.stl_load_mode);
    cJSON_AddBoolToObject(obj, "load_as_sdf", snapshot.load_as_sdf);
    cJSON_AddNumberToObject(obj, "voxel_precision",
                            static_cast<int>(snapshot.voxel_precision));
    cJSON_AddBoolToObject(obj, "mesh_only", snapshot.mesh_only);
    cJSON_AddNumberToObject(obj, "source_type", snapshot.source_type);
    cJSON_AddNumberToObject(obj, "source_node_id", snapshot.source_node_id);
    cJSON_AddBoolToObject(obj, "addon_reveal", snapshot.addon_reveal);
    cJSON_AddBoolToObject(obj, "addon_split", snapshot.addon_split);
    cJSON_AddBoolToObject(obj, "addon_sdf_boolean",
                          snapshot.addon_sdf_boolean);
    cJSON_AddBoolToObject(obj, "addon_sdf_split", snapshot.addon_sdf_split);
    // Semantic coordinate angle config
    cJSON_AddItemToObject(obj, "hair_north_pole",
                          sinriv::kigstudio::to_json(snapshot.hair_north_pole));
    cJSON_AddItemToObject(
        obj, "hair_front_reference",
        sinriv::kigstudio::to_json(snapshot.hair_front_reference));
    cJSON_AddNumberToObject(obj, "addon_base_node_id",
                            snapshot.addon_base_node_id);
    if (!snapshot.hair_angle_config.empty()) {
        cJSON* ac_arr = cJSON_CreateArray();
        for (const auto& [key, entry] : snapshot.hair_angle_config) {
            cJSON* ac_obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(ac_obj, "x", key.first);
            cJSON_AddNumberToObject(ac_obj, "y", key.second);
            cJSON_AddNumberToObject(ac_obj, "theta", entry.theta);
            cJSON_AddNumberToObject(ac_obj, "phi", entry.phi);
            cJSON_AddItemToArray(ac_arr, ac_obj);
        }
        cJSON_AddItemToObject(obj, "hair_angle_config", ac_arr);
    }
    cJSON_AddNumberToObject(obj, "node_source_data_type",
                            snapshot.node_source_data_type);
    cJSON_AddNumberToObject(obj, "node_source_sdf_subdivisions",
                            snapshot.node_source_sdf_subdivisions);
    cJSON_AddBoolToObject(obj, "node_source_sdf_simplify",
                          snapshot.node_source_sdf_simplify);
    cJSON_AddNumberToObject(obj, "node_source_sdf_simplify_ratio",
                            snapshot.node_source_sdf_simplify_ratio);
    cJSON_AddItemToObject(
        obj, "silhouette_center",
        sinriv::kigstudio::to_json(snapshot.silhouette_center));
    cJSON_AddBoolToObject(obj, "show_silhouette_center",
                          snapshot.show_silhouette_center);
    cJSON_AddItemToObject(
        obj, "addon_center_point",
        sinriv::kigstudio::to_json(snapshot.addon_center_point));
    cJSON_AddBoolToObject(obj, "show_addon_center",
                          snapshot.show_addon_center);
    cJSON_AddBoolToObject(obj, "auto_hair_root", snapshot.auto_hair_root);
    cJSON_AddItemToObject(obj, "common_hair_root_point",
                          sinriv::kigstudio::to_json(snapshot.common_hair_root_point));
    cJSON_AddNumberToObject(obj, "hair_root_center_offset",
                            static_cast<double>(snapshot.hair_root_center_offset));
    cJSON_AddNumberToObject(obj, "hair_root_vector_length",
                            static_cast<double>(snapshot.hair_root_vector_length));
    cJSON_AddBoolToObject(obj, "show_connection_faces",
                          snapshot.show_connection_faces);
    if (!snapshot.drill_paths.empty()) {
        cJSON* drill_arr = cJSON_CreateArray();
        for (const auto& path : snapshot.drill_paths)
            cJSON_AddItemToArray(drill_arr, drill_path_to_json(path));
        cJSON_AddItemToObject(obj, "drill_paths", drill_arr);
    }
    cJSON_AddBoolToObject(obj, "hairline_plane_enabled",
                          snapshot.hairline_plane_enabled);
    cJSON_AddBoolToObject(obj, "hairline_plane_use_y",
                          snapshot.hairline_plane_use_y);
    cJSON_AddNumberToObject(obj, "hairline_plane_y",
                            snapshot.hairline_plane_y);
    cJSON_AddNumberToObject(obj, "hairline_spindle_scale",
                            snapshot.hairline_spindle_scale);
    cJSON_AddItemToObject(
        obj, "hairline_plane_p0",
        sinriv::kigstudio::to_json(snapshot.hairline_plane_points[0]));
    cJSON_AddItemToObject(
        obj, "hairline_plane_p1",
        sinriv::kigstudio::to_json(snapshot.hairline_plane_points[1]));
    cJSON_AddItemToObject(
        obj, "hairline_plane_p2",
        sinriv::kigstudio::to_json(snapshot.hairline_plane_points[2]));
    cJSON_AddNumberToObject(obj, "silhouette_shape_mode",
                            static_cast<int>(snapshot.silhouette_shape_mode));
    cJSON_AddNumberToObject(obj, "silhouette_subdivision",
                            snapshot.silhouette_subdivision);
    cJSON_AddNumberToObject(obj, "silhouette_edge_subdiv",
                            snapshot.silhouette_edge_subdiv);
    cJSON_AddNumberToObject(obj, "inner_wall_radius",
                            snapshot.inner_wall_radius);
    cJSON_AddNumberToObject(obj, "simplify_ratio",
                            snapshot.simplify_ratio);
    cJSON_AddNumberToObject(obj, "repair_mode",
                            snapshot.repair_mode);
    cJSON_AddNumberToObject(obj, "alpha_wrap_alpha",
                            snapshot.alpha_wrap_alpha);
    cJSON_AddNumberToObject(obj, "alpha_wrap_offset",
                            snapshot.alpha_wrap_offset);
    cJSON_AddNumberToObject(obj, "subdivide_level",
                            snapshot.subdivide_level);

    // 仅输出当前 segment_mode 相关的编辑字段
    const auto mode =
        static_cast<RenderVoxelItem::SegmentMode>(snapshot.segment_mode);
    if (mode == RenderVoxelItem::COLLISION) {
        cJSON_AddItemToObject(
            obj, "collision_group",
            sinriv::kigstudio::to_json(snapshot.collision_group));
    } else if (mode == RenderVoxelItem::PLANE) {
        cJSON_AddItemToObject(obj, "plane",
                              sinriv::kigstudio::to_json(snapshot.plane));
    } else if (mode == RenderVoxelItem::CONCAVE_CONE) {
        cJSON_AddItemToObject(
            obj, "concave_cone",
            sinriv::kigstudio::voxel::concave::to_json(snapshot.concave_cone));
        cJSON* expanded = cJSON_CreateArray();
        for (int v : snapshot.concave_cone_expanded_vertices) {
            cJSON_AddItemToArray(expanded, cJSON_CreateNumber(v));
        }
        cJSON_AddItemToObject(obj, "concave_cone_expanded_vertices", expanded);
    } else if (mode == RenderVoxelItem::SDF_NODE_SPLIT) {
        cJSON_AddNumberToObject(obj, "sdf_split_target_id",
                                snapshot.sdf_split_target_id);
        cJSON_AddItemToObject(
            obj, "sdf_split_translation",
            sinriv::kigstudio::to_json(snapshot.sdf_split_translation));
        cJSON_AddItemToObject(
            obj, "sdf_split_rotation",
            sinriv::kigstudio::to_json(snapshot.sdf_split_rotation));
        cJSON_AddItemToObject(
            obj, "sdf_split_scale",
            sinriv::kigstudio::to_json(snapshot.sdf_split_scale));
    } else if (mode == RenderVoxelItem::CHAIN) {
        cJSON_AddNumberToObject(obj, "chain_min_radius",
                                snapshot.chain_min_radius);
        cJSON_AddBoolToObject(obj, "use_cgal_skeleton",
                              snapshot.use_cgal_skeleton);

        cJSON* skeleton_points = cJSON_CreateArray();
        for (const auto& sp : snapshot.picked_skeleton_points) {
            cJSON* sp_obj = cJSON_CreateObject();
            cJSON_AddItemToObject(
                sp_obj, "position",
                sinriv::kigstudio::to_json(sp.position));
            cJSON_AddNumberToObject(sp_obj, "order", sp.order);
            cJSON_AddBoolToObject(sp_obj, "use_custom_direction",
                                  sp.use_custom_direction);
            cJSON_AddItemToObject(
                sp_obj, "custom_direction_end",
                sinriv::kigstudio::to_json(sp.custom_direction_end));
            cJSON_AddNumberToObject(sp_obj, "socket_cone_offset",
                                    sp.socket_cone_offset);
            cJSON_AddNumberToObject(sp_obj, "socket_cone_angle",
                                    sp.socket_cone_angle);
            cJSON_AddNumberToObject(sp_obj, "socket_cone_radius",
                                    sp.socket_cone_radius);
            cJSON_AddNumberToObject(sp_obj, "head_cone_offset",
                                    sp.head_cone_offset);
            cJSON_AddNumberToObject(sp_obj, "head_cone_radius",
                                    sp.head_cone_radius);
            cJSON_AddNumberToObject(sp_obj, "socket_support_offset",
                                    sp.socket_support_offset);
            cJSON_AddNumberToObject(sp_obj, "socket_support_radius",
                                    sp.socket_support_radius);
            cJSON_AddNumberToObject(sp_obj, "head_support_offset",
                                    sp.head_support_offset);
            cJSON_AddNumberToObject(sp_obj, "head_support_radius",
                                    sp.head_support_radius);
            cJSON_AddNumberToObject(sp_obj, "male_cylinder_offset",
                                    sp.male_cylinder_offset);
            cJSON_AddNumberToObject(sp_obj, "male_cylinder_radius",
                                    sp.male_cylinder_radius);
            cJSON_AddNumberToObject(sp_obj, "female_gap", sp.female_gap);
            cJSON_AddNumberToObject(sp_obj, "slot_extra", sp.slot_extra);
            cJSON_AddNumberToObject(sp_obj, "socket_fillet_radius",
                                    sp.socket_fillet_radius);
            cJSON_AddNumberToObject(sp_obj, "socket_fillet_height",
                                    sp.socket_fillet_height);
            cJSON_AddNumberToObject(sp_obj, "socket_fillet_offset",
                                    sp.socket_fillet_offset);
            cJSON_AddNumberToObject(sp_obj, "head_fillet_height",
                                    sp.head_fillet_height);
            cJSON_AddNumberToObject(sp_obj, "rotation_angle",
                                    sp.rotation_angle);
            cJSON_AddItemToArray(skeleton_points, sp_obj);
        }
        cJSON_AddItemToObject(obj, "picked_skeleton_points", skeleton_points);

        cJSON* skeleton_lines = cJSON_CreateArray();
        for (const auto& line : snapshot.skeleton_lines) {
            cJSON* line_obj = cJSON_CreateObject();
            cJSON_AddItemToObject(line_obj, "start",
                                  sinriv::kigstudio::to_json(line.first));
            cJSON_AddItemToObject(line_obj, "end",
                                  sinriv::kigstudio::to_json(line.second));
            cJSON_AddItemToArray(skeleton_lines, line_obj);
        }
        cJSON_AddItemToObject(obj, "skeleton_lines", skeleton_lines);
    }

    return obj;
}

std::optional<CollisionEditorSnapshot> RenderVoxelList::snapshot_from_json(
    const cJSON* obj) const {
    if (!obj || !cJSON_IsObject(obj))
        return std::nullopt;

    CollisionEditorSnapshot snapshot;
    snapshot.segment_mode = RenderVoxelItem::COLLISION;

    auto parse_skeleton_point = [](const cJSON* sp_obj) -> SkeletonPointPick {
        SkeletonPointPick sp;
        const cJSON* child = nullptr;
        cJSON_ArrayForEach(child, sp_obj) {
            if (!child->string)
                continue;
            if (cJSON_IsObject(child)) {
                if (strcmp(child->string, "position") == 0) {
                    sp.position = sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::voxel::vec3f>(child);
                } else if (strcmp(child->string, "custom_direction_end") == 0) {
                    sp.custom_direction_end = sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::voxel::vec3f>(child);
                }
            } else if (cJSON_IsNumber(child)) {
                const double value = cJSON_GetNumberValue(child);
                if (strcmp(child->string, "order") == 0) {
                    sp.order = static_cast<int>(value);
                } else if (strcmp(child->string, "socket_cone_offset") == 0) {
                    sp.socket_cone_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_cone_angle") == 0) {
                    sp.socket_cone_angle = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_cone_radius") == 0) {
                    sp.socket_cone_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "head_cone_offset") == 0) {
                    sp.head_cone_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "head_cone_radius") == 0) {
                    sp.head_cone_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_support_offset") == 0) {
                    sp.socket_support_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_support_radius") == 0) {
                    sp.socket_support_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "head_support_offset") == 0) {
                    sp.head_support_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "head_support_radius") == 0) {
                    sp.head_support_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "male_cylinder_offset") == 0) {
                    sp.male_cylinder_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "male_cylinder_radius") == 0) {
                    sp.male_cylinder_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "female_gap") == 0) {
                    sp.female_gap = static_cast<float>(value);
                } else if (strcmp(child->string, "slot_extra") == 0) {
                    sp.slot_extra = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_fillet_radius") == 0) {
                    sp.socket_fillet_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_fillet_height") == 0) {
                    sp.socket_fillet_height = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_fillet_offset") == 0) {
                    sp.socket_fillet_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "head_fillet_height") == 0) {
                    sp.head_fillet_height = static_cast<float>(value);
                } else if (strcmp(child->string, "rotation_angle") == 0) {
                    sp.rotation_angle = static_cast<float>(value);
                }
            } else if (cJSON_IsBool(child)) {
                if (strcmp(child->string, "use_custom_direction") == 0) {
                    sp.use_custom_direction = cJSON_IsTrue(child);
                }
            }
        }
        return sp;
    };

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, obj) {
        if (!child->string)
            continue;

        const char* key = child->string;

        if (strcmp(key, "segment_mode") == 0 && cJSON_IsString(child)) {
            const char* mode_str = child->valuestring;
            if (strcmp(mode_str, "collision") == 0) {
                snapshot.segment_mode = RenderVoxelItem::COLLISION;
            } else if (strcmp(mode_str, "plane") == 0) {
                snapshot.segment_mode = RenderVoxelItem::PLANE;
            } else if (strcmp(mode_str, "concave_cone") == 0) {
                snapshot.segment_mode = RenderVoxelItem::CONCAVE_CONE;
            } else if (strcmp(mode_str, "split_disconnected") == 0) {
                snapshot.segment_mode = RenderVoxelItem::SPLIT_DISCONNECTED;
            } else if (strcmp(mode_str, "neighbor") == 0) {
                snapshot.segment_mode = RenderVoxelItem::NEIGHBOR;
            } else if (strcmp(mode_str, "fill_interior") == 0) {
                snapshot.segment_mode = RenderVoxelItem::FILL_INTERIOR;
            } else if (strcmp(mode_str, "chain") == 0) {
                snapshot.segment_mode = RenderVoxelItem::CHAIN;
            } else if (strcmp(mode_str, "sdf_node_split") == 0) {
                snapshot.segment_mode = RenderVoxelItem::SDF_NODE_SPLIT;
            } else if (strcmp(mode_str, "subdivide_mesh") == 0) {
                snapshot.segment_mode = RenderVoxelItem::SUBDIVIDE_MESH;
            } else if (strcmp(mode_str, "repair_mesh") == 0) {
                snapshot.segment_mode = RenderVoxelItem::REPAIR_MESH;
            } else if (strcmp(mode_str, "silhouette") == 0) {
                snapshot.segment_mode = RenderVoxelItem::SILHOUETTE;
            } else {
                snapshot.segment_mode = RenderVoxelItem::COLLISION;
            }
        } else if (cJSON_IsNumber(child)) {
            const double value = cJSON_GetNumberValue(child);
            if (strcmp(key, "sdf_split_target_id") == 0) {
                snapshot.sdf_split_target_id = child->valueint;
            } else if (strcmp(key, "chain_min_radius") == 0) {
                snapshot.chain_min_radius = child->valueint;
            } else if (strcmp(key, "stl_load_mode") == 0) {
                snapshot.stl_load_mode = child->valueint;
            } else if (strcmp(key, "source_type") == 0) {
                snapshot.source_type = child->valueint;
            } else if (strcmp(key, "source_node_id") == 0) {
                snapshot.source_node_id = child->valueint;
            } else if (strcmp(key, "node_source_data_type") == 0) {
                snapshot.node_source_data_type = child->valueint;
            } else if (strcmp(key, "node_source_sdf_subdivisions") == 0) {
                snapshot.node_source_sdf_subdivisions = child->valueint;
            } else if (strcmp(key, "silhouette_shape_mode") == 0) {
                snapshot.silhouette_shape_mode =
                    static_cast<SilhouetteShapeMode>(child->valueint);
            } else if (strcmp(key, "silhouette_subdivision") == 0) {
                snapshot.silhouette_subdivision = child->valueint;
            } else if (strcmp(key, "silhouette_edge_subdiv") == 0) {
                snapshot.silhouette_edge_subdiv = child->valueint;
            } else if (strcmp(key, "inner_wall_radius") == 0) {
                snapshot.inner_wall_radius = static_cast<float>(value);
            } else if (strcmp(key, "node_source_sdf_simplify_ratio") == 0) {
                snapshot.node_source_sdf_simplify_ratio =
                    static_cast<float>(value);
            } else if (strcmp(key, "repair_mode") == 0) {
                snapshot.repair_mode = child->valueint;
            } else if (strcmp(key, "alpha_wrap_alpha") == 0) {
                snapshot.alpha_wrap_alpha = static_cast<float>(value);
            } else if (strcmp(key, "alpha_wrap_offset") == 0) {
                snapshot.alpha_wrap_offset = static_cast<float>(value);
            } else if (strcmp(key, "subdivide_level") == 0) {
                snapshot.subdivide_level = child->valueint;
            }
        } else if (cJSON_IsBool(child)) {
            if (strcmp(key, "use_cgal_skeleton") == 0) {
                snapshot.use_cgal_skeleton = cJSON_IsTrue(child);
            } else if (strcmp(key, "load_as_sdf") == 0) {
                snapshot.load_as_sdf = cJSON_IsTrue(child);
            } else if (strcmp(key, "voxel_precision") == 0) {
                if (cJSON_IsBool(child))
                    snapshot.voxel_precision = cJSON_IsTrue(child)
                        ? sinriv::kigstudio::sdf::SDFPrecision::Precise
                        : sinriv::kigstudio::sdf::SDFPrecision::Fast;
                else if (cJSON_IsNumber(child)) {
                    int v = child->valueint;
                    if (v >= 0 && v <= 2)
                        snapshot.voxel_precision =
                            static_cast<sinriv::kigstudio::sdf::SDFPrecision>(v);
                }
            } else if (strcmp(key, "mesh_only") == 0) {
                snapshot.mesh_only = cJSON_IsTrue(child);
            } else if (strcmp(key, "node_source_sdf_simplify") == 0) {
                snapshot.node_source_sdf_simplify = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_silhouette_center") == 0) {
                snapshot.show_silhouette_center = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_addon_center") == 0) {
                snapshot.show_addon_center = cJSON_IsTrue(child);
            } else if (strcmp(key, "auto_hair_root") == 0) {
                snapshot.auto_hair_root = cJSON_IsTrue(child);
            } else if (strcmp(key, "hair_root_center_offset") == 0) {
                snapshot.hair_root_center_offset = static_cast<float>(child->valuedouble);
            } else if (strcmp(key, "hair_root_vector_length") == 0) {
                snapshot.hair_root_vector_length = static_cast<float>(child->valuedouble);
            } else if (strcmp(key, "show_connection_faces") == 0) {
                snapshot.show_connection_faces = cJSON_IsTrue(child);
            } else if (strcmp(key, "drill_paths") == 0 && cJSON_IsArray(child)) {
                int dp_count = cJSON_GetArraySize(child);
                for (int di = 0; di < dp_count; ++di) {
                    snapshot.drill_paths.push_back(drill_path_from_json(
                        cJSON_GetArrayItem(child, di)));
                }
            } else if (strcmp(key, "hairline_plane_enabled") == 0) {
                snapshot.hairline_plane_enabled = cJSON_IsTrue(child);
            } else if (strcmp(key, "hairline_plane_use_y") == 0) {
                snapshot.hairline_plane_use_y = cJSON_IsTrue(child);
            } else if (strcmp(key, "hairline_plane_y") == 0) {
                snapshot.hairline_plane_y =
                    static_cast<float>(child->valuedouble);
            } else if (strcmp(key, "hairline_spindle_scale") == 0) {
                snapshot.hairline_spindle_scale =
                    static_cast<float>(child->valuedouble);
            } else if (strcmp(key, "hairline_plane_p0") == 0) {
                snapshot.hairline_plane_points[0] =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "hairline_plane_p1") == 0) {
                snapshot.hairline_plane_points[1] =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "hairline_plane_p2") == 0) {
                snapshot.hairline_plane_points[2] =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "addon_reveal") == 0) {
                snapshot.addon_reveal = cJSON_IsTrue(child);
            } else if (strcmp(key, "addon_split") == 0) {
                snapshot.addon_split = cJSON_IsTrue(child);
            } else if (strcmp(key, "addon_sdf_boolean") == 0) {
                snapshot.addon_sdf_boolean = cJSON_IsTrue(child);
            } else if (strcmp(key, "addon_sdf_split") == 0) {
                snapshot.addon_sdf_split = cJSON_IsTrue(child);
            } else if (strcmp(key, "hair_north_pole") == 0) {
                snapshot.hair_north_pole =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "hair_front_reference") == 0) {
                snapshot.hair_front_reference =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "addon_base_node_id") == 0) {
                snapshot.addon_base_node_id = child->valueint;
            } else if (strcmp(key, "hair_angle_config") == 0) {
                snapshot.hair_angle_config.clear();
                if (cJSON_IsArray(child)) {
                    int ac_n = cJSON_GetArraySize(child);
                    for (int ai = 0; ai < ac_n; ++ai) {
                        cJSON* ac_obj = cJSON_GetArrayItem(child, ai);
                        if (!cJSON_IsObject(ac_obj)) continue;
                        float ax = static_cast<float>(
                            cJSON_GetObjectItem(ac_obj, "x")->valuedouble);
                        float ay = static_cast<float>(
                            cJSON_GetObjectItem(ac_obj, "y")->valuedouble);
                        HairAngleEntry ae;
                        ae.theta = static_cast<float>(
                            cJSON_GetObjectItem(ac_obj, "theta")->valuedouble);
                        ae.phi = static_cast<float>(
                            cJSON_GetObjectItem(ac_obj, "phi")->valuedouble);
                        // (0,0) is the origin anchor; theta must always be 0°
                        if (ax == 0.0f && ay == 0.0f)
                            ae.theta = 0.0f;
                        snapshot.hair_angle_config[{ax, ay}] = ae;
                    }
                }
            }
        } else if (cJSON_IsString(child)) {
            if (strcmp(key, "stl_path") == 0) {
                snapshot.stl_path = child->valuestring;
            }
        } else if (cJSON_IsObject(child)) {
            if (strcmp(key, "collision_group") == 0) {
                snapshot.collision_group =
                    sinriv::kigstudio::from_json_collision_group(child);
            } else if (strcmp(key, "plane") == 0) {
                snapshot.plane = sinriv::kigstudio::from_json_plane(child);
            } else if (strcmp(key, "concave_cone") == 0) {
                snapshot.concave_cone =
                    sinriv::kigstudio::voxel::concave::from_json_cone(child);
            } else if (strcmp(key, "sdf_split_translation") == 0) {
                snapshot.sdf_split_translation =
                    sinriv::kigstudio::vec3_from_json<vec3f>(child);
            } else if (strcmp(key, "sdf_split_rotation") == 0) {
                snapshot.sdf_split_rotation =
                    sinriv::kigstudio::vec3_from_json<vec3f>(child);
            } else if (strcmp(key, "sdf_split_scale") == 0) {
                snapshot.sdf_split_scale =
                    sinriv::kigstudio::vec3_from_json<vec3f>(child);
            } else if (strcmp(key, "silhouette_center") == 0) {
                snapshot.silhouette_center =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "addon_center_point") == 0) {
                snapshot.addon_center_point =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "common_hair_root_point") == 0) {
                snapshot.common_hair_root_point =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            }
        } else if (cJSON_IsArray(child)) {
            if (strcmp(key, "concave_cone_expanded_vertices") == 0) {
                int expanded_count = cJSON_GetArraySize(child);
                for (int i = 0; i < expanded_count; ++i) {
                    const cJSON* v = cJSON_GetArrayItem(child, i);
                    if (v && cJSON_IsNumber(v))
                        snapshot.concave_cone_expanded_vertices.push_back(
                            v->valueint);
                }
            } else if (strcmp(key, "picked_skeleton_points") == 0) {
                int sp_count = cJSON_GetArraySize(child);
                for (int i = 0; i < sp_count; ++i) {
                    const cJSON* sp_obj = cJSON_GetArrayItem(child, i);
                    if (sp_obj && cJSON_IsObject(sp_obj)) {
                        snapshot.picked_skeleton_points.push_back(
                            parse_skeleton_point(sp_obj));
                    }
                }
            } else if (strcmp(key, "skeleton_lines") == 0) {
                int line_count = cJSON_GetArraySize(child);
                for (int i = 0; i < line_count; ++i) {
                    const cJSON* line_obj = cJSON_GetArrayItem(child, i);
                    if (!line_obj || !cJSON_IsObject(line_obj))
                        continue;

                    sinriv::kigstudio::voxel::vec3f start;
                    sinriv::kigstudio::voxel::vec3f end;
                    const cJSON* line_child = nullptr;
                    cJSON_ArrayForEach(line_child, line_obj) {
                        if (!line_child->string)
                            continue;
                        if (!cJSON_IsObject(line_child))
                            continue;
                        if (strcmp(line_child->string, "start") == 0) {
                            start = sinriv::kigstudio::vec3_from_json<
                                sinriv::kigstudio::voxel::vec3f>(line_child);
                        } else if (strcmp(line_child->string, "end") == 0) {
                            end = sinriv::kigstudio::vec3_from_json<
                                sinriv::kigstudio::voxel::vec3f>(line_child);
                        }
                    }
                    snapshot.skeleton_lines.emplace_back(start, end);
                }
            }
        }
    }

    return snapshot;
}

bool RenderVoxelList::save_current_project() {
    if (project_path.empty()) {
        last_save_error = "no project path set";
        return false;
    }
    return save_project(project_path);
}

bool RenderVoxelList::save_project(const std::string& folder) {
    last_save_error.clear();
    std::filesystem::path dir = utf8_path(folder);
    try {
        std::filesystem::create_directories(dir / "voxels");
        std::filesystem::create_directories(dir / "marked");
    } catch (const std::exception& e) {
        last_save_error = std::string("create_directories failed: ") + e.what();
        return false;
    }

    std::lock_guard<std::mutex> lock(locker);
    cJSON* root = cJSON_CreateObject();
    if (!root) {
        last_save_error = "cJSON_CreateObject failed";
        return false;
    }
    cJSON_AddNumberToObject(root, "version", 1);
    cJSON_AddNumberToObject(root, "current_id", current_id.load());
    cJSON* arr = cJSON_CreateArray();
    for (const auto& [id, item] : items) {
        cJSON* item_json = item_to_json(*item);
        if (!item_json) {
            last_save_error =
                "item_to_json failed for item id=" + std::to_string(id);
            cJSON_Delete(root);
            return false;
        }
        cJSON_AddItemToArray(arr, item_json);
        std::filesystem::path voxel_path =
            dir / "voxels" / (std::to_string(id) + ".vxgrid");
        if (!std::filesystem::exists(dir / "voxels")) {
            last_save_error =
                "voxels directory not found: " + path_to_utf8(dir / "voxels");
            cJSON_Delete(root);
            return false;
        }
        std::string voxel_error;
        if (!sinriv::kigstudio::save(voxel_path, item->voxel_grid_data,
                                     &voxel_error)) {
            last_save_error = "save voxel failed: " + path_to_utf8(voxel_path) +
                              " (" + voxel_error + ")";
            cJSON_Delete(root);
            return false;
        }
        if (!item->marked_voxels.empty()) {
            std::filesystem::path marked_path =
                dir / "marked" / (std::to_string(id) + ".vxgrid");
            std::string marked_error;
            if (!sinriv::kigstudio::save(marked_path, item->marked_voxels,
                                         &marked_error)) {
                last_save_error = "save marked failed: " +
                                  path_to_utf8(marked_path) + " (" +
                                  marked_error + ")";
                cJSON_Delete(root);
                return false;
            }
        }
    }
    cJSON_AddItemToObject(root, "items", arr);

    // 保存每个节点的mesh数据到 meshes/<id>.mesh
    {
        try {
            std::filesystem::create_directories(dir / "meshes");
        } catch (const std::exception& e) {
            // non-fatal: continue without mesh cache
            std::cerr << "[save_project] create_directories(meshes) failed: "
                      << e.what() << std::endl;
        }
        for (const auto& [id, item] : items) {
            std::filesystem::path mesh_path =
                dir / "meshes" / (std::to_string(id) + ".mesh");
            bool saved = false;
            if (!item->cached_mesh.empty()) {
                saved = save_mesh_file(mesh_path, item->cached_mesh);
            } else if (!item->source_triangles.empty()) {
                saved = save_mesh_file(mesh_path, item->source_triangles);
            }
            // Remove stale mesh file if node no longer has mesh data
            // or if save failed (to avoid loading corrupt/incomplete data)
            if (!saved && std::filesystem::exists(mesh_path)) {
                std::error_code ec;
                std::filesystem::remove(mesh_path, ec);
            }
        }
    }

    // 保存工作流输入/输出（节点ID + 文件路径）
    {
        auto save_flow_entries =
            [](const std::vector<FlowEntry>& entries) -> cJSON* {
            cJSON* arr = cJSON_CreateArray();
            for (const auto& e : entries) {
                cJSON* obj = cJSON_CreateObject();
                cJSON_AddNumberToObject(obj, "node_id", e.node_id);
                cJSON_AddStringToObject(obj, "file_path",
                                        e.file_path.c_str());
                cJSON_AddItemToArray(arr, obj);
            }
            return arr;
        };
        cJSON_AddItemToObject(root, "flow_inputs",
                              save_flow_entries(flow_inputs));
        cJSON_AddItemToObject(root, "flow_outputs",
                              save_flow_entries(flow_outputs));
    }

    std::filesystem::path json_path = dir / "project.json";
    char* json_str = cJSON_Print(root);
    if (!json_str) {
        last_save_error = "cJSON_Print failed";
        cJSON_Delete(root);
        return false;
    }
#ifdef _WIN32
    std::ofstream ofs(json_path.wstring().c_str());
#else
    std::ofstream ofs(json_path.c_str());
#endif
    if (!ofs) {
        last_save_error = "failed to open project.json for writing: " +
                          path_to_utf8(json_path);
        cJSON_free(json_str);
        cJSON_Delete(root);
        return false;
    }
    const char utf8_bom[] = "\xEF\xBB\xBF";
    ofs.write(utf8_bom, 3);
    ofs << json_str;
    bool ok = ofs.good();
    if (!ok) {
        last_save_error = "failed to write project.json";
    }
    cJSON_free(json_str);
    cJSON_Delete(root);
    if (ok)
        clear_all_dirty();
    return ok;
}

bool RenderVoxelList::load_project(const std::string& folder) {
    last_load_error.clear();

    // Validate that the project folder and project.json exist BEFORE
    // calling release(), which destroys all current items.  This
    // prevents a catastrophic state where items are gone but the new
    // project fails to load, leaving an empty project with a stale
    // project_path.
    std::filesystem::path dir = utf8_path(folder);
    std::filesystem::path json_path = dir / "project.json";
    {
#ifdef _WIN32
        std::ifstream test_ifs(json_path.wstring().c_str());
#else
        std::ifstream test_ifs(json_path.c_str());
#endif
        if (!test_ifs) {
            last_load_error =
                "failed to open project.json: " + path_to_utf8(json_path);
            return false;
        }
    }

    // Save current project path so we can restore it on failure.
    // release() destroys all items; if loading subsequently fails we
    // want to keep the old project_path visible in the title bar so
    // the user knows what was open before the error.
    std::string previous_project_path = project_path;

    release();
    update_nav_node_status = true;
    start_thread();
    initIcons();
    current_id = 0;

#ifdef _WIN32
    std::ifstream ifs(json_path.wstring().c_str());
#else
    std::ifstream ifs(json_path.c_str());
#endif
    if (!ifs) {
        last_load_error =
            "failed to open project.json: " + path_to_utf8(json_path);
        // Restore previous project path so the title doesn't go blank.
        project_path = previous_project_path;
        return false;
    }
    std::string json_str((std::istreambuf_iterator<char>(ifs)),
                         std::istreambuf_iterator<char>());
    if (json_str.size() >= 3 &&
        static_cast<unsigned char>(json_str[0]) == 0xEF &&
        static_cast<unsigned char>(json_str[1]) == 0xBB &&
        static_cast<unsigned char>(json_str[2]) == 0xBF) {
        json_str.erase(0, 3);
    }
    cJSON* root = cJSON_Parse(json_str.c_str());
    if (!root) {
        last_load_error = "cJSON_Parse failed";
        project_path = previous_project_path;
        return false;
    }

    int version = 0;
    bool has_version = false;
    bool has_current_id = false;
    const cJSON* items_arr = nullptr;
    const cJSON* flow_inputs_arr = nullptr;
    const cJSON* flow_outputs_arr = nullptr;

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, root) {
        if (!child->string)
            continue;

        if (strcmp(child->string, "version") == 0 &&
            cJSON_IsNumber(child)) {
            version = child->valueint;
            has_version = true;
        } else if (strcmp(child->string, "current_id") == 0 &&
                   cJSON_IsNumber(child)) {
            current_id = child->valueint;
            has_current_id = true;
        } else if (strcmp(child->string, "items") == 0 &&
                   cJSON_IsArray(child)) {
            items_arr = child;
        } else if (strcmp(child->string, "flow_inputs") == 0 &&
                   cJSON_IsArray(child)) {
            flow_inputs_arr = child;
        } else if (strcmp(child->string, "flow_outputs") == 0 &&
                   cJSON_IsArray(child)) {
            flow_outputs_arr = child;
        }
    }

    if (!has_version) {
        last_load_error = "missing 'version' field";
        cJSON_Delete(root);
        project_path = previous_project_path;
	        return false;
    }
    if (version != 1) {
        last_load_error = "unsupported version: " + std::to_string(version);
        cJSON_Delete(root);
        project_path = previous_project_path;
	        return false;
    }

    if (!has_current_id) {
        last_load_error = "missing 'current_id' field";
        cJSON_Delete(root);
        project_path = previous_project_path;
	        return false;
    }

    if (!items_arr) {
        last_load_error = "missing 'items' field";
        cJSON_Delete(root);
        project_path = previous_project_path;
	        return false;
    }
    int count = cJSON_GetArraySize(items_arr);
    {
        std::lock_guard<std::mutex> lock(locker);
        for (int i = 0; i < count; ++i) {
            const cJSON* item_obj = cJSON_GetArrayItem(items_arr, i);
            auto item = item_from_json(item_obj);
            if (!item) {
                last_load_error =
                    "item_from_json failed at index " + std::to_string(i);
                cJSON_Delete(root);
                project_path = previous_project_path;
	                return false;
            }
            int id = item->id;
            std::filesystem::path voxel_path =
                dir / "voxels" / (std::to_string(id) + ".vxgrid");
            if (!sinriv::kigstudio::load(voxel_path, item->voxel_grid_data)) {
                last_load_error = "load voxel failed: " + voxel_path.string();
                cJSON_Delete(root);
                project_path = previous_project_path;
	                return false;
            }
            if (item->voxel_grid_data.num_chunk() > 0) {
                item->voxel_renderer.loadVoxelGridChunked(item->voxel_grid_data,
                                                           0.5, true);
            }

            // 尝试从工程文件加载缓存的mesh数据
            bool has_mesh_from_cache = false;
            {
                std::filesystem::path mesh_path =
                    dir / "meshes" / (std::to_string(id) + ".mesh");
                if (std::filesystem::exists(mesh_path)) {
                    bool mesh_ok = false;
                    auto loaded_mesh = load_mesh_file(mesh_path, mesh_ok);
                    if (mesh_ok && !loaded_mesh.empty()) {
                        item->cached_mesh = std::move(loaded_mesh);
                        item->cached_mesh_dirty = false;
                        item->exported_mesh_synced = false;

                        // 从 cached_mesh 重建 source_triangles（去掉法线）
                        item->source_triangles.clear();
                        item->source_triangles.reserve(
                            item->cached_mesh.size());
                        for (const auto& [tri, n] : item->cached_mesh) {
                            (void)n;
                            item->source_triangles.push_back(tri);
                        }

                        // 上传到各渲染器
                        item->mesh_renderer.clear();
                        item->mesh_renderer.loadGeometry(
                            item->cached_mesh);
                        item->origin_mesh_renderer.clear();
                        item->origin_mesh_renderer.setBaseColor(
                            0.0f, 0.0f, 1.0f, 1.0f);
                        item->origin_mesh_renderer.loadGeometry(
                            item->cached_mesh);

                        // 若需要SDF且source_triangles已恢复，构建SDF
                        if (item->load_as_sdf &&
                            !item->source_triangles.empty()) {
                            auto mesh_sdf = std::make_shared<
                                sinriv::kigstudio::sdf::SDF_Mesh>();
                            mesh_sdf->loadTriangles(
                                item->source_triangles);
                            item->sdf_data = std::move(mesh_sdf);
                        }

                        has_mesh_from_cache = true;
                        std::cout << "Loaded cached mesh for item " << id
                                  << " (" << item->cached_mesh.size()
                                  << " tris)" << std::endl;
                    }
                }
            }

            // 如果没有从缓存加载到mesh，尝试从STL文件重载
            if (!has_mesh_from_cache && !item->stl_path.empty()) {
                try {
                    std::cout << "Loading STL mesh for item " << id
                              << " from path: " << item->stl_path << std::endl;
                    auto stl_path = utf8_path(item->stl_path);
                    std::string utf8_stl_path = path_to_utf8(stl_path);
                    std::cout << "STL path after conversion: " << utf8_stl_path
                              << std::endl;
                    item->mesh_renderer.loadSTL(utf8_stl_path);
                    item->stl_path = utf8_stl_path;
                    item->source_triangles.clear();
                    for (auto [tri, n] :
                         sinriv::kigstudio::voxel::readSTL(utf8_stl_path)) {
                        (void)n;
                        item->source_triangles.push_back(tri);
                    }
                    item->origin_mesh_renderer.clear();
                    item->origin_mesh_renderer.setBaseColor(0.0f, 0.0f, 1.0f,
                                                            1.0f);
                    item->origin_mesh_renderer.loadGeometry(
                        sinriv::kigstudio::voxel::readSTL(utf8_stl_path));
                    if (item->load_as_sdf && !item->source_triangles.empty()) {
                        auto mesh_sdf = std::make_shared<
                            sinriv::kigstudio::sdf::SDF_Mesh>();
                        if (item->stl_load_mode ==
                                static_cast<int>(
                                    StlLoadMode::SILHOUETTE) ||
                            item->stl_load_mode ==
                                static_cast<int>(
                                    StlLoadMode::CONVEX_HULL)) {
                            mesh_sdf->loadTriangles(item->source_triangles);
                        } else {
                            mesh_sdf->loadSTL(item->stl_path);
                        }
                        item->sdf_data = std::move(mesh_sdf);
                    }
                } catch (const std::exception& e) {
                    std::cout << "Failed to load STL mesh for item " << id
                              << ": " << e.what() << std::endl;
                }
            }
            const cJSON* has_marked =
                cJSON_GetObjectItem(item_obj, "has_marked_voxels");
            if (has_marked && cJSON_IsTrue(has_marked)) {
                std::filesystem::path marked_path =
                    dir / "marked" / (std::to_string(id) + ".vxgrid");
                if (std::filesystem::exists(marked_path)) {
                    if (!sinriv::kigstudio::load(marked_path,
                                                 item->marked_voxels)) {
                        std::cout << "Failed to load marked voxels for item "
                                  << id << std::endl;
                    } else {
                        item->marked_voxels.global_position =
                            item->voxel_grid_data.global_position;
                        item->marked_voxels.voxel_size =
                            item->voxel_grid_data.voxel_size;
                        item->marked_voxels_dirty = true;
                    }
                }
            }
            items[id] = std::move(item);
        }
    }

    // 加载工作流输入/输出（节点ID + 文件路径，兼容旧格式）
    auto load_flow_entries =
        [](const cJSON* arr) -> std::vector<FlowEntry> {
        std::vector<FlowEntry> result;
        if (!arr) return result;
        int count = cJSON_GetArraySize(arr);
        for (int i = 0; i < count; ++i) {
            const cJSON* v = cJSON_GetArrayItem(arr, i);
            if (!v) continue;
            FlowEntry e;
            if (cJSON_IsObject(v)) {
                const cJSON* nid = cJSON_GetObjectItem(v, "node_id");
                const cJSON* fp = cJSON_GetObjectItem(v, "file_path");
                if (nid && cJSON_IsNumber(nid))
                    e.node_id = nid->valueint;
                if (fp && cJSON_IsString(fp) && fp->valuestring)
                    e.file_path = fp->valuestring;
                result.push_back(e);
            } else if (cJSON_IsNumber(v)) {
                // 旧格式：纯节点ID
                e.node_id = v->valueint;
                result.push_back(e);
            } else if (cJSON_IsString(v)) {
                // 旧格式：纯文件路径
                e.file_path = v->valuestring;
                result.push_back(e);
            }
        }
        return result;
    };
    flow_inputs = load_flow_entries(flow_inputs_arr);
    flow_outputs = load_flow_entries(flow_outputs_arr);
    flow_needs_recompute = true;

    // 重建由 segment 产生的子节点的 SDF 数据
    //（体素网格已从 .vxgrid 恢复，但 sdf_data 无法直接序列化）
    {
        std::unordered_map<int, std::vector<int>> forward_edges;
        std::unordered_map<int, int> in_degree;
        for (const auto& [id, item] : items) {
            in_degree[id] = 0;
            for (int child_id : item->children) {
                if (child_id >= 0 && items.find(child_id) != items.end()) {
                    forward_edges[id].push_back(child_id);
                    ++in_degree[child_id];
                }
            }
            if (item->segment_mode == RenderVoxelItem::SDF_NODE_SPLIT &&
                item->sdf_split_target_id >= 0) {
                auto target_it = items.find(item->sdf_split_target_id);
                if (target_it != items.end()) {
                    forward_edges[item->sdf_split_target_id].push_back(id);
                    ++in_degree[id];
                }
            }
        }

        std::queue<int> q;
        for (const auto& [id, degree] : in_degree) {
            if (degree == 0)
                q.push(id);
        }

        std::vector<int> order;
        order.reserve(items.size());
        while (!q.empty()) {
            int cur = q.front();
            q.pop();
            order.push_back(cur);
            auto it = forward_edges.find(cur);
            if (it == forward_edges.end())
                continue;
            for (int next : it->second) {
                auto deg_it = in_degree.find(next);
                if (deg_it == in_degree.end())
                    continue;
                if (--deg_it->second == 0)
                    q.push(next);
            }
        }

        for (int id : order) {
            auto it = items.find(id);
            if (it == items.end())
                continue;
            auto& item = *it->second;
            const bool is_addon = (item.source_type == 2);
            if (!item.sdf_data && !is_addon)
                continue;

            // 附加件节点：重建自身SDF（各发束SDF的并集，按需减去底模），
            // 与文件Tab中的“更新SDF”按钮行为一致
            if (is_addon && !item.sdf_data && !item.hair_strands.empty()) {
                item.sdf_data = item.build_hair_sdf();
                if (item.sdf_data && item.addon_reveal &&
                    item.addon_base_node_id >= 0) {
                    auto base_it = items.find(item.addon_base_node_id);
                    if (base_it != items.end() && base_it->second->sdf_data) {
                        item.sdf_data =
                            sinriv::kigstudio::sdf::sdf_subtraction(
                                item.sdf_data, base_it->second->sdf_data);
                    }
                }
            }

            bool has_valid_children = false;
            for (int child_id : item.children) {
                if (child_id >= 0 && items.find(child_id) != items.end()) {
                    has_valid_children = true;
                    break;
                }
            }
            if (!has_valid_children)
                continue;

            try {
                auto results = item.do_segment();
                if (results.size() != item.children.size()) {
                    std::cerr << "[load_project] segment result count mismatch "
                                 "for node "
                              << id << std::endl;
                    continue;
                }
                for (size_t i = 0; i < results.size(); ++i) {
                    int child_id = item.children[i];
                    if (child_id < 0)
                        continue;
                    auto child_it = items.find(child_id);
                    if (child_it == items.end())
                        continue;
                    if (!child_it->second->sdf_data) {
                        child_it->second->sdf_data =
                            std::move(std::get<1>(results[i]));
                    }
                    // 附加件几何布尔路径：恢复子节点的布尔结果网格
                    auto& result_tris = std::get<2>(results[i]);
                    if (!result_tris.empty()) {
                        child_it->second->source_triangles =
                            std::move(result_tris);
                        using VoxelTriangle =
                            sinriv::kigstudio::voxel::Triangle;
                        using Vec3f = sinriv::kigstudio::vec3<float>;
                        std::vector<std::tuple<VoxelTriangle, Vec3f>>
                            mesh_data;
                        mesh_data.reserve(
                            child_it->second->source_triangles.size());
                        for (const auto& t :
                             child_it->second->source_triangles) {
                            const auto& a = std::get<0>(t);
                            const auto& b = std::get<1>(t);
                            const auto& c = std::get<2>(t);
                            auto n = (b - a).cross(c - a);
                            const float len = n.length();
                            if (len > 1e-8f)
                                n = n / len;
                            else
                                n = Vec3f{0.0f, 0.0f, 0.0f};
                            mesh_data.emplace_back(t, n);
                        }
                        child_it->second->mesh_renderer.loadGeometry(
                            std::move(mesh_data));
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "[load_project] failed to rebuild SDF for node "
                          << id << ": " << e.what() << std::endl;
            }
        }
    }
	// 自动加载附加件节点的底模到 origin_mesh_renderer，
	// 避免用户每次打开工程后都要手动点击"应用基础模型"
	for (auto& [id, item_ptr] : items) {
		(void)id;
		auto& item = *item_ptr;
		if (item.source_type != 2) continue;
		if (item.addon_base_node_id < 0) continue;
		auto base_it = items.find(item.addon_base_node_id);
		if (base_it == items.end()) continue;
		auto& base = *base_it->second;
		if (!base.cached_mesh.empty()) {
			item.origin_mesh_renderer.clear();
			item.origin_mesh_renderer.loadGeometry(base.cached_mesh);
		} else if (!base.source_triangles.empty()) {
			item.origin_mesh_renderer.clear();
			using VoxelTriangle =
			    sinriv::kigstudio::voxel::Triangle;
			using Vec3f = sinriv::kigstudio::vec3<float>;
			std::vector<std::tuple<VoxelTriangle, Vec3f>> triangles;
			triangles.reserve(base.source_triangles.size());
			for (const auto& tri : base.source_triangles) {
				triangles.emplace_back(
				    tri,
				    sinriv::kigstudio::voxel::calcTriangleNormal(tri));
			}
			item.origin_mesh_renderer.loadGeometry(triangles);
		}
		item.origin_mesh_renderer.setBaseColor(1.0f, 0.9f, 0.7f, 1.0f);
		item.showOriginMesh = true;
	}

    cJSON_Delete(root);
    if (!items.empty()) {
        render_id = items.begin()->first;
    }
    project_path = folder;
    nav_layout_initialized = true;
    clear_all_dirty();
    object_editor_tab = 0;
    return true;
}

}  // namespace sinriv::ui::render
