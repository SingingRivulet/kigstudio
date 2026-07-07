#pragma once

#include <cJSON.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "kigstudio/io/pmx_loader.h"
#include "kigstudio/voxel/voxel2mesh.h"

namespace pmx_extract_json_cli {

inline std::string read_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    return std::string((std::istreambuf_iterator<char>(f)),
                       std::istreambuf_iterator<char>());
}

inline bool write_file(const std::string& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write(text.data(), static_cast<std::streamsize>(text.size()));
    return f.good();
}

inline std::vector<std::string> json_string_array(cJSON* arr) {
    std::vector<std::string> result;
    if (!arr || !cJSON_IsArray(arr)) return result;
    int n = cJSON_GetArraySize(arr);
    for (int i = 0; i < n; ++i) {
        cJSON* item = cJSON_GetArrayItem(arr, i);
        if (cJSON_IsString(item)) result.push_back(item->valuestring);
    }
    return result;
}

}  // namespace pmx_extract_json_cli

inline int extractOrgansJson_main(const std::string& prog,
                                  const std::map<std::string, std::string>& args) {
    auto in_it = args.find("json");
    if (in_it == args.end()) {
        std::cerr << "Usage: " << prog
                  << " --tools --json <input.json>\n";
        return 1;
    }

    const std::string input_path = in_it->second;
    const std::string output_path = input_path + ".out.json";

    std::string in_text = pmx_extract_json_cli::read_file(input_path);
    if (in_text.empty()) {
        cJSON* out = cJSON_CreateObject();
        cJSON_AddFalseToObject(out, "success");
        cJSON_AddStringToObject(out, "error", "failed to read input JSON");
        char* text = cJSON_Print(out);
        pmx_extract_json_cli::write_file(output_path, text ? text : "{}");
        cJSON_free(text);
        cJSON_Delete(out);
        return 1;
    }

    cJSON* in = cJSON_Parse(in_text.c_str());
    if (!in) {
        cJSON* out = cJSON_CreateObject();
        cJSON_AddFalseToObject(out, "success");
        cJSON_AddStringToObject(out, "error", "invalid input JSON");
        char* text = cJSON_Print(out);
        pmx_extract_json_cli::write_file(output_path, text ? text : "{}");
        cJSON_free(text);
        cJSON_Delete(out);
        return 1;
    }

    auto reply_error = [&](const char* msg) {
        cJSON* out = cJSON_CreateObject();
        cJSON_AddFalseToObject(out, "success");
        cJSON_AddStringToObject(out, "error", msg);
        char* text = cJSON_Print(out);
        pmx_extract_json_cli::write_file(output_path, text ? text : "{}");
        cJSON_free(text);
        cJSON_Delete(out);
    };

    cJSON* cmd_obj = cJSON_GetObjectItem(in, "command");
    if (!cJSON_IsString(cmd_obj)) {
        reply_error("missing or invalid command");
        cJSON_Delete(in);
        return 1;
    }
    std::string command = cmd_obj->valuestring;

    cJSON* pmx_obj = cJSON_GetObjectItem(in, "pmx");
    if (!cJSON_IsString(pmx_obj)) {
        reply_error("missing or invalid pmx path");
        cJSON_Delete(in);
        return 1;
    }
    std::string pmx_path = pmx_obj->valuestring;

    sinriv::kigstudio::io::PMXModel model;
    try {
        model = sinriv::kigstudio::io::load_pmx(pmx_path);
    } catch (const std::exception& e) {
        reply_error(e.what());
        cJSON_Delete(in);
        return 1;
    }

    if (model.vertices.empty() || model.faces.empty()) {
        reply_error("empty or invalid PMX model");
        cJSON_Delete(in);
        return 1;
    }

    if (command == "list") {
        cJSON* out = cJSON_CreateObject();
        cJSON_AddTrueToObject(out, "success");

        cJSON* bones = cJSON_CreateArray();
        for (const auto& b : model.bones) {
            cJSON* item = cJSON_CreateObject();
            cJSON_AddStringToObject(item, "name_local",
                                    b.name_local.c_str());
            cJSON_AddStringToObject(item, "name_universal",
                                    b.name_universal.c_str());
            cJSON_AddItemToArray(bones, item);
        }
        cJSON_AddItemToObject(out, "bones", bones);

        cJSON* materials = cJSON_CreateArray();
        for (const auto& m : model.materials) {
            cJSON* item = cJSON_CreateObject();
            cJSON_AddStringToObject(item, "name_local",
                                    m.name_local.c_str());
            cJSON_AddStringToObject(item, "name_universal",
                                    m.name_universal.c_str());
            cJSON_AddNumberToObject(item, "face_start",
                                    static_cast<double>(m.face_start));
            cJSON_AddNumberToObject(item, "face_count",
                                    static_cast<double>(m.face_count));
            cJSON_AddItemToArray(materials, item);
        }
        cJSON_AddItemToObject(out, "materials", materials);

        char* text = cJSON_Print(out);
        pmx_extract_json_cli::write_file(output_path, text ? text : "{}");
        cJSON_free(text);
        cJSON_Delete(out);
        cJSON_Delete(in);
        return 0;
    }

    if (command == "extract") {
        cJSON* mode_obj = cJSON_GetObjectItem(in, "mode");
        if (!cJSON_IsString(mode_obj)) {
            reply_error("missing or invalid mode");
            cJSON_Delete(in);
            return 1;
        }
        std::string mode = mode_obj->valuestring;

        cJSON* kw_obj = cJSON_GetObjectItem(in, "keywords");
        auto keywords = pmx_extract_json_cli::json_string_array(kw_obj);
        if (keywords.empty()) {
            reply_error("keywords array is empty");
            cJSON_Delete(in);
            return 1;
        }

        float threshold = 0.5f;
        cJSON* th_obj = cJSON_GetObjectItem(in, "threshold");
        if (cJSON_IsNumber(th_obj)) threshold = static_cast<float>(th_obj->valuedouble);

        bool case_sensitive = false;
        cJSON* cs_obj = cJSON_GetObjectItem(in, "case_sensitive");
        if (cJSON_IsBool(cs_obj)) case_sensitive = cJSON_IsTrue(cs_obj);

        cJSON* stl_obj = cJSON_GetObjectItem(in, "stl");
        if (!cJSON_IsString(stl_obj)) {
            reply_error("missing or invalid output stl path");
            cJSON_Delete(in);
            return 1;
        }
        std::string stl_path = stl_obj->valuestring;

        std::vector<std::tuple<sinriv::kigstudio::voxel::Triangle,
                               sinriv::kigstudio::voxel::vec3f>>
            result;
        if (mode == "bones") {
            result = sinriv::kigstudio::io::extract_by_bone_names(
                model, keywords, threshold, case_sensitive);
        } else if (mode == "materials") {
            result = sinriv::kigstudio::io::extract_by_material_names(
                model, keywords, case_sensitive);
        } else {
            reply_error("mode must be bones or materials");
            cJSON_Delete(in);
            return 1;
        }

        if (result.empty()) {
            reply_error("extraction produced empty mesh");
            cJSON_Delete(in);
            return 1;
        }

        sinriv::kigstudio::voxel::saveMeshToBinarySTL(result, stl_path);

        cJSON* out = cJSON_CreateObject();
        cJSON_AddTrueToObject(out, "success");
        cJSON_AddStringToObject(out, "stl", stl_path.c_str());
        cJSON_AddNumberToObject(out, "triangles",
                                static_cast<double>(result.size()));
        char* text = cJSON_Print(out);
        pmx_extract_json_cli::write_file(output_path, text ? text : "{}");
        cJSON_free(text);
        cJSON_Delete(out);
        cJSON_Delete(in);
        return 0;
    }

    reply_error("unknown command");
    cJSON_Delete(in);
    return 1;
}
