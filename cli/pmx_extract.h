#pragma once

#include <algorithm>
#include <cctype>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "kigstudio/io/pmx_loader.h"
#include "kigstudio/voxel/voxel2mesh.h"

namespace pmx_extract_cli {

inline std::vector<std::string> split_keywords(const std::string& s) {
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        // Trim whitespace
        auto start = item.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) continue;
        auto end = item.find_last_not_of(" \t\r\n");
        result.push_back(item.substr(start, end - start + 1));
    }
    return result;
}

}  // namespace pmx_extract_cli

inline int extractOrgans_main(const std::string& prog,
                              const std::map<std::string, std::string>& args) {
    auto in_it = args.find("in");
    auto out_it = args.find("out");
    if (in_it == args.end()) {
        std::cerr << "Usage: " << prog << " --tools --extractOrgans"
                  << " --in <model.pmx> --out <output.stl>"
                  << " --mode <bones|materials> --keywords <k1,k2,...>"
                  << " [--threshold <0.0-1.0>] [--case-sensitive]\n"
                  << "       " << prog << " --tools --extractOrgans"
                  << " --in <model.pmx> --list\n";
        return 1;
    }

    std::cout << "Loading PMX: " << in_it->second << std::endl;
    auto model = sinriv::kigstudio::io::load_pmx(in_it->second);
    if (model.vertices.empty() || model.faces.empty()) {
        std::cerr << "Error: failed to load PMX or empty model.\n";
        return 1;
    }

    // --list mode: dump bones and materials then exit.
    if (args.count("list")) {
        std::cout << "\nMaterials (" << model.materials.size() << "):\n";
        for (size_t i = 0; i < model.materials.size(); ++i) {
            const auto& m = model.materials[i];
            std::cout << "  [" << i << "] " << m.name_local;
            if (!m.name_universal.empty() && m.name_universal != m.name_local) {
                std::cout << " / " << m.name_universal;
            }
            std::cout << " (" << m.face_count << " triangles)\n";
        }
        std::cout << "\nBones (" << model.bones.size() << "):\n";
        for (size_t i = 0; i < model.bones.size(); ++i) {
            const auto& b = model.bones[i];
            std::cout << "  [" << i << "] " << b.name_local;
            if (!b.name_universal.empty() && b.name_universal != b.name_local) {
                std::cout << " / " << b.name_universal;
            }
            std::cout << "\n";
        }
        return 0;
    }

    if (out_it == args.end()) {
        std::cerr << "Error: --out is required (or use --list).\n";
        return 1;
    }

    auto mode_it = args.find("mode");
    if (mode_it == args.end()) {
        std::cerr << "Error: --mode is required (bones or materials).\n";
        return 1;
    }
    std::string mode = mode_it->second;
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (mode != "bones" && mode != "materials") {
        std::cerr << "Error: --mode must be 'bones' or 'materials'.\n";
        return 1;
    }

    auto kw_it = args.find("keywords");
    if (kw_it == args.end()) {
        std::cerr << "Error: --keywords is required.\n";
        return 1;
    }
    auto keywords = pmx_extract_cli::split_keywords(kw_it->second);
    if (keywords.empty()) {
        std::cerr << "Error: --keywords is empty.\n";
        return 1;
    }

    float threshold = 0.5f;
    auto th_it = args.find("threshold");
    if (th_it != args.end()) {
        try {
            threshold = std::stof(th_it->second);
        } catch (...) {
            std::cerr << "Error: invalid --threshold value '" << th_it->second << "'\n";
            return 1;
        }
        if (threshold < 0.0f || threshold > 1.0f) {
            std::cerr << "Error: --threshold must be between 0.0 and 1.0.\n";
            return 1;
        }
    }

    bool case_sensitive = args.count("case-sensitive");

    std::cout << "Extracting by " << mode << " with keywords:";
    for (const auto& k : keywords) std::cout << " '" << k << "'";
    std::cout << " (threshold=" << threshold << ")" << std::endl;

    std::vector<std::tuple<sinriv::kigstudio::voxel::Triangle,
                           sinriv::kigstudio::voxel::vec3f>>
        result;
    if (mode == "bones") {
        result = sinriv::kigstudio::io::extract_by_bone_names(
            model, keywords, threshold, case_sensitive);
    } else {
        result = sinriv::kigstudio::io::extract_by_material_names(
            model, keywords, case_sensitive);
    }

    if (result.empty()) {
        std::cerr << "Error: extraction produced empty mesh.\n";
        return 1;
    }

    std::cout << "Extracted " << result.size() << " triangles.\n";
    std::cout << "Writing STL: " << out_it->second << std::endl;
    sinriv::kigstudio::voxel::saveMeshToBinarySTL(result, out_it->second);
    std::cout << "Done.\n";
    return 0;
}
