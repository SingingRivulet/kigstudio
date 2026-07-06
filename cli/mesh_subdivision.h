#pragma once

#include <map>
#include <string>
#include <vector>
#include <tuple>
#include <iostream>

#include "kigstudio/cgal/mesh_subdivision.h"
#include "kigstudio/voxel/voxel2mesh.h"

using Triangle = sinriv::kigstudio::voxel::Triangle;
using vec3f    = sinriv::kigstudio::voxel::vec3f;
using MeshData = std::vector<std::tuple<Triangle, vec3f>>;

namespace mesh_subdivision_cli {

MeshData read_stl(const std::string& path) {
    std::cout << "Reading STL: " << path << std::endl;
    MeshData m;
    auto gen = sinriv::kigstudio::voxel::readSTL(path);
    for (auto it = gen.begin(); it != gen.end(); ++it)
        m.push_back(std::move(*it));
    std::cout << "  " << m.size() << " triangles\n";
    return m;
}

void write_stl(const MeshData& m, const std::string& path) {
    std::cout << "Writing STL (" << m.size() << " triangles): " << path << std::endl;
    sinriv::kigstudio::voxel::saveMeshToBinarySTL(m, path);
}

} // namespace mesh_subdivision_cli

inline int subdivideMesh_main(const std::string& prog,
                              const std::map<std::string, std::string>& args) {
    auto in_it  = args.find("in");
    auto out_it = args.find("out");
    if (in_it == args.end() || out_it == args.end()) {
        std::cerr << "Usage: " << prog << " --tools --subdivideMesh"
                  << " --in <input.stl> --out <output.stl>"
                  << " (--targetLength <length> | --level <1-10>)"
                  << " [--iterations <N>]\n";
        return 1;
    }

    bool use_level = false;
    int level = 1;
    double target_length = 0.0;

    auto lv = args.find("level");
    auto tl = args.find("targetLength");

    if (lv != args.end() && tl != args.end()) {
        std::cerr << "Error: --level and --targetLength are mutually exclusive.\n";
        return 1;
    }

    if (lv != args.end()) {
        use_level = true;
        try {
            level = std::stoi(lv->second);
        } catch (...) {
            std::cerr << "Error: invalid --level value '" << lv->second << "'\n";
            return 1;
        }
        if (level < 1) {
            std::cerr << "Error: --level must be >= 1.\n";
            return 1;
        }
    } else if (tl != args.end()) {
        try {
            target_length = std::stod(tl->second);
        } catch (...) {
            std::cerr << "Error: invalid --targetLength value '" << tl->second << "'\n";
            return 1;
        }
        if (target_length <= 0.0) {
            std::cerr << "Error: --targetLength must be positive.\n";
            return 1;
        }
    } else {
        std::cerr << "Error: either --targetLength or --level is required.\n";
        return 1;
    }

    unsigned int iterations = 3;
    auto it = args.find("iterations");
    if (it != args.end()) {
        try {
            int v = std::stoi(it->second);
            if (v < 1) {
                std::cerr << "Error: --iterations must be >= 1.\n";
                return 1;
            }
            iterations = static_cast<unsigned int>(v);
        } catch (...) {
            std::cerr << "Error: invalid --iterations value '" << it->second << "'\n";
            return 1;
        }
    }

    auto mesh = mesh_subdivision_cli::read_stl(in_it->second);
    if (mesh.empty()) { std::cerr << "Error: empty input.\n"; return 1; }

    MeshData result;
    if (use_level) {
        std::cout << "Subdividing (level = " << level
                  << ", iterations = " << iterations << ") ..." << std::endl;
        result = sinriv::kigstudio::cgal::subdivideMeshByLevel(
            mesh, level, iterations);
    } else {
        std::cout << "Subdividing (target_length = " << target_length
                  << ", iterations = " << iterations << ") ..." << std::endl;
        result = sinriv::kigstudio::cgal::subdivideMesh(
            mesh, target_length, iterations);
    }

    if (result.empty()) {
        std::cerr << "Error: subdivideMesh produced empty mesh.\n";
        return 1;
    }

    mesh_subdivision_cli::write_stl(result, out_it->second);
    std::cout << "Done.\n";
    return 0;
}
