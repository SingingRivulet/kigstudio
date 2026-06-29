#pragma once

#include <map>
#include <string>
#include <vector>
#include <tuple>
#include <iostream>

#include "kigstudio/cgal/mesh_simplification.h"
#include "kigstudio/voxel/voxel2mesh.h"

inline int simplifyMesh_main(const std::string& prog, const std::map<std::string, std::string>& args) {
    // Parse --in / --out (required)
    auto in_it  = args.find("in");
    auto out_it = args.find("out");

    if (in_it == args.end() || out_it == args.end()) {
        std::cerr << "Usage: " << prog << " --tools --simplifyMesh"
                  << " --in <input.stl> --out <output.stl>"
                  << " [--ratio <0.0-1.0>]\n";
        return 1;
    }

    const std::string& in_path  = in_it->second;
    const std::string& out_path = out_it->second;

    // Parse --ratio (optional, default 0.1 = keep 10% edges)
    double ratio = 0.1;
    auto ratio_it = args.find("ratio");
    if (ratio_it != args.end()) {
        try {
            ratio = std::stod(ratio_it->second);
        } catch (...) {
            std::cerr << "Error: invalid --ratio value '" << ratio_it->second << "'\n";
            return 1;
        }
        if (ratio < 0.0 || ratio > 1.0) {
            std::cerr << "Error: --ratio must be between 0.0 and 1.0\n";
            return 1;
        }
    }

    // Read STL (auto-detects ASCII / binary)
    using Triangle = sinriv::kigstudio::voxel::Triangle;
    using vec3f    = sinriv::kigstudio::voxel::vec3f;

    std::vector<std::tuple<Triangle, vec3f>> mesh;

    std::cout << "Reading STL: " << in_path << std::endl;
    auto gen = sinriv::kigstudio::voxel::readSTL(in_path);

    // Collect all triangles from the coroutine generator
    for (auto it = gen.begin(); it != gen.end(); ++it) {
        mesh.push_back(std::move(*it));
    }

    if (mesh.empty()) {
        std::cerr << "Error: no triangles read from '" << in_path << "'\n";
        return 1;
    }

    std::cout << "  Input:  " << mesh.size() << " triangles\n";
    std::cout << "Simplifying (ratio = " << ratio << ") ..." << std::endl;

    auto result = sinriv::kigstudio::cgal::simplifyMesh(mesh, ratio);

    if (result.empty()) {
        std::cerr << "Error: simplification produced an empty mesh.\n";
        return 1;
    }

    std::cout << "  Output: " << result.size() << " triangles\n";
    std::cout << "Writing STL: " << out_path << std::endl;

    sinriv::kigstudio::voxel::saveMeshToBinarySTL(result, out_path);

    std::cout << "Done.\n";
    return 0;
}
