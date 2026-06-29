#pragma once

#include <map>
#include <string>
#include <vector>
#include <tuple>
#include <iostream>

#include "kigstudio/cgal/mesh_repair.h"
#include "kigstudio/voxel/voxel2mesh.h"

using Triangle = sinriv::kigstudio::voxel::Triangle;
using vec3f    = sinriv::kigstudio::voxel::vec3f;
using MeshData = std::vector<std::tuple<Triangle, vec3f>>;

// ---- helpers ----

namespace {

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

} // anonymous namespace

// ---- alphaWrap ----

inline int alphaWrap_main(const std::string& prog,
                          const std::map<std::string, std::string>& args) {
    auto in_it  = args.find("in");
    auto out_it = args.find("out");
    if (in_it == args.end() || out_it == args.end()) {
        std::cerr << "Usage: " << prog << " --tools --alphaWrap"
                  << " --in <input.stl> --out <output.stl>"
                  << " [--alpha <10.0>] [--offset <0.1>]\n";
        return 1;
    }

    double alpha  = 10.0;
    double offset = 0.1;
    auto ai = args.find("alpha");  if (ai != args.end()) alpha  = std::stod(ai->second);
    auto oi = args.find("offset"); if (oi != args.end()) offset = std::stod(oi->second);

    auto mesh   = read_stl(in_it->second);
    if (mesh.empty()) { std::cerr << "Error: empty input.\n"; return 1; }

    auto result = sinriv::kigstudio::cgal::alpha_wrap(mesh, alpha, offset);
    if (result.empty()) { std::cerr << "Error: alpha_wrap produced empty mesh.\n"; return 1; }

    write_stl(result, out_it->second);
    std::cout << "Done.\n";
    return 0;
}

// ---- fillHoles ----

inline int fillHoles_main(const std::string& prog,
                          const std::map<std::string, std::string>& args) {
    auto in_it  = args.find("in");
    auto out_it = args.find("out");
    if (in_it == args.end() || out_it == args.end()) {
        std::cerr << "Usage: " << prog << " --tools --fillHoles"
                  << " --in <input.stl> --out <output.stl>\n";
        return 1;
    }

    auto mesh = read_stl(in_it->second);
    if (mesh.empty()) { std::cerr << "Error: empty input.\n"; return 1; }

    auto result = sinriv::kigstudio::cgal::fill_holes(mesh);
    if (result.empty()) { std::cerr << "Error: fill_holes produced empty mesh.\n"; return 1; }

    write_stl(result, out_it->second);
    std::cout << "Done.\n";
    return 0;
}

// ---- stitchBorders ----

inline int stitchBorders_main(const std::string& prog,
                              const std::map<std::string, std::string>& args) {
    auto in_it  = args.find("in");
    auto out_it = args.find("out");
    if (in_it == args.end() || out_it == args.end()) {
        std::cerr << "Usage: " << prog << " --tools --stitchBorders"
                  << " --in <input.stl> --out <output.stl>"
                  << " [--maxDist <0.001>]\n";
        return 1;
    }

    double max_dist = 0.001;
    auto md = args.find("maxDist");
    if (md != args.end()) max_dist = std::stod(md->second);

    auto mesh = read_stl(in_it->second);
    if (mesh.empty()) { std::cerr << "Error: empty input.\n"; return 1; }

    auto result = sinriv::kigstudio::cgal::stitch_borders(mesh, max_dist);
    if (result.empty()) { std::cerr << "Error: stitch_borders produced empty mesh.\n"; return 1; }

    write_stl(result, out_it->second);
    std::cout << "Done.\n";
    return 0;
}

// ---- mergeVertices ----

inline int mergeVertices_main(const std::string& prog,
                              const std::map<std::string, std::string>& args) {
    auto in_it  = args.find("in");
    auto out_it = args.find("out");
    if (in_it == args.end() || out_it == args.end()) {
        std::cerr << "Usage: " << prog << " --tools --mergeVertices"
                  << " --in <input.stl> --out <output.stl>"
                  << " [--tol <1e-6>]\n";
        return 1;
    }

    double tol = 1e-6;
    auto ti = args.find("tol");
    if (ti != args.end()) tol = std::stod(ti->second);

    auto mesh = read_stl(in_it->second);
    if (mesh.empty()) { std::cerr << "Error: empty input.\n"; return 1; }

    auto result = sinriv::kigstudio::cgal::merge_duplicate_vertices(mesh, tol);
    if (result.empty()) { std::cerr << "Error: merge_vertices produced empty mesh.\n"; return 1; }

    write_stl(result, out_it->second);
    std::cout << "Done.\n";
    return 0;
}

// ---- meshUnion ----

inline int meshUnion_main(const std::string& prog,
                          const std::map<std::string, std::string>& args) {
    auto inA_it = args.find("inA");
    auto inB_it = args.find("inB");
    auto out_it = args.find("out");
    if (inA_it == args.end() || inB_it == args.end() || out_it == args.end()) {
        std::cerr << "Usage: " << prog << " --tools --meshUnion"
                  << " --inA <a.stl> --inB <b.stl> --out <output.stl>\n";
        return 1;
    }

    auto mesh_a = read_stl(inA_it->second);
    auto mesh_b = read_stl(inB_it->second);
    if (mesh_a.empty() || mesh_b.empty()) {
        std::cerr << "Error: one or both inputs are empty.\n";
        return 1;
    }

    auto result = sinriv::kigstudio::cgal::mesh_union(mesh_a, mesh_b);
    if (result.empty()) { std::cerr << "Error: mesh_union produced empty mesh.\n"; return 1; }

    write_stl(result, out_it->second);
    std::cout << "Done.\n";
    return 0;
}

// ---- orientVolume ----

inline int orientVolume_main(const std::string& prog,
                             const std::map<std::string, std::string>& args) {
    auto in_it  = args.find("in");
    auto out_it = args.find("out");
    if (in_it == args.end() || out_it == args.end()) {
        std::cerr << "Usage: " << prog << " --tools --orientVolume"
                  << " --in <input.stl> --out <output.stl>\n";
        return 1;
    }

    auto mesh = read_stl(in_it->second);
    if (mesh.empty()) { std::cerr << "Error: empty input.\n"; return 1; }

    auto result = sinriv::kigstudio::cgal::orient_volume(mesh);
    if (result.empty()) { std::cerr << "Error: orient_volume produced empty mesh.\n"; return 1; }

    write_stl(result, out_it->second);
    std::cout << "Done.\n";
    return 0;
}
