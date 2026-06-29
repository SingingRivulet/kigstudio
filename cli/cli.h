#pragma once

#include <map>
#include <string>
#include <iostream>

#include "cli/simplifyMesh.h"
#include "cli/mesh_repair.h"

inline int cli_main(const std::string& prog, const std::map<std::string, std::string>& args) {
    if (args.count("simplifyMesh"))    return simplifyMesh_main(prog, args);
    if (args.count("alphaWrap"))       return alphaWrap_main(prog, args);
    if (args.count("fillHoles"))       return fillHoles_main(prog, args);
    if (args.count("stitchBorders"))   return stitchBorders_main(prog, args);
    if (args.count("mergeVertices"))   return mergeVertices_main(prog, args);
    if (args.count("meshUnion"))       return meshUnion_main(prog, args);
    if (args.count("orientVolume"))    return orientVolume_main(prog, args);

    std::cerr << "Error: unknown tool. Available tools:\n"
              << "  " << prog << " --tools --simplifyMesh    Simplify mesh (edge collapse)\n"
              << "  " << prog << " --tools --alphaWrap       Wrap mesh to watertight\n"
              << "  " << prog << " --tools --fillHoles       Fill holes (triangulate+refine+fair)\n"
              << "  " << prog << " --tools --stitchBorders   Stitch border edges\n"
              << "  " << prog << " --tools --mergeVertices   Merge duplicate vertices\n"
              << "  " << prog << " --tools --meshUnion       Boolean union (needs --inA, --inB)\n"
              << "  " << prog << " --tools --orientVolume    Orient faces outward\n";
    return 1;
}
