#pragma once

#include <map>
#include <string>
#include <iostream>

#include "cli/simplifyMesh.h"

inline int cli_main(const std::string& prog, const std::map<std::string, std::string>& args) {
    if (args.count("simplifyMesh")) {
        return simplifyMesh_main(prog, args);
    }

    std::cerr << "Error: unknown tool. Available tools:\n"
              << "  " << prog << " --tools --simplifyMesh  Simplify a triangle mesh (STL)\n";
    return 1;
}
