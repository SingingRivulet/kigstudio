#define SDL_MAIN_HANDLED
#include "ui.hpp"
#include <iostream>
#include <map>
#include <string>

static std::map<std::string, std::string> parse_args(int argc, char** argv) {
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg.size() >= 3 && arg[0] == '-' && arg[1] == '-') {
            auto eq = arg.find('=', 2);
            if (eq != std::string::npos) {
                // --key=value
                args[arg.substr(2, eq - 2)] = arg.substr(eq + 1);
            } else if (i + 1 < argc && argv[i + 1][0] != '-') {
                // --key value
                args[arg.substr(2)] = argv[++i];
            } else {
                // --flag (boolean, no value)
                args[arg.substr(2)] = "true";
            }
        }
    }
    return args;
}

int main(int argc, char** argv) {
    auto args = parse_args(argc, argv);
    if (args.count("help")) {
        std::cout << "Usage: kigstudio [options]\n"
                  << "  --tools [commond]   Use tools\n"
                  << "  --help              Show this help\n";
        return 0;
    }
    return ui_main(argc, argv);
}
