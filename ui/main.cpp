#define SDL_MAIN_HANDLED
#include "ui.hpp"
#include "cli/cli.h"
#include <iostream>
#include <map>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

static std::map<std::string, std::string> parse_args(
    const std::vector<std::string>& argv) {
    std::map<std::string, std::string> args;
    for (size_t i = 1; i < argv.size(); ++i) {
        const std::string& arg = argv[i];
        if (arg.size() >= 3 && arg[0] == '-' && arg[1] == '-') {
            auto eq = arg.find('=', 2);
            if (eq != std::string::npos) {
                // --key=value
                args[arg.substr(2, eq - 2)] = arg.substr(eq + 1);
            } else if (i + 1 < argv.size() && argv[i + 1][0] != '-') {
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

static int run(int argc, const char* const* argv) {
    std::vector<std::string> args_vec;
    args_vec.reserve(static_cast<size_t>(argc));
    for (int i = 0; i < argc; ++i) {
        args_vec.emplace_back(argv[i]);
    }

    auto args = parse_args(args_vec);
    const char* prog = (argc > 0) ? argv[0] : "kigstudio";

    if (args.count("help")) {
        std::cout << "Usage: " << prog << " [options]\n"
                  << "  --tools --<tool>    Run a CLI tool\n"
                  << "  --help              Show this help\n";
        return 0;
    }

    if (args.count("tools")) {
        return cli_main(prog, args);
    }

    return ui_main(argc, argv);
}

#ifdef _WIN32

static std::string wide_to_utf8(const wchar_t* wstr) {
    if (!wstr || !wstr[0]) return {};
    int size = WideCharToMultiByte(CP_UTF8, 0, wstr, -1, nullptr, 0, nullptr, nullptr);
    if (size <= 0) return {};
    std::string result(size - 1, '\0');
    WideCharToMultiByte(CP_UTF8, 0, wstr, -1, result.data(), size, nullptr, nullptr);
    return result;
}

int wmain(int argc, wchar_t* argv[]) {
    std::vector<std::string> utf8_args;
    utf8_args.reserve(static_cast<size_t>(argc));
    for (int i = 0; i < argc; ++i) {
        utf8_args.push_back(wide_to_utf8(argv[i]));
    }

    std::vector<const char*> cstr_args;
    cstr_args.reserve(utf8_args.size());
    for (const auto& s : utf8_args) {
        cstr_args.push_back(s.c_str());
    }

    return run(static_cast<int>(cstr_args.size()), cstr_args.data());
}

#else

int main(int argc, char** argv) {
    return run(argc, const_cast<const char* const*>(argv));
}

#endif
