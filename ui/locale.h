#pragma once
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#else
#include <cstdlib>
#endif
namespace sinriv::ui::render {
// 依次检测这些文件是否存在
inline std::vector<std::string> get_default_font_path() {
    std::string font_paths[] = {
#ifdef _WIN32
        "C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/simhei.ttf",
        "C:/Windows/Fonts/simsun.ttc", "C:/Windows/Fonts/msyh.ttf"
#else
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/opentype/noto/NotoSans-Regular.ttf"
#endif
    };
    std::vector<std::string> result;
    for (auto& path : font_paths) {
        if (std::filesystem::exists(path)) {
            result.push_back(path);
        }
    }
    return result;
}
inline std::string get_system_language() {
#ifdef _WIN32
    WCHAR localeName[LOCALE_NAME_MAX_LENGTH];

    if (GetUserDefaultLocaleName(localeName, LOCALE_NAME_MAX_LENGTH)) {
        int size = WideCharToMultiByte(CP_UTF8, 0, localeName, -1, nullptr, 0,
                                       nullptr, nullptr);

        std::string result(size, 0);

        WideCharToMultiByte(CP_UTF8, 0, localeName, -1, result.data(), size,
                            nullptr, nullptr);

        return result;
    }
    return "en-US";

#else
    const char* lang = getenv("LC_ALL")        ? getenv("LC_ALL")
                       : getenv("LC_MESSAGES") ? getenv("LC_MESSAGES")
                                               : getenv("LANG");

    return lang ? std::string(lang) : "en_US";
#endif
}

void locale_init();
std::string get_locale_string(const std::string& key);
}  // namespace sinriv::ui::render