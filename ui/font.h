#pragma once
#include <filesystem>
#include <string>
#include <vector>
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
}  // namespace sinriv::ui::render