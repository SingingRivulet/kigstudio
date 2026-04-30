#pragma once
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
namespace sinriv::ui::render {
// 依次检测这些文件是否存在
std::vector<std::string> get_default_font_path();
std::string get_system_language();

void locale_init();
std::string get_locale_string(const std::string& key);
const char* get_locale_cstr(const std::string& key);
}  // namespace sinriv::ui::render
