#pragma once
#include <chrono>
#include <cstdint>
#include <iostream>
namespace sinriv {
size_t getCurrentRSS();
size_t getPeakRSS();
inline int64_t getUnixTimeSeconds() {
    return std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

// 打印当前调用堆栈（Windows Debug 下需要 .pdb）
void print_stacktrace(std::ostream& out = std::cerr, int skip_frames = 1);
}  // namespace sinriv