#pragma once
#include <chrono>
#include <cstdint>
namespace sinriv {
size_t getCurrentRSS();
size_t getPeakRSS();
inline int64_t getUnixTimeSeconds() {
    return std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}
}  // namespace sinriv