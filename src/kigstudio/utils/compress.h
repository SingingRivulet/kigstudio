#pragma once
#include <zlib.h>
namespace sinriv::kigstudio {

inline bool zlibCompress(const std::vector<uint8_t>& input,
                         std::vector<uint8_t>& output) {
    uLongf destLen = compressBound(input.size());
    output.resize(destLen);

    int res = compress2(output.data(), &destLen, input.data(), input.size(),
                        Z_BEST_SPEED);  // 或 Z_BEST_COMPRESSION

    if (res != Z_OK)
        return false;

    output.resize(destLen);
    return true;
}

inline bool zlibDecompress(const std::vector<uint8_t>& input,
                           std::vector<uint8_t>& output,
                           size_t expected_size) {
    output.resize(expected_size);

    uLongf destLen = expected_size;
    int res = uncompress(output.data(), &destLen, input.data(), input.size());

    return res == Z_OK;
}
}  // namespace sinriv::kigstudio