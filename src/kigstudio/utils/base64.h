#pragma once
#include <cstdint>
#include <string>
#include <vector>
namespace sinriv::kigstudio {

inline std::string base64Encode(const std::vector<uint8_t>& input) {
    static const char* table =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((input.size() + 2) / 3 * 4);
    for (size_t i = 0; i < input.size(); i += 3) {
        uint32_t v = static_cast<uint32_t>(input[i]) << 16;
        if (i + 1 < input.size())
            v |= static_cast<uint32_t>(input[i + 1]) << 8;
        if (i + 2 < input.size())
            v |= input[i + 2];
        out.push_back(table[(v >> 18) & 63]);
        out.push_back(table[(v >> 12) & 63]);
        out.push_back(i + 1 < input.size() ? table[(v >> 6) & 63] : '=');
        out.push_back(i + 2 < input.size() ? table[v & 63] : '=');
    }
    return out;
}

inline bool base64Decode(const std::string& input,
                         std::vector<uint8_t>& output) {
    auto val_of = [](char c) -> int {
        if (c >= 'A' && c <= 'Z')
            return c - 'A';
        if (c >= 'a' && c <= 'z')
            return c - 'a' + 26;
        if (c >= '0' && c <= '9')
            return c - '0' + 52;
        if (c == '+')
            return 62;
        if (c == '/')
            return 63;
        return -1;
    };
    output.clear();
    output.reserve(input.size() / 4 * 3);
    uint32_t acc = 0;
    int nbits = 0;
    for (char c : input) {
        if (c == '=')
            break;
        int v = val_of(c);
        if (v < 0)
            continue;  // 跳过空白等非法字符
        acc = ((acc << 6) | static_cast<uint32_t>(v)) & 0x3FFFu;
        nbits += 6;
        if (nbits >= 8) {
            nbits -= 8;
            output.push_back(static_cast<uint8_t>((acc >> nbits) & 0xFF));
        }
    }
    return true;
}
}  // namespace sinriv::kigstudio
