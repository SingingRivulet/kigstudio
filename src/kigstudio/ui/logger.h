#pragma once
#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <iostream>
namespace sinriv::ui {
struct UILogger : public bgfx::CallbackI {
    void fatal(const char* _filePath,
               uint16_t _line,
               bgfx::Fatal::Enum _code,
               const char* _str) override {
        std::cerr << "BGFX FATAL: " << _str << " (" << _filePath << ":" << _line
                  << ")" << std::endl;
    }

    void traceVargs(const char* _filePath,
                    uint16_t _line,
                    const char* _format,
                    va_list _argList) override {
        char temp[1024];
        vsnprintf(temp, sizeof(temp), _format, _argList);
        // std::cout << "BGFX TRACE: " << temp << std::endl;
    }

    void profilerBegin(const char*, uint32_t, const char*, uint16_t) override {}
    void profilerBeginLiteral(const char*,
                              uint32_t,
                              const char*,
                              uint16_t) override {}
    void profilerEnd() override {}
    uint32_t cacheReadSize(uint64_t) override { return 0; }
    void cacheWrite(uint64_t, const void*, uint32_t) override {}
    void screenShot(const char*,
                    uint32_t,
                    uint32_t,
                    uint32_t,
                    bgfx::TextureFormat::Enum,
                    const void*,
                    uint32_t,
                    bool) override {}
    void captureBegin(uint32_t,
                      uint32_t,
                      uint32_t,
                      bgfx::TextureFormat::Enum,
                      bool) override {}
    void captureEnd() override {}
    void captureFrame(const void*, uint32_t) override {}
    bool cacheRead(uint64_t, void*, uint32_t) override { return false; }
};
}  // namespace sinriv::ui