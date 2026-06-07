#include "utils.h"
#include <cstddef>
#include <iomanip>
#include <vector>

#if defined(_WIN32)

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#include <dbghelp.h>

#pragma comment(lib, "psapi.lib")
#pragma comment(lib, "dbghelp.lib")

#elif defined(__linux__) || defined(__APPLE__)

#include <sys/resource.h>
#include <unistd.h>
#include <cstdio>

#endif
#ifdef __APPLE__

#include <mach/mach.h>

#endif
namespace sinriv {
// ============================================================
// 当前 RSS（Resident Set Size）
// 当前实际驻留在物理内存中的大小
// ============================================================

size_t getCurrentRSS() {
#if defined(_WIN32)

    PROCESS_MEMORY_COUNTERS info;
    GetProcessMemoryInfo(GetCurrentProcess(), &info, sizeof(info));

    return static_cast<size_t>(info.WorkingSetSize);

#elif defined(__APPLE__)

    // macOS

    struct mach_task_basic_info info;
    mach_msg_type_number_t infoCount = MACH_TASK_BASIC_INFO_COUNT;

    if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
                  reinterpret_cast<task_info_t>(&info),
                  &infoCount) != KERN_SUCCESS) {
        return 0;
    }

    return static_cast<size_t>(info.resident_size);

#elif defined(__linux__)

    // Linux:
    // /proc/self/statm 第二项是 resident pages

    long rss = 0L;

    FILE* fp = std::fopen("/proc/self/statm", "r");

    if (fp == nullptr)
        return 0;

    if (std::fscanf(fp, "%*s%ld", &rss) != 1) {
        std::fclose(fp);
        return 0;
    }

    std::fclose(fp);

    return static_cast<size_t>(rss) *
           static_cast<size_t>(sysconf(_SC_PAGESIZE));

#else

    return 0;

#endif
}

// ============================================================
// 峰值 RSS
// 进程生命周期内达到过的最大 RSS
// ============================================================

size_t getPeakRSS() {
#if defined(_WIN32)

    PROCESS_MEMORY_COUNTERS info;
    GetProcessMemoryInfo(GetCurrentProcess(), &info, sizeof(info));

    return static_cast<size_t>(info.PeakWorkingSetSize);

#elif defined(__APPLE__)

    struct rusage rusage{};

    getrusage(RUSAGE_SELF, &rusage);

    return static_cast<size_t>(rusage.ru_maxrss);

#elif defined(__linux__)

    struct rusage rusage{};

    getrusage(RUSAGE_SELF, &rusage);

    // Linux 下 ru_maxrss 单位是 KB
    return static_cast<size_t>(rusage.ru_maxrss) * 1024L;

#else

    return 0;

#endif
}

// ============================================================
// 打印当前调用堆栈（Windows 版，需要 pdb 符号文件）
// ============================================================
void print_stacktrace(std::ostream& out, int skip_frames) {
#if defined(_WIN32)
    HANDLE process = GetCurrentProcess();
    SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME);
    if (!SymInitialize(process, nullptr, TRUE)) {
        out << "  [SymInitialize failed, error=" << GetLastError() << "]\n";
        return;
    }

    constexpr int MAX_FRAMES = 64;
    void* stack[MAX_FRAMES];
    WORD frames = CaptureStackBackTrace(skip_frames, MAX_FRAMES, stack, nullptr);

    // SYMBOL_INFO + 名字缓冲区
    alignas(SYMBOL_INFO) char symbol_buf[sizeof(SYMBOL_INFO) + 256];
    SYMBOL_INFO* symbol = reinterpret_cast<SYMBOL_INFO*>(symbol_buf);
    symbol->MaxNameLen = 255;
    symbol->SizeOfStruct = sizeof(SYMBOL_INFO);

    IMAGEHLP_LINE64 line{};
    line.SizeOfStruct = sizeof(IMAGEHLP_LINE64);
    DWORD disp = 0;

    for (WORD i = 0; i < frames; ++i) {
        DWORD64 addr = reinterpret_cast<DWORD64>(stack[i]);
        out << "  [" << i << "] ";

        if (SymFromAddr(process, addr, nullptr, symbol)) {
            out << symbol->Name;
        } else {
            out << "0x" << std::hex << addr << std::dec;
        }

        if (SymGetLineFromAddr64(process, addr, &disp, &line)) {
            out << "  at " << line.FileName << ":" << line.LineNumber;
        }
        out << "\n";
    }

    SymCleanup(process);
#else
    out << "  [stack trace only implemented for Windows]\n";
#endif
}
}  // namespace sinriv