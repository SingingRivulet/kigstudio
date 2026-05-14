#include "utils.h"
#include <cstddef>

#if defined(_WIN32)

#define NOMINMAX
#include <windows.h>
#include <psapi.h>

#pragma comment(lib, "psapi.lib")

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
}  // namespace sinriv