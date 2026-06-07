#pragma once

#ifdef _WIN32
#include <windows.h>
#include <crtdbg.h>
#include <signal.h>
#include <cstdlib>

inline void setup_test_environment() {
    // 禁用 Windows 错误报告对话框 (WER)
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX |
                 SEM_NOOPENFILEERRORBOX);

    // 禁用 CRT 断言弹窗，改为输出到 stderr
    _CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_WARN, _CRTDBG_FILE_STDERR);
    _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ERROR, _CRTDBG_FILE_STDERR);
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);

    // abort() / CGAL 断言失败时直接退出，不弹窗
    signal(SIGABRT, [](int) { std::exit(3); });
}
#else
inline void setup_test_environment() {}
#endif
