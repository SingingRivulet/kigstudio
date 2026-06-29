#pragma once

/**
 * Cross-platform process launcher with timeout-aware pipe I/O.
 *
 * Usage:
 *   sinriv::kigstudio::Process proc;
 *   proc.start("some_command --flag");
 *   proc.write("input\n", 6);
 *   char buf[4096];
 *   int n = proc.read(buf, sizeof(buf), 5000); // 5-second timeout
 *   proc.close();
 */

#ifdef _WIN32
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <windows.h>
#else
    #include <sys/wait.h>
    #include <sys/select.h>
    #include <unistd.h>
    #include <errno.h>
    #include <signal.h>
#endif

#include <string>
#include <vector>
#include <cstring>
#include <chrono>
#include <thread>
#include <algorithm>

namespace sinriv::kigstudio {

class Process {
public:
    Process() = default;

    ~Process() { close(); }

    // Non-copyable
    Process(const Process&) = delete;
    Process& operator=(const Process&) = delete;

    // Movable
    Process(Process&& other) noexcept
        : child_handle_(other.child_handle_)
        , stdin_pipe_(other.stdin_pipe_)
        , stdout_pipe_(other.stdout_pipe_)
        , exit_code_(other.exit_code_)
        , running_(other.running_)
    {
        other.stdin_pipe_  = INVALID_PIPE;
        other.stdout_pipe_ = INVALID_PIPE;
        other.child_handle_ = INVALID_CHILD;
        other.running_ = false;
        other.exit_code_ = -1;
    }

    Process& operator=(Process&& other) noexcept {
        if (this != &other) {
            close();
            child_handle_  = other.child_handle_;
            stdin_pipe_    = other.stdin_pipe_;
            stdout_pipe_   = other.stdout_pipe_;
            exit_code_     = other.exit_code_;
            running_       = other.running_;
            other.stdin_pipe_  = INVALID_PIPE;
            other.stdout_pipe_ = INVALID_PIPE;
            other.child_handle_ = INVALID_CHILD;
            other.running_ = false;
            other.exit_code_ = -1;
        }
        return *this;
    }

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    /// Launch a subprocess from a shell command (like popen).
    /// The command is executed via `cmd /c` on Windows, `/bin/sh -c` on POSIX.
    /// Returns true on success.
    bool start(const std::string& cmd);

    /// Read from the child's stdout.
    /// Blocks until at least one byte is available or the timeout expires.
    /// @param buf       destination buffer
    /// @param len       max bytes to read
    /// @param timeout_ms timeout in milliseconds; -1 = no timeout (block forever)
    /// @return bytes read, 0 on EOF, -1 on timeout or error
    int read(void* buf, size_t len, int timeout_ms = -1);

    /// Write to the child's stdin.
    /// Blocks until all bytes are written or the timeout expires.
    /// @param data      source buffer
    /// @param len       bytes to write
    /// @param timeout_ms timeout in milliseconds; -1 = no timeout (block forever)
    /// @return total bytes written, or -1 on timeout / error
    int write(const void* data, size_t len, int timeout_ms = -1);

    /// Check whether the child process is still alive.
    bool isRunning();

    /// Wait for the child to exit.
    /// @param timeout_ms timeout in milliseconds; -1 = block until exit
    /// @return exit code, or -1 if the process is still running after timeout
    int wait(int timeout_ms = -1);

    /// Forcefully terminate the child process.
    void kill();

    /// Last known exit code (only valid after the process has exited).
    int exitCode() const { return exit_code_; }

    /// Close pipes and reap the process.
    /// Blocks until the process exits if `wait_for_exit` is true (default).
    /// Safe to call multiple times.
    void close(bool wait_for_exit = true);

    /// Return the full path of the current executable.
    /// Falls back to "kigstudio" on platforms where detection fails.
    static std::string self_exe_path();

private:
    // -----------------------------------------------------------------------
    // Platform-specific type aliases
    // -----------------------------------------------------------------------
#ifdef _WIN32
    using ChildHandle = HANDLE;
    using PipeHandle  = HANDLE;
    static constexpr ChildHandle INVALID_CHILD = nullptr;
    static constexpr PipeHandle  INVALID_PIPE  = nullptr;
#else
    using ChildHandle = pid_t;
    using PipeHandle  = int;
    static constexpr ChildHandle INVALID_CHILD = -1;
    static constexpr PipeHandle  INVALID_PIPE  = -1;
#endif

    ChildHandle child_handle_ = INVALID_CHILD;
    PipeHandle  stdin_pipe_   = INVALID_PIPE;  // parent writes to child
    PipeHandle  stdout_pipe_  = INVALID_PIPE;  // parent reads from child
    int         exit_code_    = -1;
    bool        running_      = false;
};

// ===========================================================================
// Windows implementation
// ===========================================================================
#ifdef _WIN32

inline std::string Process::self_exe_path() {
    char buf[MAX_PATH];
    DWORD len = GetModuleFileNameA(nullptr, buf, sizeof(buf));
    if (len > 0 && len < sizeof(buf))
        return std::string(buf, len);
    return "kigstudio.exe";
}

inline bool Process::start(const std::string& cmd) {
    close();

    HANDLE child_stdin_read  = nullptr;
    HANDLE child_stdin_write = nullptr;
    HANDLE child_stdout_read = nullptr;
    HANDLE child_stdout_write = nullptr;

    SECURITY_ATTRIBUTES sa = { sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE };

    // Create pipes. The child inherits its ends; the parent keeps the opposite ends.
    if (!CreatePipe(&child_stdin_read, &child_stdin_write, &sa, 0))
        return false;
    if (!CreatePipe(&child_stdout_read, &child_stdout_write, &sa, 0)) {
        CloseHandle(child_stdin_read);
        CloseHandle(child_stdin_write);
        return false;
    }

    // Prevent the child from inheriting the parent's ends of the pipes.
    SetHandleInformation(child_stdin_write, HANDLE_FLAG_INHERIT, 0);
    SetHandleInformation(child_stdout_read, HANDLE_FLAG_INHERIT, 0);

    // Build command line: wrap in cmd /c for shell behaviour
    std::string shell_cmd = "cmd /c \"" + cmd + "\"";
    std::vector<char> cmdline(shell_cmd.begin(), shell_cmd.end());
    cmdline.push_back('\0');

    STARTUPINFOA si = { sizeof(STARTUPINFOA) };
    si.dwFlags     = STARTF_USESTDHANDLES;
    si.hStdInput   = child_stdin_read;
    si.hStdOutput  = child_stdout_write;
    si.hStdError   = child_stdout_write;   // merge stderr → stdout

    PROCESS_INFORMATION pi = {};

    BOOL ok = CreateProcessA(
        nullptr,                // app name (from command line)
        cmdline.data(),         // command line
        nullptr, nullptr,       // process / thread security
        TRUE,                   // inherit handles
        0,                      // creation flags
        nullptr, nullptr,       // environment / cwd
        &si, &pi);

    // Close child-side pipe ends regardless of outcome
    CloseHandle(child_stdin_read);
    CloseHandle(child_stdout_write);

    if (!ok) {
        CloseHandle(child_stdin_write);
        CloseHandle(child_stdout_read);
        return false;
    }

    CloseHandle(pi.hThread);   // we don't need the thread handle

    stdin_pipe_   = child_stdin_write;
    stdout_pipe_  = child_stdout_read;
    child_handle_ = pi.hProcess;
    running_      = true;
    exit_code_    = -1;

    return true;
}

inline int Process::read(void* buf, size_t len, int timeout_ms) {
    if (stdout_pipe_ == INVALID_PIPE)
        return -1;

    auto deadline = std::chrono::steady_clock::now()
                    + std::chrono::milliseconds(timeout_ms >= 0 ? timeout_ms : 0);

    for (;;) {
        // 1. Check how much data is waiting
        DWORD available = 0;
        if (!PeekNamedPipe(stdout_pipe_, nullptr, 0, nullptr, &available, nullptr)) {
            DWORD err = GetLastError();
            if (err == ERROR_BROKEN_PIPE || err == ERROR_NO_DATA) {
                running_ = false;
                return 0;  // EOF
            }
            return -1;
        }

        if (available > 0) {
            DWORD to_read = static_cast<DWORD>(std::min(len, static_cast<size_t>(available)));
            DWORD bytes_read = 0;
            if (!ReadFile(stdout_pipe_, buf, to_read, &bytes_read, nullptr))
                return -1;
            return static_cast<int>(bytes_read);
        }

        // 2. Check whether the process has exited
        DWORD code = 0;
        if (GetExitCodeProcess(child_handle_, &code) && code != STILL_ACTIVE) {
            exit_code_ = static_cast<int>(code);
            running_   = false;
            // loop back once more to drain any remaining pipe data
            continue;
        }

        // 3. Timeout check
        if (timeout_ms >= 0) {
            auto now = std::chrono::steady_clock::now();
            if (now >= deadline)
                return -1;  // timed out with no data
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

inline int Process::write(const void* data, size_t len, int timeout_ms) {
    if (stdin_pipe_ == INVALID_PIPE)
        return -1;

    auto deadline = std::chrono::steady_clock::now()
                    + std::chrono::milliseconds(timeout_ms >= 0 ? timeout_ms : 0);

    size_t total_written = 0;
    auto* p = static_cast<const char*>(data);

    while (total_written < len) {
        // On Windows there is no portable way to wait for pipe write-space.
        // We simply attempt the write; if it fails we check the timeout.
        DWORD written = 0;
        DWORD to_write = static_cast<DWORD>(std::min(
            static_cast<size_t>(4096), len - total_written));

        if (!WriteFile(stdin_pipe_, p + total_written, to_write, &written, nullptr)) {
            DWORD err = GetLastError();
            if (err == ERROR_NO_DATA) {
                // Pipe was closed by the child
                return total_written > 0 ? static_cast<int>(total_written) : -1;
            }
            return -1;
        }

        total_written += written;

        if (total_written >= len)
            break;

        // Timeout check
        if (timeout_ms >= 0) {
            auto now = std::chrono::steady_clock::now();
            if (now >= deadline)
                return total_written > 0 ? static_cast<int>(total_written) : -1;
        }

        // Check if child is still alive (write would block forever if child died)
        DWORD code = 0;
        if (GetExitCodeProcess(child_handle_, &code) && code != STILL_ACTIVE) {
            exit_code_ = static_cast<int>(code);
            running_   = false;
            return total_written > 0 ? static_cast<int>(total_written) : -1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return static_cast<int>(total_written);
}

inline bool Process::isRunning() {
    if (!running_)
        return false;

    DWORD code = 0;
    if (GetExitCodeProcess(child_handle_, &code)) {
        if (code != STILL_ACTIVE) {
            exit_code_ = static_cast<int>(code);
            running_   = false;
            return false;
        }
        return true;
    }
    running_ = false;
    return false;
}

inline int Process::wait(int timeout_ms) {
    if (child_handle_ == INVALID_CHILD)
        return exit_code_;

    DWORD ms = (timeout_ms < 0) ? INFINITE : static_cast<DWORD>(timeout_ms);
    DWORD result = WaitForSingleObject(child_handle_, ms);

    if (result == WAIT_OBJECT_0) {
        DWORD code = 0;
        if (GetExitCodeProcess(child_handle_, &code)) {
            exit_code_ = static_cast<int>(code);
        }
        running_ = false;
        return exit_code_;
    }

    return -1;  // timeout or error
}

inline void Process::kill() {
    if (child_handle_ != INVALID_CHILD && running_) {
        TerminateProcess(child_handle_, 1);
        running_   = false;
        exit_code_ = 1;
    }
}

inline void Process::close(bool wait_for_exit) {
    // Close stdin pipe first — signals the child that input is done.
    if (stdin_pipe_ != INVALID_PIPE) {
        CloseHandle(stdin_pipe_);
        stdin_pipe_ = INVALID_PIPE;
    }

    if (child_handle_ != INVALID_CHILD) {
        if (wait_for_exit && running_) {
            WaitForSingleObject(child_handle_, INFINITE);
            DWORD code = 0;
            if (GetExitCodeProcess(child_handle_, &code))
                exit_code_ = static_cast<int>(code);
            running_ = false;
        }
        CloseHandle(child_handle_);
        child_handle_ = INVALID_CHILD;
    }

    if (stdout_pipe_ != INVALID_PIPE) {
        CloseHandle(stdout_pipe_);
        stdout_pipe_ = INVALID_PIPE;
    }
}

// ===========================================================================
// POSIX (Linux / macOS) implementation
// ===========================================================================
#else

#include <climits>

inline std::string Process::self_exe_path() {
    char buf[PATH_MAX];
    ssize_t len = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len != -1) {
        buf[len] = '\0';
        return std::string(buf);
    }
    return "kigstudio";
}

inline bool Process::start(const std::string& cmd) {
    close();

    int stdin_pair[2]  = { -1, -1 };
    int stdout_pair[2] = { -1, -1 };

    if (::pipe(stdin_pair) != 0)
        return false;
    if (::pipe(stdout_pair) != 0) {
        ::close(stdin_pair[0]);
        ::close(stdin_pair[1]);
        return false;
    }

    pid_t pid = ::fork();
    if (pid < 0) {
        ::close(stdin_pair[0]);  ::close(stdin_pair[1]);
        ::close(stdout_pair[0]); ::close(stdout_pair[1]);
        return false;
    }

    if (pid == 0) {
        // ---- child ----
        ::close(stdin_pair[1]);   // close write end
        ::close(stdout_pair[0]);  // close read end

        ::dup2(stdin_pair[0],  STDIN_FILENO);
        ::dup2(stdout_pair[1], STDOUT_FILENO);
        ::dup2(stdout_pair[1], STDERR_FILENO);   // merge stderr → stdout

        ::close(stdin_pair[0]);
        ::close(stdout_pair[1]);

        ::execl("/bin/sh", "sh", "-c", cmd.c_str(), (char*)nullptr);
        ::_exit(127);   // exec failed
    }

    // ---- parent ----
    ::close(stdin_pair[0]);   // close read end of stdin
    ::close(stdout_pair[1]);  // close write end of stdout

    stdin_pipe_   = stdin_pair[1];
    stdout_pipe_  = stdout_pair[0];
    child_handle_ = pid;
    running_      = true;
    exit_code_    = -1;

    return true;
}

inline int Process::read(void* buf, size_t len, int timeout_ms) {
    if (stdout_pipe_ == INVALID_PIPE)
        return -1;

    for (;;) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(stdout_pipe_, &fds);

        struct timeval tv;
        struct timeval* ptv = nullptr;

        if (timeout_ms >= 0) {
            tv.tv_sec  = timeout_ms / 1000;
            tv.tv_usec = (timeout_ms % 1000) * 1000;
            ptv = &tv;
        }

        int sel = ::select(stdout_pipe_ + 1, &fds, nullptr, nullptr, ptv);
        if (sel < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (sel == 0)
            return -1;   // timeout — no data available

        // Data (or EOF) is ready
        ssize_t n = ::read(stdout_pipe_, buf, len);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (n == 0) {
            running_ = false;
            return 0;   // EOF
        }
        return static_cast<int>(n);
    }
}

inline int Process::write(const void* data, size_t len, int timeout_ms) {
    if (stdin_pipe_ == INVALID_PIPE)
        return -1;

    size_t total = 0;
    auto* p = static_cast<const char*>(data);

    while (total < len) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(stdin_pipe_, &fds);

        struct timeval tv;
        struct timeval* ptv = nullptr;

        if (timeout_ms >= 0) {
            tv.tv_sec  = timeout_ms / 1000;
            tv.tv_usec = (timeout_ms % 1000) * 1000;
            ptv = &tv;
        }

        int sel = ::select(stdin_pipe_ + 1, nullptr, &fds, nullptr, ptv);
        if (sel < 0) {
            if (errno == EINTR)
                continue;
            return total > 0 ? static_cast<int>(total) : -1;
        }
        if (sel == 0)
            return total > 0 ? static_cast<int>(total) : -1;   // timeout

        ssize_t n = ::write(stdin_pipe_, p + total, len - total);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            // EPIPE: child closed its stdin
            if (errno == EPIPE) {
                running_ = false;
                return total > 0 ? static_cast<int>(total) : -1;
            }
            return total > 0 ? static_cast<int>(total) : -1;
        }
        total += static_cast<size_t>(n);
    }

    return static_cast<int>(total);
}

inline bool Process::isRunning() {
    if (!running_)
        return false;

    int status = 0;
    pid_t r = ::waitpid(child_handle_, &status, WNOHANG);
    if (r == 0)
        return true;   // still running

    if (r > 0) {
        // Child exited
        if (WIFEXITED(status))
            exit_code_ = WEXITSTATUS(status);
        else if (WIFSIGNALED(status))
            exit_code_ = -WTERMSIG(status);
        else
            exit_code_ = -1;
        running_ = false;
        return false;
    }

    // error
    running_ = false;
    return false;
}

inline int Process::wait(int timeout_ms) {
    if (child_handle_ == INVALID_CHILD)
        return exit_code_;

    if (timeout_ms < 0) {
        // Block until exit
        int status = 0;
        if (::waitpid(child_handle_, &status, 0) > 0) {
            if (WIFEXITED(status))
                exit_code_ = WEXITSTATUS(status);
            else if (WIFSIGNALED(status))
                exit_code_ = -WTERMSIG(status);
            running_ = false;
        }
        return exit_code_;
    }

    // Poll with timeout
    auto deadline = std::chrono::steady_clock::now()
                    + std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        int status = 0;
        pid_t r = ::waitpid(child_handle_, &status, WNOHANG);
        if (r > 0) {
            if (WIFEXITED(status))
                exit_code_ = WEXITSTATUS(status);
            else if (WIFSIGNALED(status))
                exit_code_ = -WTERMSIG(status);
            running_ = false;
            return exit_code_;
        }
        if (r < 0) {
            running_ = false;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return -1;   // timeout — still running
}

inline void Process::kill() {
    if (child_handle_ != INVALID_CHILD && running_) {
        ::kill(child_handle_, SIGKILL);
        // Reap the zombie immediately — the child is gone.
        int status = 0;
        ::waitpid(child_handle_, &status, 0);
        running_   = false;
        exit_code_ = -SIGKILL;
    }
}

inline void Process::close(bool wait_for_exit) {
    // Close stdin first — signals EOF to the child
    if (stdin_pipe_ != INVALID_PIPE) {
        ::close(stdin_pipe_);
        stdin_pipe_ = INVALID_PIPE;
    }

    if (child_handle_ != INVALID_CHILD) {
        if (wait_for_exit) {
            // Safe even if already reaped by kill(); waitpid returns -1 on ECHILD
            // and the > 0 guard skips overwriting exit_code_.
            int status = 0;
            if (::waitpid(child_handle_, &status, 0) > 0) {
                if (WIFEXITED(status))
                    exit_code_ = WEXITSTATUS(status);
                else if (WIFSIGNALED(status))
                    exit_code_ = -WTERMSIG(status);
                running_ = false;
            }
        }
        child_handle_ = INVALID_CHILD;
    }

    if (stdout_pipe_ != INVALID_PIPE) {
        ::close(stdout_pipe_);
        stdout_pipe_ = INVALID_PIPE;
    }
}

#endif   // _WIN32 / POSIX

}  // namespace sinriv::kigstudio
