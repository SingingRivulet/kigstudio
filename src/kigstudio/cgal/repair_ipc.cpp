#include "kigstudio/cgal/repair_ipc.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#undef near
#undef far
#endif

#include <cstring>
#include <iostream>
#include "kigstudio/utils/process.h"

namespace sinriv::kigstudio::cgal {

// ===========================================================================
// Constructor / Destructor
// ===========================================================================

RepairWorkerIPC::RepairWorkerIPC() {
    suffix_ = "_" + std::to_string(GetCurrentProcessId());
    create_objects();
}

RepairWorkerIPC::~RepairWorkerIPC() {
    shutdown();
    destroy_objects();
}

// ===========================================================================
// Object creation / destruction
// ===========================================================================

void RepairWorkerIPC::create_objects() {
    std::string shmem_name = "Local\\KGS_RepairShm" + suffix_;
    std::string task_name  = "Local\\KGS_RepairTask" + suffix_;
    std::string result_name = "Local\\KGS_RepairResult" + suffix_;

    // Shared memory
    shmem_ = CreateFileMappingA(
        INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE,
        0, kShmemSize, shmem_name.c_str());
    if (!shmem_) {
        std::cerr << "[RepairIPC] CreateFileMapping failed: "
                  << GetLastError() << "\n";
        return;
    }

    mapped_view_ = MapViewOfFile(shmem_, FILE_MAP_ALL_ACCESS, 0, 0, kShmemSize);
    if (!mapped_view_) {
        std::cerr << "[RepairIPC] MapViewOfFile failed: "
                  << GetLastError() << "\n";
        CloseHandle(shmem_);
        shmem_ = nullptr;
        return;
    }

    // Zero the entire block
    std::memset(mapped_view_, 0, kShmemSize);
    hdr_ = reinterpret_cast<RepairIPCHeader*>(mapped_view_);
    hdr_->state.store(0, std::memory_order_release);
    hdr_->shutdown.store(0, std::memory_order_release);

    // Events (auto-reset)
    task_event_ = CreateEventA(nullptr, FALSE, FALSE, task_name.c_str());
    result_event_ = CreateEventA(nullptr, FALSE, FALSE, result_name.c_str());

    if (!task_event_ || !result_event_) {
        std::cerr << "[RepairIPC] CreateEvent failed: "
                  << GetLastError() << "\n";
    }
}

void RepairWorkerIPC::destroy_objects() {
    if (mapped_view_) { UnmapViewOfFile(mapped_view_); mapped_view_ = nullptr; }
    if (shmem_)       { CloseHandle(shmem_);       shmem_       = nullptr; }
    if (task_event_)  { CloseHandle(task_event_);  task_event_  = nullptr; }
    if (result_event_){ CloseHandle(result_event_);result_event_= nullptr; }
    hdr_ = nullptr;
}

// ===========================================================================
// Worker lifecycle
// ===========================================================================

void RepairWorkerIPC::spawn_worker() {
    if (worker_) {
        worker_->kill();
        delete worker_;
    }
    worker_ = new sinriv::kigstudio::Process();

    std::string shmem_name  = "Local\\KGS_RepairShm" + suffix_;
    std::string task_name   = "Local\\KGS_RepairTask" + suffix_;
    std::string result_name = "Local\\KGS_RepairResult" + suffix_;

    std::vector<std::string> args = {
        "--tools", "--repairWorker",
        "--shmem", shmem_name,
        "--taskEvent", task_name,
        "--resultEvent", result_name};

    if (!worker_->start(sinriv::kigstudio::Process::self_exe_path(), args)) {
        std::cerr << "[RepairIPC] failed to start worker process\n";
        delete worker_;
        worker_ = nullptr;
    }
}

bool RepairWorkerIPC::ensure_worker() {
    if (worker_ && worker_->isRunning()) return true;
    if (worker_) { delete worker_; worker_ = nullptr; }

    if (hdr_) {
        hdr_->state.store(0, std::memory_order_release);
        hdr_->shutdown.store(0, std::memory_order_release);
    }
    ResetEvent(task_event_);
    ResetEvent(result_event_);

    spawn_worker();
    return worker_ != nullptr;
}

void RepairWorkerIPC::restart_worker() {
    if (worker_) {
        worker_->kill();
        delete worker_;
        worker_ = nullptr;
    }
    if (hdr_) {
        hdr_->state.store(0, std::memory_order_release);
        hdr_->shutdown.store(0, std::memory_order_release);
    }
    ResetEvent(task_event_);
    ResetEvent(result_event_);
    spawn_worker();
}

bool RepairWorkerIPC::is_alive() const {
    return worker_ && worker_->isRunning();
}

void RepairWorkerIPC::shutdown() {
    if (worker_) {
        if (hdr_) hdr_->shutdown.store(1, std::memory_order_release);
        SetEvent(task_event_);  // wake worker to check shutdown flag
        worker_->close(true);
        delete worker_;
        worker_ = nullptr;
    }
}

// ===========================================================================
// Task submission & result
// ===========================================================================

bool RepairWorkerIPC::submit(const MeshData& mesh, double alpha,
                             double offset) {
    if (!ensure_worker()) return false;

    // If worker is busy with a previous task, defer to pending (single slot)
    auto state = hdr_->state.load(std::memory_order_acquire);
    if (state == 1 || state == 2) {
        pending_        = true;
        pending_mesh_   = mesh;
        pending_alpha_  = alpha;
        pending_offset_ = offset;
        return false;
    }

    // Write input STL directly into shared memory
    auto* base = reinterpret_cast<std::uint8_t*>(mapped_view_);
    hdr_->input_size = repair_stl::write(mesh, base + kDataOffset);
    hdr_->alpha      = alpha;
    hdr_->offset     = offset;
    hdr_->output_size = 0;
    hdr_->result_error = 0;

    hdr_->state.store(1, std::memory_order_release);  // task_ready
    SetEvent(task_event_);
    pending_ = false;
    return true;
}

MeshData RepairWorkerIPC::wait_result(int timeout_ms) {
    if (!worker_ || !worker_->isRunning()) return {};

    DWORD res = WaitForSingleObject(result_event_,
        timeout_ms > 0 ? static_cast<DWORD>(timeout_ms) : INFINITE);

    if (res != WAIT_OBJECT_0) {
        std::cerr << "[RepairIPC] worker timed out, restarting\n";
        restart_worker();
        if (pending_) {
            pending_ = false;
            submit(pending_mesh_, pending_alpha_, pending_offset_);
        }
        return {};
    }

    MeshData result;
    if (hdr_->state.load(std::memory_order_acquire) == 2 &&
        hdr_->output_size > 0 &&
        hdr_->result_error == 0) {
        auto* base = reinterpret_cast<const std::uint8_t*>(mapped_view_);
        result = repair_stl::read(base + kOutputOffset, hdr_->output_size);
    }

    hdr_->state.store(0, std::memory_order_release);  // back to idle

    // Submit deferred pending task
    if (pending_) {
        pending_ = false;
        submit(pending_mesh_, pending_alpha_, pending_offset_);
    }

    return result;
}

}  // namespace sinriv::kigstudio::cgal
