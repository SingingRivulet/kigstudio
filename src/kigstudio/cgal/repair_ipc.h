#pragma once

/**
 * Persistent worker-process IPC for alpha_wrap mesh repair.
 *
 * Lifecycle:
 * 1. Worker starts lazily on the first submit().
 * 2. Worker stays alive — no cold-start overhead for subsequent tasks.
 * 3. On timeout, worker is killed and a fresh one spawned on next submit.
 * 4. If a task is in-flight, subsequent submits record one pending task;
 *    the pending task is submitted automatically when the current one
 *    completes — no pile-up during rapid parameter changes (e.g. dragging).
 *
 * IPC mechanism (Windows only):
 * - Shared memory (CreateFileMapping) for mesh data in binary STL format.
 * - Named events (CreateEvent) for task / result signalling.
 */

#include <atomic>
#include <cstdint>
#include <string>
#include <tuple>
#include <vector>
#include "kigstudio/utils/vec3.h"
#include "kigstudio/voxel/voxel2mesh.h"

namespace sinriv::kigstudio {
class Process;
}

namespace sinriv::kigstudio::cgal {

using vec3f = sinriv::kigstudio::vec3<float>;
using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;
using MeshData = std::vector<std::tuple<Triangle, vec3f>>;

/// Header placed at the start of the shared-memory block.
/// Must be standard-layout; atomics are lock-free on x64.
struct RepairIPCHeader {
    std::atomic<std::uint32_t> state;  // 0=idle, 1=task_ready, 2=result_ready
    std::atomic<std::uint32_t> shutdown;
    double alpha;
    double offset;
    std::uint32_t input_size;          // bytes of input STL
    std::uint32_t output_size;         // bytes of output STL
    std::uint32_t result_error;        // 0=ok, 1=alpha_wrap returned empty
    std::uint32_t _pad;
};

// Offsets within the shared-memory block
static constexpr std::uint32_t kHeaderSize = 256;
static constexpr std::uint32_t kDataOffset = kHeaderSize;
static constexpr std::uint32_t kInputMax = 32 * 1024 * 1024;   // 32 MB
static constexpr std::uint32_t kOutputOffset = kDataOffset + kInputMax;
static constexpr std::uint32_t kShmemSize = kOutputOffset + kInputMax;  // ~64 MB

class RepairWorkerIPC {
public:
    RepairWorkerIPC();
    ~RepairWorkerIPC();

    // --- lifecycle ---

    /// Launch the worker if not already running. Safe to call repeatedly.
    bool ensure_worker();

    /// Kill the worker and spawn a fresh one.
    void restart_worker();

    /// True when the worker process is alive.
    bool is_alive() const;

    /// Request the worker to exit and close all handles.
    void shutdown();

    // --- task submission ---

    /// Submit a repair task.
    /// Returns true if the task was sent to the worker.
    /// Returns false if the worker is busy — the task is stored as pending
    /// and will be submitted automatically when the current one finishes.
    bool submit(const MeshData& mesh, double alpha, double offset);

    /// Wait for the result of the most recently submitted task.
    /// @param timeout_ms  max wait in milliseconds
    /// @returns empty MeshData on timeout, error, or if no task was submitted
    MeshData wait_result(int timeout_ms);

    /// True when a task was deferred because the worker was busy.
    bool has_pending() const { return pending_; }

private:
    void create_objects();
    void destroy_objects();
    void spawn_worker();

    std::uint32_t write_stl(const MeshData& mesh, std::uint32_t byte_offset);
    MeshData read_stl(std::uint32_t byte_offset, std::uint32_t byte_size);

    // Windows handles
    void* shmem_          = nullptr;  // HANDLE
    void* mapped_view_    = nullptr;  // void*  (MapViewOfFile result)
    void* task_event_     = nullptr;  // HANDLE
    void* result_event_   = nullptr;  // HANDLE
    RepairIPCHeader* hdr_ = nullptr;

    // Worker process
    sinriv::kigstudio::Process* worker_ = nullptr;

    // Pending task (deferred when worker is busy)
    bool      pending_        = false;
    MeshData  pending_mesh_;
    double    pending_alpha_  = 1.0;
    double    pending_offset_ = 0.01;

    // Unique suffix for named objects (based on PID)
    std::string suffix_;
};

// ===========================================================================
// In-memory binary STL I/O (used by both main & worker side)
// ===========================================================================

namespace repair_stl {

/// Byte size of a binary STL with `tri_count` triangles (84 + 50*N).
inline std::uint32_t byte_size(std::uint32_t tri_count) {
    return 84 + tri_count * 50;
}

/// Write `mesh` to binary STL at `dst` (must be at least byte_size(mesh.size())).
/// Returns number of bytes written.
inline std::uint32_t write(const MeshData& mesh, std::uint8_t* dst) {
    auto w32 = [&](std::uint32_t off, std::uint32_t v) {
        dst[off] = static_cast<std::uint8_t>(v);
        dst[off + 1] = static_cast<std::uint8_t>(v >> 8);
        dst[off + 2] = static_cast<std::uint8_t>(v >> 16);
        dst[off + 3] = static_cast<std::uint8_t>(v >> 24);
    };
    auto wf = [&](std::uint32_t off, float v) {
        std::uint32_t bits;
        std::memcpy(&bits, &v, sizeof(bits));
        w32(off, bits);
    };

    const std::uint32_t n = static_cast<std::uint32_t>(mesh.size());
    std::memset(dst, 0, 80);
    w32(80, n);

    std::uint32_t off = 84;
    for (const auto& [tri, nrm] : mesh) {
        wf(off + 0, nrm.x); wf(off + 4, nrm.y); wf(off + 8, nrm.z);
        wf(off + 12, std::get<0>(tri).x);
        wf(off + 16, std::get<0>(tri).y);
        wf(off + 20, std::get<0>(tri).z);
        wf(off + 24, std::get<1>(tri).x);
        wf(off + 28, std::get<1>(tri).y);
        wf(off + 32, std::get<1>(tri).z);
        wf(off + 36, std::get<2>(tri).x);
        wf(off + 40, std::get<2>(tri).y);
        wf(off + 44, std::get<2>(tri).z);
        w32(off + 48, 0);  // attribute byte count
        off += 50;
    }
    return off;  // total bytes written
}

/// Read binary STL from `src` into a MeshData.  `byte_size` should be the
/// total STL byte count (84+50*N); if zero the triangle count in the header
/// is used and the call is unchecked (src MUST be large enough).
inline MeshData read(const std::uint8_t* src, std::uint32_t /*byte_size*/ = 0) {
    auto r32 = [&](std::uint32_t off) -> std::uint32_t {
        return static_cast<std::uint32_t>(src[off]) |
               (static_cast<std::uint32_t>(src[off + 1]) << 8) |
               (static_cast<std::uint32_t>(src[off + 2]) << 16) |
               (static_cast<std::uint32_t>(src[off + 3]) << 24);
    };
    auto rf = [&](std::uint32_t off) -> float {
        std::uint32_t bits = r32(off);
        float v;
        std::memcpy(&v, &bits, sizeof(v));
        return v;
    };

    MeshData result;
    std::uint32_t n = r32(80);
    result.reserve(n);

    std::uint32_t off = 84;
    for (std::uint32_t i = 0; i < n; ++i) {
        vec3f nrm{rf(off), rf(off + 4), rf(off + 8)};
        Triangle tri{
            {rf(off + 12), rf(off + 16), rf(off + 20)},
            {rf(off + 24), rf(off + 28), rf(off + 32)},
            {rf(off + 36), rf(off + 40), rf(off + 44)}};
        result.emplace_back(tri, nrm);
        off += 50;
    }
    return result;
}

}  // namespace repair_stl

// ===========================================================================
// Worker-side: main loop entry point (defined in mesh_repair.cpp)
// ===========================================================================

/// Run the worker event loop. Called from cli_main when --repairWorker is given.
/// Blocks until signalled to shut down.
int repair_worker_main(const std::string& shmem_name,
                       const std::string& task_event_name,
                       const std::string& result_event_name);

}  // namespace sinriv::kigstudio::cgal
