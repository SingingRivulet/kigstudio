#include <queue>
#include <unordered_set>
#include "voxel.h"

namespace sinriv::kigstudio::voxel {

VoxelGrid VoxelGrid::extractLitVoxels(const vec3<float>& lightPos) const {
    VoxelGrid result;

    VoxelGrid surface = this->getSurfaceVoxels();

    for (const Vec3i& voxel : surface) {
        vec3<float> center = this->voxelCenterToWorld(voxel);

        if (!this->rayOccluded(lightPos, center)) {
            result.insert(voxel);
        }
    }

    return result;
}

std::tuple<VoxelGrid, VoxelGrid> VoxelGrid::bfsSplit(
    const std::vector<Vec3i>& seeds,
    int max_distance,
    bool use_26_neighbors) const {
    VoxelGrid inside;
    VoxelGrid outside;

    inside.global_position = global_position;
    inside.voxel_size = voxel_size;

    outside.global_position = global_position;
    outside.voxel_size = voxel_size;

    // ============================================================
    // hash
    // ============================================================

    auto packVoxel = [](const Vec3i& v) -> uint64_t {
        return (uint64_t(uint32_t(v.x)) << 42) |
               (uint64_t(uint32_t(v.y)) << 21) | uint64_t(uint32_t(v.z));
    };

    // ============================================================
    // visited
    // ============================================================

    std::unordered_set<uint64_t> visited;
    visited.reserve(seeds.size() * 8);

    // ============================================================
    // BFS queue
    // ============================================================

    struct Node {
        Vec3i pos;
        uint16_t dist;
    };

    std::queue<Node> q;

    // ============================================================
    // init seeds
    // ============================================================

    for (const Vec3i& s : seeds) {
        if (!contains(s.x, s.y, s.z))
            continue;

        uint64_t key = packVoxel(s);

        if (!visited.insert(key).second)
            continue;

        q.push({s, 0});

        inside.insert(s);
    }

    // ============================================================
    // neighbors
    // ============================================================

    static const Vec3i neighbors6[6] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
    };

    static std::vector<Vec3i> neighbors26 = []() {
        std::vector<Vec3i> n;
        n.reserve(26);

        for (int z = -1; z <= 1; z++) {
            for (int y = -1; y <= 1; y++) {
                for (int x = -1; x <= 1; x++) {
                    if (x == 0 && y == 0 && z == 0)
                        continue;

                    n.emplace_back(x, y, z);
                }
            }
        }

        return n;
    }();

    const Vec3i* neighbors = nullptr;
    int neighbor_count = 0;

    if (use_26_neighbors) {
        neighbors = neighbors26.data();
        neighbor_count = 26;
    } else {
        neighbors = neighbors6;
        neighbor_count = 6;
    }

    // ============================================================
    // BFS
    // ============================================================

    while (!q.empty()) {
        Node current = q.front();
        q.pop();

        if (current.dist >= max_distance)
            continue;

        for (int i = 0; i < neighbor_count; i++) {
            Vec3i next = current.pos + neighbors[i];

            // 仅在已有 voxel 中传播
            if (!contains(next.x, next.y, next.z))
                continue;

            uint64_t key = packVoxel(next);

            if (!visited.insert(key).second)
                continue;

            inside.insert(next);

            q.push({next, static_cast<uint16_t>(current.dist + 1)});
        }
    }

    // ============================================================
    // outside = this - inside
    // chunk-level bit operation
    // ============================================================

    outside.chunks.reserve(chunks.size());

    for (const auto& [key, src_chunk] : chunks) {
        auto inside_it = inside.chunks.find(key);

        // 整个 chunk 都不在 inside
        if (inside_it == inside.chunks.end()) {
            outside.chunks.emplace(key, src_chunk);
            continue;
        }

        const Chunk& inside_chunk = inside_it->second;

        Chunk out_chunk;

        bool has_voxel = false;

        for (int i = 0; i < Chunk::WORD_COUNT; i++) {
            out_chunk.data[i] = src_chunk.data[i] & ~inside_chunk.data[i];

            has_voxel |= (out_chunk.data[i] != 0);
        }

        if (has_voxel) {
            outside.chunks.emplace(key, out_chunk);
        }
    }

    return {inside, outside};
}

std::vector<VoxelGrid> VoxelGrid::splitDisconnected(
    bool use_26_neighbors) const {
    std::vector<VoxelGrid> parts;

    if (chunks.empty())
        return parts;

    auto packVoxel = [](const Vec3i& v) -> uint64_t {
        return (uint64_t(uint32_t(v.x)) << 42) |
               (uint64_t(uint32_t(v.y)) << 21) | uint64_t(uint32_t(v.z));
    };

    static const Vec3i neighbors6[6] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
    };

    static const std::vector<Vec3i> neighbors26 = []() {
        std::vector<Vec3i> n;
        n.reserve(26);
        for (int z = -1; z <= 1; ++z) {
            for (int y = -1; y <= 1; ++y) {
                for (int x = -1; x <= 1; ++x) {
                    if (x == 0 && y == 0 && z == 0)
                        continue;
                    n.emplace_back(x, y, z);
                }
            }
        }
        return n;
    }();

    const Vec3i* neighbors = use_26_neighbors ? neighbors26.data() : neighbors6;
    const int neighbor_count =
        use_26_neighbors ? static_cast<int>(neighbors26.size()) : 6;

    std::unordered_set<uint64_t> visited;
    visited.reserve(chunks.size() * 64);

    std::queue<Vec3i> q;

    for (auto it = begin(); it != end(); ++it) {
        const Vec3i seed = *it;
        const uint64_t seed_key = packVoxel(seed);

        if (!visited.insert(seed_key).second)
            continue;

        VoxelGrid component;
        component.global_position = global_position;
        component.voxel_size = voxel_size;

        component.insert(seed);
        q.push(seed);

        while (!q.empty()) {
            Vec3i cur = q.front();
            q.pop();

            for (int i = 0; i < neighbor_count; ++i) {
                Vec3i nxt = cur + neighbors[i];

                if (!contains(nxt.x, nxt.y, nxt.z))
                    continue;

                uint64_t key = packVoxel(nxt);
                if (!visited.insert(key).second)
                    continue;

                component.insert(nxt);
                q.push(nxt);
            }
        }

        parts.emplace_back(std::move(component));
    }

    return parts;
}

}  // namespace sinriv::kigstudio::voxel