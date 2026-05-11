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

VoxelGrid VoxelGrid::fillInterior(bool use_6_neighbors) const {
    VoxelGrid result = *this;
    if (chunks.empty()) return result;

    // 1. 计算边界框
    int min_x = INT_MAX, min_y = INT_MAX, min_z = INT_MAX;
    int max_x = INT_MIN, max_y = INT_MIN, max_z = INT_MIN;
    for (auto v : *this) {
        min_x = std::min(min_x, v.x);
        min_y = std::min(min_y, v.y);
        min_z = std::min(min_z, v.z);
        max_x = std::max(max_x, v.x);
        max_y = std::max(max_y, v.y);
        max_z = std::max(max_z, v.z);
    }

    // 扩展一圈，确保外部空间被包含
    min_x -= 1; min_y -= 1; min_z -= 1;
    max_x += 1; max_y += 1; max_z += 1;

    int64_t range_x = (int64_t)max_x - min_x + 1;
    int64_t range_y = (int64_t)max_y - min_y + 1;
    int64_t range_z = (int64_t)max_z - min_z + 1;

    // 如果边界框太大，用稀疏方式
    const int64_t max_dense_cells = 200 * 1024 * 1024; // 200M
    bool use_dense = (range_x * range_y * range_z) <= max_dense_cells;

    auto inBounds = [&](int x, int y, int z) -> bool {
        return x >= min_x && x <= max_x && y >= min_y && y <= max_y && z >= min_z && z <= max_z;
    };

    // 6-连通邻居
    static const Vec3i neighbors6[6] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
    };
    // 26-连通邻居
    static const std::vector<Vec3i> neighbors26 = []() {
        std::vector<Vec3i> n;
        n.reserve(26);
        for (int z = -1; z <= 1; ++z)
            for (int y = -1; y <= 1; ++y)
                for (int x = -1; x <= 1; ++x)
                    if (x != 0 || y != 0 || z != 0)
                        n.emplace_back(x, y, z);
        return n;
    }();

    const Vec3i* neighbors = use_6_neighbors ? neighbors6 : neighbors26.data();
    const int neighbor_count = use_6_neighbors ? 6 : static_cast<int>(neighbors26.size());

    // 2. BFS 从角落标记所有外部空气
    std::queue<Vec3i> q;

    auto enqueue = [&](int x, int y, int z) {
        if (!inBounds(x, y, z)) return false;
        if (contains(x, y, z)) return false;
        return true;
    };

    if (use_dense) {
        size_t total = (size_t)(range_x * range_y * range_z);
        std::vector<bool> outside(total, false);
        auto idx = [&](int x, int y, int z) -> size_t {
            return ((size_t)(x - min_x) * (size_t)range_y + (size_t)(y - min_y)) * (size_t)range_z +
                   (size_t)(z - min_z);
        };

        if (enqueue(min_x, min_y, min_z)) {
            outside[idx(min_x, min_y, min_z)] = true;
            q.push({min_x, min_y, min_z});
        }

        while (!q.empty()) {
            Vec3i v = q.front(); q.pop();
            for (int i = 0; i < neighbor_count; ++i) {
                int nx = v.x + neighbors[i].x;
                int ny = v.y + neighbors[i].y;
                int nz = v.z + neighbors[i].z;
                if (!inBounds(nx, ny, nz)) continue;
                if (contains(nx, ny, nz)) continue;
                size_t iid = idx(nx, ny, nz);
                if (!outside[iid]) {
                    outside[iid] = true;
                    q.push({nx, ny, nz});
                }
            }
        }

        // 3. 遍历边界框，填充内部空洞
        for (int x = min_x + 1; x <= max_x - 1; ++x) {
            for (int y = min_y + 1; y <= max_y - 1; ++y) {
                for (int z = min_z + 1; z <= max_z - 1; ++z) {
                    if (contains(x, y, z)) continue;
                    if (!outside[idx(x, y, z)]) {
                        result.insert(x, y, z);
                    }
                }
            }
        }
    } else {
        auto packVoxel = [](const Vec3i& v) -> uint64_t {
            return (uint64_t(uint32_t(v.x)) << 42) |
                   (uint64_t(uint32_t(v.y)) << 21) | uint64_t(uint32_t(v.z));
        };
        std::unordered_set<uint64_t> outside;
        outside.reserve(chunks.size() * 64 * 2);

        if (enqueue(min_x, min_y, min_z)) {
            outside.insert(packVoxel({min_x, min_y, min_z}));
            q.push({min_x, min_y, min_z});
        }

        while (!q.empty()) {
            Vec3i v = q.front(); q.pop();
            for (int i = 0; i < neighbor_count; ++i) {
                int nx = v.x + neighbors[i].x;
                int ny = v.y + neighbors[i].y;
                int nz = v.z + neighbors[i].z;
                if (!inBounds(nx, ny, nz)) continue;
                if (contains(nx, ny, nz)) continue;
                uint64_t key = packVoxel({nx, ny, nz});
                if (outside.insert(key).second) {
                    q.push({nx, ny, nz});
                }
            }
        }

        for (int x = min_x + 1; x <= max_x - 1; ++x) {
            for (int y = min_y + 1; y <= max_y - 1; ++y) {
                for (int z = min_z + 1; z <= max_z - 1; ++z) {
                    if (contains(x, y, z)) continue;
                    if (outside.find(packVoxel({x, y, z})) == outside.end()) {
                        result.insert(x, y, z);
                    }
                }
            }
        }
    }

    return result;
}

}  // namespace sinriv::kigstudio::voxel