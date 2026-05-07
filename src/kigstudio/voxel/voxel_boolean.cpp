#include "voxel.h"
namespace sinriv::kigstudio::voxel {

VoxelGrid VoxelGrid::unionWith_local(const VoxelGrid& other) const {
    VoxelGrid r = *this;

    for (auto& [key, chunk] : other.chunks) {
        auto& dst = r.chunks[key];
        for (int i = 0; i < Chunk::WORD_COUNT; i++)
            dst.data[i] |= chunk.data[i];
    }
    return r;
}

VoxelGrid VoxelGrid::intersection_local(const VoxelGrid& other) const {
    VoxelGrid r;

    for (auto& [key, chunk] : chunks) {
        auto it = other.chunks.find(key);
        if (it == other.chunks.end())
            continue;

        Chunk out;
        for (int i = 0; i < Chunk::WORD_COUNT; i++)
            out.data[i] = chunk.data[i] & it->second.data[i];

        if (!out.empty())
            r.chunks[key] = out;
    }
    return r;
}

VoxelGrid VoxelGrid::difference_local(const VoxelGrid& other) const {
    VoxelGrid r;

    for (auto& [key, chunk] : chunks) {
        Chunk out = chunk;

        auto it = other.chunks.find(key);
        if (it != other.chunks.end()) {
            for (int i = 0; i < Chunk::WORD_COUNT; i++)
                out.data[i] &= ~it->second.data[i];
        }

        if (!out.empty())
            r.chunks[key] = out;
    }
    return r;
}

VoxelGrid VoxelGrid::unionWith(const VoxelGrid& other) const {
    VoxelGrid r = *this;

    for (const auto& voxel : other) {
        const vec3<float> world = other.voxelCenterToWorld(voxel);
        r.insert(r.worldToVoxel(world));
    }
    return r;
}

VoxelGrid VoxelGrid::intersection(const VoxelGrid& other) const {
    VoxelGrid r;
    r.global_position = global_position;
    r.voxel_size = voxel_size;

    for (const auto& voxel : *this) {
        const vec3<float> world = voxelCenterToWorld(voxel);
        if (other.containsWorldPoint(world)) {
            r.insert(voxel);
        }
    }
    return r;
}
VoxelGrid VoxelGrid::intersection(
    const collision::CollisionGroup& other) const {
    VoxelGrid r;
    r.global_position = global_position;
    r.voxel_size = voxel_size;

    for (const auto& voxel : *this) {
        const vec3<float> world = voxelCenterToWorld(voxel);
        if (other.contains(world)) {
            r.insert(voxel);
        }
    }
    return r;
}

VoxelGrid VoxelGrid::difference(const VoxelGrid& other) const {
    VoxelGrid r;
    r.global_position = global_position;
    r.voxel_size = voxel_size;

    for (const auto& voxel : *this) {
        const vec3<float> world = voxelCenterToWorld(voxel);
        if (!other.containsWorldPoint(world)) {
            r.insert(voxel);
        }
    }
    return r;
}
VoxelGrid VoxelGrid::difference(const collision::CollisionGroup& other) const {
    VoxelGrid r;
    r.global_position = global_position;
    r.voxel_size = voxel_size;

    for (const auto& voxel : *this) {
        const vec3<float> world = voxelCenterToWorld(voxel);
        if (!other.contains(world)) {
            r.insert(voxel);
        }
    }
    return r;
}

std::tuple<VoxelGrid, VoxelGrid> VoxelGrid::segment(
    const collision::CollisionGroup& other) const {
    VoxelGrid positive_side;
    VoxelGrid negative_side;

    positive_side.global_position = global_position;
    positive_side.voxel_size = voxel_size;
    negative_side.global_position = global_position;
    negative_side.voxel_size = voxel_size;

    for (const auto& voxel : *this) {
        const vec3<float> world_position(
            voxel.x * voxel_size.x + global_position.x,
            voxel.y * voxel_size.y + global_position.y,
            voxel.z * voxel_size.z + global_position.z);

        if (other.contains(world_position)) {
            positive_side.insert(voxel);
        } else {
            negative_side.insert(voxel);
        }
    }

    return {positive_side, negative_side};
}

std::tuple<VoxelGrid, VoxelGrid> VoxelGrid::segment(
    const Plane<float>& other) const {
    VoxelGrid positive_side;
    VoxelGrid negative_side;

    positive_side.global_position = global_position;
    positive_side.voxel_size = voxel_size;
    negative_side.global_position = global_position;
    negative_side.voxel_size = voxel_size;

    for (const auto& voxel : *this) {
        const vec3<float> world_position(
            voxel.x * voxel_size.x + global_position.x,
            voxel.y * voxel_size.y + global_position.y,
            voxel.z * voxel_size.z + global_position.z);

        if (other.getSide(world_position)) {
            positive_side.insert(voxel);
        } else {
            negative_side.insert(voxel);
        }
    }

    return {positive_side, negative_side};
}

std::tuple<VoxelGrid, VoxelGrid> VoxelGrid::segment(
    const concave::Base& other) const {
    VoxelGrid positive_side;
    VoxelGrid negative_side;

    positive_side.global_position = global_position;
    positive_side.voxel_size = voxel_size;
    negative_side.global_position = global_position;
    negative_side.voxel_size = voxel_size;

    std::string err;
    if (!other.check(err)) {
        std::cout << "Concave shape check failed: " << err << std::endl;
        return {positive_side, negative_side};
    }

    for (const auto& voxel : *this) {
        const vec3<float> world_position(
            voxel.x * voxel_size.x + global_position.x,
            voxel.y * voxel_size.y + global_position.y,
            voxel.z * voxel_size.z + global_position.z);

        if (other.contains(world_position)) {
            positive_side.insert(voxel);
        } else {
            negative_side.insert(voxel);
        }
    }

    return {positive_side, negative_side};
}

std::tuple<VoxelGrid, VoxelGrid> VoxelGrid::segment(
    const HybridSegment& other) const {
    VoxelGrid positive_side;
    VoxelGrid negative_side;

    positive_side.global_position = global_position;
    positive_side.voxel_size = voxel_size;
    negative_side.global_position = global_position;
    negative_side.voxel_size = voxel_size;

    for (const auto& voxel : *this) {
        const vec3<float> world_position(
            voxel.x * voxel_size.x + global_position.x,
            voxel.y * voxel_size.y + global_position.y,
            voxel.z * voxel_size.z + global_position.z);

        bool keep = true;
        if (other.enable_plane) {
            if (other.plane.getSide(world_position) !=
                other.use_plane_positive) {
                keep = false;
            }
        }
        if (other.enable_collision) {
            if (other.collision_group.contains(world_position) !=
                other.use_collision_inside) {
                keep = false;
            }
        }

        if (keep) {
            positive_side.insert(voxel);
        } else {
            negative_side.insert(voxel);
        }
    }

    return {positive_side, negative_side};
}
}  // namespace sinriv::kigstudio::voxel