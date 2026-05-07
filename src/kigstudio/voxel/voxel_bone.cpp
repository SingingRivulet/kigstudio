#include "voxel.h"
namespace sinriv::kigstudio::voxel {
VoxelGrid VoxelGrid::dilate(int radius,
                            bool use_26_neighbors) const {
    if (radius <= 0)
        return *this;

    VoxelGrid current = *this;
    for (int step = 0; step < radius; ++step) {
        VoxelGrid next;
        next.global_position = global_position;
        next.voxel_size = voxel_size;

        for (const auto& voxel : current) {
            next.insert(voxel);

            for (int dz = -1; dz <= 1; ++dz) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0 && dz == 0)
                            continue;
                        if (!use_26_neighbors &&
                            std::abs(dx) + std::abs(dy) + std::abs(dz) != 1)
                            continue;

                        next.insert(voxel.x + dx, voxel.y + dy, voxel.z + dz);
                    }
                }
            }
        }

        current = std::move(next);
    }

    return current;
}

VoxelGrid VoxelGrid::erode(int radius, bool use_26_neighbors) const {
    if (radius <= 0)
        return *this;

    VoxelGrid current = *this;
    for (int step = 0; step < radius; ++step) {
        VoxelGrid next;
        next.global_position = global_position;
        next.voxel_size = voxel_size;

        for (const auto& voxel : current) {
            bool keep = true;

            for (int dz = -1; dz <= 1 && keep; ++dz) {
                for (int dy = -1; dy <= 1 && keep; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0 && dz == 0)
                            continue;
                        if (!use_26_neighbors &&
                            std::abs(dx) + std::abs(dy) + std::abs(dz) != 1)
                            continue;

                        if (!current.contains(voxel.x + dx, voxel.y + dy,
                                              voxel.z + dz)) {
                            keep = false;
                            break;
                        }
                    }
                }
            }

            if (keep)
                next.insert(voxel);
        }

        current = std::move(next);
    }

    return current;
}

VoxelGrid VoxelGrid::getSurfaceVoxels(bool use_26_neighbors) const {
    VoxelGrid surface;
    surface.global_position = global_position;
    surface.voxel_size = voxel_size;

    for (const auto& voxel : *this) {
        bool touches_outside = false;

        for (int dz = -1; dz <= 1 && !touches_outside; ++dz) {
            for (int dy = -1; dy <= 1 && !touches_outside; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;
                    if (!use_26_neighbors &&
                        std::abs(dx) + std::abs(dy) + std::abs(dz) != 1)
                        continue;

                    if (!contains(voxel.x + dx, voxel.y + dy, voxel.z + dz)) {
                        touches_outside = true;
                        break;
                    }
                }
            }
        }

        if (touches_outside)
            surface.insert(voxel);
    }

    return surface;
}

VoxelGrid VoxelGrid::getOuterAirSurfaceVoxels(
    bool use_26_neighbors) const {
    VoxelGrid air;
    air.global_position = global_position;
    air.voxel_size = voxel_size;

    for (const auto& voxel : *this) {
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;
                    if (!use_26_neighbors &&
                        std::abs(dx) + std::abs(dy) + std::abs(dz) != 1)
                        continue;

                    const Vec3i neighbor(voxel.x + dx, voxel.y + dy,
                                         voxel.z + dz);
                    if (!contains(neighbor.x, neighbor.y, neighbor.z))
                        air.insert(neighbor);
                }
            }
        }
    }

    return air;
}

VoxelGrid VoxelGrid::detectWeakVoxels(int radius,
                                      bool use_26_neighbors) const {
    if (radius <= 0) {
        VoxelGrid r;
        r.global_position = global_position;
        r.voxel_size = voxel_size;
        return r;
    }

    const VoxelGrid opened =
        erode(radius, use_26_neighbors).dilate(radius, use_26_neighbors);
    VoxelGrid weak = difference_local(opened);
    weak.global_position = global_position;
    weak.voxel_size = voxel_size;
    return weak;
}

VoxelGrid VoxelGrid::extractSkeletonByMaximalBalls(int min_radius,
                                        bool use_26_neighbors) const {
    VoxelGrid skeleton;
    skeleton.global_position = global_position;
    skeleton.voxel_size = voxel_size;

    if (chunks.empty())
        return skeleton;

    const VoxelGrid air_surface = getOuterAirSurfaceVoxels(use_26_neighbors);
    if (air_surface.chunks.empty())
        return skeleton;

    kdtree::pointVec surface_points;
    for (const auto& voxel : air_surface) {
        surface_points.push_back({static_cast<double>(voxel.x),
                                  static_cast<double>(voxel.y),
                                  static_cast<double>(voxel.z)});
    }

    kdtree::KDTree surface_tree(surface_points);
    const double min_radius_clamped =
        static_cast<double>(min_radius < 1 ? 1 : min_radius);

    struct Vec3iHash {
        std::size_t operator()(const Vec3i& p) const {
            std::size_t h = 1469598103934665603ull;
            auto mix = [&h](int32_t v) {
                h ^= static_cast<uint32_t>(v);
                h *= 1099511628211ull;
            };
            mix(p.x);
            mix(p.y);
            mix(p.z);
            return h;
        }
    };

    std::unordered_map<Vec3i, double, Vec3iHash> radius_by_voxel;
    auto isNeighborEnabled = [use_26_neighbors](int dx, int dy, int dz) {
        if (dx == 0 && dy == 0 && dz == 0)
            return false;
        if (use_26_neighbors)
            return true;
        return std::abs(dx) + std::abs(dy) + std::abs(dz) == 1;
    };
    auto squaredDistance = [](const kdtree::point_t& a, const Vec3i& b) {
        const double dx = a[0] - static_cast<double>(b.x);
        const double dy = a[1] - static_cast<double>(b.y);
        const double dz = a[2] - static_cast<double>(b.z);
        return dx * dx + dy * dy + dz * dz;
    };

    for (const auto& voxel : *this) {
        const kdtree::point_t nearest = surface_tree.nearest_point(
            {static_cast<double>(voxel.x), static_cast<double>(voxel.y),
             static_cast<double>(voxel.z)});
        const double radius = std::sqrt(squaredDistance(nearest, voxel));
        radius_by_voxel[voxel] = radius;
    }

    for (const auto& voxel : *this) {
        const double radius = radius_by_voxel[voxel];
        if (radius + 1e-6 < min_radius_clamped)
            continue;

        bool covered_by_larger_ball = false;
        for (int dz = -1; dz <= 1 && !covered_by_larger_ball; ++dz) {
            for (int dy = -1; dy <= 1 && !covered_by_larger_ball; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (!isNeighborEnabled(dx, dy, dz))
                        continue;

                    const Vec3i neighbor(voxel.x + dx, voxel.y + dy,
                                         voxel.z + dz);
                    const auto it = radius_by_voxel.find(neighbor);
                    if (it == radius_by_voxel.end())
                        continue;

                    const double step_distance = std::sqrt(
                        static_cast<double>(dx * dx + dy * dy + dz * dz));
                    if (it->second + 1e-6 >= radius + step_distance) {
                        covered_by_larger_ball = true;
                        break;
                    }
                }
            }
        }

        if (!covered_by_larger_ball)
            skeleton.insert(voxel);
    }

    return skeleton;
}
}  // namespace sinriv::kigstudio::voxel