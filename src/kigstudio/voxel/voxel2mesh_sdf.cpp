#include <algorithm>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include "kigstudio/utils/locale.h"
#include "kigstudio/voxel/voxel2mesh.h"
#include "kigstudio/voxel/voxel_EDT.h"

namespace sinriv::kigstudio::voxel {

using namespace locale;

Generator<std::tuple<Triangle, vec3f>> generateSmoothMeshFromSDF(
    sinriv::kigstudio::voxel::VoxelGrid& voxelData,
    int& numTriangles,
    std::function<void(const std::string&)> status_callback,
    bool computeNormals,
    int subdivisions,
    const sdf::SDFBase* sdf_ptr) {
    subdivisions = std::max(1, subdivisions);
    numTriangles = 0;

    status_callback(
        get_locale_string("progress.surface_nets.building_dense_grid"));

    DenseGrid dense = buildDenseGrid(voxelData, 2);

    if (dense.sx <= 1 || dense.sy <= 1 || dense.sz <= 1) {
        co_return;
    }

    // ============================================================
    // Coordinate system
    //
    // GRID SPACE:
    //     integer sampling lattice
    //
    // WORLD SPACE:
    //     actual scene/world coordinate
    //
    // IMPORTANT:
    // analytic sdf queries and mesh output MUST use
    // the SAME coordinate system.
    // ============================================================

    const float cell_size_x =
        voxelData.voxel_size.x / static_cast<float>(subdivisions);

    const float cell_size_y =
        voxelData.voxel_size.y / static_cast<float>(subdivisions);

    const float cell_size_z =
        voxelData.voxel_size.z / static_cast<float>(subdivisions);

    // ABSOLUTE WORLD SPACE origin of sampling grid
    const vec3f world_min(voxelData.global_position.x +
                              dense.min_bound.x * voxelData.voxel_size.x,

                          voxelData.global_position.y +
                              dense.min_bound.y * voxelData.voxel_size.y,

                          voxelData.global_position.z +
                              dense.min_bound.z * voxelData.voxel_size.z);

    // ============================================================
    // High resolution lattice
    // ============================================================

    const int sx = (dense.sx - 1) * subdivisions + 1;

    const int sy = (dense.sy - 1) * subdivisions + 1;

    const int sz = (dense.sz - 1) * subdivisions + 1;

    // ============================================================
    // SDF cache
    // ============================================================

    SDFGrid sdf;

    sdf.min_bound = Vec3i(0, 0, 0);
    sdf.max_bound = Vec3i(sx - 1, sy - 1, sz - 1);

    sdf.sx = sx;
    sdf.sy = sy;
    sdf.sz = sz;

    sdf.sdf.resize(static_cast<size_t>(sx) * sy * sz,
                   std::numeric_limits<float>::infinity());

    // ============================================================
    // Build base voxel sdf ONCE
    // ============================================================

    std::optional<SDFGrid> base_sdf;

    if (!sdf_ptr) {
        status_callback(
            get_locale_string("progress.surface_nets.building_voxel_sdf"));
        base_sdf.emplace(buildSDF(dense));
    }

    // ============================================================
    // Grid -> world conversion
    // ============================================================

    auto gridIndexToWorld = [&](float gx, float gy, float gz) -> vec3f {
        return vec3f(world_min.x + gx * cell_size_x,
                     world_min.y + gy * cell_size_y,
                     world_min.z + gz * cell_size_z);
    };

    // ============================================================
    // Lazy cached SDF sampling
    // ============================================================

    auto sampleSDF = [&](int x, int y, int z) -> float {
        float& cached = sdf.sdf[sdf.index(x, y, z)];

        if (std::isfinite(cached)) {
            return cached;
        }

        vec3f wp =
            gridIndexToWorld(static_cast<float>(x), static_cast<float>(y),
                             static_cast<float>(z));

        float val = 0.0f;

        // ========================================================
        // Analytic/world-space sdf
        // ========================================================

        if (sdf_ptr) {
            val = sdf_ptr->get(wp);

        } else {
            // ====================================================
            // Trilinear interpolation of discrete sdf
            // ====================================================

            float gx = static_cast<float>(x) / static_cast<float>(subdivisions);

            float gy = static_cast<float>(y) / static_cast<float>(subdivisions);

            float gz = static_cast<float>(z) / static_cast<float>(subdivisions);

            gx = std::max(0.0f, std::min(gx, static_cast<float>(dense.sx - 1)));

            gy = std::max(0.0f, std::min(gy, static_cast<float>(dense.sy - 1)));

            gz = std::max(0.0f, std::min(gz, static_cast<float>(dense.sz - 1)));

            int x0 = static_cast<int>(std::floor(gx));
            int y0 = static_cast<int>(std::floor(gy));
            int z0 = static_cast<int>(std::floor(gz));

            int x1 = std::min(x0 + 1, dense.sx - 1);
            int y1 = std::min(y0 + 1, dense.sy - 1);
            int z1 = std::min(z0 + 1, dense.sz - 1);

            float tx = gx - static_cast<float>(x0);
            float ty = gy - static_cast<float>(y0);
            float tz = gz - static_cast<float>(z0);

            auto lerp = [](float a, float b, float t) {
                return a + (b - a) * t;
            };

            float c00 =
                lerp(base_sdf->get(x0, y0, z0), base_sdf->get(x1, y0, z0), tx);

            float c10 =
                lerp(base_sdf->get(x0, y1, z0), base_sdf->get(x1, y1, z0), tx);

            float c01 =
                lerp(base_sdf->get(x0, y0, z1), base_sdf->get(x1, y0, z1), tx);

            float c11 =
                lerp(base_sdf->get(x0, y1, z1), base_sdf->get(x1, y1, z1), tx);

            float c0 = lerp(c00, c10, ty);
            float c1 = lerp(c01, c11, ty);

            val = lerp(c0, c1, tz);
        }

        cached = val;
        return val;
    };

    // ============================================================
    // Surface Nets tables
    // ============================================================

    static const int CUBE_CORNERS[8][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0},
                                           {1, 1, 0}, {0, 0, 1}, {1, 0, 1},
                                           {0, 1, 1}, {1, 1, 1}};

    static const vec3f CUBE_CORNER_VECTORS[8] = {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};

    static const int CUBE_EDGES[12][2] = {{0, 1}, {0, 2}, {0, 4}, {1, 3},
                                          {1, 5}, {2, 3}, {2, 6}, {3, 7},
                                          {4, 5}, {4, 6}, {5, 7}, {6, 7}};

    auto centroid_of_edge_intersections = [&](const float dists[8]) -> vec3f {
        vec3f sum(0, 0, 0);

        int count = 0;

        for (int e = 0; e < 12; ++e) {
            int c1 = CUBE_EDGES[e][0];
            int c2 = CUBE_EDGES[e][1];

            float d1 = dists[c1];
            float d2 = dists[c2];

            if ((d1 < 0.0f) != (d2 < 0.0f)) {
                float t = d1 / (d1 - d2);

                sum += CUBE_CORNER_VECTORS[c1] * (1.0f - t) +
                       CUBE_CORNER_VECTORS[c2] * t;

                count++;
            }
        }

        if (count == 0) {
            return vec3f(0.5f, 0.5f, 0.5f);
        }

        return sum / static_cast<float>(count);
    };

    // ============================================================
    // Analytic normal estimation
    // ============================================================

    auto estimateNormal = [&](const vec3f& p) -> vec3f {
        if (!computeNormals) {
            return vec3f(0, 0, 0);
        }

        if (!sdf_ptr) {
            return vec3f(0, 0, 0);
        }

        float dx = sdf_ptr->get(p.x + cell_size_x, p.y, p.z) -
                   sdf_ptr->get(p.x - cell_size_x, p.y, p.z);

        float dy = sdf_ptr->get(p.x, p.y + cell_size_y, p.z) -
                   sdf_ptr->get(p.x, p.y - cell_size_y, p.z);

        float dz = sdf_ptr->get(p.x, p.y, p.z + cell_size_z) -
                   sdf_ptr->get(p.x, p.y, p.z - cell_size_z);

        vec3f n(dx, dy, dz);

        if (n.length2() < 1e-12f) {
            return vec3f(0, 1, 0);
        }

        return n.normalize();
    };

    // ============================================================
    // Phase 1
    // ============================================================

    status_callback(
        get_locale_string("progress.surface_nets.extracting_vertices"));

    std::vector<vec3f> positions;
    std::vector<vec3f> normals;

    std::vector<uint32_t> grid_to_vertex(static_cast<size_t>(sx) * sy * sz,
                                         UINT32_MAX);

    int total = (sx - 1) * (sy - 1) * (sz - 1);
    int count = 0;
    int update_interval = std::max(1, total / 100);
    for (int z = 0; z < sz - 1; ++z) {
        for (int y = 0; y < sy - 1; ++y) {
            for (int x = 0; x < sx - 1; ++x) {
                count++;
                if (count % update_interval == 0) {
                    int progress = (count * 100) / total;
                    status_callback(
                        get_locale_string(
                            "progress.surface_nets.sdf_sample") +
                        " " + std::to_string(progress) + "%");
                }

                float dists[8];

                int neg = 0;

                for (int i = 0; i < 8; ++i) {
                    int cx = x + CUBE_CORNERS[i][0];

                    int cy = y + CUBE_CORNERS[i][1];

                    int cz = z + CUBE_CORNERS[i][2];

                    float d = sampleSDF(cx, cy, cz);

                    dists[i] = d;

                    if (d < 0.0f) {
                        neg++;
                    }
                }

                if (neg == 0 || neg == 8) {
                    continue;
                }

                vec3f local = centroid_of_edge_intersections(dists);

                vec3f world_pos =
                    gridIndexToWorld(x + local.x, y + local.y, z + local.z);

                vec3f normal = estimateNormal(world_pos);

                uint32_t idx = static_cast<uint32_t>(positions.size());

                positions.push_back(world_pos);
                normals.push_back(normal);

                grid_to_vertex[sdf.index(x, y, z)] = idx;
            }
        }
    }

    if (positions.empty()) {
        status_callback(
            get_locale_string("progress.surface_nets.no_surface_found"));
        co_return;
    }

    // ============================================================
    // Phase 2
    // ============================================================

    status_callback(get_locale_string("progress.surface_nets.building_faces"));

    std::vector<uint32_t> indices;

    const int x_stride = 1;
    const int y_stride = sx;
    const int z_stride = sx * sy;

    auto maybe_make_quad = [&](int p1, int p2, int axis_b_stride,
                               int axis_c_stride) {
        float d1 = sdf.sdf[p1];
        float d2 = sdf.sdf[p2];

        bool flip;

        if (d1 < 0.0f && d2 >= 0.0f) {
            flip = false;
        } else if (d1 >= 0.0f && d2 < 0.0f) {
            flip = true;
        } else {
            return;
        }

        uint32_t v1 = grid_to_vertex[p1];

        uint32_t v2 = grid_to_vertex[p1 - axis_b_stride];

        uint32_t v3 = grid_to_vertex[p1 - axis_c_stride];

        uint32_t v4 = grid_to_vertex[p1 - axis_b_stride - axis_c_stride];

        if (v1 == UINT32_MAX || v2 == UINT32_MAX || v3 == UINT32_MAX ||
            v4 == UINT32_MAX) {
            return;
        }

        auto emitTri = [&](uint32_t a, uint32_t b, uint32_t c) {
            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(c);
        };

        if (!flip) {
            emitTri(v1, v2, v4);
            emitTri(v1, v4, v3);

        } else {
            emitTri(v1, v4, v2);
            emitTri(v1, v3, v4);
        }
    };

    total = (sx - 1) * (sy - 1) * (sz - 1);
    count = 0;
    update_interval = std::max(1, total / 100);
    for (int z = 0; z < sz - 1; ++z) {
        for (int y = 0; y < sy - 1; ++y) {
            for (int x = 0; x < sx - 1; ++x) {
                count++;
                if (count % update_interval == 0) {
                    int progress = (count * 100) / total;
                    status_callback(
                        get_locale_string(
                            "progress.surface_nets.building_faces") +
                        " " + std::to_string(progress) + "%");
                }

                int p = sdf.index(x, y, z);

                if (grid_to_vertex[p] == UINT32_MAX) {
                    continue;
                }

                if (y > 0 && z > 0 && x < sx - 2) {
                    maybe_make_quad(p, p + x_stride, y_stride, z_stride);
                }

                if (x > 0 && z > 0 && y < sy - 2) {
                    maybe_make_quad(p, p + y_stride, z_stride, x_stride);
                }

                if (x > 0 && y > 0 && z < sz - 2) {
                    maybe_make_quad(p, p + z_stride, x_stride, y_stride);
                }
            }
        }
    }

    // ============================================================
    // Phase 3
    // ============================================================

    status_callback(
        get_locale_string("progress.surface_nets.emitting_triangles"));

    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        uint32_t i0 = indices[i];
        uint32_t i1 = indices[i + 1];
        uint32_t i2 = indices[i + 2];

        vec3f v1 = positions[i0];
        vec3f v2 = positions[i1];
        vec3f v3 = positions[i2];

        vec3f normal(0, 0, 0);

        if (computeNormals) {
            if (sdf_ptr) {
                normal = (normals[i0] + normals[i1] + normals[i2]).normalize();

            } else {
                normal = ((v2 - v1).cross(v3 - v1)).normalize();
            }
        }

        co_yield {Triangle(v1, v2, v3), normal};

        numTriangles++;
    }

    status_callback(get_locale_string("progress.surface_nets.done"));
}

}  // namespace sinriv::kigstudio::voxel