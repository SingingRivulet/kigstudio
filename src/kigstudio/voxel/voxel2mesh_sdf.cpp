#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <vector>
#include <omp.h>

#include "kigstudio/utils/locale.h"
#include "kigstudio/voxel/voxel2mesh.h"
#include "kigstudio/voxel/voxel_EDT.h"

namespace sinriv::kigstudio::voxel {

using namespace locale;

class SmoothMeshGenerator {
   public:
    using TriangleCallback = std::function<void(const Triangle&, const vec3f&)>;

    SmoothMeshGenerator(VoxelGrid& voxelData,
                        int& numTriangles,
                        std::function<void(const std::string&)> status_callback,
                        bool computeNormals,
                        int subdivisions,
                        const sdf::SDFBase* sdf_ptr)
        : voxelData_(voxelData),
          numTriangles_(numTriangles),
          status_callback_(std::move(status_callback)),
          computeNormals_(computeNormals),
          subdivisions_(std::max(1, subdivisions)),
          sdf_ptr_(sdf_ptr) {}

    void generate(TriangleCallback callback) {
        numTriangles_ = 0;
        buildDenseGrid();

        if (dense_.sx <= 1 || dense_.sy <= 1 || dense_.sz <= 1) {
            return;
        }

        setupCoordinateSystem();
        buildSDFCache();

        extractVertices();
        if (positions_.empty()) {
            return;
        }

        buildFaces();
        emitTriangles(callback);
    }

   private:
    // ============================================================
    // 成员变量
    // ============================================================

    VoxelGrid& voxelData_;
    int& numTriangles_;
    std::function<void(const std::string&)> status_callback_;
    bool computeNormals_;
    int subdivisions_;
    const sdf::SDFBase* sdf_ptr_;

    DenseGrid dense_;

    vec3f world_min_;
    float cell_size_x_, cell_size_y_, cell_size_z_;
    int sx_, sy_, sz_;

    SDFGrid sdf_;
    std::optional<SDFGrid> base_sdf_;

    std::vector<vec3f> positions_;
    std::vector<vec3f> normals_;
    std::vector<uint32_t> grid_to_vertex_;
    std::vector<uint32_t> indices_;

    // ============================================================
    // 坐标转换
    // ============================================================

    void setupCoordinateSystem() {
        cell_size_x_ =
            voxelData_.voxel_size.x / static_cast<float>(subdivisions_);
        cell_size_y_ =
            voxelData_.voxel_size.y / static_cast<float>(subdivisions_);
        cell_size_z_ =
            voxelData_.voxel_size.z / static_cast<float>(subdivisions_);

        world_min_ = vec3f(voxelData_.global_position.x +
                               dense_.min_bound.x * voxelData_.voxel_size.x,
                           voxelData_.global_position.y +
                               dense_.min_bound.y * voxelData_.voxel_size.y,
                           voxelData_.global_position.z +
                               dense_.min_bound.z * voxelData_.voxel_size.z);

        sx_ = (dense_.sx - 1) * subdivisions_ + 1;
        sy_ = (dense_.sy - 1) * subdivisions_ + 1;
        sz_ = (dense_.sz - 1) * subdivisions_ + 1;

        sdf_.min_bound = Vec3i(0, 0, 0);
        sdf_.max_bound = Vec3i(sx_ - 1, sy_ - 1, sz_ - 1);
        sdf_.sx = sx_;
        sdf_.sy = sy_;
        sdf_.sz = sz_;
        sdf_.sdf.resize(static_cast<size_t>(sx_) * sy_ * sz_,
                        std::numeric_limits<float>::infinity());
    }

    vec3f gridIndexToWorld(float gx, float gy, float gz) const {
        return vec3f(world_min_.x + gx * cell_size_x_,
                     world_min_.y + gy * cell_size_y_,
                     world_min_.z + gz * cell_size_z_);
    }

    // ============================================================
    // 密集网格 + SDF
    // ============================================================

    void buildDenseGrid() {
        status_callback_(
            get_locale_string("progress.surface_nets.building_dense_grid"));
        dense_ = sinriv::kigstudio::voxel::buildDenseGrid(voxelData_, 2);
    }

    void buildSDFCache() {
        if (!sdf_ptr_) {
            status_callback_(
                get_locale_string("progress.surface_nets.building_voxel_sdf"));
            base_sdf_.emplace(buildSDF(dense_));
        }
    }

    float sampleSDF(int x, int y, int z) {
        float& cached = sdf_.sdf[sdf_.index(x, y, z)];
        if (std::isfinite(cached)) {
            return cached;
        }

        vec3f wp =
            gridIndexToWorld(static_cast<float>(x), static_cast<float>(y),
                             static_cast<float>(z));
        float val = 0.0f;

        if (sdf_ptr_) {
            val = sdf_ptr_->get(wp);
        } else {
            float gx =
                static_cast<float>(x) / static_cast<float>(subdivisions_);
            float gy =
                static_cast<float>(y) / static_cast<float>(subdivisions_);
            float gz =
                static_cast<float>(z) / static_cast<float>(subdivisions_);

            gx =
                std::max(0.0f, std::min(gx, static_cast<float>(dense_.sx - 1)));
            gy =
                std::max(0.0f, std::min(gy, static_cast<float>(dense_.sy - 1)));
            gz =
                std::max(0.0f, std::min(gz, static_cast<float>(dense_.sz - 1)));

            int x0 = static_cast<int>(std::floor(gx));
            int y0 = static_cast<int>(std::floor(gy));
            int z0 = static_cast<int>(std::floor(gz));

            int x1 = std::min(x0 + 1, dense_.sx - 1);
            int y1 = std::min(y0 + 1, dense_.sy - 1);
            int z1 = std::min(z0 + 1, dense_.sz - 1);

            float tx = gx - static_cast<float>(x0);
            float ty = gy - static_cast<float>(y0);
            float tz = gz - static_cast<float>(z0);

            auto lerp = [](float a, float b, float t) {
                return a + (b - a) * t;
            };

            float c00 = lerp(base_sdf_->get(x0, y0, z0),
                             base_sdf_->get(x1, y0, z0), tx);
            float c10 = lerp(base_sdf_->get(x0, y1, z0),
                             base_sdf_->get(x1, y1, z0), tx);
            float c01 = lerp(base_sdf_->get(x0, y0, z1),
                             base_sdf_->get(x1, y0, z1), tx);
            float c11 = lerp(base_sdf_->get(x0, y1, z1),
                             base_sdf_->get(x1, y1, z1), tx);

            float c0 = lerp(c00, c10, ty);
            float c1 = lerp(c01, c11, ty);
            val = lerp(c0, c1, tz);
        }

        cached = val;
        return val;
    }

    vec3f estimateNormal(const vec3f& p) {
        if (!computeNormals_ || !sdf_ptr_) {
            return vec3f(0, 0, 0);
        }

        float dx = sdf_ptr_->get(p.x + cell_size_x_, p.y, p.z) -
                   sdf_ptr_->get(p.x - cell_size_x_, p.y, p.z);
        float dy = sdf_ptr_->get(p.x, p.y + cell_size_y_, p.z) -
                   sdf_ptr_->get(p.x, p.y - cell_size_y_, p.z);
        float dz = sdf_ptr_->get(p.x, p.y, p.z + cell_size_z_) -
                   sdf_ptr_->get(p.x, p.y, p.z - cell_size_z_);

        vec3f n(dx, dy, dz);
        if (n.length2() < 1e-12f) {
            return vec3f(0, 1, 0);
        }
        return n.normalize();
    }

    // ============================================================
    // Phase 1: 提取顶点
    // ============================================================

    void extractVertices() {
        status_callback_(
            get_locale_string("progress.surface_nets.extracting_vertices"));

        positions_.clear();
        normals_.clear();
        grid_to_vertex_.assign(static_cast<size_t>(sx_) * sy_ * sz_,
                               UINT32_MAX);

        static const int CUBE_CORNERS[8][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0},
                                               {1, 1, 0}, {0, 0, 1}, {1, 0, 1},
                                               {0, 1, 1}, {1, 1, 1}};
        static const vec3f CUBE_CORNER_VECTORS[8] = {
            {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0},
            {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};
        static const int CUBE_EDGES[12][2] = {{0, 1}, {0, 2}, {0, 4}, {1, 3},
                                              {1, 5}, {2, 3}, {2, 6}, {3, 7},
                                              {4, 5}, {4, 6}, {5, 7}, {6, 7}};

        auto centroid_of_edge_intersections =
            [&](const float dists[8]) -> vec3f {
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

            return count == 0 ? vec3f(0.5f, 0.5f, 0.5f)
                              : sum / static_cast<float>(count);
        };

        int total = (sx_ - 1) * (sy_ - 1) * (sz_ - 1);
        int count = 0;
        int update_interval = std::max(1, total / 100);

        for (int z = 0; z < sz_ - 1; ++z) {
            for (int y = 0; y < sy_ - 1; ++y) {
                for (int x = 0; x < sx_ - 1; ++x) {
                    count++;
                    if (count % update_interval == 0) {
                        int progress = (count * 100) / total;
                        status_callback_(
                            get_locale_string(
                                "progress.surface_nets.sdf_sample") +
                            " " + std::to_string(progress) + "%");
                    }

                    float dists[8];
                    int neg = 0;

                    #pragma omp parallel for
                    for (int i = 0; i < 8; ++i) {
                        int cx = x + CUBE_CORNERS[i][0];
                        int cy = y + CUBE_CORNERS[i][1];
                        int cz = z + CUBE_CORNERS[i][2];
                        float d = sampleSDF(cx, cy, cz);
                        dists[i] = d;
                        if (d < 0.0f)
                            neg++;
                    }

                    if (neg == 0 || neg == 8)
                        continue;

                    vec3f local = centroid_of_edge_intersections(dists);
                    vec3f world_pos =
                        gridIndexToWorld(x + local.x, y + local.y, z + local.z);
                    vec3f normal = estimateNormal(world_pos);

                    uint32_t idx = static_cast<uint32_t>(positions_.size());
                    positions_.push_back(world_pos);
                    normals_.push_back(normal);
                    grid_to_vertex_[sdf_.index(x, y, z)] = idx;
                }
            }
        }

        if (positions_.empty()) {
            status_callback_(
                get_locale_string("progress.surface_nets.no_surface_found"));
        }
    }

    // ============================================================
    // Phase 2: 构建面
    // ============================================================

    void buildFaces() {
        status_callback_(
            get_locale_string("progress.surface_nets.building_faces"));

        indices_.clear();

        const int x_stride = 1;
        const int y_stride = sx_;
        const int z_stride = sx_ * sy_;

        auto maybe_make_quad = [&](int p1, int p2, int axis_b_stride,
                                   int axis_c_stride) {
            float d1 = sdf_.sdf[p1];
            float d2 = sdf_.sdf[p2];

            bool flip;
            if (d1 < 0.0f && d2 >= 0.0f) {
                flip = false;
            } else if (d1 >= 0.0f && d2 < 0.0f) {
                flip = true;
            } else {
                return;
            }

            uint32_t v1 = grid_to_vertex_[p1];
            uint32_t v2 = grid_to_vertex_[p1 - axis_b_stride];
            uint32_t v3 = grid_to_vertex_[p1 - axis_c_stride];
            uint32_t v4 = grid_to_vertex_[p1 - axis_b_stride - axis_c_stride];

            if (v1 == UINT32_MAX || v2 == UINT32_MAX || v3 == UINT32_MAX ||
                v4 == UINT32_MAX) {
                return;
            }

            auto emitTri = [&](uint32_t a, uint32_t b, uint32_t c) {
                indices_.push_back(a);
                indices_.push_back(b);
                indices_.push_back(c);
            };

            if (!flip) {
                emitTri(v1, v2, v4);
                emitTri(v1, v4, v3);
            } else {
                emitTri(v1, v4, v2);
                emitTri(v1, v3, v4);
            }
        };

        int total = (sx_ - 1) * (sy_ - 1) * (sz_ - 1);
        int count = 0;
        int update_interval = std::max(1, total / 100);

        for (int z = 0; z < sz_ - 1; ++z) {
            for (int y = 0; y < sy_ - 1; ++y) {
                for (int x = 0; x < sx_ - 1; ++x) {
                    count++;
                    if (count % update_interval == 0) {
                        int progress = (count * 100) / total;
                        status_callback_(
                            get_locale_string(
                                "progress.surface_nets.building_faces") +
                            " " + std::to_string(progress) + "%");
                    }

                    int p = sdf_.index(x, y, z);
                    if (grid_to_vertex_[p] == UINT32_MAX)
                        continue;

                    if (y > 0 && z > 0 && x < sx_ - 2) {
                        maybe_make_quad(p, p + x_stride, y_stride, z_stride);
                    }
                    if (x > 0 && z > 0 && y < sy_ - 2) {
                        maybe_make_quad(p, p + y_stride, z_stride, x_stride);
                    }
                    if (x > 0 && y > 0 && z < sz_ - 2) {
                        maybe_make_quad(p, p + z_stride, x_stride, y_stride);
                    }
                }
            }
        }
    }

    // ============================================================
    // Phase 3: 发射三角形（通过回调）
    // ============================================================

    void emitTriangles(TriangleCallback callback) {
        status_callback_(
            get_locale_string("progress.surface_nets.emitting_triangles"));

        for (size_t i = 0; i + 2 < indices_.size(); i += 3) {
            uint32_t i0 = indices_[i];
            uint32_t i1 = indices_[i + 1];
            uint32_t i2 = indices_[i + 2];

            vec3f v1 = positions_[i0];
            vec3f v2 = positions_[i1];
            vec3f v3 = positions_[i2];

            vec3f normal(0, 0, 0);
            if (computeNormals_) {
                if (sdf_ptr_) {
                    normal = (normals_[i0] + normals_[i1] + normals_[i2])
                                 .normalize();
                } else {
                    normal = ((v2 - v1).cross(v3 - v1)).normalize();
                }
            }

            callback(Triangle(v1, v2, v3), normal);
            numTriangles_++;
        }

        status_callback_(get_locale_string("progress.surface_nets.done"));
    }
};

// ============================================================
// 对外接口（保持原签名兼容）
// ============================================================

Generator<std::tuple<Triangle, vec3f>> generateSmoothMeshFromSDF(
    sinriv::kigstudio::voxel::VoxelGrid& voxelData,
    int& numTriangles,
    std::function<void(const std::string&)> status_callback,
    bool computeNormals,
    int subdivisions,
    const sdf::SDFBase* sdf_ptr) {
    SmoothMeshGenerator gen(voxelData, numTriangles, std::move(status_callback),
                            computeNormals, subdivisions, sdf_ptr);
    std::vector<std::tuple<Triangle, vec3f>> triangles;
    gen.generate([&](const Triangle& tri, const vec3f& normal) {
        // co_yield std::make_tuple(tri, normal);
        triangles.push_back(std::tuple<Triangle, vec3f>(tri, normal));
    });
    for (const auto& item : triangles) {
        co_yield item;
    }
}

}  // namespace sinriv::kigstudio::voxel
