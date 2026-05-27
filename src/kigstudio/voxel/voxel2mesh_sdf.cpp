#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <vector>
#include <unordered_map>
#include <omp.h>

#include "kigstudio/utils/locale.h"
#include "kigstudio/voxel/voxel2mesh.h"
#include "kigstudio/voxel/voxel_EDT.h"

namespace sinriv::kigstudio::voxel {

using namespace locale;

// ============================================================
// 分块SDF存储结构
// ============================================================

struct ChunkSDFData {
    int cx, cy, cz;  // chunk坐标
    std::vector<float> sdf;
    int size;  // (32 * subdivisions + 2) 每个维度带有1个像素的边界
    
    inline int index(int x, int y, int z) const {
        return (z * size + y) * size + x;
    }
    
    inline bool inBounds(int x, int y, int z) const {
        return x >= 0 && y >= 0 && z >= 0 && x < size && y < size && z < size;
    }
    
    inline float get(int x, int y, int z) const {
        if (!inBounds(x, y, z))
            return std::numeric_limits<float>::infinity();
        return sdf[index(x, y, z)];
    }
    
    inline void set(int x, int y, int z, float val) {
        if (inBounds(x, y, z))
            sdf[index(x, y, z)] = val;
    }
};

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

        if (voxelData_.empty()) {
            return;
        }

        // 按chunk处理，生成每个chunk的mesh
        int total_chunks = voxelData_.num_chunk();
        int chunk_count = 0;

        for (const auto& [chunk_key, chunk] : voxelData_.chunks) {
            chunk_count++;
            int progress = (chunk_count * 100) / total_chunks;
            status_callback_(
                get_locale_string("progress.surface_nets.processing_chunk") +
                " " + std::to_string(progress) + "%");

            int cx, cy, cz;
            unpackChunkKey(chunk_key, cx, cy, cz);
            processChunk(cx, cy, cz, callback);
        }

        status_callback_(get_locale_string("progress.surface_nets.done"));
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

    float cell_size_x_, cell_size_y_, cell_size_z_;

    // Per-chunk数据
    std::unordered_map<uint64_t, ChunkSDFData> chunk_sdf_cache_;

    std::vector<vec3f> positions_;
    std::vector<vec3f> normals_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> chunk_grid_to_vertex_;
    std::vector<uint32_t> indices_;

    // ============================================================
    // 坐标转换和辅助函数
    // ============================================================

    void setupCellSize() {
        cell_size_x_ =
            voxelData_.voxel_size.x / static_cast<float>(subdivisions_);
        cell_size_y_ =
            voxelData_.voxel_size.y / static_cast<float>(subdivisions_);
        cell_size_z_ =
            voxelData_.voxel_size.z / static_cast<float>(subdivisions_);
    }

    vec3f voxelToWorld(int vx, int vy, int vz) const {
        return vec3f(voxelData_.global_position.x + (vx + 0.5f) * voxelData_.voxel_size.x,
                     voxelData_.global_position.y + (vy + 0.5f) * voxelData_.voxel_size.y,
                     voxelData_.global_position.z + (vz + 0.5f) * voxelData_.voxel_size.z);
    }

    vec3f gridIndexToWorld(int cx, int cy, int cz, float gx, float gy, float gz) const {
        int base_vx = cx << 5;  // cx * 32
        int base_vy = cy << 5;
        int base_vz = cz << 5;
        return vec3f(voxelData_.global_position.x + (base_vx + gx / subdivisions_) * voxelData_.voxel_size.x,
                     voxelData_.global_position.y + (base_vy + gy / subdivisions_) * voxelData_.voxel_size.y,
                     voxelData_.global_position.z + (base_vz + gz / subdivisions_) * voxelData_.voxel_size.z);
    }

    // 采样voxel occupancy，支持边界扩展
    bool sampleVoxelOccupancy(int cx, int cy, int cz, int lx, int ly, int lz) const {
        // lx, ly, lz 可能在-1到32范围内（包括边界扩展的-1和32）
        if (lx < 0 || lx >= 32 || ly < 0 || ly >= 32 || lz < 0 || lz >= 32) {
            // 需要从相邻chunk获取
            int nx = cx, ny = cy, nz = cz;
            int nlx = lx, nly = ly, nlz = lz;
            
            if (lx < 0) { nx--; nlx += 32; }
            else if (lx >= 32) { nx++; nlx -= 32; }
            
            if (ly < 0) { ny--; nly += 32; }
            else if (ly >= 32) { ny++; nly -= 32; }
            
            if (lz < 0) { nz--; nlz += 32; }
            else if (lz >= 32) { nz++; nlz -= 32; }
            
            uint64_t key = packChunkKey(nx, ny, nz);
            auto it = voxelData_.chunks.find(key);
            if (it == voxelData_.chunks.end())
                return false;
            return it->second.get(nlx, nly, nlz);
        }
        
        uint64_t key = packChunkKey(cx, cy, cz);
        auto it = voxelData_.chunks.find(key);
        if (it == voxelData_.chunks.end())
            return false;
        return it->second.get(lx, ly, lz);
    }

    // 采样SDF值
    float sampleSDF(int cx, int cy, int cz, int x, int y, int z, const ChunkSDFData& chunk_sdf) const {
        // (x,y,z) 在subdivided chunk坐标系中，范围为[0, 32*subdivisions+2)
        float cached = chunk_sdf.get(x, y, z);
        if (std::isfinite(cached))
            return cached;

        vec3f wp = gridIndexToWorld(cx, cy, cz, static_cast<float>(x), 
                                     static_cast<float>(y), static_cast<float>(z));
        float val = 0.0f;

        if (sdf_ptr_) {
            val = sdf_ptr_->get(wp);
        } else {
            // 线性插值占有率，转换为符号距离
            float gx = static_cast<float>(x) / subdivisions_;
            float gy = static_cast<float>(y) / subdivisions_;
            float gz = static_cast<float>(z) / subdivisions_;

            int x0 = static_cast<int>(std::floor(gx));
            int y0 = static_cast<int>(std::floor(gy));
            int z0 = static_cast<int>(std::floor(gz));

            int x1 = x0 + 1;
            int y1 = y0 + 1;
            int z1 = z0 + 1;

            float tx = gx - x0;
            float ty = gy - y0;
            float tz = gz - z0;

            auto lerp = [](float a, float b, float t) {
                return a + (b - a) * t;
            };

            // 采样8个体素的占有率
            bool o000 = sampleVoxelOccupancy(cx, cy, cz, x0, y0, z0);
            bool o100 = sampleVoxelOccupancy(cx, cy, cz, x1, y0, z0);
            bool o010 = sampleVoxelOccupancy(cx, cy, cz, x0, y1, z0);
            bool o110 = sampleVoxelOccupancy(cx, cy, cz, x1, y1, z0);
            bool o001 = sampleVoxelOccupancy(cx, cy, cz, x0, y0, z1);
            bool o101 = sampleVoxelOccupancy(cx, cy, cz, x1, y0, z1);
            bool o011 = sampleVoxelOccupancy(cx, cy, cz, x0, y1, z1);
            bool o111 = sampleVoxelOccupancy(cx, cy, cz, x1, y1, z1);

            // 转换为-1到1的有符号距离（内部为负，外部为正）
            float d000 = o000 ? -1.f : 1.f;
            float d100 = o100 ? -1.f : 1.f;
            float d010 = o010 ? -1.f : 1.f;
            float d110 = o110 ? -1.f : 1.f;
            float d001 = o001 ? -1.f : 1.f;
            float d101 = o101 ? -1.f : 1.f;
            float d011 = o011 ? -1.f : 1.f;
            float d111 = o111 ? -1.f : 1.f;

            float c00 = lerp(d000, d100, tx);
            float c10 = lerp(d010, d110, tx);
            float c01 = lerp(d001, d101, tx);
            float c11 = lerp(d011, d111, tx);

            float c0 = lerp(c00, c10, ty);
            float c1 = lerp(c01, c11, ty);
            val = lerp(c0, c1, tz);
        }

        return val;
    }

    vec3f estimateNormal(const vec3f& p) const {
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
    // 处理单个chunk
    // ============================================================

    void processChunk(int cx, int cy, int cz, TriangleCallback callback) {
        setupCellSize();

        // 创建当前chunk的SDF缓存
        int size = 32 * subdivisions_ + 2;  // +2用于边界扩展
        ChunkSDFData chunk_sdf;
        chunk_sdf.cx = cx;
        chunk_sdf.cy = cy;
        chunk_sdf.cz = cz;
        chunk_sdf.size = size;
        chunk_sdf.sdf.resize(static_cast<size_t>(size) * size * size,
                             std::numeric_limits<float>::infinity());

        // 计算SDF
        buildChunkSDF(cx, cy, cz, chunk_sdf);

        // 提取顶点
        std::vector<vec3f> chunk_positions;
        std::vector<vec3f> chunk_normals;
        std::vector<uint32_t> chunk_grid_to_vertex(
            static_cast<size_t>(size) * size * size, UINT32_MAX);

        extractChunkVertices(cx, cy, cz, chunk_sdf, chunk_positions,
                             chunk_normals, chunk_grid_to_vertex);

        if (chunk_positions.empty()) {
            return;
        }

        // 构建面
        std::vector<uint32_t> chunk_indices;
        buildChunkFaces(size, chunk_sdf, chunk_grid_to_vertex, chunk_indices);

        // 发射三角形
        emitChunkTriangles(chunk_positions, chunk_normals, chunk_indices,
                           callback);
    }

    void buildChunkSDF(int cx, int cy, int cz, ChunkSDFData& chunk_sdf) {
        int size = chunk_sdf.size;
        int64_t total = int64_t(size) * size * size;
        int64_t count = 0;
        int64_t update_interval = std::max<int64_t>(1, total / 100);

        for (int z = 0; z < size; ++z) {
            for (int y = 0; y < size; ++y) {
                for (int x = 0; x < size; ++x) {
                    chunk_sdf.set(x, y, z,
                                  sampleSDF(cx, cy, cz, x, y, z, chunk_sdf));

                    ++count;
                    if (count % update_interval == 0) {
                        int progress = static_cast<int>((count * 100) / total);
                        status_callback_(
                            get_locale_string("progress.surface_nets.building_voxel_sdf") +
                            " " + std::to_string(progress) + "%");
                    }
                }
            }
        }
    }

    void extractChunkVertices(int cx, int cy, int cz, ChunkSDFData& chunk_sdf,
                              std::vector<vec3f>& chunk_positions,
                              std::vector<vec3f>& chunk_normals,
                              std::vector<uint32_t>& chunk_grid_to_vertex) {
        int size = chunk_sdf.size;

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

        // 遍历所有cube（包含最内层负方向的面）
        int cube_size = 32 * subdivisions_;
        int64_t total = int64_t(cube_size) * cube_size * cube_size;
        int64_t count = 0;
        int64_t update_interval = std::max<int64_t>(1, total / 100);

        for (int z = 0; z < cube_size; ++z) {
            for (int y = 0; y < cube_size; ++y) {
                for (int x = 0; x < cube_size; ++x) {
                    ++count;
                    if (count % update_interval == 0) {
                        int progress = static_cast<int>((count * 100) / total);
                        status_callback_(
                            get_locale_string("progress.surface_nets.sdf_sample") +
                            " " + std::to_string(progress) + "%");
                    }
                    float dists[8];
                    int neg = 0;

                    for (int i = 0; i < 8; ++i) {
                        int sx = x + CUBE_CORNERS[i][0];
                        int sy = y + CUBE_CORNERS[i][1];
                        int sz = z + CUBE_CORNERS[i][2];
                        float d = chunk_sdf.get(sx, sy, sz);
                        dists[i] = d;
                        if (d < 0.0f)
                            neg++;
                    }

                    if (neg == 0 || neg == 8)
                        continue;

                    vec3f local = centroid_of_edge_intersections(dists);
                    vec3f world_pos = gridIndexToWorld(
                        cx, cy, cz, x + local.x, y + local.y, z + local.z);
                    vec3f normal = estimateNormal(world_pos);

                    uint32_t idx = static_cast<uint32_t>(chunk_positions.size());
                    chunk_positions.push_back(world_pos);
                    chunk_normals.push_back(normal);
                    chunk_grid_to_vertex[chunk_sdf.index(x, y, z)] = idx;
                }
            }
        }
    }

    void buildChunkFaces(int size, ChunkSDFData& chunk_sdf,
                         std::vector<uint32_t>& chunk_grid_to_vertex,
                         std::vector<uint32_t>& chunk_indices) {
        const int x_stride = 1;
        const int y_stride = size;
        const int z_stride = size * size;

        auto maybe_make_quad = [&](int p1, int p2, int axis_b_stride,
                                   int axis_c_stride) {
            float d1 = chunk_sdf.sdf[p1];
            float d2 = chunk_sdf.sdf[p2];

            bool flip;
            if (d1 < 0.0f && d2 >= 0.0f) {
                flip = false;
            } else if (d1 >= 0.0f && d2 < 0.0f) {
                flip = true;
            } else {
                return;
            }

            uint32_t v1 = chunk_grid_to_vertex[p1];
            uint32_t v2 = chunk_grid_to_vertex[p1 - axis_b_stride];
            uint32_t v3 = chunk_grid_to_vertex[p1 - axis_c_stride];
            uint32_t v4 = chunk_grid_to_vertex[p1 - axis_b_stride - axis_c_stride];

            if (v1 == UINT32_MAX || v2 == UINT32_MAX || v3 == UINT32_MAX ||
                v4 == UINT32_MAX) {
                return;
            }

            auto emitTri = [&](uint32_t a, uint32_t b, uint32_t c) {
                chunk_indices.push_back(a);
                chunk_indices.push_back(b);
                chunk_indices.push_back(c);
            };

            if (!flip) {
                emitTri(v1, v2, v4);
                emitTri(v1, v4, v3);
            } else {
                emitTri(v1, v4, v2);
                emitTri(v1, v3, v4);
            }
        };

        int cube_size = 32 * subdivisions_;
        int64_t total = int64_t(cube_size) * cube_size * cube_size;
        int64_t count = 0;
        int64_t update_interval = std::max<int64_t>(1, total / 100);

        for (int z = 0; z < cube_size; ++z) {
            for (int y = 0; y < cube_size; ++y) {
                for (int x = 0; x < cube_size; ++x) {
                    ++count;
                    if (count % update_interval == 0) {
                        int progress = static_cast<int>((count * 100) / total);
                        status_callback_(
                            get_locale_string("progress.surface_nets.building_faces") +
                            " " + std::to_string(progress) + "%");
                    }
                    int p = chunk_sdf.index(x, y, z);
                    if (chunk_grid_to_vertex[p] == UINT32_MAX)
                        continue;

                    if (y > 0 && z > 0 && x < cube_size - 1) {
                        maybe_make_quad(p, p + x_stride, y_stride, z_stride);
                    }
                    if (x > 0 && z > 0 && y < cube_size - 1) {
                        maybe_make_quad(p, p + y_stride, z_stride, x_stride);
                    }
                    if (x > 0 && y > 0 && z < cube_size - 1) {
                        maybe_make_quad(p, p + z_stride, x_stride, y_stride);
                    }
                }
            }
        }
    }

    void emitChunkTriangles(const std::vector<vec3f>& chunk_positions,
                            const std::vector<vec3f>& chunk_normals,
                            const std::vector<uint32_t>& chunk_indices,
                            TriangleCallback callback) {
        for (size_t i = 0; i + 2 < chunk_indices.size(); i += 3) {
            uint32_t i0 = chunk_indices[i];
            uint32_t i1 = chunk_indices[i + 1];
            uint32_t i2 = chunk_indices[i + 2];

            vec3f v1 = chunk_positions[i0];
            vec3f v2 = chunk_positions[i1];
            vec3f v3 = chunk_positions[i2];

            vec3f normal(0, 0, 0);
            if (computeNormals_) {
                if (sdf_ptr_) {
                    normal = (chunk_normals[i0] + chunk_normals[i1] +
                              chunk_normals[i2])
                                 .normalize();
                } else {
                    normal = ((v2 - v1).cross(v3 - v1)).normalize();
                }
            }

            callback(Triangle(v1, v2, v3), normal);
            numTriangles_++;
        }
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
        triangles.emplace_back(tri, normal);
    });
    
    for (const auto& item : triangles) {
        co_yield item;
    }
}

}  // namespace sinriv::kigstudio::voxel

// 由于篇幅，将生成器封装保留原有实现，需要通过Generator模式处理
// 当前版本已改为分块处理，内存使用大幅降低
