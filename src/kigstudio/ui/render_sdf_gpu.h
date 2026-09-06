#pragma once

#include <bgfx/bgfx.h>
#include <bx/math.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "kigstudio/sdf/sdf_chunked.h"
#include "kigstudio/ui/render_mesh.h"
#include "kigstudio/voxel/voxel.h"

namespace sinriv::ui::render {

// ================= RenderSdfGpu =================
// SDF 直接渲染（实验）：把 SDFChunkedGrid 上传到 GPU ——
//   chunk map：R32F 3D 纹理（point 采样），texel 编码
//     0   → 远场（缺失/uniform 远场 chunk）
//     >0  → dense brick 索引+1（brick pool 内）
//     <0  → uniform chunk 的 SDF 值（仅负值直接编码）
//   brick pool：R32F 3D 纹理（linear 采样），(bpa*34)^3 的 brick 图集；
//     每个 brick 为 34^3 = 32^3 数据 + 四周 1 体素 ghost layer（取邻 chunk
//     真实值），否则三线性采样在 brick 边界会混入图集中相邻 brick 的
//     无关 texel，在 chunk 交界面产生虚假表面。
// 渲染用一个覆盖 chunk map AABB 的代理盒，片元着色器球体追踪
// （fs_sdf_raymarch），输出与 mesh GBuffer 完全一致的 5 个 MRT 通道。
// 雕刻时 updateChunks 只重传变化的 brick/chunk map，不做 mesh 重建。
class RenderSdfGpu {
  public:
    // brick 存储边长：32 数据 + 两侧各 1 ghost 体素
    static constexpr int kBrickStore =
        sinriv::kigstudio::sdf::SDFChunk::SIZE + 2;

    RenderSdfGpu() = default;
    ~RenderSdfGpu() { release(); }
    RenderSdfGpu(const RenderSdfGpu&) = delete;
    RenderSdfGpu& operator=(const RenderSdfGpu&) = delete;

    inline bool valid() const { return bgfx::isValid(chunk_map_); }

    inline void release() {
        if (bgfx::isValid(brick_pool_)) {
            bgfx::destroy(brick_pool_);
            brick_pool_ = BGFX_INVALID_HANDLE;
        }
        if (bgfx::isValid(chunk_map_)) {
            bgfx::destroy(chunk_map_);
            chunk_map_ = BGFX_INVALID_HANDLE;
        }
        if (bgfx::isValid(box_vbh_)) {
            bgfx::destroy(box_vbh_);
            box_vbh_ = BGFX_INVALID_HANDLE;
        }
        if (bgfx::isValid(box_ibh_)) {
            bgfx::destroy(box_ibh_);
            box_ibh_ = BGFX_INVALID_HANDLE;
        }
        brick_of_chunk_.clear();
        free_bricks_.clear();
        chunk_map_cpu_.clear();
        next_brick_ = 0;
    }

    // 全量上传。返回 false 表示无法用 GPU 路径（无数据 / 超容量 /
    // 纹理格式不支持），调用方应回退 mesh 渲染。
    inline bool uploadAll(
        const sinriv::kigstudio::sdf::SDFChunkedGrid& grid) {
        namespace sdf_ns = sinriv::kigstudio::sdf;
        using sinriv::kigstudio::voxel::unpackChunkKey;

        release();
        if (grid.chunks.empty()) {
            return false;
        }
        // R32F 必须支持 3D 纹理（brick pool 需要三线性插值采样）
        const bgfx::Caps* caps = bgfx::getCaps();
        if (!caps || !(caps->formats[bgfx::TextureFormat::R32F] &
                       BGFX_CAPS_FORMAT_TEXTURE_3D)) {
            return false;
        }

        // chunk bbox
        sinriv::kigstudio::Vec3i cmin{0, 0, 0}, cmax{0, 0, 0};
        bool first = true;
        int dense_count = 0;
        for (const auto& [key, chunk] : grid.chunks) {
            int cx, cy, cz;
            unpackChunkKey(key, cx, cy, cz);
            if (first) {
                cmin = cmax = {cx, cy, cz};
                first = false;
            } else {
                cmin.x = std::min(cmin.x, cx);
                cmin.y = std::min(cmin.y, cy);
                cmin.z = std::min(cmin.z, cz);
                cmax.x = std::max(cmax.x, cx);
                cmax.y = std::max(cmax.y, cy);
                cmax.z = std::max(cmax.z, cz);
            }
            if (chunkNeedsBrick(chunk)) {
                ++dense_count;
            }
        }
        // brick pool 轴尺寸自适应（上限 12 -> 1728 bricks）
        bpa_ = 4;
        while (bpa_ < 12 && bpa_ * bpa_ * bpa_ < dense_count) {
            bpa_ += 4;
        }
        if (dense_count > bpa_ * bpa_ * bpa_) {
            std::cerr << "RenderSdfGpu: too many dense chunks (" << dense_count
                      << "), fallback to mesh" << std::endl;
            return false;
        }
        chunk_min_ = cmin;
        chunk_dim_ = {cmax.x - cmin.x + 1, cmax.y - cmin.y + 1,
                      cmax.z - cmin.z + 1};
        origin_ = grid.global_position;
        vs_ = grid.voxel_size.x;

        const uint16_t pool_texels =
            static_cast<uint16_t>(bpa_ * kBrickStore);
        brick_pool_ = bgfx::createTexture3D(
            pool_texels, pool_texels, pool_texels, false,
            bgfx::TextureFormat::R32F,
            BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP |
                BGFX_SAMPLER_W_CLAMP);
        chunk_map_ = bgfx::createTexture3D(
            static_cast<uint16_t>(chunk_dim_.x),
            static_cast<uint16_t>(chunk_dim_.y),
            static_cast<uint16_t>(chunk_dim_.z), false,
            bgfx::TextureFormat::R32F,
            BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT |
                BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP |
                BGFX_SAMPLER_W_CLAMP);
        if (!bgfx::isValid(brick_pool_) || !bgfx::isValid(chunk_map_)) {
            release();
            return false;
        }

        chunk_map_cpu_.assign(
            static_cast<size_t>(chunk_dim_.x) * chunk_dim_.y * chunk_dim_.z,
            0.0f);
        next_brick_ = 0;
        for (const auto& [key, chunk] : grid.chunks) {
            int cx, cy, cz;
            unpackChunkKey(key, cx, cy, cz);
            setChunkMapEntry(cx, cy, cz,
                             encodeChunk(grid, cx, cy, cz, chunk));
        }
        uploadChunkMap();
        return true;
    }

    // 局部更新指定 chunk（key 为 SDF chunk 坐标打包值）。
    // 会连带重传 26 邻域内持有 brick 的 chunk：它们的 ghost layer
    // 含有本 chunk 的边缘体素。key 超出当前 chunk map 覆盖范围时
    // 返回 false，调用方应改用 uploadAll 重建。
    inline bool updateChunks(
        const sinriv::kigstudio::sdf::SDFChunkedGrid& grid,
        const std::vector<uint64_t>& keys) {
        using sinriv::kigstudio::voxel::packChunkKey;
        using sinriv::kigstudio::voxel::unpackChunkKey;
        if (!valid()) {
            return false;
        }
        std::vector<uint64_t> todo;
        {
            std::unordered_set<uint64_t> seen;
            for (uint64_t key : keys) {
                int cx, cy, cz;
                unpackChunkKey(key, cx, cy, cz);
                if (cx < chunk_min_.x || cy < chunk_min_.y ||
                    cz < chunk_min_.z || cx >= chunk_min_.x + chunk_dim_.x ||
                    cy >= chunk_min_.y + chunk_dim_.y ||
                    cz >= chunk_min_.z + chunk_dim_.z) {
                    return false;
                }
                for (int dz = -1; dz <= 1; ++dz) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            const int nx = cx + dx, ny = cy + dy,
                                      nz = cz + dz;
                            if (nx < chunk_min_.x || ny < chunk_min_.y ||
                                nz < chunk_min_.z ||
                                nx >= chunk_min_.x + chunk_dim_.x ||
                                ny >= chunk_min_.y + chunk_dim_.y ||
                                nz >= chunk_min_.z + chunk_dim_.z) {
                                continue;
                            }
                            const uint64_t nkey = packChunkKey(nx, ny, nz);
                            if (seen.insert(nkey).second) {
                                todo.push_back(nkey);
                            }
                        }
                    }
                }
            }
        }
        for (uint64_t key : todo) {
            int cx, cy, cz;
            unpackChunkKey(key, cx, cy, cz);
            auto it = grid.chunks.find(key);
            if (it == grid.chunks.end()) {
                // chunk 被删除：视为远场，回收 brick
                releaseBrick(key);
                setChunkMapEntry(cx, cy, cz, 0.0f);
                continue;
            }
            setChunkMapEntry(cx, cy, cz,
                             encodeChunk(grid, cx, cy, cz, it->second));
        }
        uploadChunkMap();
        return true;
    }

    // 代理盒 + raymarch 提交到 GBuffer（uniform 布局与 mesh 路径一致）
    inline void renderGBuffer(const float* transform,
                              RenderMeshShader& shader,
                              const float* cam_local,
                              const std::array<float, 4>& base_color,
                              const std::array<float, 4>& pick_id) {
        if (!valid() || !shader.ensureSdfRaymarchProgram()) {
            return;
        }
        ensureBoxGeometry();

        // 单位盒 → chunk map AABB 的换算在顶点着色器内完成
        // （u_sdfOrigin/u_chunkMin/u_chunkDim），这里直接用模型矩阵，
        // 保证 u_model[0] 是纯模型变换，法线变换不受盒缩放污染
        bgfx::setTransform(transform);

        bgfx::setVertexBuffer(0, box_vbh_);
        bgfx::setIndexBuffer(box_ibh_);

        shader.ensureUniforms();
        bgfx::setUniform(shader.u_base_color_, base_color.data());
        const float exclude_vec[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        bgfx::setUniform(shader.u_exclude_from_tint_, exclude_vec);
        bgfx::setUniform(shader.u_pick_id_, pick_id.data());
        const float origin_vec[4] = {origin_.x, origin_.y, origin_.z, vs_};
        bgfx::setUniform(shader.u_sdf_origin_, origin_vec);
        const float cam_vec[4] = {cam_local[0], cam_local[1], cam_local[2],
                                  0.0f};
        bgfx::setUniform(shader.u_sdf_cam_pos_, cam_vec);
        const float cmin_vec[4] = {static_cast<float>(chunk_min_.x),
                                   static_cast<float>(chunk_min_.y),
                                   static_cast<float>(chunk_min_.z), 0.0f};
        bgfx::setUniform(shader.u_chunk_min_, cmin_vec);
        const float cdim_vec[4] = {static_cast<float>(chunk_dim_.x),
                                   static_cast<float>(chunk_dim_.y),
                                   static_cast<float>(chunk_dim_.z), 0.0f};
        bgfx::setUniform(shader.u_chunk_dim_, cdim_vec);
        const float pool_vec[4] = {static_cast<float>(bpa_), 0.0f, 0.0f,
                                   0.0f};
        bgfx::setUniform(shader.u_pool_info_, pool_vec);

        bgfx::setTexture(14, shader.s_chunk_map_, chunk_map_);
        bgfx::setTexture(15, shader.s_brick_pool_, brick_pool_);

        // 相机可能在盒内，不做面剔除
        bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                       BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS |
                       BGFX_STATE_MSAA);
        bgfx::submit(shader.view_id_, shader.sdf_raymarch_program_);
    }

  private:
    // uniform 且 0<=v<far 的 chunk 无法用整数编码，按 dense brick 上传
    static inline bool chunkNeedsBrick(
        const sinriv::kigstudio::sdf::SDFChunk& chunk) {
        if (chunk.type == sinriv::kigstudio::sdf::SDFChunk::Type::Dense) {
            return true;
        }
        return chunk.uniform_value >= 0.0f &&
               chunk.uniform_value < sinriv::kigstudio::sdf::kSDFChunkedFar;
    }

    inline int allocBrick(uint64_t key) {
        auto it = brick_of_chunk_.find(key);
        if (it != brick_of_chunk_.end()) {
            return it->second;
        }
        int brick;
        if (!free_bricks_.empty()) {
            brick = free_bricks_.back();
            free_bricks_.pop_back();
        } else {
            if (next_brick_ >= bpa_ * bpa_ * bpa_) {
                return -1;
            }
            brick = next_brick_++;
        }
        brick_of_chunk_[key] = brick;
        return brick;
    }

    inline void releaseBrick(uint64_t key) {
        auto it = brick_of_chunk_.find(key);
        if (it != brick_of_chunk_.end()) {
            free_bricks_.push_back(it->second);
            brick_of_chunk_.erase(it);
        }
    }

    // 上传一个 chunk（含 ghost layer），返回写入 chunk map 的编码值
    inline float encodeChunk(
        const sinriv::kigstudio::sdf::SDFChunkedGrid& grid, int cx, int cy,
        int cz, const sinriv::kigstudio::sdf::SDFChunk& chunk) {
        namespace sdf_ns = sinriv::kigstudio::sdf;
        const uint64_t key =
            sinriv::kigstudio::voxel::packChunkKey(cx, cy, cz);
        if (chunk.type == sdf_ns::SDFChunk::Type::Uniform) {
            if (chunk.uniform_value >= sdf_ns::kSDFChunkedFar) {
                releaseBrick(key);
                return 0.0f;
            }
            if (chunk.uniform_value < 0.0f) {
                releaseBrick(key);
                return chunk.uniform_value;
            }
            // 0 <= v < far：按 brick 上传（内部常量，边界取邻 chunk 值）
        }
        const int brick = allocBrick(key);
        if (brick < 0) {
            return 0.0f;
        }
        uploadBrick(brick, grid, cx, cy, cz, chunk);
        return static_cast<float>(brick) + 1.0f;
    }

    // 组装 34^3 brick（32^3 数据 + 1 体素 ghost layer）。所有 texel 限幅
    // 到 ±band（烘焙窄带宽度 8 体素）：dense chunk 内部在窄带之外存的是
    // ±1e6 远场巨值，若原样上传，三线性会把巨值混进采样场，球体追踪
    // 步进 d*0.9 瞬间过冲跳出包围盒，产生 chunk 边界的丢失误判。
    // ghost layer 取邻 chunk 真实值；邻 chunk 缺失/远场（±far）时钳制
    // 延伸本 chunk 边缘体素值，保证跨边界插值连续。
    inline void uploadBrick(
        int brick, const sinriv::kigstudio::sdf::SDFChunkedGrid& grid,
        int cx, int cy, int cz,
        const sinriv::kigstudio::sdf::SDFChunk& chunk) {
        constexpr int S = sinriv::kigstudio::sdf::SDFChunk::SIZE;  // 32
        const float cap = 8.0f * vs_;  // 烘焙窄带宽度（band=8）
        brick_buf_.resize(static_cast<size_t>(kBrickStore) * kBrickStore *
                          kBrickStore);
        for (int z = -1; z <= S; ++z) {
            for (int y = -1; y <= S; ++y) {
                for (int x = -1; x <= S; ++x) {
                    float v;
                    if (x >= 0 && x < S && y >= 0 && y < S && z >= 0 &&
                        z < S) {
                        v = chunk.get(x, y, z);
                    } else {
                        const float nv = grid.getVoxelValue(
                            cx * S + x, cy * S + y, cz * S + z);
                        // ±far（缺失/深内部 chunk）都钳制延伸
                        if (std::fabs(nv) >=
                            sinriv::kigstudio::sdf::kSDFChunkedFar) {
                            v = chunk.get(std::min(std::max(x, 0), S - 1),
                                          std::min(std::max(y, 0), S - 1),
                                          std::min(std::max(z, 0), S - 1));
                        } else {
                            v = nv;
                        }
                    }
                    v = std::min(std::max(v, -cap), cap);
                    brick_buf_[(static_cast<size_t>(z + 1) * kBrickStore +
                                (y + 1)) *
                                   kBrickStore +
                               (x + 1)] = v;
                }
            }
        }
        const int bx = brick % bpa_;
        const int by = (brick / bpa_) % bpa_;
        const int bz = brick / (bpa_ * bpa_);
        bgfx::updateTexture3D(
            brick_pool_, 0, static_cast<uint16_t>(bx * kBrickStore),
            static_cast<uint16_t>(by * kBrickStore),
            static_cast<uint16_t>(bz * kBrickStore),
            static_cast<uint16_t>(kBrickStore),
            static_cast<uint16_t>(kBrickStore),
            static_cast<uint16_t>(kBrickStore),
            bgfx::copy(brick_buf_.data(),
                       static_cast<uint32_t>(sizeof(float) *
                                             brick_buf_.size())));
    }

    inline void setChunkMapEntry(int cx, int cy, int cz, float code) {
        const int lx = cx - chunk_min_.x;
        const int ly = cy - chunk_min_.y;
        const int lz = cz - chunk_min_.z;
        chunk_map_cpu_[(static_cast<size_t>(lz) * chunk_dim_.y + ly) *
                           chunk_dim_.x +
                       lx] = code;
    }

    inline void uploadChunkMap() {
        bgfx::updateTexture3D(
            chunk_map_, 0, 0, 0, 0, static_cast<uint16_t>(chunk_dim_.x),
            static_cast<uint16_t>(chunk_dim_.y),
            static_cast<uint16_t>(chunk_dim_.z),
            bgfx::copy(chunk_map_cpu_.data(),
                       static_cast<uint32_t>(sizeof(float) *
                                             chunk_map_cpu_.size())));
    }

    inline void ensureBoxGeometry() {
        if (!box_layout_initialized_) {
            box_layout_.begin()
                .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
                .end();
            box_layout_initialized_ = true;
        }
        if (bgfx::isValid(box_vbh_)) {
            return;
        }
        // [0,1]^3 单位盒，实际范围由变换矩阵缩放
        const float verts[8][3] = {
            {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
            {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
        };
        const uint16_t indices[36] = {0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7,
                                      0, 1, 5, 0, 5, 4, 2, 3, 7, 2, 7, 6,
                                      0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5};
        box_vbh_ = bgfx::createVertexBuffer(bgfx::copy(verts, sizeof(verts)),
                                            box_layout_);
        box_ibh_ =
            bgfx::createIndexBuffer(bgfx::copy(indices, sizeof(indices)));
    }

    bgfx::TextureHandle brick_pool_ = BGFX_INVALID_HANDLE;
    bgfx::TextureHandle chunk_map_ = BGFX_INVALID_HANDLE;
    bgfx::VertexBufferHandle box_vbh_ = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle box_ibh_ = BGFX_INVALID_HANDLE;
    bgfx::VertexLayout box_layout_;
    bool box_layout_initialized_ = false;

    int bpa_ = 4;  // brick pool 每轴 brick 数
    sinriv::kigstudio::Vec3i chunk_min_{0, 0, 0};
    sinriv::kigstudio::Vec3i chunk_dim_{0, 0, 0};
    sinriv::kigstudio::vec3<float> origin_{0.f, 0.f, 0.f};
    float vs_ = 1.0f;
    std::unordered_map<uint64_t, int> brick_of_chunk_;
    std::vector<int> free_bricks_;
    int next_brick_ = 0;
    std::vector<float> chunk_map_cpu_;
    std::vector<float> brick_buf_;  // 34^3 brick 组装缓冲（避免反复分配）
};

}  // namespace sinriv::ui::render
