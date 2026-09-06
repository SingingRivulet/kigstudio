#include "render_voxel_list.h"

#include <algorithm>
#include <cmath>

namespace sinriv::ui::render {

namespace {

using sinriv::kigstudio::Vec3i;
using RenderVoxelItem = RenderVoxelList::RenderVoxelItem;
namespace sdf_ns = sinriv::kigstudio::sdf;
namespace voxel_ns = sinriv::kigstudio::voxel;

// 向下取整除法（b > 0），用于 SDF 体素坐标 -> 体素坐标映射
inline int floor_div(int a, int b) {
    int q = a / b;
    int r = a % b;
    return (r != 0 && r < 0) ? q - 1 : q;
}

// 取 SDFChunkedGrid，仅雕刻模式（source_type == 3）有效
inline sdf_ns::SDFChunkedGrid* sculpt_sdf(RenderVoxelItem& item) {
    if (item.source_type != 3 || !item.sdf_data) {
        return nullptr;
    }
    return dynamic_cast<sdf_ns::SDFChunkedGrid*>(item.sdf_data.get());
}

// 把 [sdf_min, sdf_max]（SDF 体素坐标闭区间）映射为体素坐标闭区间
inline void sdf_region_to_voxel(const Vec3i& sdf_min, const Vec3i& sdf_max,
                                int subdiv, Vec3i& voxel_min,
                                Vec3i& voxel_max) {
    voxel_min = Vec3i(floor_div(sdf_min.x, subdiv),
                      floor_div(sdf_min.y, subdiv),
                      floor_div(sdf_min.z, subdiv));
    voxel_max = Vec3i(floor_div(sdf_max.x, subdiv),
                      floor_div(sdf_max.y, subdiv),
                      floor_div(sdf_max.z, subdiv));
}

inline void expand_region(Vec3i& lo, Vec3i& hi, const Vec3i& a,
                          const Vec3i& b) {
    lo.x = std::min(lo.x, a.x);
    lo.y = std::min(lo.y, a.y);
    lo.z = std::min(lo.z, a.z);
    hi.x = std::max(hi.x, b.x);
    hi.y = std::max(hi.y, b.y);
    hi.z = std::max(hi.z, b.z);
}

// 把快照中的 SDF/体素 chunk 写回网格（created 集合中的 key 删除）
void apply_sculpt_snapshot(RenderVoxelItem& item,
                           sdf_ns::SDFChunkedGrid& grid,
                           const SculptSnapshot& snap) {
    for (const auto& [key, chunk] : snap.sdf_chunks) {
        grid.chunks[key] = chunk;
    }
    for (uint64_t key : snap.sdf_chunks_created) {
        grid.chunks.erase(key);
    }
    for (const auto& [key, chunk] : snap.voxel_chunks) {
        item.voxel_grid_data.chunks[key] = chunk;
    }
    for (uint64_t key : snap.voxel_chunks_created) {
        item.voxel_grid_data.chunks.erase(key);
    }
}

// 对快照覆盖的 key 集合取当前状态（undo/redo 互换用）
SculptSnapshot capture_sculpt_state(RenderVoxelItem& item,
                                    sdf_ns::SDFChunkedGrid& grid,
                                    const SculptSnapshot& reference,
                                    const std::string& desc) {
    SculptSnapshot cur;
    cur.description = desc;
    cur.region_min = reference.region_min;
    cur.region_max = reference.region_max;
    cur.has_region = reference.has_region;
    auto capture = [](const auto& src_chunks, const auto& ref_chunks,
                      const auto& ref_created, auto& out_chunks,
                      auto& out_created) {
        for (const auto& [key, chunk] : ref_chunks) {
            auto it = src_chunks.find(key);
            if (it != src_chunks.end()) {
                out_chunks.emplace(key, it->second);
            } else {
                out_created.insert(key);
            }
        }
        for (uint64_t key : ref_created) {
            auto it = src_chunks.find(key);
            if (it != src_chunks.end()) {
                out_chunks.emplace(key, it->second);
            }
        }
    };
    capture(grid.chunks, reference.sdf_chunks, reference.sdf_chunks_created,
            cur.sdf_chunks, cur.sdf_chunks_created);
    capture(item.voxel_grid_data.chunks, reference.voxel_chunks,
            reference.voxel_chunks_created, cur.voxel_chunks,
            cur.voxel_chunks_created);
    return cur;
}

}  // namespace

void RenderVoxelList::begin_sculpt_stroke(int item_id) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(item_id);
    if (it == items.end()) {
        return;
    }
    auto& item = *it->second;
    if (!sculpt_sdf(item)) {
        return;
    }
    item.sculpt_stroke_active = true;
    item.sculpt_stroke_snapshot = SculptSnapshot{};
    item.sculpt_flatten_plane_valid = false;
}

void RenderVoxelList::sculpt_dab_at(int item_id, const sdf_ns::Vec3f& pos,
                                    bool invert,
                                    const sdf_ns::Vec3f& drag_delta) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(item_id);
    if (it == items.end()) {
        return;
    }
    auto& item = *it->second;
    auto* grid = sculpt_sdf(item);
    if (!grid) {
        return;
    }
    // SDF 显示后台更新正在读同一份 SDF 数据，本次 dab 跳过避免数据竞争
    if (item.sdf_display_updating) {
        return;
    }
    if (!item.sculpt_stroke_active) {
        // 容错：未显式 begin 时隐式开笔
        item.sculpt_stroke_active = true;
        item.sculpt_stroke_snapshot = SculptSnapshot{};
        item.sculpt_flatten_plane_valid = false;
    }

    const float sdf_vs = grid->voxel_size.x;
    const int subdiv = std::max(
        1, static_cast<int>(std::lround(item.voxel_grid_data.voxel_size.x /
                                        sdf_vs)));
    const float r = item.sculpt_brush_radius;

    // 笔刷 AABB（SDF 体素坐标）：连续坐标 v = (p - gp) / sdf_vs - 0.5
    const auto to_sdf_min = [&](float p, float gp) {
        return static_cast<int>(
            std::floor((p - gp) / sdf_vs - 0.5f - r / sdf_vs));
    };
    const auto to_sdf_max = [&](float p, float gp) {
        return static_cast<int>(
            std::ceil((p - gp) / sdf_vs - 0.5f + r / sdf_vs));
    };
    const Vec3i sdf_min(to_sdf_min(pos.x, grid->global_position.x),
                        to_sdf_min(pos.y, grid->global_position.y),
                        to_sdf_min(pos.z, grid->global_position.z));
    const Vec3i sdf_max(to_sdf_max(pos.x, grid->global_position.x),
                        to_sdf_max(pos.y, grid->global_position.y),
                        to_sdf_max(pos.z, grid->global_position.z));

    Vec3i voxel_min, voxel_max;
    sdf_region_to_voxel(sdf_min, sdf_max, subdiv, voxel_min, voxel_max);

    // ============ 首次触碰的 chunk 入笔画快照 ============
    auto& snap = item.sculpt_stroke_snapshot;
    for (int cz = sdf_min.z >> 5; cz <= sdf_max.z >> 5; ++cz) {
        for (int cy = sdf_min.y >> 5; cy <= sdf_max.y >> 5; ++cy) {
            for (int cx = sdf_min.x >> 5; cx <= sdf_max.x >> 5; ++cx) {
                const uint64_t key = voxel_ns::packChunkKey(cx, cy, cz);
                if (snap.sdf_chunks.count(key) ||
                    snap.sdf_chunks_created.count(key)) {
                    continue;
                }
                auto cit = grid->chunks.find(key);
                if (cit != grid->chunks.end()) {
                    snap.sdf_chunks.emplace(key, cit->second);
                } else {
                    snap.sdf_chunks_created.insert(key);
                }
            }
        }
    }
    for (int cz = voxel_min.z >> 5; cz <= voxel_max.z >> 5; ++cz) {
        for (int cy = voxel_min.y >> 5; cy <= voxel_max.y >> 5; ++cy) {
            for (int cx = voxel_min.x >> 5; cx <= voxel_max.x >> 5; ++cx) {
                const uint64_t key = voxel_ns::packChunkKey(cx, cy, cz);
                if (snap.voxel_chunks.count(key) ||
                    snap.voxel_chunks_created.count(key)) {
                    continue;
                }
                auto cit = item.voxel_grid_data.chunks.find(key);
                if (cit != item.voxel_grid_data.chunks.end()) {
                    snap.voxel_chunks.emplace(key, cit->second);
                } else {
                    snap.voxel_chunks_created.insert(key);
                }
            }
        }
    }

    // ============ 应用笔刷（按 sculpt_brush_type 分派） ============
    Vec3i out_min, out_max;
    bool changed = false;
    if (item.sculpt_brush_type == 1) {
        // 铲平：首个 dab 锁定平面——SDF 梯度法线 + 一步牛顿投影到等值面
        if (!item.sculpt_flatten_plane_valid) {
            const float eps = sdf_vs;
            const auto g = [&](float x, float y, float z) {
                return grid->get(sdf_ns::Vec3f(x, y, z));
            };
            sdf_ns::Vec3f grad(
                g(pos.x + eps, pos.y, pos.z) - g(pos.x - eps, pos.y, pos.z),
                g(pos.x, pos.y + eps, pos.z) - g(pos.x, pos.y - eps, pos.z),
                g(pos.x, pos.y, pos.z + eps) - g(pos.x, pos.y, pos.z - eps));
            const float v0 = grid->get(pos);
            const float len = grad.length();
            if (len < 1e-6f || std::fabs(v0) >= sdf_ns::kSDFChunkedFar) {
                return;  // 空中 dab：无法定义铲平平面
            }
            grad = grad / len;
            item.sculpt_flatten_plane_origin = {
                pos.x - v0 * grad.x, pos.y - v0 * grad.y, pos.z - v0 * grad.z};
            item.sculpt_flatten_plane_normal = {grad.x, grad.y, grad.z};
            item.sculpt_flatten_plane_valid = true;
        }
        changed = grid->flattenRegion(
            pos, r, item.sculpt_smooth_strength,
            sdf_ns::Vec3f(item.sculpt_flatten_plane_origin.x,
                          item.sculpt_flatten_plane_origin.y,
                          item.sculpt_flatten_plane_origin.z),
            sdf_ns::Vec3f(item.sculpt_flatten_plane_normal.x,
                          item.sculpt_flatten_plane_normal.y,
                          item.sculpt_flatten_plane_normal.z),
            out_min, out_max);
    } else if (item.sculpt_brush_type == 2) {
        // 增减料：默认增料，invert（Shift）刻槽
        const float amount = item.sculpt_draw_amount > 0.f
                                 ? item.sculpt_draw_amount
                                 : sdf_vs;
        changed = grid->drawRegion(pos, r, item.sculpt_smooth_strength, amount,
                                   invert ? -1.0f : 1.0f, out_min, out_max);
    } else if (item.sculpt_brush_type == 3) {
        // 膨胀/收缩：平台型 falloff，invert（Shift）收缩
        const float amount = item.sculpt_draw_amount > 0.f
                                 ? item.sculpt_draw_amount
                                 : sdf_vs;
        changed = grid->inflateRegion(pos, r, item.sculpt_smooth_strength,
                                      amount, invert ? -1.0f : 1.0f, out_min,
                                      out_max);
    } else if (item.sculpt_brush_type == 4) {
        // 变形：沿鼠标拖动方向平流场
        changed = grid->moveRegion(pos, r, item.sculpt_smooth_strength,
                                   drag_delta, out_min, out_max);
    } else if (item.sculpt_brush_type == 5) {
        // 场修复：局部重距离化
        changed = grid->repairRegion(pos, r, item.sculpt_smooth_strength,
                                     out_min, out_max);
    } else {
        changed = grid->smoothRegion(pos, r, item.sculpt_smooth_strength,
                                     out_min, out_max);
    }
    if (!changed) {
        return;
    }

    // ============ 体素同步：以体素中心 SDF 值重判内外 ============
    Vec3i sync_min, sync_max;
    sdf_region_to_voxel(out_min, out_max, subdiv, sync_min, sync_max);
    std::vector<Vec3i> to_add, to_remove;
    for (int z = sync_min.z; z <= sync_max.z; ++z) {
        for (int y = sync_min.y; y <= sync_max.y; ++y) {
            for (int x = sync_min.x; x <= sync_max.x; ++x) {
                const float v =
                    grid->get(item.voxel_grid_data.voxelCenterToWorld(
                        Vec3i(x, y, z)));
                const bool inside = v < 0.f;
                const bool has = item.voxel_grid_data.contains(x, y, z);
                if (inside && !has) {
                    to_add.emplace_back(x, y, z);
                } else if (!inside && has) {
                    to_remove.emplace_back(x, y, z);
                }
            }
        }
    }
    if (!to_add.empty()) {
        item.voxel_grid_data.insertMany(to_add);
    }
    if (!to_remove.empty()) {
        item.voxel_grid_data.removeMany(to_remove);
    }

    // ============ 记录 dirty 区域（体素坐标），由 flush 节流刷新显示 ============
    if (!snap.has_region) {
        snap.region_min = sync_min;
        snap.region_max = sync_max;
        snap.has_region = true;
    } else {
        expand_region(snap.region_min, snap.region_max, sync_min, sync_max);
    }
    if (!item.sdf_region_dirty) {
        item.sdf_dirty_min = sync_min;
        item.sdf_dirty_max = sync_max;
        item.sdf_region_dirty = true;
    } else {
        expand_region(item.sdf_dirty_min, item.sdf_dirty_max, sync_min,
                      sync_max);
    }
    item.dirty = true;
    item.thumbnail_dirty = true;
}

void RenderVoxelList::apply_sculpt_payload(RenderVoxelItem& item,
                                           const SculptSnapshot& snap) {
    // 重新加载等操作后 SDF 可能已被整体替换：跳过 chunk 恢复
    auto* grid = sculpt_sdf(item);
    if (!grid) {
        return;
    }
    apply_sculpt_snapshot(item, *grid, snap);
    if (snap.has_region) {
        if (!item.sdf_region_dirty) {
            item.sdf_dirty_min = snap.region_min;
            item.sdf_dirty_max = snap.region_max;
            item.sdf_region_dirty = true;
        } else {
            expand_region(item.sdf_dirty_min, item.sdf_dirty_max,
                          snap.region_min, snap.region_max);
        }
    }
    item.dirty = true;
    item.thumbnail_dirty = true;
}

SculptSnapshot RenderVoxelList::capture_sculpt_inverse(
    RenderVoxelItem& item, const SculptSnapshot& reference) {
    auto* grid = sculpt_sdf(item);
    if (!grid) {
        return SculptSnapshot{};
    }
    return capture_sculpt_state(item, *grid, reference, reference.description);
}

void RenderVoxelList::end_sculpt_stroke(int item_id) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(item_id);
    if (it == items.end()) {
        return;
    }
    auto& item = *it->second;
    if (!item.sculpt_stroke_active) {
        return;
    }
    item.sculpt_stroke_active = false;
    auto& snap = item.sculpt_stroke_snapshot;
    if (snap.has_region) {
        // 并入主撤销栈：配置快照 + 雕刻 chunk 载荷，统一时间线
        static const char* kBrushNames[] = {"平滑", "铲平", "笔刷",
                                            "膨胀", "变形", "场修复"};
        const int bt = std::clamp(item.sculpt_brush_type, 0, 5);
        CollisionEditorSnapshot entry = capture_snapshot(item);
        snap.description = std::string("雕刻·") + kBrushNames[bt];
        entry.description = snap.description;
        entry.sculpt = std::move(snap);
        push_undo_now(item_id, entry, entry.description);
        item.dirty = true;
    }
    item.sculpt_stroke_snapshot = SculptSnapshot{};
}

void RenderVoxelList::flush_sculpt_dirty_regions() {
    std::vector<std::pair<int, std::pair<Vec3i, Vec3i>>> pending;
    {
        std::lock_guard<std::mutex> lock(locker);
        for (auto& [id, item_ptr] : items) {
            auto& item = *item_ptr;
            if (!item.sdf_region_dirty || item.sdf_display_updating) {
                continue;
            }
            // SDF 直接渲染：只把变化的 chunk 重传 GPU，不做 mesh 重建，
            // 也不置 sdf_display_updating（dab 无需等待、不丢笔画）
            if (sdf_gpu_render && !item.sdf_gpu_failed) {
                auto* grid = sculpt_sdf(item);
                if (grid) {
                    if (!item.sdf_gpu) {
                        item.sdf_gpu = std::make_unique<RenderSdfGpu>();
                        item.sdf_gpu_stale = true;
                    }
                    bool ok = true;
                    if (item.sdf_gpu_stale) {
                        ok = item.sdf_gpu->uploadAll(*grid);
                        item.sdf_gpu_stale = !ok;
                    } else {
                        // 体素脏区域（voxel_grid 坐标）→ SDF chunk key 集合
                        const int subdiv =
                            std::max(1, item.node_source_sdf_subdivisions);
                        const Vec3i& vmin = item.sdf_dirty_min;
                        const Vec3i& vmax = item.sdf_dirty_max;
                        std::vector<uint64_t> keys;
                        for (int cz = (vmin.z * subdiv) >> 5;
                             cz <= ((vmax.z + 1) * subdiv - 1) >> 5; ++cz) {
                            for (int cy = (vmin.y * subdiv) >> 5;
                                 cy <= ((vmax.y + 1) * subdiv - 1) >> 5;
                                 ++cy) {
                                for (int cx = (vmin.x * subdiv) >> 5;
                                     cx <= ((vmax.x + 1) * subdiv - 1) >> 5;
                                     ++cx) {
                                    keys.push_back(
                                        voxel_ns::packChunkKey(cx, cy, cz));
                                }
                            }
                        }
                        ok = item.sdf_gpu->updateChunks(*grid, keys);
                        // 区域超出 chunk map 覆盖范围时全量重建
                        if (!ok) {
                            ok = item.sdf_gpu->uploadAll(*grid);
                        }
                    }
                    if (ok) {
                        item.sdf_region_dirty = false;
                        continue;
                    }
                    // GPU 路径不可用：回退 mesh 重建
                    item.sdf_gpu_failed = true;
                    if (item.sdf_gpu) {
                        item.sdf_gpu->release();
                    }
                }
            }
            item.sdf_display_updating = true;
            item.sdf_region_dirty = false;
            pending.emplace_back(
                id, std::make_pair(item.sdf_dirty_min, item.sdf_dirty_max));
        }
    }
    for (const auto& [id, region] : pending) {
        int subdiv = 2;
        {
            std::lock_guard<std::mutex> lock(locker);
            auto it = items.find(id);
            if (it != items.end()) {
                subdiv = it->second->node_source_sdf_subdivisions;
            }
        }
        queue_update_sdf_region(id, region.first, region.second, subdiv);
    }
}

}  // namespace sinriv::ui::render
