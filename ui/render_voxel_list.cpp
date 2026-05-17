#include "render_voxel_list.h"
#include "kigstudio/sdf/sdf_chain_joint.h"
#include <chrono>
#include <cstring>
#include <limits>
namespace sinriv::ui::render {

void RenderVoxelList::setViewportSize(int width, int height) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->mesh_renderer.setViewportSize(width, height);
        it->second->voxel_renderer.setViewportSize(width, height);
    }
}

void RenderVoxelList::setViewProjection(const float* view, const float* proj) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->mesh_renderer.setViewProjection(view, proj);
        it->second->voxel_renderer.setViewProjection(view, proj);
    }
}

void RenderVoxelList::setModelMatrix(const mat4f& model_matrix) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->mesh_renderer.setModelMatrix(model_matrix);
        it->second->voxel_renderer.setModelMatrix(model_matrix);
    }
}

void RenderVoxelList::setMeshAxisVisible(bool visible) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->mesh_renderer.showAxis = visible;
    }
}

void RenderVoxelList::setVoxelAxisVisible(bool visible) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->voxel_renderer.showAxis = visible;
    }
}

void RenderVoxelList::setMeshVisible(bool visible) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->showMesh = visible;
    }
}
void RenderVoxelList::setVoxelsVisible(bool visible) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->showVoxel = visible;
    }
}
void RenderVoxelList::setCollisionVisible(bool visible) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->showCollision = visible;
    }
}
void RenderVoxelList::setCollisionBoundsVisible(bool visible) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it != items.end()) {
        it->second->showCollisionBounds = visible;
    }
}

// 交互

RenderVoxelList::RenderVoxelItem* RenderVoxelList::create_item() {
    auto item = std::make_unique<RenderVoxelItem>();
    item->manager = this;
    item->id = current_id++;
    std::cout << "[create_item] id=" << item->id
              << " write_count=" << item->write_count.load()
              << " ref_count=" << item->ref_count.load() << std::endl;
    auto item_ptr = item.get();
    {
        std::lock_guard<std::mutex> lock(locker);
        items[item->id] = std::move(item);
        update_nav_node_status = true;
    }
    return item_ptr;
}

void RenderVoxelList::setRenderId(int id) {
    std::lock_guard<std::mutex> lock(locker);
    this->setRenderId_unsafe(id);
}

void RenderVoxelList::setRenderId_unsafe(int id) {
    auto it = items.find(id);
    if (it != items.end()) {
        render_id = id;
    }
}

void RenderVoxelList::brush_marked_voxels(
    const sinriv::kigstudio::voxel::vec3f& world_pos,
    float range, bool remove) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it == items.end()) return;
    auto& item = *it->second;
    if (!item.voxel_picking_enabled || !item.surface_cache_ready) return;

    using clock = std::chrono::high_resolution_clock;
    using ms = std::chrono::duration<double, std::milli>;
    clock::time_point t0, t1, t2, t3;
    if (debug.show_voxel_pick_debug) {
        t0 = clock::now();
    }

    item.marked_voxels.global_position = item.voxel_grid_data.global_position;
    item.marked_voxels.voxel_size = item.voxel_grid_data.voxel_size;

    auto center = item.voxel_grid_data.worldToVoxel(world_pos);
    if (debug.show_voxel_pick_debug) {
        t1 = clock::now();
    }

    float range_sq = range * range;
    std::vector<sinriv::kigstudio::voxel::Vec3i> to_mark;
    for (const auto& v : item.surface_voxels) {
        float dx = float(v.x - center.x);
        float dy = float(v.y - center.y);
        float dz = float(v.z - center.z);
        if (dx * dx + dy * dy + dz * dz <= range_sq) {
            to_mark.push_back(v);
        }
    }
    if (debug.show_voxel_pick_debug) {
        t2 = clock::now();
    }

    if (!to_mark.empty()) {
        if (remove) {
            item.marked_voxels.removeMany(to_mark);
        } else {
            item.marked_voxels.insertMany(to_mark);
        }
        item.marked_voxels_dirty = true;
    }
    if (debug.show_voxel_pick_debug) {
        t3 = clock::now();
        Debug::VoxelPickTiming timing;
        timing.world_to_voxel_ms = ms(t1 - t0).count();
        timing.iterate_surface_ms = ms(t2 - t1).count();
        timing.mark_voxels_ms = ms(t3 - t2).count();
        timing.total_ms = ms(t3 - t0).count();
        debug.voxel_pick_timings.push_back(timing);
        if (debug.voxel_pick_timings.size() > debug.max_voxel_pick_timings) {
            debug.voxel_pick_timings.erase(debug.voxel_pick_timings.begin());
        }
    }
}

void RenderVoxelList::update_mouse_pos(RenderDeferred& deferred_renderer) {
    if (mouse_world_pos_picked_auto_snapping &&
        deferred_renderer.mouse_highlight_[0] > 0.5f) {
        auto it = items.find(render_id);
        if (it != items.end()) {
            auto kdtree_ptr = &it->second->mesh_kd_tree;
            if (kdtree_ptr->empty()) {
                auto pit = items.find(it->second->root_id);
                if (pit != items.end()) {
                    kdtree_ptr = &pit->second->mesh_kd_tree;
                }
            }
            if (!kdtree_ptr->empty()) {
                auto p = kdtree_ptr->nearest_point(
                    {deferred_renderer.mouse_pos_[0],
                     deferred_renderer.mouse_pos_[1],
                     deferred_renderer.mouse_pos_[2]});
                deferred_renderer.mouse_pos_[0] = p[0];
                deferred_renderer.mouse_pos_[1] = p[1];
                deferred_renderer.mouse_pos_[2] = p[2];
            }
        }
    }
    mouse_world_pos_valid = deferred_renderer.mouse_highlight_[0] > 0.5f;
    if (mouse_world_pos_valid) {
        mouse_world_pos.x = deferred_renderer.mouse_pos_[0];
        mouse_world_pos.y = deferred_renderer.mouse_pos_[1];
        mouse_world_pos.z = deferred_renderer.mouse_pos_[2];
    } else {
        mouse_world_pos = {0, 0, 0};
    }
}

void RenderVoxelList::pick_skeleton_point_from_mouse() {
    if (!mouse_world_pos_picked || !mouse_world_pos_valid)
        return;

    auto it = items.find(render_id);
    if (it == items.end())
        return;

    auto& item = *it->second;
    const float pick_range = std::max(mouse_highlight_range, item.voxel_pick_range);

    // 优先使用 skeleton_order_cache + kd树 查询最近的骨架点
    if (!item.skeleton_order_cache.empty()) {
        kdtree::pointVec points;
        points.reserve(item.skeleton_order_cache.size());
        for (const auto& sp : item.skeleton_order_cache) {
            points.push_back({static_cast<double>(sp.position.x),
                              static_cast<double>(sp.position.y),
                              static_cast<double>(sp.position.z)});
        }
        if (!points.empty()) {
            kdtree::KDTree tree(points);
            if (!tree.empty()) {
                size_t nearest_idx = tree.nearest_index(
                    {static_cast<double>(mouse_world_pos.x),
                     static_cast<double>(mouse_world_pos.y),
                     static_cast<double>(mouse_world_pos.z)});
                if (nearest_idx < item.skeleton_order_cache.size()) {
                    const auto& nearest_pos =
                        item.skeleton_order_cache[nearest_idx].position;
                    const double dx = nearest_pos.x - mouse_world_pos.x;
                    const double dy = nearest_pos.y - mouse_world_pos.y;
                    const double dz = nearest_pos.z - mouse_world_pos.z;
                    const double dist =
                        std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (dist <= static_cast<double>(pick_range)) {
                        item.picked_skeleton_points.push_back(
                            item.skeleton_order_cache[nearest_idx]);
                        item.sort_picked_skeleton_points();
                        item.joint_wireframe_dirty = true;
                        return;
                    }
                }
            }
        }
    }

    // 回退：基于 surface_skeleton_cache 遍历查找
    if (item.surface_skeleton_cache.empty())
        return;

    const float pick_range_sq = pick_range * pick_range;
    float best_dist_sq = std::numeric_limits<float>::max();
    const SkeletonPointPick* best_skeleton = nullptr;

    for (const auto& entry : item.surface_skeleton_cache) {
        const auto surface_world =
            item.voxel_grid_data.voxelCenterToWorld(entry.surface_voxel);
        const float dx = surface_world.x - mouse_world_pos.x;
        const float dy = surface_world.y - mouse_world_pos.y;
        const float dz = surface_world.z - mouse_world_pos.z;
        const float dist_sq = dx * dx + dy * dy + dz * dz;
        if (dist_sq > pick_range_sq || dist_sq >= best_dist_sq)
            continue;

        best_dist_sq = dist_sq;
        best_skeleton = &entry.skeleton;
    }

    if (best_skeleton == nullptr)
        return;

    item.picked_skeleton_points.push_back(*best_skeleton);
    item.sort_picked_skeleton_points();
    item.joint_wireframe_dirty = true;
}

void RenderVoxelList::RenderVoxelItem::rebuild_joint_wireframe() {
    joint_wireframe_vertices.clear();
    joint_wireframe_dirty = false;
    if (picked_skeleton_points.empty()) return;

    using Vec3f = sinriv::kigstudio::sdf::joint::Vec3f;
    using Frame = sinriv::kigstudio::sdf::joint::Frame;

    auto get_pos = [&](size_t idx) -> Vec3f {
        const auto& p = picked_skeleton_points[idx].position;
        return {p.x, p.y, p.z};
    };

    for (size_t i = 0; i < picked_skeleton_points.size(); ++i) {
        const auto& params = picked_skeleton_points[i];
        Vec3f start = get_pos(i);
        Vec3f end;

        if (params.use_custom_direction) {
            end = Vec3f(params.custom_direction_end.x,
                        params.custom_direction_end.y,
                        params.custom_direction_end.z);
        } else {
            if (i + 1 < picked_skeleton_points.size()) {
                end = get_pos(i + 1);
            } else if (picked_skeleton_points.size() >= 2) {
                Vec3f prev = get_pos(i - 1);
                end = start + (start - prev);
            } else {
                if (!skeleton_lines.empty()) {
                    float best_dist = std::numeric_limits<float>::max();
                    Vec3f target = start;
                    for (const auto& line : skeleton_lines) {
                        Vec3f a(line.first.x, line.first.y, line.first.z);
                        Vec3f b(line.second.x, line.second.y, line.second.z);
                        float da = (a - start).length();
                        float db = (b - start).length();
                        if (da < best_dist) {
                            best_dist = da;
                            target = b;
                        }
                        if (db < best_dist) {
                            best_dist = db;
                            target = a;
                        }
                    }
                    end = target;
                } else if (!skeleton_order_cache.empty()) {
                    end = Vec3f(skeleton_order_cache.back().position.x,
                                skeleton_order_cache.back().position.y,
                                skeleton_order_cache.back().position.z);
                } else {
                    end = start + Vec3f(0, 0, 10);
                }
            }
        }

        if ((end - start).length() < 1e-6f) {
            end = start + Vec3f(0, 0, 1);
        }

        Frame frame;
        if (params.use_custom_direction) {
            frame = sinriv::kigstudio::sdf::joint::buildFrame(
                start, end, params.rotation_angle);
        } else {
            frame = sinriv::kigstudio::sdf::joint::buildFrameAlignedY(start, end);
        }

        sinriv::kigstudio::sdf::joint::JointNegativeSDF neg;
        sinriv::kigstudio::sdf::joint::JointPositiveSDF pos;
        neg.frame = frame;
        pos.frame = frame;

        // Apply per-node parameters
        neg.socket_cone_offset = params.socket_cone_offset;
        neg.socket_cone_angle = params.socket_cone_angle;
        neg.socket_cone_radius = params.socket_cone_radius;
        neg.head_cone_offset = params.head_cone_offset;
        neg.head_cone_radius = params.head_cone_radius;
        neg.female_gap = params.female_gap;
        neg.male_cylinder_offset = params.male_cylinder_offset;
        neg.male_cylinder_radius = params.male_cylinder_radius;
        neg.slot_extra = params.slot_extra;

        pos.socket_support_offset = params.socket_support_offset;
        pos.socket_support_radius = params.socket_support_radius;
        pos.head_support_offset = params.head_support_offset;
        pos.head_support_radius = params.head_support_radius;
        pos.male_cylinder_offset = params.male_cylinder_offset;
        pos.male_cylinder_radius = params.male_cylinder_radius;

        std::vector<std::pair<Vec3f, Vec3f>> segments;
        sinriv::kigstudio::sdf::joint::appendJointWireframe(segments, neg, pos);

        const uint32_t color = 0xff00ffff;  // yellow ABGR
        for (const auto& seg : segments) {
            Vec3f a = frame.localToWorld(seg.first);
            Vec3f b = frame.localToWorld(seg.second);
            joint_wireframe_vertices.push_back({a.x, -a.y, a.z, color});
            joint_wireframe_vertices.push_back({b.x, -b.y, b.z, color});
        }

        // center line
        Vec3f a = start;
        Vec3f b = end;
        joint_wireframe_vertices.push_back({a.x, -a.y, a.z, 0xffffffff});
        joint_wireframe_vertices.push_back({b.x, -b.y, b.z, 0xffffffff});
    }
}

sinriv::kigstudio::voxel::VoxelGrid
RenderVoxelList::RenderVoxelItem::do_segment_chain() const {
    using Vec3f = sinriv::kigstudio::sdf::joint::Vec3f;
    using Frame = sinriv::kigstudio::sdf::joint::Frame;
    using Vec3i = sinriv::kigstudio::voxel::Vec3i;
    using Chunk = sinriv::kigstudio::voxel::Chunk;

    auto result = voxel_grid_data;

    if (picked_skeleton_points.empty()) {
        return result;
    }

    auto get_pos = [&](size_t idx) -> Vec3f {
        const auto& p = picked_skeleton_points[idx].position;
        return {p.x, p.y, p.z};
    };

    for (size_t i = 0; i < picked_skeleton_points.size(); ++i) {
        const auto& params = picked_skeleton_points[i];
        Vec3f start = get_pos(i);
        Vec3f end;

        if (params.use_custom_direction) {
            end = Vec3f(params.custom_direction_end.x,
                        params.custom_direction_end.y,
                        params.custom_direction_end.z);
        } else {
            if (i + 1 < picked_skeleton_points.size()) {
                end = get_pos(i + 1);
            } else if (picked_skeleton_points.size() >= 2) {
                Vec3f prev = get_pos(i - 1);
                end = start + (start - prev);
            } else {
                end = start + Vec3f(0, 0, 10);
            }
        }

        if ((end - start).length() < 1e-6f) {
            end = start + Vec3f(0, 0, 1);
        }

        Frame frame;
        if (params.use_custom_direction) {
            frame = sinriv::kigstudio::sdf::joint::buildFrame(
                start, end, params.rotation_angle);
        } else {
            frame = sinriv::kigstudio::sdf::joint::buildFrameAlignedY(
                start, end);
        }

        sinriv::kigstudio::sdf::joint::JointNegativeSDF neg;
        neg.frame = frame;
        neg.socket_cone_offset = params.socket_cone_offset;
        neg.socket_cone_angle = params.socket_cone_angle;
        neg.socket_cone_radius = params.socket_cone_radius;
        neg.head_cone_offset = params.head_cone_offset;
        neg.head_cone_radius = params.head_cone_radius;
        neg.female_gap = params.female_gap;
        neg.male_cylinder_offset = params.male_cylinder_offset;
        neg.male_cylinder_radius = params.male_cylinder_radius;
        neg.slot_extra = params.slot_extra;

        std::vector<Vec3i> to_remove;
        for (const auto& [key, chunk] : result.chunks) {
            int cx = static_cast<int32_t>(key >> 42);
            int cy = static_cast<int32_t>((key >> 21) & 0x1FFFFF);
            int cz = static_cast<int32_t>(key & 0x1FFFFF);
            for (int ly = 0; ly < Chunk::SIZE; ++ly) {
                for (int lx = 0; lx < Chunk::SIZE; ++lx) {
                    for (int lz = 0; lz < Chunk::SIZE; ++lz) {
                        if (chunk.get(lx, ly, lz)) {
                            Vec3i voxel(cx * Chunk::SIZE + lx,
                                        cy * Chunk::SIZE + ly,
                                        cz * Chunk::SIZE + lz);
                            auto world_pos = result.voxelCenterToWorld(voxel);
                            Vec3f world_p(world_pos.x, world_pos.y, world_pos.z);
                            if (neg.contains(world_p)) {
                                to_remove.push_back(voxel);
                            }
                        }
                    }
                }
            }
        }
        if (!to_remove.empty()) {
            result.removeMany(to_remove);
        }

        // Add positive joint volume (support cones + male cylinder)
        sinriv::kigstudio::sdf::joint::JointPositiveSDF pos;
        pos.frame = frame;
        pos.socket_support_offset = params.socket_support_offset;
        pos.socket_support_radius = params.socket_support_radius;
        pos.head_support_offset = params.head_support_offset;
        pos.head_support_radius = params.head_support_radius;
        pos.male_cylinder_offset = params.male_cylinder_offset;
        pos.male_cylinder_radius = params.male_cylinder_radius;

        {
            float socket_sup_h =
                pos.socket_support_radius / std::tan(pos.socket_support_angle);
            float head_sup_h =
                pos.head_support_radius / std::tan(pos.head_support_angle);
            float local_x_min = -std::max(
                {pos.socket_support_radius, pos.head_support_radius,
                 pos.male_cylinder_half_height});
            float local_x_max = -local_x_min;
            float local_y_min = local_x_min;
            float local_y_max = local_x_max;
            float local_z_min =
                std::min({pos.socket_support_offset, pos.head_support_offset,
                          pos.male_cylinder_offset - pos.male_cylinder_radius});
            float local_z_max = std::max(
                {pos.socket_support_offset + socket_sup_h,
                 pos.head_support_offset + head_sup_h,
                 pos.male_cylinder_offset + pos.male_cylinder_radius});

            Vec3f local_corners[8] = {
                {local_x_min, local_y_min, local_z_min},
                {local_x_min, local_y_min, local_z_max},
                {local_x_min, local_y_max, local_z_min},
                {local_x_min, local_y_max, local_z_max},
                {local_x_max, local_y_min, local_z_min},
                {local_x_max, local_y_min, local_z_max},
                {local_x_max, local_y_max, local_z_min},
                {local_x_max, local_y_max, local_z_max}};

            Vec3f world_min = frame.localToWorld(local_corners[0]);
            Vec3f world_max = world_min;
            for (int i = 1; i < 8; ++i) {
                Vec3f wc = frame.localToWorld(local_corners[i]);
                world_min.x = std::min(world_min.x, wc.x);
                world_min.y = std::min(world_min.y, wc.y);
                world_min.z = std::min(world_min.z, wc.z);
                world_max.x = std::max(world_max.x, wc.x);
                world_max.y = std::max(world_max.y, wc.y);
                world_max.z = std::max(world_max.z, wc.z);
            }

            Vec3i voxel_min = result.worldToVoxel(world_min);
            Vec3i voxel_max = result.worldToVoxel(world_max);

            std::vector<Vec3i> to_add;
            for (int vx = voxel_min.x; vx <= voxel_max.x; ++vx) {
                for (int vy = voxel_min.y; vy <= voxel_max.y; ++vy) {
                    for (int vz = voxel_min.z; vz <= voxel_max.z; ++vz) {
                        Vec3i voxel(vx, vy, vz);
                        auto world_pos = result.voxelCenterToWorld(voxel);
                        Vec3f world_p(world_pos.x, world_pos.y, world_pos.z);
                        if (pos.contains(world_p)) {
                            to_add.push_back(voxel);
                        }
                    }
                }
            }
            if (!to_add.empty()) {
                result.insertMany(to_add);
            }
        }
    }

    return result;
}

void RenderVoxelList::begin_marked_edit(int item_id) {
    if (pending_marked_undo.has_value() && pending_marked_undo->item_id == item_id) {
        return;
    }
    pending_marked_undo.reset();
    auto it = items.find(item_id);
    if (it != items.end()) {
        pending_marked_undo = PendingMarkedUndo{
            item_id, MarkedVoxelsSnapshot{it->second->marked_voxels, ""}};
    }
}

void RenderVoxelList::end_marked_edit(int item_id, const std::string& desc) {
    if (!pending_marked_undo.has_value() || pending_marked_undo->item_id != item_id) {
        return;
    }
    auto it = items.find(item_id);
    if (it != items.end()) {
        // Only push if state actually changed
        const auto& before = pending_marked_undo->snapshot.marked_voxels;
        const auto& after = it->second->marked_voxels;
        bool changed = (before.chunks.size() != after.chunks.size());
        if (!changed) {
            for (const auto& [key, chunk] : before.chunks) {
                auto ait = after.chunks.find(key);
                if (ait == after.chunks.end()) {
                    changed = true;
                    break;
                }
                if (memcmp(chunk.data, ait->second.data, sizeof(chunk.data)) != 0) {
                    changed = true;
                    break;
                }
            }
            if (!changed) {
                for (const auto& [key, chunk] : after.chunks) {
                    if (before.chunks.find(key) == before.chunks.end()) {
                        changed = true;
                        break;
                    }
                }
            }
        }
        if (changed) {
            it->second->marked_undo_stack.push_back(
                MarkedVoxelsSnapshot{before, desc});
            it->second->marked_redo_stack.clear();
            if (it->second->marked_undo_stack.size() > kMaxUndoSize) {
                it->second->marked_undo_stack.erase(
                    it->second->marked_undo_stack.begin());
            }
        }
    }
    pending_marked_undo.reset();
}

void RenderVoxelList::push_marked_undo_now(int item_id, const std::string& desc) {
    auto it = items.find(item_id);
    if (it == items.end()) return;
    it->second->marked_undo_stack.push_back(
        MarkedVoxelsSnapshot{it->second->marked_voxels, desc});
    it->second->marked_redo_stack.clear();
    if (it->second->marked_undo_stack.size() > kMaxUndoSize) {
        it->second->marked_undo_stack.erase(
            it->second->marked_undo_stack.begin());
    }
    pending_marked_undo.reset();
}

void RenderVoxelList::undo_marked(int item_id) {
    auto it = items.find(item_id);
    if (it == items.end() || it->second->marked_undo_stack.empty()) return;
    it->second->marked_redo_stack.push_back(
        MarkedVoxelsSnapshot{it->second->marked_voxels, ""});
    it->second->marked_voxels = it->second->marked_undo_stack.back().marked_voxels;
    it->second->marked_undo_stack.pop_back();
    it->second->marked_voxels_dirty = true;
}

void RenderVoxelList::redo_marked(int item_id) {
    auto it = items.find(item_id);
    if (it == items.end() || it->second->marked_redo_stack.empty()) return;
    it->second->marked_undo_stack.push_back(
        MarkedVoxelsSnapshot{it->second->marked_voxels, ""});
    it->second->marked_voxels = it->second->marked_redo_stack.back().marked_voxels;
    it->second->marked_redo_stack.pop_back();
    it->second->marked_voxels_dirty = true;
}

bool RenderVoxelList::can_undo_marked(int item_id) const {
    auto it = items.find(item_id);
    return it != items.end() && !it->second->marked_undo_stack.empty();
}

bool RenderVoxelList::can_redo_marked(int item_id) const {
    auto it = items.find(item_id);
    return it != items.end() && !it->second->marked_redo_stack.empty();
}

}  // namespace sinriv::ui::render
