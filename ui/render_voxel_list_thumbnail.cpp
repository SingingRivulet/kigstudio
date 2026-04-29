#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>
#include <stb/stb_truetype.h>
#include <type_traits>
#include <unordered_set>
#include <variant>
#include "kigstudio/utils/vec3.h"
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {

void RenderVoxelList::ensureThumbnailResources() {
    if (bgfx::isValid(thumb_fb_)) {
        return;
    }
    constexpr uint16_t ts = 128;
    constexpr uint64_t flags =
        BGFX_TEXTURE_RT | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP;

    thumb_color_tex_ = bgfx::createTexture2D(ts, ts, false, 1,
                                             bgfx::TextureFormat::BGRA8, flags);
    thumb_depth_tex_ = bgfx::createTexture2D(ts, ts, false, 1,
                                             bgfx::TextureFormat::D32F, flags);

    bgfx::TextureHandle attachments[] = {thumb_color_tex_, thumb_depth_tex_};
    thumb_fb_ = bgfx::createFrameBuffer(2, attachments, false);

    thumb_shader_ = std::make_unique<RenderMeshShader>(100, 101);
}

void RenderVoxelList::destroyThumbnailResources() {
    if (bgfx::isValid(thumb_fb_)) {
        bgfx::destroy(thumb_fb_);
        thumb_fb_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(thumb_color_tex_)) {
        bgfx::destroy(thumb_color_tex_);
        thumb_color_tex_ = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(thumb_depth_tex_)) {
        bgfx::destroy(thumb_depth_tex_);
        thumb_depth_tex_ = BGFX_INVALID_HANDLE;
    }
    thumb_shader_.reset();
}

void RenderVoxelList::processThumbnails() {
    // 1. 为 dirty 的 items 入队
    {
        std::lock_guard<std::mutex> lock(locker);
        for (auto& [id, item] : items) {
            if (item->thumbnail_dirty) {
                // 避免重复入队
                bool already_queued = false;
                std::queue<ThumbnailTask> temp = thumbnail_queue;
                while (!temp.empty()) {
                    if (temp.front().item_id == id) {
                        already_queued = true;
                        break;
                    }
                    temp.pop();
                }
                if (!already_queued) {
                    thumbnail_queue.push({id, ThumbnailTask::RENDER, 0});
                }
                item->thumbnail_dirty = false;
            }
        }
    }

    if (thumbnail_queue.empty()) {
        return;
    }

    auto& task = thumbnail_queue.front();

    switch (task.stage) {
        case ThumbnailTask::RENDER: {
            std::lock_guard<std::mutex> lock(locker);
            auto it = items.find(task.item_id);
            if (it == items.end()) {
                thumbnail_queue.pop();
                return;
            }
            auto& item = it->second;

            // 如果 voxel_renderer 为空但有 voxel_grid_data，先生成 mesh（放到
            // queue 线程）
            if (item->voxel_renderer.empty() &&
                item->voxel_grid_data.num_chunk() > 0) {
                // 检查是否已有生成结果
                {
                    std::lock_guard<std::mutex> lock(thumbnail_mesh_mutex);
                    auto res_it = thumbnail_mesh_results.find(task.item_id);
                    if (res_it != thumbnail_mesh_results.end()) {
                        item->voxel_renderer.loadGeometry(res_it->second);
                        thumbnail_mesh_results.erase(res_it);
                        // 继续执行后续 RENDER 逻辑
                    } else {
                        // 没有结果，检查是否已提交任务
                        if (thumbnail_mesh_pending.find(task.item_id) ==
                            thumbnail_mesh_pending.end()) {
                            // 提交任务到 queue
                            std::lock_guard<std::mutex> qlock(queue_mutex);
                            QueueTask qtask;
                            qtask.type = TASK_GENERATE_THUMBNAIL_MESH;
                            qtask.index = task.item_id;
                            queue.push(qtask);
                            thumbnail_mesh_pending.insert(task.item_id);
                        }
                        return;  // 等待生成完成
                    }
                }
            }

            if (item->voxel_renderer.empty()) {
                thumbnail_queue.pop();
                return;
            }

            ensureThumbnailResources();
            if (!thumb_shader_ || !thumb_shader_->ensureGBufferProgram()) {
                return;
            }

            auto& mesh = item->voxel_renderer.getMeshRenderer();
            auto [min_b, max_b] = mesh.getLocalBounds();
            bx::Vec3 center((min_b.x + max_b.x) * 0.5f,
                            (min_b.y + max_b.y) * 0.5f,
                            (min_b.z + max_b.z) * 0.5f);
            float dx = max_b.x - min_b.x;
            float dy = max_b.y - min_b.y;
            float dz = max_b.z - min_b.z;
            float radius = bx::sqrt(dx * dx + dy * dy + dz * dz) * 0.5f;
            float dist = bx::max(radius * 2.5f, 1.0f);

            bx::Vec3 eye(center.x + dist, center.y + dist * 0.6f,
                         center.z + dist * 0.4f);
            float view[16];
            bx::mtxLookAt(view, eye, center);
            float proj[16];
            bx::mtxProj(proj, 45.0f, 1.0f, 0.1f, dist * 10.0f,
                        bgfx::getCaps()->homogeneousDepth);

            // 为 item 创建持久的缩略图纹理
            if (!bgfx::isValid(item->thumbnail_tex)) {
                item->thumbnail_tex = bgfx::createTexture2D(
                    128, 128, false, 1, bgfx::TextureFormat::BGRA8,
                    BGFX_TEXTURE_BLIT_DST | BGFX_SAMPLER_U_CLAMP |
                        BGFX_SAMPLER_V_CLAMP);
            }

            constexpr bgfx::ViewId kThumbView = 100;
            bgfx::setViewFrameBuffer(kThumbView, thumb_fb_);
            bgfx::setViewClear(kThumbView, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                               0x333333FF, 1.0f, 0);
            bgfx::setViewRect(kThumbView, 0, 0, 128, 128);
            bgfx::setViewTransform(kThumbView, view, proj);

            float identity[16];
            bx::mtxIdentity(identity);
            mesh.renderGBuffer(identity, *thumb_shader_);
            bgfx::touch(kThumbView);

            // blit 到 item 的持久纹理
            constexpr bgfx::ViewId kBlitView = 101;
            bgfx::blit(kBlitView, item->thumbnail_tex, 0, 0, thumb_color_tex_);

            task.stage = ThumbnailTask::WAIT;
            task.wait_frames = 1;
            break;
        }
        case ThumbnailTask::WAIT: {
            if (task.wait_frames > 0) {
                task.wait_frames--;
            }
            if (task.wait_frames <= 0) {
                task.stage = ThumbnailTask::DONE;
            }
            break;
        }
        case ThumbnailTask::DONE: {
            thumbnail_queue.pop();
            break;
        }
    }
}

}