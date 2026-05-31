#include "render_voxel_list.h"
#include <bgfx/bgfx.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

namespace sinriv::ui::render {

static float distToSegment(int px, int py, int x1, int y1, int x2, int y2) {
    float dx = float(x2 - x1);
    float dy = float(y2 - y1);
    float len2 = dx * dx + dy * dy;
    float t = 0.0f;
    if (len2 > 0.0f) {
        t = std::max(0.0f, std::min(1.0f,
            (float(px - x1) * dx + float(py - y1) * dy) / len2));
    }
    float projx = float(x1) + t * dx;
    float projy = float(y1) + t * dy;
    float ddx = float(px) - projx;
    float ddy = float(py) - projy;
    return std::sqrt(ddx * ddx + ddy * ddy);
}

static bgfx::TextureHandle createHexagonIcon() {
    constexpr int S = 64;
    constexpr int cx = S / 2;
    constexpr int cy = S / 2;
    constexpr int r = 28;
    constexpr float lineWidth = 3.0f;

    // 正六边形顶点（从最上方顶点开始，顺时针）
    const int verts[6][2] = {
        {cx, cy - r},                                    // V0: top
        {cx + int(r * 0.866f + 0.5f), cy - r / 2},     // V1
        {cx + int(r * 0.866f + 0.5f), cy + r / 2},     // V2
        {cx, cy + r},                                    // V3: bottom
        {cx - int(r * 0.866f + 0.5f), cy + r / 2},     // V4
        {cx - int(r * 0.866f + 0.5f), cy - r / 2},     // V5
    };

    // 六边形边 + 从 V0 出发分成 4 份的对角线
    const int segments[][2][2] = {
        {{verts[0][0], verts[0][1]}, {verts[1][0], verts[1][1]}}, // V0-V1
        {{verts[1][0], verts[1][1]}, {verts[2][0], verts[2][1]}}, // V1-V2
        {{verts[2][0], verts[2][1]}, {verts[3][0], verts[3][1]}}, // V2-V3
        {{verts[3][0], verts[3][1]}, {verts[4][0], verts[4][1]}}, // V3-V4
        {{verts[4][0], verts[4][1]}, {verts[5][0], verts[5][1]}}, // V4-V5
        {{verts[5][0], verts[5][1]}, {verts[0][0], verts[0][1]}}, // V5-V0
        {{verts[0][0], verts[0][1]}, {verts[2][0], verts[2][1]}}, // V0-V2
        {{verts[0][0], verts[0][1]}, {verts[3][0], verts[3][1]}}, // V0-V3
        {{verts[0][0], verts[0][1]}, {verts[4][0], verts[4][1]}}, // V0-V4
    };
    constexpr int numSegments = sizeof(segments) / sizeof(segments[0]);

    std::vector<uint32_t> pixels(S * S, 0x00000000);

    for (int y = 0; y < S; ++y) {
        for (int x = 0; x < S; ++x) {
            float minDist = 1e6f;
            for (int i = 0; i < numSegments; ++i) {
                float d = distToSegment(x, y,
                    segments[i][0][0], segments[i][0][1],
                    segments[i][1][0], segments[i][1][1]);
                if (d < minDist) minDist = d;
            }
            if (minDist < lineWidth * 0.5f) {
                pixels[y * S + x] = 0xFFE0E0E0;  // 浅灰 (RGBA8)
            }
        }
    }

    const bgfx::Memory* mem = bgfx::copy(
        pixels.data(), static_cast<uint32_t>(pixels.size() * sizeof(uint32_t)));
    return bgfx::createTexture2D(
        S, S, false, 1, bgfx::TextureFormat::RGBA8,
        BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP, mem);
}

static bgfx::TextureHandle createCirclesIcon(uint32_t color) {
    constexpr int S = 64;
    constexpr int cx = S / 2;
    constexpr int cy = S / 2;
    const float radii[3] = {10.0f, 18.0f, 26.0f};

    std::vector<uint32_t> pixels(S * S, 0x00000000);

    for (int y = 0; y < S; ++y) {
        for (int x = 0; x < S; ++x) {
            float dx = float(x - cx);
            float dy = float(y - cy);
            float dist = std::sqrt(dx * dx + dy * dy);
            for (float radius : radii) {
                if (std::abs(dist - radius) < 2.0f) {
                    pixels[y * S + x] = color;
                    break;
                }
            }
        }
    }

    const bgfx::Memory* mem = bgfx::copy(
        pixels.data(), static_cast<uint32_t>(pixels.size() * sizeof(uint32_t)));
    return bgfx::createTexture2D(
        S, S, false, 1, bgfx::TextureFormat::RGBA8,
        BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP, mem);
}

void RenderVoxelList::initIcons() {
    icons.hexagon = createHexagonIcon();
    icons.circles = createCirclesIcon(0xFF60A0FF);      // 浅蓝
    icons.circles_white = createCirclesIcon(0xFFFFFFFF); // 白色
    std::cerr << "[Icons] hexagon idx=" << icons.hexagon.idx
              << " valid=" << bgfx::isValid(icons.hexagon)
              << "; circles idx=" << icons.circles.idx
              << " valid=" << bgfx::isValid(icons.circles)
              << "; circles_white idx=" << icons.circles_white.idx
              << " valid=" << bgfx::isValid(icons.circles_white) << "\n";
}

void RenderVoxelList::destroyIcons() {
    if (bgfx::isValid(icons.hexagon)) {
        bgfx::destroy(icons.hexagon);
        icons.hexagon = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(icons.circles)) {
        bgfx::destroy(icons.circles);
        icons.circles = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(icons.circles_white)) {
        bgfx::destroy(icons.circles_white);
        icons.circles_white = BGFX_INVALID_HANDLE;
    }
}

}  // namespace sinriv::ui::render
