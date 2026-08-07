#include <SDL.h>
#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>
#include <stb/stb_truetype.h>
#include <cstring>
#include <vector>
#include "kigstudio/sdf/sdf_mesh.h"
// stb_image implementation is in render_voxel_list_ui_addons.cpp
#include <sys/stat.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <type_traits>
#include <unordered_set>
#include <variant>
#include "../../dep/bgfx.cmake/bimg/3rdparty/stb/stb_image.h"
#include "../../dep/bgfx.cmake/bimg/3rdparty/stb/stb_image_write.h"
#ifdef _WIN32
#include <windows.h>
#endif
#include "kigstudio/agent/agent_handlers.h"
#include "kigstudio/cgal/mesh_simplification.h"
#include "kigstudio/sdf/sdf_chain_joint.h"
#include "kigstudio/utils/locale.h"
#include "kigstudio/utils/triangle.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/voxel/voxel2mesh.h"
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {

// Draw a pixel on an RGBA buffer (clamped to bounds).
static void draw_pixel(std::vector<uint8_t>& rgba,
                       int w,
                       int h,
                       int px,
                       int py,
                       uint8_t r,
                       uint8_t g,
                       uint8_t b,
                       uint8_t a = 255) {
    if (px < 0 || px >= w || py < 0 || py >= h)
        return;
    size_t idx = (static_cast<size_t>(py) * w + px) * 4;
    // Alpha blend
    float sa = a / 255.0f;
    float da = rgba[idx + 3] / 255.0f;
    float out_a = sa + da * (1.0f - sa);
    if (out_a < 0.001f)
        return;
    rgba[idx + 0] = static_cast<uint8_t>(
        (r * sa + rgba[idx + 0] * da * (1.0f - sa)) / out_a);
    rgba[idx + 1] = static_cast<uint8_t>(
        (g * sa + rgba[idx + 1] * da * (1.0f - sa)) / out_a);
    rgba[idx + 2] = static_cast<uint8_t>(
        (b * sa + rgba[idx + 2] * da * (1.0f - sa)) / out_a);
    rgba[idx + 3] = static_cast<uint8_t>(out_a * 255.0f);
}

// Bresenham line draw on RGBA buffer (thin, single-pixel).
static void draw_line(std::vector<uint8_t>& rgba,
                      int w,
                      int h,
                      int x0,
                      int y0,
                      int x1,
                      int y1,
                      uint8_t r,
                      uint8_t g,
                      uint8_t b) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        draw_pixel(rgba, w, h, x0, y0, r, g, b);
        if (x0 == x1 && y0 == y1)
            break;
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

// Draw a thick line by stamping circles at each Bresenham step.
static void draw_circle_marker(std::vector<uint8_t>& rgba,
                               int w,
                               int h,
                               int cx,
                               int cy,
                               int radius,
                               uint8_t r,
                               uint8_t g,
                               uint8_t b) {
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (dx * dx + dy * dy <= radius * radius) {
                draw_pixel(rgba, w, h, cx + dx, cy + dy, r, g, b);
            }
        }
    }
}

static void draw_thick_line(std::vector<uint8_t>& rgba,
                            int w,
                            int h,
                            int x0,
                            int y0,
                            int x1,
                            int y1,
                            int thickness,
                            uint8_t r,
                            uint8_t g,
                            uint8_t b) {
    if (thickness <= 1) {
        draw_line(rgba, w, h, x0, y0, x1, y1, r, g, b);
        return;
    }
    float radius = (thickness - 1) * 0.5f;
    // Collect all Bresenham steps, then draw filled circles at each step.
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int cx = x0, cy = y0;
    while (true) {
        draw_circle_marker(rgba, w, h, cx, cy, static_cast<int>(radius + 0.5f),
                           r, g, b);
        if (cx == x1 && cy == y1)
            break;
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            cx += sx;
        }
        if (e2 <= dx) {
            err += dx;
            cy += sy;
        }
    }
}

// ---- stb_truetype font rendering ----
static stbtt_fontinfo g_font_info;
static std::vector<uint8_t> g_font_data;
static bool g_font_loaded = false;
static float g_font_scale = 14.0f;  // cached after load

static bool try_load_font_file(const char* path) {
    if (g_font_loaded)
        return true;
#ifdef _WIN32
    FILE* f = fopen(path, "rb");
#else
    FILE* f = fopen(path, "rb");
#endif
    if (!f)
        return false;
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size <= 0 || size > 64 * 1024 * 1024) {
        fclose(f);
        return false;
    }
    g_font_data.resize(static_cast<size_t>(size));
    size_t rd = fread(g_font_data.data(), 1, static_cast<size_t>(size), f);
    fclose(f);
    if (rd != static_cast<size_t>(size))
        return false;

    int offset = stbtt_GetFontOffsetForIndex(g_font_data.data(), 0);
    g_font_loaded = (offset >= 0) &&
                    stbtt_InitFont(&g_font_info, g_font_data.data(), offset);
    if (g_font_loaded) {
        std::cout << "[draw_text] Loaded font: " << path << " (" << size
                  << " bytes)" << std::endl;
    }
    return g_font_loaded;
}

static void ensure_font_loaded(float font_size) {
    if (g_font_loaded) {
        g_font_scale = stbtt_ScaleForPixelHeight(&g_font_info, font_size);
        return;
    }
    // Try multiple common font paths.
    // A CJK-capable font (like msyh.ttc) is preferred for Chinese strand names.
    const char* kFontPaths[] = {
        // Windows CJK fonts (TrueType Collection)
        "C:/Windows/Fonts/msyh.ttc",
        "C:/Windows/Fonts/simsun.ttc",
        "C:/Windows/Fonts/msgothic.ttc",
        // Windows basic fonts
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/consola.ttf",
        "C:/Windows/Fonts/segoeui.ttf",
        // Bundled font (relative to working directory)
        "dep/bgfx.cmake/bgfx/examples/runtime/font/droidsans.ttf",
        "../dep/bgfx.cmake/cmake/bgfx/Release/font/droidsans.ttf",
        "build/dep/bgfx.cmake/cmake/bgfx/Release/font/droidsans.ttf",
        // macOS
        "/System/Library/Fonts/Helvetica.ttc",
        "/System/Library/Fonts/PingFang.ttc",
        // Linux
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
    };
    for (const char* path : kFontPaths) {
        // For .ttc files (TrueType Collection), we need a different approach:
        // try loading, and if it fails with offset=0, try with offset from
        // stbtt_GetFontOffsetForIndex.
        FILE* f = fopen(path, "rb");
        if (!f)
            continue;
        fseek(f, 0, SEEK_END);
        long size = ftell(f);
        fseek(f, 0, SEEK_SET);
        if (size <= 0 || size > 64 * 1024 * 1024) {
            fclose(f);
            continue;
        }
        g_font_data.resize(static_cast<size_t>(size));
        size_t rd = fread(g_font_data.data(), 1, static_cast<size_t>(size), f);
        fclose(f);
        if (rd != static_cast<size_t>(size))
            continue;

        // Try each collection index (up to 4) for .ttc files
        bool ok = false;
        for (int idx = 0; idx < 4; ++idx) {
            int off = stbtt_GetFontOffsetForIndex(g_font_data.data(), idx);
            if (off < 0)
                break;
            ok = stbtt_InitFont(&g_font_info, g_font_data.data(), off);
            if (ok) {
                g_font_loaded = true;
                g_font_scale =
                    stbtt_ScaleForPixelHeight(&g_font_info, font_size);
                std::cout << "[draw_text] Loaded font: " << path
                          << " (idx=" << idx << ", " << size << " bytes)"
                          << std::endl;
                return;
            }
        }
        g_font_data.clear();  // bad file, retry next
    }
    // No font found; text rendering will be silently skipped.
    std::cerr << "[draw_text] WARNING: No TTF font found. "
              << "Text labels will not be rendered." << std::endl;
}

// Decode a UTF-8 codepoint; returns the codepoint and advances `s`.
static int utf8_decode(const char** s) {
    unsigned char c = static_cast<unsigned char>(**s);
    if (c < 0x80) {
        (*s)++;
        return c;
    }
    int n;
    unsigned cp;
    if ((c & 0xE0) == 0xC0) {
        n = 2;
        cp = c & 0x1F;
    } else if ((c & 0xF0) == 0xE0) {
        n = 3;
        cp = c & 0x0F;
    } else if ((c & 0xF8) == 0xF0) {
        n = 4;
        cp = c & 0x07;
    } else {
        (*s)++;
        return 0xFFFD;
    }  // replacement char
    for (int i = 1; i < n; i++) {
        unsigned char nc = static_cast<unsigned char>((*s)[i]);
        if ((nc & 0xC0) != 0x80) {
            (*s)++;
            return 0xFFFD;
        }
        cp = (cp << 6) | (nc & 0x3F);
    }
    (*s) += n;
    return static_cast<int>(cp);
}

// Draw a text string at (x,y) using stb_truetype rasterization.
// (x,y) is the top-left of the first glyph's bounding box.
static void draw_text(std::vector<uint8_t>& rgba,
                      int img_w,
                      int img_h,
                      int x,
                      int y,
                      const char* text,
                      uint8_t r,
                      uint8_t g,
                      uint8_t b,
                      float font_size) {
    if (!text || !*text)
        return;
    ensure_font_loaded(font_size);
    if (!g_font_loaded)
        return;

    float scale = stbtt_ScaleForPixelHeight(&g_font_info, font_size);
    g_font_scale = scale;

    int ascent, descent, line_gap;
    stbtt_GetFontVMetrics(&g_font_info, &ascent, &descent, &line_gap);
    float baseline = y + ascent * scale;

    float xpos = static_cast<float>(x);
    const char* p = text;
    int prev_glyph = 0;
    while (*p) {
        int cp = utf8_decode(&p);
        if (cp == 0xFFFD) {
            prev_glyph = 0;
            continue;
        }

        int glyph = stbtt_FindGlyphIndex(&g_font_info, cp);

        // Kerning with previous glyph
        if (prev_glyph) {
            xpos += scale *
                    stbtt_GetGlyphKernAdvance(&g_font_info, prev_glyph, glyph);
        }
        prev_glyph = glyph;

        int advance, lsb;
        stbtt_GetGlyphHMetrics(&g_font_info, glyph, &advance, &lsb);

        float x_shift = xpos - std::floor(xpos);
        int c_x1, c_y1, c_x2, c_y2;
        stbtt_GetGlyphBitmapBoxSubpixel(&g_font_info, glyph, scale, scale,
                                        x_shift, 0, &c_x1, &c_y1, &c_x2, &c_y2);

        int gw = c_x2 - c_x1;
        int gh = c_y2 - c_y1;
        if (gw > 0 && gh > 0) {
            if (gw > 4096 || gh > 4096) {
                xpos += advance * scale;
                continue;
            }
            std::vector<uint8_t> bitmap(static_cast<size_t>(gw) * gh);
            stbtt_MakeGlyphBitmapSubpixel(&g_font_info, bitmap.data(), gw, gh,
                                          gw, scale, scale, x_shift, 0, glyph);

            int dst_x0 = static_cast<int>(xpos) + c_x1;
            int dst_y0 = static_cast<int>(baseline) + c_y1;
            for (int by = 0; by < gh; by++) {
                int dst_y = dst_y0 + by;
                if (dst_y < 0 || dst_y >= img_h)
                    continue;
                for (int bx = 0; bx < gw; bx++) {
                    uint8_t alpha = bitmap[by * gw + bx];
                    if (alpha > 0) {
                        int dst_x = dst_x0 + bx;
                        draw_pixel(rgba, img_w, img_h, dst_x, dst_y, r, g, b,
                                   alpha);
                    }
                }
            }
        }

        xpos += advance * scale;
    }
}
// Draw guide curves on an RGBA pixel buffer using ortho camera projection.
void draw_guide_curves_on_buffer(std::vector<uint8_t>& rgba,
                                 int w,
                                 int h,
                                 const OrthoProjectionState& ortho_state,
                                 const std::vector<HairStrand>& hair_strands,
                                 bool color_code,
                                 int line_thickness,
                                 float font_size) {
    // Color palette (matching hair_guides.py)
    static const uint32_t kPalette[] = {
        0xff4040ff, 0x40c8ffff, 0xffe040ff, 0x60ff80ff, 0xff80d0ff, 0xffa040ff,
        0xa080ffff, 0x40ffc8ff, 0xffff80ff, 0xff8080ff, 0x80a0ffff, 0x80ff40ff,
    };
    constexpr int kPaletteSize = sizeof(kPalette) / sizeof(kPalette[0]);

    // Camera parameters
    vec3f center = ortho_state._center;
    vec3f cam_right = ortho_state._cam_right;
    vec3f cam_up = ortho_state._cam_up;
    float half = ortho_state.viewport_size * 0.5f;

    auto project = [&](const vec3f& world) -> std::pair<int, int> {
        float rx = (world.x - center.x) * cam_right.x +
                   (world.y - center.y) * cam_right.y +
                   (world.z - center.z) * cam_right.z;
        float ry = (world.x - center.x) * cam_up.x +
                   (world.y - center.y) * cam_up.y +
                   (world.z - center.z) * cam_up.z;
        float ndc_x = rx / half;  // [-1, 1]
        float ndc_y = ry / half;  // [-1, 1]
        int px = static_cast<int>((ndc_x * 0.5f + 0.5f) * w);
        int py = static_cast<int>((0.5f - ndc_y * 0.5f) * h);
        return {px, py};
    };

    std::cout << "[draw_guide_curves] center=(" << center.x << "," << center.y
              << "," << center.z << ") half=" << half << " res=" << w << "x"
              << h << " line_thickness=" << line_thickness
              << " font_size=" << font_size << std::endl;

    int color_idx = 0;
    for (const auto& strand : hair_strands) {
        if (!strand.visible || strand.guide_points.size() < 2)
            continue;

        // Pick color
        uint32_t col = color_code ? kPalette[color_idx % kPaletteSize]
                                  : 0xffffffff;  // white
        ++color_idx;
        uint8_t cr = static_cast<uint8_t>((col >> 24) & 0xff);
        uint8_t cg = static_cast<uint8_t>((col >> 16) & 0xff);
        uint8_t cb = static_cast<uint8_t>((col >> 8) & 0xff);

        // Sample bezier curve
        auto sampled = sample_bezier_guide_curve(
            strand.guide_points, std::max(strand.guide_samples_per_segment, 1));

        // Draw line segments with configurable thickness
        for (size_t pi = 0; pi + 1 < sampled.size(); ++pi) {
            auto [px0, py0] = project(sampled[pi]);
            auto [px1, py1] = project(sampled[pi + 1]);
            draw_thick_line(rgba, w, h, px0, py0, px1, py1, line_thickness, cr,
                            cg, cb);
        }

        // Draw control point markers (slightly larger for thicker lines)
        int marker_radius = 2 + line_thickness / 2;
        for (const auto& p : strand.guide_points) {
            auto [px, py] = project(p);
            draw_circle_marker(rgba, w, h, px, py, marker_radius, cr, cg, cb);
        }

        // Draw strand name label at the first guide point
        if (font_size > 0.0f && !strand.name.empty()) {
            auto [px, py] = project(strand.guide_points.front());
            // Offset the label so it sits above the control point marker
            int label_x = px + marker_radius + 4;
            int label_y = py - marker_radius - static_cast<int>(font_size) - 2;
            draw_text(rgba, w, h, label_x, label_y, strand.name.c_str(), cr, cg,
                      cb, font_size);
        }
    }
}

}  // namespace sinriv::ui::render