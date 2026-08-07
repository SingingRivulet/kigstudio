#pragma once

#include <SDL.h>
#include <SDL_syswm.h>
#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <bx/math.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include "kigstudio/utils/locale.h"
#include "render.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>
#include <stb/stb_truetype.h>

#include "kigstudio/ui/logger.h"
#include "kigstudio/agent/agent_handlers.h"
#include "kigstudio/agent/agent_server.h"
#include "kigstudio/ui/render_collision.h"
#include "kigstudio/ui/render_mesh.h"
#include "kigstudio/ui/render_voxel.h"
#include "kigstudio/voxel/collision.h"
#include "tinyfiledialogs.h"
#include "ui/render_deferred.h"
#include "ui/render_voxel_list.h"
#include "ui/utils.h"

int ui_main(int argc, const char* const* argv) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    float yaw = 0;
    float pitch = 0;
    float rotation_middle_1[16];
    float rotation_middle_2[16];
    bx::mtxIdentity(rotation_middle_1);
    bx::mtxIdentity(rotation_middle_2);
    float distance = 200;
    float cameraOffsetX = 0.0f;
    float cameraOffsetY = 0.0f;
    bool leftMouseDown = false;
    bool middleMouseDown = false;
    bool leftMouseDownOnPick = false;
    bool middleMouseDownOnPick = false;
    // Nav map infinite panning state (middle-mouse drag in node graph)
    bool nav_map_panning = false;
    ImVec2 nav_map_pan_start_pos;
    ImVec2 nav_map_pan_prev_pos;
    bool nav_map_pan_warped = false;
    bool guide_curve_click_valid = false;
    bool width_edit_click_valid = false;
    bool hairline_point_pick_valid = false;
    SDL_SetMainReady();
    // 显示系统 IME 候选窗口（中文/日文输入法需要）
    SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << std::endl;
        return -1;
    }

    // 1. 创建 SDL 窗口，但不要创建 OpenGL Context
    SDL_Window* window = SDL_CreateWindow(
        "kigstudio GUI SinRivProject", SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED, 1280, 720,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_MAXIMIZED);

    if (!window) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << std::endl;
        return -1;
    }

    // 2. 获取原生窗口句柄
    SDL_SysWMinfo wmi;
    SDL_VERSION(&wmi.version);
    if (!SDL_GetWindowWMInfo(window, &wmi)) {
        std::cerr << "SDL_GetWindowWMInfo failed: " << SDL_GetError()
                  << std::endl;
        return -1;
    }

    // 3. 设置 bgfx 平台数据
    bgfx::PlatformData pd{};
    pd.nwh = wmi.info.win.window;  // 只设置窗口句柄
    pd.context = nullptr;  // 关键：不要传入 SDL 的 context，让 bgfx 自己创建
    pd.ndt = nullptr;
    bgfx::setPlatformData(pd);

    // bgfx init
    sinriv::ui::UILogger s_callback{};
    bgfx::Init init{};
    init.type = bgfx::RendererType::OpenGL;
    init.resolution.width = 1280;
    init.resolution.height = 720;
    init.resolution.reset = BGFX_RESET_VSYNC;
    init.callback = &s_callback;
    init.platformData = pd;

    if (!bgfx::init(init)) {
        std::cerr << "Failed to initialize bgfx" << std::endl;
        return -1;
    }

    constexpr bgfx::ViewId kGBufferView = 0;
    constexpr bgfx::ViewId kCollisionView = 1;
    constexpr bgfx::ViewId kCollisionFillView = 2;
    constexpr bgfx::ViewId kLightingView = 3;
    constexpr bgfx::ViewId kOverlayView = 4;
    constexpr bgfx::ViewId kMeshStencilFillView = 5;

    bgfx::setViewClear(kOverlayView, 0, 0x00000000, 1.0f, 0);
    bgfx::setViewFrameBuffer(kOverlayView, BGFX_INVALID_HANDLE);

    imguiCreate();
    ImGui::CreateContext();
    {
        // 设置 IME 回调，让 SDL2 知道输入光标位置，从而显示候选框
        ImGuiPlatformIO& platform_io = ImGui::GetPlatformIO();
        platform_io.Platform_ImeUserData = window;
        platform_io.Platform_SetImeDataFn =
            [](ImGuiContext*, ImGuiViewport*, ImGuiPlatformImeData* data) {
                SDL_Window* sdl_window = static_cast<SDL_Window*>(
                    ImGui::GetPlatformIO().Platform_ImeUserData);
                if (!sdl_window)
                    return;
                if (!data->WantTextInput) {
                    if (SDL_IsTextInputActive()) {
                        SDL_StopTextInput();
                    }
                    return;
                }
                if (!SDL_IsTextInputActive()) {
                    SDL_StartTextInput();
                }
                if (data->WantVisible) {
                    SDL_Rect r;
                    r.x = static_cast<int>(data->InputPos.x);
                    r.y = static_cast<int>(data->InputPos.y);
                    r.w = 1;
                    r.h = static_cast<int>(data->InputLineHeight);
                    SDL_SetTextInputRect(&r);
                }
            };
    }
    ImNodes::CreateContext();

    int width, height;
    SDL_GetWindowSize(window, &width, &height);
    bgfx::reset(width, height, BGFX_RESET_VSYNC);
    bgfx::setViewRect(kGBufferView, 0, 0, width, height);
    bgfx::setViewRect(kLightingView, 0, 0, width, height);
    bgfx::setViewRect(kCollisionView, 0, 0, width, height);
    bgfx::setViewRect(kCollisionFillView, 0, 0, width, height);
    bgfx::setViewRect(kMeshStencilFillView, 0, 0, width, height);
    bgfx::setViewRect(kOverlayView, 0, 0, width, height);

    sinriv::ui::render::RenderMeshShader mesh_render_shader(kGBufferView,
                                                            kOverlayView);
    sinriv::ui::render::RenderCollisionShader collision_render_shader(
        kGBufferView, kOverlayView);
    sinriv::ui::render::RenderDeferred deferred_renderer(
        kGBufferView, kLightingView, kCollisionView, kCollisionFillView,
        kMeshStencilFillView);
    sinriv::ui::render::RenderVoxelList render_items;
    sinriv::ui::render::RenderCollision collision_renderer{};

    bool running = true;

    bool debugPrintRotation = false;
    int oldW = width;
    int oldH = height;
    std::string current_window_title;
    
    sinriv::locale::locale_init();

    render_items.start_thread();
    render_items.initIcons();

    // ---- AI Agent HTTP Server ----
    sinriv::kigstudio::agent::AgentServer agent_server;
    agent_server.set_handler(sinriv::kigstudio::agent::agent_dispatch);

    int agent_port = 18920;
    bool enable_agent = true;
    for (int ai = 1; ai < argc; ++ai) {
        if (std::strcmp(argv[ai], "--no-agent") == 0) {
            enable_agent = false;
        } else if (std::strcmp(argv[ai], "--agent-port") == 0 && ai + 1 < argc) {
            agent_port = std::atoi(argv[++ai]);
        }
    }
    if (enable_agent && agent_port > 0) {
        if (agent_server.start(static_cast<std::uint16_t>(agent_port))) {
            render_items.agent_server_ptr = &agent_server;
            std::cout << "Agent API: http://127.0.0.1:" << agent_port
                      << "/api/v1" << std::endl;

            // Install the guide-curve draw callback once at startup.
            // The callback captures &render_items so it always reads live
            // strand data and camera state, regardless of which UI window
            // is open.  The export_curves / color_code flags are synced
            // every frame by update_api_server_caches().
            agent_server.setGuideCurveDrawState(
                false, true,
                [&render_items](std::vector<uint8_t>& rgba, int w, int h,
                                bool color_code,
                                int line_thickness, float font_size) {
                    std::lock_guard<std::mutex> lock(render_items.locker);
                    auto it = render_items.items.find(
                        render_items.render_id);
                    if (it != render_items.items.end() &&
                        it->second->source_type == 2) {
                        draw_guide_curves_on_buffer(
                            rgba, w, h, render_items.ortho_state,
                            it->second->hair_strands, color_code,
                            line_thickness, font_size);
                    }
                });
        } else {
            std::cerr << "Agent API: failed to start on port " << agent_port
                      << std::endl;
        }
    }

    auto try_load_startup_path = [&]() {
        if (argc <= 1)
            return;

        const std::filesystem::path input_path(argv[1]);
        std::error_code ec;
        if (std::filesystem::is_directory(input_path, ec)) {
            const std::string path =
                sinriv::ui::render::path_to_utf8(input_path);
            if (render_items.load_project(path)) {
                render_items.add_recent_project(path);
            } else {
                std::cerr << "Failed to load project directory: " << path
                          << "\n" << render_items.last_load_error
                          << std::endl;
            }
            return;
        }

        if (std::filesystem::is_regular_file(input_path, ec)) {
            std::string ext =
                sinriv::ui::render::path_to_utf8(input_path.extension());
            std::transform(ext.begin(), ext.end(), ext.begin(),
                           [](unsigned char c) {
                               return static_cast<char>(std::tolower(c));
                           });
            if (ext == ".stl") {
                std::string stl_path =
                    sinriv::ui::render::path_to_utf8(input_path);
                render_items.queue_load_stl(stl_path, 0.5f);
                render_items.add_recent_file(stl_path);
            } else {
                std::cerr << "Unsupported input file, expected .stl: "
                          << sinriv::ui::render::path_to_utf8(input_path)
                          << std::endl;
            }
            return;
        }

        std::cerr << "Input path is neither a directory nor an STL file: "
                  << sinriv::ui::render::path_to_utf8(input_path)
                  << std::endl;
    };

    try_load_startup_path();

    auto confirm_quit = [&]() -> bool {
        if (!render_items.has_dirty_items())
            return true;

        const std::string unsaved_title = sinriv::ui::render::utf8_to_ansi(
            sinriv::ui::render::get_locale_cstr(
                "dialog.unsaved_changes_title"));
        const std::string unsaved_message = sinriv::ui::render::utf8_to_ansi(
            sinriv::ui::render::get_locale_cstr(
                "dialog.unsaved_changes_message"));
        int choice = tinyfd_messageBox(unsaved_title.c_str(),
                                       unsaved_message.c_str(),
                                       "yesnocancel", "warning", 1);
        if (choice == 0) {
            return false;
        }
        if (choice == 2) {
            return true;
        }

        bool saved = false;
        if (!render_items.project_path.empty()) {
            saved = render_items.save_current_project();
        } else {
            const char* folder = tinyfd_selectFolderDialog(
                sinriv::ui::render::utf8_to_ansi(
                    sinriv::ui::render::get_locale_cstr(
                        "dialog.save_project_title"))
                    .c_str(),
                "");
            if (!folder) {
                return false;
            }
            saved = render_items.save_project(
                sinriv::ui::render::tinyfd_path_to_utf8(folder));
        }

        if (!saved) {
            std::string msg =
                sinriv::ui::render::get_locale_string("error.save_failed") +
                "\n" + render_items.last_save_error;
            const std::string error_title =
                sinriv::ui::render::utf8_to_ansi("Error");
            const std::string error_message =
                sinriv::ui::render::utf8_to_ansi(msg.c_str());
            tinyfd_messageBox(error_title.c_str(), error_message.c_str(), "ok",
                              "error", 1);
            return false;
        }
        return true;
    };

    auto pathes = sinriv::ui::render::get_default_font_path();
    if (!pathes.empty()) {
        ImGuiIO& io = ImGui::GetIO();
        static const ImWchar chinese_ranges[] = {
            0x0020, 0x00FF,  // Latin
            0x3000, 0x30FF,  // 日文
            0x4E00, 0x9FAF,  // 中文
            0,
        };

        io.Fonts->Clear();
        for (const auto& path : pathes) {
            std::cout << "load font: " << path << std::endl;
            io.Fonts->AddFontFromFileTTF(path.c_str(), 16.0f, nullptr,
                                         chinese_ranges);
        }
    }

    int64_t current_time = sinriv::getUnixTimeSeconds();
    int frame_in_second = 0;
    while (running) {
        ++frame_in_second;
        int64_t current_time_tmp = sinriv::getUnixTimeSeconds();
        if (current_time != current_time_tmp) {
            render_items.memory_current = sinriv::getCurrentRSS();
            render_items.memory_peak = sinriv::getPeakRSS();
            render_items.fps = frame_in_second;
            current_time = current_time_tmp;
            frame_in_second = 0;
        }
        SDL_Event e;
        ImGuiIO& io = ImGui::GetIO();
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) {
                ImGuiKey imgui_key =
                    sinriv::kigstudio::ui::sdlToImGuiKey(e.key.keysym.sym);
                if (imgui_key != ImGuiKey_None) {
                    io.AddKeyEvent(imgui_key, e.type == SDL_KEYDOWN);
                    io.SetKeyEventNativeData(imgui_key, e.key.keysym.sym,
                                             e.key.keysym.scancode);
                }
            }

            if (e.type == SDL_TEXTINPUT) {
                io.AddInputCharactersUTF8(e.text.text);
            }

            if (e.type == SDL_MOUSEBUTTONDOWN) {
                if (e.button.button == SDL_BUTTON_LEFT) {
                    leftMouseDown = true;
                    bool picking_active = false;
                    {
                        auto it = render_items.items.find(render_items.render_id);
                        if (it != render_items.items.end() &&
                            render_items.object_editor_tab == 1 &&
                            it->second->voxel_picking_enabled) {
                            picking_active = true;
                        }
                    }
                    leftMouseDownOnPick = picking_active &&
                                          render_items.mouse_world_pos_valid;
                    if (leftMouseDownOnPick) {
                        render_items.begin_marked_edit(render_items.render_id);
                    }
                    // 引导曲线绘制模式
                    // Require mouse to be over valid geometry at press time,
                    // so clicking outside the model (empty space or UI panel)
                    // and releasing on the model doesn't trigger point addition.
                    if (!picking_active && !io.WantCaptureMouse &&
                        render_items.mouse_world_pos_valid) {
                        auto it = render_items.items.find(render_items.render_id);
                        if (it != render_items.items.end()) {
                            if (it->second->guide_curve_drawing_active) {
                                guide_curve_click_valid = true;
                            }
                            if (it->second->width_editing_active) {
                                width_edit_click_valid = true;
                            }
                            if (it->second->hairline_point_picking_active) {
                                hairline_point_pick_valid = true;
                            }
                        }
                    }
                    io.MouseDown[0] = true;
                } else if (e.button.button == SDL_BUTTON_MIDDLE) {
                    middleMouseDown = true;
                    bool picking_active = false;
                    {
                        auto it = render_items.items.find(render_items.render_id);
                        if (it != render_items.items.end() &&
                            render_items.object_editor_tab == 1 &&
                            it->second->voxel_picking_enabled) {
                            picking_active = true;
                        }
                    }
                    middleMouseDownOnPick = picking_active &&
                                            render_items.mouse_world_pos_valid;
                    io.MouseDown[2] = true;
                } else if (e.button.button == SDL_BUTTON_RIGHT) {
                    io.MouseDown[1] = true;
                }
            }

            if (e.type == SDL_MOUSEBUTTONUP) {
                if (e.button.button == SDL_BUTTON_LEFT) {
                    if (leftMouseDownOnPick) {
                        bool shift = (SDL_GetModState() & KMOD_SHIFT) != 0;
                        render_items.end_marked_edit(
                            render_items.render_id,
                            shift ? "Erase" : "Brush");
                    } else if (guide_curve_click_valid &&
                               render_items.mouse_world_pos_valid &&
                               !io.WantCaptureMouse) {
                        // 引导曲线绘制：点击添加点
                        auto it = render_items.items.find(render_items.render_id);
                        if (it != render_items.items.end()) {
                            auto& item = *it->second;
                            if (item.guide_curve_drawing_active &&
                                !item.active_guide_draw_strand.empty()) {
                                auto* strand_ptr = item.find_strand_by_uuid(item.active_guide_draw_strand);
                                if (strand_ptr) {
                                    render_items.push_undo_now(
                                        render_items.render_id, std::nullopt,
                                        "Add Guide Point");
                                    bool alt =
                                        (SDL_GetModState() & KMOD_ALT) != 0;
                                    if (alt) {
                                        strand_ptr->guide_points.insert(
                                            strand_ptr->guide_points.begin(),
                                            render_items.mouse_world_pos);
                                    } else {
                                        strand_ptr->guide_points.push_back(
                                            render_items.mouse_world_pos);
                                    }
                                    strand_ptr->mesh_dirty = true;
                                }
                            }
                        }
                    } else if (width_edit_click_valid &&
                               render_items.mouse_world_pos_valid &&
                               !io.WantCaptureMouse) {
                        // 宽度编辑：点击添加宽度参考点
                        auto it = render_items.items.find(render_items.render_id);
                        if (it != render_items.items.end()) {
                            auto& item = *it->second;
                            if (item.width_editing_active &&
                                !item.active_width_edit_strand.empty()) {
                                auto* strand_ptr = item.find_strand_by_uuid(item.active_width_edit_strand);
                                if (strand_ptr) {
                                    int strand_idx = static_cast<int>(strand_ptr - item.hair_strands.data());
                                    render_items.push_undo_now(
                                        render_items.render_id, std::nullopt,
                                        "Add Width Point");
                                    item.add_width_point_at(
                                        strand_idx,
                                        render_items.mouse_world_pos);
                                    strand_ptr->mesh_dirty = true;
                                }
                            }
                        }
                    } else if (hairline_point_pick_valid &&
                               render_items.mouse_world_pos_valid &&
                               !io.WantCaptureMouse) {
                        // 发际线三点拾取
                        auto it = render_items.items.find(render_items.render_id);
                        if (it != render_items.items.end()) {
                            auto& item = *it->second;
                            if (item.hairline_point_picking_active &&
                                item.hairline_picking_point_index >= 0 &&
                                item.hairline_picking_point_index < 3) {
                                item.hairline_plane_points
                                    [item.hairline_picking_point_index] =
                                    render_items.mouse_world_pos;
                                item.hairline_point_picking_active = false;
                            }
                        }
                    } else if (render_items.ortho_state.is_picking_point &&
                               render_items.mouse_world_pos_valid) {
                        // Ortho projection: click on model sets look direction
                        // (from picked point toward center).
                        auto it = render_items.items.find(render_items.render_id);
                        if (it != render_items.items.end()) {
                            auto& item = *it->second;
                            auto dir = sinriv::kigstudio::voxel::collision::vec3f{
                                item.addon_center_point.x - render_items.mouse_world_pos.x,
                                item.addon_center_point.y - render_items.mouse_world_pos.y,
                                item.addon_center_point.z - render_items.mouse_world_pos.z
                            };
                            float len = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
                            if (len > 1e-6f) {
                                dir.x /= len; dir.y /= len; dir.z /= len;
                            }
                            render_items.ortho_state.projection_dir = dir;
                            render_items.ortho_state.is_picking_point = false;
                            // Direction changed: re-render the off-screen texture
                            render_items.ortho_state.render_dirty = true;
                        }
                    } else if (leftMouseDown &&
                               (std::abs(pitch) > 1e-6f ||
                                std::abs(yaw) > 1e-6f)) {
                        float drag_rotation_1[16];
                        float drag_rotation_2[16];
                        float next_middle_1[16];
                        float next_middle_2[16];
                        bx::mtxRotateXY(drag_rotation_1, bx::toRad(-pitch),
                                        bx::toRad(yaw));
                        bx::mtxRotateXY(drag_rotation_2, bx::toRad(pitch),
                                        bx::toRad(yaw));
                        bx::mtxMul(next_middle_1, rotation_middle_1, drag_rotation_1);
                        bx::mtxMul(next_middle_2, rotation_middle_2, drag_rotation_2);
                        std::memcpy(rotation_middle_1, next_middle_1,
                                    sizeof(rotation_middle_1));
                        std::memcpy(rotation_middle_2, next_middle_2,
                                    sizeof(rotation_middle_2));
                        pitch = 0.0f;
                        yaw = 0.0f;
                    }
                    leftMouseDown = false;
                    leftMouseDownOnPick = false;
                    guide_curve_click_valid = false;
                    width_edit_click_valid = false;
                    hairline_point_pick_valid = false;
                    io.MouseDown[0] = false;
                } else if (e.button.button == SDL_BUTTON_MIDDLE) {
                    if (nav_map_panning) {
                        SDL_ShowCursor(SDL_ENABLE);
                        SDL_WarpMouseInWindow(window,
                            (int)nav_map_pan_start_pos.x,
                            (int)nav_map_pan_start_pos.y);
                        nav_map_panning = false;
                    }
                    middleMouseDown = false;
                    middleMouseDownOnPick = false;
                    io.MouseDown[2] = false;
                } else if (e.button.button == SDL_BUTTON_RIGHT) {
                    io.MouseDown[1] = false;
                }
            }

            if (e.type == SDL_MOUSEWHEEL) {
                io.AddMouseWheelEvent(0.0f, (float)e.wheel.y);
                if (!io.WantCaptureMouse) {
                    distance -= e.wheel.y * 10;
                    if (distance < 1) {
                        distance = 1;
                    }
                }
            }

            if (e.type == SDL_MOUSEMOTION) {
                // Nav map infinite panning: when middle-mouse is held and
                // ImGui captures the mouse (nav node graph window), hide the
                // OS cursor and warp it inward when approaching the screen
                // edge.  Uses SDL relative motion to accumulate io.MousePos
                // so that warp-generated deltas don't cause jumps.
                if (middleMouseDown && io.WantCaptureMouse &&
                    !middleMouseDownOnPick) {
                    if (!nav_map_panning) {
                        nav_map_panning = true;
                        nav_map_pan_start_pos = io.MousePos;
                        nav_map_pan_warped = false;
                        SDL_ShowCursor(SDL_DISABLE);
                    }
                    if (nav_map_pan_warped) {
                        // This motion event was caused by our own warp.
                        // Skip it — keep io.MousePos at the pre-warp value
                        // so that neither MousePos nor MouseDelta jumps.
                        nav_map_pan_warped = false;
                    } else {
                        // Accumulate relative motion — immune to warp jumps
                        io.MousePos.x += e.motion.xrel;
                        io.MousePos.y += e.motion.yrel;

                        // Warp inward when approaching screen edge
                        const int margin = 60;
                        const float push = 200.0f;
                        ImVec2 current((float)e.motion.x, (float)e.motion.y);
                        if (current.x < margin ||
                            current.x > (float)oldW - margin ||
                            current.y < margin ||
                            current.y > (float)oldH - margin) {
                            float tx = current.x, ty = current.y;
                            if (current.x < margin) tx += push;
                            else if (current.x > (float)oldW - margin) tx -= push;
                            if (current.y < margin) ty += push;
                            else if (current.y > (float)oldH - margin) ty -= push;
                            SDL_WarpMouseInWindow(window, (int)tx, (int)ty);
                            nav_map_pan_warped = true;
                        }
                    }
                } else {
                    // If we were panning in nav map but mouse left the
                    // ImGui area, restore cursor and stop panning.
                    if (nav_map_panning && middleMouseDown) {
                        SDL_ShowCursor(SDL_ENABLE);
                        SDL_WarpMouseInWindow(
                            window, (int)nav_map_pan_start_pos.x,
                            (int)nav_map_pan_start_pos.y);
                        nav_map_panning = false;
                    }
                    if (leftMouseDown && !io.WantCaptureMouse && !leftMouseDownOnPick) {
                        yaw += e.motion.xrel * 0.3f;
                        pitch += e.motion.yrel * 0.3f;
                    }
                    if (middleMouseDown && !io.WantCaptureMouse && !middleMouseDownOnPick) {
                        const float fovRadians = bx::toRad(60.0f);
                        const float viewportHeight = bx::max(1.0f, float(height));
                        const float worldUnitsPerPixel = 2.0f * distance *
                                                         tanf(fovRadians * 0.5f) /
                                                         viewportHeight;
                        cameraOffsetX += e.motion.xrel * worldUnitsPerPixel;
                        cameraOffsetY -= e.motion.yrel * worldUnitsPerPixel;
                    }
                    // 体素刷选
                    if (leftMouseDown && leftMouseDownOnPick &&
                        render_items.mouse_world_pos_valid) {
                        bool shift = (SDL_GetModState() & KMOD_SHIFT) != 0;
                        auto it = render_items.items.find(render_items.render_id);
                        if (it != render_items.items.end()) {
                            auto& item = *it->second;
                            if (item.voxel_picking_enabled && item.surface_cache_ready) {
                                render_items.brush_marked_voxels(
                                    render_items.mouse_world_pos,
                                    item.voxel_pick_range, shift);
                            }
                        }
                    }
                    io.MousePos = ImVec2((float)e.motion.x, (float)e.motion.y);
                }
            }

            if (e.type == SDL_KEYDOWN && !io.WantCaptureKeyboard) {
                bool ctrl = (SDL_GetModState() & KMOD_CTRL) != 0;
                bool shift = (SDL_GetModState() & KMOD_SHIFT) != 0;
                if (e.key.keysym.sym == SDLK_o && !ctrl) {
                    render_items.show_file_loader = true;
                } else if (e.key.keysym.sym == SDLK_s && ctrl && shift) {
                    render_items.show_save_as_dialog = true;
                } else if (e.key.keysym.sym == SDLK_s && ctrl) {
                    if (!render_items.project_path.empty()) {
                        if (!render_items.save_current_project()) {
                            std::string msg = sinriv::ui::render::get_locale_string("error.save_failed") + "\n" + render_items.last_save_error;
                            tinyfd_messageBox("Error",
                                sinriv::ui::render::utf8_to_ansi(msg.c_str()).c_str(),
                                "ok", "error", 1);
                        }
                    } else {
                        render_items.show_save_dialog = true;
                    }
                } else if (e.key.keysym.sym == SDLK_o && ctrl) {
                    render_items.show_load_dialog = true;
                } else if (e.key.keysym.sym == SDLK_z && ctrl) {
                    if (render_items.object_editor_tab == 1) {
                        render_items.undo_marked(render_items.render_id);
                    } else {
                        render_items.undo(render_items.render_id);
                    }
                } else if (e.key.keysym.sym == SDLK_y && ctrl) {
                    if (render_items.object_editor_tab == 1) {
                        render_items.redo_marked(render_items.render_id);
                    } else {
                        render_items.redo(render_items.render_id);
                    }
                }
            }

            if (e.type == SDL_QUIT && confirm_quit())
                running = false;
        }

        io.AddKeyEvent(ImGuiMod_Ctrl, (SDL_GetModState() & KMOD_CTRL) != 0);
        io.AddKeyEvent(ImGuiMod_Shift, (SDL_GetModState() & KMOD_SHIFT) != 0);
        io.AddKeyEvent(ImGuiMod_Alt, (SDL_GetModState() & KMOD_ALT) != 0);
        io.AddKeyEvent(ImGuiMod_Super, (SDL_GetModState() & KMOD_GUI) != 0);

        // ===== 读取窗口尺寸 =====
        SDL_GetWindowSize(window, &width, &height);

        // ===== 如果尺寸变化就 reset =====
        if (width != oldW || height != oldH) {
            bgfx::reset(width, height, BGFX_RESET_VSYNC);
            oldW = width;
            oldH = height;
            std::cout << "Window resized to " << width << "x" << height
                      << std::endl;
        }
        bgfx::setViewRect(kGBufferView, 0, 0, width, height);
        bgfx::setViewRect(kLightingView, 0, 0, width, height);
        bgfx::setViewRect(kCollisionView, 0, 0, width, height);
        bgfx::setViewRect(kCollisionFillView, 0, 0, width, height);
        bgfx::setViewRect(kMeshStencilFillView, 0, 0, width, height);
        bgfx::setViewRect(kOverlayView, 0, 0, width, height);

        float view_1[16];
        float view_2[16];
        float proj[16];
        const bx::Vec3 eye(cameraOffsetX, cameraOffsetY, distance);
        const bx::Vec3 at(cameraOffsetX, cameraOffsetY, 0.0f);
        bx::mtxLookAt(view_1, eye, at);
        float flip_y[16];
        bx::mtxScale(flip_y, 1.0f, -1.0f, 1.0f);
        const bx::Vec3 eye_2 = bx::mul(eye, flip_y);
        const bx::Vec3 at_2 = bx::mul(at, flip_y);
        bx::mtxLookAt(view_2, eye_2, at_2);
        bx::mtxProj(proj, 60.0f, float(width) / float(height), 0.1f, 1000.0f,
                    bgfx::getCaps()->homogeneousDepth);
        bgfx::setViewTransform(kGBufferView, view_1, proj);
        bgfx::setViewTransform(kOverlayView, view_1, proj);
        bgfx::setViewFrameBuffer(kOverlayView, BGFX_INVALID_HANDLE);
        deferred_renderer.setViewportSize(static_cast<uint16_t>(width),
                                          static_cast<uint16_t>(height));
        deferred_renderer.setSceneViewProjection(view_1, proj);
        deferred_renderer.prepareFrame();
        render_items.window_height = height;
        render_items.window_width = width;
        render_items.setViewportSize(width, height);
        render_items.setViewProjection(view_1, proj);
        collision_renderer.setViewportSize(width, height);
        collision_renderer.setViewProjection(view_2, proj);

        float mtx_1[16];
        float mtx_2[16];
        float drag_rotation_1[16];
        float drag_rotation_2[16];
        bx::mtxRotateXY(drag_rotation_1, bx::toRad(-pitch), bx::toRad(yaw));
        bx::mtxRotateXY(drag_rotation_2, bx::toRad(pitch), bx::toRad(yaw));
        bx::mtxMul(mtx_1, rotation_middle_1, drag_rotation_1);
        bx::mtxMul(mtx_2, rotation_middle_2, drag_rotation_2);
        deferred_renderer.setSceneModelTransform(mtx_2);
        sinriv::kigstudio::mat::matrix<float> cpu_model_matrix(mtx_1);
        cpu_model_matrix.transpose();
        render_items.setModelMatrix(cpu_model_matrix);

        render_items.upload_collision(deferred_renderer);
        render_items.render_gbuffer(mtx_2, mesh_render_shader);

        // During nav map panning, freeze the 3D viewport cursor so the
        // red picking crosshair doesn't follow the warped/hidden cursor.
        if (!nav_map_panning) {
            deferred_renderer.screen_mouse_pos_[0] = io.MousePos.x;
            deferred_renderer.screen_mouse_pos_[1] = io.MousePos.y;
        }
        // When the ortho edit window is active and the mouse hovers over
        // valid model surface, override the shader red-cursor position
        // so both 3D and 2D cursors stay in sync.
        // mouse_world_pos was already transformed to model space via
        // current_model_matrix in render_ortho_edit_window.
        if (render_items.ortho_state.edit_window_open &&
            render_items.ortho_state.is_hovering_model &&
            render_items.mouse_world_pos_valid) {
            deferred_renderer.mouse_pos_[0] = render_items.mouse_world_pos.x;
            deferred_renderer.mouse_pos_[1] = render_items.mouse_world_pos.y;
            deferred_renderer.mouse_pos_[2] = render_items.mouse_world_pos.z;
            deferred_renderer.mouse_highlight_[0] = 1.0f;
            deferred_renderer.mouse_highlight_[1] =
                deferred_renderer.mouse_highlight_range_;
            deferred_renderer.mouse_highlight_[2] = 1.0f;
            deferred_renderer.mouse_highlight_[3] = 1.0f;
        }
        deferred_renderer.render();
        bgfx::setViewTransform(kOverlayView, view_2, proj);

        render_items.render_overlay(collision_renderer, mtx_1, mtx_2,
                                    collision_render_shader, mesh_render_shader,
                                    &cpu_model_matrix);

        render_items.process_queue_result();
        agent_server.process_commands(render_items);
        io.DisplaySize = ImVec2((float)width, (float)height);
        render_items.update_mouse();
        ImGui::NewFrame();
        render_items.update_mouse_pos(deferred_renderer);
        deferred_renderer.mouse_highlight_range_ = render_items.mouse_highlight_range;
        render_items.render_ui();
        ImGui::Render();

        // Sync ImGui mouse-cursor requests to SDL.  bgfx's custom imgui
        // backend does not forward ImGui::SetMouseCursor → OS cursor, so
        // we read io.MouseCursor and call SDL_SetCursor ourselves.
        {
            static ImGuiMouseCursor s_last_cursor = ImGuiMouseCursor_COUNT;
            static SDL_Cursor* s_cached_cursor = nullptr;
            ImGuiMouseCursor cur = ImGui::GetMouseCursor();
            if (cur != s_last_cursor) {
                s_last_cursor = cur;
                SDL_SystemCursor sc = SDL_SYSTEM_CURSOR_ARROW;
                switch (cur) {
                case ImGuiMouseCursor_ResizeNWSE:
                    sc = SDL_SYSTEM_CURSOR_SIZENWSE; break;
                case ImGuiMouseCursor_ResizeNESW:
                    sc = SDL_SYSTEM_CURSOR_SIZENESW; break;
                case ImGuiMouseCursor_ResizeAll:
                    sc = SDL_SYSTEM_CURSOR_SIZEALL; break;
                case ImGuiMouseCursor_ResizeNS:
                    sc = SDL_SYSTEM_CURSOR_SIZENS; break;
                case ImGuiMouseCursor_ResizeEW:
                    sc = SDL_SYSTEM_CURSOR_SIZEWE; break;
                case ImGuiMouseCursor_Hand:
                    sc = SDL_SYSTEM_CURSOR_HAND; break;
                case ImGuiMouseCursor_TextInput:
                    sc = SDL_SYSTEM_CURSOR_IBEAM; break;
                default:
                    sc = SDL_SYSTEM_CURSOR_ARROW; break;
                }
                if (s_cached_cursor) SDL_FreeCursor(s_cached_cursor);
                s_cached_cursor = SDL_CreateSystemCursor(sc);
                SDL_SetCursor(s_cached_cursor);
            }
        }

        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
            !io.WantCaptureMouse) {
            render_items.mouse_world_pos_picked = true;
        } else {
            render_items.mouse_world_pos_picked = false;
        }

        // 更新窗口标题
        {
            std::string desired_title = "kigstudio";
            if (!render_items.project_path.empty()) {
                desired_title += " - [" + render_items.project_path + "]";
            }
            if (render_items.has_dirty_items()) {
                desired_title += " (*)";
            }
            if (desired_title != current_window_title) {
                current_window_title = desired_title;
                SDL_SetWindowTitle(window, current_window_title.c_str());
            }
        }

        imguiEndFrame();

        bgfx::frame();
    }

    agent_server.stop();
    render_items.release();

    deferred_renderer.release();
    collision_renderer.release();
    mesh_render_shader.release();
    collision_render_shader.release();
    ImNodes::DestroyContext();
    bgfx::shutdown();
    SDL_DestroyWindow(window);
    SDL_Quit();
    std::cout << "shutdown" << std::endl;
    return 0;
}
