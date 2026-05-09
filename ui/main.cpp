#define SDL_MAIN_HANDLED

#include <SDL.h>
#include <SDL_syswm.h>
#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <bx/math.h>
#include <iostream>
#include "locale.h"
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
#include "kigstudio/ui/render_collision.h"
#include "kigstudio/ui/render_mesh.h"
#include "kigstudio/ui/render_voxel.h"
#include "kigstudio/voxel/collision.h"
#include "tinyfiledialogs.h"
#include "ui/render_deferred.h"
#include "ui/render_voxel_list.h"

int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    float yaw = 0;
    float pitch = 0;
    float distance = 200;
    float cameraOffsetX = 0.0f;
    float cameraOffsetY = 0.0f;
    bool leftMouseDown = false;
    bool middleMouseDown = false;
    bool leftMouseDownOnPick = false;
    bool middleMouseDownOnPick = false;
    SDL_SetMainReady();
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

    bgfx::setViewClear(kOverlayView, 0, 0x00000000, 1.0f, 0);
    bgfx::setViewFrameBuffer(kOverlayView, BGFX_INVALID_HANDLE);

    imguiCreate();
    ImGui::CreateContext();
    ImNodes::CreateContext();

    int width, height;
    SDL_GetWindowSize(window, &width, &height);
    bgfx::reset(width, height, BGFX_RESET_VSYNC);
    bgfx::setViewRect(kGBufferView, 0, 0, width, height);
    bgfx::setViewRect(kLightingView, 0, 0, width, height);
    bgfx::setViewRect(kCollisionView, 0, 0, width, height);
    bgfx::setViewRect(kCollisionFillView, 0, 0, width, height);
    bgfx::setViewRect(kOverlayView, 0, 0, width, height);

    sinriv::ui::render::RenderMeshShader mesh_render_shader(kGBufferView,
                                                            kOverlayView);
    sinriv::ui::render::RenderCollisionShader collision_render_shader(
        kGBufferView, kOverlayView);
    sinriv::ui::render::RenderDeferred deferred_renderer(
        kGBufferView, kLightingView, kCollisionView, kCollisionFillView);
    sinriv::ui::render::RenderVoxelList render_items;
    sinriv::ui::render::RenderCollision collision_renderer{};

    bool running = true;

    bool debugPrintRotation = false;
    int oldW = width;
    int oldH = height;
    std::string current_window_title;
    
    sinriv::ui::render::locale_init();

    render_items.start_thread();

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

    while (running) {
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
                    leftMouseDownOnPick = render_items.disable_camera_on_pick &&
                                          render_items.mouse_world_pos_valid;
                    io.MouseDown[0] = true;
                } else if (e.button.button == SDL_BUTTON_MIDDLE) {
                    middleMouseDown = true;
                    middleMouseDownOnPick = render_items.disable_camera_on_pick &&
                                            render_items.mouse_world_pos_valid;
                    io.MouseDown[2] = true;
                }
            }

            if (e.type == SDL_MOUSEBUTTONUP) {
                if (e.button.button == SDL_BUTTON_LEFT) {
                    leftMouseDown = false;
                    leftMouseDownOnPick = false;
                    io.MouseDown[0] = false;
                } else if (e.button.button == SDL_BUTTON_MIDDLE) {
                    middleMouseDown = false;
                    middleMouseDownOnPick = false;
                    io.MouseDown[2] = false;
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
                io.MousePos = ImVec2((float)e.motion.x, (float)e.motion.y);
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
                    render_items.undo(render_items.render_id);
                } else if (e.key.keysym.sym == SDLK_y && ctrl) {
                    render_items.redo(render_items.render_id);
                }
            }

            if (e.type == SDL_QUIT)
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
        bx::mtxRotateXY(mtx_1, bx::toRad(-pitch), bx::toRad(yaw));
        bx::mtxRotateXY(mtx_2, bx::toRad(pitch), bx::toRad(yaw));
        deferred_renderer.setSceneModelTransform(mtx_2);
        sinriv::kigstudio::mat::matrix<float> cpu_model_matrix(mtx_1);
        cpu_model_matrix.transpose();
        render_items.setModelMatrix(cpu_model_matrix);

        if (debugPrintRotation) {
            sinriv::kigstudio::mat::matrix<float> gpu_raw_matrix(mtx_1);
            std::cout << "yaw=" << yaw << " pitch=" << pitch << std::endl;
            sinriv::kigstudio::ui::printMatrixAxes("gpu_raw", gpu_raw_matrix);
            sinriv::kigstudio::ui::printMatrixAxes("cpu_model",
                                                   cpu_model_matrix);
            debugPrintRotation = false;
        }

        render_items.upload_collision(deferred_renderer);
        render_items.render_gbuffer(mtx_2, mesh_render_shader);

        deferred_renderer.screen_mouse_pos_[0] = io.MousePos.x;
        deferred_renderer.screen_mouse_pos_[1] = io.MousePos.y;
        deferred_renderer.render();
        bgfx::setViewTransform(kOverlayView, view_2, proj);

        render_items.render_overlay(collision_renderer, mtx_1, mtx_2,
                                    collision_render_shader, mesh_render_shader,
                                    &cpu_model_matrix);

        render_items.process_queue_result();
        io.DisplaySize = ImVec2((float)width, (float)height);
        render_items.update_mouse();
        ImGui::NewFrame();
        render_items.update_mouse_pos(deferred_renderer);
        render_items.render_ui();
        ImGui::Render();
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
