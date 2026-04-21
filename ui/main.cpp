#define SDL_MAIN_HANDLED

#include <SDL.h>
#include <SDL_syswm.h>
#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <bx/math.h>
#include <iostream>
#include "render.hpp"

#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <stb/stb_truetype.h>

#include "kigstudio/ui/logger.h"
#include "kigstudio/ui/render_collision.h"
#include "kigstudio/ui/render_deferred.h"
#include "kigstudio/ui/render_mesh.h"
#include "kigstudio/ui/render_voxel.h"
#include "kigstudio/ui/render_voxel_list.h"
#include "kigstudio/voxel/collision.h"
#include "tinyfiledialogs.h"

int main() {
    float yaw = 0;
    float pitch = 0;
    float distance = 200;
    bool mouseDown = false;
    SDL_SetMainReady();
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << std::endl;
        return -1;
    }

    // 1. 创建 SDL 窗口，但不要创建 OpenGL Context
    SDL_Window* window = SDL_CreateWindow(
        "bgfx + SDL", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720,
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
    constexpr bgfx::ViewId kLightingView = 2;
    constexpr bgfx::ViewId kOverlayView = 3;

    bgfx::setViewClear(kOverlayView, 0, 0x00000000, 1.0f, 0);
    bgfx::setViewFrameBuffer(kOverlayView, BGFX_INVALID_HANDLE);

    imguiCreate();

    int width, height;
    SDL_GetWindowSize(window, &width, &height);
    bgfx::reset(width, height, BGFX_RESET_VSYNC);
    bgfx::setViewRect(kGBufferView, 0, 0, width, height);
    bgfx::setViewRect(kLightingView, 0, 0, width, height);
    bgfx::setViewRect(kCollisionView, 0, 0, width, height);
    bgfx::setViewRect(kOverlayView, 0, 0, width, height);

    sinriv::kigstudio::voxel::VoxelGrid voxel_grid_data;
    sinriv::ui::render::RenderMeshShader mesh_render_shader(kGBufferView,
                                                            kOverlayView);
    sinriv::ui::render::RenderCollisionShader collision_render_shader(
        kGBufferView, kOverlayView);
    sinriv::ui::render::RenderDeferred deferred_renderer(
        kGBufferView, kLightingView, kCollisionView);
    sinriv::ui::render::RenderVoxelList render_items;
    sinriv::ui::render::RenderCollision collision_renderer{};

    bool running = true;
    bool showMesh = true;
    bool showVoxels = false;
    bool showCollision = true;

    bool showMeshAxis = false;
    bool showVoxelAxis = false;
    bool showCollisionAxis = false;

    bool debugPrintRotation = false;
    int oldW = width;
    int oldH = height;

    render_items.start_thread();

    while (running) {
        SDL_Event e;
        ImGuiIO& io = ImGui::GetIO();
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) {
                ImGuiKey imgui_key = sinriv::kigstudio::ui::sdlToImGuiKey(e.key.keysym.sym);
                if (imgui_key != ImGuiKey_None) {
                    io.AddKeyEvent(imgui_key, e.type == SDL_KEYDOWN);
                    io.SetKeyEventNativeData(imgui_key, e.key.keysym.sym,
                                             e.key.keysym.scancode);
                }
            }

            if (e.type == SDL_TEXTINPUT) {
                io.AddInputCharactersUTF8(e.text.text);
            }

            if (e.type == SDL_MOUSEBUTTONDOWN &&
                e.button.button == SDL_BUTTON_LEFT) {
                mouseDown = true;
                io.MouseDown[0] = true;
            }

            if (e.type == SDL_MOUSEBUTTONUP &&
                e.button.button == SDL_BUTTON_LEFT) {
                mouseDown = false;
                io.MouseDown[0] = false;
            }

            if (e.type == SDL_MOUSEWHEEL) {
                distance -= e.wheel.y * 10;
                io.MouseWheel = (float)e.wheel.y;
            }

            if (e.type == SDL_MOUSEMOTION) {
                if (mouseDown && !io.WantCaptureMouse) {
                    yaw += e.motion.xrel * 0.3f;
                    pitch += e.motion.yrel * 0.3f;
                }
                io.MousePos = ImVec2((float)e.motion.x, (float)e.motion.y);
            }

            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_o &&
                !io.WantCaptureKeyboard) {
                const char* file = tinyfd_openFileDialog("Open STL", "", 0,
                                                         NULL, "STL file", 0);
                if (file) {
                    render_items.queue_load_stl(file);
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
        bgfx::setViewRect(kOverlayView, 0, 0, width, height);

        float view[16];
        float proj[16];
        bx::mtxLookAt(view, bx::Vec3(0, 0, distance), bx::Vec3(0, 0, 0));
        bx::mtxProj(proj, 60.0f, float(width) / float(height), 0.1f, 1000.0f,
                    bgfx::getCaps()->homogeneousDepth);
        bgfx::setViewTransform(kGBufferView, view, proj);
        bgfx::setViewTransform(kOverlayView, view, proj);
        bgfx::setViewFrameBuffer(kOverlayView, BGFX_INVALID_HANDLE);
        deferred_renderer.setViewportSize(static_cast<uint16_t>(width),
                                          static_cast<uint16_t>(height));
        deferred_renderer.prepareFrame();
        render_items.setViewportSize(width, height);
        render_items.setViewProjection(view, proj);
        render_items.setMeshAxisVisible(showMeshAxis);
        render_items.setVoxelAxisVisible(showVoxelAxis);
        render_items.setMeshVisible(showMesh);
        render_items.setVoxelsVisible(showVoxels);
        render_items.setCollisionVisible(showCollision);
        collision_renderer.setViewportSize(width, height);
        collision_renderer.setViewProjection(view, proj);
        collision_renderer.showAxis = showCollisionAxis;

        float mtx_1[16];
        float mtx_2[16];
        bx::mtxRotateXY(mtx_1, bx::toRad(-pitch), bx::toRad(yaw));
        bx::mtxRotateXY(mtx_2, bx::toRad(pitch), bx::toRad(yaw));
        sinriv::kigstudio::mat::matrix<float> cpu_model_matrix(mtx_1);
        cpu_model_matrix.transpose();
        render_items.setModelMatrix(cpu_model_matrix);

        if (debugPrintRotation) {
            sinriv::kigstudio::mat::matrix<float> gpu_raw_matrix(mtx_1);
            std::cout << "yaw=" << yaw << " pitch=" << pitch << std::endl;
            sinriv::kigstudio::ui::printMatrixAxes("gpu_raw", gpu_raw_matrix);
            sinriv::kigstudio::ui::printMatrixAxes("cpu_model", cpu_model_matrix);
            debugPrintRotation = false;
        }

        render_items.upload_collision(deferred_renderer);
        render_items.render_gbuffer(mtx_2, mesh_render_shader);
        deferred_renderer.render();
        render_items.render_overlay(collision_renderer, mtx_1, mtx_2,
                                    collision_render_shader, mesh_render_shader,
                                    &cpu_model_matrix);

        render_items.process_queue_result();

        io.DisplaySize = ImVec2((float)width, (float)height);
        ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(0.f, 0.f), ImGuiCond_Once);
        ImGui::Begin("STL Loader");

        ImGui::Text("items:%d", render_items.get_num_items());

        if (ImGui::Button("Open STL (O)")) {
            const char* file =
                tinyfd_openFileDialog("Open STL", "", 0, NULL, "STL file", 0);
            if (file) {
                render_items.queue_load_stl(file);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("update collision")) {
            std::cout << "update collision" << std::endl;
            // 应用碰撞体到两个结果体素
            render_items.queue_do_segment();
        }

        ImGui::Checkbox("show mesh", &showMesh);
        ImGui::Checkbox("show collision", &showCollision);
        ImGui::Checkbox("show voxels", &showVoxels);
        ImGui::Checkbox("show mesh axis", &showMeshAxis);
        ImGui::Checkbox("show voxel axis", &showVoxelAxis);
        ImGui::Checkbox("show collision axis", &showCollisionAxis);
        ImGui::End();

        if (render_items.isQueueRunning()) {
            ImGui::SetNextWindowPos(ImVec2((float)width, (float)height),
                                    ImGuiCond_Always, ImVec2(1.0f, 1.0f));
            ImGui::Begin("async_voxel_loader", nullptr,
                         ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoBringToFrontOnFocus);
            ImGui::Text("%s", render_items.getQueueStatus().c_str());
            ImGui::ProgressBar(render_items.getQueueProgress(), ImVec2(-1, 0));
            ImGui::End();
        }

        ImGui::Render();

        imguiEndFrame();

        bgfx::frame();
    }

    render_items.release();

    deferred_renderer.release();
    collision_renderer.release();
    mesh_render_shader.release();
    collision_render_shader.release();
    bgfx::shutdown();
    SDL_DestroyWindow(window);
    SDL_Quit();
    std::cout << "shutdown" << std::endl;
    return 0;
}