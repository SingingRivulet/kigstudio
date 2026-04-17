#define SDL_MAIN_HANDLED

#include <SDL.h>
#include <SDL_syswm.h>
#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <bx/math.h>
#include <iostream>

#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <stb/stb_truetype.h>

#include "kigstudio/ui/logger.h"
#include "kigstudio/ui/render_collision.h"
#include "kigstudio/ui/render_deferred.h"
#include "kigstudio/ui/render_mesh.h"
#include "kigstudio/ui/render_voxel.h"
#include "kigstudio/voxel/collision.h"
#include "tinyfiledialogs.h"

// --------- Main ---------
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
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE  // 移除 SDL_WINDOW_OPENGL
    );

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
    bgfx::setViewRect(kGBufferView, 0, 0, width, height);
    bgfx::setViewRect(kLightingView, 0, 0, width, height);
    bgfx::setViewRect(kCollisionView, 0, 0, width, height);
    bgfx::setViewRect(kOverlayView, 0, 0, width, height);

    sinriv::ui::render::RenderDeferred deferred_renderer(kGBufferView, kLightingView, kCollisionView);
    sinriv::ui::render::RenderMesh mesh_renderer(kGBufferView, kOverlayView);
    sinriv::ui::render::RenderVoxel voxel_renderer(kGBufferView, kOverlayView);
    sinriv::ui::render::RenderCollision collision_renderer(kOverlayView, kOverlayView);
    sinriv::kigstudio::voxel::collision::CollisionGroup collision_group;
    collision_group.add(sinriv::kigstudio::voxel::collision::Sphere{
        {0.0f, 0.0f, 0.0f}, 35.0f});

    sinriv::kigstudio::voxel::collision::Transform local_cylinder;
    local_cylinder.setPosition({0.0f, 0.0f, -40.0f});
    collision_group.add(sinriv::kigstudio::voxel::collision::Cylinder{
        {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 80.0f}, 12.0f}, local_cylinder);

    sinriv::kigstudio::voxel::collision::Transform local_capsule;
    local_capsule.setPosition({55.0f, 0.0f, 0.0f});
    local_capsule.setRotationAxisAngle({{0.0f, 1.0f, 0.0f}, bx::kPiHalf});
    collision_group.add(sinriv::kigstudio::voxel::collision::Capsule{
        {0.0f, 0.0f, -20.0f}, {0.0f, 0.0f, 20.0f}, 10.0f}, local_capsule);

    sinriv::kigstudio::voxel::collision::Transform local_obb;
    local_obb.setPosition({-60.0f, 0.0f, 0.0f});
    local_obb.setRotationEuler({0.0f, 0.0f, bx::kPi / 6.0f});
    collision_group.add(sinriv::kigstudio::voxel::collision::OBB{
        {0.0f, 0.0f, 0.0f},
        {14.0f, 24.0f, 18.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}}, local_obb);

    bool running = true;
    bool showMesh = true;
    bool showVoxels = false;
    bool showCollision = true;
    bool showMeshAxis = true;
    bool showVoxelAxis = false;
    bool showCollisionAxis = true;
    bool showDivSpace = true;
    bool debugPrintRotation = false;
    int oldW = width;
    int oldH = height;

    auto printMatrixAxes = [](const char* label,
                              const sinriv::kigstudio::mat::matrix<float>& matrix) {
        std::cout << label
                  << " X=(" << matrix[0][0] << ", " << matrix[0][1] << ", " << matrix[0][2] << ")"
                  << " Y=(" << matrix[1][0] << ", " << matrix[1][1] << ", " << matrix[1][2] << ")"
                  << " Z=(" << matrix[2][0] << ", " << matrix[2][1] << ", " << matrix[2][2] << ")"
                  << " T=(" << matrix[3][0] << ", " << matrix[3][1] << ", " << matrix[3][2] << ")"
                  << std::endl;
    };

    while (running) {
        SDL_Event e;
        ImGuiIO& io = ImGui::GetIO();
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT)
                running = false;

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
                if (mouseDown) {
                    yaw += e.motion.xrel * 0.3f;
                    pitch += e.motion.yrel * 0.3f;
                }
                io.MousePos = ImVec2((float)e.motion.x, (float)e.motion.y);
            }

            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_o) {
                const char* file = tinyfd_openFileDialog("Open STL", "", 0,
                                                         NULL, "STL file", 0);
                if (file) {
                    mesh_renderer.loadSTL(file);
                    voxel_renderer.loadSTLAsVoxel(file);
                }
            }

            if (e.type == SDL_TEXTINPUT) {
                io.AddInputCharactersUTF8(e.text.text);
                break;
            }
        }

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
        deferred_renderer.setSpaceDivVisible(showDivSpace);
        mesh_renderer.setViewportSize(width, height);
        voxel_renderer.setViewportSize(width, height);
        collision_renderer.setViewportSize(width, height);
        mesh_renderer.setViewProjection(view, proj);
        voxel_renderer.setViewProjection(view, proj);
        collision_renderer.setViewProjection(view, proj);
        mesh_renderer.showAxis = showMeshAxis;
        voxel_renderer.showAxis = showVoxelAxis;
        collision_renderer.showAxis = showCollisionAxis;

        float mtx_1[16];
        float mtx_2[16];
        bx::mtxRotateXY(mtx_1, bx::toRad(-pitch), bx::toRad(yaw));
        bx::mtxRotateXY(mtx_2, bx::toRad(pitch), bx::toRad(yaw));
        sinriv::kigstudio::mat::matrix<float> cpu_model_matrix(mtx_1);
        cpu_model_matrix.transpose();
        mesh_renderer.setModelMatrix(cpu_model_matrix);
        voxel_renderer.setModelMatrix(cpu_model_matrix);

        if (debugPrintRotation) {
            sinriv::kigstudio::mat::matrix<float> gpu_raw_matrix(mtx_1);
            std::cout << "yaw=" << yaw << " pitch=" << pitch << std::endl;
            printMatrixAxes("gpu_raw", gpu_raw_matrix);
            printMatrixAxes("cpu_model", cpu_model_matrix);
            debugPrintRotation = false;
        }

        if (showCollision) {
            deferred_renderer.setCollisionGroup(collision_group);
        } else {
            deferred_renderer.clearCollisionTint();
        }

        if (showMesh) {
            mesh_renderer.renderGBuffer(mtx_2);
        }

        if (showVoxels) {
            voxel_renderer.renderGBuffer(mtx_2);
        }

        deferred_renderer.render();

        if (showMesh) {
            mesh_renderer.renderOverlay();
        }

        if (showVoxels) {
            voxel_renderer.renderOverlay();
        }

        if (showCollision) {
            collision_renderer.render(collision_group, mtx_1, &cpu_model_matrix);
        }

        ImGui::NewFrame();
        ImGui::Begin("STL Loader");

        if (ImGui::Button("Open STL (O)")) {
            const char* file =
                tinyfd_openFileDialog("Open STL", "", 0, NULL, "STL file", 0);
            if (file) {
                mesh_renderer.loadSTL(file);
                voxel_renderer.loadSTLAsVoxel(file);
            }
        }

        ImGui::Checkbox("show mesh", &showMesh);
        ImGui::Checkbox("show voxels", &showVoxels);
        ImGui::Checkbox("show collision", &showCollision);
        ImGui::Checkbox("mesh axis", &showMeshAxis);
        ImGui::Checkbox("voxel axis", &showVoxelAxis);
        ImGui::Checkbox("collision axis", &showCollisionAxis);
        ImGui::Checkbox("div space", &showDivSpace);
        ImGui::Checkbox("debug print rotation", &debugPrintRotation);

        ImGui::End();
        ImGui::Render();

        imguiEndFrame();

        bgfx::frame();
    }

    deferred_renderer.release();
    collision_renderer.release();
    voxel_renderer.release();
    mesh_renderer.release();
    bgfx::shutdown();
    SDL_DestroyWindow(window);
    SDL_Quit();
    std::cout << "shutdown" << std::endl;
    return 0;
}
