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

    sinriv::kigstudio::voxel::VoxelGrid voxel_grid_data;

    sinriv::ui::render::RenderMeshShader mesh_render_shader(kGBufferView, kOverlayView);
    sinriv::ui::render::RenderCollisionShader collision_render_shader(kGBufferView, kOverlayView);

    sinriv::ui::render::RenderDeferred deferred_renderer(kGBufferView, kLightingView, kCollisionView);

    sinriv::ui::render::RenderMesh mesh_renderer{};
    sinriv::ui::render::RenderVoxel voxel_ori_renderer{};
    sinriv::ui::render::RenderVoxel voxel_collision_renderer{};
    sinriv::ui::render::RenderVoxel voxel_spdiv_renderer{};
    
    sinriv::ui::render::RenderCollision collision_renderer{};
    sinriv::ui::render::AsyncVoxelLoader async_voxel_loader;
    sinriv::kigstudio::voxel::collision::CollisionGroup collision_group;
    collision_group.add(sinriv::kigstudio::voxel::collision::Sphere{
        {0.0f, 0.0f, 0.0f}, 35.0f});

    sinriv::kigstudio::voxel::collision::Transform local_cylinder;
    local_cylinder.setPosition({0.0f, 0.0f, -40.0f});
    local_cylinder.setRotationAxisAngle({{1.0f, 0.0f, 0.0f}, bx::kPiHalf/1.5});
    collision_group.add(sinriv::kigstudio::voxel::collision::Cylinder{
        {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 80.0f}, 12.0f}, local_cylinder);

    sinriv::kigstudio::voxel::collision::Transform local_capsule;
    local_capsule.setPosition({55.0f, 0.0f, 0.0f});
    local_capsule.setRotationAxisAngle({{0.0f, 0.0f, 1.0f}, bx::kPiHalf/2});
    collision_group.add(sinriv::kigstudio::voxel::collision::Capsule{
        {-20.0f, 0.0f, 0.0f}, {20.0f, 0.0f, 0.0f}, 10.0f}, local_capsule);

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
    bool showCollisionProcessed_voxel = false;
    bool showSpaceDivProcessed_voxel = false;
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

    auto sdlToImGuiKey = [](SDL_Keycode key) -> ImGuiKey {
        switch (key) {
            case SDLK_TAB: return ImGuiKey_Tab;
            case SDLK_LEFT: return ImGuiKey_LeftArrow;
            case SDLK_RIGHT: return ImGuiKey_RightArrow;
            case SDLK_UP: return ImGuiKey_UpArrow;
            case SDLK_DOWN: return ImGuiKey_DownArrow;
            case SDLK_PAGEUP: return ImGuiKey_PageUp;
            case SDLK_PAGEDOWN: return ImGuiKey_PageDown;
            case SDLK_HOME: return ImGuiKey_Home;
            case SDLK_END: return ImGuiKey_End;
            case SDLK_INSERT: return ImGuiKey_Insert;
            case SDLK_DELETE: return ImGuiKey_Delete;
            case SDLK_BACKSPACE: return ImGuiKey_Backspace;
            case SDLK_SPACE: return ImGuiKey_Space;
            case SDLK_RETURN: return ImGuiKey_Enter;
            case SDLK_ESCAPE: return ImGuiKey_Escape;
            case SDLK_0: return ImGuiKey_0;
            case SDLK_1: return ImGuiKey_1;
            case SDLK_2: return ImGuiKey_2;
            case SDLK_3: return ImGuiKey_3;
            case SDLK_4: return ImGuiKey_4;
            case SDLK_5: return ImGuiKey_5;
            case SDLK_6: return ImGuiKey_6;
            case SDLK_7: return ImGuiKey_7;
            case SDLK_8: return ImGuiKey_8;
            case SDLK_9: return ImGuiKey_9;
            case SDLK_a: return ImGuiKey_A;
            case SDLK_b: return ImGuiKey_B;
            case SDLK_c: return ImGuiKey_C;
            case SDLK_d: return ImGuiKey_D;
            case SDLK_e: return ImGuiKey_E;
            case SDLK_f: return ImGuiKey_F;
            case SDLK_g: return ImGuiKey_G;
            case SDLK_h: return ImGuiKey_H;
            case SDLK_i: return ImGuiKey_I;
            case SDLK_j: return ImGuiKey_J;
            case SDLK_k: return ImGuiKey_K;
            case SDLK_l: return ImGuiKey_L;
            case SDLK_m: return ImGuiKey_M;
            case SDLK_n: return ImGuiKey_N;
            case SDLK_o: return ImGuiKey_O;
            case SDLK_p: return ImGuiKey_P;
            case SDLK_q: return ImGuiKey_Q;
            case SDLK_r: return ImGuiKey_R;
            case SDLK_s: return ImGuiKey_S;
            case SDLK_t: return ImGuiKey_T;
            case SDLK_u: return ImGuiKey_U;
            case SDLK_v: return ImGuiKey_V;
            case SDLK_w: return ImGuiKey_W;
            case SDLK_x: return ImGuiKey_X;
            case SDLK_y: return ImGuiKey_Y;
            case SDLK_z: return ImGuiKey_Z;
            case SDLK_LCTRL: return ImGuiKey_LeftCtrl;
            case SDLK_RCTRL: return ImGuiKey_RightCtrl;
            case SDLK_LSHIFT: return ImGuiKey_LeftShift;
            case SDLK_RSHIFT: return ImGuiKey_RightShift;
            case SDLK_LALT: return ImGuiKey_LeftAlt;
            case SDLK_RALT: return ImGuiKey_RightAlt;
            case SDLK_LGUI: return ImGuiKey_LeftSuper;
            case SDLK_RGUI: return ImGuiKey_RightSuper;
            default: return ImGuiKey_None;
        }
    };

    while (running) {
        SDL_Event e;
        ImGuiIO& io = ImGui::GetIO();
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) {
                ImGuiKey imgui_key = sdlToImGuiKey(e.key.keysym.sym);
                if (imgui_key != ImGuiKey_None) {
                    io.AddKeyEvent(imgui_key, e.type == SDL_KEYDOWN);
                    io.SetKeyEventNativeData(imgui_key, e.key.keysym.sym, e.key.keysym.scancode);
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
                    mesh_renderer.loadSTL(file);
                    async_voxel_loader.start(file);
                }
            }

            if (e.type == SDL_QUIT)
                running = false;
        }

        io.AddKeyEvent(ImGuiMod_Ctrl,  (SDL_GetModState() & KMOD_CTRL) != 0);
        io.AddKeyEvent(ImGuiMod_Shift, (SDL_GetModState() & KMOD_SHIFT) != 0);
        io.AddKeyEvent(ImGuiMod_Alt,   (SDL_GetModState() & KMOD_ALT) != 0);
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
        deferred_renderer.setSpaceDivVisible(showDivSpace);
        mesh_renderer.setViewportSize(width, height);
        voxel_ori_renderer.setViewportSize(width, height);
        voxel_collision_renderer.setViewportSize(width, height);
        voxel_spdiv_renderer.setViewportSize(width, height);
        collision_renderer.setViewportSize(width, height);
        mesh_renderer.setViewProjection(view, proj);
        voxel_ori_renderer.setViewProjection(view, proj);
        voxel_collision_renderer.setViewProjection(view, proj);
        voxel_spdiv_renderer.setViewProjection(view, proj);
        collision_renderer.setViewProjection(view, proj);
        mesh_renderer.showAxis = showMeshAxis;
        voxel_ori_renderer.showAxis = showVoxelAxis;
        voxel_collision_renderer.showAxis = showVoxelAxis;
        voxel_spdiv_renderer.showAxis = showVoxelAxis;
        collision_renderer.showAxis = showCollisionAxis;

        float mtx_1[16];
        float mtx_2[16];
        bx::mtxRotateXY(mtx_1, bx::toRad(-pitch), bx::toRad(yaw));
        bx::mtxRotateXY(mtx_2, bx::toRad(pitch), bx::toRad(yaw));
        sinriv::kigstudio::mat::matrix<float> cpu_model_matrix(mtx_1);
        cpu_model_matrix.transpose();
        mesh_renderer.setModelMatrix(cpu_model_matrix);
        voxel_ori_renderer.setModelMatrix(cpu_model_matrix);
        voxel_collision_renderer.setModelMatrix(cpu_model_matrix);
        voxel_spdiv_renderer.setModelMatrix(cpu_model_matrix);

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
            mesh_renderer.renderGBuffer(mtx_2, mesh_render_shader);
        }

        if (showCollisionProcessed_voxel) {
            showVoxels = false;
            showSpaceDivProcessed_voxel = false;
            voxel_collision_renderer.renderGBuffer(mtx_2, mesh_render_shader);
        }

        if (showSpaceDivProcessed_voxel) {
            showVoxels = false;
            showCollisionProcessed_voxel = false;
            voxel_spdiv_renderer.renderGBuffer(mtx_2, mesh_render_shader);
        }

        if (showVoxels) {
            showCollisionProcessed_voxel = false;
            showSpaceDivProcessed_voxel = false;
            voxel_ori_renderer.renderGBuffer(mtx_2, mesh_render_shader);
        }

        deferred_renderer.render();

        if (showMesh) {
            mesh_renderer.renderOverlay(mesh_render_shader);
        }

        if (showVoxels) {
            voxel_ori_renderer.renderOverlay(mesh_render_shader);
        }
        if (showCollisionProcessed_voxel) {
            voxel_collision_renderer.renderOverlay(mesh_render_shader);
        }
        if (showSpaceDivProcessed_voxel) {
            voxel_spdiv_renderer.renderOverlay(mesh_render_shader);
        }

        if (showCollision) {
            collision_renderer.render(
                collision_group, 
                mtx_1, 
                mtx_2, 
                collision_render_shader, 
                &cpu_model_matrix);
        }

        // Check async voxel loader result
        sinriv::ui::render::AsyncVoxelLoader::MeshData voxel_mesh_data;
        if (async_voxel_loader.tryTakeResult(voxel_mesh_data, voxel_grid_data)) {
            std::cout << "Async voxel loader finished (" << voxel_grid_data.num_chunk() << " chunks)" << std::endl;
            voxel_ori_renderer.loadGeometry(voxel_mesh_data);
        }

        io.DisplaySize = ImVec2((float)width, (float)height);
        ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(0.f, 0.f), ImGuiCond_Once);
        ImGui::Begin("STL Loader");

        if (ImGui::Button("Open STL (O)")) {
            const char* file =
                tinyfd_openFileDialog("Open STL", "", 0, NULL, "STL file", 0);
            if (file) {
                mesh_renderer.loadSTL(file);
                async_voxel_loader.start(file);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("update collision")){
            std::cout << "update collision" << std::endl;
            // 应用碰撞体到两个结果体素
            auto res = voxel_grid_data.difference(collision_group);
            voxel_collision_renderer.loadVoxelGrid(res);
        }

        ImGui::Checkbox("show mesh", &showMesh);
        ImGui::Checkbox("show collision", &showCollision);
        ImGui::Separator();
        if (ImGui::Checkbox("show main voxels", &showVoxels) && showVoxels){
            showCollisionProcessed_voxel = false;
            showSpaceDivProcessed_voxel = false;
        }
        if (ImGui::Checkbox("show collision processed voxel", &showCollisionProcessed_voxel) && showCollisionProcessed_voxel){
            showVoxels = false;
            showSpaceDivProcessed_voxel = false;
        }
        if (ImGui::Checkbox("show space div processed voxel", &showSpaceDivProcessed_voxel) && showSpaceDivProcessed_voxel){
            showVoxels = false;
            showCollisionProcessed_voxel = false;
        }
        ImGui::Separator();
        ImGui::Checkbox("mesh axis", &showMeshAxis);
        ImGui::Checkbox("voxel axis", &showVoxelAxis);
        ImGui::Checkbox("collision axis", &showCollisionAxis);
        ImGui::Separator();
        ImGui::Checkbox("show div space", &showDivSpace);
        // ImGui::Checkbox("debug print rotation", &debugPrintRotation);

        ImGui::End();
        
        if (async_voxel_loader.isRunning()) {
            ImGui::SetNextWindowPos(ImVec2((float)width, (float)height), ImGuiCond_Always, ImVec2(1.0f, 1.0f));
            ImGui::Begin(
                "async_voxel_loader", 
                nullptr,
                ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_NoTitleBar |
                ImGuiWindowFlags_NoBringToFrontOnFocus);
            ImGui::Text("%s", async_voxel_loader.getStatus().c_str());
            ImGui::ProgressBar(async_voxel_loader.getProgress(), ImVec2(-1, 0));
            ImGui::End();
        }

        // 碰撞体成员平移/旋转控制面板（合并到一个带滚动条的窗口）
        ImGui::SetNextWindowPos(ImVec2((float)width, 0.f), ImGuiCond_Once, ImVec2(1.0f, 0.0f));
        ImGui::Begin("Collision Members");
        const char* axisNames[] = {"X", "Y", "Z"};
        int memberIdx = 0;
        float btnSize = ImGui::GetFrameHeight();
        float spacing = ImGui::GetStyle().ItemSpacing.x;
        if (ImGui::CollapsingHeader("space div", ImGuiTreeNodeFlags_DefaultOpen)) {
        }
        if (ImGui::CollapsingHeader("collision root", ImGuiTreeNodeFlags_DefaultOpen)) {
            sinriv::kigstudio::vec3<float> pos = collision_group.transform.getPosition();
            float p[3] = {pos.x, pos.y, pos.z};
            {
                float inputW = 55.0f;
                for (int i = 0; i < 3; ++i) {
                    ImGui::PushID(i);
                    if (i > 0) ImGui::SameLine();
                    if (ImGui::Button("-", ImVec2(btnSize, 0))) p[i] -= 0.5f;
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(inputW);
                    ImGui::DragFloat(axisNames[i], &p[i], 0.5f, 0.0f, 0.0f, "%.2f");
                    ImGui::SameLine();
                    if (ImGui::Button("+", ImVec2(btnSize, 0))) p[i] += 0.5f;
                    ImGui::PopID();
                }
            }
            collision_group.transform.setPosition({p[0], p[1], p[2]});

            // Rotation (deg): 固定宽度紧凑排列
            sinriv::kigstudio::vec3<float> eulerRad = collision_group.transform.getRotationEuler();
            float r[3] = {bx::toDeg(eulerRad.x), bx::toDeg(eulerRad.y), bx::toDeg(eulerRad.z)};
            ImGui::Text("Rotation (deg)");
            {
                float inputW = 55.0f;
                for (int i = 0; i < 3; ++i) {
                    ImGui::PushID(i + 3);
                    if (i > 0) ImGui::SameLine();
                    if (ImGui::Button("-", ImVec2(btnSize, 0))) r[i] -= 0.5f;
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(inputW);
                    ImGui::DragFloat(axisNames[i], &r[i], 0.5f, 0.0f, 0.0f, "%.2f");
                    ImGui::SameLine();
                    if (ImGui::Button("+", ImVec2(btnSize, 0))) r[i] += 0.5f;
                    ImGui::PopID();
                }
            }
            collision_group.transform.setRotationEuler({bx::toRad(r[0]), bx::toRad(r[1]), bx::toRad(r[2])});
        }
        if (ImGui::CollapsingHeader("collision group", ImGuiTreeNodeFlags_DefaultOpen)) {

            for (auto& instance : collision_group.geometries()) {
                const char* typeName = std::visit(
                    [](const auto& geom) -> const char* {
                        using T = std::decay_t<decltype(geom)>;
                        if constexpr (std::is_same_v<T, sinriv::kigstudio::voxel::collision::Sphere>)
                            return "Sphere";
                        if constexpr (std::is_same_v<T, sinriv::kigstudio::voxel::collision::Cylinder>)
                            return "Cylinder";
                        if constexpr (std::is_same_v<T, sinriv::kigstudio::voxel::collision::Capsule>)
                            return "Capsule";
                        if constexpr (std::is_same_v<T, sinriv::kigstudio::voxel::collision::OBB>)
                            return "OBB";
                        return "Unknown";
                    },
                    instance.geometry);

                ImGui::PushID(memberIdx);
                ImGui::Text("%s [%d]", typeName, memberIdx);

                // Position: [-][X][+] [-][Y][+] [-][Z][+] 固定宽度紧凑排列
                sinriv::kigstudio::vec3<float> pos = instance.transform.getPosition();
                float p[3] = {pos.x, pos.y, pos.z};
                ImGui::Text("Position");
                {
                    float inputW = 55.0f;
                    for (int i = 0; i < 3; ++i) {
                        ImGui::PushID(i);
                        if (i > 0) ImGui::SameLine();
                        if (ImGui::Button("-", ImVec2(btnSize, 0))) p[i] -= 0.5f;
                        ImGui::SameLine();
                        ImGui::SetNextItemWidth(inputW);
                        ImGui::DragFloat(axisNames[i], &p[i], 0.5f, 0.0f, 0.0f, "%.2f");
                        ImGui::SameLine();
                        if (ImGui::Button("+", ImVec2(btnSize, 0))) p[i] += 0.5f;
                        ImGui::PopID();
                    }
                }
                instance.transform.setPosition({p[0], p[1], p[2]});

                // Rotation (deg): 固定宽度紧凑排列
                sinriv::kigstudio::vec3<float> eulerRad = instance.transform.getRotationEuler();
                float r[3] = {bx::toDeg(eulerRad.x), bx::toDeg(eulerRad.y), bx::toDeg(eulerRad.z)};
                ImGui::Text("Rotation (deg)");
                {
                    float inputW = 55.0f;
                    for (int i = 0; i < 3; ++i) {
                        ImGui::PushID(i + 3);
                        if (i > 0) ImGui::SameLine();
                        if (ImGui::Button("-", ImVec2(btnSize, 0))) r[i] -= 0.5f;
                        ImGui::SameLine();
                        ImGui::SetNextItemWidth(inputW);
                        ImGui::DragFloat(axisNames[i], &r[i], 0.5f, 0.0f, 0.0f, "%.2f");
                        ImGui::SameLine();
                        if (ImGui::Button("+", ImVec2(btnSize, 0))) r[i] += 0.5f;
                        ImGui::PopID();
                    }
                }
                instance.transform.setRotationEuler({bx::toRad(r[0]), bx::toRad(r[1]), bx::toRad(r[2])});

                ImGui::Separator();
                ImGui::PopID();
                ++memberIdx;
            }
        }
        ImGui::End();

        ImGui::Render();

        imguiEndFrame();

        bgfx::frame();
    }

    deferred_renderer.release();
    collision_renderer.release();
    voxel_ori_renderer.release();
    voxel_collision_renderer.release();
    voxel_spdiv_renderer.release();
    mesh_renderer.release();
    mesh_render_shader.release();
    collision_render_shader.release();
    bgfx::shutdown();
    SDL_DestroyWindow(window);
    SDL_Quit();
    std::cout << "shutdown" << std::endl;
    return 0;
}