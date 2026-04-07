#define SDL_MAIN_HANDLED

#include <SDL.h>
#include <SDL_syswm.h>
#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <bx/math.h>
#include <iostream>
#include <tuple>
#include <vector>

#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <stb/stb_truetype.h>

#include "kigstudio/ui/logger.h"
#include "kigstudio/voxel/voxelizer_svo.h"
#include "kigstudio/voxel/voxel2mesh.h"
#include "tinyfiledialogs.h"

struct Mesh {
    bgfx::VertexBufferHandle vbh = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle ibh = BGFX_INVALID_HANDLE;
    uint32_t indexCount = 0;
};

struct PosNormalVertex {
    float x, y, z;
    float nx, ny, nz;

    static void init(bgfx::VertexLayout& layout) {
        layout.begin()
            .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::Normal, 3, bgfx::AttribType::Float)
            .end();
    }
};

template <class T>
void loadMesh(T&& stl, bgfx::VertexLayout& layout, Mesh& mesh) {
    std::vector<PosNormalVertex> vertices;
    std::vector<uint32_t> indices;

    for (auto [tri, n] : stl) {
        uint32_t base = static_cast<uint32_t>(vertices.size());
        vertices.push_back({std::get<0>(tri).x, std::get<0>(tri).y,
                            std::get<0>(tri).z, n.x, n.y, n.z});
        vertices.push_back({std::get<1>(tri).x, std::get<1>(tri).y,
                            std::get<1>(tri).z, n.x, n.y, n.z});
        vertices.push_back({std::get<2>(tri).x, std::get<2>(tri).y,
                            std::get<2>(tri).z, n.x, n.y, n.z});

        indices.push_back(base);
        indices.push_back(base + 1);
        indices.push_back(base + 2);
    }

    if (vertices.empty())
        return;

    if (bgfx::isValid(mesh.vbh))
        bgfx::destroy(mesh.vbh);
    if (bgfx::isValid(mesh.ibh))
        bgfx::destroy(mesh.ibh);

    mesh.vbh = bgfx::createVertexBuffer(
        bgfx::copy(vertices.data(), vertices.size() * sizeof(PosNormalVertex)),
        layout);

    mesh.ibh = bgfx::createIndexBuffer(
        bgfx::copy(indices.data(), indices.size() * sizeof(uint32_t)),
        BGFX_BUFFER_INDEX32);

    mesh.indexCount = indices.size();
    std::cout << "STL loaded: " << mesh.indexCount << " indices\n";
}

// --------- STL Loader ---------
void loadSTL(const std::string& filename,
             bgfx::VertexLayout& layout,
             Mesh& mesh,
             Mesh& voxels) {
    loadMesh(sinriv::kigstudio::voxel::readSTL(filename), layout, mesh);
    sinriv::kigstudio::voxel::triangle_bvh<float> bvh;
    for (auto [tri, n] : sinriv::kigstudio::voxel::readSTL(filename)) {
        bvh.insert(tri);
    }
    sinriv::kigstudio::octree::Octree voxelData(bvh.getOctreeVoxelEdgeSize(1,1,1));
    sinriv::kigstudio::voxel::create_solid_mesh(voxelData,bvh, 1, 1, 1);
    double isolevel = 0.5;
    int numTriangles = 0;
    loadMesh(sinriv::kigstudio::voxel::generateMesh(voxelData, isolevel, numTriangles, true), layout, voxels);
}

// --------- Shader Loader ---------
bgfx::ShaderHandle loadShader(const char* path) {
    FILE* file = fopen(path, "rb");
    if (!file)
        return BGFX_INVALID_HANDLE;
    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fseek(file, 0, SEEK_SET);
    std::vector<char> data(size);
    fread(data.data(), 1, size, file);
    fclose(file);
    return bgfx::createShader(bgfx::copy(data.data(), data.size()));
}

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

    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0x303030ff, 1.0f,
                       0);

    imguiCreate();

    int width, height;
    SDL_GetWindowSize(window, &width, &height);
    bgfx::setViewRect(0, 0, 0, width, height);

    // load shaders
    bgfx::ProgramHandle program = bgfx::createProgram(
        loadShader("../../shader/base/vs_mesh_base.bin"),
        loadShader("../../shader/base/fs_mesh_base.bin"), true);

    bgfx::VertexLayout layout = {};
    PosNormalVertex::init(layout);
    Mesh mesh, voxels;

    bool running = true;
    bool showMesh = true;
    bool showVoxels = false;
    int oldW = width;
    int oldH = height;

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
                    loadSTL(file, layout, mesh, voxels);
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
        bgfx::setViewRect(0, 0, 0, width, height);

        bgfx::touch(0);

        float view[16];
        float proj[16];
        bx::mtxLookAt(view, bx::Vec3(0, 0, distance), bx::Vec3(0, 0, 0));
        bx::mtxProj(proj, 60.0f, float(width) / float(height), 0.1f, 1000.0f,
                    bgfx::getCaps()->homogeneousDepth);
        bgfx::setViewTransform(0, view, proj);

        if (showMesh && bgfx::isValid(mesh.vbh)) {
            float mtx[16];
            bx::mtxRotateXY(mtx, bx::toRad(pitch), bx::toRad(yaw));
            bgfx::setTransform(mtx);
            bgfx::setVertexBuffer(0, mesh.vbh);
            bgfx::setIndexBuffer(mesh.ibh);
            bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                           BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS |
                           BGFX_STATE_CULL_CCW | BGFX_STATE_MSAA);
            bgfx::submit(0, program);
        }

        if (showVoxels && bgfx::isValid(voxels.vbh)) {
            float mtx[16];
            bx::mtxRotateXY(mtx, bx::toRad(pitch), bx::toRad(yaw));
            bgfx::setTransform(mtx);
            bgfx::setVertexBuffer(0, voxels.vbh);
            bgfx::setIndexBuffer(voxels.ibh);
            bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                           BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS |
                           BGFX_STATE_CULL_CCW | BGFX_STATE_MSAA);
            bgfx::submit(0, program);
        }

        ImGui::NewFrame();
        ImGui::Begin("STL Loader");

        if (ImGui::Button("Open STL (O)")) {
            const char* file =
                tinyfd_openFileDialog("Open STL", "", 0, NULL, "STL file", 0);
            if (file) {
                loadSTL(file, layout, mesh, voxels);
            }
        }

        ImGui::Checkbox("show mesh", &showMesh);
        ImGui::Checkbox("show voxels", &showVoxels);

        ImGui::End();
        ImGui::Render();

        imguiEndFrame();

        bgfx::frame();
    }

    bgfx::shutdown();
    SDL_DestroyWindow(window);
    SDL_Quit();
    std::cout << "shutdown" << std::endl;
    return 0;
}