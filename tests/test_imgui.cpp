#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <SDL_syswm.h>
#include <iostream>

#include <bgfx/bgfx.h>
#include <bgfx/platform.h>

// 1. 定义一个回调结构体
struct Callback : public bgfx::CallbackI {
    void fatal(const char* _filePath,
               uint16_t _line,
               bgfx::Fatal::Enum _code,
               const char* _str) override {
        std::cerr << "BGFX FATAL: " << _str << " (" << _filePath << ":" << _line
                  << ")" << std::endl;
        // 这里可以选择 abort() 让程序崩溃以便调试器捕获，或者仅仅打印
    }

    void traceVargs(const char* _filePath,
                    uint16_t _line,
                    const char* _format,
                    va_list _argList) override {
        // char temp[1024];
        // vsnprintf(temp, sizeof(temp), _format, _argList);
        // std::cout << "BGFX TRACE: " << temp << std::endl;
    }

    void profilerBegin(const char* /*_name*/,
                       uint32_t /*_abgr*/,
                       const char* /*_filePath*/,
                       uint16_t /*_line*/) override {}
    void profilerBeginLiteral(const char* /*_name*/,
                              uint32_t /*_abgr*/,
                              const char* /*_filePath*/,
                              uint16_t /*_line*/) override {}
    void profilerEnd() override {}
    uint32_t cacheReadSize(uint64_t _id) override { return 0; }
    void cacheWrite(uint64_t _id, const void* _data, uint32_t _size) override {}
    void screenShot(const char* _filePath,
                    uint32_t _width,
                    uint32_t _height,
                    uint32_t _pitch,
                    bgfx::TextureFormat::Enum _format,
                    const void* _data,
                    uint32_t _size,
                    bool _yflip) override {}
    void captureBegin(uint32_t /*_width*/,
                      uint32_t /*_height*/,
                      uint32_t /*_pitch*/,
                      bgfx::TextureFormat::Enum /*_format*/,
                      bool /*_yflip*/) override {}
    void captureEnd() override {}
    void captureFrame(const void* _data, uint32_t _size) override {}
    bool cacheRead(uint64_t _id, void* _data, uint32_t _size) override {
        return false;
    }
};

// 2. 实例化并设置回调
static Callback s_callback;

int main(int argc, char** argv) {
    std::cout << "Initializing SDL" << std::endl;
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << std::endl;
        return -1;
    }
    std::cout << "SDL initialized" << std::endl;

    SDL_Window* window = SDL_CreateWindow(
        "bgfx + SDL", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (!window) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return -1;
    }
    std::cout << "SDL window created" << std::endl;

    SDL_SysWMinfo wmi;
    SDL_VERSION(&wmi.version);
    if (!SDL_GetWindowWMInfo(window, &wmi)) {
        std::cerr << "SDL_GetWindowWMInfo failed" << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    if (wmi.subsystem != SDL_SYSWM_WINDOWS) {
        std::cerr << "Not using Win32 window backend!" << std::endl;
        return -1;
    }


    std::cout << "Initializing bgfx platform" << std::endl;

    bgfx::PlatformData pd{};
    std::cout << "HWND = " << wmi.info.win.window << std::endl;
    
    pd.nwh = wmi.info.win.window;
    pd.ndt = nullptr;
    pd.context = nullptr;
    pd.backBuffer = nullptr;
    pd.backBufferDS = nullptr;
    // bgfx::setPlatformData(pd);

    // if (bgfx::getRendererType() == bgfx::RendererType::Count) {
    //     // 显示支持的渲染器列表
    //     uint8_t supported;
    //     bgfx::getSupportedRenderers(&supported);
    // }

    bgfx::Init init{};
    init.type = bgfx::RendererType::OpenGL;  // OpenGL/Direct3D11任选
    init.resolution.width = 1280;
    init.resolution.height = 720;
    init.resolution.reset = BGFX_RESET_VSYNC;
    init.vendorId = BGFX_PCI_ID_NONE;  // 不指定厂商ID，使用任意
    init.deviceId = 0;                 // 不指定设备ID，使用任意
    init.callback = &s_callback;
    init.platformData = pd;

    std::cout << "Initializing bgfx" << std::endl;
    if (!bgfx::init(init)) {
        std::cerr << "Failed to initialize bgfx" << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }
    std::cout << "bgfx initialized" << std::endl;

    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0x303030ff, 1.0f,
                       0);
    
    int width, height;
    SDL_GetWindowSize(window, &width, &height);

    bool running = true;
    int oldW = width;
    int oldH = height;
    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT)
                running = false;
        }
        // ===== 读取窗口尺寸 =====
        SDL_GetWindowSize(window, &width, &height);

        // ===== 如果尺寸变化就 reset =====
        if (width != oldW || height != oldH)
        {
            bgfx::reset(width, height, BGFX_RESET_VSYNC);
            oldW = width;
            oldH = height;
            std::cout << "Window resized to " << width << "x" << height << std::endl;
        }

        // ===== 关键：view 随窗口变化 =====
        bgfx::setViewRect(0, 0, 0, width, height);

        bgfx::touch(0);                           // 提交一个空 draw call
        bgfx::frame();                            // 提交一帧

    }

    bgfx::shutdown();
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}