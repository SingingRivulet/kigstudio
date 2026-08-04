#pragma once
/// external_api_server.h – Simple embedded HTTP server for external tool
/// integration.  Listens on localhost and serves the current ortho render,
/// overlay image, blended result, and camera-state JSON.
///
/// The full implementation lives in external_api_server.cpp to avoid
/// pulling winsock2 into every translation unit that includes this header.

#include <atomic>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sinriv {
namespace ui {
namespace render {

class ExternalApiServer {
   public:
    ExternalApiServer() = default;
    ~ExternalApiServer();

    // Non-copyable
    ExternalApiServer(const ExternalApiServer&) = delete;
    ExternalApiServer& operator=(const ExternalApiServer&) = delete;

    // ---- Lifecycle ----
    void start(int port = 19876);
    void stop();
    bool isRunning() const { return running_.load(); }
    int port() const { return port_; }

    // ---- Data providers (called from UI thread to update caches) ----
    void setRenderData(const uint8_t* rgba, int w, int h);
    void setOverlayData(const uint8_t* rgba, int w, int h);
    void setOverlayParams(float offset_x, float offset_y,
                          float scale, float blend_ratio);
    void setOverlayActive(bool active) { overlay_active_ = active; }
    void setStateJson(const std::string& json);

   private:
    void serverLoop();
    // Implementation helpers – defined in external_api_server.cpp
    bool handleClient(uintptr_t client_sock);
    bool sendBlendedPng(uintptr_t client_sock, float blend_ratio);

    std::thread thread_;
    std::atomic<bool> running_{false};
    int port_ = 19876;

    // Opaque socket handle (large enough for SOCKET on all platforms)
    uintptr_t listen_socket_ = ~uintptr_t{0};  // INVALID_SOCKET sentinel

    // ---- Cached data (mutex-protected: UI thread writes, server reads) ----
    std::mutex mtx_;
    std::vector<uint8_t> render_rgba_;
    int render_w_ = 0, render_h_ = 0;
    bool render_valid_ = false;

    std::vector<uint8_t> overlay_rgba_;
    int overlay_w_ = 0, overlay_h_ = 0;
    bool overlay_valid_ = false;
    bool overlay_active_ = false;

    float overlay_off_x_ = 0.f, overlay_off_y_ = 0.f;
    float overlay_scale_ = 1.f, overlay_blend_ = 0.5f;

    std::string state_json_;
};

}  // namespace render
}  // namespace ui
}  // namespace sinriv
