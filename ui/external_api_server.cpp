/// external_api_server.cpp – Implementation of ExternalApiServer.
/// All platform-specific socket code is isolated here.

#ifdef _WIN32
#include <winsock2.h>  // must precede windows.h
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#define SOCKET int
#define INVALID_SOCKET (~0U)
#define SOCKET_ERROR (-1)
#define closesocket close
#endif

#include <cstdio>
#include <iostream>

#include "external_api_server.h"

// stb_image_write linkage
#ifdef __cplusplus
extern "C" {
#endif
int stbi_write_png_to_func(
    void (*func)(void* context, void* data, int size),
    void* context, int w, int h, int comp, const void* data, int stride);
#ifdef __cplusplus
}
#endif

namespace sinriv {
namespace ui {
namespace render {

// ===================================================================
// Socket helpers (file-local)
// ===================================================================
namespace {

SOCKET to_sock(uintptr_t v) {
#ifdef _WIN32
    return (SOCKET)v;
#else
    return (SOCKET)(int)v;
#endif
}

uintptr_t from_sock(SOCKET s) {
#ifdef _WIN32
    return (uintptr_t)s;
#else
    return (uintptr_t)(int)s;
#endif
}

void close_sock(uintptr_t& v) {
    SOCKET s = to_sock(v);
    if (to_sock(~uintptr_t{0}) != s) {
        closesocket(s);
        v = from_sock(to_sock(~uintptr_t{0}));
    }
}

struct HttpRequest {
    std::string method;
    std::string path;
    std::string query;
};

HttpRequest parseRequest(const char* raw) {
    HttpRequest req;
    const char* sp = raw;
    while (*sp && *sp != ' ') sp++;
    if (*sp == ' ') {
        req.method = std::string(raw, sp - raw);
        sp++;
        const char* path_start = sp;
        while (*sp && *sp != ' ' && *sp != '?') sp++;
        req.path = std::string(path_start, sp - path_start);
        if (*sp == '?') {
            sp++;
            const char* q_start = sp;
            while (*sp && *sp != ' ') sp++;
            req.query = std::string(q_start, sp - q_start);
        }
    }
    return req;
}

float parseQueryFloat(const std::string& query, const char* key,
                      float default_val) {
    std::string prefix = std::string(key) + "=";
    size_t pos = query.find(prefix);
    if (pos == std::string::npos) return default_val;
    pos += prefix.size();
    size_t end = query.find('&', pos);
    std::string val = (end == std::string::npos) ? query.substr(pos)
                                                  : query.substr(pos, end - pos);
    try { return std::stof(val); }
    catch (...) { return default_val; }
}

bool sendHttp(SOCKET client, int code, const char* content_type,
              const void* data, size_t len) {
    char header[512];
    int hdr_len = snprintf(header, sizeof(header),
        "HTTP/1.0 %d OK\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %zu\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n"
        "\r\n",
        code, content_type, len);
    if (::send(client, header, hdr_len, 0) != hdr_len) return false;
    if (len > 0 && ::send(client, (const char*)data, (int)len, 0) != (int)len)
        return false;
    return true;
}

}  // anonymous namespace

// ===================================================================
// Lifecycle
// ===================================================================

ExternalApiServer::~ExternalApiServer() { stop(); }

void ExternalApiServer::start(int port) {
    if (running_.load()) return;
    port_ = port;

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "[api_server] WSAStartup failed" << std::endl;
        return;
    }
#endif

    SOCKET s = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s == to_sock(~uintptr_t{0})) {
        std::cerr << "[api_server] socket() failed" << std::endl;
        return;
    }

    int opt = 1;
    ::setsockopt(s, SOL_SOCKET, SO_REUSEADDR,
#ifdef _WIN32
                 (const char*)&opt, sizeof(opt));
#else
                 &opt, sizeof(opt));
#endif

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr.sin_port = htons(static_cast<uint16_t>(port_));

    if (::bind(s, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        std::cerr << "[api_server] bind() failed on port " << port_ << std::endl;
        closesocket(s);
        return;
    }
    if (::listen(s, 2) == SOCKET_ERROR) {
        std::cerr << "[api_server] listen() failed" << std::endl;
        closesocket(s);
        return;
    }

    listen_socket_ = from_sock(s);
    running_ = true;
    thread_ = std::thread(&ExternalApiServer::serverLoop, this);
    std::cout << "[api_server] Listening on http://127.0.0.1:" << port_
              << std::endl;
}

void ExternalApiServer::stop() {
    if (!running_.load()) return;
    running_ = false;

    if (to_sock(listen_socket_) != to_sock(~uintptr_t{0})) {
        closesocket(to_sock(listen_socket_));
        listen_socket_ = from_sock(to_sock(~uintptr_t{0}));
    }

    if (thread_.joinable()) thread_.join();

#ifdef _WIN32
    WSACleanup();
#endif
    std::cout << "[api_server] Stopped" << std::endl;
}

void ExternalApiServer::serverLoop() {
    while (running_.load()) {
        SOCKET ls = to_sock(listen_socket_);
        struct sockaddr_in client_addr {};
        socklen_t client_len = sizeof(client_addr);
        SOCKET client = ::accept(ls, (struct sockaddr*)&client_addr, &client_len);
        if (client == to_sock(~uintptr_t{0})) {
            if (running_.load())
                std::cerr << "[api_server] accept() failed" << std::endl;
            break;
        }

        char* client_ip = inet_ntoa(client_addr.sin_addr);
        if (std::strcmp(client_ip, "127.0.0.1") != 0) {
            closesocket(client);
            continue;
        }

        handleClient(from_sock(client));
        closesocket(client);
    }
}

// ===================================================================
// Blended PNG (member function – can access private data)
// ===================================================================

bool ExternalApiServer::sendBlendedPng(uintptr_t client_sock,
                                        float blend_ratio) {
    SOCKET client = to_sock(client_sock);
    std::lock_guard<std::mutex> lock(mtx_);

    int out_w = render_valid_ ? render_w_ : 512;
    int out_h = render_valid_ ? render_h_ : 512;
    if (out_w <= 0 || out_h <= 0) {
        const char* msg = "{\"error\":\"no render data available\"}";
        return sendHttp(client, 503, "application/json", msg, strlen(msg));
    }

    std::vector<uint8_t> out(out_w * out_h * 4, 0);

    if (render_valid_) {
        for (int y = 0; y < out_h; y++) {
            for (int x = 0; x < out_w; x++) {
                int src = (y * render_w_ + x) * 4;
                int dst = (y * out_w + x) * 4;
                if (x < render_w_ && y < render_h_) {
                    out[dst + 0] = render_rgba_[src + 0];
                    out[dst + 1] = render_rgba_[src + 1];
                    out[dst + 2] = render_rgba_[src + 2];
                    out[dst + 3] = 255;
                } else {
                    out[dst + 0] = out[dst + 1] = out[dst + 2] = 48;
                    out[dst + 3] = 255;
                }
            }
        }
    } else {
        for (int i = 0; i < out_w * out_h * 4; i += 4) {
            out[i + 0] = out[i + 1] = out[i + 2] = 48;
            out[i + 3] = 255;
        }
    }

    if (overlay_active_ && overlay_valid_ && blend_ratio > 0.001f) {
        float ratio = std::max(0.0f, std::min(1.0f, blend_ratio));
        int ov_w = overlay_w_, ov_h = overlay_h_;
        float sc = overlay_scale_;
        if (sc <= 0.0f) sc = 1.0f;
        int placed_w = (int)(ov_w * sc), placed_h = (int)(ov_h * sc);
        int off_x = (int)overlay_off_x_, off_y = (int)overlay_off_y_;

        for (int dy = 0; dy < placed_h; dy++) {
            int sy = (int)(dy / sc);
            if (sy < 0 || sy >= ov_h) continue;
            int py = off_y + dy;
            if (py < 0 || py >= out_h) continue;
            for (int dx = 0; dx < placed_w; dx++) {
                int sx = (int)(dx / sc);
                if (sx < 0 || sx >= ov_w) continue;
                int px = off_x + dx;
                if (px < 0 || px >= out_w) continue;
                int oi = (sy * ov_w + sx) * 4;
                int di = (py * out_w + px) * 4;
                float ov_a = overlay_rgba_[oi + 3] / 255.0f * ratio;
                out[di + 0] = (uint8_t)(out[di + 0] * (1.f - ov_a) + overlay_rgba_[oi + 0] * ov_a);
                out[di + 1] = (uint8_t)(out[di + 1] * (1.f - ov_a) + overlay_rgba_[oi + 1] * ov_a);
                out[di + 2] = (uint8_t)(out[di + 2] * (1.f - ov_a) + overlay_rgba_[oi + 2] * ov_a);
            }
        }
    }

    // Stream PNG via chunked transfer
    char header[256];
    int hdr_len = snprintf(header, sizeof(header),
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: image/png\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n"
        "Transfer-Encoding: chunked\r\n"
        "\r\n");
    if (::send(client, header, hdr_len, 0) != hdr_len) return false;

    struct PngCtx { SOCKET c; std::vector<uint8_t> hdr; bool sent = false; };
    PngCtx ctx{client, {}, false};
    auto cb = [](void* cx, void* d, int sz) {
        auto* p = (PngCtx*)cx;
        if (!p->sent) { p->hdr.insert(p->hdr.end(), (uint8_t*)d, (uint8_t*)d + sz); }
        else { ::send(p->c, (const char*)d, sz, 0); }
    };
    int ok = stbi_write_png_to_func(cb, &ctx, out_w, out_h, 4,
                                     out.data(), out_w * 4);
    if (!ctx.hdr.empty()) {
        char chdr[32];
        int cl = snprintf(chdr, sizeof(chdr), "%zx\r\n", ctx.hdr.size());
        ::send(client, chdr, cl, 0);
        ::send(client, (const char*)ctx.hdr.data(), (int)ctx.hdr.size(), 0);
        ::send(client, "\r\n", 2, 0);
        ctx.sent = true;
    }
    ::send(client, "0\r\n\r\n", 5, 0);
    return ok != 0;
}

// ===================================================================
// Request dispatcher (member function)
// ===================================================================

bool ExternalApiServer::handleClient(uintptr_t client_sock) {
    SOCKET client = to_sock(client_sock);
    char buf[4096];
    int recvd = ::recv(client, buf, sizeof(buf) - 1, 0);
    if (recvd <= 0) return false;
    buf[recvd] = '\0';

    HttpRequest req = parseRequest(buf);

    // GET /ping
    if (req.path == "/ping") {
        const char* json = "{\"ok\":true,\"service\":\"kigstudio-ortho-api\"}";
        return sendHttp(client, 200, "application/json", json, strlen(json));
    }

    // GET /state
    if (req.path == "/state") {
        std::lock_guard<std::mutex> lock(mtx_);
        std::string json = state_json_.empty() ? "{}" : state_json_;
        return sendHttp(client, 200, "application/json",
                        json.data(), json.size());
    }

    // GET /render
    if (req.path == "/render") {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!render_valid_) {
            const char* msg = "{\"error\":\"no render data available\"}";
            return sendHttp(client, 503, "application/json", msg, strlen(msg));
        }
        std::vector<uint8_t> png;
        png.reserve(render_w_ * render_h_ / 2);
        struct MC { std::vector<uint8_t>* b; } mc{&png};
        auto mw = [](void* cx, void* d, int sz) {
            ((MC*)cx)->b->insert(((MC*)cx)->b->end(), (uint8_t*)d, (uint8_t*)d + sz);
        };
        int ok = stbi_write_png_to_func(mw, &mc, render_w_, render_h_, 4,
                                         render_rgba_.data(), render_w_ * 4);
        if (!ok) {
            const char* msg = "{\"error\":\"png encode failed\"}";
            return sendHttp(client, 500, "application/json", msg, strlen(msg));
        }
        return sendHttp(client, 200, "image/png", png.data(), png.size());
    }

    // GET /overlay
    if (req.path == "/overlay") {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!overlay_valid_) {
            const char* msg = "{\"error\":\"no overlay loaded\"}";
            return sendHttp(client, 404, "application/json", msg, strlen(msg));
        }
        std::vector<uint8_t> png;
        png.reserve(overlay_w_ * overlay_h_ / 2);
        struct MC { std::vector<uint8_t>* b; } mc{&png};
        auto mw = [](void* cx, void* d, int sz) {
            ((MC*)cx)->b->insert(((MC*)cx)->b->end(), (uint8_t*)d, (uint8_t*)d + sz);
        };
        int ok = stbi_write_png_to_func(mw, &mc, overlay_w_, overlay_h_, 4,
                                         overlay_rgba_.data(), overlay_w_ * 4);
        if (!ok) {
            const char* msg = "{\"error\":\"png encode failed\"}";
            return sendHttp(client, 500, "application/json", msg, strlen(msg));
        }
        return sendHttp(client, 200, "image/png", png.data(), png.size());
    }

    // GET /blend?ratio=0.5
    if (req.path == "/blend") {
        float ratio = parseQueryFloat(req.query, "ratio", 0.5f);
        return sendBlendedPng(client_sock, ratio);
    }

    // 404
    const char* nf = "{\"error\":\"not found\",\"endpoints\":[\"/ping\","
                     "\"/state\",\"/render\",\"/overlay\",\"/blend?ratio=0.5\"]}";
    return sendHttp(client, 404, "application/json", nf, strlen(nf));
}

// ===================================================================
// Data-provider setters
// ===================================================================
void ExternalApiServer::setRenderData(const uint8_t* rgba, int w, int h) {
    if (!rgba || w <= 0 || h <= 0) return;
    std::lock_guard<std::mutex> lock(mtx_);
    size_t size = (size_t)w * h * 4;
    render_rgba_.resize(size);
    memcpy(render_rgba_.data(), rgba, size);
    render_w_ = w;
    render_h_ = h;
    render_valid_ = true;
}

void ExternalApiServer::setOverlayData(const uint8_t* rgba, int w, int h) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!rgba || w <= 0 || h <= 0) {
        overlay_rgba_.clear();
        overlay_w_ = overlay_h_ = 0;
        overlay_valid_ = false;
        return;
    }
    size_t size = (size_t)w * h * 4;
    overlay_rgba_.resize(size);
    memcpy(overlay_rgba_.data(), rgba, size);
    overlay_w_ = w;
    overlay_h_ = h;
    overlay_valid_ = true;
}

void ExternalApiServer::setOverlayParams(float offset_x, float offset_y,
                                         float scale, float blend_ratio) {
    std::lock_guard<std::mutex> lock(mtx_);
    overlay_off_x_ = offset_x;
    overlay_off_y_ = offset_y;
    overlay_scale_ = scale;
    overlay_blend_ = blend_ratio;
}

void ExternalApiServer::setStateJson(const std::string& json) {
    std::lock_guard<std::mutex> lock(mtx_);
    state_json_ = json;
}

}  // namespace render
}  // namespace ui
}  // namespace sinriv
