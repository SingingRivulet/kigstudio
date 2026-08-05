/**
 * Embedded HTTP server implementation (cpp-httplib backend).
 */

#include "kigstudio/agent/agent_server.h"

// Windows sockets MUST come before windows.h (otherwise winsock.h vs
// winsock2.h conflict in httplib)
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <windows.h>
#undef TEXT  // windows.h defines TEXT macro, conflicts with httplib
#endif

#include <cstring>
#include <sstream>

#include <cJSON.h>

// stb_image_write for PNG encoding (ortho /blend endpoint)
#ifdef __cplusplus
extern "C" {
#endif
int stbi_write_png_to_func(
    void (*func)(void* context, void* data, int size),
    void* context, int w, int h, int comp, const void* data, int stride);
#ifdef __cplusplus
}
#endif

// Suppress some warnings from httplib.h in MSVC
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)  // conversion, possible loss of data
#endif

#include <httplib.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include "kigstudio/agent/agent_mcp.h"
#include "ui/render_voxel_list.h"

namespace sinriv::kigstudio::agent {

// ==========================================================================
// PIMPL — hide httplib::Server from the header
// ==========================================================================

struct AgentServer::Impl {
	std::unique_ptr<httplib::Server> svr;
	std::thread listen_thread;
	std::mutex ws_mutex;
	std::vector<httplib::ws::WebSocket*> ws_clients;

	// MCP SSE handler (initialised in start() with queue reference)
	std::unique_ptr<McpHandler> mcp_handler;

	// ---- Ortho render data cache (mutex-protected) ----
	std::mutex ortho_mtx_;
	std::vector<uint8_t> render_rgba_;
	int render_w_ = 0, render_h_ = 0;
	bool render_valid_ = false;

	std::vector<uint8_t> overlay_rgba_;
	int overlay_w_ = 0, overlay_h_ = 0;
	bool overlay_valid_ = false;
	bool overlay_active_ = false;

	float overlay_off_x_ = 0.f, overlay_off_y_ = 0.f;
	float overlay_scale_x_ = 1.f, overlay_scale_y_ = 1.f;
	float overlay_blend_ = 0.5f;

	std::string ortho_state_json_;
};

// ==========================================================================
// Helpers
// ==========================================================================

std::string AgentServer::json_print(cJSON* obj) {
	if (!obj) return "{}";
	char* raw = cJSON_PrintUnformatted(obj);
	std::string s(raw ? raw : "{}");
	if (raw) cJSON_free(raw);
	return s;
}

cJSON* AgentServer::json_parse_body(const std::string& body,
                                    std::string& error_out) {
	if (body.empty()) return nullptr;  // GET requests have no body

	cJSON* obj = cJSON_Parse(body.c_str());
	if (!obj) {
		error_out = cJSON_GetErrorPtr() ? cJSON_GetErrorPtr() : "unknown JSON error";
		return nullptr;
	}
	return obj;
}

// ==========================================================================
// Constructor / Destructor
// ==========================================================================

AgentServer::AgentServer() : impl_(std::make_unique<Impl>()) {}

AgentServer::~AgentServer() { stop(); }

// ==========================================================================
// Start / Stop
// ==========================================================================

bool AgentServer::start(std::uint16_t port) {
	if (running_.load(std::memory_order_acquire)) return true;

	impl_->svr = std::make_unique<httplib::Server>();

	// Initialise MCP handler (SSE transport on same HTTP server)
	impl_->mcp_handler = std::make_unique<McpHandler>(queue_);

	register_routes();

	port_ = port;

	// httplib::Server::listen() is blocking.  Run it on a dedicated
	// thread so the main loop is unaffected.
	impl_->listen_thread = std::thread([this, port]() {
		if (!impl_->svr->listen("127.0.0.1", static_cast<int>(port))) {
			running_.store(false, std::memory_order_release);
			return;
		}
	});

	// Give the listener a moment to bind
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	bool bound = impl_->svr->is_running();
	running_.store(bound, std::memory_order_release);
	if (!bound) {
		port_ = 0;
	}
	return bound;
}

void AgentServer::stop() {
	if (!running_.load(std::memory_order_acquire)) return;

	// Mark as stopping first — new requests will get 503 immediately
	running_.store(false, std::memory_order_release);

	// 1. Close all MCP SSE sessions so their content providers unblock
	if (impl_->mcp_handler) {
		impl_->mcp_handler->shutdown();
	}

	// 2. Drain the command queue — resolves all pending promises to
	//    nullptr so any blocked tools/call or run_command futures wake up
	queue_.drain();

	// 3. Now stop the HTTP server — content providers and request
	//    handlers are no longer blocked, so this returns promptly
	impl_->svr->stop();
	if (impl_->listen_thread.joinable()) {
		impl_->listen_thread.join();
	}
}

// ==========================================================================
// Main-thread integration
// ==========================================================================

void AgentServer::process_commands(
    sinriv::ui::render::RenderVoxelList& list) {
	if (!handler_) return;
	queue_.process_commands(list, handler_);
}

void AgentServer::set_handler(AgentHandlerFn handler) {
	handler_ = std::move(handler);
}

// ==========================================================================
// WebSocket broadcast
// ==========================================================================

void AgentServer::broadcast_event(const char* type, cJSON* data) {
	if (!impl_ || !impl_->svr) return;

	cJSON* event = cJSON_CreateObject();
	cJSON_AddStringToObject(event, "type", type);
	if (data) {
		cJSON_AddItemToObject(event, "data", data);
	} else {
		cJSON_AddNullToObject(event, "data");
	}

	std::string msg = json_print(event);
	cJSON_Delete(event);

	std::lock_guard<std::mutex> lk(impl_->ws_mutex);
	// Remove dead connections
	impl_->ws_clients.erase(
	    std::remove_if(impl_->ws_clients.begin(), impl_->ws_clients.end(),
	                   [](httplib::ws::WebSocket* ws) { return !ws->is_open(); }),
	    impl_->ws_clients.end());

	for (auto* ws : impl_->ws_clients) {
		ws->send(msg);
	}
}

// ==========================================================================
// Ortho render data providers (called from UI thread)
// ==========================================================================

void AgentServer::setOrthoRenderData(const uint8_t* rgba, int w, int h) {
	if (!rgba || w <= 0 || h <= 0) return;
	std::lock_guard<std::mutex> lock(impl_->ortho_mtx_);
	size_t size = (size_t)w * h * 4;
	impl_->render_rgba_.resize(size);
	memcpy(impl_->render_rgba_.data(), rgba, size);
	impl_->render_w_ = w;
	impl_->render_h_ = h;
	impl_->render_valid_ = true;
}

void AgentServer::setOrthoOverlayData(const uint8_t* rgba, int w, int h) {
	std::lock_guard<std::mutex> lock(impl_->ortho_mtx_);
	if (!rgba || w <= 0 || h <= 0) {
		impl_->overlay_rgba_.clear();
		impl_->overlay_w_ = impl_->overlay_h_ = 0;
		impl_->overlay_valid_ = false;
		return;
	}
	size_t size = (size_t)w * h * 4;
	impl_->overlay_rgba_.resize(size);
	memcpy(impl_->overlay_rgba_.data(), rgba, size);
	impl_->overlay_w_ = w;
	impl_->overlay_h_ = h;
	impl_->overlay_valid_ = true;
}

void AgentServer::setOrthoOverlayParams(float offset_x, float offset_y,
                                         float scale_x, float scale_y,
                                         float blend_ratio) {
	std::lock_guard<std::mutex> lock(impl_->ortho_mtx_);
	impl_->overlay_off_x_ = offset_x;
	impl_->overlay_off_y_ = offset_y;
	impl_->overlay_scale_x_ = scale_x;
	impl_->overlay_scale_y_ = scale_y;
	impl_->overlay_blend_ = blend_ratio;
}

void AgentServer::setOrthoOverlayActive(bool active) {
	impl_->overlay_active_ = active;
}

void AgentServer::setOrthoState(const std::string& json) {
	std::lock_guard<std::mutex> lock(impl_->ortho_mtx_);
	impl_->ortho_state_json_ = json;
}

// ==========================================================================
// Ortho blended PNG helper (CPU-side render + overlay blend)
// ==========================================================================

bool AgentServer::sendOrthoBlendedPng(const httplib::Request& req,
                                       httplib::Response& res) {
	auto& impl = *impl_;
	std::lock_guard<std::mutex> lock(impl.ortho_mtx_);

	// Parse blend ratio from query
	float blend_ratio = 0.5f;
	if (req.has_param("ratio")) {
		try { blend_ratio = std::stof(req.get_param_value("ratio")); }
		catch (...) { blend_ratio = 0.5f; }
	}
	blend_ratio = std::max(0.0f, std::min(1.0f, blend_ratio));

	int out_w = impl.render_valid_ ? impl.render_w_ : 512;
	int out_h = impl.render_valid_ ? impl.render_h_ : 512;
	if (out_w <= 0 || out_h <= 0) {
		res.status = 503;
		res.set_content("{\"error\":\"no render data available\"}", "application/json");
		return true;
	}

	std::vector<uint8_t> out(out_w * out_h * 4, 0);

	// Fill with render data (or dark grey if no render)
	if (impl.render_valid_) {
		for (int y = 0; y < out_h; y++) {
			for (int x = 0; x < out_w; x++) {
				int src = (y * impl.render_w_ + x) * 4;
				int dst = (y * out_w + x) * 4;
				if (x < impl.render_w_ && y < impl.render_h_) {
					out[dst + 0] = impl.render_rgba_[src + 0];
					out[dst + 1] = impl.render_rgba_[src + 1];
					out[dst + 2] = impl.render_rgba_[src + 2];
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

	// Blend overlay on top
	if (impl.overlay_active_ && impl.overlay_valid_ && blend_ratio > 0.001f) {
		int ov_w = impl.overlay_w_, ov_h = impl.overlay_h_;
		float sc_x = impl.overlay_scale_x_, sc_y = impl.overlay_scale_y_;
		if (sc_x <= 0.0f) sc_x = 1.0f; if (sc_y <= 0.0f) sc_y = 1.0f;
		int placed_w = (int)(ov_w * sc_x), placed_h = (int)(ov_h * sc_y);
		int off_x = (int)impl.overlay_off_x_, off_y = (int)impl.overlay_off_y_;

		for (int dy = 0; dy < placed_h; dy++) {
			int sy = (int)(dy / sc_y);
			if (sy < 0 || sy >= ov_h) continue;
			int py = off_y + dy;
			if (py < 0 || py >= out_h) continue;
			for (int dx = 0; dx < placed_w; dx++) {
				int sx = (int)(dx / sc_x);
				if (sx < 0 || sx >= ov_w) continue;
				int px = off_x + dx;
				if (px < 0 || px >= out_w) continue;
				int oi = (sy * ov_w + sx) * 4;
				int di = (py * out_w + px) * 4;
				float ov_a = impl.overlay_rgba_[oi + 3] / 255.0f * blend_ratio;
				out[di + 0] = (uint8_t)(out[di + 0] * (1.f - ov_a) + impl.overlay_rgba_[oi + 0] * ov_a);
				out[di + 1] = (uint8_t)(out[di + 1] * (1.f - ov_a) + impl.overlay_rgba_[oi + 1] * ov_a);
				out[di + 2] = (uint8_t)(out[di + 2] * (1.f - ov_a) + impl.overlay_rgba_[oi + 2] * ov_a);
			}
		}
	}

	// Render PNG to buffer via stb_image_write callback
	struct PngBuf { std::vector<uint8_t> buf; };
	PngBuf png;
	auto writer = [](void* ctx, void* d, int sz) {
		auto* p = (PngBuf*)ctx;
		p->buf.insert(p->buf.end(), (uint8_t*)d, (uint8_t*)d + sz);
	};
	int ok = stbi_write_png_to_func(writer, &png, out_w, out_h, 4,
	                                 out.data(), out_w * 4);
	if (!ok) {
		res.status = 500;
		res.set_content("{\"error\":\"png encode failed\"}", "application/json");
	} else {
		res.set_content(
		    std::string(reinterpret_cast<const char*>(png.buf.data()), png.buf.size()),
		    "image/png");
	}
	return true;
}

// ==========================================================================
// Route registration
// ==========================================================================

void AgentServer::register_routes() {
	auto& svr = *impl_->svr;

	// Helper: extract method string from the request path + HTTP method
	auto make_method = [](const std::string& http_method,
	                      const std::string& path) -> std::string {
		return http_method + ":" + path;
	};

	// Helper: run a command and fill the HTTP response
	auto run_command = [this](const httplib::Request& req,
	                          httplib::Response& res,
	                          const std::string& method,
	                          cJSON* params) {
		// Quick bail-out if the server is shutting down
		if (!running_.load(std::memory_order_acquire)) {
			res.status = 503;
			res.set_content("{\"ok\":false,\"error\":\"server shutting down\"}",
			                "application/json");
			if (params) cJSON_Delete(params);
			return;
		}

		AgentCommand cmd;
		cmd.method = method;
		cmd.params = params;   // ownership transfers to main thread

		auto future = cmd.result_promise.get_future();
		queue_.push(std::move(cmd));

		auto status = future.wait_for(std::chrono::milliseconds(30000));
		if (status == std::future_status::timeout) {
			res.status = 504;
			res.set_content(
			    "{\"ok\":false,\"error\":\"timeout waiting for main thread\"}",
			    "application/json");
			return;
		}

		cJSON* result = future.get();
		if (!result) {
			res.status = 500;
			res.set_content(
			    "{\"ok\":false,\"error\":\"internal error\"}",
			    "application/json");
			return;
		}

		cJSON* ok_item = cJSON_GetObjectItem(result, "ok");
		bool ok = ok_item && cJSON_IsTrue(ok_item);

		cJSON* err_item = cJSON_GetObjectItem(result, "error");
		if (!ok && err_item && cJSON_IsString(err_item)) {
			// Determine HTTP status from error code if present
			cJSON* code_item = cJSON_GetObjectItem(result, "code");
			if (code_item && cJSON_IsString(code_item)) {
				const char* code = code_item->valuestring;
				if (std::strcmp(code, "NODE_NOT_FOUND") == 0)
					res.status = 404;
				else if (std::strcmp(code, "INVALID_PARAMS") == 0)
					res.status = 400;
				else
					res.status = 500;
			} else {
				res.status = 500;
			}
		}

		res.set_content(json_print(result), "application/json");
		cJSON_Delete(result);
	};

	// Helper: extract an integer path segment like /api/v1/nodes/123
	auto path_int_param = [](const httplib::Request& req,
	                         size_t segment_idx) -> int {
		// httplib's req.matches can be used, but for simplicity we
		// parse the path manually
		const std::string& path = req.path;
		size_t pos = 0;
		for (size_t i = 0; i <= segment_idx && pos < path.size(); ++i) {
			pos = path.find('/', pos);
			if (pos == std::string::npos) return -1;
			++pos;
		}
		if (pos >= path.size()) return -1;
		size_t end = path.find('/', pos);
		std::string num = (end == std::string::npos)
		                      ? path.substr(pos)
		                      : path.substr(pos, end - pos);
		return std::atoi(num.c_str());
	};

	// Helper: extract a path segment by index as a string
	auto path_str_param = [](const httplib::Request& req,
	                         size_t segment_idx) -> std::string {
		const std::string& path = req.path;
		size_t pos = 0;
		for (size_t i = 0; i <= segment_idx && pos < path.size(); ++i) {
			pos = path.find('/', pos);
			if (pos == std::string::npos) return "";
			++pos;
		}
		if (pos >= path.size()) return "";
		size_t end = path.find('/', pos);
		return (end == std::string::npos) ? path.substr(pos)
		                                  : path.substr(pos, end - pos);
	};

	// Helper: parse query params into a cJSON object
	auto parse_query = [](const httplib::Request& req) -> cJSON* {
		if (req.params.empty()) return cJSON_CreateObject();
		cJSON* obj = cJSON_CreateObject();
		for (const auto& [key, value] : req.params) {
			// Try as number first, fall back to string
			char* end = nullptr;
			double num = std::strtod(value.c_str(), &end);
			if (end && *end == '\0') {
				cJSON_AddNumberToObject(obj, key.c_str(), num);
			} else {
				cJSON_AddStringToObject(obj, key.c_str(), value.c_str());
			}
		}
		return obj;
	};

	// ================================================================
	// System endpoints
	// ================================================================

	svr.Get("/api/v1/system/status", [=](const httplib::Request& req,
	                                     httplib::Response& res) {
		run_command(req, res, "system.status", nullptr);
	});

	svr.Get("/api/v1/system/queue", [=](const httplib::Request& req,
	                                    httplib::Response& res) {
		run_command(req, res, "system.queue", nullptr);
	});

	svr.Get("/api/v1/system/log", [=](const httplib::Request& req,
	                                  httplib::Response& res) {
		cJSON* params = parse_query(req);
		run_command(req, res, "system.log", params);
	});

	// ================================================================
	// Project endpoints
	// ================================================================

	svr.Get("/api/v1/project", [=](const httplib::Request& req,
	                               httplib::Response& res) {
		run_command(req, res, "project.info", nullptr);
	});

	svr.Post("/api/v1/project/open", [=](const httplib::Request& req,
	                                     httplib::Response& res) {
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) {
			res.status = 400;
			res.set_content(
			    "{\"ok\":false,\"error\":\"" + err + "\"}",
			    "application/json");
			return;
		}
		run_command(req, res, "project.open", params);
	});

	svr.Post("/api/v1/project/save", [=](const httplib::Request& req,
	                                     httplib::Response& res) {
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) params = cJSON_CreateObject();
		run_command(req, res, "project.save", params);
	});

	svr.Post("/api/v1/project/save-as", [=](const httplib::Request& req,
	                                        httplib::Response& res) {
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) {
			res.status = 400;
			res.set_content(
			    "{\"ok\":false,\"error\":\"" + err + "\"}",
			    "application/json");
			return;
		}
		run_command(req, res, "project.saveAs", params);
	});

	svr.Post("/api/v1/project/create", [=](const httplib::Request& req,
	                                       httplib::Response& res) {
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) {
			res.status = 400;
			res.set_content(
			    "{\"ok\":false,\"error\":\"" + err + "\"}",
			    "application/json");
			return;
		}
		run_command(req, res, "project.create", params);
	});

	// ================================================================
	// Node endpoints
	// ================================================================

	svr.Get("/api/v1/nodes", [=](const httplib::Request& req,
	                             httplib::Response& res) {
		run_command(req, res, "node.list", nullptr);
	});

	// GET /api/v1/nodes/:id
	svr.Get(R"(/api/v1/nodes/(\d+))", [=](const httplib::Request& req,
	                                      httplib::Response& res) {
		int id = path_int_param(req, 3);
		cJSON* params = cJSON_CreateObject();
		cJSON_AddNumberToObject(params, "id", id);
		run_command(req, res, "node.get", params);
	});

	// POST /api/v1/nodes
	svr.Post("/api/v1/nodes", [=](const httplib::Request& req,
	                              httplib::Response& res) {
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) params = cJSON_CreateObject();
		run_command(req, res, "node.create", params);
	});

	// DELETE /api/v1/nodes/:id
	svr.Delete(R"(/api/v1/nodes/(\d+))", [=](const httplib::Request& req,
	                                         httplib::Response& res) {
		int id = path_int_param(req, 3);
		cJSON* params = cJSON_CreateObject();
		cJSON_AddNumberToObject(params, "id", id);
		run_command(req, res, "node.delete", params);
	});

	// PATCH /api/v1/nodes/:id
	svr.Patch(R"(/api/v1/nodes/(\d+))", [=](const httplib::Request& req,
	                                        httplib::Response& res) {
		int id = path_int_param(req, 3);
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) {
			res.status = 400;
			res.set_content(
			    "{\"ok\":false,\"error\":\"" + err + "\"}",
			    "application/json");
			return;
		}
		cJSON_AddNumberToObject(params, "id", id);
		run_command(req, res, "node.update", params);
	});

	// GET /api/v1/nodes/:id/children
	svr.Get(R"(/api/v1/nodes/(\d+)/children)", [=](const httplib::Request& req,
	                                               httplib::Response& res) {
		int id = path_int_param(req, 3);
		cJSON* params = cJSON_CreateObject();
		cJSON_AddNumberToObject(params, "id", id);
		run_command(req, res, "node.getChildren", params);
	});

	// GET /api/v1/nodes/:id/bounds
	svr.Get(R"(/api/v1/nodes/(\d+)/bounds)", [=](const httplib::Request& req,
	                                             httplib::Response& res) {
		int id = path_int_param(req, 3);
		cJSON* params = cJSON_CreateObject();
		cJSON_AddNumberToObject(params, "id", id);
		run_command(req, res, "node.getBounds", params);
	});

	// ================================================================
	// Mesh endpoints
	// ================================================================

	svr.Post("/api/v1/mesh/import", [=](const httplib::Request& req,
	                                    httplib::Response& res) {
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) {
			res.status = 400;
			res.set_content(
			    "{\"ok\":false,\"error\":\"" + err + "\"}",
			    "application/json");
			return;
		}
		run_command(req, res, "mesh.import", params);
	});

	svr.Post("/api/v1/mesh/export", [=](const httplib::Request& req,
	                                    httplib::Response& res) {
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) {
			res.status = 400;
			res.set_content(
			    "{\"ok\":false,\"error\":\"" + err + "\"}",
			    "application/json");
			return;
		}
		run_command(req, res, "mesh.export", params);
	});

	svr.Post("/api/v1/mesh/export-all", [=](const httplib::Request& req,
	                                        httplib::Response& res) {
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) {
			res.status = 400;
			res.set_content(
			    "{\"ok\":false,\"error\":\"" + err + "\"}",
			    "application/json");
			return;
		}
		run_command(req, res, "mesh.exportAll", params);
	});

	svr.Post("/api/v1/mesh/repair", [=](const httplib::Request& req,
	                                    httplib::Response& res) {
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) {
			res.status = 400;
			res.set_content(
			    "{\"ok\":false,\"error\":\"" + err + "\"}",
			    "application/json");
			return;
		}
		run_command(req, res, "mesh.repair", params);
	});

	svr.Post("/api/v1/mesh/subdivide", [=](const httplib::Request& req,
	                                       httplib::Response& res) {
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) {
			res.status = 400;
			res.set_content(
			    "{\"ok\":false,\"error\":\"" + err + "\"}",
			    "application/json");
			return;
		}
		run_command(req, res, "mesh.subdivide", params);
	});

	svr.Post("/api/v1/mesh/simplify", [=](const httplib::Request& req,
	                                      httplib::Response& res) {
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) {
			res.status = 400;
			res.set_content(
			    "{\"ok\":false,\"error\":\"" + err + "\"}",
			    "application/json");
			return;
		}
		run_command(req, res, "mesh.simplify", params);
	});

	svr.Post("/api/v1/mesh/boolean-union", [=](const httplib::Request& req,
	                                           httplib::Response& res) {
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) {
			res.status = 400;
			res.set_content(
			    "{\"ok\":false,\"error\":\"" + err + "\"}",
			    "application/json");
			return;
		}
		run_command(req, res, "mesh.booleanUnion", params);
	});

	svr.Get(R"(/api/v1/mesh/(\d+)/is-manifold)", [=](const httplib::Request& req,
	                                                  httplib::Response& res) {
		int id = path_int_param(req, 3);
		cJSON* params = cJSON_CreateObject();
		cJSON_AddNumberToObject(params, "node_id", id);
		run_command(req, res, "mesh.isManifold", params);
	});

	// ================================================================
	// Strand (hair) endpoints
	// ================================================================

	// GET /api/v1/nodes/:id/strands
	svr.Get(R"(/api/v1/nodes/(\d+)/strands)",
	        [=](const httplib::Request& req, httplib::Response& res) {
		        int node_id = path_int_param(req, 3);
		        cJSON* params = cJSON_CreateObject();
		        cJSON_AddNumberToObject(params, "node_id", node_id);
		        run_command(req, res, "strand.list", params);
	        });

	// GET /api/v1/nodes/:id/strands/:index
	svr.Get(R"(/api/v1/nodes/(\d+)/strands/(\d+))",
	        [=](const httplib::Request& req, httplib::Response& res) {
		        int node_id = path_int_param(req, 3);
		        int strand_index = path_int_param(req, 5);
		        cJSON* params = cJSON_CreateObject();
		        cJSON_AddNumberToObject(params, "node_id", node_id);
		        cJSON_AddNumberToObject(params, "strand_index",
		                                strand_index);
		        run_command(req, res, "strand.get", params);
	        });

	// POST /api/v1/nodes/:id/strands
	svr.Post(R"(/api/v1/nodes/(\d+)/strands)",
	         [=](const httplib::Request& req, httplib::Response& res) {
		         int node_id = path_int_param(req, 3);
		         std::string err;
		         cJSON* params = json_parse_body(req.body, err);
		         if (!params) params = cJSON_CreateObject();
		         cJSON_AddNumberToObject(params, "node_id", node_id);
		         run_command(req, res, "strand.create", params);
	         });

	// DELETE /api/v1/nodes/:id/strands/:index
	svr.Delete(R"(/api/v1/nodes/(\d+)/strands/(\d+))",
	           [=](const httplib::Request& req, httplib::Response& res) {
		           int node_id = path_int_param(req, 3);
		           int strand_index = path_int_param(req, 5);
		           cJSON* params = cJSON_CreateObject();
		           cJSON_AddNumberToObject(params, "node_id", node_id);
		           cJSON_AddNumberToObject(params, "strand_index",
		                                  strand_index);
		           run_command(req, res, "strand.delete", params);
	           });

	// PATCH /api/v1/nodes/:id/strands/:index
	svr.Patch(R"(/api/v1/nodes/(\d+)/strands/(\d+))",
	          [=](const httplib::Request& req, httplib::Response& res) {
		          int node_id = path_int_param(req, 3);
		          int strand_index = path_int_param(req, 5);
		          std::string err;
		          cJSON* params = json_parse_body(req.body, err);
		          if (!params) {
			          res.status = 400;
			          res.set_content(
			              "{\"ok\":false,\"error\":\"" + err + "\"}",
			              "application/json");
			          return;
		          }
		          cJSON_AddNumberToObject(params, "node_id", node_id);
		          cJSON_AddNumberToObject(params, "strand_index",
		                                 strand_index);
		          run_command(req, res, "strand.update", params);
	          });

	// POST /api/v1/nodes/:id/strands/:index/move
	svr.Post(R"(/api/v1/nodes/(\d+)/strands/(\d+)/move)",
	         [=](const httplib::Request& req, httplib::Response& res) {
		         int node_id = path_int_param(req, 3);
		         int strand_index = path_int_param(req, 5);
		         std::string err;
		         cJSON* params = json_parse_body(req.body, err);
		         if (!params) {
			         res.status = 400;
			         res.set_content(
			             "{\"ok\":false,\"error\":\"" + err + "\"}",
			             "application/json");
			         return;
		         }
		         cJSON_AddNumberToObject(params, "node_id", node_id);
		         cJSON_AddNumberToObject(params, "strand_index",
		                                strand_index);
		         run_command(req, res, "strand.move", params);
	         });

	// PUT /api/v1/nodes/:id/strands/center-point
	svr.Put(R"(/api/v1/nodes/(\d+)/strands/center-point)",
	        [=](const httplib::Request& req, httplib::Response& res) {
		        int node_id = path_int_param(req, 3);
		        std::string err;
		        cJSON* params = json_parse_body(req.body, err);
		        if (!params) {
			        res.status = 400;
			        res.set_content(
			            "{\"ok\":false,\"error\":\"" + err + "\"}",
			            "application/json");
			        return;
		        }
		        cJSON_AddNumberToObject(params, "node_id", node_id);
		        run_command(req, res, "strand.setCenterPoint", params);
	        });

	// PUT /api/v1/nodes/:id/addon-options
	svr.Put(R"(/api/v1/nodes/(\d+)/addon-options)",
	        [=](const httplib::Request& req, httplib::Response& res) {
		        int node_id = path_int_param(req, 3);
		        std::string err;
		        cJSON* params = json_parse_body(req.body, err);
		        if (!params) {
			        res.status = 400;
			        res.set_content(
			            "{\"ok\":false,\"error\":\"" + err + "\"}",
			            "application/json");
			        return;
		        }
		        cJSON_AddNumberToObject(params, "node_id", node_id);
		        run_command(req, res, "strand.setAddonOptions", params);
	        });

	// ================================================================
	// Semantic-coordinate endpoints (head coordinate system)
	// ================================================================

	// POST /api/v1/nodes/:id/hair/angle-config
	// Sets per-(x,y) ray angles and builds BVH tree from base model.
	svr.Post(R"(/api/v1/nodes/(\d+)/hair/angle-config)",
	         [=](const httplib::Request& req, httplib::Response& res) {
		         int node_id = path_int_param(req, 3);
		         std::string err;
		         cJSON* params = json_parse_body(req.body, err);
		         if (!params) {
			         res.status = 400;
			         res.set_content(
			             "{\"ok\":false,\"error\":\"" + err + "\"}",
			             "application/json");
			         return;
		         }
		         cJSON_AddNumberToObject(params, "node_id", node_id);
		         run_command(req, res, "strand.setAngleConfig", params);
	         });

	// POST /api/v1/nodes/:id/strands/:index/guide-points/semantic
	// Adds a guide point by casting a ray from the configured angle.
	svr.Post(R"(/api/v1/nodes/(\d+)/strands/(\d+)/guide-points/semantic)",
	         [=](const httplib::Request& req, httplib::Response& res) {
		         int node_id = path_int_param(req, 3);
		         int strand_index = path_int_param(req, 5);
		         std::string err;
		         cJSON* params = json_parse_body(req.body, err);
		         if (!params) {
			         res.status = 400;
			         res.set_content(
			             "{\"ok\":false,\"error\":\"" + err + "\"}",
			             "application/json");
			         return;
		         }
		         cJSON_AddNumberToObject(params, "node_id", node_id);
		         cJSON_AddNumberToObject(params, "strand_index",
		                                strand_index);
		         run_command(req, res, "strand.addSemanticGuidePoint",
		                     params);
	         });

	// POST /api/v1/nodes/:id/strands/:index/width-points/semantic
	// Adds a width vector by casting a ray and computing curve projection.
	svr.Post(R"(/api/v1/nodes/(\d+)/strands/(\d+)/width-points/semantic)",
	         [=](const httplib::Request& req, httplib::Response& res) {
		         int node_id = path_int_param(req, 3);
		         int strand_index = path_int_param(req, 5);
		         std::string err;
		         cJSON* params = json_parse_body(req.body, err);
		         if (!params) {
			         res.status = 400;
			         res.set_content(
			             "{\"ok\":false,\"error\":\"" + err + "\"}",
			             "application/json");
			         return;
		         }
		         cJSON_AddNumberToObject(params, "node_id", node_id);
		         cJSON_AddNumberToObject(params, "strand_index",
		                                strand_index);
		         run_command(req, res, "strand.addSemanticWidthPoint",
		                     params);
	         });

	// --- UUID-based strand routes (alternative to :index) ---

	// GET /api/v1/nodes/:id/strands/by-uuid/:uuid
	svr.Get(R"(/api/v1/nodes/(\d+)/strands/by-uuid/([a-f0-9]+))",
	        [=](const httplib::Request& req, httplib::Response& res) {
		        int node_id = path_int_param(req, 3);
		        std::string strand_uuid = path_str_param(req, 6);
		        cJSON* params = cJSON_CreateObject();
		        cJSON_AddNumberToObject(params, "node_id", node_id);
		        cJSON_AddStringToObject(params, "strand_uuid",
		                                strand_uuid.c_str());
		        run_command(req, res, "strand.get", params);
	        });

	// DELETE /api/v1/nodes/:id/strands/by-uuid/:uuid
	svr.Delete(R"(/api/v1/nodes/(\d+)/strands/by-uuid/([a-f0-9]+))",
	           [=](const httplib::Request& req, httplib::Response& res) {
		           int node_id = path_int_param(req, 3);
		           std::string strand_uuid = path_str_param(req, 6);
		           cJSON* params = cJSON_CreateObject();
		           cJSON_AddNumberToObject(params, "node_id", node_id);
		           cJSON_AddStringToObject(params, "strand_uuid",
		                                   strand_uuid.c_str());
		           run_command(req, res, "strand.delete", params);
	           });

	// PATCH /api/v1/nodes/:id/strands/by-uuid/:uuid
	svr.Patch(R"(/api/v1/nodes/(\d+)/strands/by-uuid/([a-f0-9]+))",
	          [=](const httplib::Request& req, httplib::Response& res) {
		          int node_id = path_int_param(req, 3);
		          std::string strand_uuid = path_str_param(req, 6);
		          std::string err;
		          cJSON* params = json_parse_body(req.body, err);
		          if (!params) {
			          res.status = 400;
			          res.set_content(
			              "{\"ok\":false,\"error\":\"" + err + "\"}",
			              "application/json");
			          return;
		          }
		          cJSON_AddNumberToObject(params, "node_id", node_id);
		          cJSON_AddStringToObject(params, "strand_uuid",
		                                  strand_uuid.c_str());
		          run_command(req, res, "strand.update", params);
	          });

	// POST /api/v1/nodes/:id/strands/by-uuid/:uuid/move
	svr.Post(R"(/api/v1/nodes/(\d+)/strands/by-uuid/([a-f0-9]+)/move)",
	         [=](const httplib::Request& req, httplib::Response& res) {
		         int node_id = path_int_param(req, 3);
		         std::string strand_uuid = path_str_param(req, 6);
		         std::string err;
		         cJSON* params = json_parse_body(req.body, err);
		         if (!params) {
			         res.status = 400;
			         res.set_content(
			             "{\"ok\":false,\"error\":\"" + err + "\"}",
			             "application/json");
			         return;
		         }
		         cJSON_AddNumberToObject(params, "node_id", node_id);
		         cJSON_AddStringToObject(params, "strand_uuid",
		                                 strand_uuid.c_str());
		         run_command(req, res, "strand.move", params);
	         });

	// POST /api/v1/nodes/:id/strands/by-uuid/:uuid/rename
	svr.Post(R"(/api/v1/nodes/(\d+)/strands/by-uuid/([a-f0-9]+)/rename)",
	         [=](const httplib::Request& req, httplib::Response& res) {
		         int node_id = path_int_param(req, 3);
		         std::string strand_uuid = path_str_param(req, 6);
		         std::string err;
		         cJSON* params = json_parse_body(req.body, err);
		         if (!params) {
			         res.status = 400;
			         res.set_content(
			             "{\"ok\":false,\"error\":\"" + err + "\"}",
			             "application/json");
			         return;
		         }
		         cJSON_AddNumberToObject(params, "node_id", node_id);
		         cJSON_AddStringToObject(params, "strand_uuid",
		                                 strand_uuid.c_str());
		         run_command(req, res, "strand.rename", params);
	         });

	// POST /api/v1/nodes/:id/strands/by-uuid/:uuid/guide-points/semantic
	svr.Post(R"(/api/v1/nodes/(\d+)/strands/by-uuid/([a-f0-9]+)/guide-points/semantic)",
	         [=](const httplib::Request& req, httplib::Response& res) {
		         int node_id = path_int_param(req, 3);
		         std::string strand_uuid = path_str_param(req, 6);
		         std::string err;
		         cJSON* params = json_parse_body(req.body, err);
		         if (!params) {
			         res.status = 400;
			         res.set_content(
			             "{\"ok\":false,\"error\":\"" + err + "\"}",
			             "application/json");
			         return;
		         }
		         cJSON_AddNumberToObject(params, "node_id", node_id);
		         cJSON_AddStringToObject(params, "strand_uuid",
		                                 strand_uuid.c_str());
		         run_command(req, res, "strand.addSemanticGuidePoint",
		                     params);
	         });

	// POST /api/v1/nodes/:id/strands/by-uuid/:uuid/width-points/semantic
	svr.Post(R"(/api/v1/nodes/(\d+)/strands/by-uuid/([a-f0-9]+)/width-points/semantic)",
	         [=](const httplib::Request& req, httplib::Response& res) {
		         int node_id = path_int_param(req, 3);
		         std::string strand_uuid = path_str_param(req, 6);
		         std::string err;
		         cJSON* params = json_parse_body(req.body, err);
		         if (!params) {
			         res.status = 400;
			         res.set_content(
			             "{\"ok\":false,\"error\":\"" + err + "\"}",
			             "application/json");
			         return;
		         }
		         cJSON_AddNumberToObject(params, "node_id", node_id);
		         cJSON_AddStringToObject(params, "strand_uuid",
		                                 strand_uuid.c_str());
		         run_command(req, res, "strand.addSemanticWidthPoint",
		                     params);
	         });

	// POST /api/v1/nodes/:id/hair/spindle
	// Applies hairline spindle: computes hairline-plane intersection
	// for each strand and generates tapered width points.
	svr.Post(R"(/api/v1/nodes/(\d+)/hair/spindle)",
	         [=](const httplib::Request& req, httplib::Response& res) {
		         int node_id = path_int_param(req, 3);
		         std::string err;
		         cJSON* params = json_parse_body(req.body, err);
		         if (!params) params = cJSON_CreateObject();
		         cJSON_AddNumberToObject(params, "node_id", node_id);
		         run_command(req, res, "strand.applyHairlineSpindle",
		                     params);
	         });

	// ================================================================
	// WebSocket endpoint for progress events
	// ================================================================

	svr.WebSocket("/api/v1/ws",
	    [this](const httplib::Request& /*req*/, httplib::ws::WebSocket& ws) {
	        // Client connected
	        {
	            std::lock_guard<std::mutex> lk(impl_->ws_mutex);
	            impl_->ws_clients.push_back(&ws);
	        }
	        // Keep connection alive; httplib calls this for each message.
	        // On close, the client is removed on next broadcast_event.
	        while (ws.is_open()) {
	            std::this_thread::sleep_for(std::chrono::seconds(1));
	        }
	    });

	// ================================================================
	// POST /api/v1/system/wait-idle
	// ================================================================

	svr.Post("/api/v1/system/wait-idle", [=](const httplib::Request& req,
	                                         httplib::Response& res) {
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) params = cJSON_CreateObject();
		run_command(req, res, "system.waitIdle", params);
	});

	// ================================================================
	// POST /api/v1/system/toast
	// ================================================================

	svr.Post("/api/v1/system/toast", [=](const httplib::Request& req,
	                                     httplib::Response& res) {
		std::string err;
		cJSON* params = json_parse_body(req.body, err);
		if (!params) {
			res.status = 400;
			res.set_content(
			    "{\"ok\":false,\"error\":\"" + err + "\"}",
			    "application/json");
			return;
		}
		run_command(req, res, "system.toast", params);
	});

	// ================================================================
	// MCP (Model Context Protocol) SSE transport
	// ================================================================

	// GET /mcp/sse — Open Server-Sent Events stream.
	// The first event carries the endpoint URL for POSTing messages.
	svr.Get("/mcp/sse", [this](const httplib::Request& req,
	                           httplib::Response& res) {
		if (!impl_->mcp_handler) {
			res.status = 503;
			res.set_content("MCP not available", "text/plain");
			return;
		}
		impl_->mcp_handler->handle_sse_connect(req, res);
	});

	// POST /mcp/messages — Client sends JSON-RPC 2.0 requests here.
	// Session is identified by ?sessionId=xxx query parameter.
	svr.Post("/mcp/messages", [this](const httplib::Request& req,
	                                 httplib::Response& res) {
		if (!impl_->mcp_handler) {
			res.status = 503;
			res.set_content("MCP not available", "text/plain");
			return;
		}
		impl_->mcp_handler->handle_message(req, res);
	});

		// ================================================================
		// Ortho projection endpoints (render, overlay, blend, state)
		// ================================================================

		svr.Get("/api/v1/ortho/ping", [this](const httplib::Request& /*req*/,
		                                     httplib::Response& res) {
			res.set_content("{\"ok\":true,\"service\":\"kigstudio-ortho-api\"}",
			                "application/json");
		});

		svr.Get("/api/v1/ortho/state", [this](const httplib::Request& /*req*/,
		                                      httplib::Response& res) {
			std::lock_guard<std::mutex> lock(impl_->ortho_mtx_);
			res.set_content(
			    impl_->ortho_state_json_.empty() ? "{}" : impl_->ortho_state_json_,
			    "application/json");
		});

		svr.Get("/api/v1/ortho/render", [this](const httplib::Request& /*req*/,
		                                       httplib::Response& res) {
			std::lock_guard<std::mutex> lock(impl_->ortho_mtx_);
			if (!impl_->render_valid_) {
				res.status = 503;
				res.set_content("{\"error\":\"no render data available\"}",
				                "application/json");
				return;
			}
			// Encode to PNG via callback
			struct Buf { std::vector<uint8_t> d; };
			Buf buf;
			auto w = [](void* c, void* d, int s) {
				((Buf*)c)->d.insert(((Buf*)c)->d.end(), (uint8_t*)d, (uint8_t*)d + s);
			};
			int ok = stbi_write_png_to_func(w, &buf,
			    impl_->render_w_, impl_->render_h_, 4,
			    impl_->render_rgba_.data(), impl_->render_w_ * 4);
			if (!ok) {
				res.status = 500;
				res.set_content("{\"error\":\"png encode failed\"}", "application/json");
			} else {
				res.set_content(
				    std::string((const char*)buf.d.data(), buf.d.size()),
				    "image/png");
			}
		});

		svr.Get("/api/v1/ortho/overlay", [this](const httplib::Request& /*req*/,
		                                        httplib::Response& res) {
			std::lock_guard<std::mutex> lock(impl_->ortho_mtx_);
			if (!impl_->overlay_valid_) {
				res.status = 404;
				res.set_content("{\"error\":\"no overlay loaded\"}",
				                "application/json");
				return;
			}
			struct Buf { std::vector<uint8_t> d; };
			Buf buf;
			auto w = [](void* c, void* d, int s) {
				((Buf*)c)->d.insert(((Buf*)c)->d.end(), (uint8_t*)d, (uint8_t*)d + s);
			};
			int ok = stbi_write_png_to_func(w, &buf,
			    impl_->overlay_w_, impl_->overlay_h_, 4,
			    impl_->overlay_rgba_.data(), impl_->overlay_w_ * 4);
			if (!ok) {
				res.status = 500;
				res.set_content("{\"error\":\"png encode failed\"}", "application/json");
			} else {
				res.set_content(
				    std::string((const char*)buf.d.data(), buf.d.size()),
				    "image/png");
			}
		});

		svr.Get("/api/v1/ortho/blend", [this](const httplib::Request& req,
		                                      httplib::Response& res) {
			sendOrthoBlendedPng(req, res);
		});

		// POST /api/v1/ortho/strand — create/update strand from 2D pixel coords
		svr.Post("/api/v1/ortho/strand", [=](const httplib::Request& req,
		                                     httplib::Response& res) {
			std::string err;
			cJSON* params = json_parse_body(req.body, err);
			if (!params) {
				res.status = 400;
				res.set_content(
				    "{\"ok\":false,\"error\":\"" + err + "\"}",
				    "application/json");
				return;
			}
			run_command(req, res, "strand.create2d", params);
		});
	}

}  // namespace sinriv::kigstudio::agent
