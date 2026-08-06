#pragma once

/**
 * Embedded HTTP server (cpp-httplib) that exposes the KigStudio API to
 * AI agents while the GUI is running.  See docs/ai-agent-api.md.
 */

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <cJSON.h>

#include "kigstudio/agent/agent_queue.h"

// Forward declarations
namespace sinriv::ui::render {
class RenderVoxelList;
}
namespace httplib {
struct Request;
struct Response;
}

namespace sinriv::kigstudio::agent {

/// Callback type for the main-thread command processor.
/// Receives (method, params, list) and returns a cJSON* result.
/// Takes ownership of `params`; return value ownership transfers to caller.
using AgentHandlerFn = std::function<cJSON*(
	const std::string& method,
	cJSON* params,
	sinriv::ui::render::RenderVoxelList& list)>;

/// Manages the embedded HTTP server lifecycle and the command queue.
///
/// Usage from ui_main():
///   AgentServer agent_server;
///   agent_server.start(18920);
///   // ... each frame ...
///   agent_server.process_commands(render_items);
///   // ... on shutdown ...
///   agent_server.stop();
class AgentServer {
public:
	AgentServer();
	~AgentServer();

	// ---- lifecycle ----

	/// Start the HTTP server on `port` (127.0.0.1 only).
	/// Non-blocking — the server runs on its own thread pool.
	/// Returns false if the port is already in use.
	bool start(std::uint16_t port = 18920);

	/// Stop the server and drain pending commands.
	void stop();

	/// True once start() has succeeded.
	bool is_running() const { return running_.load(std::memory_order_acquire); }

	/// Port the server is listening on (0 if not started).
	std::uint16_t port() const { return port_; }

	// ---- main-thread integration ----

	/// Called once per frame from the main render loop.
	/// Dispatches all pending commands to `handler` on the calling thread.
	void process_commands(sinriv::ui::render::RenderVoxelList& list);

	/// Set the handler that processes agent commands on the main thread.
	/// Must be set before start() or before the first process_commands().
	void set_handler(AgentHandlerFn handler);

	/// Push an event notification to all connected WebSocket clients.
	/// Thread-safe — can be called from any thread.
	void broadcast_event(const char* type, cJSON* data);

	// ---- Ortho render data providers (called from UI thread) ----
	/// Set the latest ortho-projection render as RGBA pixel data.
	void setOrthoRenderData(const uint8_t* rgba, int w, int h);
	/// Set the overlay/reference image as RGBA pixel data.
	void setOrthoOverlayData(const uint8_t* rgba, int w, int h);
	/// Update overlay placement parameters.
	void setOrthoOverlayParams(float offset_x, float offset_y,
	                           float scale_x, float scale_y,
	                           float blend_ratio);
	/// Enable/disable overlay blending.
	void setOrthoOverlayActive(bool active);
	/// Set the current camera/projection state as JSON string.
	void setOrthoState(const std::string& json);

	/// Callback to draw guide-curve overlays on an RGBA pixel buffer.
	/// Called from the HTTP handler thread when serving /ortho/render.
	/// Parameters: (rgba buffer, width, height, color_code_flag,
	///               line_thickness, font_size).
	using GuideCurveDrawFn = std::function<void(
	    std::vector<uint8_t>& rgba, int w, int h, bool color_code,
	    int line_thickness, float font_size)>;

	/// Set the guide-curve draw callback and the export flags.
	/// Called once at startup with the callback; use setGuideCurveFlags()
	/// afterwards to update just the flags without recreating the callback.
	void setGuideCurveDrawState(bool export_curves, bool color_code,
	                            GuideCurveDrawFn draw_fn);

	/// Update only the export flags without touching the draw callback.
	void setGuideCurveFlags(bool export_curves, bool color_code);

	// ---- accessors ----

	AgentCommandQueue& command_queue() { return queue_; }
	const AgentCommandQueue& command_queue() const { return queue_; }

private:
	void register_routes();
	static std::string json_print(cJSON* obj);
	cJSON* json_parse_body(const std::string& body, std::string& error_out);

	// Ortho blend helper (implementation in .cpp; fwd-declared httplib types above)
	bool sendOrthoBlendedPng(const httplib::Request& req,
	                         httplib::Response& res);

	AgentCommandQueue queue_;
	AgentHandlerFn handler_;

	std::atomic<bool> running_{false};
	std::uint16_t port_ = 0;

	// PIMPL — hide httplib types from header
	struct Impl;
	std::unique_ptr<Impl> impl_;
};

}  // namespace sinriv::kigstudio::agent
