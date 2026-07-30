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

	// ---- accessors ----

	AgentCommandQueue& command_queue() { return queue_; }
	const AgentCommandQueue& command_queue() const { return queue_; }

private:
	void register_routes();
	static std::string json_print(cJSON* obj);
	cJSON* json_parse_body(const std::string& body, std::string& error_out);

	AgentCommandQueue queue_;
	AgentHandlerFn handler_;

	std::atomic<bool> running_{false};
	std::uint16_t port_ = 0;

	// PIMPL — hide httplib types from header
	struct Impl;
	std::unique_ptr<Impl> impl_;
};

}  // namespace sinriv::kigstudio::agent
