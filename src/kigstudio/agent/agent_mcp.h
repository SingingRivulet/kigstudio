#pragma once

/**
 * MCP (Model Context Protocol) handler — SSE transport over the existing
 * HTTP server.  Compliant with MCP spec 2024-11-05.
 *
 * Endpoints (registered on AgentServer's httplib::Server):
 *   GET  /mcp/sse             – open SSE stream, returns session endpoint
 *   POST /mcp/messages        – client→server JSON-RPC 2.0 messages
 *
 * The handler reuses the existing AgentCommandQueue so that all state
 * mutations happen on the main thread, just like REST API calls.
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

#include <cJSON.h>
#include <httplib.h>

#include "kigstudio/agent/agent_queue.h"

namespace sinriv::kigstudio::agent {

// Forward declaration – AgentServer is in agent_server.h
class AgentServer;

// ==========================================================================
// MCP SSE session
// ==========================================================================

struct McpSession {
	std::string id;
	std::mutex mtx;
	std::condition_variable cv;
	std::deque<std::string> outbox;  // pending SSE frames
	bool closed = false;

	/// Push a complete SSE frame (including "data:" prefix and "\n\n" suffix).
	void push_sse(const std::string& sse_text) {
		{
			std::lock_guard<std::mutex> lk(mtx);
			if (closed) return;
			outbox.push_back(sse_text);
		}
		cv.notify_one();
	}

	/// Wait for and pop the next SSE frame. Blocks until data or close.
	/// Returns empty string when session is closed.
	std::string pop_sse() {
		std::unique_lock<std::mutex> lk(mtx);
		cv.wait(lk, [this] { return !outbox.empty() || closed; });
		if (closed && outbox.empty()) return {};
		auto s = std::move(outbox.front());
		outbox.pop_front();
		return s;
	}
};

// ==========================================================================
// MCP Handler
// ==========================================================================

class McpHandler {
public:
	McpHandler(AgentCommandQueue& queue)
	    : queue_(queue) {}

	// ---- SSE endpoint (GET /mcp/sse) ----

	void handle_sse_connect(const httplib::Request& /*req*/,
	                        httplib::Response& res) {
		// Generate a random session id (8 hex chars)
		auto sess = std::make_shared<McpSession>();
		sess->id = gen_session_id();

		{
			std::lock_guard<std::mutex> lk(sessions_mtx_);
			sessions_[sess->id] = sess;
		}

		// Tell the client where to POST messages
		std::string endpoint_path =
		    "/mcp/messages?sessionId=" + sess->id;

		// SSE headers
		res.set_header("Content-Type", "text/event-stream");
		res.set_header("Cache-Control", "no-cache");
		res.set_header("Connection", "keep-alive");
		res.set_header("X-Accel-Buffering", "no");

		res.set_chunked_content_provider(
		    "text/event-stream",
		    // Content provider — called repeatedly
		    [sess](size_t /*offset*/, httplib::DataSink& sink) -> bool {
			    // First chunk: send endpoint event
			    std::string endpoint_path_local =
			        "/mcp/messages?sessionId=" + sess->id;
			    std::string endpoint_event =
			        "event: endpoint\ndata: " + endpoint_path_local +
			        "\n\n";
			    sink.write(endpoint_event.data(),
			               endpoint_event.size());

			    // Subsequent chunks: wait for outbox messages
			    while (true) {
				    std::string sse = sess->pop_sse();
				    if (sse.empty()) {
					    // Session closed
					    sink.done();
					    return true;
				    }
				    // Send as SSE data event
				    std::string frame =
				        "event: message\ndata: " + sse + "\n\n";
				    if (!sink.write(frame.data(), frame.size())) {
					    // Sink closed by client disconnect
					    std::lock_guard<std::mutex> lk(
					        sess->mtx);
					    sess->closed = true;
					    sess->cv.notify_all();
					    return false;
				    }
			    }
		    },
		    // Resource releaser — called when SSE connection ends
		    [this, sess](bool /*success*/) {
			    std::lock_guard<std::mutex> lk(sessions_mtx_);
			    sessions_.erase(sess->id);
		    });
	}

	// ---- Message endpoint (POST /mcp/messages) ----

	void handle_message(const httplib::Request& req,
	                    httplib::Response& res) {
		// Extract session id from query params
		auto it = req.params.find("sessionId");
		std::string sid =
		    (it != req.params.end()) ? it->second : "";

		std::shared_ptr<McpSession> sess;
		{
			std::lock_guard<std::mutex> lk(sessions_mtx_);
			auto si = sessions_.find(sid);
			if (si == sessions_.end()) {
				res.status = 400;
				res.set_content(
				    "{\"error\":\"invalid session\"}",
				    "application/json");
				return;
			}
			sess = si->second;
		}

		// Parse JSON-RPC body
		cJSON* body = cJSON_Parse(req.body.c_str());
		if (!body) {
			res.status = 400;
			res.set_content("{\"error\":\"invalid JSON\"}",
			                "application/json");
			return;
		}

		cJSON* id_item = cJSON_GetObjectItem(body, "id");
		cJSON* method_item = cJSON_GetObjectItem(body, "method");
		const char* method =
		    method_item && cJSON_IsString(method_item)
		        ? method_item->valuestring
		        : nullptr;

		if (!method) {
			cJSON_Delete(body);
			res.status = 400;
			res.set_content("{\"error\":\"missing method\"}",
			                "application/json");
			return;
		}

		std::string response_json;

		if (std::strcmp(method, "initialize") == 0) {
			response_json = handle_initialize(id_item, body);
		} else if (std::strcmp(method,
		                       "notifications/initialized") == 0) {
			// Notification — no response needed
			cJSON_Delete(body);
			res.status = 202;
			return;
		} else if (std::strcmp(method, "ping") == 0) {
			response_json = handle_ping(id_item);
		} else if (std::strcmp(method, "tools/list") == 0) {
			response_json = handle_tools_list(id_item);
		} else if (std::strcmp(method, "tools/call") == 0) {
			response_json =
			    handle_tools_call(id_item, body, sess);
		} else {
			response_json = jsonrpc_error(
			    id_item, -32601, "Method not found",
			    std::string("unknown method: ") + method);
		}

		cJSON_Delete(body);

		// Send response via SSE if present
		if (!response_json.empty()) {
			sess->push_sse(response_json);
		}

		res.status = 202;
		res.set_content("{}", "application/json");
	}

	// ---- Periodic maintenance (call once per frame) ----
	void cleanup() {
		// Nothing to do — sessions auto-clean on SSE disconnect
	}

private:
	AgentCommandQueue& queue_;
	std::mutex sessions_mtx_;
	std::unordered_map<std::string, std::shared_ptr<McpSession>>
	    sessions_;

	// ---- helpers ----

	std::string gen_session_id() {
		thread_local std::mt19937 rng(
		    static_cast<unsigned>(
		        std::chrono::steady_clock::now()
		            .time_since_epoch()
		            .count()) ^
		    std::hash<std::thread::id>{}(std::this_thread::get_id()));
		std::uniform_int_distribution<int> dist(0, 15);
		const char hex[] = "0123456789abcdef";
		std::string id;
		for (int i = 0; i < 16; ++i)
			id += hex[dist(rng)];
		return id;
	}

	static std::string jsonrpc_response(const cJSON* id,
	                                    cJSON* result) {
		cJSON* r = cJSON_CreateObject();
		cJSON_AddStringToObject(r, "jsonrpc", "2.0");
		if (id) {
			if (cJSON_IsString(id))
				cJSON_AddStringToObject(r, "id", id->valuestring);
			else if (cJSON_IsNumber(id))
				cJSON_AddNumberToObject(r, "id", id->valuedouble);
		}
		cJSON_AddItemToObject(r, "result", result);
		char* raw = cJSON_PrintUnformatted(r);
		std::string s(raw);
		cJSON_free(raw);
		cJSON_Delete(r);
		return s;
	}

	static std::string jsonrpc_error(const cJSON* id,
	                                 int code,
	                                 const std::string& message,
	                                 const std::string& data = "") {
		cJSON* r = cJSON_CreateObject();
		cJSON_AddStringToObject(r, "jsonrpc", "2.0");
		if (id) {
			if (cJSON_IsString(id))
				cJSON_AddStringToObject(r, "id", id->valuestring);
			else if (cJSON_IsNumber(id))
				cJSON_AddNumberToObject(r, "id", id->valuedouble);
		}
		cJSON* err = cJSON_CreateObject();
		cJSON_AddNumberToObject(err, "code", code);
		cJSON_AddStringToObject(err, "message", message.c_str());
		if (!data.empty())
			cJSON_AddStringToObject(err, "data", data.c_str());
		cJSON_AddItemToObject(r, "error", err);
		char* raw = cJSON_PrintUnformatted(r);
		std::string s(raw);
		cJSON_free(raw);
		cJSON_Delete(r);
		return s;
	}

	static cJSON* build_text_result(const std::string& text) {
		cJSON* result = cJSON_CreateObject();
		cJSON* content = cJSON_CreateArray();
		cJSON* item = cJSON_CreateObject();
		cJSON_AddStringToObject(item, "type", "text");
		cJSON_AddStringToObject(item, "text", text.c_str());
		cJSON_AddItemToArray(content, item);
		cJSON_AddItemToObject(result, "content", content);
		return result;
	}

	// ---- MCP protocol handlers ----

	std::string handle_initialize(const cJSON* id, cJSON* body) {
		cJSON* result = cJSON_CreateObject();
		cJSON_AddStringToObject(result, "protocolVersion",
		                        "2024-11-05");

		cJSON* caps = cJSON_CreateObject();
		cJSON* tools_cap = cJSON_CreateObject();
		cJSON_AddItemToObject(caps, "tools", tools_cap);
		cJSON_AddItemToObject(result, "capabilities", caps);

		cJSON* info = cJSON_CreateObject();
		cJSON_AddStringToObject(info, "name", "KigStudio");
		cJSON_AddStringToObject(info, "version", "1.0.0");
		cJSON_AddItemToObject(result, "serverInfo", info);

		return jsonrpc_response(id, result);
	}

	std::string handle_ping(const cJSON* id) {
		cJSON* result = cJSON_CreateObject();
		return jsonrpc_response(id, result);
	}

	// ---- tools/list ----

	std::string handle_tools_list(const cJSON* id) {
		cJSON* result = cJSON_CreateObject();
		cJSON* tools = build_tool_list();
		cJSON_AddItemToObject(result, "tools", tools);
		return jsonrpc_response(id, result);
	}

	/// Build the complete MCP tool list (cJSON array) from all
	/// registered dispatch methods.
	static cJSON* build_tool_list() {
		cJSON* arr = cJSON_CreateArray();

		// --- helpers for building param schemas ---
		auto add_tool = [&](const char* name,
		                    const char* desc,
		                    const char* dispatch_method,
		                    cJSON* input_schema) {
			cJSON* t = cJSON_CreateObject();
			cJSON_AddStringToObject(t, "name", name);
			cJSON_AddStringToObject(t, "description", desc);
			cJSON_AddItemToObject(t, "inputSchema", input_schema);

			// Store the dispatch method name in a private annotation
			cJSON_AddStringToObject(t, "_dispatch_method",
			                        dispatch_method);
			cJSON_AddItemToArray(arr, t);
		};

		auto schema_obj = [](cJSON* props, cJSON* required,
		                     const char* title = nullptr) {
			cJSON* s = cJSON_CreateObject();
			cJSON_AddStringToObject(s, "type", "object");
			cJSON_AddItemToObject(s, "properties", props);
			if (required)
				cJSON_AddItemToObject(s, "required", required);
			if (title)
				cJSON_AddStringToObject(s, "title", title);
			return s;
		};

		auto prop_str =
		    [](const char* name, const char* desc,
		       const char* def = nullptr,
		       const char* enum_csv = nullptr) -> cJSON* {
			cJSON* p = cJSON_CreateObject();
			cJSON_AddStringToObject(p, "type", "string");
			cJSON_AddStringToObject(p, "description", desc);
			if (def) cJSON_AddStringToObject(p, "default", def);
			if (enum_csv) {
				cJSON* e = cJSON_CreateArray();
				std::istringstream ss(enum_csv);
				std::string tok;
				while (std::getline(ss, tok, ','))
					cJSON_AddItemToArray(
					    e, cJSON_CreateString(tok.c_str()));
				cJSON_AddItemToObject(p, "enum", e);
			}
			return p;
		};

		auto prop_num =
		    [](const char* name, const char* desc,
		       double def = 0) -> cJSON* {
			cJSON* p = cJSON_CreateObject();
			cJSON_AddStringToObject(p, "type", "number");
			cJSON_AddStringToObject(p, "description", desc);
			if (def != 0)
				cJSON_AddNumberToObject(p, "default", def);
			return p;
		};

		auto prop_bool =
		    [](const char* name, const char* desc,
		       bool def = false) -> cJSON* {
			cJSON* p = cJSON_CreateObject();
			cJSON_AddStringToObject(p, "type", "boolean");
			cJSON_AddStringToObject(p, "description", desc);
			cJSON_AddBoolToObject(p, "default",
			                      def ? 1 : 0);
			return p;
		};

		auto prop_int = [](const char* name, const char* desc,
		                   int def = 0) -> cJSON* {
			cJSON* p = cJSON_CreateObject();
			cJSON_AddStringToObject(p, "type", "integer");
			cJSON_AddStringToObject(p, "description", desc);
			cJSON_AddNumberToObject(p, "default",
			                        static_cast<double>(def));
			return p;
		};

		auto prop_arr =
		    [](const char* name, const char* desc,
		       const char* item_type = "number") -> cJSON* {
			cJSON* p = cJSON_CreateObject();
			cJSON_AddStringToObject(p, "type", "array");
			cJSON_AddStringToObject(p, "description", desc);
			cJSON* items = cJSON_CreateObject();
			cJSON_AddStringToObject(items, "type", item_type);
			cJSON_AddItemToObject(p, "items", items);
			return p;
		};

		auto prop_obj =
		    [](const char* name, const char* desc) -> cJSON* {
			cJSON* p = cJSON_CreateObject();
			cJSON_AddStringToObject(p, "type", "object");
			cJSON_AddStringToObject(p, "description", desc);
			return p;
		};

		auto req_list = [](std::initializer_list<const char*> names) {
			cJSON* r = cJSON_CreateArray();
			for (auto n : names)
				cJSON_AddItemToArray(r, cJSON_CreateString(n));
			return r;
		};

		// ===================================================
		// System tools
		// ===================================================

		{
			cJSON* p = cJSON_CreateObject();
			add_tool("system_status",
			         "Get system status: FPS, memory, queue state, node count",
			         "system.status",
			         schema_obj(p, nullptr));
		}
		{
			cJSON* p = cJSON_CreateObject();
			add_tool("system_queue",
			         "Get task queue progress and running state",
			         "system.queue",
			         schema_obj(p, nullptr));
		}
		{
			cJSON* p = cJSON_CreateObject();
			add_tool("system_log", "Get recent queue log messages",
			         "system.log", schema_obj(p, nullptr));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(p, "timeout_ms",
			                      prop_int("timeout_ms",
			                               "Max wait time in ms", 5000));
			add_tool("system_wait_idle",
			         "Block until the task queue is idle or timeout",
			         "system.waitIdle", schema_obj(p, nullptr));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "message",
			    prop_str("message", "Toast text to display", ""));
			cJSON_AddItemToObject(
			    p, "duration_ms",
			    prop_int("duration_ms", "Display duration in ms", 1000));
			add_tool("system_toast",
			         "Show a toast notification in the GUI",
			         "system.toast",
			         schema_obj(p, nullptr, "System Toast"));
		}

		// ===================================================
		// Project tools
		// ===================================================

		{
			cJSON* p = cJSON_CreateObject();
			add_tool("project_info",
			         "Get current project path, node count, dirty flag",
			         "project.info",
			         schema_obj(p, nullptr));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "path",
			    prop_str("path", "Project file path to open", "",
			             nullptr));
			add_tool(
			    "project_open",
			    "Open a project file (replaces current state)",
			    "project.open",
			    schema_obj(p, req_list({"path"}), "Open Project"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			add_tool("project_save", "Save current project",
			         "project.save", schema_obj(p, nullptr));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "path",
			    prop_str("path", "Target project file path", "",
			             nullptr));
			add_tool(
			    "project_save_as",
			    "Save project to a new file path",
			    "project.saveAs",
			    schema_obj(p, req_list({"path"}), "Save As"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			add_tool("project_create",
			         "Create a new empty project (clears all nodes)",
			         "project.create",
			         schema_obj(p, nullptr));
		}

		// ===================================================
		// Node tools
		// ===================================================

		{
			cJSON* p = cJSON_CreateObject();
			add_tool("node_list",
			         "List all nodes with id, title, triangle_count, children",
			         "node.list", schema_obj(p, nullptr));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "id",
			    prop_int("id", "Node ID to retrieve", 0));
			add_tool("node_get",
			         "Get full node details including mesh, voxel, visibility",
			         "node.get",
			         schema_obj(p, req_list({"id"}), "Get Node"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			add_tool("node_create", "Create a new empty node",
			         "node.create", schema_obj(p, nullptr));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "id",
			    prop_int("id", "Node ID to delete", 0));
			add_tool(
			    "node_delete", "Delete a node by ID",
			    "node.delete",
			    schema_obj(p, req_list({"id"}), "Delete Node"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "id",
			    prop_int("id", "Node ID to update", 0));
			cJSON_AddItemToObject(
			    p, "title",
			    prop_str("title", "New node title", "", nullptr));
			cJSON_AddItemToObject(
			    p, "alpha_wrap_alpha",
			    prop_num("alpha_wrap_alpha",
			             "Alpha wrap alpha parameter", 1.0));
			cJSON_AddItemToObject(
			    p, "alpha_wrap_offset",
			    prop_num("alpha_wrap_offset",
			             "Alpha wrap offset parameter", 0.01));
			cJSON_AddItemToObject(
			    p, "subdivide_level",
			    prop_int("subdivide_level",
			             "Mesh subdivision level (1=coarse)", 1));
			cJSON_AddItemToObject(
			    p, "repair_mode",
			    prop_int("repair_mode",
			             "Repair mode: 0=alpha_wrap, 1=fill_holes, "
			             "2=stitch, 3=merge_vertices, "
			             "4=orient",
			             0));
			add_tool("node_update",
			         "Update node properties (all fields optional)",
			         "node.update",
			         schema_obj(p, req_list({"id"}), "Update Node"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "id",
			    prop_int("id", "Node ID", 0));
			add_tool("node_get_children",
			         "Get child node IDs for a node",
			         "node.getChildren",
			         schema_obj(p, req_list({"id"})));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "id",
			    prop_int("id", "Node ID", 0));
			add_tool("node_get_bounds",
			         "Get voxel bounding box (min/max) for a node",
			         "node.getBounds",
			         schema_obj(p, req_list({"id"})));
		}

		// ===================================================
		// Mesh tools
		// ===================================================

		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "path",
			    prop_str("path", "STL file path to import", "",
			             nullptr));
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id", "Target node ID (-1 = create new)",
			             -1));
			cJSON_AddItemToObject(
			    p, "voxel_size",
			    prop_num("voxel_size", "Voxel size for SDF", 0.5));
			cJSON_AddItemToObject(
			    p, "load_mode",
			    prop_int("load_mode",
			             "0=default,1=silhouette,2=surface,3=mesh,4=convex",
			             0));
			cJSON_AddItemToObject(
			    p, "load_as_sdf",
			    prop_bool("load_as_sdf", "Load as SDF node",
			              false));
			cJSON_AddItemToObject(
			    p, "precision",
			    prop_str("precision",
			             "Voxel precision for SDF", "fast",
			             "fast,precise"));
			add_tool(
			    "mesh_import",
			    "Import a 3D mesh file (STL) and create a node",
			    "mesh.import",
			    schema_obj(p, req_list({"path"}), "Import Mesh"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id", "Node ID to export", 0));
			cJSON_AddItemToObject(
			    p, "path",
			    prop_str("path", "Output STL file path", "",
			             nullptr));
			cJSON_AddItemToObject(
			    p, "mode",
			    prop_int("mode", "Export mode (0=default)", 0));
			cJSON_AddItemToObject(
			    p, "simplify",
			    prop_bool("simplify",
			              "Whether to simplify mesh", false));
			cJSON_AddItemToObject(
			    p, "simplify_ratio",
			    prop_num("simplify_ratio",
			             "Simplification ratio (0-1)", 0.1));
			cJSON_AddItemToObject(
			    p, "subdivisions",
			    prop_int("subdivisions",
			             "SDF subdivisions for export", 3));
			add_tool("mesh_export",
			         "Export a node's mesh to STL file",
			         "mesh.export",
			         schema_obj(p, req_list({"node_id", "path"}),
			                    "Export Mesh"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "export_dir",
			    prop_str("export_dir", "Output directory path",
			             "", nullptr));
			cJSON_AddItemToObject(
			    p, "mode",
			    prop_int("mode", "Export mode (0=default)", 0));
			cJSON_AddItemToObject(
			    p, "simplify",
			    prop_bool("simplify",
			              "Whether to simplify meshes", false));
			cJSON_AddItemToObject(
			    p, "simplify_ratio",
			    prop_num("simplify_ratio",
			             "Simplification ratio (0-1)", 0.1));
			cJSON_AddItemToObject(
			    p, "subdivisions",
			    prop_int("subdivisions",
			             "SDF subdivisions for export", 3));
			add_tool("mesh_export_all",
			         "Export all nodes' meshes to a directory",
			         "mesh.exportAll",
			         schema_obj(p, req_list({"export_dir"}),
			                    "Export All"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id", "Node ID to repair", 0));
			cJSON_AddItemToObject(
			    p, "method",
			    prop_str("method",
			             "Repair method", "alpha_wrap",
			             "alpha_wrap,fill_holes,"
			             "stitch_borders,merge_vertices,"
			             "orient_volume"));
			cJSON_AddItemToObject(
			    p, "alpha",
			    prop_num("alpha",
			             "Alpha wrap parameter (for alpha_wrap)",
			             1.0));
			cJSON_AddItemToObject(
			    p, "offset",
			    prop_num("offset",
			             "Alpha wrap offset (for alpha_wrap)",
			             0.01));
			cJSON_AddItemToObject(
			    p, "tolerance",
			    prop_num("tolerance",
			             "Tolerance (for stitch/merge)", 1e-6));
			add_tool(
			    "mesh_repair",
			    "Repair mesh using CGAL (alpha_wrap, fill_holes, etc.)",
			    "mesh.repair",
			    schema_obj(p, req_list({"node_id"}),
			               "Repair Mesh"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id", "Node ID to subdivide", 0));
			cJSON_AddItemToObject(
			    p, "level",
			    prop_int("level",
			             "Subdivision level (1=coarse)", 1));
			add_tool(
			    "mesh_subdivide",
			    "Subdivide mesh triangles (Loop/Catmull-Clark)",
			    "mesh.subdivide",
			    schema_obj(p, req_list({"node_id"}),
			               "Subdivide Mesh"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id", "Node ID to simplify", 0));
			cJSON_AddItemToObject(
			    p, "ratio",
			    prop_num("ratio",
			             "Keep ratio (0-1, smaller = sparser)",
			             0.5));
			add_tool(
			    "mesh_simplify",
			    "Simplify mesh by edge collapse",
			    "mesh.simplify",
			    schema_obj(p, req_list({"node_id"}),
			               "Simplify Mesh"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_a",
			    prop_int("node_a", "First node ID", 0));
			cJSON_AddItemToObject(
			    p, "node_b",
			    prop_int("node_b", "Second node ID", 0));
			add_tool(
			    "mesh_boolean_union",
			    "Boolean union of two nodes' meshes",
			    "mesh.booleanUnion",
			    schema_obj(p, req_list({"node_a", "node_b"}),
			               "Boolean Union"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id", "Node ID to check", 0));
			add_tool("mesh_is_manifold",
			         "Check if a node's mesh is manifold (watertight)",
			         "mesh.isManifold",
			         schema_obj(p, req_list({"node_id"})));
		}

		// ===================================================
		// Strand tools
		// ===================================================

		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id",
			             "Node ID containing hair strands", 0));
			add_tool("strand_list",
			         "List all hair strands for a node (includes center point and addon options)",
			         "strand.list",
			         schema_obj(p, req_list({"node_id"})));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id",
			             "Node ID containing hair strands", 0));
			cJSON_AddItemToObject(
			    p, "strand_index",
			    prop_int("strand_index",
			             "Strand index (0-based)", 0));
			add_tool(
			    "strand_get",
			    "Get full data for one hair strand (guide points, width points, section)",
			    "strand.get",
			    schema_obj(p, req_list({"node_id", "strand_index"})));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id",
			             "Node ID to add strand to", 0));
			cJSON_AddItemToObject(
			    p, "name",
			    prop_str("name", "Strand name (auto if empty)", ""));
			add_tool(
			    "strand_create",
			    "Create a new hair strand on a node",
			    "strand.create",
			    schema_obj(p, req_list({"node_id"}),
			               "Create Strand"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id",
			             "Node ID containing the strand", 0));
			cJSON_AddItemToObject(
			    p, "strand_index",
			    prop_int("strand_index", "Strand index to delete",
			             0));
			add_tool(
			    "strand_delete",
			    "Delete a hair strand from a node",
			    "strand.delete",
			    schema_obj(
			        p, req_list({"node_id", "strand_index"}),
			        "Delete Strand"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id",
			             "Node ID containing the strand", 0));
			cJSON_AddItemToObject(
			    p, "strand_index",
			    prop_int("strand_index", "Strand index to update",
			             0));
			cJSON_AddItemToObject(
			    p, "name",
			    prop_str("name", "New strand name", ""));
			cJSON_AddItemToObject(
			    p, "section_rotation",
			    prop_num("section_rotation",
			             "Section rotation in degrees", 0));
			cJSON_AddItemToObject(
			    p, "guide_samples_per_segment",
			    prop_int("guide_samples_per_segment",
			             "Bezier samples per segment (4-128)", 32));
			cJSON_AddItemToObject(
			    p, "section_subdiv",
			    prop_int("section_subdiv",
			             "Section subdivision count (1-32)", 8));
			cJSON_AddItemToObject(
			    p, "repair_alpha",
			    prop_num("repair_alpha",
			             "Alpha wrap repair alpha", 1.0));
			cJSON_AddItemToObject(
			    p, "repair_offset",
			    prop_num("repair_offset",
			             "Alpha wrap repair offset", 0.01));
			cJSON_AddItemToObject(
			    p, "guide_points",
			    prop_arr("guide_points",
			             "Replace guide curve points: array of [x,y,z]",
			             "array"));
			cJSON_AddItemToObject(
			    p, "width_points",
			    prop_arr("width_points",
			             "Replace width vector points",
			             "object"));
			cJSON_AddItemToObject(
			    p, "section_vertices",
			    prop_arr("section_vertices",
			             "Cross-section polygon vertices: array of {x,y}",
			             "object"));
			cJSON_AddItemToObject(
			    p, "section_use_bezier",
			    prop_bool("section_use_bezier",
			              "Use Catmull-Rom smoothing on section",
			              false));
			add_tool(
			    "strand_update",
			    "Update hair strand properties (all fields optional, replacing arrays replaces entire data)",
			    "strand.update",
			    schema_obj(
			        p, req_list({"node_id", "strand_index"}),
			        "Update Strand"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id",
			             "Node ID containing the strand", 0));
			cJSON_AddItemToObject(
			    p, "strand_index",
			    prop_int("strand_index", "Strand index to move",
			             0));
			cJSON_AddItemToObject(
			    p, "direction",
			    prop_str("direction",
			             "Move direction", "up", "up,down"));
			add_tool(
			    "strand_move",
			    "Move a hair strand up or down in the list",
			    "strand.move",
			    schema_obj(
			        p,
			        req_list(
			            {"node_id", "strand_index", "direction"}),
			        "Move Strand"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id", "Hair node ID", 0));
			cJSON_AddItemToObject(p, "x",
			                      prop_num("x", "Center X", 0));
			cJSON_AddItemToObject(p, "y",
			                      prop_num("y", "Center Y", -5.0));
			cJSON_AddItemToObject(p, "z",
			                      prop_num("z", "Center Z", 0));
			cJSON_AddItemToObject(
			    p, "show",
			    prop_bool("show", "Show/enable center point",
			              true));
			add_tool("strand_set_center_point",
			         "Set the shared root convergence center point for all strands",
			         "strand.setCenterPoint",
			         schema_obj(p, req_list({"node_id"}),
			                    "Set Center Point"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id", "Hair addon node ID", 0));
			cJSON_AddItemToObject(
			    p, "addon_type",
			    prop_int("addon_type", "Addon type (0=hair)", 0));
			cJSON_AddItemToObject(
			    p, "base_node_id",
			    prop_int("base_node_id",
			             "Base model node ID (-1 = none)", -1));
			cJSON_AddItemToObject(
			    p, "reveal",
			    prop_bool("reveal",
			              "Reveal mode: subtract base from SDF",
			              false));
			cJSON_AddItemToObject(
			    p, "split",
			    prop_bool("split",
			              "Split mode: each strand to own node",
			              false));
			cJSON_AddItemToObject(
			    p, "sdf_boolean",
			    prop_bool("sdf_boolean",
			              "Use SDF boolean for reveal (vs CGAL)",
			              true));
			cJSON_AddItemToObject(
			    p, "sdf_split",
			    prop_bool("sdf_split",
			              "Use SDF boolean for split (vs CGAL)",
			              true));
			add_tool("strand_set_addon_options",
			         "Set addon options: base model binding, reveal/split modes",
			         "strand.setAddonOptions",
			         schema_obj(p, req_list({"node_id"}),
			                    "Set Addon Options"));
		}

		// ===================================================
		// Semantic coordinate tools
		// ===================================================

		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id", "Hair node ID", 0));
			cJSON_AddItemToObject(
			    p, "base_node_id",
			    prop_int("base_node_id",
			             "Base model node ID for ray casting", 0));
			cJSON_AddItemToObject(
			    p, "angles",
			    prop_arr("angles",
			             "Array of angle entries: [{x,y,theta,phi},...]",
			             "object"));
			add_tool("strand_set_angle_config",
			         "Configure per-position ray angles for semantic coordinate system. "
			         "Builds BVH tree from base model. MUST be called before "
			         "semantic point addition.",
			         "strand.setAngleConfig",
			         schema_obj(
			             p,
			             req_list(
			                 {"node_id", "base_node_id", "angles"}),
			             "Set Angle Config"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id",
			             "Node ID containing the strand", 0));
			cJSON_AddItemToObject(
			    p, "strand_index",
			    prop_int("strand_index",
			             "Strand index to add guide point to", 0));
			cJSON_AddItemToObject(
			    p, "x",
			    prop_num("x",
			             "Semantic X coordinate (-10 to +10)",
			             0));
			cJSON_AddItemToObject(
			    p, "y",
			    prop_num("y",
			             "Semantic Y coordinate (-10 to +14)",
			             0));
			add_tool(
			    "strand_add_semantic_guide_point",
			    "Add a guide point using semantic head coordinates (X,Y). "
			    "Casts a ray from the configured angle toward the center point, "
			    "finds the first hit on the base model surface.",
			    "strand.addSemanticGuidePoint",
			    schema_obj(
			        p,
			        req_list(
			            {"node_id", "strand_index", "x", "y"}),
			        "Add Semantic Guide Point"));
		}
		{
			cJSON* p = cJSON_CreateObject();
			cJSON_AddItemToObject(
			    p, "node_id",
			    prop_int("node_id",
			             "Node ID containing the strand", 0));
			cJSON_AddItemToObject(
			    p, "strand_index",
			    prop_int("strand_index",
			             "Strand index to add width point to", 0));
			cJSON_AddItemToObject(
			    p, "x",
			    prop_num("x",
			             "Semantic X coordinate (-10 to +10)",
			             0));
			cJSON_AddItemToObject(
			    p, "y",
			    prop_num("y",
			             "Semantic Y coordinate (-10 to +14)",
			             0));
			cJSON_AddItemToObject(
			    p, "scale",
			    prop_num("scale", "Width scale factor", 1.0));
			add_tool(
			    "strand_add_semantic_width_point",
			    "Add a width vector point using semantic coordinates. "
			    "Casts a ray, finds the surface point, projects it onto the guide "
			    "curve to compute curve_id, and sets the width direction toward "
			    "the surface. Requires ≥2 guide points on the strand.",
			    "strand.addSemanticWidthPoint",
			    schema_obj(
			        p,
			        req_list(
			            {"node_id", "strand_index", "x", "y"}),
			        "Add Semantic Width Point"));
		}

		return arr;
	}

	// ---- tools/call ----

	std::string handle_tools_call(const cJSON* id,
	                              cJSON* body,
	                              std::shared_ptr<McpSession> sess) {
		cJSON* params_item = cJSON_GetObjectItem(body, "params");
		if (!params_item || !cJSON_IsObject(params_item)) {
			return jsonrpc_error(id, -32602, "Invalid params",
			                     "params must be an object");
		}

		cJSON* name_item =
		    cJSON_GetObjectItem(params_item, "name");
		cJSON* args_item =
		    cJSON_GetObjectItem(params_item, "arguments");

		if (!name_item || !cJSON_IsString(name_item)) {
			return jsonrpc_error(id, -32602, "Invalid params",
			                     "missing tool name");
		}

		const char* tool_name = name_item->valuestring;

		// Build the tools list to find the dispatch method
		cJSON* tools = build_tool_list();
		std::string dispatch_method;
		int tool_count = cJSON_GetArraySize(tools);
		for (int i = 0; i < tool_count; ++i) {
			cJSON* tool = cJSON_GetArrayItem(tools, i);
			cJSON* tn =
			    cJSON_GetObjectItem(tool, "name");
			if (tn && cJSON_IsString(tn) &&
			    std::strcmp(tn->valuestring, tool_name) == 0) {
				cJSON* dm = cJSON_GetObjectItem(
				    tool, "_dispatch_method");
				if (dm && cJSON_IsString(dm))
					dispatch_method = dm->valuestring;
				break;
			}
		}
		cJSON_Delete(tools);

		if (dispatch_method.empty()) {
			return jsonrpc_error(
			    id, -32601, "Tool not found",
			    std::string("unknown tool: ") + tool_name);
		}

		// Convert MCP arguments to the params cJSON expected by the
		// dispatch handler. The arguments are a flat JSON object;
		// we pass them directly as the params.
		cJSON* dispatch_params = nullptr;
		if (args_item && cJSON_IsObject(args_item)) {
			// Deep-copy the arguments so the handler can take ownership
			dispatch_params = cJSON_Duplicate(args_item, 1);
		} else {
			dispatch_params = cJSON_CreateObject();
		}

		// Push to the command queue and wait for result
		AgentCommand cmd;
		cmd.method = dispatch_method;
		cmd.params = dispatch_params;  // ownership transfers
		cmd.timeout_ms = 30000;

		auto future = cmd.result_promise.get_future();
		queue_.push(std::move(cmd));

		auto status =
		    future.wait_for(std::chrono::milliseconds(30000));
		if (status == std::future_status::timeout) {
			return jsonrpc_error(
			    id, -32000, "Timeout",
			    "command timed out waiting for main thread");
		}

		cJSON* result = future.get();
		if (!result) {
			return jsonrpc_error(id, -32000, "Internal error",
			                     "null result from handler");
		}

		// Wrap the result for MCP
		char* raw = cJSON_PrintUnformatted(result);
		cJSON_Delete(result);
		std::string text(raw);
		cJSON_free(raw);

		cJSON* mcp_result = build_text_result(text);
		return jsonrpc_response(id, mcp_result);
	}
};

}  // namespace sinriv::kigstudio::agent
