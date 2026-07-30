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

// Suppress some warnings from httplib.h in MSVC
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)  // conversion, possible loss of data
#endif

#include <httplib.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

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

	impl_->svr->stop();
	if (impl_->listen_thread.joinable()) {
		impl_->listen_thread.join();
	}

	// Drain any remaining commands so promises don't hang
	// (we can't process them, so set them to nullptr)
	running_.store(false, std::memory_order_release);
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
}

}  // namespace sinriv::kigstudio::agent
