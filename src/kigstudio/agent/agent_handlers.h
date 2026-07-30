#pragma once

/**
 * Agent-command dispatch: runs on the main thread and routes method
 * strings to the appropriate RenderVoxelList / CGAL operations.
 *
 * Every handler receives (cJSON* params, RenderVoxelList& list).
 * It TAKES OWNERSHIP of params (must cJSON_Delete or pass to helper).
 * It RETURNS a cJSON* that the caller owns (the HTTP thread will
 * serialise it and then cJSON_Delete it).
 */

#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <string>

#include <cJSON.h>

#include "kigstudio/cgal/mesh_repair.h"
#include "kigstudio/cgal/mesh_simplification.h"
#include "kigstudio/cgal/mesh_subdivision.h"
#include "ui/render_voxel_list.h"

namespace sinriv::kigstudio::agent {

using List = sinriv::ui::render::RenderVoxelList;
using Item = sinriv::ui::render::RenderVoxelList::RenderVoxelItem;
using MeshData = std::vector<std::tuple<
    sinriv::kigstudio::voxel::triangle_bvh<float>::triangle,
    sinriv::kigstudio::vec3<float>>>;

// ---- helpers ----

/// Build a standard success response: {"ok":true}
inline cJSON* ok_response() {
	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	return r;
}

/// Build a standard error: {"ok":false,"error":"...","code":"..."}
inline cJSON* error_response(const char* code, const char* msg) {
	cJSON* r = cJSON_CreateObject();
	cJSON_AddFalseToObject(r, "ok");
	cJSON_AddStringToObject(r, "error", msg);
	cJSON_AddStringToObject(r, "code", code);
	return r;
}

/// Find an item by id; returns nullptr and sets up `err` response.
inline Item* find_item(List& list, int id, cJSON*& err) {
	auto it = list.items.find(id);
	if (it == list.items.end()) {
		err = error_response("NODE_NOT_FOUND", "node id not found");
		return nullptr;
	}
	return it->second.get();
}

/// Extract an int from a cJSON object field, with a default.
inline int json_int(const cJSON* obj, const char* key, int def = 0) {
	if (!obj) return def;
	cJSON* item = cJSON_GetObjectItem(obj, key);
	if (!item || !cJSON_IsNumber(item)) return def;
	return static_cast<int>(item->valuedouble);
}

/// Extract a float from cJSON, with default.
inline float json_float(const cJSON* obj, const char* key, float def = 0.0f) {
	if (!obj) return def;
	cJSON* item = cJSON_GetObjectItem(obj, key);
	if (!item || !cJSON_IsNumber(item)) return def;
	return static_cast<float>(item->valuedouble);
}

/// Extract a string from cJSON, with default.
inline const char* json_str(const cJSON* obj, const char* key,
                            const char* def = "") {
	if (!obj) return def;
	cJSON* item = cJSON_GetObjectItem(obj, key);
	if (!item || !cJSON_IsString(item)) return def;
	return item->valuestring;
}

/// Extract a bool from cJSON, with default.
inline bool json_bool(const cJSON* obj, const char* key, bool def = false) {
	if (!obj) return def;
	cJSON* item = cJSON_GetObjectItem(obj, key);
	if (!item) return def;
	if (cJSON_IsTrue(item)) return true;
	if (cJSON_IsFalse(item)) return false;
	if (cJSON_IsNumber(item)) return item->valuedouble != 0.0;
	return def;
}

/// Serialise a vec3f to a cJSON array [x,y,z].
inline cJSON* vec3_to_json(const sinriv::kigstudio::voxel::vec3f& v) {
	cJSON* arr = cJSON_CreateArray();
	cJSON_AddItemToArray(arr, cJSON_CreateNumber(v.x));
	cJSON_AddItemToArray(arr, cJSON_CreateNumber(v.y));
	cJSON_AddItemToArray(arr, cJSON_CreateNumber(v.z));
	return arr;
}

// ---- handler declarations ----

cJSON* h_system_status(cJSON* params, List& list);
cJSON* h_system_queue(cJSON* params, List& list);
cJSON* h_system_log(cJSON* params, List& list);
cJSON* h_system_wait_idle(cJSON* params, List& list);
cJSON* h_system_toast(cJSON* params, List& list);
cJSON* h_project_info(cJSON* params, List& list);
cJSON* h_project_open(cJSON* params, List& list);
cJSON* h_project_save(cJSON* params, List& list);
cJSON* h_project_save_as(cJSON* params, List& list);
cJSON* h_project_create(cJSON* params, List& list);
cJSON* h_node_list(cJSON* params, List& list);
cJSON* h_node_get(cJSON* params, List& list);
cJSON* h_node_create(cJSON* params, List& list);
cJSON* h_node_delete(cJSON* params, List& list);
cJSON* h_node_update(cJSON* params, List& list);
cJSON* h_node_get_children(cJSON* params, List& list);
cJSON* h_node_get_bounds(cJSON* params, List& list);
cJSON* h_mesh_import(cJSON* params, List& list);
cJSON* h_mesh_export(cJSON* params, List& list);
cJSON* h_mesh_export_all(cJSON* params, List& list);
cJSON* h_mesh_repair(cJSON* params, List& list);
cJSON* h_mesh_subdivide(cJSON* params, List& list);
cJSON* h_mesh_simplify(cJSON* params, List& list);
cJSON* h_mesh_boolean_union(cJSON* params, List& list);
cJSON* h_mesh_is_manifold(cJSON* params, List& list);

// ---- dispatch ----

/// Main dispatch: route a method string to the appropriate handler.
/// Called from AgentServer::process_commands() on the main thread.
/// Takes ownership of `params`; returns a result that the caller owns.
inline cJSON* agent_dispatch(const std::string& method, cJSON* params,
                             List& list) {
	// Build the dispatch table once (static)
	static const std::map<std::string,
	                      std::function<cJSON*(cJSON*, List&)>>
	    table = {
	        // system
	        {"system.status", h_system_status},
	        {"system.queue", h_system_queue},
	        {"system.log", h_system_log},
	        {"system.waitIdle", h_system_wait_idle},
	        {"system.toast", h_system_toast},
	        // project
	        {"project.info", h_project_info},
	        {"project.open", h_project_open},
	        {"project.save", h_project_save},
	        {"project.saveAs", h_project_save_as},
	        {"project.create", h_project_create},
	        // node
	        {"node.list", h_node_list},
	        {"node.get", h_node_get},
	        {"node.create", h_node_create},
	        {"node.delete", h_node_delete},
	        {"node.update", h_node_update},
	        {"node.getChildren", h_node_get_children},
	        {"node.getBounds", h_node_get_bounds},
	        // mesh
	        {"mesh.import", h_mesh_import},
	        {"mesh.export", h_mesh_export},
	        {"mesh.exportAll", h_mesh_export_all},
	        {"mesh.repair", h_mesh_repair},
	        {"mesh.subdivide", h_mesh_subdivide},
	        {"mesh.simplify", h_mesh_simplify},
	        {"mesh.booleanUnion", h_mesh_boolean_union},
	        {"mesh.isManifold", h_mesh_is_manifold},
	    };

	auto it = table.find(method);
	if (it == table.end()) {
		if (params) cJSON_Delete(params);
		return error_response("UNKNOWN_METHOD", method.c_str());
	}

	cJSON* result = it->second(params, list);

	// If the handler didn't set "ok", add it
	if (result && !cJSON_GetObjectItem(result, "ok")) {
		// Assume success if no error field
		if (!cJSON_GetObjectItem(result, "error")) {
			cJSON_AddTrueToObject(result, "ok");
		} else {
			cJSON_AddFalseToObject(result, "ok");
		}
	}

	return result;
}

// ===================================================================
// system.* handlers
// ===================================================================

inline cJSON* h_system_status(cJSON* /*params*/, List& list) {
	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddBoolToObject(r, "queue_running", list.isQueueRunning());
	cJSON_AddNumberToObject(r, "queue_progress",
	                        static_cast<double>(list.getQueueProgress()));
	cJSON_AddNumberToObject(r, "fps", static_cast<double>(list.fps));
	cJSON_AddNumberToObject(r, "memory_mb",
	                        static_cast<double>(list.memory_current / 1024 / 1024));
	cJSON_AddStringToObject(r, "queue_status", list.getQueueStatus().c_str());
	cJSON_AddNumberToObject(r, "node_count",
	                        static_cast<int>(list.items.size()));
	return r;
}

inline cJSON* h_system_queue(cJSON* /*params*/, List& list) {
	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddBoolToObject(r, "running", list.isQueueRunning());
	cJSON_AddNumberToObject(r, "progress",
	                        static_cast<double>(list.getQueueProgress()));
	cJSON_AddStringToObject(r, "status", list.getQueueStatus().c_str());
	return r;
}

inline cJSON* h_system_log(cJSON* /*params*/, List& list) {
	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");

	std::lock_guard<std::mutex> lock(list.queue_log_mutex);
	cJSON_AddStringToObject(r, "log", list.queue_log_text.c_str());
	return r;
}

inline cJSON* h_system_wait_idle(cJSON* params, List& list) {
	int timeout_ms = json_int(params, "timeout_ms", 5000);
	cJSON_Delete(params);

	int waited = 0;
	const int kPollMs = 100;
	while (list.isQueueRunning() && waited < timeout_ms) {
		std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
		waited += kPollMs;
	}

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddBoolToObject(r, "timeout", waited >= timeout_ms);
	cJSON_AddNumberToObject(r, "waited_ms", waited);
	return r;
}

inline cJSON* h_system_toast(cJSON* params, List& list) {
	// Store the toast message for the UI to render
	// (the actual rendering happens in render_ui via a member variable)
	const char* msg = json_str(params, "message", "");
	int duration_ms = json_int(params, "duration_ms", 1000);

	// We use append_queue_log as a simple way to store the toast message
	// for now. A dedicated toast system can be added later.
	if (msg && msg[0]) {
		list.append_queue_log(msg);
	}

	cJSON_Delete(params);
	return ok_response();
}

// ===================================================================
// project.* handlers
// ===================================================================

inline cJSON* h_project_info(cJSON* /*params*/, List& list) {
	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddStringToObject(r, "path", list.project_path.c_str());
	cJSON_AddNumberToObject(r, "node_count",
	                        static_cast<int>(list.items.size()));
	cJSON_AddNumberToObject(r, "memory_mb",
	                        static_cast<double>(list.memory_current / 1024 / 1024));
	cJSON_AddBoolToObject(r, "dirty", list.has_dirty_items());
	return r;
}

inline cJSON* h_project_open(cJSON* params, List& list) {
	const char* path = json_str(params, "path", "");
	if (!path || !path[0]) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS", "path is required");
	}

	bool ok = list.load_project(path);
	cJSON_Delete(params);

	if (!ok) {
		cJSON* r = error_response("LOAD_FAILED", list.last_load_error.c_str());
		return r;
	}

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddNumberToObject(r, "node_count",
	                        static_cast<int>(list.items.size()));
	return r;
}

inline cJSON* h_project_save(cJSON* params, List& list) {
	cJSON_Delete(params);
	if (list.project_path.empty()) {
		return error_response("NO_PROJECT_PATH",
		                      "no project path; use saveAs instead");
	}
	bool ok = list.save_current_project();
	if (!ok) {
		cJSON* r = error_response("SAVE_FAILED", list.last_save_error.c_str());
		return r;
	}
	return ok_response();
}

inline cJSON* h_project_save_as(cJSON* params, List& list) {
	const char* path = json_str(params, "path", "");
	if (!path || !path[0]) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS", "path is required");
	}
	bool ok = list.save_project(path);
	cJSON_Delete(params);
	if (!ok) {
		cJSON* r = error_response("SAVE_FAILED", list.last_save_error.c_str());
		return r;
	}
	return ok_response();
}

inline cJSON* h_project_create(cJSON* /*params*/, List& list) {
	// Create an empty project — just clear everything
	list.items.clear();
	list.project_path.clear();
	return ok_response();
}

// ===================================================================
// node.* handlers
// ===================================================================

inline cJSON* h_node_list(cJSON* /*params*/, List& list) {
	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON* nodes = cJSON_CreateArray();
	cJSON_AddItemToObject(r, "nodes", nodes);

	std::lock_guard<std::mutex> lock(list.locker);
	for (const auto& [id, item_ptr] : list.items) {
		const auto& item = *item_ptr;
		cJSON* n = cJSON_CreateObject();
		cJSON_AddNumberToObject(n, "id", id);
		cJSON_AddStringToObject(n, "title", item.title.c_str());
		cJSON_AddNumberToObject(n, "source_type", item.source_type);
		cJSON_AddNumberToObject(n, "source_node_id", item.source_node_id);
		cJSON_AddNumberToObject(n, "segment_mode",
		                        static_cast<int>(item.segment_mode));
		cJSON_AddBoolToObject(n, "dirty", item.dirty);

		// Children
		cJSON* ch = cJSON_CreateArray();
		for (int child_id : item.children)
			cJSON_AddItemToArray(ch, cJSON_CreateNumber(child_id));
		cJSON_AddItemToObject(n, "children", ch);

		cJSON_AddNumberToObject(n, "root_id", item.root_id);

		// Mesh info
		cJSON_AddNumberToObject(n, "triangle_count",
		                        static_cast<int>(item.cached_mesh.size()));

		cJSON_AddItemToArray(nodes, n);
	}
	return r;
}

inline cJSON* h_node_get(cJSON* params, List& list) {
	int id = json_int(params, "id", -1);
	cJSON_Delete(params);

	cJSON* err = nullptr;
	std::lock_guard<std::mutex> lock(list.locker);
	Item* item = find_item(list, id, err);
	if (!item) return err;

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");

	cJSON* data = cJSON_CreateObject();
	cJSON_AddItemToObject(r, "data", data);

	cJSON_AddNumberToObject(data, "id", id);
	cJSON_AddStringToObject(data, "title", item->title.c_str());
	cJSON_AddNumberToObject(data, "root_id", item->root_id);
	cJSON_AddNumberToObject(data, "source_type", item->source_type);
	cJSON_AddNumberToObject(data, "source_node_id", item->source_node_id);
	cJSON_AddNumberToObject(data, "segment_mode",
	                        static_cast<int>(item->segment_mode));
	cJSON_AddNumberToObject(data, "repair_mode",
	                        static_cast<int>(item->repair_mode));
	cJSON_AddNumberToObject(data, "alpha_wrap_alpha",
	                        static_cast<double>(item->alpha_wrap_alpha));
	cJSON_AddNumberToObject(data, "alpha_wrap_offset",
	                        static_cast<double>(item->alpha_wrap_offset));
	cJSON_AddNumberToObject(data, "subdivide_level", item->subdivide_level);
	cJSON_AddBoolToObject(data, "dirty", item->dirty ? 1 : 0);

	// Children
	cJSON* ch = cJSON_CreateArray();
	for (int child_id : item->children)
		cJSON_AddItemToArray(ch, cJSON_CreateNumber(child_id));
	cJSON_AddItemToObject(data, "children", ch);

	// Mesh
	cJSON* mesh = cJSON_CreateObject();
	cJSON_AddNumberToObject(mesh, "triangle_count",
	                        static_cast<int>(item->cached_mesh.size()));
	cJSON_AddStringToObject(mesh, "stl_path", item->stl_path.c_str());
	cJSON_AddItemToObject(data, "mesh", mesh);

	// Voxel
	cJSON* voxel = cJSON_CreateObject();
	cJSON_AddBoolToObject(voxel, "has_grid", !item->voxel_grid_data.empty());
	cJSON_AddItemToObject(data, "voxel", voxel);

	// Visibility
	cJSON* vis = cJSON_CreateObject();
	cJSON_AddBoolToObject(vis, "origin_mesh", item->showOriginMesh);
	cJSON_AddBoolToObject(vis, "mesh", item->showMesh);
	cJSON_AddBoolToObject(vis, "exported_mesh", item->showExportedMesh);
	cJSON_AddBoolToObject(vis, "voxel", item->showVoxel);
	cJSON_AddBoolToObject(vis, "collision", item->showCollision);
	cJSON_AddItemToObject(data, "visibility", vis);

	return r;
}

inline cJSON* h_node_create(cJSON* params, List& list) {
	cJSON_Delete(params);
	Item* item = list.create_item();
	if (!item) {
		return error_response("CREATE_FAILED", "could not create node");
	}
	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddNumberToObject(r, "id", item->id);
	return r;
}

inline cJSON* h_node_delete(cJSON* params, List& list) {
	int id = json_int(params, "id", -1);
	cJSON_Delete(params);

	{
		std::lock_guard<std::mutex> lock(list.locker);
		if (list.items.find(id) == list.items.end()) {
			return error_response("NODE_NOT_FOUND", "node id not found");
		}
	}

	list.queue_remove_item(id);
	return ok_response();
}

inline cJSON* h_node_update(cJSON* params, List& list) {
	int id = json_int(params, "id", -1);
	if (id < 0) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS", "id is required");
	}

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, id, err);
	if (!item) {
		cJSON_Delete(params);
		return err;
	}

	// Apply each updatable field
	if (cJSON_HasObjectItem(params, "title")) {
		item->title = json_str(params, "title", "");
	}
	if (cJSON_HasObjectItem(params, "alpha_wrap_alpha")) {
		item->alpha_wrap_alpha = json_float(params, "alpha_wrap_alpha", 1.0f);
	}
	if (cJSON_HasObjectItem(params, "alpha_wrap_offset")) {
		item->alpha_wrap_offset = json_float(params, "alpha_wrap_offset", 0.01f);
	}
	if (cJSON_HasObjectItem(params, "subdivide_level")) {
		item->subdivide_level = json_int(params, "subdivide_level", 1);
	}
	if (cJSON_HasObjectItem(params, "repair_mode")) {
		item->repair_mode = static_cast<Item::RepairMeshMode>(
		    json_int(params, "repair_mode", 0));
	}

	cJSON_Delete(params);
	return ok_response();
}

inline cJSON* h_node_get_children(cJSON* params, List& list) {
	int id = json_int(params, "id", -1);
	cJSON_Delete(params);

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, id, err);
	if (!item) return err;

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON* ch = cJSON_CreateArray();
	for (int child_id : item->children)
		cJSON_AddItemToArray(ch, cJSON_CreateNumber(child_id));
	cJSON_AddItemToObject(r, "children", ch);
	return r;
}

inline cJSON* h_node_get_bounds(cJSON* params, List& list) {
	int id = json_int(params, "id", -1);
	cJSON_Delete(params);

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, id, err);
	if (!item) return err;

	auto [vmin, vmax] = item->voxel_renderer.getLocalBounds();

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON* bmin = cJSON_CreateObject();
	cJSON_AddNumberToObject(bmin, "x", static_cast<double>(vmin.x));
	cJSON_AddNumberToObject(bmin, "y", static_cast<double>(vmin.y));
	cJSON_AddNumberToObject(bmin, "z", static_cast<double>(vmin.z));
	cJSON_AddItemToObject(r, "min", bmin);
	cJSON* bmax = cJSON_CreateObject();
	cJSON_AddNumberToObject(bmax, "x", static_cast<double>(vmax.x));
	cJSON_AddNumberToObject(bmax, "y", static_cast<double>(vmax.y));
	cJSON_AddNumberToObject(bmax, "z", static_cast<double>(vmax.z));
	cJSON_AddItemToObject(r, "max", bmax);
	return r;
}

// ===================================================================
// mesh.* handlers
// ===================================================================

inline cJSON* h_mesh_import(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	const char* path = json_str(params, "path", "");
	float voxel_size = json_float(params, "voxel_size", 0.5f);
	int load_mode = json_int(params, "load_mode", 0);
	bool load_as_sdf = json_bool(params, "load_as_sdf", false);
	const char* precision_str = json_str(params, "precision", "fast");

	auto voxel_precision = sinriv::kigstudio::sdf::SDFPrecision::Fast;
	if (std::strcmp(precision_str, "precise") == 0) {
		voxel_precision = sinriv::kigstudio::sdf::SDFPrecision::Precise;
	}

	cJSON_Delete(params);

	if (!path || !path[0]) {
		return error_response("INVALID_PARAMS", "path is required");
	}

	if (node_id >= 0) {
		list.load_stl(path, voxel_size, 0.5, true, node_id, load_mode,
		              load_as_sdf, voxel_precision);
	} else {
		list.queue_load_stl(path, voxel_size, load_mode, load_as_sdf,
		                    voxel_precision);
	}

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");

	// Return triangle count if we have a node
	if (node_id >= 0) {
		std::lock_guard<std::mutex> lock(list.locker);
		auto it = list.items.find(node_id);
		if (it != list.items.end()) {
			cJSON_AddNumberToObject(r, "triangle_count",
			                        static_cast<int>(
			                            it->second->cached_mesh.size()));
		}
	}
	return r;
}

inline cJSON* h_mesh_export(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	const char* path = json_str(params, "path", "");
	int mode = json_int(params, "mode", 0);
	bool simplify = json_bool(params, "simplify", false);
	float ratio = json_float(params, "simplify_ratio", 0.1f);
	int subdivisions = json_int(params, "subdivisions", 3);

	if (node_id < 0 || !path || !path[0]) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS",
		                      "node_id and path are required");
	}

	list.queue_export_stl(node_id, path, mode, simplify, ratio, subdivisions,
	                      true);
	cJSON_Delete(params);

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddStringToObject(r, "file_path", path);
	return r;
}

inline cJSON* h_mesh_export_all(cJSON* params, List& list) {
	const char* export_dir = json_str(params, "export_dir", "");
	int mode = json_int(params, "mode", 0);
	bool simplify = json_bool(params, "simplify", false);
	float ratio = json_float(params, "simplify_ratio", 0.1f);
	int subdivisions = json_int(params, "subdivisions", 3);

	if (!export_dir || !export_dir[0]) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS", "export_dir is required");
	}

	list.queue_export_stl_all(export_dir, mode, simplify, ratio, subdivisions,
	                          true);
	cJSON_Delete(params);

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddNumberToObject(r, "exported",
	                        static_cast<int>(list.items.size()));
	return r;
}

inline cJSON* h_mesh_repair(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	const char* method = json_str(params, "method", "alpha_wrap");
	float alpha = json_float(params, "alpha", 1.0f);
	float offset = json_float(params, "offset", 0.01f);
	float tolerance = json_float(params, "tolerance",
	                             static_cast<float>(1e-6));

	if (node_id < 0) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS", "node_id is required");
	}

	// Lock and get the mesh data
	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) {
		cJSON_Delete(params);
		return err;
	}

	if (item->cached_mesh.empty()) {
		cJSON_Delete(params);
		return error_response("NO_MESH", "node has no cached mesh");
	}

	// Run the repair synchronously on the main thread
	// (for async, use the queue — but these CGAL ops are the queue tasks)
	MeshData result;
	if (std::strcmp(method, "alpha_wrap") == 0) {
		result = sinriv::kigstudio::cgal::alpha_wrap(
		    item->cached_mesh, static_cast<double>(alpha),
		    static_cast<double>(offset));
	} else if (std::strcmp(method, "fill_holes") == 0) {
		result = sinriv::kigstudio::cgal::fill_holes(item->cached_mesh);
	} else if (std::strcmp(method, "stitch_borders") == 0) {
		result = sinriv::kigstudio::cgal::stitch_borders(
		    item->cached_mesh, static_cast<double>(tolerance));
	} else if (std::strcmp(method, "merge_vertices") == 0) {
		result = sinriv::kigstudio::cgal::merge_duplicate_vertices(
		    item->cached_mesh, static_cast<double>(tolerance));
	} else if (std::strcmp(method, "orient_volume") == 0) {
		result = sinriv::kigstudio::cgal::orient_volume(item->cached_mesh);
	} else {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS", "unknown repair method");
	}

	if (!result.empty()) {
		item->cached_mesh = std::move(result);
		item->cached_mesh_dirty = false;
	}

	cJSON_Delete(params);

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddNumberToObject(r, "result_triangle_count",
	                        static_cast<int>(
	                            result.empty() ? item->cached_mesh.size()
	                                          : result.size()));
	if (result.empty() && !item->cached_mesh.empty()) {
		cJSON_AddStringToObject(r, "warning", "repair returned empty; mesh unchanged");
	}
	return r;
}

inline cJSON* h_mesh_subdivide(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	int level = json_int(params, "level", 1);

	if (node_id < 0) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS", "node_id is required");
	}

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) {
		cJSON_Delete(params);
		return err;
	}

	if (item->cached_mesh.empty()) {
		cJSON_Delete(params);
		return error_response("NO_MESH", "node has no cached mesh");
	}

	auto result = sinriv::kigstudio::cgal::subdivideMeshByLevel(
	    item->cached_mesh, level);
	if (!result.empty()) {
		item->cached_mesh = std::move(result);
		item->cached_mesh_dirty = false;
	}

	cJSON_Delete(params);

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddNumberToObject(r, "result_triangle_count",
	                        static_cast<int>(item->cached_mesh.size()));
	return r;
}

inline cJSON* h_mesh_simplify(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	float ratio = json_float(params, "ratio", 0.5f);

	if (node_id < 0) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS", "node_id is required");
	}

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) {
		cJSON_Delete(params);
		return err;
	}

	if (item->cached_mesh.empty()) {
		cJSON_Delete(params);
		return error_response("NO_MESH", "node has no cached mesh");
	}

	auto result = sinriv::kigstudio::cgal::simplifyMesh(item->cached_mesh,
	                                                     static_cast<double>(ratio));
	if (!result.empty()) {
		item->cached_mesh = std::move(result);
		item->cached_mesh_dirty = false;
	}

	cJSON_Delete(params);

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddNumberToObject(r, "result_triangle_count",
	                        static_cast<int>(item->cached_mesh.size()));
	return r;
}

inline cJSON* h_mesh_boolean_union(cJSON* params, List& list) {
	int node_a = json_int(params, "node_a", -1);
	int node_b = json_int(params, "node_b", -1);

	if (node_a < 0 || node_b < 0) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS",
		                      "node_a and node_b are required");
	}

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item_a = find_item(list, node_a, err);
	if (!item_a) { cJSON_Delete(params); return err; }
	Item* item_b = find_item(list, node_b, err);
	if (!item_b) { cJSON_Delete(params); return err; }

	if (item_a->cached_mesh.empty() || item_b->cached_mesh.empty()) {
		cJSON_Delete(params);
		return error_response("NO_MESH",
		                      "one or both nodes have no cached mesh");
	}

	auto result = sinriv::kigstudio::cgal::mesh_union(
	    item_a->cached_mesh, item_b->cached_mesh);

	cJSON_Delete(params);

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddNumberToObject(r, "result_triangle_count",
	                        static_cast<int>(result.size()));
	return r;
}

inline cJSON* h_mesh_is_manifold(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	cJSON_Delete(params);

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) return err;

	if (item->cached_mesh.empty()) {
		return error_response("NO_MESH", "node has no cached mesh");
	}

	bool ready = sinriv::kigstudio::cgal::is_boolean_ready(item->cached_mesh);

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddBoolToObject(r, "is_manifold", ready);
	return r;
}

}  // namespace sinriv::kigstudio::agent
