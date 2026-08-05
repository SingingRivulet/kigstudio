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
using HairStrand = sinriv::ui::render::HairStrand;
using HairAngleEntry = sinriv::ui::render::HairAngleEntry;
using MeshData = std::vector<std::tuple<
    sinriv::kigstudio::voxel::triangle_bvh<float>::triangle,
    sinriv::kigstudio::vec3<float>>>;
using vec3f = sinriv::kigstudio::vec3<float>;

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

// strand
cJSON* h_strand_list(cJSON* params, List& list);
cJSON* h_strand_get(cJSON* params, List& list);
cJSON* h_strand_create(cJSON* params, List& list);
cJSON* h_strand_delete(cJSON* params, List& list);
cJSON* h_strand_update(cJSON* params, List& list);
cJSON* h_strand_move(cJSON* params, List& list);
cJSON* h_strand_set_center_point(cJSON* params, List& list);
cJSON* h_strand_set_addon_options(cJSON* params, List& list);
cJSON* h_strand_set_angle_config(cJSON* params, List& list);
cJSON* h_strand_add_semantic_guide_point(cJSON* params, List& list);
cJSON* h_strand_add_semantic_width_point(cJSON* params, List& list);
cJSON* h_strand_apply_hairline_spindle(cJSON* params, List& list);
cJSON* h_strand_create_2d(cJSON* params, List& list);

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
	        // strand
	        {"strand.list", h_strand_list},
	        {"strand.get", h_strand_get},
	        {"strand.create", h_strand_create},
	        {"strand.delete", h_strand_delete},
	        {"strand.update", h_strand_update},
	        {"strand.move", h_strand_move},
	        {"strand.setCenterPoint", h_strand_set_center_point},
	        {"strand.setAddonOptions", h_strand_set_addon_options},
	        {"strand.setAngleConfig", h_strand_set_angle_config},
	        {"strand.addSemanticGuidePoint",
	         h_strand_add_semantic_guide_point},
	        {"strand.addSemanticWidthPoint",
	         h_strand_add_semantic_width_point},
	        {"strand.applyHairlineSpindle",
	         h_strand_apply_hairline_spindle},
	        {"strand.create2d", h_strand_create_2d},
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

	cJSON* data = cJSON_CreateObject();
	cJSON_AddStringToObject(data, "path", list.project_path.c_str());
	cJSON_AddNumberToObject(data, "node_count",
	                        static_cast<int>(list.items.size()));
	cJSON_AddNumberToObject(data, "memory_mb",
	                        static_cast<double>(list.memory_current / 1024 / 1024));
	cJSON_AddBoolToObject(data, "dirty", list.has_dirty_items());
	cJSON_AddItemToObject(r, "data", data);
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

	cJSON* data = cJSON_CreateObject();
	cJSON* nodes = cJSON_CreateArray();
	cJSON_AddItemToObject(data, "nodes", nodes);

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

	cJSON_AddItemToObject(r, "data", data);
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

	cJSON* data = cJSON_CreateObject();
	cJSON* bmin = cJSON_CreateObject();
	cJSON_AddNumberToObject(bmin, "x", static_cast<double>(vmin.x));
	cJSON_AddNumberToObject(bmin, "y", static_cast<double>(vmin.y));
	cJSON_AddNumberToObject(bmin, "z", static_cast<double>(vmin.z));
	cJSON_AddItemToObject(data, "min", bmin);
	cJSON* bmax = cJSON_CreateObject();
	cJSON_AddNumberToObject(bmax, "x", static_cast<double>(vmax.x));
	cJSON_AddNumberToObject(bmax, "y", static_cast<double>(vmax.y));
	cJSON_AddNumberToObject(bmax, "z", static_cast<double>(vmax.z));
	cJSON_AddItemToObject(data, "max", bmax);
	cJSON_AddItemToObject(r, "data", data);
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

// ===================================================================
// strand.* handlers
// ===================================================================

/// Serialise a vec2f to cJSON object {x, y}
inline cJSON* vec2_to_json(const sinriv::kigstudio::vec2<float>& v) {
	cJSON* obj = cJSON_CreateObject();
	cJSON_AddNumberToObject(obj, "x", static_cast<double>(v.x));
	cJSON_AddNumberToObject(obj, "y", static_cast<double>(v.y));
	return obj;
}

/// Deserialise a cJSON array [x, y, z] to vec3f
inline bool json_to_vec3(cJSON* arr,
                         sinriv::kigstudio::voxel::vec3f& out) {
	if (!arr || !cJSON_IsArray(arr)) return false;
	cJSON* x = cJSON_GetArrayItem(arr, 0);
	cJSON* y = cJSON_GetArrayItem(arr, 1);
	cJSON* z = cJSON_GetArrayItem(arr, 2);
	if (!cJSON_IsNumber(x) || !cJSON_IsNumber(y) || !cJSON_IsNumber(z))
		return false;
	out.x = static_cast<float>(x->valuedouble);
	out.y = static_cast<float>(y->valuedouble);
	out.z = static_cast<float>(z->valuedouble);
	return true;
}

/// Deserialise a cJSON object {x, y} to vec2f
inline bool json_to_vec2(cJSON* obj,
                         sinriv::kigstudio::vec2<float>& out) {
	if (!obj) return false;
	cJSON* x = cJSON_GetObjectItem(obj, "x");
	cJSON* y = cJSON_GetObjectItem(obj, "y");
	if (!cJSON_IsNumber(x) || !cJSON_IsNumber(y)) return false;
	out.x = static_cast<float>(x->valuedouble);
	out.y = static_cast<float>(y->valuedouble);
	return true;
}

/// Helper: get strand by index, returns nullptr and sets err on failure
inline HairStrand* find_strand(Item* item, int strand_index, cJSON*& err) {
	if (strand_index < 0 ||
	    static_cast<size_t>(strand_index) >= item->hair_strands.size()) {
		err = error_response("STRAND_NOT_FOUND", "strand index out of range");
		return nullptr;
	}
	return &item->hair_strands[strand_index];
}

inline cJSON* h_strand_list(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	cJSON_Delete(params);

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) return err;

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");

	cJSON* data = cJSON_CreateObject();
	cJSON_AddNumberToObject(data, "strand_count",
	                        static_cast<int>(item->hair_strands.size()));

	cJSON* strands = cJSON_CreateArray();
	for (size_t i = 0; i < item->hair_strands.size(); ++i) {
		const auto& s = item->hair_strands[i];
		cJSON* so = cJSON_CreateObject();
		cJSON_AddNumberToObject(so, "index", static_cast<int>(i));
		cJSON_AddStringToObject(so, "name", s.name.c_str());
		cJSON_AddNumberToObject(so, "guide_point_count",
		                        static_cast<int>(s.guide_points.size()));
		cJSON_AddNumberToObject(so, "width_point_count",
		                        static_cast<int>(s.width_points.size()));
		cJSON_AddBoolToObject(so, "mesh_dirty", s.mesh_dirty);
		cJSON_AddBoolToObject(so, "repair_failed", s.repair_failed);
		cJSON_AddItemToArray(strands, so);
	}
	cJSON_AddItemToObject(data, "strands", strands);

	// Also include shared center point
	cJSON* cp = cJSON_CreateObject();
	cJSON_AddNumberToObject(cp, "x",
	                        static_cast<double>(item->addon_center_point.x));
	cJSON_AddNumberToObject(cp, "y",
	                        static_cast<double>(item->addon_center_point.y));
	cJSON_AddNumberToObject(cp, "z",
	                        static_cast<double>(item->addon_center_point.z));
	cJSON_AddBoolToObject(cp, "show", item->show_addon_center);
	cJSON_AddItemToObject(data, "center_point", cp);

	// Addon options
	cJSON* opts = cJSON_CreateObject();
	cJSON_AddNumberToObject(opts, "addon_type", item->addon_type);
	cJSON_AddNumberToObject(opts, "base_node_id", item->addon_base_node_id);
	cJSON_AddBoolToObject(opts, "reveal", item->addon_reveal);
	cJSON_AddBoolToObject(opts, "split", item->addon_split);
	cJSON_AddBoolToObject(opts, "sdf_boolean", item->addon_sdf_boolean);
	cJSON_AddBoolToObject(opts, "sdf_split", item->addon_sdf_split);
	cJSON_AddItemToObject(data, "addon_options", opts);

	cJSON_AddItemToObject(r, "data", data);
	return r;
}

inline cJSON* h_strand_get(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	int strand_index = json_int(params, "strand_index", -1);
	cJSON_Delete(params);

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) return err;
	HairStrand* strand = find_strand(item, strand_index, err);
	if (!strand) return err;

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");

	cJSON* sd = cJSON_CreateObject();
	cJSON_AddNumberToObject(sd, "index", strand_index);
	cJSON_AddStringToObject(sd, "name", strand->name.c_str());
	cJSON_AddNumberToObject(sd, "section_rotation",
	                        static_cast<double>(strand->section_rotation));
	cJSON_AddNumberToObject(sd, "guide_samples_per_segment",
	                        strand->guide_samples_per_segment);
	cJSON_AddNumberToObject(sd, "section_subdiv", strand->section_subdiv);
	cJSON_AddNumberToObject(sd, "repair_alpha",
	                        static_cast<double>(strand->repair_alpha));
	cJSON_AddNumberToObject(sd, "repair_offset",
	                        static_cast<double>(strand->repair_offset));
	cJSON_AddBoolToObject(sd, "mesh_dirty", strand->mesh_dirty);
	cJSON_AddBoolToObject(sd, "repair_failed", strand->repair_failed);
	cJSON_AddBoolToObject(sd, "expanded", strand->expanded);

	// Guide points
	cJSON* gps = cJSON_CreateArray();
	for (const auto& gp : strand->guide_points) {
		cJSON_AddItemToArray(gps, vec3_to_json(gp));
	}
	cJSON_AddItemToObject(sd, "guide_points", gps);

	// Width points
	cJSON* wps = cJSON_CreateArray();
	for (const auto& wp : strand->width_points) {
		cJSON* wo = cJSON_CreateObject();
		cJSON_AddNumberToObject(wo, "curve_id",
		                        static_cast<double>(wp.curve_id));
		cJSON_AddNumberToObject(wo, "scale",
		                        static_cast<double>(wp.scale));
		cJSON_AddItemToObject(wo, "direction", vec3_to_json(wp.direction));

		// Per-point section override
		if (wp.section_state.committed.size() >= 3) {
			cJSON* sec = cJSON_CreateObject();
			cJSON* verts = cJSON_CreateArray();
			for (const auto& v : wp.section_state.committed)
				cJSON_AddItemToArray(verts, vec2_to_json(v));
			cJSON_AddItemToObject(sec, "vertices", verts);
			cJSON_AddBoolToObject(sec, "use_bezier",
			                      wp.section_state.use_bezier_section);
			cJSON_AddItemToObject(wo, "section_override", sec);
		}
		cJSON_AddItemToArray(wps, wo);
	}
	cJSON_AddItemToObject(sd, "width_points", wps);

	// Section state (global)
	cJSON* sec = cJSON_CreateObject();
	cJSON* verts = cJSON_CreateArray();
	for (const auto& v : strand->section_state.committed)
		cJSON_AddItemToArray(verts, vec2_to_json(v));
	cJSON_AddItemToObject(sec, "vertices", verts);
	cJSON_AddBoolToObject(sec, "use_bezier",
	                      strand->section_state.use_bezier_section);
	cJSON_AddItemToObject(sd, "section_state", sec);

	cJSON* data = cJSON_CreateObject();
	cJSON_AddItemToObject(data, "strand", sd);
	cJSON_AddItemToObject(r, "data", data);
	return r;
}

inline cJSON* h_strand_create(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	const char* name = json_str(params, "name", "");
	cJSON_Delete(params);

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) return err;

	HairStrand strand;
	if (name && name[0]) {
		strand.name = name;
	} else {
		strand.name =
		    "Strand " + std::to_string(item->hair_strands.size() + 1);
	}
	strand.expanded = true;
	item->hair_strands.push_back(std::move(strand));

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON* data = cJSON_CreateObject();
	cJSON_AddNumberToObject(data, "strand_index",
	                        static_cast<int>(item->hair_strands.size() - 1));
	cJSON_AddItemToObject(r, "data", data);
	return r;
}

inline cJSON* h_strand_delete(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	int strand_index = json_int(params, "strand_index", -1);
	cJSON_Delete(params);

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) return err;
	HairStrand* strand = find_strand(item, strand_index, err);
	if (!strand) return err;

	// Deactivate any active editing on this strand
	if (item->active_guide_draw_strand == strand_index) {
		item->guide_curve_drawing_active = false;
		item->active_guide_draw_strand = -1;
	} else if (item->active_guide_draw_strand > strand_index) {
		item->active_guide_draw_strand--;
	}
	if (item->active_width_edit_strand == strand_index) {
		item->width_editing_active = false;
		item->active_width_edit_strand = -1;
	} else if (item->active_width_edit_strand > strand_index) {
		item->active_width_edit_strand--;
	}
	if (item->active_section_edit_strand == strand_index) {
		item->active_section_edit_strand = -1;
	} else if (item->active_section_edit_strand > strand_index) {
		item->active_section_edit_strand--;
	}

	item->hair_strands.erase(item->hair_strands.begin() + strand_index);
	return ok_response();
}

inline cJSON* h_strand_update(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	int strand_index = json_int(params, "strand_index", -1);
	if (node_id < 0 || strand_index < 0) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS",
		                      "node_id and strand_index are required");
	}

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) { cJSON_Delete(params); return err; }
	HairStrand* strand = find_strand(item, strand_index, err);
	if (!strand) { cJSON_Delete(params); return err; }

	// Apply updatable fields
	if (cJSON_HasObjectItem(params, "name")) {
		strand->name = json_str(params, "name", "");
	}
	if (cJSON_HasObjectItem(params, "section_rotation")) {
		strand->section_rotation =
		    json_float(params, "section_rotation", 0.0f);
		strand->mesh_dirty = true;
	}
	if (cJSON_HasObjectItem(params, "guide_samples_per_segment")) {
		strand->guide_samples_per_segment =
		    json_int(params, "guide_samples_per_segment", 32);
		strand->mesh_dirty = true;
	}
	if (cJSON_HasObjectItem(params, "section_subdiv")) {
		strand->section_subdiv = json_int(params, "section_subdiv", 8);
		strand->mesh_dirty = true;
	}
	if (cJSON_HasObjectItem(params, "repair_alpha")) {
		strand->repair_alpha =
		    json_float(params, "repair_alpha", 1.0f);
		strand->mesh_dirty = true;
	}
	if (cJSON_HasObjectItem(params, "repair_offset")) {
		strand->repair_offset =
		    json_float(params, "repair_offset", 0.01f);
		strand->mesh_dirty = true;
	}

	// Replace guide_points entirely if provided
	if (cJSON_HasObjectItem(params, "guide_points")) {
		cJSON* gps = cJSON_GetObjectItem(params, "guide_points");
		if (cJSON_IsArray(gps)) {
			strand->guide_points.clear();
			int n = cJSON_GetArraySize(gps);
			for (int i = 0; i < n; ++i) {
				cJSON* pt = cJSON_GetArrayItem(gps, i);
				sinriv::kigstudio::voxel::vec3f v;
				if (json_to_vec3(pt, v))
					strand->guide_points.push_back(v);
			}
			strand->mesh_dirty = true;
		}
	}

	// Replace width_points entirely if provided
	if (cJSON_HasObjectItem(params, "width_points")) {
		cJSON* wps = cJSON_GetObjectItem(params, "width_points");
		if (cJSON_IsArray(wps)) {
			strand->width_points.clear();
			int n = cJSON_GetArraySize(wps);
			for (int i = 0; i < n; ++i) {
				cJSON* wo = cJSON_GetArrayItem(wps, i);
				if (!cJSON_IsObject(wo)) continue;
				HairStrand::WidthPoint wp;
				wp.curve_id = json_float(wo, "curve_id", 0.0f);
				wp.scale = json_float(wo, "scale", 1.0f);
				cJSON* dir = cJSON_GetObjectItem(wo, "direction");
				if (!json_to_vec3(dir, wp.direction))
					wp.direction = {0.0f, 0.0f, 1.0f};

				// Optional per-point section override
				cJSON* sec = cJSON_GetObjectItem(wo, "section_override");
				if (cJSON_IsObject(sec)) {
					cJSON* verts = cJSON_GetObjectItem(sec, "vertices");
					if (cJSON_IsArray(verts)) {
						int vn = cJSON_GetArraySize(verts);
						for (int vi = 0; vi < vn; ++vi) {
							sinriv::kigstudio::vec2<float> v2;
							if (json_to_vec2(
							        cJSON_GetArrayItem(verts, vi), v2))
								wp.section_state.committed.push_back(v2);
						}
						wp.section_state.vertices =
						    wp.section_state.committed;
					}
					wp.section_state.use_bezier_section =
					    json_bool(sec, "use_bezier", false);
				}
				strand->width_points.push_back(std::move(wp));
			}
			strand->mesh_dirty = true;
		}
	}

	// Replace section state if provided
	if (cJSON_HasObjectItem(params, "section_vertices")) {
		cJSON* verts = cJSON_GetObjectItem(params, "section_vertices");
		if (cJSON_IsArray(verts)) {
			strand->section_state.committed.clear();
			int n = cJSON_GetArraySize(verts);
			for (int i = 0; i < n; ++i) {
				sinriv::kigstudio::vec2<float> v;
				if (json_to_vec2(cJSON_GetArrayItem(verts, i), v))
					strand->section_state.committed.push_back(v);
			}
			strand->section_state.vertices =
			    strand->section_state.committed;
			strand->mesh_dirty = true;
		}
	}
	if (cJSON_HasObjectItem(params, "section_use_bezier")) {
		strand->section_state.use_bezier_section =
		    json_bool(params, "section_use_bezier", true);
		strand->mesh_dirty = true;
	}
	if (cJSON_HasObjectItem(params, "section_normalize_mode")) {
		int mode = json_int(params, "section_normalize_mode", 2);
		strand->section_state.normalize_mode =
		    static_cast<sinriv::ui::render::NormalizeMode>(
		        std::clamp(mode, 0, 2));
		strand->mesh_dirty = true;
	}

	cJSON_Delete(params);
	return ok_response();
}

/// strand.create2d — create/update a strand from 2D pixel coordinates.
///
/// Takes 2D guide points in render-pixel space and raycasts each one
/// against the base model's triangle mesh to find the true 3D surface
/// intersection — exactly the same path as manual guide-point clicking
/// in the ortho editor.
///
/// If a pixel does not hit the model, the point is silently projected
/// onto the image plane as a fallback.
///
/// Body params:
///   node_id (int)              — the node that owns the strand
///   guide_points_2d (array)    — [[x0,y0], [x1,y1], …] in render-resolution pixels
///   name (string, optional)    — strand display name
///   strand_index (int, opt)    — if set, update an existing strand instead of creating
///   guide_samples_per_segment (int, opt, default 64)
/// Extrapolate a new guide point from existing 3D curve when raycast misses
/// the model.  Builds a curvature-preserving tangent from the last 2+ guide
/// points, then finds the closest approach between the camera ray and the
/// extrapolated line.  Returns true and sets out_pt (on the ray).
inline bool extrapolate_guide_along_ray(const vec3f& ro, const vec3f& rd,
                                        const std::vector<vec3f>& existing,
                                        vec3f& out_pt, float vp_size) {
	const size_t n = existing.size();
	if (n < 2) return false;

	const vec3f& P_last = existing.back();

	// ---- Build curvature-preserving tangent direction ----
	vec3f T;
	if (n == 2) {
		const vec3f& A = existing[n - 2];
		T = vec3f{P_last.x - A.x, P_last.y - A.y, P_last.z - A.z};
	} else {
		// Blend last 2-3 normalised segment directions, weighted toward recent
		size_t seg_count = std::min(n - 1, size_t(3));
		float w_sum = 0.0f;
		T = vec3f{0, 0, 0};
		for (size_t i = 0; i < seg_count; ++i) {
			size_t idx = n - seg_count - 1 + i;
			const vec3f& A = existing[idx];
			const vec3f& B = existing[idx + 1];
			vec3f dir{B.x - A.x, B.y - A.y, B.z - A.z};
			float len = std::sqrt(dir.x * dir.x + dir.y * dir.y +
			                      dir.z * dir.z);
			if (len < 1e-8f) continue;
			dir.x /= len; dir.y /= len; dir.z /= len;
			float w = static_cast<float>(i + 1);  // linear ramp
			T.x += dir.x * w; T.y += dir.y * w; T.z += dir.z * w;
			w_sum += w;
		}
		if (w_sum > 0.0f) {
			T.x /= w_sum; T.y /= w_sum; T.z /= w_sum;
		} else {
			const vec3f& A = existing[n - 2];
			T = vec3f{P_last.x - A.x, P_last.y - A.y, P_last.z - A.z};
		}
	}

	// Normalise
	float t_len = std::sqrt(T.x * T.x + T.y * T.y + T.z * T.z);
	if (t_len < 1e-8f) return false;
	T.x /= t_len; T.y /= t_len; T.z /= t_len;

	// ---- Two-line closest-point (ray vs extrapolation line) ----
	float b = rd.x * T.x + rd.y * T.y + rd.z * T.z;
	if (std::abs(b) > 0.9999f) return false;  // near-parallel

	vec3f v{ro.x - P_last.x, ro.y - P_last.y, ro.z - P_last.z};
	float d = -(v.x * rd.x + v.y * rd.y + v.z * rd.z);
	float e = -(v.x * T.x + v.y * T.y + v.z * T.z);
	float inv_det = 1.0f / (b * b - 1.0f);
	float s = (d - b * e) * inv_det;
	float t = (b * d - e) * inv_det;

	// Clamp to forward direction (s>=0: in front of image plane;
	// t>=0: forward along the extrapolated curve)
	if (s < 0.0f) s = 0.0f;
	if (t < 0.0f) t = 0.0f;

	// Constrained closest-point pair
	vec3f c_ray{ro.x + s * rd.x, ro.y + s * rd.y, ro.z + s * rd.z};
	vec3f c_line{P_last.x + t * T.x, P_last.y + t * T.y,
	             P_last.z + t * T.z};
	float dist = std::sqrt((c_ray.x - c_line.x) * (c_ray.x - c_line.x) +
	                       (c_ray.y - c_line.y) * (c_ray.y - c_line.y) +
	                       (c_ray.z - c_line.z) * (c_ray.z - c_line.z));

	if (dist > vp_size * 0.5f) return false;

	out_pt = c_ray;
	return true;}

inline cJSON* h_strand_create_2d(cJSON* params, List& list) {
	using vec3f = sinriv::kigstudio::voxel::vec3f;

	int node_id = json_int(params, "node_id", -1);
	const char* name = json_str(params, "name", "");
	int strand_index = json_int(params, "strand_index", -1);
	int guide_samples = json_int(params, "guide_samples_per_segment", 64);

	cJSON* gps_2d = cJSON_GetObjectItem(params, "guide_points_2d");
	if (!gps_2d || !cJSON_IsArray(gps_2d) || cJSON_GetArraySize(gps_2d) < 2) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS",
		                      "guide_points_2d array with >=2 points is required");
	}

	// --- Read ortho camera state for 2D → 3D conversion ---
	auto& os = list.ortho_state;
	int res = os.render_resolution;
	float half = os.viewport_size * 0.5f;
	vec3f center = os._center;
	vec3f cam_right = os._cam_right;
	vec3f cam_up = os._cam_up;
	vec3f ray_dir = os.projection_dir;       // toward the model
	float vp = os.viewport_size;

	// --- Möller–Trumbore ray-triangle intersection lambda ---
	auto ray_tri_hit = [](const vec3f& ro, const vec3f& rd,
	                      const vec3f& v0, const vec3f& v1,
	                      const vec3f& v2, float& t) -> bool {
		const float eps = 1e-8f;
		vec3f e1{v1.x - v0.x, v1.y - v0.y, v1.z - v0.z};
		vec3f e2{v2.x - v0.x, v2.y - v0.y, v2.z - v0.z};
		vec3f pvec{rd.y * e2.z - rd.z * e2.y,
		            rd.z * e2.x - rd.x * e2.z,
		            rd.x * e2.y - rd.y * e2.x};
		float det = e1.x * pvec.x + e1.y * pvec.y + e1.z * pvec.z;
		if (std::abs(det) < eps) return false;
		float inv_det = 1.0f / det;
		vec3f tvec{ro.x - v0.x, ro.y - v0.y, ro.z - v0.z};
		float u = (tvec.x * pvec.x + tvec.y * pvec.y + tvec.z * pvec.z) * inv_det;
		if (u < 0.0f || u > 1.0f) return false;
		vec3f qvec{tvec.y * e1.z - tvec.z * e1.y,
		            tvec.z * e1.x - tvec.x * e1.z,
		            tvec.x * e1.y - tvec.y * e1.x};
		float v = (rd.x * qvec.x + rd.y * qvec.y + rd.z * qvec.z) * inv_det;
		if (v < 0.0f || u + v > 1.0f) return false;
		t = (e2.x * qvec.x + e2.y * qvec.y + e2.z * qvec.z) * inv_det;
		return t > eps;
	};

	// --- Convert each 2D pixel → 3D surface point ---
	auto& triangles = os._base_triangles;
	int n = cJSON_GetArraySize(gps_2d);
	int hit_count = 0;
	std::vector<vec3f> guide_3d;
	guide_3d.reserve(n);

	for (int i = 0; i < n; ++i) {
		cJSON* pt = cJSON_GetArrayItem(gps_2d, i);
		if (!cJSON_IsArray(pt) || cJSON_GetArraySize(pt) < 2) {
			cJSON_Delete(params);
			return error_response("INVALID_PARAMS",
			                      "each guide_points_2d entry must be [x, y]");
		}
		float px = static_cast<float>(cJSON_GetArrayItem(pt, 0)->valuedouble);
		float py = static_cast<float>(cJSON_GetArrayItem(pt, 1)->valuedouble);

		// Image-plane point (in front of the model)
		float u = (px / res - 0.5f);
		float v = (0.5f - py / res);
		vec3f plane_pt{
			center.x + cam_right.x * u * vp + cam_up.x * v * vp,
			center.y + cam_right.y * u * vp + cam_up.y * v * vp,
			center.z + cam_right.z * u * vp + cam_up.z * v * vp
		};

		// Raycast against the base-model surface
		float best_t = 1e30f;
		bool hit = false;
		for (const auto& tri : triangles) {
			float t;
			if (ray_tri_hit(plane_pt, ray_dir,
			                std::get<0>(tri), std::get<1>(tri),
			                std::get<2>(tri), t)) {
				if (t < best_t) { best_t = t; hit = true; }
			}
		}

		if (hit) {
			guide_3d.push_back({
				plane_pt.x + ray_dir.x * best_t,
				plane_pt.y + ray_dir.y * best_t,
				plane_pt.z + ray_dir.z * best_t
			});
			hit_count++;
		} else {
			// Try extrapolation from existing guide points
			vec3f extrapolated;
			if (guide_3d.size() >= 2 &&
			    extrapolate_guide_along_ray(plane_pt, ray_dir,
			                                guide_3d, extrapolated,
			                                vp)) {
				guide_3d.push_back(extrapolated);
				hit_count++;  // count as surface hit
			} else {
				guide_3d.push_back(plane_pt);
			}
		}
	}

	// --- Process width_points_2d if provided ---
	int wp_total = 0, wp_surface_hits = 0;
	std::vector<HairStrand::WidthPoint> width_3d;

	cJSON* wps_2d = cJSON_GetObjectItem(params, "width_points_2d");
	if (wps_2d && cJSON_IsArray(wps_2d)) {
		int wn = cJSON_GetArraySize(wps_2d);
		for (int i = 0; i < wn; ++i) {
			cJSON* wo = cJSON_GetArrayItem(wps_2d, i);
			if (!cJSON_IsObject(wo)) continue;

			HairStrand::WidthPoint wp;
			wp.curve_id = json_float(wo, "curve_id", static_cast<float>(i));
			wp.scale = json_float(wo, "scale", 1.0f);

			// Convert 2D direction → 3D image-plane direction
			cJSON* dir2d = cJSON_GetObjectItem(wo, "direction_2d");
			float dx = 1.0f, dy = 0.0f;
			if (cJSON_IsArray(dir2d) && cJSON_GetArraySize(dir2d) >= 2) {
				dx = static_cast<float>(
				    cJSON_GetArrayItem(dir2d, 0)->valuedouble);
				dy = static_cast<float>(
				    cJSON_GetArrayItem(dir2d, 1)->valuedouble);
			}
			// Normalise 2D direction
			float dlen2 = std::sqrt(dx*dx + dy*dy);
			if (dlen2 > 1e-8f) { dx /= dlen2; dy /= dlen2; }

			// 3D = cam_right * dx + cam_up * dy
			vec3f wdir{
				cam_right.x * dx + cam_up.x * dy,
				cam_right.y * dx + cam_up.y * dy,
				cam_right.z * dx + cam_up.z * dy
			};
			float dlen3 = std::sqrt(wdir.x*wdir.x + wdir.y*wdir.y + wdir.z*wdir.z);
			if (dlen3 > 1e-8f) {
				wdir.x /= dlen3; wdir.y /= dlen3; wdir.z /= dlen3;
			}

			wp.direction = wdir;
			width_3d.push_back(std::move(wp));
			wp_total++;
		}
	}

	cJSON_Delete(params);

	// --- Create or update strand ---
	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) return err;

	HairStrand* strand = nullptr;
	int actual_index = strand_index;
	bool is_new = false;

	if (strand_index >= 0 && strand_index < static_cast<int>(item->hair_strands.size())) {
		strand = &item->hair_strands[strand_index];
	} else {
		HairStrand s;
		if (name && name[0])
			s.name = name;
		else
			s.name = "Strand " + std::to_string(item->hair_strands.size() + 1);
		s.expanded = true;
		item->hair_strands.push_back(std::move(s));
		actual_index = static_cast<int>(item->hair_strands.size() - 1);
		strand = &item->hair_strands[actual_index];
		is_new = true;
	}

	strand->guide_points = guide_3d;
	guide_3d.clear();
	strand->guide_samples_per_segment = std::max(guide_samples, 2);
	if (!width_3d.empty()) {
		strand->width_points = std::move(width_3d);
	}
	strand->mesh_dirty = true;

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON* data = cJSON_CreateObject();
	cJSON_AddNumberToObject(data, "strand_index", actual_index);
	cJSON_AddBoolToObject(data, "created", is_new);
	cJSON_AddNumberToObject(data, "guide_point_count",
	                        static_cast<int>(strand->guide_points.size()));
	cJSON_AddNumberToObject(data, "surface_hits", hit_count);
	cJSON_AddNumberToObject(data, "width_point_count",
	                        static_cast<int>(strand->width_points.size()));
	cJSON_AddItemToObject(r, "data", data);
	return r;
}

inline cJSON* h_strand_move(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	int strand_index = json_int(params, "strand_index", -1);
	const char* direction = json_str(params, "direction", "up");
	cJSON_Delete(params);

	if (node_id < 0 || strand_index < 0) {
		return error_response("INVALID_PARAMS",
		                      "node_id and strand_index are required");
	}

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) return err;
	HairStrand* strand = find_strand(item, strand_index, err);
	if (!strand) return err;

	size_t si = static_cast<size_t>(strand_index);
	if (std::strcmp(direction, "up") == 0) {
		if (si == 0) {
			return error_response("INVALID_PARAMS",
			                      "strand is already at the top");
		}
		std::swap(item->hair_strands[si], item->hair_strands[si - 1]);
		item->hair_strands[si].mesh_dirty = true;
		item->hair_strands[si - 1].mesh_dirty = true;

		// Adjust active indices
		if (item->active_guide_draw_strand == strand_index)
			item->active_guide_draw_strand--;
		else if (item->active_guide_draw_strand == strand_index - 1)
			item->active_guide_draw_strand++;
		if (item->active_width_edit_strand == strand_index)
			item->active_width_edit_strand--;
		else if (item->active_width_edit_strand == strand_index - 1)
			item->active_width_edit_strand++;
		if (item->active_section_edit_strand == strand_index)
			item->active_section_edit_strand--;
		else if (item->active_section_edit_strand == strand_index - 1)
			item->active_section_edit_strand++;
	} else if (std::strcmp(direction, "down") == 0) {
		if (si >= item->hair_strands.size() - 1) {
			return error_response("INVALID_PARAMS",
			                      "strand is already at the bottom");
		}
		std::swap(item->hair_strands[si], item->hair_strands[si + 1]);
		item->hair_strands[si].mesh_dirty = true;
		item->hair_strands[si + 1].mesh_dirty = true;

		if (item->active_guide_draw_strand == strand_index)
			item->active_guide_draw_strand++;
		else if (item->active_guide_draw_strand == strand_index + 1)
			item->active_guide_draw_strand--;
		if (item->active_width_edit_strand == strand_index)
			item->active_width_edit_strand++;
		else if (item->active_width_edit_strand == strand_index + 1)
			item->active_width_edit_strand--;
		if (item->active_section_edit_strand == strand_index)
			item->active_section_edit_strand++;
		else if (item->active_section_edit_strand == strand_index + 1)
			item->active_section_edit_strand--;
	} else {
		return error_response("INVALID_PARAMS",
		                      "direction must be 'up' or 'down'");
	}

	return ok_response();
}

inline cJSON* h_strand_set_center_point(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	if (node_id < 0) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS", "node_id is required");
	}

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) { cJSON_Delete(params); return err; }

	if (cJSON_HasObjectItem(params, "show")) {
		item->show_addon_center = json_bool(params, "show", false);
	}
	if (cJSON_HasObjectItem(params, "x")) {
		item->addon_center_point.x = json_float(params, "x", 0.0f);
	}
	if (cJSON_HasObjectItem(params, "y")) {
		item->addon_center_point.y = json_float(params, "y", 0.0f);
	}
	if (cJSON_HasObjectItem(params, "z")) {
		item->addon_center_point.z = json_float(params, "z", 0.0f);
	}

	// Mark all strands dirty when center point changes
	for (auto& s : item->hair_strands) s.mesh_dirty = true;

	cJSON_Delete(params);
	return ok_response();
}

inline cJSON* h_strand_set_addon_options(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	if (node_id < 0) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS", "node_id is required");
	}

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) { cJSON_Delete(params); return err; }

	if (cJSON_HasObjectItem(params, "addon_type")) {
		item->addon_type = json_int(params, "addon_type", 0);
	}
	if (cJSON_HasObjectItem(params, "base_node_id")) {
		item->addon_base_node_id =
		    json_int(params, "base_node_id", -1);
	}
	if (cJSON_HasObjectItem(params, "reveal")) {
		item->addon_reveal = json_bool(params, "reveal", false);
	}
	if (cJSON_HasObjectItem(params, "split")) {
		item->addon_split = json_bool(params, "split", false);
	}
	if (cJSON_HasObjectItem(params, "sdf_boolean")) {
		item->addon_sdf_boolean =
		    json_bool(params, "sdf_boolean", true);
	}
	if (cJSON_HasObjectItem(params, "sdf_split")) {
		item->addon_sdf_split =
		    json_bool(params, "sdf_split", true);
	}

	cJSON_Delete(params);
	return ok_response();
}

// ===================================================================
// strand semantic-coordinate handlers
// ===================================================================

inline cJSON* h_strand_set_angle_config(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	int base_node_id = json_int(params, "base_node_id", -1);
	cJSON* angles_arr = cJSON_GetObjectItem(params, "angles");

	if (node_id < 0 || base_node_id < 0 || !cJSON_IsArray(angles_arr)) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS",
		                      "node_id, base_node_id, and angles[] are required");
	}

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) { cJSON_Delete(params); return err; }
	Item* base_item = find_item(list, base_node_id, err);
	if (!base_item) { cJSON_Delete(params); return err; }

	// Determine triangle source: prefer cached_mesh, then source_triangles,
	// then try to reload from the STL file.
	std::vector<sinriv::kigstudio::voxel::Triangle> bvh_triangles;

	if (!base_item->cached_mesh.empty()) {
		bvh_triangles.reserve(base_item->cached_mesh.size());
		for (const auto& [tri, _] : base_item->cached_mesh) {
			bvh_triangles.push_back(tri);
		}
	} else if (!base_item->source_triangles.empty()) {
		bvh_triangles = base_item->source_triangles;
	} else if (!base_item->stl_path.empty()) {
		// Reload from the original STL file
		for (auto [tri, n] :
		     sinriv::kigstudio::voxel::readSTL(base_item->stl_path)) {
			(void)n;
			bvh_triangles.push_back(tri);
		}
	}

	if (bvh_triangles.empty()) {
		cJSON_Delete(params);
		return error_response("NO_MESH", "base node has no mesh data");
	}

	// Clear old config and build new
	item->hair_angle_config.clear();

	// Parse optional north_pole vector (default: world +Y)
	cJSON* np_arr = cJSON_GetObjectItem(params, "north_pole");
	if (np_arr && cJSON_IsArray(np_arr)) {
		sinriv::kigstudio::voxel::vec3f np;
		if (json_to_vec3(np_arr, np)) {
			item->hair_north_pole = np.normalize();
		}
	}

	// Parse optional front_reference vector (default: world +Z)
	cJSON* fr_arr = cJSON_GetObjectItem(params, "front_reference");
	if (fr_arr && cJSON_IsArray(fr_arr)) {
		sinriv::kigstudio::voxel::vec3f fr;
		if (json_to_vec3(fr_arr, fr)) {
			item->hair_front_reference = fr.normalize();
		}
	}

	int n = cJSON_GetArraySize(angles_arr);
	for (int i = 0; i < n; ++i) {
		cJSON* entry = cJSON_GetArrayItem(angles_arr, i);
		if (!cJSON_IsObject(entry)) continue;
		float x = json_float(entry, "x", 0.0f);
		float y = json_float(entry, "y", 0.0f);
		HairAngleEntry ae;
		ae.theta = json_float(entry, "theta", 0.0f);
		ae.phi = json_float(entry, "phi", 45.0f);
		item->hair_angle_config[{x, y}] = ae;
	}

	// Build BVH tree from the resolved triangle source
	auto bvh = std::make_unique<
	    sinriv::kigstudio::voxel::triangle_bvh<float>>();
	for (const auto& tri : bvh_triangles) {
		bvh->insert(tri);
	}
	item->hair_bvh = std::move(bvh);
	item->hair_bvh_base_node_id = base_node_id;
	item->dirty = true;  // persist angle config changes
		// Mark BVH as current with respect to the base mesh state.
		// cached_mesh_dirty defaults to true and is only flipped to
		// false by the export task queue; setAngleConfig may build the
		// BVH from source_triangles (before any export), so we must
		// clear the dirty flag to prevent spurious BVH_STALE errors.
		base_item->cached_mesh_dirty = false;

	cJSON_Delete(params);

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddNumberToObject(r, "angle_count",
	                        static_cast<int>(item->hair_angle_config.size()));
	cJSON_AddNumberToObject(r, "bvh_triangle_count",
	                        static_cast<int>(bvh_triangles.size()));
	return r;
}

/// Spherical (theta, phi) in degrees → unit direction vector.
/// Uses configurable north pole and front reference to build the local frame.
/// theta=0° → "front" (V axis), +90° → "right" (U axis)
/// phi=0° → horizontal (equatorial plane), +90° → north pole
/// Default north_pole {0,1,0}, front_reference {0,0,1} produces:
///   theta=0→+Z, theta=+90→+X, phi=+90→+Y
inline sinriv::kigstudio::voxel::vec3f spherical_to_dir(
    float theta_deg,
    float phi_deg,
    const sinriv::kigstudio::voxel::vec3f& north_pole = {0.0f, 1.0f, 0.0f},
    const sinriv::kigstudio::voxel::vec3f& front_reference = {0.0f, 0.0f, 1.0f}) {
	constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
	float t = theta_deg * kDegToRad;
	float p = phi_deg * kDegToRad;
	float cos_p = std::cos(p);
	float sin_p = std::sin(p);
	float sin_t = std::sin(t);
	float cos_t = std::cos(t);

	// Build local orthonormal frame
	// N = north pole (phi=+90° direction), normalized
	sinriv::kigstudio::voxel::vec3f N = north_pole.normalize();

	// Project front_reference onto the equatorial plane → V (theta=0°)
	sinriv::kigstudio::voxel::vec3f F = front_reference.normalize();
	float f_dot_n = F.dot(N);
	sinriv::kigstudio::voxel::vec3f V = F - N * f_dot_n;
	float v_len2 = V.length2();

	// If front_reference is nearly parallel to north_pole, fall back to heuristic
	if (v_len2 < 1e-10f) {
		sinriv::kigstudio::voxel::vec3f A =
		    (std::abs(N.z) < 0.99f)
		        ? sinriv::kigstudio::voxel::vec3f(0.0f, 0.0f, 1.0f)
		        : sinriv::kigstudio::voxel::vec3f(1.0f, 0.0f, 0.0f);
		V = A - N * A.dot(N);
		v_len2 = V.length2();
	}

	V = V / std::sqrt(v_len2);  // normalize
	sinriv::kigstudio::voxel::vec3f U = cross(N, V);  // theta=+90° (right)

	return U * (sin_t * cos_p) + N * sin_p + V * (cos_t * cos_p);
}

/// Cast a ray from outside toward the center point, find closest hit on BVH.
/// Returns true and sets `out_hit` on success.
inline bool raycast_to_bvh(
    sinriv::kigstudio::voxel::triangle_bvh<float>& bvh,
    const sinriv::kigstudio::voxel::vec3f& center,
    const sinriv::kigstudio::voxel::vec3f& dir,
    sinriv::kigstudio::voxel::vec3f& out_hit) {
	constexpr float kMaxDist = 10000.0f;
	sinriv::kigstudio::ray<float> r;
	r.begin = center + dir * kMaxDist;  // far outside
	r.end = center - dir * kMaxDist;    // past center on other side

	float closest_dist = std::numeric_limits<float>::max();
	bool hit = false;

	bvh.rayTest(r, [&](auto /*node_data*/,
	                   const sinriv::kigstudio::voxel::vec3f& coll_pos) {
		float dist = (coll_pos - r.begin).length();
		if (dist < closest_dist) {
			closest_dist = dist;
			out_hit = coll_pos;
			hit = true;
		}
	});
	return hit;
}

/// Interpolate (theta, phi) for a semantic (x, y) coordinate using
/// inverse distance weighting (IDW) over all configured sample points.
/// Exact match is returned directly; for non-exact queries the weighted
/// average of all sample points is used so that the angle varies smoothly
/// across the semantic domain.
inline bool interpolate_angle_config(
    const std::map<std::pair<float, float>, HairAngleEntry>& config,
    float x, float y,
    float& out_theta, float& out_phi)
{
    if (config.empty()) return false;

    // Exact match
    auto it = config.find({x, y});
    if (it != config.end()) {
        out_theta = it->second.theta;
        out_phi   = it->second.phi;
        return true;
    }

    // Single entry → return directly (avoids division by zero)
    if (config.size() == 1) {
        auto& e = config.begin()->second;
        out_theta = e.theta;
        out_phi   = e.phi;
        return true;
    }

    // IDW with power 2: weight = 1 / distance²
    float wsum = 0.0f, tsum = 0.0f, psum = 0.0f;
    constexpr float kEps = 1e-12f;

    for (const auto& [key, entry] : config) {
        float dx = x - key.first;
        float dy = y - key.second;
        float d2 = dx * dx + dy * dy;

        if (d2 < kEps) {  // nearly on top of a sample
            out_theta = entry.theta;
            out_phi   = entry.phi;
            return true;
        }

        float w = 1.0f / d2;
        wsum += w;
        tsum += w * entry.theta;
        psum += w * entry.phi;
    }

    if (wsum < kEps) return false;
    out_theta = tsum / wsum;
    out_phi   = psum / wsum;
    return true;
}

inline cJSON* h_strand_add_semantic_guide_point(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	int strand_index = json_int(params, "strand_index", -1);
	float x = json_float(params, "x",
	                     std::numeric_limits<float>::quiet_NaN());
	float y = json_float(params, "y",
	                     std::numeric_limits<float>::quiet_NaN());

	if (node_id < 0 || strand_index < 0 || std::isnan(x) || std::isnan(y)) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS",
		                      "node_id, strand_index, x, y are required");
	}

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) { cJSON_Delete(params); return err; }
	HairStrand* strand = find_strand(item, strand_index, err);
	if (!strand) { cJSON_Delete(params); return err; }

	// Must have BVH built (via setAngleConfig)
	if (!item->hair_bvh) {
		cJSON_Delete(params);
		return error_response("NO_BVH",
		                      "BVH not built; call setAngleConfig first");
	}

	// Look up (or interpolate) angle config
	float theta, phi;
	if (!interpolate_angle_config(item->hair_angle_config, x, y, theta, phi)) {
		cJSON_Delete(params);
		return error_response("NO_ANGLE_CONFIG",
		                      "no angle configured for (x,y); "
		                      "call setAngleConfig first");
	}

	// Verify base model still valid
	if (item->hair_bvh_base_node_id >= 0) {
		auto base_it = list.items.find(item->hair_bvh_base_node_id);
		if (base_it == list.items.end() ||
		    base_it->second->cached_mesh_dirty) {
			item->hair_bvh.reset();
			item->hair_bvh_base_node_id = -1;
			cJSON_Delete(params);
			return error_response(
			    "BVH_STALE",
			    "base model changed; call setAngleConfig again");
		}
	}

	// Ray cast
	auto dir = spherical_to_dir(theta, phi, item->hair_north_pole, item->hair_front_reference);
	auto center = item->addon_center_point;

	sinriv::kigstudio::voxel::vec3f hit_point;
	if (!raycast_to_bvh(*item->hair_bvh, center, dir, hit_point)) {
		cJSON_Delete(params);
		return error_response("RAY_MISS",
		                      "ray did not hit the base model");
	}

	strand->guide_points.push_back(hit_point);
	strand->mesh_dirty = true;

	cJSON_Delete(params);

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddItemToObject(r, "point", vec3_to_json(hit_point));
	cJSON_AddNumberToObject(r, "guide_point_index",
	                        static_cast<int>(strand->guide_points.size() - 1));
	return r;
}

inline cJSON* h_strand_add_semantic_width_point(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	int strand_index = json_int(params, "strand_index", -1);
	float x = json_float(params, "x",
	                     std::numeric_limits<float>::quiet_NaN());
	float y = json_float(params, "y",
	                     std::numeric_limits<float>::quiet_NaN());
	float scale = json_float(params, "scale", 1.0f);

	if (node_id < 0 || strand_index < 0 || std::isnan(x) || std::isnan(y)) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS",
		                      "node_id, strand_index, x, y are required");
	}

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) { cJSON_Delete(params); return err; }
	HairStrand* strand = find_strand(item, strand_index, err);
	if (!strand) { cJSON_Delete(params); return err; }

	// Must have guide points to compute curve_id
	if (strand->guide_points.size() < 2) {
		cJSON_Delete(params);
		return error_response("NO_GUIDE_POINTS",
		                      "strand needs at least 2 guide points");
	}

	// Must have BVH built
	if (!item->hair_bvh) {
		cJSON_Delete(params);
		return error_response("NO_BVH",
		                      "BVH not built; call setAngleConfig first");
	}

	// Look up (or interpolate) angle config
	float theta, phi;
	if (!interpolate_angle_config(item->hair_angle_config, x, y, theta, phi)) {
		cJSON_Delete(params);
		return error_response("NO_ANGLE_CONFIG",
		                      "no angle configured for (x,y); "
		                      "call setAngleConfig first");
	}

	// Verify base model still valid
	if (item->hair_bvh_base_node_id >= 0) {
		auto base_it = list.items.find(item->hair_bvh_base_node_id);
		if (base_it == list.items.end() ||
		    base_it->second->cached_mesh_dirty) {
			item->hair_bvh.reset();
			item->hair_bvh_base_node_id = -1;
			cJSON_Delete(params);
			return error_response(
			    "BVH_STALE",
			    "base model changed; call setAngleConfig again");
		}
	}

	// Ray cast
	auto dir = spherical_to_dir(theta, phi, item->hair_north_pole, item->hair_front_reference);
	auto center = item->addon_center_point;

	sinriv::kigstudio::voxel::vec3f hit_point;
	if (!raycast_to_bvh(*item->hair_bvh, center, dir, hit_point)) {
		cJSON_Delete(params);
		return error_response("RAY_MISS",
		                      "ray did not hit the base model");
	}

	// Compute curve_id: find the closest point on the guide curve segments
	const auto& gpts = strand->guide_points;
	float best_curve_id = 0.0f;
	float best_dist_sq = std::numeric_limits<float>::max();
	sinriv::kigstudio::voxel::vec3f best_curve_point = gpts[0];

	for (size_t seg = 0; seg + 1 < gpts.size(); ++seg) {
		const auto& a = gpts[seg];
		const auto& b = gpts[seg + 1];
		auto ab = b - a;
		float ab_len_sq = ab.dot(ab);
		float t = 0.0f;
		if (ab_len_sq > 1e-12f) {
			t = std::max(0.0f, std::min(1.0f,
			                            (hit_point - a).dot(ab) / ab_len_sq));
		}
		auto proj = a + ab * t;
		float d2 = (proj - hit_point).length2();
		if (d2 < best_dist_sq) {
			best_dist_sq = d2;
			best_curve_id = static_cast<float>(seg) + t;
			best_curve_point = proj;
		}
	}

	// Direction from curve toward surface
	auto width_dir = (hit_point - best_curve_point);
	float wlen = width_dir.length();
	if (wlen > 1e-8f) {
		width_dir = width_dir * (1.0f / wlen);
	} else {
		width_dir = {0.0f, 0.0f, 1.0f};
	}

	HairStrand::WidthPoint wp;
	wp.curve_id = best_curve_id;
	wp.scale = scale;
	wp.direction = width_dir;
	strand->width_points.push_back(std::move(wp));
	strand->mesh_dirty = true;

	cJSON_Delete(params);

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddNumberToObject(r, "width_point_index",
	                        static_cast<int>(strand->width_points.size() - 1));
	cJSON* wp_json = cJSON_CreateObject();
	cJSON_AddNumberToObject(wp_json, "curve_id",
	                        static_cast<double>(best_curve_id));
	cJSON_AddNumberToObject(wp_json, "scale", static_cast<double>(scale));
	cJSON_AddItemToObject(wp_json, "direction", vec3_to_json(width_dir));
	cJSON_AddItemToObject(wp_json, "surface_point", vec3_to_json(hit_point));
	cJSON_AddItemToObject(r, "width_point", wp_json);
	return r;
}

// ===================================================================
// strand.applyHairlineSpindle
// ===================================================================

inline cJSON* h_strand_apply_hairline_spindle(cJSON* params, List& list) {
	int node_id = json_int(params, "node_id", -1);
	if (node_id < 0) {
		cJSON_Delete(params);
		return error_response("INVALID_PARAMS", "node_id is required");
	}

	float scale = json_float(params, "scale", 0.8f);

	std::lock_guard<std::mutex> lock(list.locker);
	cJSON* err = nullptr;
	Item* item = find_item(list, node_id, err);
	if (!item) { cJSON_Delete(params); return err; }

	item->hairline_spindle_scale = scale;
	item->apply_hairline_spindle();

	cJSON_Delete(params);

	cJSON* r = cJSON_CreateObject();
	cJSON_AddTrueToObject(r, "ok");
	cJSON_AddStringToObject(r, "message", "hairline spindle applied");
	return r;
}

}  // namespace sinriv::kigstudio::agent
