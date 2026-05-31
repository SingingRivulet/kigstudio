#include <memory>
#include "render_voxel_list.h"
namespace sinriv::ui::render {

cJSON* RenderVoxelList::item_to_json(const RenderVoxelItem& item) const {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(obj, "id", item.id);
    cJSON* children = cJSON_CreateArray();
    for (int child_id : item.children) {
        cJSON_AddItemToArray(children, cJSON_CreateNumber(child_id));
    }
    cJSON_AddItemToObject(obj, "children", children);
    cJSON* nav_pos = cJSON_CreateArray();
    cJSON_AddItemToArray(nav_pos,
                         cJSON_CreateNumber(item.nav_node_position[0]));
    cJSON_AddItemToArray(nav_pos,
                         cJSON_CreateNumber(item.nav_node_position[1]));
    cJSON_AddItemToObject(obj, "nav_node_position", nav_pos);
    const char* mode_str;
    switch (item.segment_mode) {
        case RenderVoxelItem::COLLISION:
            mode_str = "collision";
            break;
        case RenderVoxelItem::PLANE:
            mode_str = "plane";
            break;
        case RenderVoxelItem::CONCAVE_CONE:
            mode_str = "concave_cone";
            break;
        case RenderVoxelItem::SPLIT_DISCONNECTED:
            mode_str = "split_disconnected";
            break;
        case RenderVoxelItem::NEIGHBOR:
            mode_str = "neighbor";
            break;
        case RenderVoxelItem::FILL_INTERIOR:
            mode_str = "fill_interior";
            break;
        case RenderVoxelItem::CHAIN:
            mode_str = "chain";
            break;
        default:
            mode_str = "collision";
            break;
    }
    cJSON_AddStringToObject(obj, "segment_mode", mode_str);
    cJSON_AddBoolToObject(obj, "show_mesh", item.showMesh);
    cJSON_AddBoolToObject(obj, "show_exported_mesh", item.showExportedMesh);
    cJSON_AddBoolToObject(obj, "show_voxel", item.showVoxel);
    cJSON_AddBoolToObject(obj, "show_collision", item.showCollision);
    cJSON_AddBoolToObject(obj, "show_collision_bounds",
                          item.showCollisionBounds);
    cJSON_AddBoolToObject(obj, "auto_segment_update",
                          item.auto_segment_update);
    cJSON_AddBoolToObject(obj, "voxel_picking_enabled",
                          item.voxel_picking_enabled);
    cJSON_AddNumberToObject(obj, "voxel_pick_range", item.voxel_pick_range);
    cJSON_AddNumberToObject(obj, "neighbor_max_distance",
                            item.neighbor_max_distance);
    cJSON_AddNumberToObject(obj, "chain_min_radius",
                            item.chain_min_radius);
    cJSON_AddBoolToObject(obj, "has_marked_voxels",
                          !item.marked_voxels.empty());
    cJSON_AddStringToObject(obj, "stl_path", item.stl_path.c_str());
    cJSON_AddStringToObject(obj, "voxel_path", item.voxel_path.c_str());
    cJSON_AddNumberToObject(obj, "stl_voxel_size", item.stl_voxel_size);
    cJSON_AddStringToObject(obj, "err_info", item.err_info.c_str());
    cJSON_AddItemToObject(obj, "collision_group",
                          sinriv::kigstudio::to_json(item.collision_group));
    cJSON_AddItemToObject(obj, "plane", sinriv::kigstudio::to_json(item.plane));
    cJSON_AddItemToObject(
        obj, "concave_cone",
        sinriv::kigstudio::voxel::concave::to_json(item.concave_cone));
    cJSON* expanded = cJSON_CreateArray();
    for (int v : item.concave_cone_expanded_vertices) {
        cJSON_AddItemToArray(expanded, cJSON_CreateNumber(v));
    }
    cJSON_AddItemToObject(obj, "concave_cone_expanded_vertices", expanded);
    cJSON_AddItemToObject(
        obj, "voxel_global_position",
        sinriv::kigstudio::to_json(item.voxel_grid_data.global_position));
    cJSON_AddItemToObject(
        obj, "voxel_size",
        sinriv::kigstudio::to_json(item.voxel_grid_data.voxel_size));
    cJSON_AddBoolToObject(obj, "use_cgal_skeleton",
                          item.use_cgal_skeleton);
    cJSON* skeleton_points = cJSON_CreateArray();
    for (const auto& sp : item.picked_skeleton_points) {
        cJSON* sp_obj = cJSON_CreateObject();
        cJSON_AddItemToObject(
            sp_obj, "position",
            sinriv::kigstudio::to_json(sp.position));
        cJSON_AddNumberToObject(sp_obj, "order", sp.order);
        cJSON_AddBoolToObject(sp_obj, "use_custom_direction",
                              sp.use_custom_direction);
        cJSON_AddItemToObject(
            sp_obj, "custom_direction_end",
            sinriv::kigstudio::to_json(sp.custom_direction_end));
        cJSON_AddNumberToObject(sp_obj, "socket_cone_offset",
                                sp.socket_cone_offset);
        cJSON_AddNumberToObject(sp_obj, "socket_cone_angle",
                                sp.socket_cone_angle);
        cJSON_AddNumberToObject(sp_obj, "socket_cone_radius",
                                sp.socket_cone_radius);
        cJSON_AddNumberToObject(sp_obj, "head_cone_offset",
                                sp.head_cone_offset);
        cJSON_AddNumberToObject(sp_obj, "head_cone_radius",
                                sp.head_cone_radius);
        cJSON_AddNumberToObject(sp_obj, "socket_support_offset",
                                sp.socket_support_offset);
        cJSON_AddNumberToObject(sp_obj, "socket_support_radius",
                                sp.socket_support_radius);
        cJSON_AddNumberToObject(sp_obj, "head_support_offset",
                                sp.head_support_offset);
        cJSON_AddNumberToObject(sp_obj, "head_support_radius",
                                sp.head_support_radius);
        cJSON_AddNumberToObject(sp_obj, "male_cylinder_offset",
                                sp.male_cylinder_offset);
        cJSON_AddNumberToObject(sp_obj, "male_cylinder_radius",
                                sp.male_cylinder_radius);
        cJSON_AddNumberToObject(sp_obj, "female_gap",
                                sp.female_gap);
        cJSON_AddNumberToObject(sp_obj, "slot_extra",
                                sp.slot_extra);
        cJSON_AddNumberToObject(sp_obj, "rotation_angle",
                                sp.rotation_angle);
        cJSON_AddItemToArray(skeleton_points, sp_obj);
    }
    cJSON_AddItemToObject(obj, "picked_skeleton_points",
                          skeleton_points);
    return obj;
}

std::unique_ptr<RenderVoxelList::RenderVoxelItem>
RenderVoxelList::item_from_json(const cJSON* obj) {
    auto item = std::make_unique<RenderVoxelItem>();
    item->manager = this;
    item->id = cJSON_GetObjectItem(obj, "id")->valueint;
    const cJSON* children = cJSON_GetObjectItem(obj, "children");
    int children_count = cJSON_GetArraySize(children);
    item->children.clear();
    for (int i = 0; i < children_count; ++i) {
        item->children.push_back(cJSON_GetArrayItem(children, i)->valueint);
    }
    const cJSON* nav_pos = cJSON_GetObjectItem(obj, "nav_node_position");
    item->nav_node_position[0] = cJSON_GetArrayItem(nav_pos, 0)->valueint;
    item->nav_node_position[1] = cJSON_GetArrayItem(nav_pos, 1)->valueint;
    const char* mode_str =
        cJSON_GetObjectItem(obj, "segment_mode")->valuestring;
    if (strcmp(mode_str, "collision") == 0) {
        item->segment_mode = RenderVoxelItem::COLLISION;
    } else if (strcmp(mode_str, "plane") == 0) {
        item->segment_mode = RenderVoxelItem::PLANE;
    } else if (strcmp(mode_str, "concave_cone") == 0) {
        item->segment_mode = RenderVoxelItem::CONCAVE_CONE;
    } else if (strcmp(mode_str, "split_disconnected") == 0) {
        item->segment_mode = RenderVoxelItem::SPLIT_DISCONNECTED;
    } else if (strcmp(mode_str, "neighbor") == 0) {
        item->segment_mode = RenderVoxelItem::NEIGHBOR;
    } else if (strcmp(mode_str, "fill_interior") == 0) {
        item->segment_mode = RenderVoxelItem::FILL_INTERIOR;
    } else if (strcmp(mode_str, "chain") == 0) {
        item->segment_mode = RenderVoxelItem::CHAIN;
    } else {
        item->segment_mode = RenderVoxelItem::COLLISION;
    }
    item->showMesh = cJSON_IsTrue(cJSON_GetObjectItem(obj, "show_mesh"));
    item->showExportedMesh =
        cJSON_IsTrue(cJSON_GetObjectItem(obj, "show_exported_mesh"));
    item->showVoxel = cJSON_IsTrue(cJSON_GetObjectItem(obj, "show_voxel"));
    item->showCollision =
        cJSON_IsTrue(cJSON_GetObjectItem(obj, "show_collision"));
    item->showCollisionBounds =
        cJSON_IsTrue(cJSON_GetObjectItem(obj, "show_collision_bounds"));
    const cJSON* auto_update_json =
        cJSON_GetObjectItem(obj, "auto_segment_update");
    if (auto_update_json) {
        item->auto_segment_update = cJSON_IsTrue(auto_update_json);
    }
    item->stl_path = cJSON_GetObjectItem(obj, "stl_path")->valuestring;
    item->voxel_path = cJSON_GetObjectItem(obj, "voxel_path")->valuestring;
    const cJSON* stl_voxel_size_json =
        cJSON_GetObjectItem(obj, "stl_voxel_size");
    if (stl_voxel_size_json) {
        item->stl_voxel_size =
            static_cast<float>(stl_voxel_size_json->valuedouble);
    }
    item->err_info = cJSON_GetObjectItem(obj, "err_info")->valuestring;
    item->collision_group = sinriv::kigstudio::from_json_collision_group(
        cJSON_GetObjectItem(obj, "collision_group"));
    item->plane =
        sinriv::kigstudio::from_json_plane(cJSON_GetObjectItem(obj, "plane"));
    item->concave_cone = sinriv::kigstudio::voxel::concave::from_json_cone(
        cJSON_GetObjectItem(obj, "concave_cone"));
    const cJSON* expanded =
        cJSON_GetObjectItem(obj, "concave_cone_expanded_vertices");
    int expanded_count = cJSON_GetArraySize(expanded);
    for (int i = 0; i < expanded_count; ++i) {
        item->concave_cone_expanded_vertices.push_back(
            cJSON_GetArrayItem(expanded, i)->valueint);
    }
    item->voxel_grid_data.global_position =
        sinriv::kigstudio::vec3_from_json<sinriv::kigstudio::vec3<float>>(
            cJSON_GetObjectItem(obj, "voxel_global_position"));
    item->voxel_grid_data.voxel_size =
        sinriv::kigstudio::vec3_from_json<sinriv::kigstudio::vec3<float>>(
            cJSON_GetObjectItem(obj, "voxel_size"));

    const cJSON* voxel_picking_json =
        cJSON_GetObjectItem(obj, "voxel_picking_enabled");
    if (voxel_picking_json) {
        item->voxel_picking_enabled = cJSON_IsTrue(voxel_picking_json);
    }
    const cJSON* pick_range_json =
        cJSON_GetObjectItem(obj, "voxel_pick_range");
    if (pick_range_json) {
        item->voxel_pick_range =
            static_cast<float>(pick_range_json->valuedouble);
    }
    const cJSON* neighbor_dist_json =
        cJSON_GetObjectItem(obj, "neighbor_max_distance");
    if (neighbor_dist_json) {
        item->neighbor_max_distance = neighbor_dist_json->valueint;
    }
    const cJSON* chain_min_radius_json =
        cJSON_GetObjectItem(obj, "chain_min_radius");
    if (chain_min_radius_json) {
        item->chain_min_radius = chain_min_radius_json->valueint;
    }
    const cJSON* use_cgal_json =
        cJSON_GetObjectItem(obj, "use_cgal_skeleton");
    if (use_cgal_json) {
        item->use_cgal_skeleton = cJSON_IsTrue(use_cgal_json);
    }
    const cJSON* skeleton_points =
        cJSON_GetObjectItem(obj, "picked_skeleton_points");
    if (skeleton_points) {
        int sp_count = cJSON_GetArraySize(skeleton_points);
        for (int i = 0; i < sp_count; ++i) {
            const cJSON* sp_obj =
                cJSON_GetArrayItem(skeleton_points, i);
            SkeletonPointPick sp;
            sp.position =
                sinriv::kigstudio::vec3_from_json<
                    sinriv::kigstudio::voxel::vec3f>(
                    cJSON_GetObjectItem(sp_obj, "position"));
            cJSON* order_json =
                cJSON_GetObjectItem(sp_obj, "order");
            if (order_json)
                sp.order = order_json->valueint;
            cJSON* use_custom_dir_json = cJSON_GetObjectItem(
                sp_obj, "use_custom_direction");
            if (use_custom_dir_json)
                sp.use_custom_direction =
                    cJSON_IsTrue(use_custom_dir_json);
            sp.custom_direction_end =
                sinriv::kigstudio::vec3_from_json<
                    sinriv::kigstudio::voxel::vec3f>(
                    cJSON_GetObjectItem(sp_obj,
                                        "custom_direction_end"));
            cJSON* socket_cone_offset_json = cJSON_GetObjectItem(
                sp_obj, "socket_cone_offset");
            if (socket_cone_offset_json)
                sp.socket_cone_offset = static_cast<float>(
                    socket_cone_offset_json->valuedouble);
            cJSON* socket_cone_angle_json = cJSON_GetObjectItem(
                sp_obj, "socket_cone_angle");
            if (socket_cone_angle_json)
                sp.socket_cone_angle = static_cast<float>(
                    socket_cone_angle_json->valuedouble);
            cJSON* socket_cone_radius_json = cJSON_GetObjectItem(
                sp_obj, "socket_cone_radius");
            if (socket_cone_radius_json)
                sp.socket_cone_radius = static_cast<float>(
                    socket_cone_radius_json->valuedouble);
            cJSON* head_cone_offset_json = cJSON_GetObjectItem(
                sp_obj, "head_cone_offset");
            if (head_cone_offset_json)
                sp.head_cone_offset = static_cast<float>(
                    head_cone_offset_json->valuedouble);
            cJSON* head_cone_radius_json = cJSON_GetObjectItem(
                sp_obj, "head_cone_radius");
            if (head_cone_radius_json)
                sp.head_cone_radius = static_cast<float>(
                    head_cone_radius_json->valuedouble);
            cJSON* socket_support_offset_json =
                cJSON_GetObjectItem(sp_obj,
                                    "socket_support_offset");
            if (socket_support_offset_json)
                sp.socket_support_offset = static_cast<float>(
                    socket_support_offset_json->valuedouble);
            cJSON* socket_support_radius_json =
                cJSON_GetObjectItem(sp_obj,
                                    "socket_support_radius");
            if (socket_support_radius_json)
                sp.socket_support_radius = static_cast<float>(
                    socket_support_radius_json->valuedouble);
            cJSON* head_support_offset_json = cJSON_GetObjectItem(
                sp_obj, "head_support_offset");
            if (head_support_offset_json)
                sp.head_support_offset = static_cast<float>(
                    head_support_offset_json->valuedouble);
            cJSON* head_support_radius_json = cJSON_GetObjectItem(
                sp_obj, "head_support_radius");
            if (head_support_radius_json)
                sp.head_support_radius = static_cast<float>(
                    head_support_radius_json->valuedouble);
            cJSON* male_cylinder_offset_json =
                cJSON_GetObjectItem(sp_obj,
                                    "male_cylinder_offset");
            if (male_cylinder_offset_json)
                sp.male_cylinder_offset = static_cast<float>(
                    male_cylinder_offset_json->valuedouble);
            cJSON* male_cylinder_radius_json =
                cJSON_GetObjectItem(sp_obj,
                                    "male_cylinder_radius");
            if (male_cylinder_radius_json)
                sp.male_cylinder_radius = static_cast<float>(
                    male_cylinder_radius_json->valuedouble);
            cJSON* female_gap_json =
                cJSON_GetObjectItem(sp_obj, "female_gap");
            if (female_gap_json)
                sp.female_gap = static_cast<float>(
                    female_gap_json->valuedouble);
            cJSON* slot_extra_json =
                cJSON_GetObjectItem(sp_obj, "slot_extra");
            if (slot_extra_json)
                sp.slot_extra = static_cast<float>(
                    slot_extra_json->valuedouble);
            cJSON* rotation_angle_json =
                cJSON_GetObjectItem(sp_obj, "rotation_angle");
            if (rotation_angle_json)
                sp.rotation_angle = static_cast<float>(
                    rotation_angle_json->valuedouble);
            item->picked_skeleton_points.push_back(sp);
        }
    }
    return item;
}

bool RenderVoxelList::save_current_project() {
    if (project_path.empty()) {
        last_save_error = "no project path set";
        return false;
    }
    return save_project(project_path);
}

bool RenderVoxelList::save_project(const std::string& folder) {
    last_save_error.clear();
    std::filesystem::path dir = utf8_path(folder);
    try {
        std::filesystem::create_directories(dir / "voxels");
        std::filesystem::create_directories(dir / "marked");
    } catch (const std::exception& e) {
        last_save_error = std::string("create_directories failed: ") + e.what();
        return false;
    }

    std::lock_guard<std::mutex> lock(locker);
    cJSON* root = cJSON_CreateObject();
    if (!root) {
        last_save_error = "cJSON_CreateObject failed";
        return false;
    }
    cJSON_AddNumberToObject(root, "version", 1);
    cJSON_AddNumberToObject(root, "current_id", current_id.load());
    cJSON* arr = cJSON_CreateArray();
    for (const auto& [id, item] : items) {
        cJSON* item_json = item_to_json(*item);
        if (!item_json) {
            last_save_error =
                "item_to_json failed for item id=" + std::to_string(id);
            cJSON_Delete(root);
            return false;
        }
        cJSON_AddItemToArray(arr, item_json);
        std::filesystem::path voxel_path =
            dir / "voxels" / (std::to_string(id) + ".vxgrid");
        if (!std::filesystem::exists(dir / "voxels")) {
            last_save_error =
                "voxels directory not found: " + path_to_utf8(dir / "voxels");
            cJSON_Delete(root);
            return false;
        }
        std::string voxel_error;
        if (!sinriv::kigstudio::save(voxel_path, item->voxel_grid_data,
                                     &voxel_error)) {
            last_save_error = "save voxel failed: " + path_to_utf8(voxel_path) +
                              " (" + voxel_error + ")";
            cJSON_Delete(root);
            return false;
        }
        if (!item->marked_voxels.empty()) {
            std::filesystem::path marked_path =
                dir / "marked" / (std::to_string(id) + ".vxgrid");
            std::string marked_error;
            if (!sinriv::kigstudio::save(marked_path, item->marked_voxels,
                                         &marked_error)) {
                last_save_error = "save marked failed: " +
                                  path_to_utf8(marked_path) + " (" +
                                  marked_error + ")";
                cJSON_Delete(root);
                return false;
            }
        }
    }
    cJSON_AddItemToObject(root, "items", arr);

    std::filesystem::path json_path = dir / "project.json";
    char* json_str = cJSON_Print(root);
    if (!json_str) {
        last_save_error = "cJSON_Print failed";
        cJSON_Delete(root);
        return false;
    }
#ifdef _WIN32
    std::ofstream ofs(json_path.wstring().c_str());
#else
    std::ofstream ofs(json_path.c_str());
#endif
    if (!ofs) {
        last_save_error = "failed to open project.json for writing: " +
                          path_to_utf8(json_path);
        cJSON_free(json_str);
        cJSON_Delete(root);
        return false;
    }
    const char utf8_bom[] = "\xEF\xBB\xBF";
    ofs.write(utf8_bom, 3);
    ofs << json_str;
    bool ok = ofs.good();
    if (!ok) {
        last_save_error = "failed to write project.json";
    }
    cJSON_free(json_str);
    cJSON_Delete(root);
    if (ok)
        clear_all_dirty();
    return ok;
}

bool RenderVoxelList::load_project(const std::string& folder) {
    last_load_error.clear();
    release();
    start_thread();
    initIcons();
    current_id = 0;

    std::filesystem::path dir = utf8_path(folder);
    std::filesystem::path json_path = dir / "project.json";
#ifdef _WIN32
    std::ifstream ifs(json_path.wstring().c_str());
#else
    std::ifstream ifs(json_path.c_str());
#endif
    if (!ifs) {
        last_load_error =
            "failed to open project.json: " + path_to_utf8(json_path);
        return false;
    }
    std::string json_str((std::istreambuf_iterator<char>(ifs)),
                         std::istreambuf_iterator<char>());
    if (json_str.size() >= 3 &&
        static_cast<unsigned char>(json_str[0]) == 0xEF &&
        static_cast<unsigned char>(json_str[1]) == 0xBB &&
        static_cast<unsigned char>(json_str[2]) == 0xBF) {
        json_str.erase(0, 3);
    }
    cJSON* root = cJSON_Parse(json_str.c_str());
    if (!root) {
        last_load_error = "cJSON_Parse failed";
        return false;
    }

    cJSON* version_obj = cJSON_GetObjectItem(root, "version");
    if (!version_obj) {
        last_load_error = "missing 'version' field";
        cJSON_Delete(root);
        return false;
    }
    int version = version_obj->valueint;
    if (version != 1) {
        last_load_error = "unsupported version: " + std::to_string(version);
        cJSON_Delete(root);
        return false;
    }

    cJSON* current_id_obj = cJSON_GetObjectItem(root, "current_id");
    if (!current_id_obj) {
        last_load_error = "missing 'current_id' field";
        cJSON_Delete(root);
        return false;
    }
    current_id = current_id_obj->valueint;

    cJSON* arr = cJSON_GetObjectItem(root, "items");
    if (!arr) {
        last_load_error = "missing 'items' field";
        cJSON_Delete(root);
        return false;
    }
    int count = cJSON_GetArraySize(arr);
    {
        std::lock_guard<std::mutex> lock(locker);
        for (int i = 0; i < count; ++i) {
            const cJSON* item_obj = cJSON_GetArrayItem(arr, i);
            auto item = item_from_json(item_obj);
            int id = item->id;
            std::filesystem::path voxel_path =
                dir / "voxels" / (std::to_string(id) + ".vxgrid");
            if (!sinriv::kigstudio::load(voxel_path, item->voxel_grid_data)) {
                last_load_error = "load voxel failed: " + voxel_path.string();
                cJSON_Delete(root);
                return false;
            }
            if (item->voxel_grid_data.num_chunk() > 0) {
                item->voxel_renderer.loadVoxelGridChunked(item->voxel_grid_data,
                                                           0.5, true);
            }
            if (!item->stl_path.empty()) {
                try {
                    std::cout << "Loading STL mesh for item " << id
                              << " from path: " << item->stl_path << std::endl;
                    auto stl_path = utf8_path(item->stl_path);
                    std::string utf8_stl_path = path_to_utf8(stl_path);
                    std::cout << "STL path after conversion: " << utf8_stl_path
                              << std::endl;
                    item->mesh_renderer.loadSTL(utf8_stl_path);
                    item->stl_path = utf8_stl_path;
                    item->source_triangles.clear();
                    for (auto [tri, n] : sinriv::kigstudio::voxel::readSTL(utf8_stl_path)) {
                        (void)n;
                        item->source_triangles.push_back(tri);
                    }
                } catch (const std::exception& e) {
                    std::cout << "Failed to load STL mesh for item " << id
                              << ": " << e.what() << std::endl;
                }
            }
            const cJSON* has_marked =
                cJSON_GetObjectItem(item_obj, "has_marked_voxels");
            if (cJSON_IsTrue(has_marked)) {
                std::filesystem::path marked_path =
                    dir / "marked" / (std::to_string(id) + ".vxgrid");
                if (std::filesystem::exists(marked_path)) {
                    if (!sinriv::kigstudio::load(marked_path,
                                                 item->marked_voxels)) {
                        std::cout << "Failed to load marked voxels for item "
                                  << id << std::endl;
                    } else {
                        item->marked_voxels.global_position = item->voxel_grid_data.global_position;
                        item->marked_voxels.voxel_size = item->voxel_grid_data.voxel_size;
                        item->marked_voxels_dirty = true;
                    }
                }
            }
            items[id] = std::move(item);
        }
    }

    cJSON_Delete(root);
    if (!items.empty()) {
        render_id = items.begin()->first;
    }
    project_path = folder;
    update_nav_node_status = true;
    clear_all_dirty();
    object_editor_tab = 0;
    return true;
}

}  // namespace sinriv::ui::render