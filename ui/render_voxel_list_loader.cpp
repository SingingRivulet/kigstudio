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
        default:
            mode_str = "collision";
            break;
    }
    cJSON_AddStringToObject(obj, "segment_mode", mode_str);
    cJSON_AddBoolToObject(obj, "show_mesh", item.showMesh);
    cJSON_AddBoolToObject(obj, "show_voxel", item.showVoxel);
    cJSON_AddBoolToObject(obj, "show_collision", item.showCollision);
    cJSON_AddBoolToObject(obj, "show_collision_bounds",
                          item.showCollisionBounds);
    cJSON_AddBoolToObject(obj, "auto_segment_update",
                          item.auto_segment_update);
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
    } else {
        item->segment_mode = RenderVoxelItem::COLLISION;
    }
    item->showMesh = cJSON_IsTrue(cJSON_GetObjectItem(obj, "show_mesh"));
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
            if (!item->stl_path.empty()) {
                try {
                    item->mesh_renderer.loadSTL(item->stl_path);
                } catch (const std::exception& e) {
                    std::cout << "Failed to load STL mesh for item " << id
                              << ": " << e.what() << std::endl;
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
    return true;
}

}  // namespace sinriv::ui::render