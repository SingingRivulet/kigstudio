#include <memory>
#include <optional>
#include <queue>
#include <unordered_map>
#include "render_voxel_list.h"
#include "kigstudio/sdf/sdf_mesh.h"
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
    // 力导向下保存当前浮点位置（同步到整型快照以保持兼容）
    int save_x = this->nav_layout_force_directed
                     ? static_cast<int>(item.nav_layout_pos[0])
                     : item.nav_node_position[0];
    int save_y = this->nav_layout_force_directed
                     ? static_cast<int>(item.nav_layout_pos[1])
                     : item.nav_node_position[1];
    cJSON_AddItemToArray(nav_pos, cJSON_CreateNumber(save_x));
    cJSON_AddItemToArray(nav_pos, cJSON_CreateNumber(save_y));
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
        case RenderVoxelItem::SDF_NODE_SPLIT:
            mode_str = "sdf_node_split";
            break;
        default:
            mode_str = "collision";
            break;
    }
    cJSON_AddStringToObject(obj, "segment_mode", mode_str);
    cJSON_AddNumberToObject(obj, "sdf_split_target_id",
                            item.sdf_split_target_id);
    cJSON_AddItemToObject(
        obj, "sdf_split_translation",
        sinriv::kigstudio::to_json(item.sdf_split_translation));
    cJSON_AddItemToObject(
        obj, "sdf_split_rotation",
        sinriv::kigstudio::to_json(item.sdf_split_rotation));
    cJSON_AddItemToObject(
        obj, "sdf_split_scale",
        sinriv::kigstudio::to_json(item.sdf_split_scale));
    cJSON_AddBoolToObject(obj, "show_origin_mesh", item.showOriginMesh);
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
    cJSON_AddNumberToObject(obj, "stl_load_mode", item.stl_load_mode);
    cJSON_AddBoolToObject(obj, "load_as_sdf", item.load_as_sdf);
    cJSON_AddBoolToObject(obj, "use_precise_voxelization",
                          item.use_precise_voxelization);
    cJSON_AddBoolToObject(obj, "mesh_only", item.mesh_only);
    cJSON_AddNumberToObject(obj, "source_type", item.source_type);
    cJSON_AddNumberToObject(obj, "source_node_id", item.source_node_id);
    cJSON_AddNumberToObject(obj, "node_source_data_type",
                            item.node_source_data_type);
    cJSON_AddNumberToObject(obj, "node_source_sdf_subdivisions",
                            item.node_source_sdf_subdivisions);
    cJSON_AddBoolToObject(obj, "node_source_sdf_simplify",
                          item.node_source_sdf_simplify);
    cJSON_AddNumberToObject(obj, "node_source_sdf_simplify_ratio",
                            item.node_source_sdf_simplify_ratio);
    cJSON_AddItemToObject(
        obj, "silhouette_center",
        sinriv::kigstudio::to_json(item.silhouette_center));
    cJSON_AddBoolToObject(obj, "show_silhouette_center",
                          item.showSilhouetteCenter);
    cJSON_AddStringToObject(obj, "err_info", item.err_info.c_str());
    cJSON_AddStringToObject(obj, "title", item.title.c_str());
    cJSON_AddStringToObject(obj, "comment_text", item.comment_text.c_str());
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
        cJSON_AddNumberToObject(sp_obj, "socket_fillet_radius",
                                sp.socket_fillet_radius);
        cJSON_AddNumberToObject(sp_obj, "socket_fillet_height",
                                sp.socket_fillet_height);
        cJSON_AddNumberToObject(sp_obj, "socket_fillet_offset",
                                sp.socket_fillet_offset);
        cJSON_AddNumberToObject(sp_obj, "head_fillet_height",
                                sp.head_fillet_height);
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
    if (!obj || !cJSON_IsObject(obj))
        return nullptr;

    auto item = std::make_unique<RenderVoxelItem>();
    item->manager = this;

    // 与旧实现保持一致的缺失默认值
    item->sdf_split_target_id = -1;
    item->showOriginMesh = false;
    item->showMesh = true;
    item->showExportedMesh = true;
    item->showVoxel = true;
    item->showCollision = true;
    item->showCollisionBounds = true;
    item->use_precise_voxelization = true;

    auto parse_skeleton_point = [](const cJSON* sp_obj) -> SkeletonPointPick {
        SkeletonPointPick sp;
        const cJSON* child = nullptr;
        cJSON_ArrayForEach(child, sp_obj) {
            if (!child->string)
                continue;
            if (cJSON_IsObject(child)) {
                if (strcmp(child->string, "position") == 0) {
                    sp.position = sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::voxel::vec3f>(child);
                } else if (strcmp(child->string, "custom_direction_end") == 0) {
                    sp.custom_direction_end = sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::voxel::vec3f>(child);
                }
            } else if (cJSON_IsNumber(child)) {
                const double value = cJSON_GetNumberValue(child);
                if (strcmp(child->string, "order") == 0) {
                    sp.order = static_cast<int>(value);
                } else if (strcmp(child->string, "socket_cone_offset") == 0) {
                    sp.socket_cone_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_cone_angle") == 0) {
                    sp.socket_cone_angle = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_cone_radius") == 0) {
                    sp.socket_cone_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "head_cone_offset") == 0) {
                    sp.head_cone_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "head_cone_radius") == 0) {
                    sp.head_cone_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_support_offset") == 0) {
                    sp.socket_support_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_support_radius") == 0) {
                    sp.socket_support_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "head_support_offset") == 0) {
                    sp.head_support_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "head_support_radius") == 0) {
                    sp.head_support_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "male_cylinder_offset") == 0) {
                    sp.male_cylinder_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "male_cylinder_radius") == 0) {
                    sp.male_cylinder_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "female_gap") == 0) {
                    sp.female_gap = static_cast<float>(value);
                } else if (strcmp(child->string, "slot_extra") == 0) {
                    sp.slot_extra = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_fillet_radius") == 0) {
                    sp.socket_fillet_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_fillet_height") == 0) {
                    sp.socket_fillet_height = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_fillet_offset") == 0) {
                    sp.socket_fillet_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "head_fillet_height") == 0) {
                    sp.head_fillet_height = static_cast<float>(value);
                } else if (strcmp(child->string, "rotation_angle") == 0) {
                    sp.rotation_angle = static_cast<float>(value);
                }
            } else if (cJSON_IsBool(child)) {
                if (strcmp(child->string, "use_custom_direction") == 0) {
                    sp.use_custom_direction = cJSON_IsTrue(child);
                }
            }
        }
        return sp;
    };

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, obj) {
        if (!child->string)
            continue;

        const char* key = child->string;

        if (strcmp(key, "id") == 0 && cJSON_IsNumber(child)) {
            item->id = child->valueint;
        } else if (strcmp(key, "children") == 0 && cJSON_IsArray(child)) {
            int children_count = cJSON_GetArraySize(child);
            item->children.clear();
            for (int i = 0; i < children_count; ++i) {
                const cJSON* c = cJSON_GetArrayItem(child, i);
                if (c && cJSON_IsNumber(c))
                    item->children.push_back(c->valueint);
            }
        } else if (strcmp(key, "nav_node_position") == 0 &&
                   cJSON_IsArray(child)) {
            if (cJSON_GetArraySize(child) >= 2) {
                const cJSON* x = cJSON_GetArrayItem(child, 0);
                const cJSON* y = cJSON_GetArrayItem(child, 1);
                if (x && cJSON_IsNumber(x) && y && cJSON_IsNumber(y)) {
                    item->nav_node_position[0] = x->valueint;
                    item->nav_node_position[1] = y->valueint;
                }
            }
        } else if (strcmp(key, "segment_mode") == 0 &&
                   cJSON_IsString(child)) {
            const char* mode_str = child->valuestring;
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
            } else if (strcmp(mode_str, "sdf_node_split") == 0) {
                item->segment_mode = RenderVoxelItem::SDF_NODE_SPLIT;
            } else {
                item->segment_mode = RenderVoxelItem::COLLISION;
            }
        } else if (cJSON_IsNumber(child)) {
            const double value = cJSON_GetNumberValue(child);
            if (strcmp(key, "sdf_split_target_id") == 0) {
                item->sdf_split_target_id = child->valueint;
            } else if (strcmp(key, "voxel_pick_range") == 0) {
                item->voxel_pick_range = static_cast<float>(value);
            } else if (strcmp(key, "neighbor_max_distance") == 0) {
                item->neighbor_max_distance = child->valueint;
            } else if (strcmp(key, "chain_min_radius") == 0) {
                item->chain_min_radius = child->valueint;
            } else if (strcmp(key, "stl_voxel_size") == 0) {
                item->stl_voxel_size = static_cast<float>(value);
            } else if (strcmp(key, "stl_load_mode") == 0) {
                item->stl_load_mode = child->valueint;
            } else if (strcmp(key, "source_type") == 0) {
                item->source_type = child->valueint;
            } else if (strcmp(key, "source_node_id") == 0) {
                item->source_node_id = child->valueint;
            } else if (strcmp(key, "node_source_data_type") == 0) {
                item->node_source_data_type = child->valueint;
            } else if (strcmp(key, "node_source_sdf_subdivisions") == 0) {
                item->node_source_sdf_subdivisions = child->valueint;
            } else if (strcmp(key, "node_source_sdf_simplify_ratio") == 0) {
                item->node_source_sdf_simplify_ratio =
                    static_cast<float>(value);
            }
        } else if (cJSON_IsBool(child)) {
            if (strcmp(key, "show_origin_mesh") == 0) {
                item->showOriginMesh = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_mesh") == 0) {
                item->showMesh = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_exported_mesh") == 0) {
                item->showExportedMesh = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_voxel") == 0) {
                item->showVoxel = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_collision") == 0) {
                item->showCollision = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_collision_bounds") == 0) {
                item->showCollisionBounds = cJSON_IsTrue(child);
            } else if (strcmp(key, "auto_segment_update") == 0) {
                item->auto_segment_update = cJSON_IsTrue(child);
            } else if (strcmp(key, "load_as_sdf") == 0) {
                item->load_as_sdf = cJSON_IsTrue(child);
            } else if (strcmp(key, "use_precise_voxelization") == 0) {
                item->use_precise_voxelization = cJSON_IsTrue(child);
            } else if (strcmp(key, "mesh_only") == 0) {
                item->mesh_only = cJSON_IsTrue(child);
            } else if (strcmp(key, "node_source_sdf_simplify") == 0) {
                item->node_source_sdf_simplify = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_silhouette_center") == 0) {
                item->showSilhouetteCenter = cJSON_IsTrue(child);
            } else if (strcmp(key, "voxel_picking_enabled") == 0) {
                item->voxel_picking_enabled = cJSON_IsTrue(child);
            } else if (strcmp(key, "use_cgal_skeleton") == 0) {
                item->use_cgal_skeleton = cJSON_IsTrue(child);
            }
        } else if (cJSON_IsString(child)) {
            if (strcmp(key, "stl_path") == 0) {
                item->stl_path = child->valuestring;
            } else if (strcmp(key, "voxel_path") == 0) {
                item->voxel_path = child->valuestring;
            } else if (strcmp(key, "err_info") == 0) {
                item->err_info = child->valuestring;
            } else if (strcmp(key, "title") == 0) {
                item->title = child->valuestring;
            } else if (strcmp(key, "comment_text") == 0) {
                item->comment_text = child->valuestring;
            }
        } else if (cJSON_IsObject(child)) {
            if (strcmp(key, "sdf_split_translation") == 0) {
                item->sdf_split_translation =
                    sinriv::kigstudio::vec3_from_json<vec3f>(child);
            } else if (strcmp(key, "sdf_split_rotation") == 0) {
                item->sdf_split_rotation =
                    sinriv::kigstudio::vec3_from_json<vec3f>(child);
            } else if (strcmp(key, "sdf_split_scale") == 0) {
                item->sdf_split_scale =
                    sinriv::kigstudio::vec3_from_json<vec3f>(child);
            } else if (strcmp(key, "silhouette_center") == 0) {
                item->silhouette_center =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "voxel_global_position") == 0) {
                item->voxel_grid_data.global_position =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "voxel_size") == 0) {
                item->voxel_grid_data.voxel_size =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            } else if (strcmp(key, "collision_group") == 0) {
                item->collision_group =
                    sinriv::kigstudio::from_json_collision_group(child);
            } else if (strcmp(key, "plane") == 0) {
                item->plane = sinriv::kigstudio::from_json_plane(child);
            } else if (strcmp(key, "concave_cone") == 0) {
                item->concave_cone =
                    sinriv::kigstudio::voxel::concave::from_json_cone(child);
            }
        } else if (cJSON_IsArray(child)) {
            if (strcmp(key, "concave_cone_expanded_vertices") == 0) {
                int expanded_count = cJSON_GetArraySize(child);
                for (int i = 0; i < expanded_count; ++i) {
                    const cJSON* v = cJSON_GetArrayItem(child, i);
                    if (v && cJSON_IsNumber(v))
                        item->concave_cone_expanded_vertices.push_back(
                            v->valueint);
                }
            } else if (strcmp(key, "picked_skeleton_points") == 0) {
                int sp_count = cJSON_GetArraySize(child);
                for (int i = 0; i < sp_count; ++i) {
                    const cJSON* sp_obj = cJSON_GetArrayItem(child, i);
                    if (sp_obj && cJSON_IsObject(sp_obj)) {
                        item->picked_skeleton_points.push_back(
                            parse_skeleton_point(sp_obj));
                    }
                }
            }
        }
    }

    // nav_layout_pos 与 nav_node_position 同轴
    item->nav_layout_pos[0] = (float)item->nav_node_position[0];
    item->nav_layout_pos[1] = (float)item->nav_node_position[1];
    item->nav_layout_vel[0] = 0.0f;
    item->nav_layout_vel[1] = 0.0f;
    item->nav_layout_pinned = false;
    item->nav_layout_pos_set = true;

    return item;
}

cJSON* RenderVoxelList::snapshot_to_json(
    const CollisionEditorSnapshot& snapshot) const {
    cJSON* obj = cJSON_CreateObject();

    const char* mode_str;
    switch (static_cast<RenderVoxelItem::SegmentMode>(snapshot.segment_mode)) {
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
        case RenderVoxelItem::SDF_NODE_SPLIT:
            mode_str = "sdf_node_split";
            break;
        default:
            mode_str = "collision";
            break;
    }
    cJSON_AddStringToObject(obj, "segment_mode", mode_str);

    // 公共源数据配置（各模式通用）
    cJSON_AddStringToObject(obj, "stl_path", snapshot.stl_path.c_str());
    cJSON_AddNumberToObject(obj, "stl_load_mode", snapshot.stl_load_mode);
    cJSON_AddBoolToObject(obj, "load_as_sdf", snapshot.load_as_sdf);
    cJSON_AddBoolToObject(obj, "use_precise_voxelization",
                          snapshot.use_precise_voxelization);
    cJSON_AddBoolToObject(obj, "mesh_only", snapshot.mesh_only);
    cJSON_AddNumberToObject(obj, "source_type", snapshot.source_type);
    cJSON_AddNumberToObject(obj, "source_node_id", snapshot.source_node_id);
    cJSON_AddNumberToObject(obj, "node_source_data_type",
                            snapshot.node_source_data_type);
    cJSON_AddNumberToObject(obj, "node_source_sdf_subdivisions",
                            snapshot.node_source_sdf_subdivisions);
    cJSON_AddBoolToObject(obj, "node_source_sdf_simplify",
                          snapshot.node_source_sdf_simplify);
    cJSON_AddNumberToObject(obj, "node_source_sdf_simplify_ratio",
                            snapshot.node_source_sdf_simplify_ratio);
    cJSON_AddItemToObject(
        obj, "silhouette_center",
        sinriv::kigstudio::to_json(snapshot.silhouette_center));
    cJSON_AddBoolToObject(obj, "show_silhouette_center",
                          snapshot.show_silhouette_center);

    // 仅输出当前 segment_mode 相关的编辑字段
    const auto mode =
        static_cast<RenderVoxelItem::SegmentMode>(snapshot.segment_mode);
    if (mode == RenderVoxelItem::COLLISION) {
        cJSON_AddItemToObject(
            obj, "collision_group",
            sinriv::kigstudio::to_json(snapshot.collision_group));
    } else if (mode == RenderVoxelItem::PLANE) {
        cJSON_AddItemToObject(obj, "plane",
                              sinriv::kigstudio::to_json(snapshot.plane));
    } else if (mode == RenderVoxelItem::CONCAVE_CONE) {
        cJSON_AddItemToObject(
            obj, "concave_cone",
            sinriv::kigstudio::voxel::concave::to_json(snapshot.concave_cone));
        cJSON* expanded = cJSON_CreateArray();
        for (int v : snapshot.concave_cone_expanded_vertices) {
            cJSON_AddItemToArray(expanded, cJSON_CreateNumber(v));
        }
        cJSON_AddItemToObject(obj, "concave_cone_expanded_vertices", expanded);
    } else if (mode == RenderVoxelItem::SDF_NODE_SPLIT) {
        cJSON_AddNumberToObject(obj, "sdf_split_target_id",
                                snapshot.sdf_split_target_id);
        cJSON_AddItemToObject(
            obj, "sdf_split_translation",
            sinriv::kigstudio::to_json(snapshot.sdf_split_translation));
        cJSON_AddItemToObject(
            obj, "sdf_split_rotation",
            sinriv::kigstudio::to_json(snapshot.sdf_split_rotation));
        cJSON_AddItemToObject(
            obj, "sdf_split_scale",
            sinriv::kigstudio::to_json(snapshot.sdf_split_scale));
    } else if (mode == RenderVoxelItem::CHAIN) {
        cJSON_AddNumberToObject(obj, "chain_min_radius",
                                snapshot.chain_min_radius);
        cJSON_AddBoolToObject(obj, "use_cgal_skeleton",
                              snapshot.use_cgal_skeleton);

        cJSON* skeleton_points = cJSON_CreateArray();
        for (const auto& sp : snapshot.picked_skeleton_points) {
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
            cJSON_AddNumberToObject(sp_obj, "female_gap", sp.female_gap);
            cJSON_AddNumberToObject(sp_obj, "slot_extra", sp.slot_extra);
            cJSON_AddNumberToObject(sp_obj, "socket_fillet_radius",
                                    sp.socket_fillet_radius);
            cJSON_AddNumberToObject(sp_obj, "socket_fillet_height",
                                    sp.socket_fillet_height);
            cJSON_AddNumberToObject(sp_obj, "socket_fillet_offset",
                                    sp.socket_fillet_offset);
            cJSON_AddNumberToObject(sp_obj, "head_fillet_height",
                                    sp.head_fillet_height);
            cJSON_AddNumberToObject(sp_obj, "rotation_angle",
                                    sp.rotation_angle);
            cJSON_AddItemToArray(skeleton_points, sp_obj);
        }
        cJSON_AddItemToObject(obj, "picked_skeleton_points", skeleton_points);

        cJSON* skeleton_lines = cJSON_CreateArray();
        for (const auto& line : snapshot.skeleton_lines) {
            cJSON* line_obj = cJSON_CreateObject();
            cJSON_AddItemToObject(line_obj, "start",
                                  sinriv::kigstudio::to_json(line.first));
            cJSON_AddItemToObject(line_obj, "end",
                                  sinriv::kigstudio::to_json(line.second));
            cJSON_AddItemToArray(skeleton_lines, line_obj);
        }
        cJSON_AddItemToObject(obj, "skeleton_lines", skeleton_lines);
    }

    return obj;
}

std::optional<CollisionEditorSnapshot> RenderVoxelList::snapshot_from_json(
    const cJSON* obj) const {
    if (!obj || !cJSON_IsObject(obj))
        return std::nullopt;

    CollisionEditorSnapshot snapshot;
    snapshot.segment_mode = RenderVoxelItem::COLLISION;

    auto parse_skeleton_point = [](const cJSON* sp_obj) -> SkeletonPointPick {
        SkeletonPointPick sp;
        const cJSON* child = nullptr;
        cJSON_ArrayForEach(child, sp_obj) {
            if (!child->string)
                continue;
            if (cJSON_IsObject(child)) {
                if (strcmp(child->string, "position") == 0) {
                    sp.position = sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::voxel::vec3f>(child);
                } else if (strcmp(child->string, "custom_direction_end") == 0) {
                    sp.custom_direction_end = sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::voxel::vec3f>(child);
                }
            } else if (cJSON_IsNumber(child)) {
                const double value = cJSON_GetNumberValue(child);
                if (strcmp(child->string, "order") == 0) {
                    sp.order = static_cast<int>(value);
                } else if (strcmp(child->string, "socket_cone_offset") == 0) {
                    sp.socket_cone_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_cone_angle") == 0) {
                    sp.socket_cone_angle = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_cone_radius") == 0) {
                    sp.socket_cone_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "head_cone_offset") == 0) {
                    sp.head_cone_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "head_cone_radius") == 0) {
                    sp.head_cone_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_support_offset") == 0) {
                    sp.socket_support_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_support_radius") == 0) {
                    sp.socket_support_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "head_support_offset") == 0) {
                    sp.head_support_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "head_support_radius") == 0) {
                    sp.head_support_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "male_cylinder_offset") == 0) {
                    sp.male_cylinder_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "male_cylinder_radius") == 0) {
                    sp.male_cylinder_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "female_gap") == 0) {
                    sp.female_gap = static_cast<float>(value);
                } else if (strcmp(child->string, "slot_extra") == 0) {
                    sp.slot_extra = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_fillet_radius") == 0) {
                    sp.socket_fillet_radius = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_fillet_height") == 0) {
                    sp.socket_fillet_height = static_cast<float>(value);
                } else if (strcmp(child->string, "socket_fillet_offset") == 0) {
                    sp.socket_fillet_offset = static_cast<float>(value);
                } else if (strcmp(child->string, "head_fillet_height") == 0) {
                    sp.head_fillet_height = static_cast<float>(value);
                } else if (strcmp(child->string, "rotation_angle") == 0) {
                    sp.rotation_angle = static_cast<float>(value);
                }
            } else if (cJSON_IsBool(child)) {
                if (strcmp(child->string, "use_custom_direction") == 0) {
                    sp.use_custom_direction = cJSON_IsTrue(child);
                }
            }
        }
        return sp;
    };

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, obj) {
        if (!child->string)
            continue;

        const char* key = child->string;

        if (strcmp(key, "segment_mode") == 0 && cJSON_IsString(child)) {
            const char* mode_str = child->valuestring;
            if (strcmp(mode_str, "collision") == 0) {
                snapshot.segment_mode = RenderVoxelItem::COLLISION;
            } else if (strcmp(mode_str, "plane") == 0) {
                snapshot.segment_mode = RenderVoxelItem::PLANE;
            } else if (strcmp(mode_str, "concave_cone") == 0) {
                snapshot.segment_mode = RenderVoxelItem::CONCAVE_CONE;
            } else if (strcmp(mode_str, "split_disconnected") == 0) {
                snapshot.segment_mode = RenderVoxelItem::SPLIT_DISCONNECTED;
            } else if (strcmp(mode_str, "neighbor") == 0) {
                snapshot.segment_mode = RenderVoxelItem::NEIGHBOR;
            } else if (strcmp(mode_str, "fill_interior") == 0) {
                snapshot.segment_mode = RenderVoxelItem::FILL_INTERIOR;
            } else if (strcmp(mode_str, "chain") == 0) {
                snapshot.segment_mode = RenderVoxelItem::CHAIN;
            } else if (strcmp(mode_str, "sdf_node_split") == 0) {
                snapshot.segment_mode = RenderVoxelItem::SDF_NODE_SPLIT;
            } else {
                snapshot.segment_mode = RenderVoxelItem::COLLISION;
            }
        } else if (cJSON_IsNumber(child)) {
            const double value = cJSON_GetNumberValue(child);
            if (strcmp(key, "sdf_split_target_id") == 0) {
                snapshot.sdf_split_target_id = child->valueint;
            } else if (strcmp(key, "chain_min_radius") == 0) {
                snapshot.chain_min_radius = child->valueint;
            } else if (strcmp(key, "stl_load_mode") == 0) {
                snapshot.stl_load_mode = child->valueint;
            } else if (strcmp(key, "source_type") == 0) {
                snapshot.source_type = child->valueint;
            } else if (strcmp(key, "source_node_id") == 0) {
                snapshot.source_node_id = child->valueint;
            } else if (strcmp(key, "node_source_data_type") == 0) {
                snapshot.node_source_data_type = child->valueint;
            } else if (strcmp(key, "node_source_sdf_subdivisions") == 0) {
                snapshot.node_source_sdf_subdivisions = child->valueint;
            } else if (strcmp(key, "node_source_sdf_simplify_ratio") == 0) {
                snapshot.node_source_sdf_simplify_ratio =
                    static_cast<float>(value);
            }
        } else if (cJSON_IsBool(child)) {
            if (strcmp(key, "use_cgal_skeleton") == 0) {
                snapshot.use_cgal_skeleton = cJSON_IsTrue(child);
            } else if (strcmp(key, "load_as_sdf") == 0) {
                snapshot.load_as_sdf = cJSON_IsTrue(child);
            } else if (strcmp(key, "use_precise_voxelization") == 0) {
                snapshot.use_precise_voxelization = cJSON_IsTrue(child);
            } else if (strcmp(key, "mesh_only") == 0) {
                snapshot.mesh_only = cJSON_IsTrue(child);
            } else if (strcmp(key, "node_source_sdf_simplify") == 0) {
                snapshot.node_source_sdf_simplify = cJSON_IsTrue(child);
            } else if (strcmp(key, "show_silhouette_center") == 0) {
                snapshot.show_silhouette_center = cJSON_IsTrue(child);
            }
        } else if (cJSON_IsString(child)) {
            if (strcmp(key, "stl_path") == 0) {
                snapshot.stl_path = child->valuestring;
            }
        } else if (cJSON_IsObject(child)) {
            if (strcmp(key, "collision_group") == 0) {
                snapshot.collision_group =
                    sinriv::kigstudio::from_json_collision_group(child);
            } else if (strcmp(key, "plane") == 0) {
                snapshot.plane = sinriv::kigstudio::from_json_plane(child);
            } else if (strcmp(key, "concave_cone") == 0) {
                snapshot.concave_cone =
                    sinriv::kigstudio::voxel::concave::from_json_cone(child);
            } else if (strcmp(key, "sdf_split_translation") == 0) {
                snapshot.sdf_split_translation =
                    sinriv::kigstudio::vec3_from_json<vec3f>(child);
            } else if (strcmp(key, "sdf_split_rotation") == 0) {
                snapshot.sdf_split_rotation =
                    sinriv::kigstudio::vec3_from_json<vec3f>(child);
            } else if (strcmp(key, "sdf_split_scale") == 0) {
                snapshot.sdf_split_scale =
                    sinriv::kigstudio::vec3_from_json<vec3f>(child);
            } else if (strcmp(key, "silhouette_center") == 0) {
                snapshot.silhouette_center =
                    sinriv::kigstudio::vec3_from_json<
                        sinriv::kigstudio::vec3<float>>(child);
            }
        } else if (cJSON_IsArray(child)) {
            if (strcmp(key, "concave_cone_expanded_vertices") == 0) {
                int expanded_count = cJSON_GetArraySize(child);
                for (int i = 0; i < expanded_count; ++i) {
                    const cJSON* v = cJSON_GetArrayItem(child, i);
                    if (v && cJSON_IsNumber(v))
                        snapshot.concave_cone_expanded_vertices.push_back(
                            v->valueint);
                }
            } else if (strcmp(key, "picked_skeleton_points") == 0) {
                int sp_count = cJSON_GetArraySize(child);
                for (int i = 0; i < sp_count; ++i) {
                    const cJSON* sp_obj = cJSON_GetArrayItem(child, i);
                    if (sp_obj && cJSON_IsObject(sp_obj)) {
                        snapshot.picked_skeleton_points.push_back(
                            parse_skeleton_point(sp_obj));
                    }
                }
            } else if (strcmp(key, "skeleton_lines") == 0) {
                int line_count = cJSON_GetArraySize(child);
                for (int i = 0; i < line_count; ++i) {
                    const cJSON* line_obj = cJSON_GetArrayItem(child, i);
                    if (!line_obj || !cJSON_IsObject(line_obj))
                        continue;

                    sinriv::kigstudio::voxel::vec3f start;
                    sinriv::kigstudio::voxel::vec3f end;
                    const cJSON* line_child = nullptr;
                    cJSON_ArrayForEach(line_child, line_obj) {
                        if (!line_child->string)
                            continue;
                        if (!cJSON_IsObject(line_child))
                            continue;
                        if (strcmp(line_child->string, "start") == 0) {
                            start = sinriv::kigstudio::vec3_from_json<
                                sinriv::kigstudio::voxel::vec3f>(line_child);
                        } else if (strcmp(line_child->string, "end") == 0) {
                            end = sinriv::kigstudio::vec3_from_json<
                                sinriv::kigstudio::voxel::vec3f>(line_child);
                        }
                    }
                    snapshot.skeleton_lines.emplace_back(start, end);
                }
            }
        }
    }

    return snapshot;
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

    // 保存工作流输入/输出（节点ID + 文件路径）
    {
        auto save_flow_entries =
            [](const std::vector<FlowEntry>& entries) -> cJSON* {
            cJSON* arr = cJSON_CreateArray();
            for (const auto& e : entries) {
                cJSON* obj = cJSON_CreateObject();
                cJSON_AddNumberToObject(obj, "node_id", e.node_id);
                cJSON_AddStringToObject(obj, "file_path",
                                        e.file_path.c_str());
                cJSON_AddItemToArray(arr, obj);
            }
            return arr;
        };
        cJSON_AddItemToObject(root, "flow_inputs",
                              save_flow_entries(flow_inputs));
        cJSON_AddItemToObject(root, "flow_outputs",
                              save_flow_entries(flow_outputs));
    }

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
    update_nav_node_status = true;
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

    int version = 0;
    bool has_version = false;
    bool has_current_id = false;
    const cJSON* items_arr = nullptr;
    const cJSON* flow_inputs_arr = nullptr;
    const cJSON* flow_outputs_arr = nullptr;

    const cJSON* child = nullptr;
    cJSON_ArrayForEach(child, root) {
        if (!child->string)
            continue;

        if (strcmp(child->string, "version") == 0 &&
            cJSON_IsNumber(child)) {
            version = child->valueint;
            has_version = true;
        } else if (strcmp(child->string, "current_id") == 0 &&
                   cJSON_IsNumber(child)) {
            current_id = child->valueint;
            has_current_id = true;
        } else if (strcmp(child->string, "items") == 0 &&
                   cJSON_IsArray(child)) {
            items_arr = child;
        } else if (strcmp(child->string, "flow_inputs") == 0 &&
                   cJSON_IsArray(child)) {
            flow_inputs_arr = child;
        } else if (strcmp(child->string, "flow_outputs") == 0 &&
                   cJSON_IsArray(child)) {
            flow_outputs_arr = child;
        }
    }

    if (!has_version) {
        last_load_error = "missing 'version' field";
        cJSON_Delete(root);
        return false;
    }
    if (version != 1) {
        last_load_error = "unsupported version: " + std::to_string(version);
        cJSON_Delete(root);
        return false;
    }

    if (!has_current_id) {
        last_load_error = "missing 'current_id' field";
        cJSON_Delete(root);
        return false;
    }

    if (!items_arr) {
        last_load_error = "missing 'items' field";
        cJSON_Delete(root);
        return false;
    }
    int count = cJSON_GetArraySize(items_arr);
    {
        std::lock_guard<std::mutex> lock(locker);
        for (int i = 0; i < count; ++i) {
            const cJSON* item_obj = cJSON_GetArrayItem(items_arr, i);
            auto item = item_from_json(item_obj);
            if (!item) {
                last_load_error =
                    "item_from_json failed at index " + std::to_string(i);
                cJSON_Delete(root);
                return false;
            }
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
                    for (auto [tri, n] :
                         sinriv::kigstudio::voxel::readSTL(utf8_stl_path)) {
                        (void)n;
                        item->source_triangles.push_back(tri);
                    }
                    if (item->load_as_sdf && !item->source_triangles.empty()) {
                        auto mesh_sdf = std::make_shared<
                            sinriv::kigstudio::sdf::SDF_Mesh>();
                        if (item->stl_load_mode ==
                                static_cast<int>(
                                    StlLoadMode::SILHOUETTE) ||
                            item->stl_load_mode ==
                                static_cast<int>(
                                    StlLoadMode::CONVEX_HULL)) {
                            mesh_sdf->loadTriangles(item->source_triangles);
                        } else {
                            mesh_sdf->loadSTL(item->stl_path);
                        }
                        item->sdf_data = std::move(mesh_sdf);
                    }
                } catch (const std::exception& e) {
                    std::cout << "Failed to load STL mesh for item " << id
                              << ": " << e.what() << std::endl;
                }
            }
            const cJSON* has_marked =
                cJSON_GetObjectItem(item_obj, "has_marked_voxels");
            if (has_marked && cJSON_IsTrue(has_marked)) {
                std::filesystem::path marked_path =
                    dir / "marked" / (std::to_string(id) + ".vxgrid");
                if (std::filesystem::exists(marked_path)) {
                    if (!sinriv::kigstudio::load(marked_path,
                                                 item->marked_voxels)) {
                        std::cout << "Failed to load marked voxels for item "
                                  << id << std::endl;
                    } else {
                        item->marked_voxels.global_position =
                            item->voxel_grid_data.global_position;
                        item->marked_voxels.voxel_size =
                            item->voxel_grid_data.voxel_size;
                        item->marked_voxels_dirty = true;
                    }
                }
            }
            items[id] = std::move(item);
        }
    }

    // 加载工作流输入/输出（节点ID + 文件路径，兼容旧格式）
    auto load_flow_entries =
        [](const cJSON* arr) -> std::vector<FlowEntry> {
        std::vector<FlowEntry> result;
        if (!arr) return result;
        int count = cJSON_GetArraySize(arr);
        for (int i = 0; i < count; ++i) {
            const cJSON* v = cJSON_GetArrayItem(arr, i);
            if (!v) continue;
            FlowEntry e;
            if (cJSON_IsObject(v)) {
                const cJSON* nid = cJSON_GetObjectItem(v, "node_id");
                const cJSON* fp = cJSON_GetObjectItem(v, "file_path");
                if (nid && cJSON_IsNumber(nid))
                    e.node_id = nid->valueint;
                if (fp && cJSON_IsString(fp) && fp->valuestring)
                    e.file_path = fp->valuestring;
                result.push_back(e);
            } else if (cJSON_IsNumber(v)) {
                // 旧格式：纯节点ID
                e.node_id = v->valueint;
                result.push_back(e);
            } else if (cJSON_IsString(v)) {
                // 旧格式：纯文件路径
                e.file_path = v->valuestring;
                result.push_back(e);
            }
        }
        return result;
    };
    flow_inputs = load_flow_entries(flow_inputs_arr);
    flow_outputs = load_flow_entries(flow_outputs_arr);
    flow_needs_recompute = true;

    // 重建由 segment 产生的子节点的 SDF 数据
    //（体素网格已从 .vxgrid 恢复，但 sdf_data 无法直接序列化）
    {
        std::unordered_map<int, std::vector<int>> forward_edges;
        std::unordered_map<int, int> in_degree;
        for (const auto& [id, item] : items) {
            in_degree[id] = 0;
            for (int child_id : item->children) {
                if (child_id >= 0 && items.find(child_id) != items.end()) {
                    forward_edges[id].push_back(child_id);
                    ++in_degree[child_id];
                }
            }
            if (item->segment_mode == RenderVoxelItem::SDF_NODE_SPLIT &&
                item->sdf_split_target_id >= 0) {
                auto target_it = items.find(item->sdf_split_target_id);
                if (target_it != items.end()) {
                    forward_edges[item->sdf_split_target_id].push_back(id);
                    ++in_degree[id];
                }
            }
        }

        std::queue<int> q;
        for (const auto& [id, degree] : in_degree) {
            if (degree == 0)
                q.push(id);
        }

        std::vector<int> order;
        order.reserve(items.size());
        while (!q.empty()) {
            int cur = q.front();
            q.pop();
            order.push_back(cur);
            auto it = forward_edges.find(cur);
            if (it == forward_edges.end())
                continue;
            for (int next : it->second) {
                auto deg_it = in_degree.find(next);
                if (deg_it == in_degree.end())
                    continue;
                if (--deg_it->second == 0)
                    q.push(next);
            }
        }

        for (int id : order) {
            auto it = items.find(id);
            if (it == items.end())
                continue;
            auto& item = *it->second;
            if (!item.sdf_data)
                continue;

            bool has_valid_children = false;
            for (int child_id : item.children) {
                if (child_id >= 0 && items.find(child_id) != items.end()) {
                    has_valid_children = true;
                    break;
                }
            }
            if (!has_valid_children)
                continue;

            try {
                auto results = item.do_segment();
                if (results.size() != item.children.size()) {
                    std::cerr << "[load_project] segment result count mismatch "
                                 "for node "
                              << id << std::endl;
                    continue;
                }
                for (size_t i = 0; i < results.size(); ++i) {
                    int child_id = item.children[i];
                    if (child_id < 0)
                        continue;
                    auto child_it = items.find(child_id);
                    if (child_it == items.end())
                        continue;
                    if (!child_it->second->sdf_data) {
                        child_it->second->sdf_data =
                            std::move(std::get<1>(results[i]));
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "[load_project] failed to rebuild SDF for node "
                          << id << ": " << e.what() << std::endl;
            }
        }
    }

    cJSON_Delete(root);
    if (!items.empty()) {
        render_id = items.begin()->first;
    }
    project_path = folder;
    nav_layout_initialized = true;
    clear_all_dirty();
    object_editor_tab = 0;
    return true;
}

}  // namespace sinriv::ui::render
