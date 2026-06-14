
#include <sstream>
#include "kigstudio/cgal/mesh_simplification.h"
#include "render_voxel_list.h"
#include "utils.h"
namespace sinriv::ui::render {

#ifdef _DEBUG
#define TRACE_STACK() sinriv::print_stacktrace(std::cerr)
#else
#define TRACE_STACK() ((void)0)
#endif

void RenderVoxelList::process_queue_result() {
    // 在 UI 线程中安全释放被后台线程移入的 item
    {
        std::lock_guard<std::mutex> lock(pending_deletion_mutex);
        pending_deletion.clear();
    }
    // 回收ref_count和write_count为0的item
    std::lock_guard<std::mutex> lock(locker);
    std::vector<int> to_remove;
    for (auto& it : items) {
        int ref_count = it.second->ref_count;
        int write_count = it.second->write_count;
        if (it.second->queue_release) {
            ref_count--;
        }
        if (ref_count == 0 && write_count == 0) {
            to_remove.push_back(it.first);
            // 级联标记子节点释放
            for (int child_id : it.second->children) {
                auto child_it = items.find(child_id);
                if (child_it != items.end()) {
                    child_it->second->queue_release = true;
                }
            }
        }
    }
    for (auto& it : to_remove) {
        int removed_id = it;
        items.erase(it);
        update_nav_node_status = true;
        // 断开指向被删除节点的 source-node 引用
        for (auto& [id, item] : items) {
            if (item->source_type == 1 &&
                item->source_node_id == removed_id) {
                item->source_type = 0;
                item->source_node_id = -1;
            }
        }
    }
    // 重新选一个可用的render_id
    if (items.find(render_id) == items.end()) {
        auto it = items.begin();
        if (it != items.end()) {
            render_id = it->first;
        }
    }
}

void RenderVoxelList::queue_thread() {
    std::cout << "Queue thread started" << std::endl;
    while (true) {
        if (!queue_should_continue.load()) {
            queue_running = false;
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                std::queue<QueueTask> empty;
                std::swap(queue, empty);
            }
            queue_should_continue = true;
            continue;
        }

        queue_mutex.lock();
        this->queue_num = static_cast<int>(queue.size());
        if (queue.empty()) {
            queue_mutex.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        auto task = queue.front();
        queue.pop();
        queue_mutex.unlock();

        switch (task.type) {
            case TASK_STOP:
                append_queue_logf("log.queue.stop");
                std::cout << "Stop queue" << std::endl;
                return;
            // case TASK_REMOVE_ITEM:
            //     // 移除item
            //     break;
            case TASK_LOAD_STL:
                // 加载stl文件
                append_queue_logf("log.queue.start_load_stl",
                                  task.file_path.c_str());
                std::cout << "Load stl file: " << task.file_path << std::endl;
                queue_running = true;
                try {
                    load_stl(task.file_path, task.voxel_size, 0.5, true, -1,
                             task.load_mode, task.load_as_sdf);
                    append_queue_logf("log.queue.done_load_stl",
                                      task.file_path.c_str());
                } catch (std::runtime_error& e) {
                    append_queue_logf("log.queue.error_load_stl",
                                      task.file_path.c_str(), e.what());
                    std::cerr << "Runtime error loading STL file: " << e.what()
                              << std::endl;
                    TRACE_STACK();
                } catch (std::logic_error& e) {
                    append_queue_logf("log.queue.error_load_stl",
                                      task.file_path.c_str(), e.what());
                    std::cerr << "Logic error loading STL file: " << e.what()
                              << std::endl;
                    TRACE_STACK();
                } catch (std::exception& e) {
                    append_queue_logf("log.queue.error_load_stl",
                                      task.file_path.c_str(), e.what());
                    std::cerr << "Error loading STL file: " << e.what()
                              << std::endl;
                    TRACE_STACK();
                } catch (...) {
                    append_queue_logf(
                        "log.queue.error_load_stl", task.file_path.c_str(),
                        get_locale_string("log.queue.unknown_error").c_str());
                    std::cerr << "Unknown error loading STL file. "
                              << std::endl;
                    TRACE_STACK();
                }

                queue_running = false;
                break;
            case TASK_RELOAD_STL:
                // 重新加载stl文件（更改体素大小）或从节点加载
                queue_running = true;
                try {
                    if (task.source_node_id >= 0) {
                        append_queue_logf("log.queue.start_reload_stl",
                                          task.index,
                                          ("node " +
                                           std::to_string(task.source_node_id))
                                              .c_str());
                        std::cout << "Reload from node: "
                                  << task.source_node_id << " for item "
                                  << task.index << std::endl;
                        load_from_node(
                            task.index, task.source_node_id,
                            task.node_source_data_type,
                            task.node_source_sdf_subdivisions,
                            task.node_source_sdf_simplify,
                            task.node_source_sdf_simplify_ratio,
                            task.load_mode);
                    } else {
                        append_queue_logf("log.queue.start_reload_stl",
                                          task.index,
                                          task.file_path.c_str());
                        std::cout << "Reload stl file: " << task.file_path
                                  << " for item " << task.index << std::endl;
                        load_stl(task.file_path, task.voxel_size, 0.5, true,
                                 task.index, task.load_mode,
                                 task.load_as_sdf);
                    }
                    append_queue_logf("log.queue.done_reload_stl", task.index);
                } catch (std::runtime_error& e) {
                    append_queue_logf("log.queue.error_reload_stl", task.index,
                                      e.what());
                    std::cerr
                        << "Runtime error reloading STL file: " << e.what()
                        << std::endl;
                    TRACE_STACK();
                } catch (std::logic_error& e) {
                    append_queue_logf("log.queue.error_reload_stl", task.index,
                                      e.what());
                    std::cerr << "Logic error reloading STL file: " << e.what()
                              << std::endl;
                    TRACE_STACK();
                } catch (std::exception& e) {
                    append_queue_logf("log.queue.error_reload_stl", task.index,
                                      e.what());
                    std::cerr << "Error reloading STL file: " << e.what()
                              << std::endl;
                    TRACE_STACK();
                } catch (...) {
                    append_queue_logf(
                        "log.queue.error_reload_stl", task.index,
                        get_locale_string("log.queue.unknown_error").c_str());
                    std::cerr << "Unknown error reloading STL file. "
                              << std::endl;
                    TRACE_STACK();
                }
                queue_running = false;
                break;
            case TASK_SEGMENT:
                // 分割
                append_queue_logf("log.queue.start_segment", task.index);
                queue_running = true;
                try {
                    do_segment(task.index);
                    append_queue_logf("log.queue.done_segment", task.index);
                } catch (std::runtime_error& e) {
                    append_queue_logf("log.queue.error_segment", task.index,
                                      e.what());
                    std::cerr << "Runtime error doing segment: " << e.what()
                              << std::endl;
                    TRACE_STACK();
                } catch (std::logic_error& e) {
                    append_queue_logf("log.queue.error_segment", task.index,
                                      e.what());
                    std::cerr << "Logic error doing segment: " << e.what()
                              << std::endl;
                    TRACE_STACK();
                } catch (std::exception& e) {
                    append_queue_logf("log.queue.error_segment", task.index,
                                      e.what());
                    std::cerr << "Error doing segment: " << e.what()
                              << std::endl;
                    TRACE_STACK();
                } catch (...) {
                    append_queue_logf(
                        "log.queue.error_segment", task.index,
                        get_locale_string("log.queue.unknown_error").c_str());
                    std::cerr << "Unknown error doing segment. " << std::endl;
                    TRACE_STACK();
                }
                queue_running = false;
                break;
            case TASK_CHECK_NON_MANIFOLD: {
                append_queue_logf("log.queue.start_check_manifold", task.index);
                queue_running = true;
                setQueueStatus(get_locale_string("status.checking_manifold"));
                queue_progress = 0.0f;
                try {
                    // 锁定 item
                    locker.lock();
                    auto it = items.find(task.index);
                    if (it == items.end() || it->second->write_count != 0) {
                        locker.unlock();
                        append_queue_logf("log.queue.skip_item_busy",
                                          task.index);
                        break;
                    }
                    it->second->write_count++;
                    auto item_ptr = it->second.get();
                    locker.unlock();

                    // 生成网格
                    std::vector<std::tuple<sinriv::kigstudio::voxel::Triangle,
                                           sinriv::kigstudio::voxel::vec3f>>
                        mesh;
                    int numTriangles = 0;
                    for (auto tri : sinriv::kigstudio::voxel::generateMesh(
                             item_ptr->voxel_grid_data, 0.5, numTriangles, true,
                             0.0f, [&](const std::string&) {
                                 return queue_should_continue.load() &&
                                        queue_running.load();
                             })) {
                        mesh.push_back(tri);
                    }

                    // 执行检测
                    queue_progress = 0.3f;
                    auto edges =
                        sinriv::kigstudio::voxel::checkNonManifoldEdges(mesh);
                    queue_progress = 0.9f;

                    // 输出结果到日志
                    if (edges.empty()) {
                        append_queue_logf("log.queue.done_no_manifold",
                                          task.index);
                    } else {
                        append_queue_logf("log.queue.found_manifold",
                                          task.index,
                                          static_cast<int>(edges.size()));
                        for (const auto& edge : edges) {
                            append_queue_logf("log.queue.manifold_edge",
                                              edge.v0.x, edge.v0.y, edge.v0.z,
                                              edge.v1.x, edge.v1.y, edge.v1.z,
                                              edge.triangle_count);
                        }
                    }

                    // 解锁
                    locker.lock();
                    it = items.find(task.index);
                    if (it != items.end()) {
                        it->second->write_count--;
                    }
                    locker.unlock();

                    queue_progress = 1.0f;
                } catch (std::exception& e) {
                    append_queue_logf("log.queue.error_check_manifold",
                                      task.index, e.what());
                    locker.lock();
                    auto it = items.find(task.index);
                    if (it != items.end()) {
                        it->second->write_count--;
                    }
                    locker.unlock();
                }
                queue_running = false;
                break;
            }
            case TASK_EXTRACT_SKELETON: {
                append_queue_logf("log.queue.start_extract_skeleton",
                                  task.index);
                queue_running = true;
                setQueueStatus(get_locale_string("status.extracting_skeleton"));
                queue_progress = 0.0f;
                try {
                    extract_skeleton(task.index);
                    append_queue_logf("log.queue.done_extract_skeleton",
                                      task.index);
                } catch (std::runtime_error& e) {
                    append_queue_logf("log.queue.error_extract_skeleton",
                                      task.index, e.what());
                    std::cerr
                        << "Runtime error extracting skeleton: " << e.what()
                        << std::endl;
                    TRACE_STACK();
                } catch (std::logic_error& e) {
                    append_queue_logf("log.queue.error_extract_skeleton",
                                      task.index, e.what());
                    std::cerr << "Logic error extracting skeleton: " << e.what()
                              << std::endl;
                    TRACE_STACK();
                } catch (std::exception& e) {
                    append_queue_logf("log.queue.error_extract_skeleton",
                                      task.index, e.what());
                    std::cerr << "Error extracting skeleton: " << e.what()
                              << std::endl;
                    TRACE_STACK();
                } catch (...) {
                    append_queue_logf(
                        "log.queue.error_extract_skeleton", task.index,
                        get_locale_string("log.queue.unknown_error").c_str());
                    std::cerr << "Unknown error extracting skeleton. "
                              << std::endl;
                }
                queue_running = false;
                break;
            }
            case TASK_GENERATE_THUMBNAIL_MESH: {
                append_queue_logf("log.queue.start_thumbnail", task.index);
                queue_running = true;
                setQueueStatus(
                    get_locale_string("status.generating_thumbnail"));
                queue_progress = 0.0f;

                try {
                    sinriv::kigstudio::voxel::VoxelGrid voxel_data;
                    {
                        std::lock_guard<std::mutex> lock(locker);
                        auto it = items.find(task.index);
                        if (it != items.end()) {
                            voxel_data = it->second->voxel_grid_data;
                        }
                    }

                    mesh_detail::AsyncVoxelMeshData data;
                    if (voxel_data.num_chunk() > 0) {
                        int num_triangles = 0;
                        auto generator = sinriv::kigstudio::voxel::generateMesh(
                            voxel_data, 0.5, num_triangles, true, 0.0f,
                            [&](const std::string&) {
                                return queue_should_continue.load() &&
                                       queue_running.load();
                            });
                        size_t processed_tris = 0;
                        size_t estimated_tris = voxel_data.num_chunk() * 200;
                        if (estimated_tris == 0)
                            estimated_tris = 1;

                        for (auto [tri, n] : generator) {
                            const uint32_t base =
                                static_cast<uint32_t>(data.vertices.size());
                            data.vertices.push_back(
                                {std::get<0>(tri).x, std::get<0>(tri).y,
                                 std::get<0>(tri).z, n.x, n.y, n.z});
                            data.vertices.push_back(
                                {std::get<1>(tri).x, std::get<1>(tri).y,
                                 std::get<1>(tri).z, n.x, n.y, n.z});
                            data.vertices.push_back(
                                {std::get<2>(tri).x, std::get<2>(tri).y,
                                 std::get<2>(tri).z, n.x, n.y, n.z});
                            data.indices.push_back(base);
                            data.indices.push_back(base + 1);
                            data.indices.push_back(base + 2);

                            ++processed_tris;
                            if (processed_tris % 1000 == 0) {
                                queue_progress =
                                    0.10f +
                                    0.80f *
                                        std::min(1.0f, static_cast<float>(
                                                           processed_tris) /
                                                           static_cast<float>(
                                                               estimated_tris));
                            }
                        }
                    }

                    {
                        std::lock_guard<std::mutex> lock(thumbnail_mesh_mutex);
                        thumbnail_mesh_results[task.index] = std::move(data);
                        thumbnail_mesh_pending.erase(task.index);
                    }
                    append_queue_logf("log.queue.done_thumbnail", task.index);

                } catch (std::runtime_error& e) {
                    append_queue_logf("log.queue.error_thumbnail", task.index,
                                      e.what());
                    std::cerr << "Runtime error generating thumbnail mesh: "
                              << e.what() << std::endl;
                    TRACE_STACK();
                } catch (std::logic_error& e) {
                    append_queue_logf("log.queue.error_thumbnail", task.index,
                                      e.what());
                    std::cerr
                        << "Logic error generating thumbnail mesh: " << e.what()
                        << std::endl;
                    TRACE_STACK();
                } catch (std::exception& e) {
                    append_queue_logf("log.queue.error_thumbnail", task.index,
                                      e.what());
                    std::cerr << "Error generating thumbnail mesh: " << e.what()
                              << std::endl;
                    TRACE_STACK();
                } catch (...) {
                    append_queue_logf(
                        "log.queue.error_thumbnail", task.index,
                        get_locale_string("log.queue.unknown_error").c_str());
                    std::cerr << "Unknown error generating thumbnail mesh. "
                              << std::endl;
                    TRACE_STACK();
                }

                queue_progress = 1.0f;
                setQueueStatus("Done");
                queue_running = false;
                break;
            }
            case TASK_EXPORT_STL: {
                append_queue_logf("log.queue.start_export_stl", task.index,
                                  task.file_path.c_str());
                queue_running = true;
                setQueueStatus(get_locale_string("status.exporting_stl"));
                queue_progress = 0.0f;
                try {
                    // 锁定 item
                    locker.lock();
                    auto it = items.find(task.index);
                    if (it == items.end() || it->second->write_count != 0) {
                        locker.unlock();
                        append_queue_logf("log.queue.skip_item_busy",
                                          task.index);
                        break;
                    }
                    it->second->write_count++;
                    auto item_ptr = it->second.get();
                    locker.unlock();

                    std::vector<std::tuple<sinriv::kigstudio::voxel::Triangle,
                                           sinriv::kigstudio::voxel::vec3f>>
                        mesh;
                    int numTriangles = 0;
                    if (task.export_mode == 1) {
                        for (auto triangles : sinriv::kigstudio::voxel::
                                 generateSmoothMeshFromSDF(
                                     item_ptr->voxel_grid_data, numTriangles,
                                     [&](const std::string& status) {
                                         setQueueStatus(
                                             get_locale_string(
                                                 "status.exporting_stl") +
                                             " " + status);
                                         return queue_should_continue.load() &&
                                                queue_running.load();
                                     },
                                     true, task.subdivisions,
                                     item_ptr->sdf_data.get())) {
                            mesh.push_back(triangles);
                        }
                    } else {
                        for (auto triangles :
                             sinriv::kigstudio::voxel::generateMesh(
                                 item_ptr->voxel_grid_data, 0.5, numTriangles,
                                 true, 0.0f, [&](const std::string& status) {
                                     setQueueStatus(
                                         get_locale_string(
                                             "status.exporting_stl") +
                                         " " + status);
                                     return queue_should_continue.load() &&
                                            queue_running.load();
                                 })) {
                            mesh.push_back(triangles);
                        }
                    }
                    queue_progress = 0.4f;
                    if (!mesh.empty()) {
                        setQueueStatus(
                            get_locale_string("status.exporting_stl") + " " +
                            get_locale_string(
                                "status.exporting_stl.cleaning_mesh"));
                        mesh = sinriv::kigstudio::voxel::cleanMesh(mesh);
                    }
                    queue_progress = 0.7f;

                    if (task.export_simplify && !mesh.empty()) {
                        size_t before = mesh.size();
                        setQueueStatus(
                            get_locale_string("status.exporting_stl") + " " +
                            get_locale_string(
                                "status.exporting_stl.simplifying_mesh"));
                        mesh = sinriv::kigstudio::cgal::simplifyMesh(
                            mesh,
                            static_cast<double>(task.export_simplify_ratio));
                        append_queue_logf("log.queue.simplify_result",
                                          task.index, static_cast<int>(before),
                                          static_cast<int>(mesh.size()));
                    }
                    queue_progress = 0.9f;
                    if (!mesh.empty()) {
                        std::cout << "[TASK_EXPORT_STL] item=" << task.index
                                  << " mesh size=" << mesh.size()
                                  << " save_to_file=" << task.save_to_file
                                  << std::endl;
                        // 缓存到 RenderVoxelItem
                        {
                            std::lock_guard<std::mutex> lock(locker);
                            auto itc = items.find(task.index);
                            if (itc != items.end()) {
                                itc->second->cached_mesh = mesh;
                                itc->second->cached_mesh_dirty = false;
                                std::cout << "[TASK_EXPORT_STL] cached_mesh saved"
                                          << std::endl;
                            }
                        }
                        if (task.save_to_file && !task.file_path.empty()) {
                            setQueueStatus(
                                get_locale_string("status.exporting_stl") + " " +
                                get_locale_string(
                                    "status.exporting_stl.saveing_mesh"));
                            sinriv::kigstudio::voxel::saveMeshToASCIISTL(
                                mesh, task.file_path);
                            append_queue_logf("log.queue.done_export_stl",
                                              task.index,
                                              task.file_path.c_str());
                        } else {
                            append_queue_logf(
                                "log.queue.done_export_stl_cached",
                                task.index);
                        }
                    } else {
                        append_queue_logf("log.queue.error_export_stl_empty",
                                          task.index);
                    }

                    // 解锁
                    locker.lock();
                    it = items.find(task.index);
                    if (it != items.end()) {
                        it->second->write_count--;
                    }
                    locker.unlock();
                    queue_progress = 1.0f;
                } catch (std::exception& e) {
                    append_queue_logf("log.queue.error_export_stl", task.index,
                                      e.what());
                    locker.lock();
                    auto it = items.find(task.index);
                    if (it != items.end()) {
                        it->second->write_count--;
                    }
                    locker.unlock();
                }
                queue_running = false;
                break;
            }
            case TASK_EXPORT_STL_ALL: {
                queue_running = true;
                setQueueStatus(get_locale_string("status.exporting_stl_all"));
                queue_progress = 0.0f;
                try {
                    std::vector<int> target_ids;
                    {
                        std::lock_guard<std::mutex> lock(locker);
                        for (auto& [id, item] : items) {
                            bool is_leaf = true;
                            for (int cid : item->children) {
                                if (cid >= 0) {
                                    is_leaf = false;
                                    break;
                                }
                            }
                            if (is_leaf) {
                                target_ids.push_back(id);
                            }
                        }
                    }

                    if (target_ids.empty()) {
                        append_queue_logf(
                            "log.queue.error_export_stl_all_empty");
                        queue_progress = 1.0f;
                        queue_running = false;
                        break;
                    }

                    std::filesystem::path export_dir =
                        utf8_path(task.file_path);
                    std::filesystem::create_directories(export_dir);

                    int total = static_cast<int>(target_ids.size());
                    int success = 0;
                    for (int i = 0; i < total; ++i) {
                        if (!queue_should_continue.load() ||
                            !queue_running.load()) {
                            break;
                        }
                        int id = (int)target_ids[i];
                        queue_progress =
                            static_cast<float>(i) / static_cast<float>(total);
                        std::string status_prefix;
                        {
                            std::string fmt = get_locale_string(
                                "status.exporting_stl_all_item");
                            char buf[256];
                            snprintf(buf, sizeof(buf), fmt.c_str(), id, i + 1,
                                     total);
                            setQueueStatus(buf);
                            status_prefix = buf;
                        }

                        locker.lock();
                        auto it = items.find(id);
                        if (it == items.end() || it->second->write_count != 0) {
                            locker.unlock();
                            append_queue_logf("log.queue.skip_item_busy", id);
                            continue;
                        }
                        it->second->write_count++;
                        auto item_ptr = it->second.get();
                        locker.unlock();

                        try {
                            std::vector<
                                std::tuple<sinriv::kigstudio::voxel::Triangle,
                                           sinriv::kigstudio::voxel::vec3f>>
                                mesh;
                            int numTriangles = 0;
                            if (task.export_mode == 1) {
                                for (auto triangles : sinriv::kigstudio::voxel::
                                         generateSmoothMeshFromSDF(
                                             item_ptr->voxel_grid_data,
                                             numTriangles,
                                             [&](const std::string& status) {
                                                 setQueueStatus(status_prefix +
                                                                status);
                                                 return queue_should_continue
                                                            .load() &&
                                                        queue_running.load();
                                             },
                                             true, task.subdivisions,
                                             item_ptr->sdf_data.get())) {
                                    mesh.push_back(triangles);
                                }
                            } else {
                                for (auto triangles :
                                     sinriv::kigstudio::voxel::generateMesh(
                                         item_ptr->voxel_grid_data, 0.5,
                                         numTriangles, true, 0.0f,
                                         [&](const std::string& status) {
                                             setQueueStatus(status_prefix +
                                                            status);
                                             return queue_should_continue
                                                        .load() &&
                                                    queue_running.load();
                                         })) {
                                    mesh.push_back(triangles);
                                }
                            }

                            if (!mesh.empty()) {
                                setQueueStatus(
                                    status_prefix + " " +
                                    get_locale_string(
                                        "status.exporting_stl.cleaning_mesh"));
                                mesh =
                                    sinriv::kigstudio::voxel::cleanMesh(mesh);
                            }

                            if (task.export_simplify && !mesh.empty()) {
                                size_t before = mesh.size();
                                setQueueStatus(
                                    status_prefix + " " +
                                    get_locale_string("status.exporting_stl."
                                                      "simplifying_mesh"));
                                mesh = sinriv::kigstudio::cgal::simplifyMesh(
                                    mesh, static_cast<double>(
                                              task.export_simplify_ratio));
                                append_queue_logf(
                                    "log.queue.simplify_result", id,
                                    static_cast<int>(before),
                                    static_cast<int>(mesh.size()));
                            }

                            if (!mesh.empty()) {
                                // 缓存到 RenderVoxelItem
                                {
                                    std::lock_guard<std::mutex> lock(locker);
                                    auto itc = items.find(id);
                                    if (itc != items.end()) {
                                        itc->second->cached_mesh = mesh;
                                        itc->second->cached_mesh_dirty = false;
                                    }
                                }
                                if (task.save_to_file && !task.file_path.empty()) {
                                    setQueueStatus(
                                        status_prefix + " " +
                                        get_locale_string(
                                            "status.exporting_stl.saveing_mesh"));
                                    std::string filename =
                                        "node_" + std::to_string(id) + ".stl";
                                    std::filesystem::path filepath =
                                        export_dir / filename;
                                    sinriv::kigstudio::voxel::saveMeshToASCIISTL(
                                        mesh, path_to_utf8(filepath));
                                }
                                success++;
                            }
                        } catch (std::exception& e) {
                            append_queue_logf("log.queue.error_export_stl", id,
                                              e.what());
                            TRACE_STACK();
                        }

                        locker.lock();
                        it = items.find(id);
                        if (it != items.end()) {
                            it->second->write_count--;
                        }
                        locker.unlock();
                    }

                    append_queue_logf("log.queue.done_export_stl_all", success,
                                      total);
                    queue_progress = 1.0f;
                } catch (std::exception& e) {
                    append_queue_logf("log.queue.error_export_stl_all",
                                      e.what());
                    TRACE_STACK();
                }
                queue_running = false;
                break;
            }
        }
    }
}

void RenderVoxelList::start_thread() {
    // 启动进程
    queue_thread_ = std::thread(&RenderVoxelList::queue_thread, this);
}

void RenderVoxelList::stop_thread() {
    if (queue_thread_.joinable()) {
        std::cout << "Waiting for queue thread to stop" << std::endl;
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            QueueTask task;
            task.type = TASK_STOP;
            queue.push(task);
        }
        queue_thread_.join();
        std::cout << "Queue thread stopped" << std::endl;
    }
}

size_t RenderVoxelList::get_num_items() {
    std::lock_guard<std::mutex> lock(locker);
    auto res = items.size();
    return res;
}

void RenderVoxelList::queue_load_stl(const std::string& file_path,
                                     float voxel_size,
                                     int load_mode,
                                     bool load_as_sdf) {
    // 将加载任务加入队列
    std::lock_guard<std::mutex> lock(queue_mutex);
    QueueTask task;
    task.type = TASK_LOAD_STL;
    task.file_path = file_path;
    task.voxel_size = voxel_size;
    task.load_mode = load_mode;
    task.load_as_sdf = load_as_sdf;
    queue.push(task);
    this->queue_num = static_cast<int>(queue.size());
}

void RenderVoxelList::queue_reload_stl(int item_id,
                                       float voxel_size,
                                       const std::string& stl_path,
                                       int load_mode,
                                       bool load_as_sdf,
                                       int source_node_id,
                                       int node_source_data_type,
                                       int node_source_sdf_subdivisions,
                                       bool node_source_sdf_simplify,
                                       float node_source_sdf_simplify_ratio) {
    if (stl_path.empty() && source_node_id < 0)
        return;
    std::lock_guard<std::mutex> lock(queue_mutex);
    QueueTask task;
    task.type = TASK_RELOAD_STL;
    task.index = item_id;
    task.file_path = stl_path;
    task.voxel_size = voxel_size;
    task.load_mode = load_mode;
    task.load_as_sdf = load_as_sdf;
    task.source_node_id = source_node_id;
    task.node_source_data_type = node_source_data_type;
    task.node_source_sdf_subdivisions = node_source_sdf_subdivisions;
    task.node_source_sdf_simplify = node_source_sdf_simplify;
    task.node_source_sdf_simplify_ratio = node_source_sdf_simplify_ratio;
    queue.push(task);
    this->queue_num = static_cast<int>(queue.size());
}

void RenderVoxelList::queue_do_segment(int index) {
    // 将分割任务加入队列
    std::lock_guard<std::mutex> lock(queue_mutex);
    QueueTask task;
    task.type = TASK_SEGMENT;
    task.index = index;
    queue.push(task);
    this->queue_num = static_cast<int>(queue.size());
}

void RenderVoxelList::queue_do_segment() {
    std::lock_guard<std::mutex> lock(locker);
    queue_do_segment_unsafe();
}
void RenderVoxelList::queue_do_segment_unsafe() {
    auto it = items.find(render_id);
    if (it != items.end()) {
        queue_do_segment(it->second->id);
    }
}

void RenderVoxelList::queue_check_non_manifold(int index) {
    std::lock_guard<std::mutex> lock(queue_mutex);
    QueueTask task;
    task.type = TASK_CHECK_NON_MANIFOLD;
    task.index = index;
    queue.push(task);
    this->queue_num = static_cast<int>(queue.size());
}

void RenderVoxelList::queue_extract_skeleton(int index) {
    std::lock_guard<std::mutex> lock(queue_mutex);
    QueueTask task;
    task.type = TASK_EXTRACT_SKELETON;
    task.index = index;
    queue.push(task);
    this->queue_num = static_cast<int>(queue.size());
}

void RenderVoxelList::queue_remove_item(int index) {
    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(index);
    if (it != items.end()) {
        it->second->ref_count--;
    }
}

bool RenderVoxelList::isQueueRunning() {
    return queue_running.load();
}

std::string RenderVoxelList::getQueueStatus() {
    std::lock_guard<std::mutex> lock(queue_status_mtx);
    return queue_status;
}

void RenderVoxelList::setQueueStatus(const std::string& status) {
    std::lock_guard<std::mutex> lock(queue_status_mtx);
    queue_status = status;
}

float RenderVoxelList::getQueueProgress() {
    return queue_progress;
}

void RenderVoxelList::release() {
    stop_thread();
    destroyIcons();
    destroyThumbnailResources();
    items.clear();
    pending_deletion.clear();
}

void RenderVoxelList::queue_export_stl(int item_id,
                                       const std::string& file_path,
                                       int mode,
                                       bool simplify,
                                       float ratio,
                                       int subdivisions,
                                       bool save_to_file) {
    std::lock_guard<std::mutex> lock(queue_mutex);
    QueueTask task;
    task.type = TASK_EXPORT_STL;
    task.index = item_id;
    task.file_path = file_path;
    task.export_mode = mode;
    task.export_simplify = simplify;
    task.export_simplify_ratio = ratio;
    task.subdivisions = subdivisions;
    task.save_to_file = save_to_file;
    queue.push(task);
    this->queue_num = static_cast<int>(queue.size());
}

void RenderVoxelList::queue_export_stl_all(const std::string& export_dir,
                                           int mode,
                                           bool simplify,
                                           float ratio,
                                           int subdivisions,
                                           bool save_to_file) {
    std::lock_guard<std::mutex> lock(queue_mutex);
    QueueTask task;
    task.type = TASK_EXPORT_STL_ALL;
    task.file_path = export_dir;
    task.export_mode = mode;
    task.export_simplify = simplify;
    task.export_simplify_ratio = ratio;
    task.subdivisions = subdivisions;
    task.save_to_file = save_to_file;
    queue.push(task);
    this->queue_num = static_cast<int>(queue.size());
}

}  // namespace sinriv::ui::render