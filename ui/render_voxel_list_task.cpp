
#include "render_voxel_list.h"
namespace sinriv::ui::render {

void RenderVoxelList::queue_thread() {
    std::cout << "Queue thread started" << std::endl;
    while (true) {
        queue_mutex.lock();
        this->queue_num = queue.size();
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
                std::cout << "Stop queue" << std::endl;
                return;
            // case TASK_REMOVE_ITEM:
            //     // 移除item
            //     break;
            case TASK_LOAD_STL:
                // 加载stl文件
                std::cout << "Load stl file: " << task.file_path << std::endl;
                queue_running = true;
                try {
                    load_stl(task.file_path);
                } catch (std::runtime_error& e) {
                    std::cerr << "Runtime error loading STL file: " << e.what()
                              << std::endl;
                } catch (std::logic_error& e) {
                    std::cerr << "Logic error loading STL file: " << e.what()
                              << std::endl;
                } catch (std::exception& e) {
                    std::cerr << "Error loading STL file: " << e.what()
                              << std::endl;
                } catch (... ) {
                    std::cerr << "Unknown error loading STL file. " << std::endl;
                }

                queue_running = false;
                break;
            case TASK_SEGMENT:
                // 分割
                queue_running = true;
                try {
                    do_segment(task.index);
                } catch (std::runtime_error& e) {
                    std::cerr << "Runtime error doing segment: " << e.what()
                              << std::endl;
                } catch (std::logic_error& e) {
                    std::cerr << "Logic error doing segment: " << e.what()
                              << std::endl;
                } catch (std::exception& e) {
                    std::cerr << "Error doing segment: " << e.what()
                              << std::endl;
                } catch (... ) {
                    std::cerr << "Unknown error doing segment. " << std::endl;
                }
                queue_running = false;
                break;
            case TASK_GENERATE_THUMBNAIL_MESH: {
                queue_running = true;
                queue_status = "Generating thumbnail mesh...";
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
                            voxel_data, 0.5, num_triangles, true);
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

                } catch (std::runtime_error& e) {
                    std::cerr << "Runtime error generating thumbnail mesh: "
                              << e.what() << std::endl;
                } catch (std::logic_error& e) {
                    std::cerr << "Logic error generating thumbnail mesh: "
                              << e.what() << std::endl;
                } catch (std::exception& e) {
                    std::cerr << "Error generating thumbnail mesh: " << e.what()
                              << std::endl;
                } catch (... ) {
                    std::cerr << "Unknown error generating thumbnail mesh. "
                              << std::endl;
                }

                queue_progress = 1.0f;
                queue_status = "Done";
                queue_running = false;
                break;
            }
        }
    }
}
}  // namespace sinriv::ui::render