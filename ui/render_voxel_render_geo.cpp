#include "render_voxel_list.h"
namespace sinriv::ui::render {
std::tuple<RenderVoxelList::RenderVoxelItem*, RenderVoxelList::RenderVoxelItem*>
RenderVoxelList::do_segment(int index) {
    // 执行分割，并在manager中创建两个，然后返回
    locker.lock();
    auto it = items.find(index);
    if (it != items.end()) {
        if (it->second->write_count != 0) {
            locker.unlock();
            return std::make_tuple(nullptr, nullptr);
        }
        it->second->ref_count++;
        it->second->write_count++;
        locker.unlock();
        auto res = it->second->do_segment();
        it->second->ref_count--;
        it->second->write_count--;
        auto item1 = std::make_unique<RenderVoxelItem>();
        auto item2 = std::make_unique<RenderVoxelItem>();
        item1->manager = this;
        item2->manager = this;
        item1->id = current_id++;
        item2->id = current_id++;
        item1->voxel_grid_data = std::get<0>(res);
        item2->voxel_grid_data = std::get<1>(res);
        auto item1_ptr = item1.get();
        auto item2_ptr = item2.get();
        {
            std::lock_guard<std::mutex> lock(locker);
            items[item1->id] = std::move(item1);
            items[item2->id] = std::move(item2);
        }
        return std::make_tuple(item1_ptr, item2_ptr);
    } else {
        locker.unlock();
    }
    return std::make_tuple(nullptr, nullptr);
}

void RenderVoxelList::load_stl(std::string filename,
                               float voxel_size,
                               double isolevel,
                               bool smooth_normals) {
    using namespace sinriv::kigstudio::voxel;
    using MeshData = mesh_detail::AsyncVoxelMeshData;
    using Triangle = triangle_bvh<float>::triangle;

    // Phase 1: Read STL
    queue_progress = 0.05f;
    triangle_bvh<float> bvh;
    size_t tri_count = 0;
    for (auto [tri, n] : sinriv::kigstudio::voxel::readSTL(filename)) {
        (void)n;
        bvh.insert(tri);
        ++tri_count;
        if (tri_count % 1000 == 0) {
            queue_progress =
                (0.05f + 0.08f * std::min(1.0f, static_cast<float>(tri_count) /
                                                    50000.0f));
        }
    }

    // Phase 2: Voxelize
    queue_status = "Voxelizing...";
    queue_progress = 0.15f;

    VoxelGrid voxel_data;
    voxel_data.voxel_size.x = voxel_size;
    voxel_data.voxel_size.y = voxel_size;
    voxel_data.voxel_size.z = voxel_size;
    voxel_data.global_position.x = bvh.global_boundBox_min.x;
    voxel_data.global_position.y = bvh.global_boundBox_min.y;
    voxel_data.global_position.z = bvh.global_boundBox_min.z;

    float minx = floor(bvh.global_boundBox_min.x / voxel_size) * voxel_size;
    float miny = floor(bvh.global_boundBox_min.y / voxel_size) * voxel_size;
    float minz = floor(bvh.global_boundBox_min.z / voxel_size) * voxel_size;
    float maxx = ceil(bvh.global_boundBox_max.x / voxel_size) * voxel_size;
    float maxy = ceil(bvh.global_boundBox_max.y / voxel_size) * voxel_size;
    float maxz = ceil(bvh.global_boundBox_max.z / voxel_size) * voxel_size;
    int num_block_y = static_cast<int>(ceil((maxy - miny) / voxel_size));
    int num_block_z = static_cast<int>(ceil((maxz - minz) / voxel_size));
    size_t total_rays = static_cast<size_t>(num_block_y) * num_block_z;
    if (total_rays == 0)
        total_rays = 1;

    std::mutex bvh_locker;
    std::atomic<size_t> callback_count{0};

    bvh.getSolidByFace(
        voxel_size, voxel_size, voxel_size, triangle_bvh<float>::voxel_face_X,
        [&](auto start, auto end) {
            int start_x = static_cast<int>(std::round(
                (start.x - voxel_data.global_position.x) / voxel_size));
            int start_y = static_cast<int>(std::round(
                (start.y - voxel_data.global_position.y) / voxel_size));
            int start_z = static_cast<int>(std::round(
                (start.z - voxel_data.global_position.z) / voxel_size));
            int end_x = static_cast<int>(std::round(
                (end.x - voxel_data.global_position.x) / voxel_size));
            int end_y = static_cast<int>(std::round(
                (end.y - voxel_data.global_position.y) / voxel_size));
            int end_z = static_cast<int>(std::round(
                (end.z - voxel_data.global_position.z) / voxel_size));

            bvh_locker.lock();
            for (int i = start_x; i <= end_x; ++i) {
                for (int j = start_y; j <= end_y; ++j) {
                    for (int k = start_z; k <= end_z; ++k) {
                        if (i >= 0 && j >= 0 && k >= 0) {
                            voxel_data.insert({i, j, k});
                        }
                    }
                }
            }
            bvh_locker.unlock();

            size_t cnt =
                callback_count.fetch_add(1, std::memory_order_relaxed) + 1;
            if (cnt % 100 == 0) {
                float p =
                    0.15f + 0.35f * std::min(1.0f, static_cast<float>(cnt) /
                                                       static_cast<float>(
                                                           total_rays * 2));
                queue_progress = p;
            }
        });

    queue_progress = 0.50f;

    // Phase 3: Generate mesh
    queue_status = "Generating mesh...";
    int num_triangles = 0;
    auto generator =
        generateMesh(voxel_data, isolevel, num_triangles, smooth_normals);
    MeshData data;
    size_t processed_tris = 0;
    size_t estimated_tris = voxel_data.num_chunk() * 200;
    if (estimated_tris == 0)
        estimated_tris = 1;

    for (auto [tri, n] : generator) {
        const uint32_t base = static_cast<uint32_t>(data.vertices.size());
        data.vertices.push_back({std::get<0>(tri).x, std::get<0>(tri).y,
                                 std::get<0>(tri).z, n.x, n.y, n.z});
        data.vertices.push_back({std::get<1>(tri).x, std::get<1>(tri).y,
                                 std::get<1>(tri).z, n.x, n.y, n.z});
        data.vertices.push_back({std::get<2>(tri).x, std::get<2>(tri).y,
                                 std::get<2>(tri).z, n.x, n.y, n.z});
        data.indices.push_back(base);
        data.indices.push_back(base + 1);
        data.indices.push_back(base + 2);

        ++processed_tris;
        if (processed_tris % 1000 == 0) {
            float p =
                0.50f +
                0.40f * std::min(1.0f, static_cast<float>(processed_tris) /
                                           static_cast<float>(estimated_tris));
            queue_progress = p;
        }
    }

    queue_status = "Uploading...";
    queue_progress = 0.95f;

    {
        // std::lock_guard<std::mutex> lock(result_mutex_);
        // result_mesh = std::move(data);
        // result_voxel = std::move(voxel_data);

        auto item = std::make_unique<RenderVoxelItem>();
        item->manager = this;
        item->id = current_id++;
        item->mesh_renderer.loadGeometry(
            sinriv::kigstudio::voxel::readSTL(filename));
        item->voxel_renderer.loadGeometry(data);
        item->voxel_grid_data = std::move(voxel_data);
        {
            std::lock_guard<std::mutex> lock(locker);
            items[item->id] = std::move(item);
        }
    }

    queue_status = "Done";
    queue_progress = 1.0f;
}
}  // namespace sinriv::ui::render