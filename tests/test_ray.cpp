#include "kigstudio/voxel/triangle_bvh.h"
#include "kigstudio/voxel/voxel2mesh.h"

void test_bvh() {
    sinriv::kigstudio::voxel::triangle_bvh<float> bvh;
    for (auto [tri, n] : sinriv::kigstudio::voxel::readSTL(
             "../../assets/test/test/yumenikki_face.stl")) {
        bvh.insert(tri);
    }
    std::cout << "bvh size: " << bvh.global_boundBox_min << "->"
              << bvh.global_boundBox_max << std::endl;
    bvh.getSolidByFace(1, 1, 1,
                       sinriv::kigstudio::voxel::triangle_bvh<
                           float>::voxel_face_e::voxel_face_X,
                       [&](auto start, auto end) {
                           std::cout << start << "->" << end << std::endl;
                       });
}
void test_triangle_coll() {
    using namespace sinriv::kigstudio::voxel;
    using namespace sinriv::kigstudio;
    using number_t = float;
    using triangle = std::tuple<vec3<number_t>, vec3<number_t>, vec3<number_t>>;
    std::cout << std::fixed << std::setprecision(4);

    // 1. 准备射线数据
    // 射线: (-116.60, -115.20, 56.03) -> (116.60, -115.20, 56.03)
    vec3<number_t> r_start(-116.60, -115.20, 56.03);
    vec3<number_t> r_end(116.60, -115.20, 56.03);
    vec3<number_t> r_dir_long = r_end - r_start;  // 长度 ~233.2

    // 构造两个射线对象：一个用长向量，一个用归一化向量
    ray<number_t> ray_original(r_start, r_end);
    ray<number_t> ray_normalized(r_start, r_end + r_dir_long.normalize());

    std::cout << "ray begin:" << r_start << std::endl;
    std::cout << "ray length:: " << r_dir_long.length() << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    // 2. 准备三角形数据 (从日志中提取的几个典型三角形)
    std::vector<triangle> test_tris = {
        triangle(std::make_tuple(vec3<number_t>(static_cast<number_t>(17.44),
                                                static_cast<number_t>(-115.29),
                                                static_cast<number_t>(54.96)),
                                 vec3<number_t>(static_cast<number_t>(20.27),
                                                static_cast<number_t>(-114.92),
                                                static_cast<number_t>(56.73)),
                                 vec3<number_t>(static_cast<number_t>(17.40),
                                                static_cast<number_t>(-115.25),
                                                static_cast<number_t>(56.75)))),
        triangle(std::make_tuple(vec3<number_t>(static_cast<number_t>(17.44),
                                                static_cast<number_t>(-115.29),
                                                static_cast<number_t>(54.96)),
                                 vec3<number_t>(static_cast<number_t>(20.32),
                                                static_cast<number_t>(-114.96),
                                                static_cast<number_t>(54.94)),
                                 vec3<number_t>(static_cast<number_t>(20.27),
                                                static_cast<number_t>(-114.92),
                                                static_cast<number_t>(56.73)))),
        triangle(
            std::make_tuple(vec3<number_t>(static_cast<number_t>(-17.44),
                                           static_cast<number_t>(-115.29),
                                           static_cast<number_t>(54.96)),
                            vec3<number_t>(static_cast<number_t>(-17.40),
                                           static_cast<number_t>(-115.25),
                                           static_cast<number_t>(56.75)),
                            vec3<number_t>(static_cast<number_t>(-20.27),
                                           static_cast<number_t>(-114.92),
                                           static_cast<number_t>(56.73))))};

    sinriv::kigstudio::voxel::triangle_bvh<number_t>::trangle_box box;
    vec3<number_t> hit_pos;

    for (int i = 0; i < test_tris.size(); ++i) {
        box.vertex = test_tris[i];

        const auto& v0 = std::get<0>(box.vertex);
        const auto& v1 = std::get<1>(box.vertex);
        const auto& v2 = std::get<2>(box.vertex);

        std::cout << "triangle:" << i << ": V0(" << v0.x << "," << v0.y << ","
                  << v0.z << ")..." << std::endl;

        // --- 调用 1: 使用原始长向量射线 ---
        bool hit_raw = box.rayTest(ray_original, hit_pos);
        if (hit_raw) {
            std::cout << "hit pos:" << hit_pos << std::endl;
        } else {
            std::cout << "no hit" << std::endl;
        }

        // --- 调用 2: 使用归一化射线 ---
        bool hit_norm = box.rayTest(ray_normalized, hit_pos);
        if (hit_norm) {
            std::cout << "hit pos:" << hit_pos << std::endl;
        } else {
            std::cout << "no hit" << std::endl;
        }

        std::cout << std::endl;
    }
}
int main() {
    test_bvh();
    // test_triangle_coll();
    return 0;
}