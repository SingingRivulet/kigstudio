#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "kigstudio/mesh/conebox.h"
#include "kigstudio/voxel/voxel2mesh.h"

using namespace sinriv::kigstudio::mesh::conebox;
using namespace sinriv::kigstudio;

static void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

static bool vec3_near(const vec3f& a, const vec3f& b, float eps = 1e-4f) {
    return (a - b).length2() < eps * eps;
}

static bool has_vertex_near(const std::vector<Triangle>& mesh,
                            const vec3f& target,
                            float eps = 1e-4f) {
    for (const auto& tri : mesh) {
        if (vec3_near(std::get<0>(tri), target, eps) ||
            vec3_near(std::get<1>(tri), target, eps) ||
            vec3_near(std::get<2>(tri), target, eps)) {
            return true;
        }
    }
    return false;
}

static bool all_vertices_finite(const std::vector<Triangle>& mesh) {
    for (const auto& tri : mesh) {
        for (const auto& v :
             {std::get<0>(tri), std::get<1>(tri), std::get<2>(tri)}) {
            if (!std::isfinite(v.x) || !std::isfinite(v.y) ||
                !std::isfinite(v.z))
                return false;
        }
    }
    return true;
}

// ------------------------------------------------------------------
// 1. 空输入返回空
static void test_empty_input() {
    std::vector<Triangle> input;
    auto output = build_closed_mesh_from_triangles(input);
    expect(output.empty(), "empty input should produce empty output");
}

// 2. 单个三角形 + 手动中心（自动中心会把单三角放在立方体内部导致无输出）
static void test_single_triangle() {
    std::vector<Triangle> input = {
        std::make_tuple(vec3f(-0.2f, -0.2f, 1.0f),
                        vec3f(0.2f, -0.2f, 1.0f),
                        vec3f(0.0f, 0.2f, 1.0f))};
    vec3f center(0.0f, 10.0f, 0.0f);
    auto output = build_closed_mesh_from_triangles(input, false, center);
    expect(!output.empty(),
           "single triangle should produce non-empty mesh");
    expect(all_vertices_finite(output),
           "all output vertices should be finite");
    expect(has_vertex_near(output, center),
           "output should contain the center vertex");
}

// 3. 小模型自动缩放（min_half < 0.6）
static void test_auto_scale_small_mesh() {
    std::vector<Triangle> input = {
        std::make_tuple(vec3f(0.05f, 0.05f, 0.05f),
                        vec3f(0.05f, -0.05f, -0.05f),
                        vec3f(-0.05f, 0.05f, -0.05f)),
        std::make_tuple(vec3f(0.05f, 0.05f, 0.05f),
                        vec3f(0.05f, -0.05f, -0.05f),
                        vec3f(-0.05f, -0.05f, 0.05f)),
        std::make_tuple(vec3f(0.05f, 0.05f, 0.05f),
                        vec3f(-0.05f, 0.05f, -0.05f),
                        vec3f(-0.05f, -0.05f, 0.05f)),
        std::make_tuple(vec3f(0.05f, -0.05f, -0.05f),
                        vec3f(-0.05f, 0.05f, -0.05f),
                        vec3f(-0.05f, -0.05f, 0.05f)),
    };
    auto output = build_closed_mesh_from_triangles(input);
    expect(!output.empty(),
           "small mesh should be auto-scaled and produce output");
    expect(all_vertices_finite(output),
           "all output vertices should be finite after auto-scaling");
}

// 4. 手动指定中心（偏移到非原点）
static void test_manual_center() {
    std::vector<Triangle> input = {
        std::make_tuple(vec3f(10.2f, 10.2f, 10.0f),
                        vec3f(9.8f, 10.2f, 10.0f),
                        vec3f(10.0f, 9.8f, 10.0f))};
    vec3f manual_center(10.0f, 10.0f, 9.0f);
    auto output =
        build_closed_mesh_from_triangles(input, false, manual_center);
    expect(!output.empty(),
           "manual center should produce non-empty mesh");
    expect(all_vertices_finite(output),
           "all output vertices should be finite with manual center");
    expect(has_vertex_near(output, manual_center),
           "output should contain the manual center vertex");
}

// 5. 近重复三角形去重（不应崩溃）
static void test_near_duplicate_deduplication() {
    // 两个几乎相同的四面体，验证去重不会崩溃且输出正常
    std::vector<Triangle> input = {
        std::make_tuple(vec3f(1.0f, 1.0f, 1.0f),
                        vec3f(1.0f, -1.0f, -1.0f),
                        vec3f(-1.0f, 1.0f, -1.0f)),
        std::make_tuple(vec3f(1.0f, 1.0f, 1.0f),
                        vec3f(1.0f, -1.0f, -1.0f),
                        vec3f(-1.0f, -1.0f, 1.0f)),
        std::make_tuple(vec3f(1.0f, 1.0f, 1.0f),
                        vec3f(-1.0f, 1.0f, -1.0f),
                        vec3f(-1.0f, -1.0f, 1.0f)),
        std::make_tuple(vec3f(1.0f, -1.0f, -1.0f),
                        vec3f(-1.0f, 1.0f, -1.0f),
                        vec3f(-1.0f, -1.0f, 1.0f)),
        // 第二个几乎相同的四面体，顶点微移
        std::make_tuple(vec3f(1.0000001f, 1.0000001f, 1.0000001f),
                        vec3f(1.0000001f, -1.0f, -1.0f),
                        vec3f(-1.0f, 1.0000001f, -1.0f)),
        std::make_tuple(vec3f(1.0000001f, 1.0000001f, 1.0000001f),
                        vec3f(1.0000001f, -1.0f, -1.0f),
                        vec3f(-1.0f, -1.0f, 1.0000001f)),
        std::make_tuple(vec3f(1.0000001f, 1.0000001f, 1.0000001f),
                        vec3f(-1.0f, 1.0000001f, -1.0f),
                        vec3f(-1.0f, -1.0f, 1.0000001f)),
        std::make_tuple(vec3f(1.0000001f, -1.0f, -1.0f),
                        vec3f(-1.0f, 1.0000001f, -1.0f),
                        vec3f(-1.0f, -1.0f, 1.0000001f)),
    };
    auto output = build_closed_mesh_from_triangles(input);
    expect(!output.empty(),
           "near-duplicate triangles should be handled");
    expect(all_vertices_finite(output),
           "all output vertices should be finite with near-duplicates");
}

// 6. 封闭四面体
static void test_closed_tetrahedron() {
    std::vector<Triangle> input = {
        std::make_tuple(vec3f(1.0f, 1.0f, 1.0f),
                        vec3f(1.0f, -1.0f, -1.0f),
                        vec3f(-1.0f, 1.0f, -1.0f)),
        std::make_tuple(vec3f(1.0f, 1.0f, 1.0f),
                        vec3f(1.0f, -1.0f, -1.0f),
                        vec3f(-1.0f, -1.0f, 1.0f)),
        std::make_tuple(vec3f(1.0f, 1.0f, 1.0f),
                        vec3f(-1.0f, 1.0f, -1.0f),
                        vec3f(-1.0f, -1.0f, 1.0f)),
        std::make_tuple(vec3f(1.0f, -1.0f, -1.0f),
                        vec3f(-1.0f, 1.0f, -1.0f),
                        vec3f(-1.0f, -1.0f, 1.0f)),
    };
    auto output = build_closed_mesh_from_triangles(input);
    expect(!output.empty(),
           "closed tetrahedron should produce non-empty mesh");
    expect(all_vertices_finite(output),
           "all output vertices should be finite for tetrahedron");
}

// 7. 四棱锥（多个三角形组成的封闭体）
static void test_square_pyramid() {
    std::vector<Triangle> input = {
        // 侧面
        std::make_tuple(vec3f(0.0f, 2.0f, 0.0f),
                        vec3f(1.0f, 0.0f, 1.0f),
                        vec3f(-1.0f, 0.0f, 1.0f)),
        std::make_tuple(vec3f(0.0f, 2.0f, 0.0f),
                        vec3f(-1.0f, 0.0f, 1.0f),
                        vec3f(-1.0f, 0.0f, -1.0f)),
        std::make_tuple(vec3f(0.0f, 2.0f, 0.0f),
                        vec3f(-1.0f, 0.0f, -1.0f),
                        vec3f(1.0f, 0.0f, -1.0f)),
        std::make_tuple(vec3f(0.0f, 2.0f, 0.0f),
                        vec3f(1.0f, 0.0f, -1.0f),
                        vec3f(1.0f, 0.0f, 1.0f)),
        // 底面
        std::make_tuple(vec3f(1.0f, 0.0f, 1.0f),
                        vec3f(-1.0f, 0.0f, 1.0f),
                        vec3f(-1.0f, 0.0f, -1.0f)),
        std::make_tuple(vec3f(1.0f, 0.0f, 1.0f),
                        vec3f(-1.0f, 0.0f, -1.0f),
                        vec3f(1.0f, 0.0f, -1.0f)),
    };
    auto output = build_closed_mesh_from_triangles(input);
    expect(!output.empty(),
           "square pyramid should produce non-empty mesh");
    expect(all_vertices_finite(output),
           "all output vertices should be finite for square pyramid");
}

// 8. 加载 assets/test/block.stl 进行真实文件测试
static void test_load_block_stl() {
    // 尝试多个可能的相对路径（不同构建系统/运行目录）
    std::vector<std::string> candidates = {
        "assets/test/block.stl",
        "../assets/test/block.stl",
        "../../assets/test/block.stl",
    };
    std::vector<Triangle> input;
    std::string path;
    for (const auto& c : candidates) {
        input.clear();
        bool ok = false;
        try {
            for (auto [tri, normal] : sinriv::kigstudio::voxel::readSTL(c)) {
                input.push_back(tri);
                ok = true;
            }
        } catch (...) {
            ok = false;
        }
        if (ok && !input.empty()) {
            path = c;
            break;
        }
    }
    expect(!input.empty(), "block.stl should contain triangles");

    auto output = build_closed_mesh_from_triangles(input, false, vec3f(0.0f, 10.0f, 0.0f));
    expect(!output.empty(),
           "block.stl should produce non-empty mesh after cone-box");
    expect(all_vertices_finite(output),
           "all output vertices should be finite for block.stl");
}

// ------------------------------------------------------------------
int main() {
    std::cout << "Running conebox tests..." << std::endl;

    test_empty_input();
    std::cout << "  [PASS] empty_input" << std::endl;

    test_single_triangle();
    std::cout << "  [PASS] single_triangle" << std::endl;

    test_auto_scale_small_mesh();
    std::cout << "  [PASS] auto_scale_small_mesh" << std::endl;

    test_manual_center();
    std::cout << "  [PASS] manual_center" << std::endl;

    test_near_duplicate_deduplication();
    std::cout << "  [PASS] near_duplicate_deduplication" << std::endl;

    test_closed_tetrahedron();
    std::cout << "  [PASS] closed_tetrahedron" << std::endl;

    test_square_pyramid();
    std::cout << "  [PASS] square_pyramid" << std::endl;

    test_load_block_stl();
    std::cout << "  [PASS] load_block_stl" << std::endl;

    std::cout << "All tests passed!" << std::endl;
    return 0;
}
