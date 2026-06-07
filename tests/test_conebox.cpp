#include "test_common.h"
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

// ------------------------------------------------------------------
// Silhouette 算法专用测试
// ------------------------------------------------------------------

static void test_silhouette_empty_input() {
    std::vector<Triangle> input;
    auto output = build_closed_mesh_from_triangles_silhouette(
        input, vec3f(0.0f, 0.0f, 0.0f));
    expect(output.empty(), "silhouette empty input should produce empty output");
}

static void test_silhouette_single_triangle() {
    std::vector<Triangle> input = {
        std::make_tuple(vec3f(-0.2f, -0.2f, 1.0f),
                        vec3f(0.2f, -0.2f, 1.0f),
                        vec3f(0.0f, 0.2f, 1.0f))};
    vec3f center(0.0f, 10.0f, 0.0f);
    auto output = build_closed_mesh_from_triangles_silhouette(input, center);
    expect(!output.empty(),
           "silhouette single triangle should produce non-empty mesh");
    expect(all_vertices_finite(output),
           "all silhouette output vertices should be finite");
    expect(has_vertex_near(output, center),
           "silhouette output should contain the center vertex");
}

static void test_silhouette_tetrahedron() {
    // 标准四面体，center 在外部上方
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
    vec3f center(0.0f, 10.0f, 0.0f);
    auto output = build_closed_mesh_from_triangles_silhouette(input, center);
    expect(!output.empty(),
           "silhouette tetrahedron should produce non-empty mesh");
    expect(all_vertices_finite(output),
           "all silhouette output vertices should be finite");

    // 从外部上方看，面 2 和面 4 是最远可见面（共 2 个三角形）。
    // 它们共享 1 条边，边界边 = 2 + 2 = 4 条，生成 4 个侧面三角形。
    // 总输出 = 2 + 4 = 6。
    std::cout << "  silhouette tetrahedron output triangles: " << output.size()
              << std::endl;
    expect(output.size() == 6,
           "tetrahedron with external center should produce 2 visible + 4 side triangles");
}

static void test_silhouette_tetrahedron_internal_center() {
    // 标准四面体，center 在内部（近似重心）
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
    vec3f center(0.0f, 0.0f, 0.0f);
    auto output = build_closed_mesh_from_triangles_silhouette(input, center);
    expect(!output.empty(),
           "silhouette tetrahedron internal center should produce non-empty mesh");
    expect(all_vertices_finite(output),
           "all silhouette output vertices should be finite");

    // 中心在内部时，4 个面都可见；所有边被相邻可见面共享，无边界边，
    // 因此不需要侧面。总输出 = 4。
    std::cout << "  silhouette tetrahedron internal output triangles: " << output.size()
              << std::endl;
    expect(output.size() == 4,
           "tetrahedron with internal center should produce 4 visible triangles, no sides needed");
}

// 8. silhouette 算法：部分遮挡 — 近三角形遮挡远三角形的一部分
static void test_silhouette_partial_occlusion() {
    // T1: 近三角形（小）
    // T2: 远三角形（大，中间部分被 T1 遮挡）
    // T1 在 z=1，T2 在 z=2，center 在 z=0，避免共面导致射线检测失败
    std::vector<Triangle> input = {
        std::make_tuple(vec3f(0.0f, 0.0f, 1.0f),
                        vec3f(2.0f, 0.0f, 1.0f),
                        vec3f(1.0f, 1.0f, 1.0f)),   // T1: 近
        std::make_tuple(vec3f(-2.0f, 2.0f, 2.0f),
                        vec3f(4.0f, 2.0f, 2.0f),
                        vec3f(1.0f, 5.0f, 2.0f)),   // T2: 远
    };
    vec3f center(1.0f, -10.0f, 0.0f);
    auto output = build_closed_mesh_from_triangles_silhouette(input, center);
    expect(!output.empty(),
           "partial occlusion should produce non-empty mesh");
    expect(all_vertices_finite(output),
           "all output vertices should be finite");

    // 锥体裁剪把 T2 切成了多片，总输出应大于无裁剪时的 2 个可见面
    std::cout << "  partial occlusion output triangles: " << output.size()
              << std::endl;
    expect(output.size() > 2,
           "partial occlusion should produce more than 2 triangles");
}

// 9. silhouette 算法：block.stl + center=(0,10,0) 应得到房子形状
static void test_silhouette_block_stl() {
    std::vector<std::string> candidates = {
        "assets/test/block.stl",
        "../assets/test/block.stl",
        "../../assets/test/block.stl",
    };
    std::vector<Triangle> input;
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
        if (ok && !input.empty())
            break;
    }
    expect(!input.empty(), "block.stl should contain triangles");

    vec3f center(0.0f, 10.0f, 0.0f);
    auto output =
        build_closed_mesh_from_triangles_silhouette(input, center);
    expect(!output.empty(),
           "silhouette should produce non-empty mesh for block.stl");
    expect(all_vertices_finite(output),
           "all output vertices should be finite");

    // block.stl 是边长为 8 的立方体，center=(0,10,0) 在立方体上方。
    // silhouette 算法保留的是“从外部看离 center 最远的面”：
    //   - 侧面（±X, ±Z）的重心射线穿过该面后从立方体侧面穿出，
    //     所以这些面自身就是最远交点，共 8 个三角形。
    //   - +Y 面（顶面）被遮挡（射线继续向下会打到 -Y 面），不可见。
    //   - -Y 面（底面）是最远交点，共 2 个三角形。
    // 可见面总计 10 个。边界边是侧面与缺失顶面之间的 4 条上边缘，
    // 生成 4 个侧面三角形。所以总输出 = 10 + 4 = 14。
    std::cout << "  silhouette output triangles: " << output.size()
              << std::endl;
    expect(output.size() == 14,
           "cube with external center should produce 10 visible + 4 side triangles");

    // 检查侧面三角形都包含 center 点
    int side_with_center = 0;
    for (const auto& tri : output) {
        if (vec3_near(std::get<0>(tri), center) ||
            vec3_near(std::get<1>(tri), center) ||
            vec3_near(std::get<2>(tri), center)) {
            ++side_with_center;
        }
    }
    expect(side_with_center == 4,
           "should have exactly 4 side triangles containing center");
}

// 9. 加载 assets/test/block.stl 进行真实文件测试
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
    setup_test_environment();
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

    test_silhouette_empty_input();
    std::cout << "  [PASS] silhouette_empty_input" << std::endl;

    test_silhouette_single_triangle();
    std::cout << "  [PASS] silhouette_single_triangle" << std::endl;

    test_silhouette_tetrahedron();
    std::cout << "  [PASS] silhouette_tetrahedron" << std::endl;

    test_silhouette_tetrahedron_internal_center();
    std::cout << "  [PASS] silhouette_tetrahedron_internal_center" << std::endl;

    test_silhouette_partial_occlusion();
    std::cout << "  [PASS] silhouette_partial_occlusion" << std::endl;

    test_silhouette_block_stl();
    std::cout << "  [PASS] silhouette_block_stl" << std::endl;

    std::cout << "All tests passed!" << std::endl;
    return 0;
}
