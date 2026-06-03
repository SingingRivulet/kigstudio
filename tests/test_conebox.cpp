// #include "kigstudio/mesh/conebox.h"
// #include "kigstudio/utils/vec3.h"
// #include <iostream>
// #include <iomanip>
// #include <cmath>

// using namespace sinriv::kigstudio::mesh::conebox;
// using namespace sinriv::kigstudio;

// // 简单的测试：创建一个在立方体一个面外的三角形，验证侧面生成
// void test_simple_triangle_with_cone_sides() {
//     std::cout << "\n=== Test: Simple Triangle with Cone Sides ===" << std::endl;
    
//     // 立方体中心
//     vec3<float> center(0.0f, 0.0f, 0.0f);
    
//     // 创建一个在+Z面外的三角形
//     // 立方体上表面在 z = 0.5
//     // 三角形在 z = 1.0（在立方体外）
//     Triangle_status status;
//     status.triangle = {
//         vec3<float>(-0.2f, -0.2f, 1.0f),
//         vec3<float>(0.2f, -0.2f, 1.0f),
//         vec3<float>(0.0f, 0.2f, 1.0f)
//     };
    
//     // 计算投影
//     status.compute_projection(center);
    
//     // 创建Triangle_group并添加三角形
//     Triangle_group group;
//     group.center = center;
//     group.add_triangle(status);
//     group.build_face_trees();
    
//     // 获取基础可见网格
//     auto visible_mesh = group.compute_visible_mesh_from_outside();
//     std::cout << "Base visible mesh triangles: " << visible_mesh.size() << std::endl;
    
//     // 获取带有锥体侧面的网格
//     auto mesh_with_sides = group.compute_visible_mesh_with_cone_sides();
//     std::cout << "Mesh with cone sides triangles: " << mesh_with_sides.size() << std::endl;
    
//     // 验证
//     if (mesh_with_sides.size() > visible_mesh.size()) {
//         std::cout << "✓ Successfully added " 
//                   << (mesh_with_sides.size() - visible_mesh.size()) 
//                   << " side triangles" << std::endl;
//     } else {
//         std::cout << "✗ No side triangles were added" << std::endl;
//     }
// }

// // 测试：多个三角形的情况
// void test_multiple_triangles_with_cone_sides() {
//     std::cout << "\n=== Test: Multiple Triangles with Cone Sides ===" << std::endl;
    
//     vec3<float> center(0.0f, 0.0f, 0.0f);
//     Triangle_group group;
//     group.center = center;
    
//     // 添加3个三角形，分别在不同的面外
//     Triangle_status triangles[3];
    
//     // 第一个三角形：在+Z面外
//     triangles[0].triangle = {
//         vec3<float>(-0.15f, -0.15f, 1.0f),
//         vec3<float>(0.15f, -0.15f, 1.0f),
//         vec3<float>(0.0f, 0.15f, 1.0f)
//     };
    
//     // 第二个三角形：在+X面外
//     triangles[1].triangle = {
//         vec3<float>(1.0f, -0.15f, -0.15f),
//         vec3<float>(1.0f, 0.15f, -0.15f),
//         vec3<float>(1.0f, 0.0f, 0.15f)
//     };
    
//     // 第三个三角形：在+Y面外
//     triangles[2].triangle = {
//         vec3<float>(-0.15f, 1.0f, -0.15f),
//         vec3<float>(0.15f, 1.0f, -0.15f),
//         vec3<float>(0.0f, 1.0f, 0.15f)
//     };
    
//     for (auto& tri : triangles) {
//         tri.compute_projection(center);
//         group.add_triangle(tri);
//     }
//     group.build_face_trees();
    
//     auto visible_mesh = group.compute_visible_mesh_from_outside();
//     auto mesh_with_sides = group.compute_visible_mesh_with_cone_sides();
    
//     std::cout << "Base visible mesh triangles: " << visible_mesh.size() << std::endl;
//     std::cout << "Mesh with cone sides triangles: " << mesh_with_sides.size() << std::endl;
    
//     if (mesh_with_sides.size() > visible_mesh.size()) {
//         std::cout << "✓ Successfully added " 
//                   << (mesh_with_sides.size() - visible_mesh.size()) 
//                   << " side triangles" << std::endl;
//     }
// }

// // 测试：共线边界的处理（防止非流形边）
// void test_collinear_boundary_handling() {
//     std::cout << "\n=== Test: Collinear Boundary Handling ===" << std::endl;
    
//     vec3<float> center(0.0f, 0.0f, 0.0f);
//     Triangle_group group;
//     group.center = center;
    
//     // 创建一个三角形，其一条边的两个端点到中心的方向相同
//     // 这模拟了一条共线的边
//     Triangle_status status;
    
//     // 三个顶点都在从中心出发的同一条射线上
//     float r = 1.0f;  // 距离
//     status.triangle = {
//         vec3<float>(r * 0.6f, 0.0f, 0.8f),
//         vec3<float>(r * 0.75f, 0.0f, 0.66f),
//         vec3<float>(0.0f, 0.0f, 1.0f)
//     };
    
//     // 标准化以满足共线条件
//     for (auto& v : {status.triangle.get<0>(), status.triangle.get<1>(), status.triangle.get<2>()}) {
//         auto dir = v - center;
//         float len = dir.length();
//         if (len > 1e-6f) {
//             v = center + (dir / len) * r;  // 放置在同一条射线上
//         }
//     }
    
//     status.compute_projection(center);
//     group.add_triangle(status);
//     group.build_face_trees();
    
//     auto visible_mesh = group.compute_visible_mesh_from_outside();
//     auto mesh_with_sides = group.compute_visible_mesh_with_cone_sides();
    
//     std::cout << "Base visible mesh triangles: " << visible_mesh.size() << std::endl;
//     std::cout << "Mesh with cone sides triangles: " << mesh_with_sides.size() << std::endl;
//     std::cout << "✓ Collinear boundary handling test completed" << std::endl;
// }

// // 测试：去重检查
// void test_deduplication() {
//     std::cout << "\n=== Test: Deduplication ===" << std::endl;
    
//     vec3<float> center(0.0f, 0.0f, 0.0f);
//     Triangle_group group;
//     group.center = center;
    
//     // 创建一个简单的三角形
//     Triangle_status status;
//     status.triangle = {
//         vec3<float>(-0.1f, -0.1f, 1.0f),
//         vec3<float>(0.1f, -0.1f, 1.0f),
//         vec3<float>(0.0f, 0.1f, 1.0f)
//     };
    
//     status.compute_projection(center);
//     group.add_triangle(status);
//     group.build_face_trees();
    
//     // 调用两次，检查是否有重复的三角形
//     auto mesh1 = group.compute_visible_mesh_with_cone_sides();
//     auto mesh2 = group.compute_visible_mesh_with_cone_sides();
    
//     std::cout << "First call: " << mesh1.size() << " triangles" << std::endl;
//     std::cout << "Second call: " << mesh2.size() << " triangles" << std::endl;
    
//     if (mesh1.size() == mesh2.size()) {
//         std::cout << "✓ Deduplication working correctly" << std::endl;
//     }
// }

// // 测试：侧面三角形的几何有效性
// void test_cone_triangle_geometry() {
//     std::cout << "\n=== Test: Cone Triangle Geometry ===" << std::endl;
    
//     vec3<float> center(0.0f, 0.0f, 0.0f);
//     Triangle_group group;
//     group.center = center;
    
//     Triangle_status status;
//     status.triangle = {
//         vec3<float>(-0.2f, -0.2f, 1.0f),
//         vec3<float>(0.2f, -0.2f, 1.0f),
//         vec3<float>(0.0f, 0.2f, 1.0f)
//     };
    
//     status.compute_projection(center);
//     group.add_triangle(status);
//     group.build_face_trees();
    
//     auto mesh = group.compute_visible_mesh_with_cone_sides();
    
//     // 检查所有三角形中是否有中心点
//     int side_triangles_with_center = 0;
//     const float epsilon = 1e-6f;
    
//     for (const auto& tri : mesh) {
//         const auto& v0 = std::get<0>(tri);
//         const auto& v1 = std::get<1>(tri);
//         const auto& v2 = std::get<2>(tri);
        
//         // 检查是否有顶点等于中心（允许浮点误差）
//         auto is_center = [&](const vec3<float>& v) {
//             return (std::abs(v.x - center.x) < epsilon &&
//                     std::abs(v.y - center.y) < epsilon &&
//                     std::abs(v.z - center.z) < epsilon);
//         };
        
//         if (is_center(v0) || is_center(v1) || is_center(v2)) {
//             side_triangles_with_center++;
//         }
//     }
    
//     std::cout << "Total triangles: " << mesh.size() << std::endl;
//     std::cout << "Side triangles containing center: " << side_triangles_with_center << std::endl;
    
//     if (side_triangles_with_center > 0) {
//         std::cout << "✓ Cone triangles have correct geometry (connected to center)" << std::endl;
//     }
// }

int main() {
//     std::cout << std::fixed << std::setprecision(4);
//     std::cout << "========================================" << std::endl;
//     std::cout << "   Test: compute_visible_mesh_with_cone_sides" << std::endl;
//     std::cout << "========================================" << std::endl;
    
//     test_simple_triangle_with_cone_sides();
//     test_multiple_triangles_with_cone_sides();
//     test_collinear_boundary_handling();
//     test_deduplication();
//     test_cone_triangle_geometry();
    
//     std::cout << "\n========================================" << std::endl;
//     std::cout << "   All tests completed" << std::endl;
//     std::cout << "========================================" << std::endl;
    
//     return 0;
}
