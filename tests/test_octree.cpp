#include "kigstudio/mesh/octree.h"

using namespace sinriv::kigstudio::octree;
// int main() {
//     Octree octree(1024); // 假设空间边长为1024

//     Vec3i point1(100, 200, 300);
//     Vec3i point2(500, 600, 700);
//     // Vec3i point3(200, 600, 700);
//     // Vec3i point4(500, 300, 700);

//     octree.insert(point1);
//     octree.insert(point2);
//     // octree.insert(point3);
//     // octree.insert(point4);

//     std::cout << "Point1 found: " << octree.find(point1) << std::endl;
//     std::cout << "Point2 found: " << octree.find(point2) << std::endl;
//     std::cout << "Point (0,0,0) found: " << octree.find(Vec3i(0, 0, 0)) << std::endl;

//     std::cout<<"test1"<<std::endl;
//     for (const auto& point : octree) {
//         std::cout << "Intersection point: (" << point.x << ", " << point.y << ", " << point.z << ")\n";
//     }
//     std::cout<<"test2"<<std::endl;

//     for (const auto& point : octree) {
//         std::cout << "Intersection point: (" << point.x << ", " << point.y << ", " << point.z << ")\n";
//     }

//     std::cout<<"success"<<std::endl;

//     return 0;
// }

int main() {
    
    Octree tree1(16);
    Octree tree2(16);

    // 插入点
    tree1.insert({1, 1, 1});
    tree1.insert({2, 2, 2});
    tree1.insert({3, 3, 3});

    tree2.insert({2, 2, 2});
    tree2.insert({3, 3, 3});
    tree2.insert({4, 4, 4});

    // 交集
    Octree intersectionTree = tree1.intersection(tree2);
    for (auto it = intersectionTree.begin(); it != intersectionTree.end(); ++it) {
        auto point = *it;
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")\n";
    }

    // 测试并集
    Octree unionTree = tree1.unionWith(tree2);
    std::cout << "Union of tree1 and tree2:\n";
    for (auto it = unionTree.begin(); it != unionTree.end(); ++it) {
        Vec3i point = *it;
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")\n";
    }

    // 测试差集 tree1 - tree2
    Octree differenceTree = tree1.difference(tree2);
    std::cout << "\nDifference of tree1 - tree2:\n";
    for (auto it = differenceTree.begin(); it != differenceTree.end(); ++it) {
        Vec3i point = *it;
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")\n";
    }

    // 测试差集 tree2 - tree1
    Octree differenceTree2 = tree2.difference(tree1);
    std::cout << "\nDifference of tree2 - tree1:\n";
    for (auto it = differenceTree2.begin(); it != differenceTree2.end(); ++it) {
        Vec3i point = *it;
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")\n";
    }
    
    return 0;
}