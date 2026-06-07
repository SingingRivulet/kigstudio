#include "test_common.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>

#include "kigstudio/mesh/conebvh.h"

using namespace sinriv::kigstudio::mesh::conebvh;

static void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

int main() {
    setup_test_environment();
    const vec3f apex(0.0f, 0.0f, 0.0f);
    constexpr float pi = 3.14159265358979323846f;
    std::vector<Cone> cones;
    cones.emplace_back(vec3f(0.0f, 0.0f, 1.0f), pi / 6.0f, 2.0f);
    cones.emplace_back(vec3f(0.0f, 1.0f, 0.0f), pi / 6.0f, 2.0f);

    ConeBVHTree tree;
    tree.build(cones, apex);

    // 确认 BVH 根节点存在
    expect(tree.root >= 0, "BVH should have a valid root");
    expect(!tree.nodes.empty(), "BVH nodes should not be empty");
    expect(tree.primitives.size() == cones.size(), "BVH should preserve all primitives");

    bool found_in_z = false;
    tree.query_point_strict(vec3f(0.0f, 0.0f, 1.0f), [&](int idx) {
        if (idx == 0)
            found_in_z = true;
    });
    expect(found_in_z, "Point on cone Z axis should be contained by cone 0");

    bool found_in_y = false;
    tree.query_point_strict(vec3f(0.0f, 1.0f, 0.0f), [&](int idx) {
        if (idx == 1)
            found_in_y = true;
    });
    expect(found_in_y, "Point on cone Y axis should be contained by cone 1");

    bool found_outside = false;
    tree.query_point_strict(vec3f(1.0f, 1.0f, 1.0f), [&](int) {
        found_outside = true;
    });
    expect(!found_outside, "Point outside both cones should not be contained");

    int nearest_idx = -1;
    float nearest_dist = 0.0f;
    bool nearest_found = tree.query_nearest(vec3f(0.0f, 0.0f, 0.5f), nearest_idx, nearest_dist);
    expect(nearest_found, "Nearest cone query should find a valid primitive");
    expect(nearest_idx == 0, "Nearest cone to a point near the z-axis should be cone 0");
    expect(nearest_dist < 0.0f, "Point inside cone 0 should return a negative signed distance");

    int nearest_idx_y = -1;
    float nearest_dist_y = 0.0f;
    bool nearest_found_y = tree.query_nearest(vec3f(0.0f, 0.9f, 0.5f), nearest_idx_y, nearest_dist_y);
    expect(nearest_found_y, "Nearest cone query should find a valid primitive near the y-axis");
    expect(nearest_idx_y == 1, "Nearest cone to a point near the y-axis should be cone 1");
    expect(nearest_dist_y < 0.0f, "Point inside cone 1 should return a negative signed distance");

    {
        std::vector<TetraConeBVHTree::triangle> tetraBases;
        tetraBases.emplace_back(vec3f(1.0f, 0.0f, 0.0f),
                                vec3f(0.0f, 1.0f, 0.0f),
                                vec3f(0.0f, 0.0f, 1.0f));

        TetraConeBVHTree tetra_tree;
        tetra_tree.build(tetraBases, apex);

        int tetra_idx = -1;
        float tetra_dist = 0.0f;
        bool tetra_found = tetra_tree.query_nearest_tetra(vec3f(0.1f, 0.1f, 0.1f), tetra_idx, tetra_dist);
        expect(tetra_found, "Nearest tetra query should find a tetra candidate");
        expect(tetra_idx == 0, "Nearest tetra should be the only inserted tetra");
        expect(tetra_dist <= 0.0f, "Point inside the sample tetra should return a non-positive signed distance");

        std::vector<std::pair<int, float>> nearest_k;
        bool nearest_k_found = tetra_tree.query_nearest_tetra_k(vec3f(0.1f, 0.1f, 0.1f), 1, nearest_k);
        expect(nearest_k_found, "Nearest tetra K query should find candidates");
        expect(nearest_k.size() == 1, "Nearest tetra K query should return exactly one result for K=1");
        expect(nearest_k[0].first == 0, "Nearest tetra K query should return the only inserted tetra");
        expect(nearest_k[0].second <= 0.0f, "Nearest tetra K candidate should have a non-positive signed distance");
    }

    std::cout << "test_conebvh passed" << std::endl;
    return 0;
}
