#include "kigstudio/voxel/collision.h"
#include <iostream>

using namespace sinriv::kigstudio::voxel::collision;

namespace {
bool expectTrue(bool value, const char* message) {
    if (!value) {
        std::cerr << "[FAIL] " << message << std::endl;
        return false;
    }
    std::cout << "[PASS] " << message << std::endl;
    return true;
}

bool expectFalse(bool value, const char* message) {
    return expectTrue(!value, message);
}

bool testSphere() {
    Sphere sphere{{0.0f, 0.0f, 0.0f}, 2.0f};

    bool ok = true;
    ok &= expectTrue(sphere.contains({0.0f, 0.0f, 0.0f}), "sphere contains center");
    ok &= expectTrue(sphere.contains({2.0f, 0.0f, 0.0f}), "sphere contains boundary point");
    ok &= expectFalse(sphere.contains({2.1f, 0.0f, 0.0f}), "sphere rejects outside point");

    ok &= expectTrue(pointIntersects({1.0f, 1.0f, 0.0f}, sphere), "pointIntersects works for sphere");
    ok &= expectFalse(pointIntersects({3.0f, 0.0f, 0.0f}, sphere), "pointIntersects rejects sphere miss");
    return ok;
}

bool testCylinder() {
    Cylinder cylinder{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 4.0f}, 1.0f};

    bool ok = true;
    ok &= expectTrue(cylinder.contains({0.5f, 0.0f, 2.0f}), "cylinder contains inner point");
    ok &= expectTrue(cylinder.contains({1.0f, 0.0f, 3.0f}), "cylinder contains radial boundary");
    ok &= expectFalse(cylinder.contains({1.1f, 0.0f, 2.0f}), "cylinder rejects radial outside point");
    ok &= expectFalse(cylinder.contains({0.0f, 0.0f, 4.1f}), "cylinder rejects cap outside point");

    ok &= expectTrue(pointIntersects({0.0f, 1.0f, 1.5f}, cylinder), "pointIntersects works for cylinder");
    ok &= expectFalse(pointIntersects({0.0f, 1.1f, 1.5f}, cylinder), "pointIntersects rejects cylinder miss");
    return ok;
}

bool testCapsule() {
    Capsule capsule{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 4.0f}, 1.0f};

    bool ok = true;
    ok &= expectTrue(capsule.contains({0.0f, 0.0f, -0.5f}), "capsule contains lower hemisphere point");
    ok &= expectTrue(capsule.contains({0.0f, 1.0f, 2.0f}), "capsule contains cylinder boundary point");
    ok &= expectTrue(capsule.contains({0.0f, 0.0f, 4.5f}), "capsule contains upper hemisphere point");
    ok &= expectFalse(capsule.contains({0.0f, 0.0f, 5.1f}), "capsule rejects far outside point");

    ok &= expectTrue(pointIntersects({0.5f, 0.5f, 0.0f}, capsule), "pointIntersects works for capsule");
    ok &= expectFalse(pointIntersects({1.1f, 0.0f, 2.0f}, capsule), "pointIntersects rejects capsule miss");
    return ok;
}

bool testOBB() {
    OBB obb;
    obb.center = {0.0f, 0.0f, 0.0f};
    obb.half_extent = {2.0f, 1.0f, 1.0f};
    obb.axis_x = vec3f(1.0f, 1.0f, 0.0f).normalize();
    obb.axis_y = vec3f(-1.0f, 1.0f, 0.0f).normalize();
    obb.axis_z = {0.0f, 0.0f, 1.0f};

    const vec3f inside = obb.center + obb.axis_x * 1.5f + obb.axis_y * 0.5f + obb.axis_z * 0.5f;
    const vec3f boundary = obb.center + obb.axis_x * 2.0f;
    const vec3f outside = obb.center + obb.axis_x * 2.1f;

    bool ok = true;
    ok &= expectTrue(obb.contains(inside), "obb contains inner point");
    ok &= expectTrue(obb.contains(boundary), "obb contains boundary point");
    ok &= expectFalse(obb.contains(outside), "obb rejects outside point");

    ok &= expectTrue(pointIntersects(inside, obb), "pointIntersects works for obb");
    ok &= expectFalse(pointIntersects(outside, obb), "pointIntersects rejects obb miss");
    return ok;
}
}  // namespace

int main() {
    bool ok = true;
    ok &= testSphere();
    ok &= testCylinder();
    ok &= testCapsule();
    ok &= testOBB();

    if (!ok) {
        std::cerr << "collision tests failed" << std::endl;
        return 1;
    }

    std::cout << "collision tests passed" << std::endl;
    return 0;
}
