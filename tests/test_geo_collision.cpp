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

bool nearlyEqual(float a, float b, float eps = 1e-4f) {
    return std::fabs(a - b) <= eps;
}

bool nearlyEqualVec3(const vec3f& a, const vec3f& b, float eps = 1e-4f) {
    return nearlyEqual(a.x, b.x, eps) &&
           nearlyEqual(a.y, b.y, eps) &&
           nearlyEqual(a.z, b.z, eps);
}

bool testTransformRoundTrip() {
    Transform transform;
    transform.setPosition({1.0f, 2.0f, 3.0f});
    transform.setRotationEuler({0.0f, 0.0f, 3.14159265358979323846f * 0.5f});
    transform.setScale({2.0f, 3.0f, 4.0f});

    const mat4f matrix = transform.getMatrix();

    Transform restored;
    restored.setMatrix(matrix);

    bool ok = true;
    ok &= expectTrue(nearlyEqualVec3(restored.getPosition(), {1.0f, 2.0f, 3.0f}),
                     "transform restores position from matrix");
    ok &= expectTrue(nearlyEqualVec3(restored.getScale(), {2.0f, 3.0f, 4.0f}),
                     "transform restores scale from matrix");

    const vec3f euler = restored.getRotationEuler();
    ok &= expectTrue(nearlyEqual(euler.z, 3.14159265358979323846f * 0.5f),
                     "transform restores euler rotation from matrix");
    return ok;
}

bool testCollisionGroupGlobalAndLocalTransform() {
    CollisionGroup group;
    group.setPosition({5.0f, 0.0f, 0.0f});

    Transform local_sphere;
    local_sphere.setPosition({2.0f, 0.0f, 0.0f});
    group.add(Sphere{{0.0f, 0.0f, 0.0f}, 1.0f}, local_sphere);

    bool ok = true;
    ok &= expectTrue(group.containsWorldPoint({7.0f, 0.0f, 0.0f}),
                     "collision group applies global and local translation");
    ok &= expectTrue(pointIntersectsWorld({6.5f, 0.0f, 0.0f}, group),
                     "pointIntersects works for collision group");
    ok &= expectFalse(group.containsWorldPoint({8.2f, 0.0f, 0.0f}),
                      "collision group rejects point outside translated geometry");
    return ok;
}

bool testCollisionGroupQuaternionRotation() {
    CollisionGroup group;
    group.setRotationQuaternion(quaternionFromAxisAngle({{0.0f, 0.0f, 1.0f},
                                                         3.14159265358979323846f * 0.5f}));

    group.add(Cylinder{{0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, 0.5f});

    bool ok = true;
    ok &= expectTrue(group.containsWorldPoint({0.0f, 1.0f, 0.0f}),
                     "quaternion rotation rotates geometry group");
    ok &= expectFalse(group.containsWorldPoint({1.0f, 0.6f, 0.0f}),
                      "quaternion rotation rejects miss after rotation");
    return ok;
}

bool testCollisionGroupAxisAngleAndScale() {
    CollisionGroup group;
    group.setRotationAxisAngle({{0.0f, 0.0f, 1.0f}, 3.14159265358979323846f * 0.5f});
    group.setScale({2.0f, 1.0f, 1.0f});

    Transform local_box;
    local_box.setPosition({1.0f, 0.0f, 0.0f});
    group.add(OBB{{0.0f, 0.0f, 0.0f},
                  {0.5f, 0.5f, 0.5f},
                  {1.0f, 0.0f, 0.0f},
                  {0.0f, 1.0f, 0.0f},
                  {0.0f, 0.0f, 1.0f}},
              local_box);

    bool ok = true;
    ok &= expectTrue(group.containsWorldPoint({0.0f, 2.0f, 0.0f}),
                     "axis-angle rotation and scale affect collision group");
    ok &= expectFalse(group.containsWorldPoint({0.8f, 2.0f, 0.0f}),
                      "axis-angle rotation and scale still reject outside point");
    return ok;
}
}  // namespace

int main() {
    bool ok = true;
    ok &= testSphere();
    ok &= testCylinder();
    ok &= testCapsule();
    ok &= testOBB();
    ok &= testTransformRoundTrip();
    ok &= testCollisionGroupGlobalAndLocalTransform();
    ok &= testCollisionGroupQuaternionRotation();
    ok &= testCollisionGroupAxisAngleAndScale();

    if (!ok) {
        std::cerr << "collision tests failed" << std::endl;
        return 1;
    }

    std::cout << "collision tests passed" << std::endl;
    return 0;
}
