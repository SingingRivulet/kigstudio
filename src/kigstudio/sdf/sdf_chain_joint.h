#pragma once
#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>
#include "kigstudio/sdf/sdf.h"
#include "kigstudio/utils/vec3.h"

namespace sinriv::kigstudio::sdf::joint {
/*
 *   链条关节结构及参数：
 *      中心线：
 *           起点（一定位于skeleton point上）
 *           终点（一定位于skeleton point上）
 *           起点和终点构成一根直线
 *      关节窝：
 *           切割圆锥 用于切割出关节窝的圆锥，需要三个参数定义：
 *               距离中心线起点的距离
 *               圆锥张开的角度
 *               底面半径
 *               *圆锥开口方向一定指向终点，所以无需定义*
 *           实体圆锥 将关节窝后面填充一定范围，以增加强度：
 *               距离关节窝切割圆锥顶点的距离（实体圆锥顶点位于切割圆锥顶点与中心线起点之间）
 *               底面半径
 *               *实体圆锥表面与切割圆锥平行，所以其他参数无需定义*
 *           连接柱（公）
 * 一个圆柱，与圆锥底面平行，轴穿过中心线，两边在与切割圆锥相交处终止（所以底面不是正圆），需要三个参数定义：
 *               距离关节窝切割圆锥顶点的距离（连接柱顶点位于切割圆锥顶点与中心线起点之间）
 *               圆柱底面半径
 *               旋转角度关节连接柱的旋转角度（一个向量，从中心线上一点往这个方向作射线，圆柱的轴位于该射线与中心线围成的平面上）
 *      关节头：
 *           切割圆锥 用于切割出关节头的圆锥，需要两个参数定义：
 *               距离关节窝切割圆锥的距离
 *               底面半径
 *               *张开的角度一定等于关节窝切割圆锥的角度，所以无需定义*
 *               *圆锥开口方向一定指向终点，所以无需定义*
 *           实体圆锥 将关节头后面填充一定范围，以增加强度：
 *               距离关节头切割圆锥顶点的距离（实体圆锥顶点位于切割圆锥顶点后方）
 *               底面半径
 *               *实体圆锥表面与切割圆锥平行，所以其他参数无需定义*
 *           连接柱（母） 切割一个圆柱，离公连接柱有一定距离，使其能活动
 *               半径（通过与公连接柱的差值来定义）
 *               *轴和角度无需定义（和公连接柱共用轴）*
 *      连接槽：
 *           在关节头和关节窝的切割圆锥之间进行切割一个带厚度的锥形，便于活动
 *           没有可设置的参数，形状取决于关节头的切割圆锥和关节窝的切割圆锥
 *   构造链条流程：
 *       1. 构造负mesh
 *           母连接柱
 *           两个切割圆锥之间的区域
 *       2. 构造正mesh
 *           公连接柱
 *           两个切割圆锥分别与各自实体圆锥之间构成的区域
 *       3.
 * 利用射线追踪算法对体素执行布尔，先执行负mesh，再执行正mesh，被切掉的部分直接丢弃，不需要返回新的item
 */
// ============================================================
// Basic Vec3
// ============================================================

using Vec3f = sinriv::kigstudio::vec3<float>;

// ============================================================
// Local Frame
// ============================================================

struct Frame {
    Vec3f origin;

    Vec3f x_axis;
    Vec3f y_axis;
    Vec3f z_axis;

    inline Vec3f worldToLocal(const Vec3f& p) const {
        Vec3f d = p - origin;

        return {dot(d, x_axis), dot(d, y_axis), dot(d, z_axis)};
    }

    inline Vec3f localToWorld(const Vec3f& p) const {
        return origin + x_axis * p.x + y_axis * p.y + z_axis * p.z;
    }
};

Frame buildFrame(const Vec3f& start,
                 const Vec3f& end,
                 float rotation_angle_rad);

// Build a frame whose z-axis points from start to end,
// and x-axis aligns with the world +Y axis (projected onto the plane
// perpendicular to z). This makes the male cylinder point toward +Y.
Frame buildFrameAlignedY(const Vec3f& start, const Vec3f& end);

// ============================================================
// SDF helpers
// ============================================================

float sdCappedCylinder(const Vec3f& p, float radius, float half_height);

// Capped cylinder aligned with the local x-axis.
// The cylinder axis passes through the origin and points along +x.
float sdCappedCylinderX(const Vec3f& p, float radius, float half_height);

// Infinite cone with vertex at origin, opening along +z.
// angle_rad is the half-opening angle.
float sdCone(const Vec3f& p, float angle_rad);

float opUnion(float a, float b);

float opSubtraction(float a, float b);

float opIntersection(float a, float b);

// Finite cone: vertex at origin, opening along +z, truncated at z = height.
// The base radius at the truncation plane is height * tan(angle_rad).
float sdFiniteCone(const Vec3f& p, float angle_rad, float height);

// ============================================================
// SDF Primitives (inherited from SDFBase)
// ============================================================

struct SDF_FiniteCone : public sinriv::kigstudio::sdf::SDFBase {
    float angle_rad;
    float height;

    inline SDF_FiniteCone(float angle_rad, float height)
        : angle_rad(angle_rad), height(height) {}

    float get(const Vec3f& p) const override;
};

struct SDF_CappedCylinderX : public sinriv::kigstudio::sdf::SDFBase {
    float radius;
    float half_height;

    inline SDF_CappedCylinderX(float radius, float half_height)
        : radius(radius), half_height(half_height) {}

    float get(const Vec3f& p) const override;
};

struct SDF_FrameTransform : public sinriv::kigstudio::sdf::SDFBase {
    Frame frame;
    std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> child;

    inline SDF_FrameTransform(
        const Frame& frame,
        std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> child)
        : frame(frame), child(std::move(child)) {}

    float get(const Vec3f& p) const override;
};

// ============================================================
// Negative Joint Volume
// ============================================================

class JointNegativeSDF : public sinriv::kigstudio::sdf::SDFBase {
   public:
    Frame frame;

    // socket cutting cone
    float socket_cone_offset = 5.f;
    float socket_cone_angle = 0.5f;
    float socket_cone_radius = 4.f;

    // female cylinder clearance (adds to male radius)
    float female_gap = 0.3f;

    // male cylinder source (shared axis/angle for female cylinder)
    float male_cylinder_offset = 3.f;
    float male_cylinder_radius = 1.5f;
    float male_cylinder_half_height = 10.f;

    // head cutting cone
    float head_cone_offset = 10.f;
    float head_cone_radius = 3.5f;

    // slot thickness between socket and head cones
    float slot_extra = 0.5f;

    std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> buildTree() const;
    float get(const Vec3f& world_p) const override;

    inline bool contains(const Vec3f& p) const { return get(p) <= 0.f; }
};

// ============================================================
// Positive Joint Volume
// ============================================================

class JointPositiveSDF : public sinriv::kigstudio::sdf::SDFBase {
   public:
    Frame frame;

    // socket support cone (fills behind socket cutting cone)
    float socket_support_offset = 2.f;
    float socket_support_angle = 0.5f;
    float socket_support_radius = 5.f;

    // head support cone (fills behind head cutting cone)
    float head_support_offset = 12.f;
    float head_support_angle = 0.5f;
    float head_support_radius = 5.f;

    // male cylinder
    float male_cylinder_offset = 3.f;
    float male_cylinder_radius = 1.5f;
    float male_cylinder_half_height = 1000.f;

    inline float effectiveHalfHeight() const {
        return std::min(male_cylinder_half_height,
                        socket_support_radius * 4.0f);
    }

    std::shared_ptr<sinriv::kigstudio::sdf::SDFBase> buildTree() const;
    float get(const Vec3f& world_p) const override;

    inline bool contains(const Vec3f& p) const { return get(p) <= 0.f; }
};

// ============================================================
// Joint Wireframe Helpers
// ============================================================

void appendJointWireframe(std::vector<std::pair<Vec3f, Vec3f>>& segments,
                          const JointNegativeSDF& neg,
                          const JointPositiveSDF& pos);

}  // namespace sinriv::kigstudio::sdf::joint
