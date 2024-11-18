#include "kigstudio/mesh/octree.h"

namespace sinriv::kigstudio::octree {
    void Octree::insert(std::unique_ptr<OctreeNode>& node, const Vec3i& point, const Vec3i& minCorner, const Vec3i& maxCorner) {
        if (!node) {
            node = std::make_unique<OctreeNode>(point);
            return;
        }

        // 如果是叶节点
        if (node->isLeaf) {
            if (node->position == point) return; // 已存在相同点，直接返回

            // 将叶节点转为内部节点
            Vec3i existingPoint = node->position;
            node->isLeaf = false;

            // 清空子节点
            for (auto& child : node->children) {
                child = nullptr;
            }

            // 计算当前节点的中心点
            Vec3i center = {
                (minCorner.x + maxCorner.x) / 2,
                (minCorner.y + maxCorner.y) / 2,
                (minCorner.z + maxCorner.z) / 2
            };

            // 重新插入已有的点
            int existingIndex = getChildIndex(existingPoint, center);
            Vec3i existingMinCorner = minCorner;
            Vec3i existingMaxCorner = maxCorner;

            // 确定已有点的子节点范围
            if (existingIndex & 1) existingMinCorner.x = center.x; else existingMaxCorner.x = center.x;
            if (existingIndex & 2) existingMinCorner.y = center.y; else existingMaxCorner.y = center.y;
            if (existingIndex & 4) existingMinCorner.z = center.z; else existingMaxCorner.z = center.z;

            insert(node->children[existingIndex], existingPoint, existingMinCorner, existingMaxCorner);
        }

        // 插入新点
        Vec3i center = {
            (minCorner.x + maxCorner.x) / 2,
            (minCorner.y + maxCorner.y) / 2,
            (minCorner.z + maxCorner.z) / 2
        };

        int childIndex = getChildIndex(point, center);
        Vec3i childMinCorner = minCorner;
        Vec3i childMaxCorner = maxCorner;

        // 确定新点的子节点范围
        if (childIndex & 1) childMinCorner.x = center.x; else childMaxCorner.x = center.x;
        if (childIndex & 2) childMinCorner.y = center.y; else childMaxCorner.y = center.y;
        if (childIndex & 4) childMinCorner.z = center.z; else childMaxCorner.z = center.z;

        insert(node->children[childIndex], point, childMinCorner, childMaxCorner);
    }


    // 查询点的递归实现
    bool Octree::find(const std::unique_ptr<OctreeNode>& node, const Vec3i& point, const Vec3i& minCorner, const Vec3i& maxCorner) const {
        if (!node) return false;

        if (node->isLeaf) {
            return node->position == point;
        }

        Vec3i center = {
            (minCorner.x + maxCorner.x) / 2,
            (minCorner.y + maxCorner.y) / 2,
            (minCorner.z + maxCorner.z) / 2
        };

        int childIndex = getChildIndex(point, center);
        Vec3i childMinCorner = minCorner;
        Vec3i childMaxCorner = maxCorner;

        if (childIndex & 1) childMinCorner.x = center.x; else childMaxCorner.x = center.x;
        if (childIndex & 2) childMinCorner.y = center.y; else childMaxCorner.y = center.y;
        if (childIndex & 4) childMinCorner.z = center.z; else childMaxCorner.z = center.z;

        return find(node->children[childIndex], point, childMinCorner, childMaxCorner);
    }
}