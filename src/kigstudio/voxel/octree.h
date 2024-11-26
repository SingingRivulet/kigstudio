#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <stack>
#include <memory>

namespace sinriv::kigstudio::octree {
    // 定义三维向量结构体
    struct Vec3i {
        int x, y, z;

        inline Vec3i(int x = 0, int y = 0, int z = 0) : x(x), y(y), z(z) {}

        // 比较两个向量是否相等
        inline bool operator==(const Vec3i& other) const {
            return x == other.x && y == other.y && z == other.z;
        }

        inline bool operator!=(const Vec3i& other) const {
            return !(*this == other);
        }

        inline bool operator<(const Vec3i& other) const {
            return x < other.x || (x == other.x && y < other.y) || (x == other.x && y == other.y && z < other.z);
        }
        inline bool operator<=(const Vec3i& other) const {
            return *this < other || *this == other;
        }
        inline bool operator>(const Vec3i& other) const {
            return !(*this <= other);
        }
        inline bool operator>=(const Vec3i& other) const {
            return !(*this < other);
        }
        inline Vec3i operator+(const Vec3i& other) const {
            return Vec3i(x + other.x, y + other.y, z + other.z);
        }
        inline Vec3i operator-(const Vec3i& other) const {
            return Vec3i(x - other.x, y - other.y, z - other.z);
        }
        inline Vec3i operator*(int scalar) const {
            return Vec3i(x * scalar, y * scalar, z * scalar);
        }
        inline Vec3i operator/(int scalar) const {
            return Vec3i(x / scalar, y / scalar, z / scalar);
        }
        inline Vec3i& operator+=(const Vec3i& other) {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }
        inline Vec3i& operator-=(const Vec3i& other) {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }
        inline Vec3i& operator*=(int scalar) {
            x *= scalar;
            y *= scalar;
            z *= scalar;
            return *this;
        }
        inline Vec3i& operator/=(int scalar) {
            x /= scalar;
            y /= scalar;
            z /= scalar;
            return *this;
        }
    };

    // 定义八叉树节点类
    class OctreeNode {
    public:
        Vec3i position; // 当前节点保存的点
        bool isLeaf;    // 是否为叶节点
        std::array<std::unique_ptr<OctreeNode>, 8> children; // 子节点

        inline OctreeNode(const Vec3i& pos)
            : position(pos), isLeaf(true) {
            for (auto& child : children) {
                child = nullptr;
            }
        }
    };

    // 定义八叉树类
    class Octree {
    private:
        std::unique_ptr<OctreeNode> root; // 根节点
        int size; // 八叉树空间的边长（假设为立方体）

        // 计算点在哪个子节点
        inline int getChildIndex(const Vec3i& point, const Vec3i& center) const {
            int index = 0;
            if (point.x >= center.x) index |= 1;
            if (point.y >= center.y) index |= 2;
            if (point.z >= center.z) index |= 4;
            return index;
        }

        // 插入点的递归实现
        void insert(std::unique_ptr<OctreeNode>& node, const Vec3i& point, const Vec3i& minCorner, const Vec3i& maxCorner);


        // 查询点的递归实现
        bool find(const std::unique_ptr<OctreeNode>& node, const Vec3i& point, const Vec3i& minCorner, const Vec3i& maxCorner) const;

    public:
        inline Octree(int size) : size(size) {}

        // 插入点
        inline void insert(const Vec3i& point) {
            Vec3i minCorner(0, 0, 0);
            Vec3i maxCorner(size, size, size);
            insert(root, point, minCorner, maxCorner);
        }

        // 查询点是否存在
        inline bool find(const Vec3i& point) const {
            Vec3i minCorner(0, 0, 0);
            Vec3i maxCorner(size, size, size);
            return find(root, point, minCorner, maxCorner);
        }
        class Iterator {
        private:
            std::stack<std::pair<OctreeNode*, Vec3i>> stack; // 用于DFS的栈
            Vec3i size; // 空间大小
            Vec3i currentPosition; // 当前迭代器指向的位置
            bool valid; // 标志当前迭代器是否有效

            inline void pushChildren(OctreeNode* node, const Vec3i& minCorner, const Vec3i& maxCorner) {
                if (!node || node->isLeaf) return;

                Vec3i center = {
                    (minCorner.x + maxCorner.x) / 2,
                    (minCorner.y + maxCorner.y) / 2,
                    (minCorner.z + maxCorner.z) / 2
                };

                for (int i = 0; i < 8; ++i) {
                    Vec3i childMin = minCorner;
                    Vec3i childMax = maxCorner;

                    if (i & 1) childMin.x = center.x; else childMax.x = center.x;
                    if (i & 2) childMin.y = center.y; else childMax.y = center.y;
                    if (i & 4) childMin.z = center.z; else childMax.z = center.z;

                    if (node->children[i]) {
                        stack.push({ node->children[i].get(), childMin });
                    }
                }
            }

            inline void advance() {
                valid = false;
                while (!stack.empty()) {
                    auto [node, minCorner] = stack.top();
                    stack.pop();

                    if (node) {
                        if (node->isLeaf) {
                            currentPosition = node->position;
                            valid = true;
                            break;
                        }
                        Vec3i maxCorner = { size.x, size.y, size.z };
                        pushChildren(node, minCorner, maxCorner);
                    }
                }
            }

        public:
            inline Iterator(OctreeNode* root, const Vec3i& size) : size(size), valid(false) {
                if (root) {
                    stack.push({ root, Vec3i(0, 0, 0) });
                    advance(); // 初始化到第一个有效节点
                }
            }

            inline Vec3i operator*() const {
                if (!valid) {
                    throw std::out_of_range("Iterator is out of range or points to a null node.");
                }
                return currentPosition;
            }

            inline Iterator& operator++() {
                advance();
                return *this;
            }

            inline bool operator!=(const Iterator& other) const {
                // 比较栈是否为空作为结束判断条件
                return valid != other.valid || (!stack.empty() != !other.stack.empty());
            }
        };

        inline Iterator begin() {
            return Iterator(root.get(), Vec3i(size, size, size));
        }

        inline  Iterator end() {
            // end() 的迭代器应初始化为无效状态
            return Iterator(nullptr, Vec3i(size, size, size));
        }
    public:
        class ConstIterator {
            // 和原 Iterator 类的实现类似，只是它只提供只读访问
            using Node = const OctreeNode*; // 保证不可修改
            std::stack<std::pair<Node, Vec3i>> stack;
            Vec3i currentPosition;
            bool valid;

            inline void advance() {
                valid = false;
                while (!stack.empty()) {
                    auto [node, minCorner] = stack.top();
                    stack.pop();

                    if (node) {
                        if (node->isLeaf) {
                            currentPosition = node->position;
                            valid = true;
                            break;
                        }

                        Vec3i center = {
                            (minCorner.x + node->position.x) / 2,
                            (minCorner.y + node->position.y) / 2,
                            (minCorner.z + node->position.z) / 2
                        };

                        for (int i = 0; i < 8; ++i) {
                            if (node->children[i]) {
                                Vec3i childMin = minCorner;
                                Vec3i childMax = node->position;
                                if (i & 1) childMin.x = center.x; else childMax.x = center.x;
                                if (i & 2) childMin.y = center.y; else childMax.y = center.y;
                                if (i & 4) childMin.z = center.z; else childMax.z = center.z;

                                stack.push({ node->children[i].get(), childMin });
                            }
                        }
                    }
                }
            }

        public:
            inline ConstIterator(Node root, const Vec3i& size) : valid(false) {
                if (root) {
                    stack.push({ root, Vec3i(0, 0, 0) });
                    advance();
                }
            }

            inline Vec3i operator*() const {
                if (!valid) {
                    throw std::out_of_range("Iterator is out of range or points to a null node.");
                }
                return currentPosition;
            }

            inline ConstIterator& operator++() {
                advance();
                return *this;
            }

            inline bool operator!=(const ConstIterator& other) const {
                return valid != other.valid || (!stack.empty() != !other.stack.empty());
            }
        };

        inline ConstIterator begin() const {
            return ConstIterator(root.get(), Vec3i(size, size, size));
        }

        inline ConstIterator end() const {
            return ConstIterator(nullptr, Vec3i(size, size, size));
        }
    public:
        inline Octree intersection(const Octree& other) const {
            Octree result(size);
            for (auto it = begin(); it != end(); ++it) {
                if (other.find(*it)) {
                    result.insert(*it);
                }
            }
            return result;
        }

        inline Octree unionWith(const Octree& other) const {
            Octree result(size);
            for (auto it = begin(); it != end(); ++it) {
                result.insert(*it);
            }
            for (auto it = other.begin(); it != other.end(); ++it) {
                result.insert(*it);
            }
            return result;
        }

        inline Octree difference(const Octree& other) const {
            Octree result(size);
            for (auto it = begin(); it != end(); ++it) {
                if (!other.find(*it)) {
                    result.insert(*it);
                }
            }
            return result;
        }

    };

}
