#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <functional>
#include <stack>
#include "mempool.h"
#include "vec3.h"

namespace sinriv::kigstudio {
template <sinriv::kigstudio::Numeric number_t, typename data_t>
class dbvt3d {
   public:
    using vec3_n = sinriv::kigstudio::vec3<number_t>;
    class AABB {
       public:
        AABB *left, *right, *parent, *next;

        dbvt3d* box;

        vec3_n from, to;

        int id;

        data_t* data;

        inline bool isDataNode() { return data != NULL; }

        inline void setLeft(AABB* in) {
            left = in;
            in->parent = this;
        }
        inline void setRight(AABB* in) {
            right = in;
            in->parent = this;
        }

        inline number_t getMergeSizeSq(const AABB* other) const {
            vec3_n tf, tt;
            tf.x = std::min(from.x, other->from.x);
            tf.y = std::min(from.y, other->from.y);
            tf.z = std::min(from.z, other->from.z);

            tt.x = std::max(to.x, other->to.x);
            tt.y = std::max(to.y, other->to.y);
            tt.z = std::max(to.z, other->to.z);

            auto l = tt - tf;
            return (l.x * l.x + l.y * l.y + l.z * l.z);
        }

        inline void merge(const AABB* other, AABB* out) const {
            out->from.x = std::min(from.x, other->from.x);
            out->from.y = std::min(from.y, other->from.y);
            out->from.z = std::min(from.z, other->from.z);

            out->to.x = std::max(to.x, other->to.x);
            out->to.y = std::max(to.y, other->to.y);
            out->to.z = std::max(to.z, other->to.z);
        }

        inline bool isEmpty() const {
            return from.x > to.x || from.y > to.y || from.z > to.z;
        }

        inline bool inBox(const vec3_n& point) const {
            return ((point.x >= from.x && point.x <= to.x) &&
                    (point.y >= from.y && point.y <= to.y) &&
                    (point.z >= from.z && point.z <= to.z));
        }

        inline bool intersects(const AABB* in) const {
            return ((from.x >= in->from.x && from.x <= in->to.x) ||
                    (in->from.x >= from.x && in->from.x <= to.x)) &&
                   ((from.y >= in->from.y && from.y <= in->to.y) ||
                    (in->from.y >= from.y && in->from.y <= to.y)) &&
                   ((from.z >= in->from.z && from.z <= in->to.z) ||
                    (in->from.z >= from.z && in->from.z <= to.z));
        }

        inline bool inBox(const AABB* in) const {
            return (((from.x >= in->from.x) && (to.x <= in->to.x)) &&
                    ((from.y >= in->from.y) && (to.y <= in->to.y)) &&
                    ((from.z >= in->from.z) && (to.z <= in->to.z)));
        }

        inline number_t getSizeSq() const {
            auto l = to - from;
            return (l.x * l.x + l.y * l.y + l.z * l.z);
        }

        inline vec3_n getCenter() const { return (from + to) / 2; }

        inline vec3_n getExtent() const { return to - from; }

        inline bool intersects(const vec3_n& linemiddle,
                               const vec3_n& linevect,
                               number_t halflength) const {
            const vec3_n e = getExtent() * 0.5f;
            const vec3_n t = getCenter() - linemiddle;

            if ((fabs(t.x) > e.x + halflength * fabs(linevect.x)) ||
                (fabs(t.y) > e.y + halflength * fabs(linevect.y)) ||
                (fabs(t.z) > e.z + halflength * fabs(linevect.z)))
                return false;

            auto r = e.y * fabs(linevect.z) + e.z * fabs(linevect.y);
            if (fabs(t.y * linevect.z - t.z * linevect.y) > r)
                return false;

            r = e.x * fabs(linevect.z) + e.z * fabs(linevect.x);
            if (fabs(t.z * linevect.x - t.x * linevect.z) > r)
                return false;

            r = e.x * fabs(linevect.y) + e.y * fabs(linevect.x);
            if (fabs(t.x * linevect.y - t.y * linevect.x) > r)
                return false;

            return true;
        }

        inline bool intersects(const ray<number_t>& line) const {
            return intersects(line.getMiddle(), line.direction(),
                              line.getLength() * 0.5f);
        }

        inline void construct() {
            left = NULL;
            right = NULL;
            parent = NULL;
            data = NULL;
            from = vec3_n(0, 0, 0);
            to = vec3_n(0, 0, 0);
        }

        template <typename Func_t>
        inline void rayTest(const ray<number_t>& ray, Func_t callback) {
            // 使用栈存储待处理的节点
            std::stack<AABB*> nodeStack;
            nodeStack.push(this);  // 从当前节点开始

            while (!nodeStack.empty()) {
                AABB* current = nodeStack.top();
                nodeStack.pop();

                // 处理左节点
                if (current->left && current->left->intersects(ray)) {
                    if (current->left->isDataNode()) {
                        callback(current->left);
                    } else {
                        nodeStack.push(current->left);  // 非叶子节点入栈
                    }
                }

                // 处理右节点
                if (current->right && current->right->intersects(ray)) {
                    if (current->right->isDataNode()) {
                        callback(current->right);
                    } else {
                        nodeStack.push(current->right);  // 非叶子节点入栈
                    }
                }
            }
        }

        template <typename Func_t>
        inline void collisionTest(const AABB* in, Func_t callback) {
            std::stack<const AABB*> nodeStack;
            nodeStack.push(this);  // 从当前节点开始

            while (!nodeStack.empty()) {
                const AABB* current = nodeStack.top();
                nodeStack.pop();

                // 处理左节点（深度优先）
                if (current->left && current->left->intersects(in)) {
                    if (current->left->isDataNode()) {
                        callback(const_cast<AABB*>(current->left));
                    } else {
                        nodeStack.push(current->left);  // 非叶子节点入栈
                    }
                }

                // 处理右节点（深度优先）
                if (current->right && current->right->intersects(in)) {
                    if (current->right->isDataNode()) {
                        callback(const_cast<AABB*>(current->right));
                    } else {
                        nodeStack.push(current->right);  // 非叶子节点入栈
                    }
                }
            }
        }

        template <typename Func_t>
        inline void fetchByPoint(const vec3_n& point, Func_t callback) {
            std::stack<const AABB*> nodeStack;
            nodeStack.push(this);  // 从当前节点开始

            while (!nodeStack.empty()) {
                const AABB* current = nodeStack.top();
                nodeStack.pop();

                // 处理左节点（深度优先）
                if (current->left && current->left->inBox(point)) {
                    if (current->left->isDataNode()) {
                        callback(const_cast<AABB*>(current->left));
                    } else {
                        nodeStack.push(current->left);  // 非叶子节点入栈
                    }
                }

                // 处理右节点（深度优先）
                if (current->right && current->right->inBox(point)) {
                    if (current->right->isDataNode()) {
                        callback(const_cast<AABB*>(current->right));
                    } else {
                        nodeStack.push(current->right);  // 非叶子节点入栈
                    }
                }
            }
        }

        inline void autoclean() {
            if (left == NULL && right == NULL && !isDataNode()) {
                if (parent) {
                    if (parent->left == this) {
                        parent->left = NULL;
                    }
                    if (parent->right == this) {
                        parent->right = NULL;
                    }
                    parent->autoclean();
                    box->delAABB(this);
                }
            } else if (parent && parent->parent) {
                if (parent->left && parent->right == NULL)
                    parent->left = NULL;
                else if (parent->left == NULL && parent->right)
                    parent->right = NULL;
                else
                    return;

                if (parent->parent->left == parent) {
                    parent->parent->left = this;
                } else {
                    parent->parent->right = this;
                }

                auto tmp = parent;
                parent = parent->parent;
                box->delAABB(tmp);
                parent->autoclean();
            }
        }
        inline void autodrop() {
            auto p = parent;
            this->drop();
            if (p)
                p->autoclean();
        }
        inline void add(AABB* in) {
            if (left) {
                if (!left->isDataNode() && in->inBox(left)) {
                    left->add(in);
                    return;
                } else if (right == NULL) {
                    setRight(in);
                    return;
                }
            }
            if (right) {
                if (!right->isDataNode() && in->inBox(right)) {
                    right->add(in);
                    return;
                } else if (left == NULL) {
                    setLeft(in);
                    return;
                }
            }
            if (right == NULL && left == NULL) {
                setLeft(in);
                return;
            }

            auto ls = left->getMergeSizeSq(in);
            auto rs = right->getMergeSizeSq(in);
            auto nnode = box->createAABB();

            // nnode->parent=this;

            if (ls < rs) {
                in->merge(left, nnode);
                nnode->setLeft(left);
                nnode->setRight(in);
                this->setLeft(nnode);
            } else {
                in->merge(right, nnode);
                nnode->setLeft(right);
                nnode->setRight(in);
                this->setRight(nnode);
            }
        }
        inline void remove() {
            if (parent) {
                if (parent->left == this) {
                    parent->left = NULL;
                }
                if (parent->right == this) {
                    parent->right = NULL;
                }
                parent->autoclean();
                parent = NULL;
            }
        }
        inline void drop() {
            if (left) {
                left->drop();
                left = NULL;
            }
            if (right) {
                right->drop();
                right = NULL;
            }
            if (parent) {
                if (parent->left == this) {
                    parent->left = NULL;
                }
                if (parent->right == this) {
                    parent->right = NULL;
                }
                parent = NULL;
            }
            box->delAABB(this);
        }
    };

    AABB* root;

    inline AABB* createAABB() {
        auto p = pool.get();
        p->construct();
        p->box = this;
        return p;
    }

    inline void delAABB(AABB* p) { pool.del(p); }

    inline void add(AABB* in) { root->add(in); }
    inline void remove(AABB* in) { in->remove(); }
    inline AABB* add(const vec3_n& from, const vec3_n& to, data_t* data) {
        auto p = createAABB();
        p->from = from;
        p->to = to;
        p->data = data;
        root->add(p);
        return p;
    }

    template <typename Func_t>
    inline void rayTest(const ray<number_t>& ray, Func_t callback) {
        root->rayTest(ray, callback);
    }

    template <typename Func_t>
    inline void collisionTest(const AABB* in, Func_t callback) {
        root->collisionTest(in, callback);
    }
    template <typename Func_t>
    inline void fetchByPoint(const vec3_n& point, Func_t callback) {
        root->fetchByPoint(point, callback);
    }
    inline void makeID() {
        tmpid = 0;
        if (root)
            makeID(root);
    }
    inline dbvt3d() { root = createAABB(); }
    inline ~dbvt3d() {
        if (root)
            root->drop();
    }

   private:
    int tmpid;
    inline void makeID(AABB* p) {
        p->id = ++tmpid;
        if (p->left)
            makeID(p->left);
        if (p->right)
            makeID(p->right);
    }

    mempool<AABB> pool;
};
}  // namespace sinriv::kigstudio