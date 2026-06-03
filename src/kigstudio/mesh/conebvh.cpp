#include "conebvh.h"
#include <algorithm>
#include <cmath>
#include <queue>

namespace sinriv::kigstudio::mesh::conebvh {
typedef ::sinriv::kigstudio::vec3<float> vec3f;

float ConeGroup::get_distance(const Cone& cone, const vec3f& point) const {
    const vec3f axis = cone.direction.length2() > 0.0f
                           ? cone.direction.normalize()
                           : vec3f(0.0f, 0.0f, 1.0f);
    const vec3f delta = point - apex;
    const float q = delta.dot(axis);
    const float v2 = delta.length2();
    const float r2 = std::max(v2 - q * q, 0.0f);
    const float r = std::sqrt(r2);
    const float cosA = std::cos(cone.angle);
    const float sinA = std::sin(cone.angle);
    const float tanA = std::tan(cone.angle);
    const float base_radius = cone.length * tanA;

    if (q < 0.0f) {
        return std::sqrt(q * q + r2);
    }

    if (q <= cone.length) {
        float lateral = r * cosA - q * sinA;
        if (r <= q * tanA) {
            float dist_lateral = -lateral;
            float dist_base = cone.length - q;
            return -std::min(dist_lateral, dist_base);
        }
        return lateral;
    }

    if (r <= base_radius) {
        return q - cone.length;
    }

    const float dr = r - base_radius;
    const float dz = q - cone.length;
    return std::sqrt(dr * dr + dz * dz);
}

bool ConeGroup::contains(const Cone& cone, const vec3f& point) const {
    const vec3f axis = cone.direction.length2() > 0.0f
                           ? cone.direction.normalize()
                           : vec3f(0.0f, 0.0f, 1.0f);
    const vec3f delta = point - apex;
    const float q = delta.dot(axis);
    if (q < 0.0f || q > cone.length) {
        return false;
    }

    const float r2 = std::max(delta.length2() - q * q, 0.0f);
    const float r = std::sqrt(r2);
    const float tanA = std::tan(cone.angle);
    return r <= q * tanA;
}

void ConeGroup::merge(const Cone& cone_A,
                      const Cone& cone_B,
                      Cone& cone_out) const {
    const vec3f dirA = cone_A.direction.length2() > 0.0f
                           ? cone_A.direction.normalize()
                           : vec3f(0.0f, 0.0f, 1.0f);
    const vec3f dirB = cone_B.direction.length2() > 0.0f
                           ? cone_B.direction.normalize()
                           : vec3f(0.0f, 0.0f, 1.0f);

    const float dotAB =
        std::max<float>(-1.0f, std::min<float>(1.0f, dirA.dot(dirB)));
    const float theta = std::acos(dotAB);
    if (cone_A.angle >= cone_B.angle + theta) {
        cone_out = cone_A;
        return;
    }
    if (cone_B.angle >= cone_A.angle + theta) {
        cone_out = cone_B;
        return;
    }

    vec3f axis = dirA + dirB;
    if (axis.length2() == 0.0f) {
        axis = dirA;
    } else {
        axis = axis.normalize();
    }

    const float angleA = std::acos(
        std::max<float>(-1.0f, std::min<float>(1.0f, axis.dot(dirA))));
    const float angleB = std::acos(
        std::max<float>(-1.0f, std::min<float>(1.0f, axis.dot(dirB))));
    cone_out.direction = axis;
    cone_out.angle = std::max(angleA + cone_A.angle, angleB + cone_B.angle);
    cone_out.length = std::max(cone_A.length, cone_B.length);
}
// ---------- ConeBVHTree method implementations ----------

void ConeBVHTree::set_tree(std::vector<Node>&& n, std::vector<Cone>&& prims) {
    nodes = std::move(n);
    primitives = std::move(prims);
    primitive_indices.clear();
    root = nodes.empty() ? -1 : 0;
}

void ConeBVHTree::build(const std::vector<Cone>& cones, const vec3f& apex) {
    this->apex = apex;
    primitives = cones;
    primitive_indices.clear();
    nodes.clear();

    if (primitives.empty()) {
        root = -1;
        return;
    }

    struct BuildEntry {
        int primitive_index;
        Cone bound;
        vec3f centroid;
    };

    std::vector<BuildEntry> entries;
    entries.reserve(primitives.size());
    for (int i = 0; i < (int)primitives.size(); ++i) {
        BuildEntry entry;
        entry.primitive_index = i;
        entry.bound = primitives[i];
        entry.centroid = primitives[i].direction.length2() > 0.0f
                             ? primitives[i].direction * primitives[i].length
                             : vec3f(0.0f, 0.0f, primitives[i].length);
        entries.push_back(entry);
    }

    primitive_indices.reserve(primitives.size());
    nodes.reserve(primitives.size() * 2);

    auto build_node = [&](auto&& self, int begin, int end) -> int {
        int node_index = (int)nodes.size();
        nodes.emplace_back();

        nodes[node_index].bound = entries[begin].bound;
        for (int i = begin + 1; i < end; ++i)
            merge(nodes[node_index].bound, entries[i].bound, nodes[node_index].bound);

        const int count = end - begin;
        if (count <= 4) {
            nodes[node_index].first = (int)primitive_indices.size();
            nodes[node_index].count = count;
            for (int i = begin; i < end; ++i)
                primitive_indices.push_back(entries[i].primitive_index);
            return node_index;
        }

        vec3f minC = entries[begin].centroid;
        vec3f maxC = entries[begin].centroid;
        for (int i = begin + 1; i < end; ++i) {
            minC.x = std::min(minC.x, entries[i].centroid.x);
            minC.y = std::min(minC.y, entries[i].centroid.y);
            minC.z = std::min(minC.z, entries[i].centroid.z);
            maxC.x = std::max(maxC.x, entries[i].centroid.x);
            maxC.y = std::max(maxC.y, entries[i].centroid.y);
            maxC.z = std::max(maxC.z, entries[i].centroid.z);
        }

        vec3f extent = maxC - minC;
        int axis = 0;
        if (extent.y > extent.x && extent.y >= extent.z)
            axis = 1;
        else if (extent.z > extent.x && extent.z > extent.y)
            axis = 2;

        std::sort(entries.begin() + begin, entries.begin() + end,
                  [axis](const BuildEntry& a, const BuildEntry& b) {
                      float va = axis == 0   ? a.centroid.x
                                 : axis == 1 ? a.centroid.y
                                             : a.centroid.z;
                      float vb = axis == 0   ? b.centroid.x
                                 : axis == 1 ? b.centroid.y
                                             : b.centroid.z;
                      return va < vb;
                  });

        int mid = begin + count / 2;
        nodes[node_index].left = self(self, begin, mid);
        nodes[node_index].right = self(self, mid, end);
        nodes[node_index].bound = nodes[nodes[node_index].left].bound;
        merge(nodes[node_index].bound, nodes[nodes[node_index].right].bound, nodes[node_index].bound);
        return node_index;
    };

    root = build_node(build_node, 0, (int)entries.size());
}

void ConeBVHTree::query_point_strict(
    const vec3f& p,
    const std::function<void(int)>& callback) const {
    if (root < 0 || nodes.empty())
        return;
    std::vector<int> stack;
    stack.push_back(root);
    while (!stack.empty()) {
        int ni = stack.back();
        stack.pop_back();
        const Node& node = nodes[ni];
        if (!contains(node.bound, p))
            continue;
        if (node.isLeaf()) {
            for (int i = 0; i < node.count; ++i) {
                int primIdx = node.first + i;
                if (!primitive_indices.empty())
                    primIdx = primitive_indices[primIdx];
                if (primIdx >= 0 && primIdx < (int)primitives.size()) {
                    if (this->contains(primitives[primIdx], p))
                        callback(primIdx);
                }
            }
        } else {
            if (node.left >= 0)
                stack.push_back(node.left);
            if (node.right >= 0)
                stack.push_back(node.right);
        }
    }
}

bool ConeBVHTree::query_nearest(const vec3f& p,
                                int& outIndex,
                                float& outDistance) const {
    if (root < 0 || nodes.empty())
        return false;

    outIndex = -1;
    outDistance = std::numeric_limits<float>::infinity();

    std::vector<int> stack;
    stack.push_back(root);
    while (!stack.empty()) {
        int ni = stack.back();
        stack.pop_back();
        const Node& node = nodes[ni];

        float nodeBoundDist = std::abs(this->get_distance(node.bound, p));
        float threshold = std::abs(outDistance);
        if (nodeBoundDist > threshold)
            continue;

        if (node.isLeaf()) {
            for (int i = 0; i < node.count; ++i) {
                int primIdx = node.first + i;
                if (!primitive_indices.empty())
                    primIdx = primitive_indices[primIdx];
                if (primIdx < 0 || primIdx >= (int)primitives.size())
                    continue;
                float d = this->get_distance(primitives[primIdx], p);
                if (std::abs(d) < std::abs(outDistance) || outIndex < 0) {
                    outDistance = d;
                    outIndex = primIdx;
                }
            }
        } else {
            if (node.left >= 0)
                stack.push_back(node.left);
            if (node.right >= 0)
                stack.push_back(node.right);
        }
    }

    return outIndex >= 0;
}

bool ConeBVHTree::query_nearest_k(
    const vec3f& p,
    int k,
    std::vector<std::pair<int, float>>& outResults) const {
    if (root < 0 || nodes.empty() || k <= 0)
        return false;

    struct Candidate {
        float absDist;
        float signedDist;
        int index;
    };

    struct CandidateCompare {
        bool operator()(const Candidate& a, const Candidate& b) const {
            return a.absDist < b.absDist;
        }
    };

    std::priority_queue<Candidate, std::vector<Candidate>, CandidateCompare>
        best;
    std::vector<int> stack;
    stack.push_back(root);
    while (!stack.empty()) {
        int ni = stack.back();
        stack.pop_back();
        const Node& node = nodes[ni];

        float threshold = best.empty() ? std::numeric_limits<float>::infinity()
                                       : best.top().absDist;
        float nodeBoundDist = std::abs(this->get_distance(node.bound, p));
        if (!best.empty() && nodeBoundDist > threshold)
            continue;

        if (node.isLeaf()) {
            for (int i = 0; i < node.count; ++i) {
                int primIdx = node.first + i;
                if (!primitive_indices.empty())
                    primIdx = primitive_indices[primIdx];
                if (primIdx < 0 || primIdx >= (int)primitives.size())
                    continue;
                float signedDist = this->get_distance(primitives[primIdx], p);
                float absDist = std::abs(signedDist);
                if ((int)best.size() < k || absDist < best.top().absDist) {
                    best.push({absDist, signedDist, primIdx});
                    if ((int)best.size() > k)
                        best.pop();
                }
            }
        } else {
            if (node.left >= 0)
                stack.push_back(node.left);
            if (node.right >= 0)
                stack.push_back(node.right);
        }
    }

    if (best.empty())
        return false;

    outResults.clear();
    while (!best.empty()) {
        const auto& c = best.top();
        outResults.emplace_back(c.index, c.signedDist);
        best.pop();
    }
    std::reverse(outResults.begin(), outResults.end());
    return true;
}

Cone TetraConeBVHTree::make_cone_from_triangle(const triangle& t,
                                               const vec3f& apex) {
    const vec3f v0 = std::get<0>(t) - apex;
    const vec3f v1 = std::get<1>(t) - apex;
    const vec3f v2 = std::get<2>(t) - apex;

    auto safe_normalize = [](const vec3f& v) {
        return v.length2() > 0.0f ? v.normalize() : vec3f(0.0f, 0.0f, 1.0f);
    };

    const vec3f n0 = safe_normalize(v0);
    const vec3f n1 = safe_normalize(v1);
    const vec3f n2 = safe_normalize(v2);

    vec3f axis = n0 + n1 + n2;
    if (axis.length2() == 0.0f) {
        axis = n0;
    } else {
        axis = axis.normalize();
    }

    auto angle_for = [&](const vec3f& n) {
        float dotp = std::max<float>(-1.0f, std::min<float>(1.0f, axis.dot(n)));
        return std::acos(dotp);
    };

    float angle = std::max({angle_for(n0), angle_for(n1), angle_for(n2)});
    float length = std::max(std::max(v0.length(), v1.length()), v2.length());
    return Cone(axis, angle, length);
}

float TetraConeBVHTree::point_to_triangle_distance(const vec3f& p,
                                                   const vec3f& a,
                                                   const vec3f& b,
                                                   const vec3f& c) {
    const vec3f ab = b - a;
    const vec3f ac = c - a;
    const vec3f ap = p - a;
    const float d1 = ab.dot(ap);
    const float d2 = ac.dot(ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
        return ap.length();

    const vec3f bp = p - b;
    const float d3 = ab.dot(bp);
    const float d4 = ac.dot(bp);
    if (d3 >= 0.0f && d4 <= d3)
        return bp.length();

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        vec3f proj = a + ab * v;
        return (p - proj).length();
    }

    const vec3f cp = p - c;
    const float d5 = ab.dot(cp);
    const float d6 = ac.dot(cp);
    if (d6 >= 0.0f && d5 <= d6)
        return cp.length();

    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        vec3f proj = a + ac * w;
        return (p - proj).length();
    }

    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        vec3f proj = b + (c - b) * w;
        return (p - proj).length();
    }

    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    vec3f proj = a + ab * v + ac * w;
    return (p - proj).length();
}

bool TetraConeBVHTree::point_in_tetra(const vec3f& p,
                                      const vec3f& a,
                                      const vec3f& b,
                                      const vec3f& c,
                                      const vec3f& d) {
    auto orient3d = [](const vec3f& a, const vec3f& b, const vec3f& c,
                       const vec3f& d) {
        double bax = double(b.x) - a.x;
        double bay = double(b.y) - a.y;
        double baz = double(b.z) - a.z;

        double cax = double(c.x) - a.x;
        double cay = double(c.y) - a.y;
        double caz = double(c.z) - a.z;

        double dax = double(d.x) - a.x;
        double day = double(d.y) - a.y;
        double daz = double(d.z) - a.z;

        return (bay * caz - baz * cay) * dax + (baz * cax - bax * caz) * day +
               (bax * cay - bay * cax) * daz;
    };

    float s = orient3d(a, b, c, d);

    if (std::abs(s) < 1e-20f)
        return false;

    float s0 = orient3d(a, b, c, p);
    float s1 = orient3d(a, b, p, d);
    float s2 = orient3d(a, p, c, d);
    float s3 = orient3d(p, b, c, d);

    if (s > 0.0f) {
        return s0 >= 0 && s1 >= 0 && s2 >= 0 && s3 >= 0;
    } else {
        return s0 <= 0 && s1 <= 0 && s2 <= 0 && s3 <= 0;
    }
}

float TetraConeBVHTree::signed_distance_to_tetra(const vec3f& p,
                                                 const vec3f& a,
                                                 const vec3f& b,
                                                 const vec3f& c,
                                                 const vec3f& d) {
    float d0 = point_to_triangle_distance(p, a, b, c);
    float d1 = point_to_triangle_distance(p, a, b, d);
    float d2 = point_to_triangle_distance(p, a, c, d);
    float d3 = point_to_triangle_distance(p, b, c, d);
    float minDist = std::min(std::min(d0, d1), std::min(d2, d3));
    if (point_in_tetra(p, a, b, c, d))
        return -minDist;
    return minDist;
}

void TetraConeBVHTree::build(const std::vector<triangle>& bases,
                             const vec3f& apex) {
    tetra_bases = bases;
    std::vector<Cone> cones;
    cones.reserve(bases.size());
    for (const auto& tri : bases)
        cones.push_back(make_cone_from_triangle(tri, apex));
    ConeBVHTree::build(cones, apex);
}

bool TetraConeBVHTree::query_nearest_tetra(const vec3f& p,
                                           int& outTetraIndex,
                                           float& outSignedDistance) const {
    int nearestCone = -1;
    float coneDist = 0.0f;
    if (!query_nearest(p, nearestCone, coneDist))
        return false;

    if (nearestCone < 0 || nearestCone >= (int)tetra_bases.size())
        return false;

    outTetraIndex = nearestCone;
    const auto& tri = tetra_bases[nearestCone];
    outSignedDistance = signed_distance_to_tetra(
        p, apex, std::get<0>(tri), std::get<1>(tri), std::get<2>(tri));
    return true;
}

bool TetraConeBVHTree::query_nearest_tetra_k(
    const vec3f& p,
    int k,
    std::vector<std::pair<int, float>>& outTetraDistances) const {
    if (k <= 0)
        return false;

    std::vector<std::pair<int, float>> coneCandidates;
    if (!query_nearest_k(p, k, coneCandidates))
        return false;

    outTetraDistances.clear();
    outTetraDistances.reserve(coneCandidates.size());
    for (const auto& [coneIndex, coneSignedDist] : coneCandidates) {
        if (coneIndex < 0 || coneIndex >= (int)tetra_bases.size())
            continue;
        const auto& tri = tetra_bases[coneIndex];
        float tetraDist = signed_distance_to_tetra(
            p, apex, std::get<0>(tri), std::get<1>(tri), std::get<2>(tri));
        outTetraDistances.emplace_back(coneIndex, tetraDist);
    }

    std::sort(
        outTetraDistances.begin(), outTetraDistances.end(),
        [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
            return std::abs(a.second) < std::abs(b.second);
        });
    return !outTetraDistances.empty();
}
struct QueueItem {
    float lowerBound;
    int nodeIndex;

    bool operator<(const QueueItem& rhs) const {
        // priority_queue 默认大顶堆
        return lowerBound > rhs.lowerBound;
    }
};
bool TetraConeBVHTree::query_exact_nearest_tetra(
    const vec3f& p,
    int& outTetraIndex,
    float& outSignedDistance) const {
    if (root < 0 || nodes.empty())
        return false;

    std::priority_queue<QueueItem> pq;

    float rootBound = std::abs(ConeGroup::get_distance(nodes[root].bound, p));

    pq.push({rootBound, root});

    outTetraIndex = -1;
    outSignedDistance = std::numeric_limits<float>::infinity();

    while (!pq.empty()) {
        QueueItem item = pq.top();
        pq.pop();

        float currentBest = std::abs(outSignedDistance);

        //
        // 提前结束
        //
        if (outTetraIndex >= 0 && item.lowerBound >= currentBest) {
            break;
        }

        const Node& node = nodes[item.nodeIndex];

        //
        // 叶节点
        //
        if (node.isLeaf()) {
            for (int i = 0; i < node.count; ++i) {
                int primIdx = node.first + i;

                if (!primitive_indices.empty())
                    primIdx = primitive_indices[primIdx];

                if (primIdx < 0 || primIdx >= (int)tetra_bases.size())
                    continue;

                const auto& tri = tetra_bases[primIdx];

                float sdf = signed_distance_to_tetra(p, apex, std::get<0>(tri),
                                                     std::get<1>(tri),
                                                     std::get<2>(tri));

                if (outTetraIndex < 0 ||
                    std::abs(sdf) < std::abs(outSignedDistance)) {
                    outSignedDistance = sdf;
                    outTetraIndex = primIdx;
                }
            }

            continue;
        }

        //
        // 左子节点
        //
        if (node.left >= 0) {
            float lb =
                std::abs(ConeGroup::get_distance(nodes[node.left].bound, p));

            if (outTetraIndex < 0 || lb < std::abs(outSignedDistance)) {
                pq.push({lb, node.left});
            }
        }

        //
        // 右子节点
        //
        if (node.right >= 0) {
            float lb =
                std::abs(ConeGroup::get_distance(nodes[node.right].bound, p));

            if (outTetraIndex < 0 || lb < std::abs(outSignedDistance)) {
                pq.push({lb, node.right});
            }
        }
    }

    return outTetraIndex >= 0;
}

float TetraConeBVHTree::get_distance(const vec3f& p) const {
    int idx;
    float dist;

    if (!query_exact_nearest_tetra(p, idx, dist)) {
        return std::numeric_limits<float>::infinity();
    }

    return dist;
}

}  // namespace sinriv::kigstudio::mesh::conebvh
