#include "accel/KDTree.hpp"

namespace RayCastKD
{
    using std::vector;

    static NRenderer::Vec3 triCentroid(const Tri &t)
    {
        return (t.v1 + t.v2 + t.v3) / 3.0f;
    }

    AABB KDTree::boundsOf(int begin, int end) const
    {
        AABB b;
        for (int i = begin; i < end; ++i)
        {
            const auto &T = tris[indices[i]];
            b.expand(T.v1);
            b.expand(T.v2);
            b.expand(T.v3);
        }
        return b;
    }

    AABB KDTree::centroidBounds(int begin, int end) const
    {
        AABB b;
        for (int i = begin; i < end; ++i)
            b.expand(triCentroid(tris[indices[i]]));
        return b;
    }

    int KDTree::buildRecursive(int begin, int end, int depth, int leafSize, int maxDepth)
    {
        KDNode node;
        node.begin = begin;
        node.end = end;
        node.box = boundsOf(begin, end);
        int nodeIndex = (int)nodes.size();
        nodes.push_back(node);

        int count = end - begin;
        if (count <= leafSize || depth >= maxDepth)
        {
            return nodeIndex; // leaf
        }

        // ???????????????????
        auto cb = centroidBounds(begin, end);
        int axis = cb.longestAxis();
        float mid = (axis == 0 ? (cb.bmin.x + cb.bmax.x) * 0.5f : (axis == 1 ? (cb.bmin.y + cb.bmax.y) * 0.5f : (cb.bmin.z + cb.bmax.z) * 0.5f));

        int m = begin;
        auto splitter = [&](int idx)
        {
            auto c = triCentroid(tris[idx]);
            float v = axis == 0 ? c.x : (axis == 1 ? c.y : c.z);
            return v < mid;
        };
        // ????? [begin, end)
        int i = begin, j = end - 1;
        while (i <= j)
        {
            if (splitter(indices[i]))
            {
                ++i;
            }
            else
            {
                std::swap(indices[i], indices[j]);
                --j;
            }
        }
        m = i;
        if (m == begin || m == end)
        {
            // ??????????????
            m = begin + count / 2;
            std::nth_element(indices.begin() + begin, indices.begin() + m, indices.begin() + end,
                             [&](int a, int b)
                             {
                                 auto ca = triCentroid(tris[a]);
                                 auto cb2 = triCentroid(tris[b]);
                                 float va = axis == 0 ? ca.x : (axis == 1 ? ca.y : ca.z);
                                 float vb = axis == 0 ? cb2.x : (axis == 1 ? cb2.y : cb2.z);
                                 return va < vb;
                             });
        }

        int left = buildRecursive(begin, m, depth + 1, leafSize, maxDepth);
        int right = buildRecursive(m, end, depth + 1, leafSize, maxDepth);
        nodes[nodeIndex].left = left;
        nodes[nodeIndex].right = right;
        return nodeIndex;
    }

    void KDTree::build(const vector<Tri> &primitives, int leafSize, int maxDepth)
    {
        tris = primitives;
        indices.resize((int)tris.size());
        for (int i = 0; i < (int)indices.size(); ++i)
            indices[i] = i;
        nodes.clear();
        nodes.reserve(tris.size() * 2);
        if (!tris.empty())
            buildRecursive(0, (int)tris.size(), 0, leafSize, maxDepth);
    }

    HitRecord KDTree::intersect(const Ray &r, float tMin, float tMax) const
    {
        if (nodes.empty())
            return getMissRecord();
        HitRecord best = getMissRecord();
        float closest = tMax;

        // 使用显式栈进行深度优先遍历（避免递归开销）
        std::vector<int> stack;
        stack.reserve(64);
        stack.push_back(0);
        while (!stack.empty())
        {
            int ni = stack.back();
            stack.pop_back();
            const KDNode &n = nodes[ni];
            if (!n.box.intersect(r, tMin, closest))
                continue;
            if (n.isLeaf())
            {
                for (int i = n.begin; i < n.end; ++i)
                {
                    int triId = indices[i];
                    const auto &T = tris[triId];
                    auto hit = Intersection::xTriangle(r, T, tMin, closest);
                    if (hit && hit->t < closest)
                    {
                        closest = hit->t;
                        best = hit;
                    }
                }
            }
            else
            {
                // 前端优先遍历：根据 AABB 相交的 tEntry/tExit，优先访问更近的子节点
                // 由于栈是 LIFO（后进先出），为保证“近端先处理”，应先压入远端，再压入近端
                const KDNode &nl = nodes[n.left];
                const KDNode &nr = nodes[n.right];
                float tEntryL, tExitL, tEntryR, tExitR;
                bool hitL = nl.box.intersectWithT(r, tMin, closest, tEntryL, tExitL);
                bool hitR = nr.box.intersectWithT(r, tMin, closest, tEntryR, tExitR);

                if (hitL && hitR)
                {
                    // 近远次序由 tEntry 决定
                    if (tEntryL < tEntryR)
                    {
                        stack.push_back(n.right);
                        stack.push_back(n.left);
                    }
                    else
                    {
                        stack.push_back(n.left);
                        stack.push_back(n.right);
                    }
                }
                else if (hitL)
                {
                    stack.push_back(n.left);
                }
                else if (hitR)
                {
                    stack.push_back(n.right);
                }
            }
        }
        return best;
    }
}