#include "acceleration/KDTree.hpp"

#include <algorithm>
#include <limits>

#include "intersections/intersections.hpp"

namespace MyPathTracing
{
    namespace
    {
        constexpr int MAX_DEPTH = 32;
        constexpr int MAX_LEAF_PRIMS = 4;
    }

    KDTree::AABB::AABB()
        : min(std::numeric_limits<float>::max())
        , max(std::numeric_limits<float>::lowest())
    {}

    void KDTree::AABB::expand(const Vec3& p)
    {
        min = glm::min(min, p);
        max = glm::max(max, p);
    }

    void KDTree::AABB::expand(const AABB& other)
    {
        expand(other.min);
        expand(other.max);
    }

    bool KDTree::AABB::intersect(const Ray& ray, float tMin, float tMax, float& tEnter, float& tExit) const
    {
        tEnter = tMin;
        tExit = tMax;
        for (int axis = 0; axis < 3; ++axis)
        {
            float invD = 1.0f / ray.direction[axis];
            float t0 = (min[axis] - ray.origin[axis]) * invD;
            float t1 = (max[axis] - ray.origin[axis]) * invD;
            if (invD < 0.0f)
                std::swap(t0, t1);
            tEnter = t0 > tEnter ? t0 : tEnter;
            tExit = t1 < tExit ? t1 : tExit;
            if (tExit <= tEnter)
                return false;
        }
        return true;
    }

    KDTree::KDTree(const std::vector<Triangle>& triangles)
    {
        build(triangles);
    }

    void KDTree::build(const std::vector<Triangle>& triangles)
    {
        if (triangles.empty())
        {
            root.reset();
            return;
        }

        std::vector<Primitive> primitives;
        primitives.reserve(triangles.size());
        for (const auto& tri : triangles)
        {
            Primitive prim;
            prim.tri = &tri;
            prim.bounds = computeBounds(tri);
            prim.centroid = computeCentroid(tri);
            primitives.push_back(prim);
        }

        root = buildRecursive(primitives, 0);
    }

    KDTree::AABB KDTree::computeBounds(const Triangle& tri)
    {
        AABB bounds;
        bounds.expand(tri.v1);
        bounds.expand(tri.v2);
        bounds.expand(tri.v3);
        return bounds;
    }

    Vec3 KDTree::computeCentroid(const Triangle& tri)
    {
        return (tri.v1 + tri.v2 + tri.v3) / 3.0f;
    }

    std::unique_ptr<KDTree::Node> KDTree::buildRecursive(std::vector<Primitive>& primitives, int depth)
    {
        auto node = std::make_unique<Node>();
        node->triangles.reserve(primitives.size());

        for (const auto& prim : primitives)
        {
            node->bounds.expand(prim.bounds);
        }

        if (primitives.size() <= MAX_LEAF_PRIMS || depth >= MAX_DEPTH)
        {
            for (const auto& prim : primitives)
            {
                node->triangles.push_back(prim.tri);
            }
            return node;
        }

        Vec3 diagonal = node->bounds.max - node->bounds.min;
        int axis = 0;
        if (diagonal.y > diagonal.x && diagonal.y >= diagonal.z)
            axis = 1;
        else if (diagonal.z > diagonal.x && diagonal.z >= diagonal.y)
            axis = 2;

        float splitPos = node->bounds.min[axis] + diagonal[axis] * 0.5f;

        std::vector<Primitive> left;
        std::vector<Primitive> right;
        left.reserve(primitives.size());
        right.reserve(primitives.size());

        for (const auto& prim : primitives)
        {
            if (prim.centroid[axis] <= splitPos)
                left.push_back(prim);
            else
                right.push_back(prim);
        }

        if (left.empty() || right.empty())
        {
            for (const auto& prim : primitives)
            {
                node->triangles.push_back(prim.tri);
            }
            return node;
        }

        node->left = buildRecursive(left, depth + 1);
        node->right = buildRecursive(right, depth + 1);
        return node;
    }

    HitRecord KDTree::intersect(const Ray& ray, float tMin, float tMax) const
    {
        HitRecord best = getMissRecord();
        float closest = tMax;
        if (!root)
            return best;
        intersectNode(root.get(), ray, tMin, closest, best);
        return best;
    }

    bool KDTree::intersectNode(const Node* node, const Ray& ray, float tMin, float& tMax, HitRecord& best) const
    {
        if (!node)
            return false;

        float entry = 0.0f, exit = 0.0f;
        if (!node->bounds.intersect(ray, tMin, tMax, entry, exit))
            return false;

        bool hit = false;
        if (node->isLeaf())
        {
            for (const Triangle* tri : node->triangles)
            {
                auto record = Intersection::xTriangle(ray, *tri, tMin, tMax);
                if (record && record->t < tMax)
                {
                    tMax = record->t;
                    best = record;
                    hit = true;
                }
            }
            return hit;
        }

        bool hitLeft = intersectNode(node->left.get(), ray, tMin, tMax, best);
        bool hitRight = intersectNode(node->right.get(), ray, tMin, tMax, best);
        return hitLeft || hitRight;
    }
}

