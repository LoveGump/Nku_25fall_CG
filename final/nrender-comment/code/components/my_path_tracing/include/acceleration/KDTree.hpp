#pragma once
#ifndef __SIMPLE_PT_KDTREE_HPP__
#define __SIMPLE_PT_KDTREE_HPP__

#include <memory>
#include <vector>

#include "geometry/vec.hpp"
#include "scene/Model.hpp"
#include "intersections/HitRecord.hpp"
#include "Ray.hpp"

namespace MyPathTracing
{
    // KD树加速结构：对三角形做空间划分并提供快速相交查询
    class KDTree
    {
    public:
        KDTree() = default;
        explicit KDTree(const std::vector<Triangle>& triangles);
        ~KDTree() = default;

        void build(const std::vector<Triangle>& triangles);
        bool empty() const { return !root; }

        HitRecord intersect(const Ray& ray, float tMin, float tMax) const;

    private:
        struct AABB
        {
            Vec3 min;
            Vec3 max;

            AABB();
            void expand(const Vec3& p);
            void expand(const AABB& other);
            bool intersect(const Ray& ray, float tMin, float tMax, float& tEnter, float& tExit) const;
        };

        struct Node
        {
            AABB bounds;
            std::unique_ptr<Node> left;
            std::unique_ptr<Node> right;
            std::vector<const Triangle*> triangles;

            bool isLeaf() const { return !left && !right; }
        };

        struct Primitive
        {
            const Triangle* tri = nullptr;
            AABB bounds;
            Vec3 centroid;
        };

        std::unique_ptr<Node> root;

        std::unique_ptr<Node> buildRecursive(std::vector<Primitive>& primitives, int depth);
        bool intersectNode(const Node* node, const Ray& ray, float tMin, float& tMax, HitRecord& best) const;

        static AABB computeBounds(const Triangle& tri);
        static Vec3 computeCentroid(const Triangle& tri);
    };
}

#endif

