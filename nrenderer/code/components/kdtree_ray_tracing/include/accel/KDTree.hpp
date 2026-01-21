#pragma once
#ifndef __KD_TREE_HPP__
#define __KD_TREE_HPP__

#include <vector>
#include <memory>
#include "geometry/vec.hpp"
#include "scene/Scene.hpp"
#include "accel/AABB.hpp"
#include "intersections/intersections.hpp"

namespace RayCastKD
{
    using namespace NRenderer;

    struct KDNode
    {
        AABB box{};
        int left = -1;
        int right = -1;
        int begin = 0; // leaf range
        int end = 0;   // [begin, end)
        bool isLeaf() const { return left < 0 && right < 0; }
    };

    // 为 KDTree 统一的三角面片表示（直接使用场景 Triangle）
    using Tri = Triangle;

    class KDTree
    {
    public:
        std::vector<Tri> tris;     // 叶子中的三角形列表
        std::vector<int> indices;  // 构建用索引
        std::vector<KDNode> nodes; // 扁平化节点

        void build(const std::vector<Tri> &primitives, int leafSize = 8, int maxDepth = 32);

        // 与光线求交，返回最近命中记录
        HitRecord intersect(const Ray &r, float tMin = 0.001f, float tMax = FLOAT_INF) const;

    private:
        int buildRecursive(int begin, int end, int depth, int leafSize, int maxDepth);
        AABB boundsOf(int begin, int end) const;
        AABB centroidBounds(int begin, int end) const;
    };
}

#endif
