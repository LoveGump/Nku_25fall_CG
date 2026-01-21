#pragma once
#ifndef __BVH_HPP__
#define __BVH_HPP__

#include "BoundingVolume.hpp"
#include "scene/Scene.hpp"
#include "intersections/intersections.hpp"
#include <limits>
#include <vector>

namespace SimplePathTracer
{
    // BVH 基类
    class BVH
    {
      public:
        virtual bool      build(const SharedScene&)                 = 0;
        virtual void      destory()                                 = 0;
        virtual HitRecord intersect(const Ray&, float, float) const = 0;
    };

    // 轴对齐包围盒BVH类
    class AABBBVH
    {
      private:
        AABB* root;
        std::vector<AABB*> objects;

      public:
        AABBBVH();
        ~AABBBVH();

      private:
        AABB* _build(size_t, size_t);
        HitRecord _intersect(AABB*, const Ray&, float, float) const;

      public:
        virtual bool      build(const SharedScene&);
        virtual void      destory();
        virtual HitRecord intersect(const Ray&, float, float) const;
    };
}

#endif