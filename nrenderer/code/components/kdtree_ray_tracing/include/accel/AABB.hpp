#pragma once
#ifndef __KD_AABB_HPP__
#define __KD_AABB_HPP__

#include "geometry/vec.hpp"
#include "Ray.hpp"

namespace RayCastKD {
    struct AABB {
        NRenderer::Vec3 bmin{INFINITY, INFINITY, INFINITY};
        NRenderer::Vec3 bmax{-INFINITY, -INFINITY, -INFINITY};

        void expand(const NRenderer::Vec3 &p)
        {
            bmin = NRenderer::Vec3{std::min(bmin.x, p.x), std::min(bmin.y, p.y), std::min(bmin.z, p.z)};
            bmax = NRenderer::Vec3{std::max(bmax.x, p.x), std::max(bmax.y, p.y), std::max(bmax.z, p.z)};
        }
        void expand(const AABB &b)
        {
            expand(b.bmin);
            expand(b.bmax);
        }
        NRenderer::Vec3 extent() const { return bmax - bmin; }
        int longestAxis() const
        {
            auto e = extent();
            if (e.x > e.y && e.x > e.z)
                return 0;
            if (e.y > e.z)
                return 1;
            return 2;
        }
        // Ray-AABB ???????????????slab ???
        bool intersect(const Ray &r, float tMin, float tMax) const {
            using namespace NRenderer;
            for (int a = 0; a < 3; ++a)
            {
                float invD = 1.0f / (a == 0 ? r.direction.x : (a == 1 ? r.direction.y : r.direction.z));
                float orig = (a == 0 ? r.origin.x : (a == 1 ? r.origin.y : r.origin.z));
                float t0 = ((a == 0 ? bmin.x : (a == 1 ? bmin.y : bmin.z)) - orig) * invD;
                float t1 = ((a == 0 ? bmax.x : (a == 1 ? bmax.y : bmax.z)) - orig) * invD;
                if (invD < 0.0f)
                    std::swap(t0, t1);
                tMin = t0 > tMin ? t0 : tMin;
                tMax = t1 < tMax ? t1 : tMax;
                if (tMax <= tMin)
                    return false;
            }
            return true;
        }

        // Ray-AABB ?????slab ???????/??? tEntry/tExit??? KDTree ???????
        bool intersectWithT(const Ray &r, float tMin, float tMax, float &tEntry, float &tExit) const {
            // ??????? 0 ??????????????
            using namespace NRenderer;
            tEntry = tMin;
            tExit = tMax;
            for (int a = 0; a < 3; ++a)
            {
                float dir = (a == 0 ? r.direction.x : (a == 1 ? r.direction.y : r.direction.z));
                float orig = (a == 0 ? r.origin.x : (a == 1 ? r.origin.y : r.origin.z));

                
                float invD;
                if (std::abs(dir) < 1e-12f)
                {
                    // ????????????????????????????????????? epsilon
                    invD = 1e12f; 
                }
                else
                {
                    invD = 1.0f / dir;
                }

                float t0 = ((a == 0 ? bmin.x : (a == 1 ? bmin.y : bmin.z)) - orig) * invD;
                float t1 = ((a == 0 ? bmax.x : (a == 1 ? bmax.y : bmax.z)) - orig) * invD;
                if (invD < 0.0f)
                    std::swap(t0, t1);
                tEntry = std::max(tEntry, t0);
                tExit = std::min(tExit, t1);
                if (tExit <= tEntry)
                    return false;
            }
            return true;
        }
    };
}

#endif