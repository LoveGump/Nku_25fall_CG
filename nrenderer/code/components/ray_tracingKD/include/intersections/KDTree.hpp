#pragma once
#include <vector>
#include <memory>
#include <limits>
#include "intersections/HitRecord.hpp"
#include "Ray.hpp"
#include "scene/Scene.hpp"

namespace Ray_KD_tracing::Intersection
{

    struct AABB
    {
        Vec3 min = Vec3(std::numeric_limits<float>::infinity());
        Vec3 max = Vec3(-std::numeric_limits<float>::infinity());
        void expand(const Vec3 &p)
        {
            min = glm::min(min, p);
            max = glm::max(max, p);
        }
        void expand(const AABB &b)
        {
            min = glm::min(min, b.min);
            max = glm::max(max, b.max);
        }
        // 求光线与包围盒相交，返回是否相交并输出近远交点参数
        bool intersect(const Ray &r, float tMin, float tMax, float &outNear, float &outFar) const;
    };

    struct KDNode
    {
        AABB box;
        int left = -1;
        int right = -1;
        std::vector<int> triIndices; // indices into triangle list
    };

    class KDTree
    {
    public:
        KDTree() = default;
        // build from mesh (triangles implied by positionIndices)
        void buildFromMesh(const NRenderer::Mesh &mesh);
        // build directly from triangle list
        void buildFromTriangles(const std::vector<Triangle> &tris);
        HitRecord intersect(const Ray &ray, float tMin, float tMax) const;

    private:
        std::vector<KDNode> nodes;
        std::vector<Triangle> triangles;

        int buildNode(std::vector<int> &tris, int depth);
        void computeTriangle(int idx, const NRenderer::Mesh &mesh, Triangle &out) const;
        // recursive helper (near-first) that has access to private members
        HitRecord intersectNode(int nodeIndex, const Ray &ray, float tMin, float tMax, float &closest) const;
    };

}
