#include "intersections/KDTree.hpp"
#include "intersections/intersections.hpp"
#include <algorithm>
#include <limits>

namespace Ray_KD_tracing::Intersection
{

    static inline float safe_minf(float a, float b) { return a < b ? a : b; }

    bool AABB::intersect(const Ray &r, float tMin, float tMax, float &outNear, float &outFar) const
    {
        outNear = tMin;
        outFar = tMax;
        for (int a = 0; a < 3; ++a)
        {
            float invD = 1.0f / r.direction[a];
            float t0 = (min[a] - r.origin[a]) * invD;
            float t1 = (max[a] - r.origin[a]) * invD;
            if (invD < 0.0f)
                std::swap(t0, t1);
            outNear = t0 > outNear ? t0 : outNear;
            outFar = t1 < outFar ? t1 : outFar;
            if (outFar <= outNear)
                return false;
        }
        return true;
    }

    void KDTree::computeTriangle(int idx, const NRenderer::Mesh &mesh, Triangle &out) const
    {
        Index i0 = mesh.positionIndices[idx * 3 + 0];
        Index i1 = mesh.positionIndices[idx * 3 + 1];
        Index i2 = mesh.positionIndices[idx * 3 + 2];
        out.v1 = mesh.positions[i0];
        out.v2 = mesh.positions[i1];
        out.v3 = mesh.positions[i2];
        out.normal = glm::normalize(glm::cross(out.v2 - out.v1, out.v3 - out.v1));
        out.vertexNormals[0] = (mesh.hasVertexNormal() && i0 < mesh.vertexNormals.size()) ? mesh.vertexNormals[i0] : Vec3(0);
        out.vertexNormals[1] = (mesh.hasVertexNormal() && i1 < mesh.vertexNormals.size()) ? mesh.vertexNormals[i1] : Vec3(0);
        out.vertexNormals[2] = (mesh.hasVertexNormal() && i2 < mesh.vertexNormals.size()) ? mesh.vertexNormals[i2] : Vec3(0);
        out.material = mesh.material;
    }

    int KDTree::buildNode(std::vector<int> &tris, int depth)
    {
        KDNode node;
        // compute bounds
        for (int ti : tris)
        {
            const Triangle &t = triangles[ti];
            node.box.expand(t.v1);
            node.box.expand(t.v2);
            node.box.expand(t.v3);
        }
        int nodeIndex = (int)nodes.size();
        nodes.push_back(node);

        // leaf criteria
        if (tris.size() <= 8 || depth >= 24)
        {
            nodes[nodeIndex].triIndices = tris;
            return nodeIndex;
        }

        // choose split axis by extent
        Vec3 ext = nodes[nodeIndex].box.max - nodes[nodeIndex].box.min;
        int axis = 0;
        if (ext.y > ext.x)
            axis = 1;
        if (ext.z > ext[axis])
            axis = 2;

        // compute centroids and sort
        std::sort(tris.begin(), tris.end(), [&](int a, int b)
                  {
            Vec3 ca = (triangles[a].v1 + triangles[a].v2 + triangles[a].v3) / 3.0f;
            Vec3 cb = (triangles[b].v1 + triangles[b].v2 + triangles[b].v3) / 3.0f;
            return ca[axis] < cb[axis]; });

        size_t mid = tris.size() / 2;
        std::vector<int> leftTris(tris.begin(), tris.begin() + mid);
        std::vector<int> rightTris(tris.begin() + mid, tris.end());

        int left = buildNode(leftTris, depth + 1);
        int right = buildNode(rightTris, depth + 1);

        nodes[nodeIndex].left = left;
        nodes[nodeIndex].right = right;
        return nodeIndex;
    }

    void KDTree::buildFromMesh(const NRenderer::Mesh &mesh)
    {
        triangles.clear();
        size_t triCount = mesh.positionIndices.size() / 3;
        triangles.reserve(triCount);
        for (size_t i = 0; i < triCount; ++i)
        {
            Triangle t;
            computeTriangle((int)i, mesh, t);
            triangles.push_back(t);
        }
        nodes.clear();
        std::vector<int> all(triangles.size());
        for (size_t i = 0; i < triangles.size(); ++i)
            all[i] = (int)i;
        if (!all.empty())
            buildNode(all, 0);
    }

    void KDTree::buildFromTriangles(const std::vector<Triangle> &tris)
    {
        triangles = tris;
        nodes.clear();
        std::vector<int> all(triangles.size());
        for (size_t i = 0; i < triangles.size(); ++i)
            all[i] = (int)i;
        if (!all.empty())
            buildNode(all, 0);
    }

    HitRecord KDTree::intersectNode(int nodeIndex, const Ray &ray, float tMin, float tMax, float &closest) const
    {
        const KDNode &n = nodes[nodeIndex];
        float tNear, tFar;
        if (!n.box.intersect(ray, tMin, tMax, tNear, tFar))
            return nullopt;

        // if leaf, test contained triangles
        if (n.left == -1 && n.right == -1)
        {
            HitRecord best = nullopt;
            for (int ti : n.triIndices)
            {
                auto hit = xTriangle(ray, triangles[ti], tMin, closest);
                if (hit && hit->t < closest)
                {
                    closest = hit->t;
                    best = hit;
                }
            }
            return best;
        }

        int left = n.left;
        int right = n.right;
        float leftNear = std::numeric_limits<float>::infinity();
        float leftFar = std::numeric_limits<float>::infinity();
        float rightNear = std::numeric_limits<float>::infinity();
        float rightFar = std::numeric_limits<float>::infinity();
        bool lhit = false, rhit = false;
        if (left != -1)
            lhit = nodes[left].box.intersect(ray, tMin, tMax, leftNear, leftFar);
        if (right != -1)
            rhit = nodes[right].box.intersect(ray, tMin, tMax, rightNear, rightFar);

        // decide near/far order
        int nearChild = -1, farChild = -1;
        if (lhit && rhit)
        {
            if (leftNear < rightNear)
            {
                nearChild = left;
                farChild = right;
            }
            else
            {
                nearChild = right;
                farChild = left;
            }
        }
        else if (lhit)
        {
            nearChild = left;
        }
        else if (rhit)
        {
            nearChild = right;
        }

        HitRecord hitRec = nullopt;
        if (nearChild != -1)
        {
            auto h = intersectNode(nearChild, ray, tMin, tMax, closest);
            if (h)
                hitRec = h;
        }
        if (!hitRec && farChild != -1)
        {
            auto h = intersectNode(farChild, ray, tMin, tMax, closest);
            if (h)
                hitRec = h;
        }
        return hitRec;
    }

    HitRecord KDTree::intersect(const Ray &ray, float tMin, float tMax) const
    {
        if (nodes.empty())
            return nullopt;
        float closest = tMax;
        return intersectNode(0, ray, tMin, tMax, closest);
    }

}
