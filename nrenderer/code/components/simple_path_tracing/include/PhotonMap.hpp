#pragma once
#ifndef __SIMPLE_PATH_TRACING_PHOTON_MAP_HPP__
#define __SIMPLE_PATH_TRACING_PHOTON_MAP_HPP__

#include "scene/Scene.hpp"
#include "Ray.hpp"
#include "intersections/HitRecord.hpp"

#include <optional>
#include <queue>
#include <vector>

namespace SimplePathTracer
{
    using namespace NRenderer;

    struct Photon
    {
        Vec3 position{};
        Vec3 normal{};
        Vec3 flux{};
        Vec3 incident{};
    };

    struct PhotonMapConfig
    {
        size_t photonCount = 10000;
        size_t gatherCount = 64;
        float maxSearchRadius = 0.35f;
        int maxBounces = 5;
    };

    class PhotonMap
    {
    public:
        explicit PhotonMap(Scene &scene);

        void build(const PhotonMapConfig &config);

        [[nodiscard]] bool isReady() const noexcept;
        [[nodiscard]] size_t totalPhotons() const noexcept;

        [[nodiscard]] Vec3 estimateIndirect(const Vec3 &position,
                                            const Vec3 &normal,
                                            const Vec3 &outDirection,
                                            const Vec3 &albedo,
                                            size_t gatherCount = 0,
                                            float maxRadius = 0.0f) const;

    private:
        struct Node
        {
            int photonIndex = -1;
            int axis = 0;
            int left = -1;
            int right = -1;
        };

        Scene &scene;
        PhotonMapConfig config = {};

        std::vector<Photon> photons;
        std::vector<Node> nodes;
        int root = -1;
        bool built = false;

    private:
        void emitPhotons();
        void tracePhoton(const Ray &ray, Vec3 flux, int bounce = 0);
        HitRecord intersectScene(const Ray &ray, float tMin, float tMax) const;
        Vec3 fetchAlbedo(const Handle &materialHandle) const;
        void buildTree();
        int buildTreeRecursive(std::vector<int> &indices, int begin, int end, int depth);
        void gatherRecursive(int nodeIndex,
                             const Vec3 &position,
                             float &maxRadius2,
                             size_t gatherCount,
                             std::priority_queue<std::pair<float, int>> &heap) const;
    };
}

#endif
