#include "PhotonMap.hpp"

#include "Onb.hpp"
#include "intersections/intersections.hpp"
#include "samplers/SamplerInstance.hpp"

#include <algorithm>
#include <numeric>

#include "glm/gtc/constants.hpp"

namespace SimplePathTracer
{
    namespace
    {
        constexpr float EPS = 1e-4f;

        inline Vec3 offsetPoint(const Vec3 &p, const Vec3 &n)
        {
            return p + n * 5e-4f;
        }

        inline float luminance(const Vec3 &v)
        {
            return 0.2126f * v.r + 0.7152f * v.g + 0.0722f * v.b;
        }
    }

    PhotonMap::PhotonMap(Scene &scene)
        : scene(scene)
    {
    }

    void PhotonMap::build(const PhotonMapConfig &cfg)
    {
        config = cfg;
        photons.clear();
        nodes.clear();
        root = -1;
        built = false;

        if (scene.areaLightBuffer.empty())
        {
            return;
        }

        photons.reserve(config.photonCount * std::max(1, config.maxBounces));
        emitPhotons();
        buildTree();
        built = !photons.empty() && root != -1;
    }

    bool PhotonMap::isReady() const noexcept
    {
        return built;
    }

    size_t PhotonMap::totalPhotons() const noexcept
    {
        return photons.size();
    }

    Vec3 PhotonMap::estimateIndirect(const Vec3 &position,
                                     const Vec3 & /*normal*/,
                                     const Vec3 & /*outDirection*/,
                                     const Vec3 &albedo,
                                     size_t gatherCount,
                                     float maxRadius) const
    {
        if (!isReady())
            return Vec3(0);

        size_t count = gatherCount == 0 ? config.gatherCount : gatherCount;
        float radius = maxRadius <= 0.0f ? config.maxSearchRadius : maxRadius;
        if (count == 0 || radius <= 0.0f)
            return Vec3(0);

        std::priority_queue<std::pair<float, int>> heap;
        float radius2 = radius * radius;
        gatherRecursive(root, position, radius2, count, heap);

        if (heap.empty())
            return Vec3(0);

        float effectiveRadius2 = std::max(radius2, heap.top().first);
        Vec3 fluxSum(0);
        while (!heap.empty())
        {
            fluxSum += photons[heap.top().second].flux;
            heap.pop();
        }

        float area = glm::pi<float>() * std::max(effectiveRadius2, 1e-6f);
        Vec3 brdf = albedo / glm::pi<float>();
        return brdf * (fluxSum / area);
    }

    void PhotonMap::emitPhotons()
    {
        struct LightEmission
        {
            const AreaLight *light = nullptr;
            Vec3 normal = {};
            float area = 0.0f;
            float weight = 0.0f;
            size_t samples = 0;
        };

        std::vector<LightEmission> emissions;
        emissions.reserve(scene.areaLightBuffer.size());
        float totalWeight = 0.0f;
        for (const auto &light : scene.areaLightBuffer)
        {
            Vec3 crossVec = glm::cross(light.u, light.v);
            float area = glm::length(crossVec);
            if (area <= EPS)
                continue;
            float power = luminance(light.radiance) * area;
            if (power <= EPS)
                continue;
            LightEmission em;
            em.light = &light;
            em.normal = glm::normalize(crossVec);
            em.area = area;
            em.weight = power;
            emissions.push_back(em);
            totalWeight += power;
        }

        if (emissions.empty() || totalWeight <= EPS)
            return;

        for (auto &em : emissions)
        {
            float ratio = em.weight / totalWeight;
            em.samples = std::max<size_t>(1, static_cast<size_t>(ratio * config.photonCount));
        }

        size_t assigned = 0;
        for (const auto &em : emissions)
            assigned += em.samples;
        size_t cursor = 0;
        while (assigned < config.photonCount)
        {
            emissions[cursor % emissions.size()].samples++;
            assigned++;
            cursor++;
        }

        auto &sampler1d = defaultSamplerInstance<UniformSampler>();
        auto &hemiSampler = defaultSamplerInstance<HemiSphere>();

        for (const auto &em : emissions)
        {
            if (em.samples == 0)
                continue;
            Vec3 u = em.light->u;
            Vec3 v = em.light->v;
            for (size_t i = 0; i < em.samples; ++i)
            {
                float su = sampler1d.sample1d();
                float sv = sampler1d.sample1d();
                Vec3 origin = em.light->position + u * su + v * sv;
                Vec3 local = hemiSampler.sample3d();
                Onb onb(em.normal);
                Vec3 direction = glm::normalize(onb.local(local));
                Vec3 flux = em.light->radiance * (em.area / static_cast<float>(em.samples));
                Ray ray{offsetPoint(origin, em.normal), direction};
                tracePhoton(ray, flux, 0);
            }
        }
    }

    void PhotonMap::tracePhoton(const Ray &initialRay, Vec3 flux, int bounce)
    {
        Ray ray = initialRay;
        Vec3 throughput = flux;
        auto &sampler1d = defaultSamplerInstance<UniformSampler>();
        auto &hemiSampler = defaultSamplerInstance<HemiSphere>();

        int depth = bounce;
        while (depth < config.maxBounces)
        {
            auto hit = intersectScene(ray, EPS, FLOAT_INF);
            if (!hit)
                return;

            Vec3 albedo = fetchAlbedo(hit->material);
            if (glm::dot(albedo, albedo) <= EPS)
                return;

            Photon photon;
            photon.position = hit->hitPoint;
            photon.normal = hit->normal;
            photon.incident = -ray.direction;
            photon.flux = throughput;
            photons.push_back(photon);

            float survive = std::max({albedo.r, albedo.g, albedo.b});
            survive = glm::clamp(survive, 0.15f, 0.95f);
            if (sampler1d.sample1d() > survive)
                break;

            Vec3 local = hemiSampler.sample3d();
            Onb basis(hit->normal);
            Vec3 newDir = glm::normalize(basis.local(local));
            float cosTheta = glm::max(0.0f, glm::dot(hit->normal, newDir));
            if (cosTheta <= EPS)
                break;

            Vec3 brdf = albedo / glm::pi<float>();
            float pdf = 1.0f / (2.0f * glm::pi<float>());
            throughput = throughput * brdf * (cosTheta / (pdf * survive));

            if (glm::dot(throughput, throughput) <= EPS)
                break;

            ray.origin = offsetPoint(hit->hitPoint, hit->normal);
            ray.direction = newDir;
            depth++;
        }
    }

    HitRecord PhotonMap::intersectScene(const Ray &ray, float tMin, float tMax) const
    {
        HitRecord closest = nullopt;
        float closestT = tMax;

        for (const auto &sphere : scene.sphereBuffer)
        {
            auto hit = Intersection::xSphere(ray, sphere, tMin, closestT);
            if (hit && hit->t < closestT)
            {
                closestT = hit->t;
                closest = hit;
            }
        }

        for (const auto &tri : scene.triangleBuffer)
        {
            auto hit = Intersection::xTriangle(ray, tri, tMin, closestT);
            if (hit && hit->t < closestT)
            {
                closestT = hit->t;
                closest = hit;
            }
        }

        for (const auto &plane : scene.planeBuffer)
        {
            auto hit = Intersection::xPlane(ray, plane, tMin, closestT);
            if (hit && hit->t < closestT)
            {
                closestT = hit->t;
                closest = hit;
            }
        }

        return closest;
    }

    Vec3 PhotonMap::fetchAlbedo(const Handle &materialHandle) const
    {
        if (!materialHandle.valid())
            return Vec3(0.0f);
        auto index = materialHandle.index();
        if (index >= scene.materials.size())
            return Vec3(0.0f);
        auto &material = scene.materials[index];
        auto diffuse = material.getProperty<Property::Wrapper::RGBType>("diffuseColor");
        if (diffuse)
            return diffuse->value;
        return Vec3(1.0f);
    }

    void PhotonMap::buildTree()
    {
        if (photons.empty())
        {
            root = -1;
            nodes.clear();
            return;
        }
        std::vector<int> indices(photons.size());
        std::iota(indices.begin(), indices.end(), 0);
        nodes.clear();
        nodes.reserve(photons.size());
        root = buildTreeRecursive(indices, 0, static_cast<int>(indices.size()), 0);
    }

    int PhotonMap::buildTreeRecursive(std::vector<int> &indices, int begin, int end, int depth)
    {
        if (begin >= end)
            return -1;
        int axis = depth % 3;
        int mid = (begin + end) / 2;
        auto comparator = [this, axis](int lhs, int rhs)
        {
            return photons[lhs].position[axis] < photons[rhs].position[axis];
        };
        std::nth_element(indices.begin() + begin, indices.begin() + mid, indices.begin() + end, comparator);

        int nodeIndex = static_cast<int>(nodes.size());
        nodes.push_back({indices[mid], axis, -1, -1});
        nodes[nodeIndex].left = buildTreeRecursive(indices, begin, mid, depth + 1);
        nodes[nodeIndex].right = buildTreeRecursive(indices, mid + 1, end, depth + 1);
        return nodeIndex;
    }

    void PhotonMap::gatherRecursive(int nodeIndex,
                                    const Vec3 &position,
                                    float &maxRadius2,
                                    size_t gatherCount,
                                    std::priority_queue<std::pair<float, int>> &heap) const
    {
        if (nodeIndex < 0)
            return;
        const Node &node = nodes[nodeIndex];
        const Photon &photon = photons[node.photonIndex];
        float dist2 = glm::dot(photon.position - position, photon.position - position);
        if (dist2 <= maxRadius2)
        {
            heap.emplace(dist2, node.photonIndex);
            if (heap.size() > gatherCount)
            {
                heap.pop();
            }
            if (heap.size() == gatherCount)
            {
                maxRadius2 = heap.top().first;
            }
        }

        float diff = position[node.axis] - photon.position[node.axis];
        int first = diff <= 0.0f ? node.left : node.right;
        int second = diff <= 0.0f ? node.right : node.left;

        gatherRecursive(first, position, maxRadius2, gatherCount, heap);

        if (diff * diff < maxRadius2 || heap.size() < gatherCount)
        {
            gatherRecursive(second, position, maxRadius2, gatherCount, heap);
        }
    }
}
