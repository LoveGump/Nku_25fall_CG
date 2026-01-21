#pragma once
#ifndef __SIMPLE_PATH_TRACER_HPP__
#define __SIMPLE_PATH_TRACER_HPP__

#include "scene/Scene.hpp"
#include "Ray.hpp"
#include "Camera.hpp"
#include "intersections/HitRecord.hpp"

#include "shaders/ShaderCreator.hpp"
#include "Photon.hpp"
#include "PhotonMap.hpp"

#include <tuple>
namespace SimplePathTracer
{
    using namespace NRenderer;
    using namespace std;

    class SimplePathTracerRenderer
    {
    public:
    private:
        SharedScene spScene;
        Scene& scene;

        unsigned int width;
        unsigned int height;
        unsigned int depth;
        unsigned int samples;

        using SCam = SimplePathTracer::Camera;
        SCam camera;

        vector<SharedShader> shaderPrograms;
        // Photon mapping
        std::unique_ptr<class PhotonMap> photonMap;
        std::vector<class Photon> photons;
        float gatherRadius {0.0f};
        bool usePhotonMapRendering {false};
        unsigned int photonCount {50000};
    public:
        SimplePathTracerRenderer(SharedScene spScene)
            : spScene               (spScene)
            , scene                 (*spScene)
            , camera                (spScene->camera)
        {
            width = scene.renderOption.width;
            height = scene.renderOption.height;
            depth = scene.renderOption.depth;
            samples = scene.renderOption.samplesPerPixel;
            // 从场景参数读取独立配置
            photonCount = scene.renderOption.photonCount;
            gatherRadius = scene.renderOption.photonGatherRadius; // 若为0将在构建后自动估计
            usePhotonMapRendering = scene.renderOption.enablePhotonMapping;
        }
        ~SimplePathTracerRenderer() = default;

        using RenderResult = tuple<RGBA*, unsigned int, unsigned int>;
        RenderResult render();
        void release(const RenderResult& r);

    private:
        void renderTask(RGBA* pixels, int width, int height, int off, int step);

        RGB gamma(const RGB& rgb);
        RGB trace(const Ray& ray, int currDepth);
        RGB traceFinalGather(const Ray& ray, int currDepth);
        HitRecord closestHitObject(const Ray& r) const;
        tuple<float, Vec3> closestHitLight(const Ray& r);

        // Photon mapping helpers
        // 建立光子图并记录能量守恒信息
        void buildPhotonMapAndLog(unsigned int N);
        // 在击中点处进行辐射度估计
        RGB gatherRadianceAtHit(const Vec3& hitPoint, const Vec3& normal, int materialIndex) const;
        // 计算场景边界盒
        void computeSceneBounds(Vec3& bmin, Vec3& bmax) const;
        // Export photon map visualization as a point image from current camera
        void exportPhotonMapImagePoints(const std::string& filenameBase) const;
    };
}

#endif
