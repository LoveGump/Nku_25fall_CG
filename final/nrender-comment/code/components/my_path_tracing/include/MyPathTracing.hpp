#pragma once
#ifndef __MY_PATH_TRACING_HPP__
#define __MY_PATH_TRACING_HPP__

#include "scene/Scene.hpp"
#include "Ray.hpp"
#include "Camera.hpp"
#include "intersections/HitRecord.hpp"
#include "acceleration/KDTree.hpp"

#include "shaders/ShaderCreator.hpp"

#include <tuple>
#include <memory>

namespace MyPathTracing
{
    using namespace NRenderer;
    using namespace std;

    class MyPathTracingRenderer
    {
    private:
        SharedScene spScene;
        Scene &scene;

        unsigned int width;
        unsigned int height;
        unsigned int depth;
        unsigned int samples;

        using SCam = MyPathTracing::Camera;
        SCam camera;

        vector<SharedShader> shaderPrograms;
        std::unique_ptr<KDTree> kdTree;
        bool useKDTree = true;

    public:
        MyPathTracingRenderer(SharedScene spScene)
            : spScene(spScene), scene(*spScene), camera(spScene->camera)
        {
            width = scene.renderOption.width;
            height = scene.renderOption.height;
            depth = scene.renderOption.depth;
            samples = scene.renderOption.samplesPerPixel;
        }
        ~MyPathTracingRenderer() = default;

        using RenderResult = tuple<RGBA *, unsigned int, unsigned int>;

        RenderResult render();
        void release(const RenderResult &r);

    private:
        void renderTask(RGBA *pixels, int width, int height, int off, int step);
        RGB gamma(const RGB &rgb);
        RGB trace(const Ray &r, int currDepth, bool isSpecularBounce = false);

        HitRecord closestHitObject(const Ray &r);
        HitRecord closestHitLinear(const Ray &r);
        void buildAcceleration();
        void setUseKDTree(bool enable) { useKDTree = enable; }

        tuple<float, Vec3> closestHitLight(const Ray &r);
    };
}

#endif
