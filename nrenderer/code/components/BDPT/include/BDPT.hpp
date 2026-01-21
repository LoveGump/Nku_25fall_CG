#pragma once
#ifndef __SIMPLE_PATH_TRACER_HPP__
#define __SIMPLE_PATH_TRACER_HPP__

#include "Camera.hpp"
#include "Ray.hpp"
#include "Vertex.hpp"
#include "intersections/HitRecord.hpp"
#include "scene/Scene.hpp"

#include "shaders/ShaderCreator.hpp"

#include <tuple>
namespace BDPT
{
using namespace NRenderer;
using namespace std;

class BDPT_Render
{
  public:
  private:
    SharedScene spScene;
    Scene &scene;

    unsigned int width;
    unsigned int height;
    unsigned int depth;
    unsigned int samples;

    // float epsilon = 1000.0f;

    using SCam = BDPT::Camera;
    SCam camera;

    vector<SharedShader> shaderPrograms;

  public:
    BDPT_Render(SharedScene spScene) : spScene(spScene), scene(*spScene), camera(spScene->camera)
    {
        width = scene.renderOption.width;
        height = scene.renderOption.height;
        depth = scene.renderOption.depth;
        samples = scene.renderOption.samplesPerPixel;
    }
    ~BDPT_Render() = default;

    using RenderResult = tuple<RGBA *, unsigned int, unsigned int>;
    RenderResult render();
    void release(const RenderResult &r);

  private:
    void renderTask(RGBA *pixels, int width, int height, int off, int step);

    RGB gamma(const RGB &rgb);
    RGB evaluateRGB(auto &sampler, Ray cameraRay, int width, int height);
    HitRecord closestHitObject(const Ray &r);
    tuple<float, Vec3, HitRecord, AreaLight> closestHitLight(const Ray &r);

    Vertex randomWalk(const Ray &r, auto &sampler, Vertex lastVertex, bool isLight = false);
    int generateCameraSubpath(Camera camera, Ray ray, auto &sampler, int depth, std::vector<Vertex>& subpath);
    int generateLightSubpath(AreaLight light, Camera camera, auto &sampler, int depth, std::vector<Vertex>& subpath);
    Ray sampleRay(AreaLight a, auto &sampler);
    RGB connectBDPT(Camera camera, AreaLight light, auto &sampler, std::vector<Vertex>& cameraPath,
                    std::vector<Vertex>& LightPath, int t, int s, int width, int height);
    float MISWeight(Camera camera, std::vector<Vertex>& lightVertices, std::vector<Vertex>& cameraVertices,
                    Vertex &sampled, int s, int t, auto lightSampler);
};

} // namespace BDPT

#endif