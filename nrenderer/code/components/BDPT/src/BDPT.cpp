#include "server/Server.hpp"

#include "BDPT.hpp"

#include "Vertex.hpp"
#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"

#include "glm/gtc/matrix_transform.hpp"
#include <cmath>
#include <vector>

namespace BDPT
{
RGB BDPT_Render::gamma(const RGB &rgb)
{
    return glm::sqrt(rgb);
}

void BDPT_Render::renderTask(RGBA *pixels, int width, int height, int off, int step)
{
    for (int i = off; i < height; i += step)
    {
        for (int j = 0; j < width; j++)
        {
            Vec3 color{0, 0, 0};
            for (int k = 0; k < samples; k++)
            {
                auto r = defaultSamplerInstance<UniformInSquare>().sample2d();
                float rx = r.x;
                float ry = r.y;
                float x = (float(j) + rx) / float(width);
                float y = (float(i) + ry) / float(height);
                auto ray = camera.shoot(x, y);
                color += evaluateRGB(defaultSamplerInstance<UniformSampler>(), ray, width, height);
                // color += trace(ray, 0);
            }
            color /= samples;
            color = gamma(color);
            pixels[(height - i - 1) * width + j] = {color, 1};
        }
    }
}

auto BDPT_Render::render() -> RenderResult
{
    // shaders
    shaderPrograms.clear();
    ShaderCreator shaderCreator{};
    for (auto &m : scene.materials)
    {
        shaderPrograms.push_back(shaderCreator.create(m, scene.textures));
    }

    RGBA *pixels = new RGBA[width * height]{};

    // 局部坐标转换成世界坐标
    VertexTransformer vertexTransformer{};
    vertexTransformer.exec(spScene);

    const auto taskNums = 8;
    thread t[taskNums];
    for (int i = 0; i < taskNums; ++i)
    {
        t[i] = thread(&BDPT_Render::renderTask, this, pixels, width, height, i, taskNums);
    }
    for (int i = 0; i < taskNums; ++i)
    {
        t[i].join();
    }
    getServer().logger.log("Done...");
    return {pixels, width, height};
}

void BDPT_Render::release(const RenderResult &r)
{
    auto [p, w, h] = r;
    delete[] p;
}

HitRecord BDPT_Render::closestHitObject(const Ray &r)
{
    HitRecord closestHit = nullopt;
    float closest = FLOAT_INF;
    for (auto &s : scene.sphereBuffer)
    {
        auto hitRecord = Intersection::xSphere(r, s, 0.000001, closest);
        if (hitRecord && hitRecord->t < closest)
        {
            closest = hitRecord->t;
            closestHit = hitRecord;
        }
    }
    for (auto &t : scene.triangleBuffer)
    {
        auto hitRecord = Intersection::xTriangle(r, t, 0.000001, closest);
        if (hitRecord && hitRecord->t < closest)
        {
            closest = hitRecord->t;
            closestHit = hitRecord;
        }
    }
    for (auto &p : scene.planeBuffer)
    {
        auto hitRecord = Intersection::xPlane(r, p, 0.000001, closest);
        if (hitRecord && hitRecord->t < closest)
        {
            closest = hitRecord->t;
            closestHit = hitRecord;
        }
    }
    return closestHit;
}

tuple<float, Vec3, HitRecord, AreaLight> BDPT_Render::closestHitLight(const Ray &r)
{
    Vec3 v = {};
    HitRecord closest = getHitRecord(FLOAT_INF, {}, {}, {});
    AreaLight light = {};
    for (auto &a : scene.areaLightBuffer)
    {
        auto hitRecord = Intersection::xAreaLight(r, a, 0.000001, closest->t);
        if (hitRecord && closest->t > hitRecord->t)
        {
            closest = hitRecord;
            light = a;
            v = a.radiance;
        }
    }
    return {closest->t, v, closest, light};
}

RGB BDPT_Render::evaluateRGB(auto &sampler, Ray cameraRay, int width, int height)
{
    std::vector<Vertex> cameraSubpath = std::vector<Vertex>(depth + 2);
    int nCamera = generateCameraSubpath(camera, cameraRay, sampler, depth + 2, cameraSubpath);

    AreaLight light = scene.areaLightBuffer[int(sampler.sample1d() * scene.areaLightBuffer.size())];
    std::vector<Vertex> lightSubpath = std::vector<Vertex>(depth + 1);
    int nLight = generateLightSubpath(light, camera, sampler, depth + 1, lightSubpath);

    RGB L = {0.0f, 0.0f, 0.0f};

    for (int t = 1; t <= nCamera; ++t)
    {
        for (int s = 0; s <= nLight; ++s)
        {
            int depth = t + s - 2;
            if ((s == 1 && t == 1) || depth < 0 || depth > this->depth)
                continue;

            RGB LPath = connectBDPT(camera, light, sampler, lightSubpath, cameraSubpath, s, t, width, height);

            L += LPath;
        }
    }

    // return L / (100.0f * nCamera * nLight);
    return L;
}

int BDPT_Render::generateCameraSubpath(Camera camera, Ray ray, auto &sampler, int depth, std::vector<Vertex> &subpath)
{
    Ray current_ray = ray;
    Vertex lastVertex =
        Vertex(ray, getHitRecord(0, ray.origin, ray.direction, Handle{}), Vec3{0}, 1, 1, Vec3{1}, Vec3{1});
    subpath[0] = lastVertex;
    for (int i = 1; i < depth; ++i)
    {
        Vertex vertex = randomWalk(current_ray, sampler, lastVertex);
        current_ray = vertex.ray;

        vertex.setBeta(lastVertex.beta * vertex.attenuation * vertex.n_dot_in / vertex.pdf);
        subpath[i] = lastVertex = vertex;
        if (vertex.getLightType() == LightType::DeltaDirection)
        {
            return i + 1;
        }
    }
    return depth;
}

int BDPT_Render::generateLightSubpath(AreaLight light, Camera camera, auto &sampler, int depth,
                                      std::vector<Vertex> &subpath)
{
    Ray ray = sampleRay(light, sampler);
    Ray current_ray = ray;
    Vertex lastVertex = Vertex(ray, getHitRecord(0, ray.origin, glm::normalize(glm::cross(light.u, light.v)), {}),
                               light.radiance, 1, 1, Vec3{1}, light.radiance);
    lastVertex.setAttanuation(scene.ambient.constant);
    subpath[0] = lastVertex;
    for (int i = 1; i < depth; ++i)
    {
        Vertex vertex;
        vertex = randomWalk(current_ray, sampler, lastVertex, true);

        current_ray = vertex.ray;

        vertex.beta = vertex.radiance + lastVertex.beta * vertex.attenuation * vertex.n_dot_in / vertex.pdf;
        subpath[i] = lastVertex = vertex;
        if (vertex.getLightType() == LightType::DeltaDirection)
        {
            return i + 1;
        }
    }
    return depth;
}

Vertex BDPT_Render::randomWalk(const Ray &r, auto &sampler, Vertex lastVertex, bool isLight)
{
    HitRecord hitObject = closestHitObject(r);
    auto [t, emitted, hitLight, light] = closestHitLight(r);
    if (!hitObject) // 无限距离，返回反向光线
    {
        Ray back_r = Ray(r.origin + r.direction, -r.direction);
        HitRecord back_p = getHitRecord(1, r.origin + r.direction, -r.direction, lastVertex.hitRecord->material);
        return Vertex(back_r, back_p, lastVertex.getRadiance(), 1, 1, Vec3{1}, Vec3{0}, LightType::Other);
    }
    // 有交点，返回散射光线
    auto mtlHandle = hitObject->material;
    auto scattered = shaderPrograms[mtlHandle.index()]->shade(r, hitObject->hitPoint, hitObject->normal);
    auto scatteredRay = scattered.ray;
    auto attenuation = scattered.attenuation;

    float n_dot_in = abs(glm::dot(hitObject->normal, scatteredRay.direction));
    float pdf = scattered.pdf;

    if (hitObject->t < t)
    {
        auto emitted = scattered.emitted;
        return Vertex(scatteredRay, hitObject, emitted, pdf, n_dot_in, attenuation, Vec3{0});
    }
    else if (hitLight && t != FLOAT_INF)
    {
        // 打到光源上
        return Vertex(scatteredRay, hitLight, emitted, pdf, n_dot_in, attenuation, Vec3{0}, LightType::DeltaDirection);
    }
    Ray back_r = Ray(r.origin + r.direction, -r.direction);
    HitRecord back_p = getHitRecord(1, r.origin + r.direction, -r.direction, mtlHandle);
    return Vertex(back_r, back_p, lastVertex.getRadiance(), 1, 1, Vec3{1}, Vec3{0}, LightType::Other);
}

Ray BDPT_Render::sampleRay(AreaLight a, auto &sampler)
{
    Vec3 normal = glm::normalize(glm::cross(a.v, a.u));
    Vec3 position = a.position + a.u * (sampler.sample1d() - 0.5f) + a.v * (sampler.sample1d() - 0.5f);

    auto new_sampler = defaultSamplerInstance<HemiSphere>();
    Vec3 direction = glm::normalize(new_sampler.sample3d());

    // 确保方向在半球体上，并朝向光源的法线方向
    if (glm::dot(direction, normal) < 0.0f)
    {
        direction = -direction;
    }

    // 归一化方向向量
    direction = glm::normalize(direction);
    swap(direction.x, direction.y);

    return Ray(position, direction);
}

RGB BDPT_Render::connectBDPT(Camera camera, AreaLight light, auto &sampler, std::vector<Vertex> &lightVertices,
                             std::vector<Vertex> &cameraVertices, int s, int t, int width, int height)
{
    RGB L = Vec3{0};

    Vertex sampled;
    if (s == 0)
    {
        const Vertex &pt = cameraVertices[t - 1];
        L = pt.beta * light.radiance;
    }
    else if (t == 1)
    {
        const Vertex &qs = lightVertices[s - 1];
        HitRecord hitRecord = qs.hitRecord;

        if (qs.isConnectible())
        {
            // 采样相机路径
            Vec3 hitpoint = hitRecord->hitPoint;
            Vec2 pRaster = VertexTransformer::worldToRaster(hitpoint, camera, width, height);
            Ray connectionRay = camera.shoot(pRaster.x, pRaster.y);
            sampled = randomWalk(connectionRay, sampler, cameraVertices[t - 1]);
            auto beta = sampled.attenuation * sampled.n_dot_in / sampled.pdf;

            if (closestHitObject(connectionRay)->t >= glm::length(connectionRay.origin - hitpoint))
            {
                float G = abs(glm::dot(qs.hitRecord->normal, -connectionRay.direction)) *
                          abs(glm::dot(sampled.hitRecord->normal, connectionRay.direction)) /
                          pow(glm::distance(qs.hitRecord->hitPoint, sampled.hitRecord->hitPoint), 2);
                int index = qs.hitRecord->material.index();
                auto f = index >= 0 ? shaderPrograms[qs.hitRecord->material.index()]->evaluate(
                                          qs.ray.direction, connectionRay.direction, qs.hitRecord->normal)
                                    : scene.ambient.constant;
                L = qs.beta * f * G * beta;
            }
        }
    }
    else if (s == 1)
    {
        const Vertex &pt = cameraVertices[t - 1];
        HitRecord hitRecord = pt.hitRecord;
        if (!hitRecord.has_value())
        {
            return L;
        }
        if (pt.isConnectible())
        {
            // 采样光源路径
            Vec3 hitpoint = hitRecord.value().hitPoint;
            Ray light_r = sampleRay(light, sampler);
            Ray connectionRay(hitpoint, glm::normalize(light_r.origin - hitpoint));
            sampled = randomWalk(connectionRay, sampler, lightVertices[s - 1], true);
            auto beta = sampled.attenuation * sampled.n_dot_in / sampled.pdf;

            if (closestHitObject(connectionRay)->t >= glm::length(light_r.origin - hitpoint))
            {
                float G = abs(glm::dot(pt.hitRecord->normal, connectionRay.direction)) *
                          abs(glm::dot(sampled.hitRecord->normal, -connectionRay.direction)) /
                          pow(glm::distance(pt.hitRecord->hitPoint, sampled.hitRecord->hitPoint), 2);
                int index = pt.hitRecord->material.index();
                auto f = index >= 0 ? shaderPrograms[index]->evaluate(pt.ray.direction, connectionRay.direction,
                                                                      pt.hitRecord->normal)
                                    : scene.ambient.constant;
                L = pt.beta * f * G * beta;
            }
            // std::cout << "L: " << L.x << ", " << L.y << ", " << L.z << std::endl;
        }
    }
    else
    {
        // 连接子路径
        const Vertex &qs = lightVertices[s - 1], &pt = cameraVertices[t - 1];
        Vec3 direction = glm::normalize(pt.hitRecord->hitPoint - qs.hitRecord->hitPoint);
        Ray connectionRay(qs.hitRecord->hitPoint, direction);

        if (qs.isConnectible() && pt.isConnectible())
        {
            // Check visibility
            if (closestHitObject(connectionRay)->t >= glm::length(pt.hitRecord->hitPoint - qs.hitRecord->hitPoint))
            {
                float G =
                    abs(glm::dot(qs.hitRecord->normal, direction)) * abs(glm::dot(pt.hitRecord->normal, -direction));

                int index = qs.hitRecord->material.index();
                auto f = index >= 0 ? shaderPrograms[index]->evaluate(qs.ray.direction, direction, qs.hitRecord->normal)
                                    : scene.ambient.constant;

                L = qs.beta * qs.attenuation * f * G * pt.beta;
            }
        }
    }

    float misWeight = MISWeight(camera, lightVertices, cameraVertices, sampled, s, t, sampler);

    if (std::isnan(L.x) || std::isnan(L.y) || std::isnan(L.z))
    {
        return Vec3{0};
    }
    return L * misWeight;
}

float BDPT_Render::MISWeight(Camera camera, std::vector<Vertex> &lightVertices, std::vector<Vertex> &cameraVertices,
                             Vertex &sampled, int s, int t, auto lightSampler)
{
    float weight = 1.0f;
    if (s == 0)
    {
        if (t == 1)
        {
            weight = 1.0f;
        }
        else
        {
            weight = 0.0f;
        }
    }
    else if (t == 1)
    {
        if (s == 1)
        {
            weight = 1.0f;
        }
        else
        {
            weight = 0.0f;
        }
    }
    else
    {
        float sum = 0.0f;
        for (int i = 0; i < s; ++i)
        {
            sum += lightVertices[i].pdf;
        }
        for (int i = 0; i < t; ++i)
        {
            sum += cameraVertices[i].pdf;
        }
        weight = 1.0f / sum;
    }
    return weight;
}
} // namespace BDPT
