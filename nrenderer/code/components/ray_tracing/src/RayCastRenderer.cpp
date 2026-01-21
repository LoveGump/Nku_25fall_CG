// 光线投射渲染器实现
// 实现了基本的光线投射渲染算法，包括阴影计算
#include "RayCastRenderer.hpp"
#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"
#include <random>
#include <iostream>
#include <cmath>

namespace Ray_Tracing
{
    // 释放渲染结果
    // r: 渲染结果
    void RayCastRenderer::release(const RenderResult &r)
    {
        auto [p, w, h] = r;
        delete[] p;
    }

    // 伽马校正
    // rgb: 输入的RGB颜色
    // 返回校正后的颜色
    RGB RayCastRenderer::gamma(const RGB &rgb)
    {
        return glm::sqrt(rgb);
    }

    // 渲染场景
    // 返回渲染结果
    auto RayCastRenderer::render() -> RenderResult
    {
        auto width = scene.renderOption.width;
        auto height = scene.renderOption.height;
        // 创建像素缓冲区
        auto pixels = new RGBA[width * height];

        // 执行顶点变换
        VertexTransformer vertexTransformer{};
        vertexTransformer.exec(spScene);

        // 创建着色器程序
        ShaderCreator shaderCreator{};
        for (auto &mtl : scene.materials)
        {
            shaderPrograms.push_back(shaderCreator.create(mtl, scene.textures));
        }

        // 将场景的 samplesPerPixel 用作面光源采样数
        areaSamples = std::max(1u, scene.renderOption.samplesPerPixel);

        // 对每个像素进行光线追踪
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                // 生成光线
                auto ray = camera.shoot(float(j) / float(width), float(i) / float(height));
                // 追踪光线获取颜色
                auto color = trace(ray, int(scene.renderOption.depth));
                // 颜色后处理
                color = clamp(color);
                color = gamma(color);

                // 存储到像素缓冲区（注意翻转y轴）
                pixels[(height - i - 1) * width + j] = {color, 1};
            }
        }

        return {pixels, width, height};
    }

    // 光线追踪
    // r: 输入光线
    // 返回该光线对应的颜色
    RGB RayCastRenderer::trace(const Ray &r)
    {
        return trace(r, int(scene.renderOption.depth));
    }

    RGB RayCastRenderer::trace(const Ray &r, int depth)
    {
        if (depth <= 0)
            return {0, 0, 0};

        // 1) 直接命中面光源：返回其辐射度
        float tLight = FLOAT_INF;
        Vec3 emitted = {0, 0, 0};
        for (auto &a : scene.areaLightBuffer)
        {
            auto hitL = Intersection::xAreaLight(r, a, 0.000001f, tLight);
            if (hitL && hitL->t < tLight)
            {
                tLight = hitL->t;
                emitted = a.radiance;
            }
        }

        // 2) 查找最近物体
        auto closestHitObj = closestHit(r);
        if ((!closestHitObj && tLight != FLOAT_INF) || (closestHitObj && closestHitObj->t > tLight))
        {
            return emitted;
        }

        // 3) 物体命中：直射（点光与面光）
        if (closestHitObj)
        {
            auto &hitRec = *closestHitObj;
            Vec3 result{0};

            // 点光源直射 + 硬阴影 + 1/r^2 衰减
            for (auto &l : scene.pointLightBuffer)
            {
                auto out = glm::normalize(l.position - hitRec.hitPoint);
                if (glm::dot(out, hitRec.normal) <= 0)
                    continue;
                auto distance = glm::length(l.position - hitRec.hitPoint);
                Ray shadowRay{hitRec.hitPoint, out};
                auto shadowHit = closestHit(shadowRay);
                bool visible = (!shadowHit) || (shadowHit && shadowHit->t > distance);
                if (!visible)
                    continue;
                auto c = shaderPrograms[hitRec.material.index()]->shade(-r.direction, out, hitRec.normal);
                float att = 1.0f / (distance * distance + 1e-4f);
                result += c * (l.intensity * att);
            }

            // 面光源多样本软阴影
            for (auto &a : scene.areaLightBuffer)
            {
                static thread_local std::mt19937 rng(1337);
                std::uniform_real_distribution<float> U(0.0f, 1.0f);
                Vec3 sum{0};
                unsigned int ns = areaSamples;
                if (ns == 0)
                    ns = 1;
                for (unsigned int s = 0; s < ns; ++s)
                {
                    float ru = U(rng), rv = U(rng);
                    Vec3 lp = a.position + ru * a.u + rv * a.v;
                    auto out = glm::normalize(lp - hitRec.hitPoint);
                    float nDotL = glm::dot(out, hitRec.normal);
                    if (nDotL <= 0)
                        continue;
                    float distance = glm::length(lp - hitRec.hitPoint);
                    Ray shadowRay{hitRec.hitPoint, out};
                    auto shadowHit = closestHit(shadowRay);
                    bool visible = (!shadowHit) || (shadowHit && shadowHit->t > distance);
                    if (!visible)
                        continue;
                    auto c = shaderPrograms[hitRec.material.index()]->shade(-r.direction, out, hitRec.normal);
                    Vec3 ln = glm::normalize(glm::cross(a.u, a.v));
                    float cosL = glm::max(0.0f, glm::dot(ln, -out));
                    float area = glm::length(glm::cross(a.u, a.v));
                    float att = cosL * area / (distance * distance + 1e-4f);
                    sum += c * (a.radiance * att);
                }
                result += (sum / float(ns));
            }

            // 4) 反射/折射递归（理想模型）
            const float eps = 1e-3f;
            Vec3 n = glm::normalize(hitRec.normal);
            Vec3 inDir = glm::normalize(r.direction);
            auto &mtl = scene.materials[hitRec.material.index()];
            using PW = NRenderer::Property::Wrapper;

            float roughness = 0.5f;
            if (auto ropt = mtl.getProperty<PW::FloatType>("roughness"))
                roughness = glm::clamp(ropt->value, 0.0f, 1.0f);
            float ior = 1.0f;
            if (auto iopt = mtl.getProperty<PW::FloatType>("ior"))
                ior = glm::max(1e-4f, iopt->value);
            Vec3 F0 = Vec3(0.04f);
            if (auto f0opt = mtl.getProperty<PW::RGBType>("F0"))
                F0 = f0opt->value;
            else if (ior > 1.0001f)
            {
                float f0s = glm::pow((ior - 1.f) / (ior + 1.f), 2.f);
                F0 = Vec3(f0s);
            }
            float transparency = 0.0f;
            if (auto topt = mtl.getProperty<PW::FloatType>("transparency"))
                transparency = glm::clamp(topt->value, 0.0f, 1.0f);
            else if (auto trgb = mtl.getProperty<PW::RGBType>("transparency"))
            {
                auto v = trgb->value;
                transparency = glm::clamp((v.r + v.g + v.b) / 3.0f, 0.0f, 1.0f);
            }

            if (depth > 0 && roughness < 0.1f)
            {
                float NoI = glm::dot(n, inDir);
                Vec3 nl = (NoI > 0.0f) ? -n : n; // 从内部射出翻转法线
                float cosTheta = glm::abs(glm::dot(nl, -inDir));
                Vec3 F = F0 + (Vec3(1.0f) - F0) * glm::pow(1.0f - cosTheta, 5.0f);
                float Fr = glm::clamp((F.r + F.g + F.b) / 3.0f, 0.0f, 1.0f);

                // 反射
                Vec3 reflDir = glm::reflect(inDir, nl);
                Ray reflRay{hitRec.hitPoint + nl * eps, glm::normalize(reflDir)};
                Vec3 reflCol = trace(reflRay, depth - 1);

                // 折射
                Vec3 refrCol{0.0f};
                if (transparency > 0.0f)
                {
                    // 明确 n1/n2 的含义并保证数值稳定
                    float n1 = (NoI < 0.0f) ? 1.0f : ior;
                    float n2 = (NoI < 0.0f) ? ior : 1.0f;
                    float eta = glm::max(1e-8f, n1 / n2);
                    Vec3 refrDir = glm::refract(inDir, nl, eta);
                    if (glm::dot(refrDir, refrDir) > 1e-8f)
                    {
                        // 将折射射线起点沿折射方向偏移，避免自相交
                        Ray refrRay{hitRec.hitPoint + refrDir * eps, glm::normalize(refrDir)};
                        refrCol = trace(refrRay, depth - 1);

#ifdef _DEBUG
                        // 调试：验证 Snell 定律（仅在调试构建时输出）
                        float cos1 = glm::clamp(glm::dot(-inDir, nl), -1.0f, 1.0f);
                        float sin1 = std::sqrt(glm::max(0.0f, 1.0f - cos1 * cos1));
                        float cos2 = glm::clamp(glm::dot(-refrDir, nl), -1.0f, 1.0f);
                        float sin2 = std::sqrt(glm::max(0.0f, 1.0f - cos2 * cos2));
                        float lhs = n1 * sin1;
                        float rhs = n2 * sin2;
                        float err = std::fabs(lhs - rhs);
                        if (err > 1e-3f)
                        {
                            std::cerr << "Snell check failed: n1*sin1=" << lhs << " n2*sin2=" << rhs << " err=" << err << std::endl;
                        }
#endif
                    }
                    else
                    {
                        Fr = 1.0f; // 全反射
                    }
                }
                result += reflCol * Fr + refrCol * ((1.0f - Fr) * transparency);
            }
            return result;
        }
        // 未命中
        return {0, 0, 0};
    }

    // 计算最近的相交物体
    // r: 输入光线
    // 返回最近的相交记录
    HitRecord RayCastRenderer::closestHit(const Ray &r)
    {
        HitRecord closestHit = nullopt;
        float closest = FLOAT_INF;
        // 检查与球体的相交
        for (auto &s : scene.sphereBuffer)
        {
            auto hitRecord = Intersection::xSphere(r, s, 0.01, closest);
            if (hitRecord && hitRecord->t < closest)
            {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        // 检查与三角形的相交
        for (auto &t : scene.triangleBuffer)
        {
            auto hitRecord = Intersection::xTriangle(r, t, 0.01, closest);
            if (hitRecord && hitRecord->t < closest)
            {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        // 检查与平面的相交
        for (auto &p : scene.planeBuffer)
        {
            auto hitRecord = Intersection::xPlane(r, p, 0.01, closest);
            if (hitRecord && hitRecord->t < closest)
            {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        // 检查与网格的相交
        for (auto &m : scene.meshBuffer)
        {
            auto hitRecord = Intersection::xMesh(r, m, 0.01, closest);
            if (hitRecord && hitRecord->t < closest)
            {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        return closestHit;
    }
}