// ???????????????
// ??????????????????????????????????
#include "RayCastRenderer.hpp"
#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"
#include <random>

// ???????????? KD-Tree ???????????????
#ifndef NR_USE_KDTREE
#define NR_USE_KDTREE 1
#endif

namespace RayCastKD
{
    // ?????????
    // r: ??????
    void RayCastRenderer::release(const RenderResult &r)
    {
        auto [p, w, h] = r;
        delete[] p;
    }

    // ???????
    // rgb: ?????RGB???
    // ??????????????
    RGB RayCastRenderer::gamma(const RGB &rgb)
    {
        return glm::sqrt(rgb);
    }

    // ???????
    // ??????????
    auto RayCastRenderer::render() -> RenderResult
    {
        auto width = scene.renderOption.width;
        auto height = scene.renderOption.height;
        // ?????????????
        auto pixels = new RGBA[width * height];

        // ?????????
        VertexTransformer vertexTransformer{};
        vertexTransformer.exec(spScene);

        // ?????????????
        ShaderCreator shaderCreator{};
        for (auto &mtl : scene.materials)
        {
            shaderPrograms.push_back(shaderCreator.create(mtl, scene.textures));
        }

        // ??????????????????
        // ∑÷≈‰÷°ª∫≥ÂœÒÀÿ£®∞¥≈‰÷√µƒ∑÷±Ê¬ £©
        auto width = scene.renderOption.width;
        auto height = scene.renderOption.height;
        // ?????????????
        auto pixels = new RGBA[width * height];

        // ?????????
        VertexTransformer vertexTransformer{};
        vertexTransformer.exec(spScene);

        // ?????????????
        ShaderCreator shaderCreator{};
        for (auto &mtl : scene.materials)
        {
            shaderPrograms.push_back(shaderCreator.create(mtl, scene.textures));
        }

        // ??????????????????
        buildAccelOnce();

        // ???????????????????
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                // ???????
                auto ray = camera.shoot(float(j) / float(width), float(i) / float(height));
                // ???????????
            auto color = trace(ray, int(scene.renderOption.depth));
                // ???????
                color = clamp(color);
                color = gamma(color);

                // ????????????????????y??
                pixels[(height - i - 1) * width + j] = {color, 1};
            }
        }

        return {pixels, width, height};
    }

    // ???????
    // r: ???????
    // ????????????????
    RGB RayCastRenderer::trace(const Ray &r)
    {
        return trace(r, int(scene.renderOption.depth));
    }

    RGB RayCastRenderer::trace(const Ray &r, int depth)
    {
        if (depth <= 0)
        {
            return {0, 0, 0};
        }
        // 1) ????????????????????????????
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

        // 2) ???????????????
        auto closestHitObj = closestHit(r);

        // ?????????????????????????????????????????????
        if ((!closestHitObj && tLight != FLOAT_INF) || (closestHitObj && closestHitObj->t > tLight))
        {
            return emitted;
        }

        // 3) ?????????????????? + ??????????????
        if (closestHitObj)
        {
            auto &hitRec = *closestHitObj;
            Vec3 result{0, 0, 0};
            // ??????????+???
            for (auto &l : scene.pointLightBuffer)
            {
                auto out = glm::normalize(l.position - hitRec.hitPoint);
                if (glm::dot(out, hitRec.normal) <= 0)
                    continue; // ????????

                auto distance = glm::length(l.position - hitRec.hitPoint);
                auto shadowRay = Ray{hitRec.hitPoint, out};
                auto shadowHit = closestHit(shadowRay);

                bool visible = (!shadowHit) || (shadowHit && shadowHit->t > distance);
                if (!visible)
                    continue;

                auto c = shaderPrograms[hitRec.material.index()]->shade(-r.direction, out, hitRec.normal);
                float att = 1.0f / (distance * distance + 1e-4f);
                result += c * (l.intensity * att);
            }

            // ??????????????????????????????
            for (auto &a : scene.areaLightBuffer)
            {
                // ??????????????????????????
                static thread_local std::mt19937 rng(1337);
                std::uniform_real_distribution<float> U(0.0f, 1.0f);
                Vec3 sum{0};
                unsigned int ns = areaSamples;
                if (ns == 0)
                    ns = 1;
                for (unsigned int s = 0; s < ns; ++s)
                {
                    float ru = U(rng), rv = U(rng);
                    Vec3 lp = a.position + ru * a.u + rv * a.v; // ??????
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

            // ????/??????
            const float eps = 1e-3f;
            Vec3 n = glm::normalize(hitRec.normal);
            Vec3 inDir = glm::normalize(r.direction);

            auto &mtl = scene.materials[hitRec.material.index()];
            using PW = NRenderer::Property::Wrapper;

            // ????????????????????????
            float roughness = 0.5f;
            if (auto ropt = mtl.getProperty<PW::FloatType>("roughness")) roughness = glm::clamp(ropt->value, 0.0f, 1.0f);

            // ??? IOR -> ???????????????
            float ior = 1.0f;
            if (auto iopt = mtl.getProperty<PW::FloatType>("ior")) ior = glm::max(1e-4f, iopt->value);

            // ??? F0???????
            Vec3 F0 = Vec3(0.04f);
            if (auto f0opt = mtl.getProperty<PW::RGBType>("F0")) F0 = f0opt->value;
            else if (ior > 1.0001f) {
                float f0s = glm::pow((ior - 1.f) / (ior + 1.f), 2.f);
                F0 = Vec3(f0s);
            }

            // ?????????????????????????
            float transparency = 0.0f;
            if (auto topt = mtl.getProperty<PW::FloatType>("transparency")) transparency = glm::clamp(topt->value, 0.0f, 1.0f);
            else if (auto trgb = mtl.getProperty<PW::RGBType>("transparency")) {
                auto v = trgb->value; transparency = glm::clamp((v.r + v.g + v.b) / 3.0f, 0.0f, 1.0f);
            }

            // ????????????????????/???????????
            if (depth > 0 && roughness < 0.1f)
            {
                // ?????????????
                float NoI = glm::dot(n, inDir);
                Vec3 nl = n;
                // glm::refract ???? eta = n1/n2
                // ??->??: n1=1, n2=ior, eta=1/ior; ??->??: n1=ior, n2=1, eta=ior
                if (NoI > 0.0f) nl = -n; // ?????????????????

                float cosTheta = glm::abs(glm::dot(nl, -inDir));
                Vec3 F = F0 + (Vec3(1.0f) - F0) * glm::pow(1.0f - cosTheta, 5.0f);
                float Fr = glm::clamp((F.r + F.g + F.b) / 3.0f, 0.0f, 1.0f);

                // ???????
                Vec3 reflDir = glm::reflect(inDir, nl);
                Ray reflRay{hitRec.hitPoint + nl * eps, glm::normalize(reflDir)};
                Vec3 reflCol = trace(reflRay, depth - 1);

                // ????????????????????
                Vec3 refrCol{0.0f};
                if (transparency > 0.0f)
                {
                    // glm::refract??I ???????N ?????eta ?????????n1/n2??
                    float eta = (NoI < 0.0f) ? (1.0f / glm::max(1e-4f, ior)) : (glm::max(1e-4f, ior));
                    Vec3 refrDir = glm::refract(inDir, nl, eta);
                    // ???????????refrDir ???????????
                    if (glm::dot(refrDir, refrDir) > 1e-8f)
                    {
                        Ray refrRay{hitRec.hitPoint - nl * eps, glm::normalize(refrDir)};
                        refrCol = trace(refrRay, depth - 1);
                    }
                    else
                    {
                        // ????????????????
                        refrCol = Vec3(0.0f);
                        Fr = 1.0f;
                    }
                }

                // ?????????????????? transparency ?????
                result += reflCol * Fr + refrCol * ((1.0f - Fr) * transparency);
            }
            return result;
        }

        // 4) ??????????????????
        return {0, 0, 0};
    }

    // ???????????????
    // r: ???????
    // ??????????????
    HitRecord RayCastRenderer::closestHit(const Ray &r)
    {
        HitRecord closestHit = nullopt;
        float closest = FLOAT_INF;
        // ????????????
        for (auto &s : scene.sphereBuffer)
        {
            auto hitRecord = Intersection::xSphere(r, s, 0.01f, closest);
            if (hitRecord && hitRecord->t < closest)
            {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        // ??????????????? KD-Tree
        if (kdtree)
        {
            auto hitRecord = kdtree->intersect(r, 0.01f, closest);
            if (hitRecord && hitRecord->t < closest)
            {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        else
        {
            // ??????????????
            for (auto &t : scene.triangleBuffer)
            {
                auto hitRecord = Intersection::xTriangle(r, t, 0.01f, closest);
                if (hitRecord && hitRecord->t < closest)
                {
                    closest = hitRecord->t;
                    closestHit = hitRecord;
                }
            }
            for (auto &m : scene.meshBuffer)
            {
                auto hitRecord = Intersection::xMesh(r, m, 0.01f, closest);
                if (hitRecord && hitRecord->t < closest)
                {
                    closest = hitRecord->t;
                    closestHit = hitRecord;
                }
            }
        }
        // ???????????
        for (auto &p : scene.planeBuffer)
        {
            auto hitRecord = Intersection::xPlane(r, p, 0.01f, closest);
            if (hitRecord && hitRecord->t < closest)
            {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        return closestHit;
    }

    void RayCastRenderer::buildAccelOnce()
    {
        if (accelReady)
            return;
        // ??????????????????????????
        triPrims.clear();
        // ?????????????????????????? + ???????????
        size_t estMeshTris = 0;
        for (const auto &m : scene.meshBuffer) estMeshTris += (m.positionIndices.size() / 3);
        triPrims.reserve(scene.triangleBuffer.size() + estMeshTris);

        // ??????????????
        for (const auto &t : scene.triangleBuffer)
        {
            triPrims.push_back(t);
        }
        // ???????????????
        for (const auto &m : scene.meshBuffer)
        {
            for (size_t i = 0; i + 2 < m.positionIndices.size(); i += 3)
            {
                Triangle tri;
                auto i0 = m.positionIndices[i];
                auto i1 = m.positionIndices[i + 1];
                auto i2 = m.positionIndices[i + 2];
                tri.v1 = m.positions[i0];
                tri.v2 = m.positions[i1];
                tri.v3 = m.positions[i2];
                // ?????????????
                if (m.hasVertexNormal() && i0 < m.vertexNormals.size() && i1 < m.vertexNormals.size() && i2 < m.vertexNormals.size())
                {
                    tri.vertexNormals[0] = m.vertexNormals[i0];
                    tri.vertexNormals[1] = m.vertexNormals[i1];
                    tri.vertexNormals[2] = m.vertexNormals[i2];
                }
                else
                {
                    tri.vertexNormals[0] = tri.vertexNormals[1] = tri.vertexNormals[2] = NRenderer::Vec3(0.0f);
                }
                // ??Q??
                tri.normal = glm::normalize(glm::cross(tri.v2 - tri.v1, tri.v3 - tri.v1));
                tri.material = m.material;
                triPrims.push_back(tri);
            }
        }

        #if NR_USE_KDTREE
            if (!triPrims.empty())
            {
                kdtree = std::make_unique<KDTree>();
                // ????????????????? KDTree???????????????????
                if (triPrims.size() >= 1024)
                    kdtree->build(triPrims);
                else
                    kdtree->build(triPrims, 32, 32);
            }
            else
            {
                kdtree.reset();
            }
        #else
            // ??? KD-Tree?????????????
            kdtree.reset();
        #endif

        accelReady = true;
    }
}