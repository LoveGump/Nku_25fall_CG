#include "server/Server.hpp"

#include "SimplePathTracer.hpp"

#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"

#include "glm/gtc/matrix_transform.hpp"
#include "Photon.hpp"
#include "PhotonMap.hpp"
#include "Onb.hpp"
#include "samplers/SamplerInstance.hpp"
#include <fstream>
#include <string>
#include <algorithm>

namespace SimplePathTracer
{
    RGB SimplePathTracerRenderer::gamma(const RGB& rgb) {
        return glm::sqrt(rgb);
    }

    void SimplePathTracerRenderer::renderTask(RGBA* pixels, int width, int height, int off, int step) {
        for(int i=off; i<height; i+=step) {
            for (int j=0; j<width; j++) {
                Vec3 color{0, 0, 0};
                for (int k=0; k < samples; k++) {
                    auto r = defaultSamplerInstance<UniformInSquare>().sample2d();
                    float rx = r.x;
                    float ry = r.y;
                    float x = (float(j)+rx)/float(width);
                    float y = (float(i)+ry)/float(height);
                    auto ray = camera.shoot(x, y);
                    if (usePhotonMapRendering && photonMap && !photonMap->empty()) {
                        color += traceFinalGather(ray, 0);
                    } else {
                        color += trace(ray, 0);
                    }
                }
                color /= samples;
                color = gamma(color);
                pixels[(height-i-1)*width+j] = {color, 1};
            }
        }
    }

    auto SimplePathTracerRenderer::render() -> RenderResult {
        // shaders
        shaderPrograms.clear();
        ShaderCreator shaderCreator{};
        for (auto& m : scene.materials) {
            shaderPrograms.push_back(shaderCreator.create(m, scene.textures));
        }

        RGBA* pixels = new RGBA[width*height]{};

        // 模型顶点变换到世界坐标
        VertexTransformer vertexTransformer{};
        vertexTransformer.exec(spScene);

        // 构建光子图并记录能量守恒信息
        if (usePhotonMapRendering) {
            buildPhotonMapAndLog(photonCount);
            // 导出光子图点可视化（基于当前摄像机视角）
            exportPhotonMapImagePoints(std::string("photon_map_") + std::to_string(photonCount));
        }

        const auto taskNums = 8;
        thread t[taskNums];
        for (int i=0; i < taskNums; i++) {
            t[i] = thread(&SimplePathTracerRenderer::renderTask,
                this, pixels, width, height, i, taskNums);
        }
        for(int i=0; i < taskNums; i++) {
            t[i].join();
        }
        getServer().logger.log("Done...");
        return {pixels, width, height};
    }

    void SimplePathTracerRenderer::release(const RenderResult& r) {
        auto [p, w, h] = r;
        delete[] p;
    }

    HitRecord SimplePathTracerRenderer::closestHitObject(const Ray& r) const {
        HitRecord closestHit = nullopt;
        float closest = FLOAT_INF;
        for (auto& s : scene.sphereBuffer) {
            auto hitRecord = Intersection::xSphere(r, s, 0.000001, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        for (auto& t : scene.triangleBuffer) {
            auto hitRecord = Intersection::xTriangle(r, t, 0.000001, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        for (auto& p : scene.planeBuffer) {
            auto hitRecord = Intersection::xPlane(r, p, 0.000001, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        return closestHit; 
    }
    
    tuple<float, Vec3> SimplePathTracerRenderer::closestHitLight(const Ray& r) {
        Vec3 v = {};
        HitRecord closest = getHitRecord(FLOAT_INF, {}, {}, {});
        for (auto& a : scene.areaLightBuffer) {
            auto hitRecord = Intersection::xAreaLight(r, a, 0.000001, closest->t);
            if (hitRecord && closest->t > hitRecord->t) {
                closest = hitRecord;
                v = a.radiance;
            }
        }
        return { closest->t, v };
    }

    RGB SimplePathTracerRenderer::traceFinalGather(const Ray& r, int currDepth) {
        if (currDepth >= depth) return Vec3{0};

        auto hitObject = closestHitObject(r);
        auto [tLight, Le] = closestHitLight(r);

        if (hitObject && hitObject->t < tLight) {
            const auto& m = scene.materials[hitObject->material.index()];

            auto getFloat = [&](const std::string& key, float defVal) {
                for (const auto& prop : m.properties) {
                    if (prop.key == key && prop.type == Property::Type::FLOAT) {
                        return std::get<Property::Wrapper::FloatType>(prop.valueWrapper).value;
                    }
                }
                return defVal;
            };
            auto getRGB = [&](const std::string& key, Vec3 defVal) {
                for (const auto& prop : m.properties) {
                    if (prop.key == key && prop.type == Property::Type::RGB) {
                        return std::get<Property::Wrapper::RGBType>(prop.valueWrapper).value;
                    }
                }
                return defVal;
            };

            float reflectivity = getFloat("reflectivity", 0.0f);
            float transparency = getFloat("transparency", 0.0f);
            float ior = getFloat("refractionIndex", 1.5f);
            Vec3 normal = hitObject->normal;
            Vec3 dir = glm::normalize(r.direction);
            const float eps = 1e-4f;

            if (reflectivity > 1e-3f || transparency > 1e-3f) {
                Vec3 result{0.0f};
                if (reflectivity > 1e-3f) {
                    Vec3 reflDir = glm::reflect(dir, normal);
                    result += reflectivity * traceFinalGather(Ray{ hitObject->hitPoint + normal * eps, glm::normalize(reflDir) }, currDepth + 1);
                }
                if (transparency > 1e-3f) {
                    Vec3 n = normal;
                    float etai = 1.0f, etat = ior;
                    float cosi = glm::clamp(glm::dot(-dir, n), -1.0f, 1.0f);
                    if (cosi < 0.0f) { cosi = -cosi; std::swap(etai, etat); n = -n; }
                    float eta = etai / etat;
                    float k = 1.0f - eta * eta * (1.0f - cosi * cosi);
                    if (k >= 0.0f) {
                        Vec3 refrDir = glm::normalize(eta * dir + (eta * cosi - std::sqrt(k)) * n);
                        result += transparency * traceFinalGather(Ray{ hitObject->hitPoint - n * eps, refrDir }, currDepth + 1);
                    }
                }
                return result;
            } else {
                return gatherRadianceAtHit(hitObject->hitPoint, hitObject->normal, hitObject->material.index());
            }
        } else if (tLight != FLOAT_INF) {
            return Le;
        } else {
            return Vec3{0};
        }
    }

    RGB SimplePathTracerRenderer::trace(const Ray& r, int currDepth) {
        if (currDepth == depth) return scene.ambient.constant;
        auto hitObject = closestHitObject(r);
        auto [ t, emitted ] = closestHitLight(r);
        // hit object
        if (hitObject && hitObject->t < t) {
            auto mtlHandle = hitObject->material;
            auto scattered = shaderPrograms[mtlHandle.index()]->shade(r, hitObject->hitPoint, hitObject->normal);
            auto scatteredRay = scattered.ray;
            auto attenuation = scattered.attenuation;
            auto emitted = scattered.emitted;
            auto next = trace(scatteredRay, currDepth+1);
            float n_dot_in = glm::dot(hitObject->normal, scatteredRay.direction);
            float pdf = scattered.pdf;
            /**
             * emitted      - Le(p, w_0)
             * next         - Li(p, w_i)
             * n_dot_in     - cos<n, w_i>
             * atteunation  - BRDF
             * pdf          - p(w)
             **/
            return emitted + attenuation * next * n_dot_in / pdf;
        }
        // 
        else if (t != FLOAT_INF) {
            return emitted;
        }
        else {
            return Vec3{0};
        }
    }

    void SimplePathTracerRenderer::computeSceneBounds(Vec3& bmin, Vec3& bmax) const {
        bmin = Vec3{FLT_MAX};
        bmax = Vec3{-FLT_MAX};
        for (auto& t : scene.triangleBuffer) {
            for (int i=0;i<3;++i) {
                bmin = glm::min(bmin, t.v[i]);
                bmax = glm::max(bmax, t.v[i]);
            }
        }
        for (auto& s : scene.sphereBuffer) {
            Vec3 c = s.position;
            float r = s.radius;
            bmin = glm::min(bmin, c - Vec3{r});
            bmax = glm::max(bmax, c + Vec3{r});
        }
        for (auto& p : scene.planeBuffer) {
            // 近似地用位置点作为参考，不改变已有范围
            bmin = glm::min(bmin, p.position);
            bmax = glm::max(bmax, p.position);
        }
        if (bmin.x == FLT_MAX) { bmin = Vec3{-1}; bmax = Vec3{1}; }
    }

    void SimplePathTracerRenderer::buildPhotonMapAndLog(unsigned int N) {
        photons.clear();
        photons.reserve(N);

        // 统计所有面积光的总功率(按亮度加权)
        auto luminance = [](const Vec3& c){ return 0.2126f*c.x + 0.7152f*c.y + 0.0722f*c.z; };
        vector<float> lightWeights;
        vector<Vec3>  lightPowers;
        float totalWeight = 0.0f;
        for (auto& a : scene.areaLightBuffer) {
            float area = glm::length(glm::cross(a.u, a.v));
            Vec3 power = a.radiance * (area * PI); // 出射度 M=πL, 总功率 P=M*A
            float w = luminance(power);
            lightWeights.push_back(w);
            lightPowers.push_back(power);
            totalWeight += w;
        }
        if (scene.areaLightBuffer.empty()) {
            getServer().logger.log("[PhotonMap] No area lights found; fallback to path tracing.");
            usePhotonMapRendering = false;
            return;
        }

        // 计算每个光源分配的光子数量
        vector<unsigned> photonsPerLight(scene.areaLightBuffer.size(), 0);
        unsigned allocated = 0;
        for (size_t i=0;i<scene.areaLightBuffer.size();++i) {
            unsigned ni = (totalWeight>0) ? static_cast<unsigned>(float(N) * (lightWeights[i]/totalWeight)) : (N/scene.areaLightBuffer.size());
            photonsPerLight[i] = ni;
            allocated += ni;
        }
        // 补齐因四舍五入差额
        while (allocated < N) {
            for (size_t i=0; i<photonsPerLight.size() && allocated<N; ++i) { photonsPerLight[i]++; allocated++; }
        }

        // 发射与传播
        std::mt19937 rng(12345);
        std::uniform_real_distribution<float> uni(0.0f, 1.0f);
        const int maxBounce = 20;
        const float eps = 1e-4f;

        // 能量统计：初次发射的总通量、每次交互的吸收通量、逃逸通量
        Vec3 sumInitialFlux{0.0f}; // 光子初次从光源发射的总能量
        Vec3 sumAbsorbedFlux{0.0f}; // 每次与材质交互时吸收的能量
        Vec3 sumEscapedFlux{0.0f}; //  逃逸出场景的能量

        for (size_t li=0; li<scene.areaLightBuffer.size(); ++li) {
            const auto& a = scene.areaLightBuffer[li];
            float area = glm::length(glm::cross(a.u, a.v));
            Vec3 power = a.radiance * (area * PI);
            unsigned count = photonsPerLight[li];
            // 每个光子的能量（RGB）
            Vec3 fluxPerPhoton = power / float(count);
            // 累计该光源的初次发射总能量（用于与 expected emit 对比）
            sumInitialFlux += fluxPerPhoton * float(count);

            Vec3 normal = glm::normalize(glm::cross(a.u, a.v));
            Onb onb{normal};

            for (unsigned i=0; i<count; ++i) {
                float rx = uni(rng);
                float ry = uni(rng);
                Vec3 origin = a.position + a.u * rx + a.v * ry;
                Vec3 localDir = defaultSamplerInstance<HemiSphere>().sample3d();
                Vec3 dir = glm::normalize(onb.local(localDir));
                Ray ray{origin + normal*eps, dir};
                Vec3 flux = fluxPerPhoton;

                for (int bounce=0; bounce<maxBounce; ++bounce) {
                    auto hit = closestHitObject(ray);
                    if (!hit) {
                        // 无几何命中，视为逃逸
                        sumEscapedFlux += flux;
                        break;
                    }

                    // 材质属性
                    auto& m = scene.materials[hit->material.index()];
                    auto albedoProp = m.getProperty<Property::Wrapper::RGBType>("diffuseColor");
                    Vec3 albedo = albedoProp ? (*albedoProp).value : Vec3{1,1,1};
                    auto reflProp = m.getProperty<Property::Wrapper::FloatType>("reflectivity");
                    auto transpProp = m.getProperty<Property::Wrapper::FloatType>("transparency");
                    auto iorProp = m.getProperty<Property::Wrapper::FloatType>("refractionIndex");
                    float reflectivity = reflProp ? (*reflProp).value : 0.0f;
                    float transparency = transpProp ? (*transpProp).value : 0.0f;
                    float ior = iorProp ? (*iorProp).value : 1.5f;

                    Vec3 n = hit->normal;
                    Vec3 inDir = glm::normalize(ray.direction);

                    bool isSpecular = (reflectivity > 1e-3f) || (transparency > 1e-3f);
                    if (!isSpecular) {
                        // 在漫反射表面存储光子（形成全局照明与焦散着落点）
                        photons.push_back(Photon{ hit->hitPoint, -ray.direction, flux });

                        // 吸收与俄罗斯轮盘（逐通道）
                        sumAbsorbedFlux += flux * (Vec3{1.0f,1.0f,1.0f} - albedo);
                        float p = std::max(0.0f, std::min(1.0f, std::max(albedo.x, std::max(albedo.y, albedo.z))));
                        if (uni(rng) > p) break;
                        flux = flux * (albedo / p); // 无偏缩放

                        Onb shOnb{ n };
                        Vec3 newLocal = defaultSamplerInstance<HemiSphere>().sample3d();
                        Vec3 newDir = glm::normalize(shOnb.local(newLocal));
                        ray = Ray{ hit->hitPoint + n*eps, newDir };
                    } else {
                        // 理想镜面/理想介质：沿反射或折射方向传播，不在镜面/玻璃上存储光子
                        // Fresnel（Schlick）概率选择反射/折射
                        float etai = 1.0f, etat = ior;
                        Vec3 nn = n;
                        float cosi = glm::clamp(glm::dot(-inDir, nn), -1.0f, 1.0f);
                        if (cosi < 0.0f) { cosi = -cosi; std::swap(etai, etat); nn = -nn; }
                        float F0 = ( (etai - etat) / (etai + etat) ); F0 = F0*F0;
                        float Fr = F0 + (1.0f - F0) * std::pow(1.0f - cosi, 5.0f);

                        // 若只设置了反射或折射，强制选择对应分支；同时考虑用户的 reflectivity/transparency 值
                        float pReflect;
                        if (reflectivity > 1e-3f && transparency > 1e-3f) {
                            // 结合 Fresnel 与用户权重（归一化）
                            float sumRT = reflectivity + transparency;
                            float userR = reflectivity / sumRT;
                            pReflect = std::min(1.0f, std::max(0.0f, 0.5f*Fr + 0.5f*userR));
                        } else if (reflectivity > 1e-3f) {
                            pReflect = 1.0f;
                        } else if (transparency > 1e-3f) {
                            pReflect = 0.0f;
                        } else {
                            pReflect = Fr; // 应不触发，但留兜底
                        }

                        float xi = uni(rng);
                        if (xi < pReflect) {
                            Vec3 reflDir = glm::reflect(inDir, n);
                            ray = Ray{ hit->hitPoint + n*eps, glm::normalize(reflDir) };
                            // 理想反射：假设能量不衰减（已由材质权重考虑），保持 flux
                        } else {
                            float eta = etai / etat;
                            float k = 1.0f - eta*eta*(1.0f - cosi*cosi);
                            if (k < 0.0f) {
                                // 全内反射
                                Vec3 reflDir = glm::reflect(inDir, nn);
                                ray = Ray{ hit->hitPoint + nn*eps, glm::normalize(reflDir) };
                            } else {
                                Vec3 refrDir = glm::normalize(eta*inDir + (eta*cosi - std::sqrt(k))*nn);
                                ray = Ray{ hit->hitPoint - nn*eps, refrDir };
                            }
                            // 简化：未实现介质内部吸收，保持 flux
                        }
                    }
                }
            }
        }

        photonMap = std::make_unique<PhotonMap>();
        photonMap->build(&photons);

        // 根据场景大小设置默认聚合半径（若场景参数未指定或为0）
        Vec3 bmin, bmax; computeSceneBounds(bmin, bmax);
        float diag = glm::length(bmax - bmin);
        if (gatherRadius <= 0.0f) {
            gatherRadius = std::max(1e-3f, diag * 0.02f);
        }

        // 能量守恒统计
        Vec3 expectedEmit{0};
        for (size_t i=0;i<scene.areaLightBuffer.size();++i) {
            float area = glm::length(glm::cross(scene.areaLightBuffer[i].u, scene.areaLightBuffer[i].v));
            expectedEmit += scene.areaLightBuffer[i].radiance * (area * PI);
        }
        Vec3 sumFlux{0};
        for (auto& ph : photons) sumFlux += ph.flux;
        // 核对项：发射守恒（expected vs initial），全局守恒（expected vs absorbed+escaped）
        Vec3 measuredGlobal = sumAbsorbedFlux + sumEscapedFlux;
        auto safeRatio = [](float num, float den){ return (den > 0.0f) ? (num / den) : 0.0f; };
        Vec3 relErrInitial{ safeRatio(sumInitialFlux.x, expectedEmit.x) ,
                             safeRatio(sumInitialFlux.y, expectedEmit.y) ,
                             safeRatio(sumInitialFlux.z, expectedEmit.z) };
        Vec3 relErrGlobal{ safeRatio(measuredGlobal.x, expectedEmit.x) ,
                           safeRatio(measuredGlobal.y, expectedEmit.y) ,
                           safeRatio(measuredGlobal.z, expectedEmit.z) };
        // 输出精简日志（仅保留必要的能量基线与误差）
        auto& logger = getServer().logger;
        logger.log("[PhotonMap] expected emit power = (" + std::to_string(expectedEmit.x) + ", " + std::to_string(expectedEmit.y) + ", " + std::to_string(expectedEmit.z) + ")");
        logger.log("[PhotonMap] emission / Initial = (" + std::to_string(relErrInitial.x) + ", " + std::to_string(relErrInitial.y) + ", " + std::to_string(relErrInitial.z) + ")");
        logger.log("[PhotonMap] Global / Initial = (" + std::to_string(relErrGlobal.x) + ", " + std::to_string(relErrGlobal.y) + ", " + std::to_string(relErrGlobal.z) + ")");
    }

    RGB SimplePathTracerRenderer::gatherRadianceAtHit(const Vec3& hitPoint, const Vec3& normal, int materialIndex) const {
        if (!photonMap) return Vec3{0};

        // 为避免自相交，将查询点沿法线微偏移
        const float eps = 1e-4f;
        Vec3 queryPoint = hitPoint + normal * eps;

        // K 近邻 + 圆锥加权 + 半空间过滤（参考 Jensen 基本光子映射）
        const int kNearest = 32;         // 目标光子数，兼顾稳定与细节
        const int maxIters = 6;          // 半径扩展最多次数

        // 初始半径：使用当前基础半径（可能为自动估算值）
        float baseRadius = std::max(1e-6f, gatherRadius);
        vector<const Photon*> bucket;
        photonMap->queryRadius(queryPoint, baseRadius, bucket);

        int it = 0;
        while ((int)bucket.size() < kNearest && it < maxIters) {
            baseRadius *= 1.5f;
            bucket.clear();
            photonMap->queryRadius(queryPoint, baseRadius, bucket);
            ++it;
        }

        if (bucket.empty()) {
            // 若仍无光子，返回极弱环境以避免纯黑斑点（与材质漫反射耦合）
            Vec3 albedoFallback = Vec3{1,1,1};
            const auto& mf = scene.materials[materialIndex];
            for (const auto& prop : mf.properties) {
                if (prop.key == "diffuseColor" && prop.type == Property::Type::RGB) {
                    const auto rgb = std::get<Property::Wrapper::RGBType>(prop.valueWrapper);
                    albedoFallback = rgb.value;
                    break;
                }
            }
            return scene.ambient.constant * (albedoFallback / PI) * 0.1f; // 10% 环境占比
        }

        // 使用 K 近邻的局部半径：取桶内最大距离作为 R
        float R = 0.0f;
        std::vector<float> dists; dists.reserve(bucket.size());
        for (auto* ph : bucket) {
            Vec3 d = ph->position - queryPoint;
            float dist = std::sqrt(glm::dot(d, d));
            dists.push_back(dist);
            if (dist > R) R = dist;
        }
        R = std::max(R, 1e-6f);

        // 圆锥加权 w = (1 - d/R)，并做半空间过滤：dot(n, incident) >= 0
        // Jensen 的圆锥滤波归一化：3/(π R^2)
        Vec3 weightedFlux{0};
        for (size_t i=0; i<bucket.size(); ++i) {
            const Photon* ph = bucket[i];
            float d = dists[i];
            float wCone = std::max(0.0f, 1.0f - d / R);
            float wHem  = std::max(0.0f, glm::dot(normal, ph->incident));
            weightedFlux += ph->flux * (wCone * wHem);
        }
        float norm = (3.0f / (PI * R * R));
        Vec3 irradiance = weightedFlux * norm;

        // 漫反射BRDF = ρ/π（避免在 const 方法中调用非 const 的 getProperty）
        Vec3 albedo = Vec3{1,1,1};
        const auto& m = scene.materials[materialIndex];
        for (const auto& prop : m.properties) {
            if (prop.key == "diffuseColor" && prop.type == Property::Type::RGB) {
                const auto rgb = std::get<Property::Wrapper::RGBType>(prop.valueWrapper);
                albedo = rgb.value;
                break;
            }
        }
        Vec3 radiance = irradiance * (albedo / PI);
        return radiance;
    }

    // --- photon map visualization as point image ---
    static void savePPM(const std::string& path, const std::vector<Vec3>& img, unsigned int w, unsigned int h) {
        std::ofstream out(path, std::ios::binary);
        if (!out) return;
        out << "P6\n" << w << " " << h << "\n255\n";
        for (unsigned int i=0;i<w*h;++i) {
            auto toByte = [](float c){
                float v = std::max(0.0f, std::min(1.0f, c));
                return static_cast<unsigned char>(v * 255.0f + 0.5f);
            };
            unsigned char r = toByte(img[i].x);
            unsigned char g = toByte(img[i].y);
            unsigned char b = toByte(img[i].z);
            out.write(reinterpret_cast<char*>(&r), 1);
            out.write(reinterpret_cast<char*>(&g), 1);
            out.write(reinterpret_cast<char*>(&b), 1);
        }
    }

    void SimplePathTracerRenderer::exportPhotonMapImagePoints(const std::string& filenameBase) const {
        if (!photonMap || photons.empty()) {
            getServer().logger.log("[PhotonMap] No photons to export.");
            return;
        }

        // Build camera basis (mirror of SimplePathTracer::Camera setup)
        const auto& c = scene.camera;
        Vec3 position = c.position;
        float vfov = c.fov;
        // Clamp like camera
        vfov = std::clamp(vfov, 20.0f, 160.0f);
        float theta = glm::radians(vfov);
        float halfHeight = std::tan(theta/2.0f);
        float halfWidth = c.aspect * halfHeight;
        Vec3 w = glm::normalize(c.position - c.lookAt); // camera back
        Vec3 u = glm::normalize(glm::cross(c.up, w));
        Vec3 v = glm::cross(w, u);
        float focusDis = c.focusDistance;
        Vec3 lowerLeft = position - halfWidth*focusDis*u - halfHeight*focusDis*v - focusDis*w;
        Vec3 horizontal = 2.0f*halfWidth*focusDis*u;
        Vec3 vertical   = 2.0f*halfHeight*focusDis*v;
        // Accumulation buffers
        std::vector<Vec3> accum(width*height, Vec3{0,0,0});
        std::vector<unsigned int> hits(width*height, 0);

        // Helper: index from (x,y)
        auto idx = [&](int x, int y){ return (height-1-y)*width + x; };

        // Visibility epsilon based on scene scale
        Vec3 bmin, bmax; computeSceneBounds(bmin, bmax);
        float diag = glm::length(bmax - bmin);
        float visEps = std::max(1e-4f, diag * 1e-4f);

        // Project photons to pixels and perform depth/visibility check
        for (const auto& ph : photons) {
            // Project to image plane
            Vec3 dirToPhoton = glm::normalize(ph.position - position);
            float denom = glm::dot(dirToPhoton, -w);
            if (denom <= 1e-6f) continue;
            float tplane = focusDis / denom;
            Vec3 hitPlane = position + dirToPhoton * tplane;
            Vec3 delta = hitPlane - lowerLeft;
            float s = glm::dot(delta, u) / glm::length(horizontal);
            float timg = glm::dot(delta, v) / glm::length(vertical);
            if (s < 0.0f || s > 1.0f || timg < 0.0f || timg > 1.0f) continue;

            int px = std::min<int>(width-1, std::max<int>(0, static_cast<int>(s * float(width))));
            int py = std::min<int>(height-1, std::max<int>(0, static_cast<int>(timg * float(height))));

            // Build the camera ray through this pixel (no lens offset)
            Vec3 pixelDir = glm::normalize(lowerLeft + s*horizontal + timg*vertical - position);
            Ray camRay{ position, pixelDir };
            auto rec = closestHitObject(camRay);
            if (!rec) continue;

            // Visibility: only show photons that match the pixel’s surface point
            float dist = glm::length(rec->hitPoint - ph.position);
            if (dist > visEps) continue; // occluded or on a different surface

            // Accumulate photon flux and count
            accum[idx(px, py)] += ph.flux;
            hits[idx(px, py)] += 1u;
        }

        // Compute log-average luminance for adaptive exposure
        const float epsLum = 1e-6f;
        auto luminance = [](const Vec3& c){ return 0.2126f*c.x + 0.7152f*c.y + 0.0722f*c.z; };
        double logSum = 0.0;
        for (const auto& cpx : accum) logSum += std::log(epsLum + luminance(cpx));
        double logAvg = std::exp(logSum / double(width*height));
        float key = 0.18f; // exposure key
        float exposure = (logAvg > 0.0) ? (key / float(logAvg)) : 1.0f;

        // Find max hit count for brightness boost
        unsigned int maxHits = 0;
        for (auto h : hits) maxHits = std::max(maxHits, h);
        float boostStrength = 0.3f; // contribution weight of hit count

        // Tone map (Reinhard), add hit-count brightness, gamma correct
        std::vector<Vec3> img(width*height, Vec3{0,0,0});
        for (unsigned int y=0; y<height; ++y) {
            for (unsigned int x=0; x<width; ++x) {
                size_t id = idx(x, y);
                Vec3 c = accum[id] * exposure;                 // scale by adaptive exposure
                c = c / (Vec3{1.0f,1.0f,1.0f} + c);             // Reinhard mapping per channel
                float hitBoost = (maxHits > 0) ? (boostStrength * (float(hits[id]) / float(maxHits))) : 0.0f;
                c += Vec3{hitBoost, hitBoost, hitBoost};        // add brightness from hit counts
                // gamma 2.2
                c.x = std::pow(std::max(0.0f, c.x), 1.0f/2.2f);
                c.y = std::pow(std::max(0.0f, c.y), 1.0f/2.2f);
                c.z = std::pow(std::max(0.0f, c.z), 1.0f/2.2f);
                // clamp
                c = Vec3{ std::min(1.0f, c.x), std::min(1.0f, c.y), std::min(1.0f, c.z) };
                img[id] = c;
            }
        }

        std::string path = filenameBase + "_points_depth_tonemap.ppm";
        savePPM(path, img, width, height);
        getServer().logger.log(std::string("[PhotonMap] Exported photon points image (depth+tone-mapped): ") + path);
    }
}
