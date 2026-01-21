#pragma once
#ifndef __DIELECTRIC_SHADER_HPP__
#define __DIELECTRIC_SHADER_HPP__

#include "Shader.hpp"
#include "samplers/SamplerInstance.hpp"

#include <algorithm>
#include <cmath>

namespace SimplePathTracer
{
    // 理想玻璃/介质材质
    class Dielectric : public Shader
    {
    private:
        float ior{1.5f};
        float reflectivity{0.0f};
        float transparency{1.0f};
        Vec3 transmissionColor{1.0f, 1.0f, 1.0f};
        Vec3 reflectColor{1.0f, 1.0f, 1.0f};

        static inline float clamp01(float v) {
            return std::clamp(v, 0.0f, 1.0f);
        }

        static inline float fresnelSchlick(float cosTheta, float etai, float etat) {
            cosTheta = std::clamp(cosTheta, 0.0f, 1.0f);
            float r0 = (etai - etat) / (etai + etat);
            r0 *= r0;
            float oneMinusCos = 1.0f - cosTheta;
            return r0 + (1.0f - r0) * oneMinusCos * oneMinusCos * oneMinusCos * oneMinusCos * oneMinusCos;
        }
    public:
        Dielectric(Material& material, vector<Texture>& textures)
            : Shader(material, textures) {
            auto diff = material.getProperty<Property::Wrapper::RGBType>("diffuseColor");
            if (diff) transmissionColor = (*diff).value;
            reflectColor = transmissionColor;
            auto spec = material.getProperty<Property::Wrapper::RGBType>("specularColor");
            if (spec) reflectColor = (*spec).value;

            auto refl = material.getProperty<Property::Wrapper::FloatType>("reflectivity");
            if (refl) reflectivity = std::max(0.0f, (*refl).value);
            auto transp = material.getProperty<Property::Wrapper::FloatType>("transparency");
            if (transp) transparency = std::max(0.0f, (*transp).value);
            auto refIdx = material.getProperty<Property::Wrapper::FloatType>("refractionIndex");
            if (refIdx) ior = std::max(1.01f, (*refIdx).value);
        }

        Scattered shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const override {
            const float eps = 1e-4f;
            const Vec3 geomNormal = normal;
            Vec3 inDir = glm::normalize(ray.direction);
            Vec3 n = geomNormal;
            float cosi = glm::clamp(glm::dot(-inDir, n), -1.0f, 1.0f);
            float etai = 1.0f;
            float etat = ior;
            if (cosi < 0.0f) {
                cosi = -cosi;
                std::swap(etai, etat);
                n = -n;
            }
            float eta = etai / etat;
            float sin2t = eta * eta * std::max(0.0f, 1.0f - cosi * cosi);
            bool totalInternalReflection = sin2t > 1.0f;
            float Fr = fresnelSchlick(cosi, etai, etat);
            float probReflect;
            if (totalInternalReflection) {
                probReflect = 1.0f;
            } else if (reflectivity > 1e-3f || transparency > 1e-3f) {
                if (reflectivity <= 1e-3f) {
                    probReflect = 0.0f;
                } else if (transparency <= 1e-3f) {
                    probReflect = 1.0f;
                } else {
                    float sum = reflectivity + transparency;
                    float userR = reflectivity / sum;
                    probReflect = clamp01(0.5f * Fr + 0.5f * userR);
                }
            } else {
                probReflect = Fr;
            }

            float xi = defaultSamplerInstance<UniformSampler>().sample1d();
            bool chooseReflect = totalInternalReflection || (xi < probReflect);
            float eventProb = chooseReflect ? std::max(1e-4f, probReflect) : std::max(1e-4f, 1.0f - probReflect);

            if (chooseReflect) {
                Vec3 reflDir = glm::normalize(glm::reflect(inDir, n));
                float cosTerm = std::max(1e-4f, std::abs(glm::dot(n, reflDir)));
                float weight = totalInternalReflection ? 1.0f : Fr;
                if (reflectivity > 1e-3f) weight *= reflectivity;
                Vec3 attenuation = reflectColor * (weight / (eventProb * cosTerm));
                return {
                    Ray{hitPoint + n * eps, reflDir},
                    attenuation,
                    Vec3{0},
                    1.0f
                };
            } else {
                float cost = std::sqrt(std::max(0.0f, 1.0f - sin2t));
                Vec3 refrDir = glm::normalize(eta * inDir + (eta * cosi - cost) * n);
                float cosTerm = std::max(1e-4f, std::abs(glm::dot(-n, refrDir)));
                float Ft = std::max(0.0f, 1.0f - Fr);
                Vec3 attenuation = transmissionColor;
                if (transparency > 1e-3f) attenuation *= transparency;
                attenuation *= (Ft / (eventProb * cosTerm));
                Vec3 offsetNormal = (glm::dot(refrDir, geomNormal) > 0.0f) ? geomNormal : -geomNormal;
                return {
                    Ray{hitPoint + offsetNormal * eps, refrDir},
                    attenuation,
                    Vec3{0},
                    1.0f
                };
            }
        }
    };
}

#endif
