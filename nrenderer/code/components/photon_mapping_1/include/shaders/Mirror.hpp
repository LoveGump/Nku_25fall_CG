#pragma once
#ifndef __MIRROR_SHADER_HPP__
#define __MIRROR_SHADER_HPP__

#include "Shader.hpp"

#include <algorithm>
#include <cmath>

namespace SimplePathTracer
{
    // 理想镜面反射材质
    class Mirror : public Shader
    {
    private:
        Vec3 reflectColor{1.0f, 1.0f, 1.0f};
        float reflectivity{1.0f};
    public:
        Mirror(Material& material, vector<Texture>& textures)
            : Shader(material, textures) {
            auto specular = material.getProperty<Property::Wrapper::RGBType>("specularColor");
            if (specular) {
                reflectColor = (*specular).value;
            } else {
                auto diffuse = material.getProperty<Property::Wrapper::RGBType>("diffuseColor");
                if (diffuse) reflectColor = (*diffuse).value;
            }
            auto refl = material.getProperty<Property::Wrapper::FloatType>("reflectivity");
            if (refl) reflectivity = std::clamp((*refl).value, 0.0f, 1.0f);
        }

        Scattered shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const override {
            const float eps = 1e-4f;
            Vec3 inDir = glm::normalize(ray.direction);
            Vec3 n = normal;
            if (glm::dot(n, inDir) > 0.0f) {
                n = -n;
            }
            Vec3 dir = glm::normalize(glm::reflect(inDir, n));
            float cosTerm = std::max(1e-4f, std::abs(glm::dot(n, dir)));
            Vec3 attenuation = reflectColor * reflectivity / cosTerm;
            return {
                Ray{hitPoint + n * eps, dir},
                attenuation,
                Vec3{0},
                1.0f
            };
        }
    };
}

#endif
