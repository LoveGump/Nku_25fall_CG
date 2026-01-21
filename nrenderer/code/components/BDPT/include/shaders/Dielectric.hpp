#pragma once
#ifndef __DIELECTRIC_HPP__
#define __DIELECTRIC_HPP__

#include "Shader.hpp"

namespace BDPT
{
    class Dielectric : public Shader
    {
    private:
        Vec3 albedo;
    public:
        Dielectric(Material& material, vector<Texture>& textures);
        Scattered shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const;
        Vec3 evaluate(const Vec3& wo, const Vec3& wi, const Vec3& normal) const;
        bool refract(const Vec3& v, const Vec3& n, float ni_over_nt, Vec3& refracted) const;
        float schlick(float cosine, float refractiveIndex) const;
    };
}

#endif