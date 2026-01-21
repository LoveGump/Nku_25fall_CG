#pragma once
#ifndef __LAMBERTIAN_HPP__
#define __LAMBERTIAN_HPP__

#include "Shader.hpp"

namespace BDPT
{
    class Lambertian : public Shader
    {
    private:
        Vec3 albedo;
    public:
        Lambertian(Material& material, vector<Texture>& textures);
        Scattered shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const;
        Vec3 evaluate(const Vec3& wo, const Vec3& wi, const Vec3& normal) const;
    };
}

#endif