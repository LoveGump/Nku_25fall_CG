#pragma once
#ifndef __DIELECTRIC_HPP__
#define __DIELECTRIC_HPP__

#include "Shader.hpp"

namespace SimplePathTracer
{
    // 介质材质着色器类
    class Dielectric : public Shader
    {
    private:
        Vec3 absorbed; // 吸收系数
        float ior;     // 折射率

    public:
        Dielectric(Material &material, vector<Texture> &textures);
        Scattered shade(const Ray &ray, const Vec3 &hitPoint, const Vec3 &normal) const;
    };

}

#endif
