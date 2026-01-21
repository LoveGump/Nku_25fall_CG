#pragma once
#ifndef __SIMPLE_PATH_TRACER_GLASS_HPP__
#define __SIMPLE_PATH_TRACER_GLASS_HPP__

#include "Shader.hpp"

namespace SimplePathTracer
{
    /**
     * 理想玻璃材质，包含反射与折射两条路径
     */
    class Glass : public Shader
    {
    private:
        Vec3 absorbed; // 进入材质后的能量保留
        float ior;     // 折射率
    public:
        Glass(Material &material, vector<Texture> &textures);
        Scattered shade(const Ray &ray, const Vec3 &hitPoint, const Vec3 &normal) const override;
    };
}

#endif
