#pragma once
#ifndef __METAL_HPP__
#define __METAL_HPP__

#include "Shader.hpp"
#include <glm/glm.hpp>
#include <algorithm> // for min/max

namespace MyPathTracing
{

    class Metal : public Shader
    {
    private:
        Vec3 albedo; // 反射颜色 (如果是金子就是黄色，银子就是白色)
        float fuzz;  // 模糊度 (0.0 = 完美镜面, 1.0 = 非常粗糙)

    public:
        // 构造函数
        Metal(Material &material, std::vector<Texture> &textures);

        // 核心 Shading 函数
        Scattered shade(const Ray &ray, const Vec3 &hitPoint, const Vec3 &normal) const override;

        // 对于镜面材质，PDF 是 Delta 分布，无法评估，返回 0
        Vec3 eval(const Vec3 &wi, const Vec3 &wo, const Vec3 &normal) const override
        {
            return Vec3(0.0f);
        }

        float pdf(const Vec3 &wi, const Vec3 &wo, const Vec3 &normal) const
        {
            return 0.0f;
        }
    };
}

#endif