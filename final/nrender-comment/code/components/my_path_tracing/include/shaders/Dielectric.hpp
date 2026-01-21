#pragma once
#ifndef __DIELECTRIC_HPP__
#define __DIELECTRIC_HPP__

#include "Shader.hpp"
#include <glm/glm.hpp>

namespace MyPathTracing
{

    class Dielectric : public Shader
    {
    private:
        float ir; // 折射率 (Index of Refraction), 例如玻璃通常是 1.5

    public:
        // 构造函数：尝试从材质属性中读取 ior，默认为 1.5
        Dielectric(Material &material, std::vector<Texture> &textures)
            : Shader(material, textures)
        {
            auto prop = material.getProperty<Property::Wrapper::FloatType>("ior");
            if (prop)
                ir = prop->value;
            else
                ir = 1.5f;
        }

        // 核心渲染函数：决定光线是反射还是折射
        Scattered shade(const Ray &ray, const Vec3 &hitPoint, const Vec3 &normal) const override;

        // 对于完美镜面/折射材质，PDF 是 Delta 分布（无穷大），NEE 无法采样
        // 必须返回 0，告诉积分器不要试图对它进行直接光照采样
        Vec3 eval(const Vec3 &wi, const Vec3 &wo, const Vec3 &normal) const override
        {
            return Vec3(0.0f);
        }

        // 同样，PDF 也应该返回 0 (由 Delta 决定，不由概率密度函数决定)
        float pdf(const Vec3 &wi, const Vec3 &wo, const Vec3 &normal) const
        {
            return 0.0f;
        }

    private:
        // 辅助函数：计算反射率 (Schlick's Approximation)
        static float reflectance(float cosine, float ref_idx);

        // 辅助函数：计算折射方向 (Snell's Law)
        static Vec3 refract(const Vec3 &uv, const Vec3 &n, float etai_over_etat);
    };
}

#endif