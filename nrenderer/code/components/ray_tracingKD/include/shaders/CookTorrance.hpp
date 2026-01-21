#pragma once
#ifndef __COOK_TORRANCE_HPP__
#define __COOK_TORRANCE_HPP__

// Cook-Torrance 着色器头文件
// 实现微表面（Cook-Torrance）反射模型

#include "Shader.hpp"

namespace Ray_KD_tracing
{
    // Cook-Torrance 着色器类
    class CookTorrance : public Shader
    {
    private:
        Vec3 albedo;     // 漫反射基色（非金属时使用）
        Vec3 F0;         // 法向入射菲涅尔反射率（导体为颜色，绝缘体为标量转Vec3）
        float roughness; // 粗糙度 [0,1]

        // 计算 Schlick 近似的菲涅尔项
        static Vec3 fresnelSchlick(float cosTheta, const Vec3 &F0)
        {
            // F = F0 + (1 - F0) * (1 - cosTheta)^5
            return F0 + (Vec3{1.f, 1.f, 1.f} - F0) * glm::pow(1.f - cosTheta, 5.f);
        }

        // GGX/Trowbridge-Reitz 法线分布函数
        static float distributionGGX(float NoH, float alpha)
        {
            // D = a^2 / (pi * ((NoH^2) * (a^2 - 1) + 1)^2)
            float a2 = alpha * alpha;
            float denom = (NoH * NoH) * (a2 - 1.f) + 1.f;
            return (a2) / (PI * denom * denom + 1e-7f);
        }

        // Schlick-GGX 近似几何遮蔽-阴影项（Smith）
        static float geometrySchlickGGX(float NoX, float k)
        {
            // G1 = NoX / (NoX * (1 - k) + k)
            return NoX / (NoX * (1.f - k) + k + 1e-7f);
        }

        static float geometrySmith(float NoV, float NoL, float alpha)
        {
            // k = (a + 1)^2 / 8
            float k = (alpha + 1.f);
            k = (k * k) / 8.f;
            float Gv = geometrySchlickGGX(NoV, k);
            float Gl = geometrySchlickGGX(NoL, k);
            return Gv * Gl;
        }

    public:
        // 构造函数：从材质属性中读取 F0、roughness、diffuseColor
        CookTorrance(Material &material, vector<Texture> &textures);

        // 着色：返回 BRDF * cosTheta 的值（与点光强度相乘在渲染器中完成）
        virtual RGB shade(const Vec3 &in, const Vec3 &out, const Vec3 &normal) const;
    };
}

#endif