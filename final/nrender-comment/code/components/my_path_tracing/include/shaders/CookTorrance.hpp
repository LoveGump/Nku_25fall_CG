#pragma once

#include "Shader.hpp"
#include "glm/glm.hpp"

namespace MyPathTracing
{
    class CookTorrance : public Shader
    {
    public:
        // 构造函数
        CookTorrance(Material &material, std::vector<Texture> &textures);

        // 核心接口 (Override)
        virtual Scattered shade(const Ray &ray, const Vec3 &hitPoint, const Vec3 &normal) const override;
        virtual Vec3 eval(const Vec3 &wi, const Vec3 &wo, const Vec3 &normal) const override;
        virtual float pdf(const Vec3 &wi, const Vec3 &wo, const Vec3 &normal) const;

    private:
        // --- 材质属性 ---
        Vec3 albedo;        // 颜色 (金属的F0 / 玻璃的透射色)
        float roughness;    // 粗糙度
        float transmission; // 透射率 (0=金属, 1=玻璃)
        float ior;          // 折射率 (1.5=玻璃)

        // --- PBR 几何函数 (GGX & Smith) ---
        // 这些是通用的，金属和玻璃都会用到
        float DistributionGGX(const Vec3 &N, const Vec3 &H, float roughness) const;
        float GeometrySchlickGGX(float NdotV, float roughness) const;
        float GeometrySmith(const Vec3 &N, const Vec3 &V, const Vec3 &L, float roughness) const;

        // 注意：fresnelSchlickConductor 和 fresnelDielectric 在 cpp 里作为独立函数实现，
        // 不需要在这里声明为成员函数，保持头文件清爽。
    };
}