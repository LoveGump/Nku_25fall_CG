#include "shaders/CookTorrance.hpp"
#include "samplers/SamplerInstance.hpp"
#include "Onb.hpp"
#include <cmath>
#include <algorithm>

#ifndef PI
#define PI 3.14159265359f
#endif

namespace MyPathTracing
{
    CookTorrance::CookTorrance(Material &material, std::vector<Texture> &textures)
        : Shader(material, textures)
    {
        // 读取参数
        auto colorProp = material.getProperty<Property::Wrapper::RGBType>("diffuseColor");
        albedo = colorProp ? colorProp->value : Vec3(1.0f);

        auto roughProp = material.getProperty<Property::Wrapper::FloatType>("roughness");
        roughness = roughProp ? std::clamp(roughProp->value, 0.001f, 1.0f) : 0.001f;

        auto transProp = material.getProperty<Property::Wrapper::FloatType>("transmission");
        transmission = transProp ? std::clamp(transProp->value, 0.0f, 1.0f) : 0.0f;

        auto iorProp = material.getProperty<Property::Wrapper::FloatType>("ior");
        ior = iorProp ? iorProp->value : 1.5f;
    }

    // --- 通用 PBR 函数 (GGX & Smith) ---
    // 这两个几何函数是通用的，金属玻璃都得用
    float CookTorrance::DistributionGGX(const Vec3 &N, const Vec3 &H, float roughness) const
    {
        float a = roughness * roughness;
        float a2 = a * a;
        float NdotH = std::max(glm::dot(N, H), 0.0f);
        float NdotH2 = NdotH * NdotH;
        float denom = (NdotH2 * (a2 - 1.0f) + 1.0f);
        return a2 / (PI * denom * denom + 1e-7f);
    }
    float CookTorrance::GeometrySchlickGGX(float NdotV, float roughness) const
    {
        // 对于路径追踪(间接光)，k = a^2 / 2 是更准确的经验值
        float r = roughness;
        float k = (r * r) / 2.0f;
        return NdotV / (NdotV * (1.0f - k) + k);
    }
    float CookTorrance::GeometrySmith(const Vec3 &N, const Vec3 &V, const Vec3 &L, float roughness) const
    {
        return GeometrySchlickGGX(std::abs(glm::dot(N, V)), roughness) * GeometrySchlickGGX(std::abs(glm::dot(N, L)), roughness);
    }

    // 金属专用菲涅尔 (F0 = 颜色)
    Vec3 fresnelSchlickConductor(float cosTheta, const Vec3 &F0)
    {
        return F0 + (Vec3(1.0f) - F0) * std::pow(std::clamp(1.0f - cosTheta, 0.0f, 1.0f), 5.0f);
    }

    // 玻璃专用菲涅尔 (F0 由 IOR 计算)
    float fresnelDielectricSchlick(float cosTheta, float ior)
    {
        float R0 = (1.0f - ior) / (1.0f + ior);
        R0 = R0 * R0;
        return R0 + (1.0f - R0) * std::pow(1.0f - cosTheta, 5.0f);
    }

    // ==========================================
    // Shade: 逻辑彻底分离版
    // ==========================================
    Scattered CookTorrance::shade(const Ray &ray, const Vec3 &hitPoint, const Vec3 &normal) const
    {
        Vec3 V = glm::normalize(-ray.direction);
        Vec3 N = glm::normalize(normal);

        // 几何准备
        float NdotV_raw = glm::dot(N, V);
        bool front_facing = (NdotV_raw > 0.0f);
        Vec3 face_N = front_facing ? N : -N;

        // 生成微表面法线 H (不管是金属还是玻璃，表面都是粗糙的，所以都需要 H)
        Vec2 xi = {defaultSamplerInstance<UniformSampler>().sample1d(), defaultSamplerInstance<UniformSampler>().sample1d()};
        float a = roughness * roughness;
        float phi = 2.0f * PI * xi.x;
        float cosTheta = std::sqrt((1.0f - xi.y) / (1.0f + (a * a - 1.0f) * xi.y));
        float sinTheta = std::sqrt(1.0f - cosTheta * cosTheta);
        Vec3 H_local = {sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta};
        Onb onb(face_N);
        Vec3 H = glm::normalize(onb.local(H_local));
        float VdotH = std::max(glm::dot(V, H), 0.0f);

        // ========================================================
        // 分支一：我是玻璃 (Transmission > 0)
        // ========================================================
        if (transmission > 0.0f)
        {
            float eta = front_facing ? (1.0f / ior) : ior;
            float F = 0.0f;

            // 计算折射角 (Snell's Law)
            float sin2ThetaV = 1.0f - VdotH * VdotH;
            float sin2ThetaT = eta * eta * sin2ThetaV;

            // 【关键修正 1：黑圈修复】
            if (sin2ThetaT >= 1.0f)
            {
                F = 1.0f; // 全内反射
            }
            else
            {
                // 如果在玻璃内部（eta > 1.0），要用“折射角的余弦”来查 Schlick
                // 这样当接近全反射角时，cosThetaT 趋向 0，F 就会平滑趋向 1.0
                float cosThetaT = std::sqrt(1.0f - sin2ThetaT);
                float cosForSchlick = front_facing ? VdotH : cosThetaT;
                F = fresnelDielectricSchlick(cosForSchlick, ior);
            }

            bool do_reflect = (defaultSamplerInstance<UniformSampler>().sample1d() < F);

            if (do_reflect)
            {
                // [反射]
                Vec3 L = glm::normalize(2.0f * glm::dot(V, H) * H - V);
                if (glm::dot(face_N, L) <= 0.0f)
                    return {Ray(), Vec3(0), Vec3(0), 0.0f, false};

                float G = GeometrySmith(face_N, V, L, roughness);
                float NdotH = std::max(glm::dot(face_N, H), 0.0f);
                float NdotV = std::max(glm::dot(face_N, V), 0.0f);

                // 玻璃反射权重
                float weight_val = G * VdotH / (NdotV * NdotH + 1e-7f);
                weight_val = std::min(weight_val, 10.0f);

                return {Ray(hitPoint + L * 1e-4f, L), Vec3(weight_val), Vec3(0), 1.0f, true};
            }
            else
            {
                // [折射]
                // 这里的 k 计算其实和上面的 sin2ThetaT 是一回事，但为了稳健再算一次
                float HdotV = glm::dot(H, V);
                float k = 1.0f - eta * eta * (1.0f - HdotV * HdotV);
                if (k < 0.0f)
                    return {Ray(), Vec3(0), Vec3(0), 0.0f, false};

                Vec3 L = glm::normalize((eta * HdotV - std::sqrt(k)) * -H - eta * V);

                float G = GeometrySmith(face_N, V, L, roughness); // GeometrySmith 内部已加 abs
                float NdotH = std::abs(glm::dot(face_N, H));
                float NdotV_abs = std::abs(glm::dot(face_N, V));

                // 【关键修正 2：暗玻璃修复】
                // 1. 去掉你代码里那个错误的 4.0f 和 HdotL
                // 2. 加上 1/eta^2 的辐射度缩放
                // 公式：Weight = Color * G * (V.H) / ((N.V) * (N.H)) * (1/eta^2)
                float weight_val = G * VdotH / (NdotV_abs * NdotH + 1e-7f);

                // 辐射度缩放：让光线进入玻璃时变亮，抵消能量密度的变化
                float radiance_scaling = 1.0f / (eta * eta);
                weight_val *= radiance_scaling;

                weight_val = std::min(weight_val, 10.0f);

                return {Ray(hitPoint + L * 1e-4f, L), albedo * weight_val, Vec3(0), 1.0f, true};
            }
        }

        // ========================================================
        // 分支二：我是金属 (Transmission == 0)
        // ========================================================
        else
        {
            // 1. 金属绝对没有折射，只有反射
            Vec3 L = glm::normalize(2.0f * glm::dot(V, H) * H - V);
            if (glm::dot(face_N, L) <= 0.0f)
                return {Ray(), Vec3(0), Vec3(0), 0.0f, false};

            // 2. 金属的菲涅尔 (用 Albedo 算)
            // 金属的 F0 就是它的本色 (albedo)
            Vec3 F_metal = fresnelSchlickConductor(VdotH, albedo);

            float G = GeometrySmith(face_N, V, L, roughness);
            float NdotH = std::max(glm::dot(face_N, H), 0.0f);
            float NdotV = std::max(glm::dot(face_N, V), 0.0f);

            // 3. 计算最终权重
            // Weight = F * G * (V.H) / ((N.V) * (N.H))
            // 注意：这里 F_metal 已经包含了颜色，所以不需要再乘 albedo
            Vec3 weight = F_metal * G * VdotH / (NdotV * NdotH + 1e-7f);

            // 4. 返回
            // 强制 isSpecular = true (为了保证高光锐利，防止被 NEE 抹平)
            return {Ray(hitPoint + L * 1e-4f, L), weight, Vec3(0), 1.0f, true};
        }
    }

    // ==========================================
    // Eval: 彻底禁用 NEE
    // ==========================================
    // 既然我们决定让 CookTorrance 走高质量的 BSDF 采样，
    // 这里就必须返回 0，防止 MyPathTracing 的 NEE 逻辑插手搞乱结果。
    Vec3 CookTorrance::eval(const Vec3 &wi, const Vec3 &wo, const Vec3 &normal) const
    {
        return Vec3(0.0f);
    }

    float CookTorrance::pdf(const Vec3 &wi, const Vec3 &wo, const Vec3 &normal) const
    {
        return 0.0f;
    }
}