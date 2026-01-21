// Cook-Torrance 着色器实现
// 实现基于物理的渲染（PBR）Cook-Torrance 微表面 BRDF 模型
#include "shaders/CookTorrance.hpp"

namespace RayCast_CookTorrance
{
    // 构造函数：从材质属性中读取 PBR 参数
    CookTorrance::CookTorrance(Material& material, vector<Texture>& textures)
        : Shader(material, textures)
    {
        using PW = Property::Wrapper;
        
        // 读取 F0（菲涅尔反射率）
        // 优先级：F0 > reflect > ior 计算 > 默认值
        auto optF0 = material.getProperty<PW::RGBType>("F0");
        if (optF0) {
            F0 = (*optF0).value;
        }
        else {
            // 尝试读取 reflect 属性（兼容旧场景文件）
            auto optReflect = material.getProperty<PW::RGBType>("reflect");
            if (optReflect) {
                F0 = (*optReflect).value;
            }
            else {
                // 绝缘体：F0 = ((ior-1)/(ior+1))^2
                auto optIor = material.getProperty<PW::FloatType>("ior");
                if (optIor) {
                    float ior = (*optIor).value;
                    float f0s = glm::pow((ior - 1.f) / (ior + 1.f), 2.f);
                    F0 = Vec3{f0s, f0s, f0s};
                }
                else {
                    F0 = Vec3{0.04f, 0.04f, 0.04f}; // 默认值（塑料/电介质）
                }
            }
        }

        // 读取粗糙度
        auto optRoughness = material.getProperty<PW::FloatType>("roughness");
        roughness = optRoughness ? (*optRoughness).value : 0.5f;
        // 限制范围避免数值问题
        roughness = glm::clamp(roughness, 0.02f, 0.99f);

        // 读取漫反射颜色（非金属适用）
        auto optDiffuseColor = material.getProperty<PW::RGBType>("diffuseColor");
        if (optDiffuseColor) {
            albedo = (*optDiffuseColor).value;
        }
        else {
            // 如果没有 diffuseColor，对于金属材质设为黑色
            albedo = Vec3{0.f, 0.f, 0.f};
        }
    }

    RGB CookTorrance::shade(const Vec3& in, const Vec3& out, const Vec3& normal) const {
        // 确保所有向量归一化
        Vec3 n = glm::normalize(normal);  // 法线方向
        Vec3 v = glm::normalize(in);      // 视线方向（从交点指向相机）
        Vec3 l = glm::normalize(out);     // 光源方向（从交点指向光源）

        // 计算关键的点积
        float NoV = glm::max(glm::dot(n, v), 0.0f);
        float NoL = glm::max(glm::dot(n, l), 0.0f);
        
        // 如果视线或光源在表面下方，不产生反射
        if (NoV <= 1e-7f || NoL <= 1e-7f) {
            return Vec3{0.f, 0.f, 0.f};
        }

        // 计算半程向量（视线和光源方向的中间方向）
        Vec3 h = glm::normalize(v + l);
        float NoH = glm::max(glm::dot(n, h), 0.0f);
        float VoH = glm::max(glm::dot(v, h), 0.0f);

        // 计算 Cook-Torrance 模型的三个关键项
        float alpha = roughness * roughness;  // 将粗糙度映射到 alpha
        float D = distributionGGX(NoH, alpha);           // 法线分布函数
        float G = geometrySmith(NoV, NoL, alpha);        // 几何遮蔽项
        Vec3  F = fresnelSchlick(VoH, F0);               // 菲涅尔项

        // Cook-Torrance 镜面反射 BRDF: f_spec = (D * F * G) / (4 * NoV * NoL)
        Vec3 numerator = D * G * F;
        float denominator = 4.0f * NoV * NoL;
        Vec3 specular = numerator / glm::max(denominator, 1e-7f);

        // 判断材质类型：通过 albedo 的亮度判断是否为金属
        // 如果 albedo 很暗（接近黑色），认为是纯金属
        float albedoLuminance = (albedo.r + albedo.g + albedo.b) / 3.0f;
        bool isMetal = (albedoLuminance < 0.01f);
        
        Vec3 result;
        if (isMetal) {
            // 纯金属工作流：没有漫反射，只有镜面反射
            // 金属的颜色完全由 F0 决定，通过菲涅尔项调制
            result = specular * NoL;
        }
        else {
            // 非金属工作流：有漫反射和镜面反射
            // 能量守恒：kS = F（镜面反射比例），kD = (1 - kS)（漫反射比例）
            Vec3 kS = F;
            Vec3 kD = Vec3{1.0f, 1.0f, 1.0f} - kS;
            
            // Lambert 漫反射 BRDF
            Vec3 diffuse = kD * albedo / PI;
            
            // 最终结果 = (漫反射 + 镜面反射) * NoL
            result = (diffuse + specular) * NoL;
        }
        
        return result;
    }
}