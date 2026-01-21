// Phong材质着色器实现
// 实现了Phong光照模型的着色计算
#include "shaders/Phong.hpp"

namespace RayCast_PhongShading
{
    // 计算反射向量
    // normal: 表面法线
    // dir: 入射方向
    // 返回反射方向
    Vec3 reflect(const Vec3& normal, const Vec3& dir) {
        return dir - 2 * glm::dot(dir, normal) * normal;
    }

    // 构造函数
    // material: 材质参数
    // textures: 纹理数组
    Phong::Phong(Material& material, vector<Texture>& textures)
        : Shader(material, textures)
    {
        using PW = Property::Wrapper;
        //获取环境光颜色
        auto optAmbientColor = material.getProperty<PW::RGBType>("ambientColor");
        if (optAmbientColor) ambientColor = (*optAmbientColor).value;
        else ambientColor = { 1, 1, 1 };  // 默认为白色

        // 获取漫反射颜色
        auto optDiffuseColor = material.getProperty<PW::RGBType>("diffuseColor");
        if (optDiffuseColor) diffuseColor = (*optDiffuseColor).value;
        else diffuseColor = { 1, 1, 1 };  // 默认为白色

        // 获取镜面反射颜色
        auto optSpecularColor = material.getProperty<PW::RGBType>("specularColor");
        if (optSpecularColor) specularColor = (*optSpecularColor).value;
        else specularColor = { 1, 1, 1 };  // 默认为白色

        // 获取镜面反射指数
        auto optSpecularEx = material.getProperty<PW::FloatType>("specularEx");
        if (optSpecularEx) specularEx = (*optSpecularEx).value;
        else specularEx = 1;  // 默认为1
    }

    // 计算着色结果
    // in: 视线方向（从相机指向交点，已归一化）
    // out: 光源方向（从交点指向光源，已归一化）
    // normal: 表面法线（已归一化）
    // 返回着色计算的颜色值
    RGB Phong::shade(const Vec3& in, const Vec3& out, const Vec3& normal) const {
        Vec3 v = glm::normalize(in);  // 视线方向（确保归一化）
        // 计算光线的反射方向：R = 2(N·L)N - L
        Vec3 r = 2.0f * glm::dot(normal, out) * normal - out;
        r = glm::normalize(r);  // 归一化反射方向
        
        // 计算环境光项：Ka * Ia
        auto ambient = ambientColor;

        // 计算漫反射项：Kd * Id * max(0, N·L)
        float diffuseFactor = glm::max(0.0f, glm::dot(out, normal));
        auto diffuse = diffuseColor * diffuseFactor;
        
        // 计算镜面反射项：Ks * Is * max(0, V·R)^n
        float specularFactor = glm::max(0.0f, glm::dot(v, r));
        auto specular = specularColor * glm::pow(specularFactor, specularEx);
        
        // 返回环境光、漫反射和镜面反射的叠加
        return ambient + diffuse + specular;
    }
}