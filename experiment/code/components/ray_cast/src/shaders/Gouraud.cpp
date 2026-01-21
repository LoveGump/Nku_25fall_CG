// Gouraud 着色器实现
#include "shaders/Gouraud.hpp"

namespace RayCast
{
    // 计算反射向量
    // normal: 表面法线
    // dir: 入射方向
    // 返回反射方向
    Vec3 reflectDir(const Vec3& normal, const Vec3& dir) {
        return dir - 2*glm::dot(dir, normal)*normal;
    }

    // 构造函数
    // material: 材质参数
    // textures: 纹理数组
    Gouraud::Gouraud(Material& material, vector<Texture>& textures)
        : Shader                (material, textures)
    {
        using PW = Property::Wrapper;
		 //获取环境光颜色
		 auto optAmbientColor = material.getProperty<PW::RGBType>("ambientColor");
         if (optAmbientColor) ambientColor = (*optAmbientColor).value;
         else ambientColor = {1, 1, 1};  // 默认为白色

        // 获取漫反射颜色
        auto optDiffuseColor = material.getProperty<PW::RGBType>("diffuseColor");
        if (optDiffuseColor) diffuseColor = (*optDiffuseColor).value;
        else diffuseColor = {1, 1, 1};  // 默认为白色
        
        // 获取镜面反射颜色
        auto optSpecularColor = material.getProperty<PW::RGBType>("specularColor");
        if (optSpecularColor) specularColor = (*optSpecularColor).value;
        else specularColor = {1, 1, 1};  // 默认为白色

        // 获取镜面反射指数
        auto optSpecularEx = material.getProperty<PW::FloatType>("specularEx");
        if (optSpecularEx) specularEx = (*optSpecularEx).value;
        else specularEx = 1;  // 默认为1
    }
    // 计算着色结果
    // in: 视线方向
    // out: 光源方向
    // normal: 表面法线
    // 返回着色计算的颜色值
    RGB Gouraud::shade(const Vec3& in, const Vec3& out, const Vec3& normal) const {
        // 与 Phong 一致的局部光照项，用于对比
        Vec3 n = glm::normalize(normal);// 法线
        Vec3 v = glm::normalize(in); // 观察方向
        Vec3 l = glm::normalize(out); // 光源方向
        Vec3 r = reflectDir(n, l); // 反射方向

        auto ambient = ambientColor;

        auto diffuse = diffuseColor * glm::max(0.f, glm::dot(n, l));
        
        auto spec = glm::max(0.f, glm::dot(v, r));
        auto specular = specularColor * (float)glm::pow(spec, specularEx);
        return ambient + diffuse + specular;
    }
}
