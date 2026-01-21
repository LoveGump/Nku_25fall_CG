#include "shaders/Glass.hpp"
#include "samplers/SamplerInstance.hpp"

#include <cmath>

namespace SimplePathTracer
{
    /**
     * 构造函数
     * @param material 材质对象
     * @param textures 纹理缓冲区
     */
    Glass::Glass(Material &material, vector<Texture> &textures)
        : Shader(material, textures)
    {
        if (auto prop = material.getProperty<Property::Wrapper::FloatType>("ior"))
        {
            ior = (*prop).value;
        }
        else
        {
            ior = 1.5f;
        }
        if (auto absorbedProp = material.getProperty<Property::Wrapper::RGBType>("absorbed"))
        {
            absorbed = (*absorbedProp).value;
        }
        else
        {
            absorbed = Vec3{1.0f};
        }
    }

    /**
     * 计算光线与材质的交互结果
     * @param ray 入射光线
     * @param hitPoint 相交点
     * @param normal 法向量
     */
    Scattered Glass::shade(const Ray &ray, const Vec3 &hitPoint, const Vec3 &normal) const
    {
        Vec3 origin = hitPoint;
        Vec3 n = glm::normalize(normal);
        float eta = ior;
        Vec3 in = glm::normalize(ray.direction);
        // 入射方向与法线同向则说明从内部射出，需要反转法线并取倒数折射率
        if (glm::dot(in, n) > 0.f)
        {
            n = -n;
            eta = 1.f / eta;
        }

        Vec3 reflexDir = glm::normalize(in + n); // 根据几何关系构造反射方向
        Vec3 direction = reflexDir;
        float pdf = 1.f;

        Vec3 R0 = Vec3(std::pow((eta - 1.f) / (eta + 1.f), 2.f));
        float cosTheta = std::abs(glm::dot(in, n));
        Vec3 reflex = (R0 + (Vec3{1.f} - R0) * std::pow(1.f - cosTheta, 5.f)) * absorbed;
        Vec3 refraction = absorbed - reflex;

        Vec3 x = glm::normalize(reflexDir + in);
        Vec3 y = glm::normalize(-n);
        float sin_r = std::sqrt(std::max(0.f, 1.f - cosTheta * cosTheta)) / eta;
        float cos_r = std::sqrt(std::max(0.f, 1.f - sin_r * sin_r));

        Vec3 refractionDir = Vec3{0.f};
        if (sin_r <= 1.f)
        {
            refractionDir = glm::normalize(x * sin_r + y * cos_r);
        }
        else
        {
            // 全反射
            reflex = absorbed;
            refraction = Vec3{0.f};
        }

        return {
            Ray{origin, direction},
            reflex,
            Vec3{0},
            pdf,
            Ray{origin, refractionDir},
            refraction};
    }
}
