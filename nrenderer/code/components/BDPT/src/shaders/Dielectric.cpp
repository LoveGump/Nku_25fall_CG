#include "shaders/Dielectric.hpp"
#include "samplers/SamplerInstance.hpp"

#include "Onb.hpp"

namespace BDPT
{
Dielectric::Dielectric(Material &material, vector<Texture> &textures) : Shader(material, textures)
{
    auto absorbed = material.getProperty<Property::Wrapper::RGBType>("absorbed");
    if (absorbed)
        albedo = Vec3{1} - (*absorbed).value;
    else
        albedo = {1, 1, 1};
}
Scattered Dielectric::shade(const Ray &ray, const Vec3 &hitPoint, const Vec3 &normal) const
{

    Vec3 origin = hitPoint;
    Vec3 direction;

    Vec3 reflected = glm::reflect(ray.direction, normal);
    Vec3 refracted;

    // 菲涅尔项
    float ni_over_nt;
    auto reflectiveIndex = material.getProperty<Property::Wrapper::FloatType>("ior");
    if (reflectiveIndex)
        ni_over_nt = (*reflectiveIndex).value;
    else
        ni_over_nt = 1.5; // 默认折射率

    Vec3 outward_normal;
    float cosine;
    float reflect_prob;

    if (glm::dot(ray.direction, normal) > 0)
    {
        // 入射角大于0，说明在物体内部
        outward_normal = -normal;
        cosine = glm::dot(ray.direction, normal) / glm::length(ray.direction);
    }
    else
    {
        // 入射角小于0，说明在物体外部
        outward_normal = normal;
        ni_over_nt = 1.0 / ni_over_nt;
        cosine = -glm::dot(ray.direction, normal) / glm::length(ray.direction);
    }

    if (refract(ray.direction, outward_normal, ni_over_nt, refracted))
    {
        reflect_prob = schlick(cosine, ni_over_nt);
    }
    else
    {
        reflect_prob = 1.0;
    }

    float pdf;
    auto sampler = defaultSamplerInstance<UniformSampler>();
    if (sampler.sample1d() < reflect_prob)
    {
        direction = reflected;
    }
    else
    {
        direction = refracted;
    }
    pdf = 1;

    auto attenuation = albedo;

    return {Ray{origin, direction}, attenuation, Vec3{0}, pdf};
}

Vec3 Dielectric::evaluate(const Vec3 &wo, const Vec3 &wi, const Vec3 &normal) const
{
    float cosTheta = glm::dot(normal, wi);
    if (cosTheta > 0)
    {
        return albedo / PI;
    }
    else
    {
        return Vec3{0};
    }
}
bool Dielectric::refract(const Vec3 &v, const Vec3 &n, float ni_over_nt, Vec3 &refracted) const
{
    Vec3 uv = glm::normalize(v);
    float dt = glm::dot(uv, n);
    float discriminant = 1.0 - ni_over_nt * ni_over_nt * (1 - dt * dt);
    // 折射率大于1，且判别式大于0，说明有折射
    if (discriminant > 0)
    {
        refracted = ni_over_nt * (uv - n * dt) - n * sqrt(discriminant);
        return true;
    }
    return false;
}

float Dielectric::schlick(float cosine, float refractiveIndex) const
{
    float r0 = (1 - refractiveIndex) / (1 + refractiveIndex);
    r0 = r0 * r0;
    return r0 + (1 - r0) * pow((1 - cosine), 5);
}
} // namespace BDPT
