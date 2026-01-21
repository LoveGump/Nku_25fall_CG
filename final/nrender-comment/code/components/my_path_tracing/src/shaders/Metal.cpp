#include "shaders/Metal.hpp"
#include "samplers/SamplerInstance.hpp"

namespace MyPathTracing
{

    Metal::Metal(Material &material, std::vector<Texture> &textures)
        : Shader(material, textures)
    {
        // 1. 读取颜色
        // 我们复用 "diffuseColor" 属性作为金属的反射率 (Albedo)
        auto colorProp = material.getProperty<Property::Wrapper::RGBType>("diffuseColor");
        if (colorProp)
            albedo = colorProp->value;
        else
            albedo = {1.0f, 1.0f, 1.0f}; // 默认亮银色

        // 2. 读取模糊度 (Fuzz)
        // 假如你的材质文件里没有定义 fuzz，默认为 0 (完美镜面)
        auto fuzzProp = material.getProperty<Property::Wrapper::FloatType>("fuzz");
        if (fuzzProp)
        {
            // 限制 fuzz 在 0~1 之间
            fuzz = std::min(fuzzProp->value, 1.0f);
        }
        else
        {
            fuzz = 0.0f;
        }
    }

    Scattered Metal::shade(const Ray &ray, const Vec3 &hitPoint, const Vec3 &normal) const
    {
        // 1. 计算理想反射方向
        // 公式：v - 2*dot(v,n)*n
        Vec3 reflected = glm::reflect(glm::normalize(ray.direction), normal);

        // 2. 处理磨砂效果 (Fuzz)
        // 如果 fuzz > 0，我们在理想反射方向的顶端加一个随机球体扰动
        if (fuzz > 1e-6f)
        {
            // 这里使用单位球内的随机采样 (假设你有 UniformInUnitSphere，如果没有可以用 HemiSphere 凑合)
            Vec3 fuzzVec = defaultSamplerInstance<HemiSphere>().sample3d();
            reflected = glm::normalize(reflected + fuzz * fuzzVec);
        }

        // 3. 构造散射光线
        // 同样要做微小的起点偏移 (epsilon)，防止自我遮挡
        Ray scattered_ray(hitPoint + reflected * 1e-4f, reflected);

        // 4. 边界检查
        // 只有当反射光线依然指向表面外侧 (dot > 0) 时才有效。
        // 如果 fuzz 太大，光线可能会被扰动到物体内部去，那种光线应该被吸收掉。
        if (glm::dot(scattered_ray.direction, normal) > 0)
        {
            return {
                scattered_ray,
                albedo,     // 能量衰减 (颜色)
                Vec3(0.0f), // 无自发光
                1.0f,       // PDF (占位符)
                true        // 【关键】isSpecular = true! 开启 NEE 直连特权
            };
        }
        else
        {
            // 光线射进内部了，吸收 (返回空/黑)
            return {Ray(), Vec3(0.0f), Vec3(0.0f), 0.0f, true};
        }
    }
}