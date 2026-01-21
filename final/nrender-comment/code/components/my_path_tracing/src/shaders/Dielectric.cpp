#include "shaders/Dielectric.hpp"
#include "samplers/SamplerInstance.hpp" // 用于获取随机数

namespace MyPathTracing
{

    // ==========================================
    // 辅助函数：折射计算 (斯涅尔定律向量版)
    // R_out = etai_over_etat * (R_in + cos_theta * N) - sqrt(1 - |R_out_perp|^2) * N
    // ==========================================
    Vec3 Dielectric::refract(const Vec3 &uv, const Vec3 &n, float etai_over_etat)
    {
        // uv 是入射光方向 (单位向量)
        // n  是法线 (单位向量，指向入射侧)

        // 1. 计算入射角余弦 (cos_theta)
        // fmin 是为了防止浮点误差导致值略微大于 1.0
        float cos_theta = fmin(glm::dot(-uv, n), 1.0f);

        // 2. 计算折射光线的垂直分量 (R_out_perp)
        Vec3 r_out_perp = etai_over_etat * (uv + cos_theta * n);

        // 3. 计算折射光线的平行分量 (R_out_parallel)
        // 长度平方 = dot(v, v)
        float r_out_perp_len_sq = glm::dot(r_out_perp, r_out_perp);
        Vec3 r_out_parallel = -std::sqrt(std::fabs(1.0f - r_out_perp_len_sq)) * n;

        // 4. 合成最终方向
        return r_out_perp + r_out_parallel;
    }

    // ==========================================
    // 辅助函数：菲涅尔效应 (Schlick 近似)
    // 随着角度变大，反射率会急剧增加（比如从侧面看玻璃）
    // ==========================================
    float Dielectric::reflectance(float cosine, float ref_idx)
    {
        auto r0 = (1.0f - ref_idx) / (1.0f + ref_idx);
        r0 = r0 * r0;
        return r0 + (1.0f - r0) * std::pow((1.0f - cosine), 5.0f);
    }

    // ==========================================
    // 核心 Shade 函数
    // ==========================================
    Scattered Dielectric::shade(const Ray &ray, const Vec3 &hitPoint, const Vec3 &normal) const
    {
        // 1. 归一化入射光线方向
        Vec3 unit_direction = glm::normalize(ray.direction);

        // 2. 判断光线是“进入”还是“射出”？
        // 这一点至关重要！如果反了，图像就不会倒转，或者变成凹透镜效果。

        float refraction_ratio; // 折射率之比 (η / η')
        Vec3 correct_normal;    // 指向入射侧的法线

        // glm::dot(ray, normal) > 0 表示光线和法线方向相同。
        // 因为几何法线默认向外，所以这意味着光线正在【从内部射向外部】。
        if (glm::dot(unit_direction, normal) > 0.0f)
        {
            // === 状态：出射 (Inside -> Outside) ===
            // 物理过程：从玻璃 (ir) 射入 空气 (1.0)
            // 斯涅尔公式要求：refraction_ratio = n_in / n_out = ir / 1.0
            refraction_ratio = ir;

            // 斯涅尔公式要求法线必须指向“入射光来源”的那一侧。
            // 既然我们在内部，原来的法线是指向外部的，所以必须取反！
            correct_normal = -normal;
        }
        else
        {
            // === 状态：入射 (Outside -> Inside) ===
            // 物理过程：从空气 (1.0) 射入 玻璃 (ir)
            // 斯涅尔公式要求：refraction_ratio = n_in / n_out = 1.0 / ir
            refraction_ratio = 1.0f / ir;

            // 我们在外部，法线本来就是指出来的，指向我们，所以不用动。
            correct_normal = normal;
        }

        // 3. 计算入射角的 cos 和 sin
        // 注意要用 correct_normal，确保夹角是锐角
        float cos_theta = fmin(glm::dot(-unit_direction, correct_normal), 1.0f);
        float sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);

        // 4. 判断是否发生“全内反射” (Total Internal Reflection)
        // 只有当从密介质射向疏介质 (ratio > 1) 时才可能发生。
        // 如果 sin_theta * ratio > 1.0，说明折射角无法计算（sin不能大于1），必须反射。
        bool cannot_refract = (refraction_ratio * sin_theta) > 1.0f;

        // 5. 决定光线路径：反射还是折射？
        Vec3 direction;

        // 获取一个 0~1 的随机数
        float random_float = defaultSamplerInstance<UniformSampler>().sample1d();

        // 计算菲涅尔反射概率
        float reflect_prob = reflectance(cos_theta, refraction_ratio);

        if (cannot_refract || reflect_prob > random_float)
        {
            // 情况 A: 必须反射 (全内反射) 或者 随机到了反射 (菲涅尔效应)
            direction = glm::reflect(unit_direction, correct_normal);
        }
        else
        {
            // 情况 B: 折射
            direction = refract(unit_direction, correct_normal, refraction_ratio);
        }

        // 6. 构造散射光线
        // 【重要】起点偏移：必须沿着新的方向稍微推一点点，防止光线刚生成就撞到自己。
        // 1e-4f 是一个经验值 (EPSILON)
        Ray scattered_ray(hitPoint + direction * 1e-4f, direction);

        // 7. 颜色衰减
        // 玻璃是完美传输，不吸光。设为纯白 (1.0, 1.0, 1.0)。
        // 注意：这里不需要除以 PDF，也不需要除以 cos_theta。
        // 因为我们是用“俄罗斯轮盘赌”选择的路径，能量已经在概率上守恒了。
        Vec3 attenuation = Vec3(1.0f, 1.0f, 1.0f);

        // 8. 返回结果
        // isSpecular = true 是关键！告诉积分器这是镜面路径。
        return {
            scattered_ray,
            attenuation,
            Vec3(0.0f), // 无自发光
            1.0f,       // PDF (占位符)
            true        // isSpecular: 标记为镜面材质
        };
    }
}