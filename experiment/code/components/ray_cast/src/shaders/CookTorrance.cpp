#include "shaders/CookTorrance.hpp"

namespace RayCast
{
    static inline float saturate(float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
    static inline Vec3  saturate(const Vec3& v) { return Vec3{saturate(v.x), saturate(v.y), saturate(v.z)}; }

    static inline float D_GGX(float NoH, float a)
    {
        float a2 = a*a;
        float d = (NoH*NoH) * (a2 - 1.f) + 1.f;
        return a2 / (PI * d * d + 1e-7f);
    }

    static inline float V_SmithGGX(float NoV, float NoL, float a)
    {
        auto lambda = [&](float NoX){
            float a2 = a*a;
            float t = (1.f - NoX*NoX) / (NoX*NoX + 1e-7f);
            return 0.5f * (sqrt(1.f + a2 * t) - 1.f);
        };
        return 1.f / (1.f + lambda(NoV) + lambda(NoL) + 1e-7f);
    }

    static inline Vec3 F_Schlick(const Vec3& F0, float VoH)
    {
        return F0 + (Vec3{1,1,1} - F0) * powf(1.f - VoH, 5.f);
    }

    static inline Vec3 fresnelFromIor(float ior)
    {
        float f0 = (ior - 1.f) / (ior + 1.f);
        f0 *= f0;
        return Vec3{f0, f0, f0};
    }

    CookTorrance::CookTorrance(Material& material, vector<Texture>& textures)
        : Shader(material, textures)
    {
        using PW = Property::Wrapper;
        if (auto c = material.getProperty<PW::RGBType>("baseColor")) baseColor = c->value;
        else if (auto d = material.getProperty<PW::RGBType>("diffuseColor")) baseColor = d->value;
        if (auto m = material.getProperty<PW::FloatType>("metallic")) metallic = saturate(m->value);
        if (auto r = material.getProperty<PW::FloatType>("roughness")) roughness = saturate(r->value);
        if (auto n = material.getProperty<PW::FloatType>("ior")) ior = n->value;
        if (auto a = material.getProperty<PW::RGBType>("ambientColor")) ambientColor = a->value;
        roughness = glm::max(0.04f, roughness);
    }

    RGB CookTorrance::shade(const Vec3& in, const Vec3& out, const Vec3& normal) const
    {
        Vec3 N = glm::normalize(normal);
        Vec3 V = glm::normalize(in);   // view (to eye)
        Vec3 L = glm::normalize(out);  // light
        Vec3 H = glm::normalize(V + L);

        float NoV = saturate(glm::dot(N, V));
        float NoL = saturate(glm::dot(N, L));
        float NoH = saturate(glm::dot(N, H));
        float VoH = saturate(glm::dot(V, H));
        if (NoL <= 0.f || NoV <= 0.f) return Vec3{0};

        // Fresnel base reflectance
        Vec3 F0 = metallic > 0.5f ? baseColor : fresnelFromIor(ior);
        Vec3  F = F_Schlick(F0, VoH);

        float a = roughness * roughness; // perceptual to microfacet roughness
        float  D = D_GGX(NoH, a);
        float  G = V_SmithGGX(NoV, NoL, a);

        Vec3  spec = (F * (D * G)) / (4.f * NoL * NoV + 1e-7f);

        // Disney-like diffuse (Lambert scaled by (1 - metallic))
        Vec3  kd = (1.f - metallic) * (Vec3{1,1,1} - F);
        Vec3  diff = (baseColor / PI) * kd;

        Vec3 color = diff * NoL + spec * NoL + ambientColor;
        return saturate(color);
    }
}
