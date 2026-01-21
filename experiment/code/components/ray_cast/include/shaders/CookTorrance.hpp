#pragma once
#ifndef __COOK_TORRANCE_HPP__
#define __COOK_TORRANCE_HPP__

// Cook-Torrance 微表面模型着色器
// 支持属性：
//  - baseColor / diffuseColor (RGB)
//  - metallic (Float, [0,1])
//  - roughness (Float, [0,1])
//  - ior (Float, 默认 1.5)
//  - ambientColor (RGB, 可选)

#include "Shader.hpp"

namespace RayCast
{
    class CookTorrance : public Shader
    {
    private:
        Vec3 baseColor {1,1,1};
        float metallic {0.0f};
        float roughness {0.5f};
        float ior {1.5f};
        Vec3 ambientColor {0,0,0};

    public:
        CookTorrance(Material& material, vector<Texture>& textures);
        virtual RGB shade(const Vec3& in, const Vec3& out, const Vec3& normal) const override;
    };
}

#endif
