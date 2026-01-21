#pragma once
#ifndef __SHADER_CREATOR_HPP__
#define __SHADER_CREATOR_HPP__

#include "Shader.hpp"
#include "Lambertian.hpp"
#include "Dielectric.hpp"
#include "Metal.hpp"
#include "CookTorrance.hpp"

namespace MyPathTracing
{
    /**
     * 着色器创建器
     * 根据材质类型创建相应的着色器实例
     */
    class ShaderCreator
    {
    public:
        ShaderCreator() = default;

        /**
         * 根据材质类型创建着色器
         * @param material 材质对象
         * @param t 纹理缓冲区
         * @return 着色器共享指针
         */
        SharedShader create(Material &material, vector<Texture> &t)
        {
            SharedShader shader{nullptr};
            switch (material.type)
            {
            case 0: // Lambertian材质
                shader = make_shared<Lambertian>(material, t);
                break;
            case 2: // Dielectric材质
                shader = make_shared<Dielectric>(material, t);
                break;
            case 3: // Metal材质
                shader = make_shared<Metal>(material, t);
                break;
            case 4: // CookTorrance材质
                shader = make_shared<CookTorrance>(material, t);
                break;
            default: // 默认使用Lambertian
                shader = make_shared<Lambertian>(material, t);
                break;
            }
            return shader;
        }
    };
}

#endif
