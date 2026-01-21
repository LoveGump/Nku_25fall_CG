#pragma once
#ifndef __SHADER_CREATOR_HPP__
#define __SHADER_CREATOR_HPP__

// 着色器创建器头文件
// 定义了用于创建各种着色器的工厂类

#include "Shader.hpp"
#include "Lambertian.hpp"
#include "Phong.hpp"
#include "CookTorrance.hpp"

namespace RayCastKD
{
    // 着色器创建器类
    // 根据材质类型创建相应的着色器
    class ShaderCreator
    {
    public:
        // 默认构造函数
        ShaderCreator() = default;

        // 创建着色器
        // material: 材质参数
        // t: 纹理数组
        // 返回创建的着色器指针
        SharedShader create(Material &material, vector<Texture> &t)
        {
            SharedShader shader{nullptr};
            // 根据材质类型选择着色器
            switch (material.type)
            {
            case 0: // Lambert材质
                shader = make_shared<Lambertian>(material, t);
                break;
            case 1: // Phong材质
                shader = make_shared<Phong>(material, t);
                break;
            case 2: // Cook-Torrance材质
                shader = make_shared<CookTorrance>(material, t);
                break;
            case 3: // Cook-Torrance材质（导体）
                shader = make_shared<CookTorrance>(material, t);
                break;
            default: // 默认使用Lambert材质
                shader = make_shared<Lambertian>(material, t);
                break;
            }
            return shader;
        }
    };
}

#endif