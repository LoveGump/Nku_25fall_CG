#pragma once
#ifndef __SHADER_CREATOR_HPP__
#define __SHADER_CREATOR_HPP__

#include "Shader.hpp"
#include "Lambertian.hpp"
#include "Conductor.hpp"
#include "Dielectric.hpp"

namespace SimplePathTracer
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
            case 1: // Conductor / 导体
                shader = make_shared<Conductor>(material, t);
                break;
            case 2: // Dielectric / 介质（折射）
                shader = make_shared<Dielectric>(material, t);
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