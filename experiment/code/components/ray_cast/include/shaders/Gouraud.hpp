#pragma once
#ifndef __GOURAUD_HPP__
#define __GOURAUD_HPP__

// Gouraud 着色器头文件
// 注意：在当前光线投射流水线中没有逐三角形的顶点颜色插值阶段，
// 这里实现的 Gouraud 仅复用 Phong 光照项（环境/漫反射/高光）
// 的公式接口，便于在材质/渲染设置中进行模式对比与切换。

#include "Shader.hpp"

namespace RayCast
{
    // Gouraud 着色器类
    // 接口与 Phong 一致，便于在工厂中按材质类型切换
    class Gouraud : public Shader
    {
    private:
        Vec3 ambientColor {1,1,1};
        Vec3 diffuseColor {1,1,1};
        Vec3 specularColor{1,1,1};
        float specularEx {1.0f};

    public:
        // 构造函数
        // material: 材质参数
        // textures: 纹理数组
        Gouraud(Material& material, vector<Texture>& textures);

        // 计算着色结果
        // in: 入射方向
        // out: 出射方向
        // normal: 表面法线
        // 返回着色计算的颜色值
        virtual RGB shade(const Vec3& in, const Vec3& out, const Vec3& normal) const override;
    };
}

#endif
