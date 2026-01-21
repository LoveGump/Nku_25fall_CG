#pragma once
#ifndef __NR_SCENE_HPP__
#define __NR_SCENE_HPP__

#include "Texture.hpp"
#include "Material.hpp"
#include "Model.hpp"
#include "Light.hpp"
#include "Camera.hpp"

namespace NRenderer
{
    // 渲染选项结构体
    // 定义了渲染时的各种参数设置
    struct RenderOption
    {
        // 宽度是指渲染图像的水平像素数量
        // 高度是指渲染图像的垂直像素数量
        unsigned int width; // 渲染宽度
        unsigned int height; // 渲染高度
        unsigned int depth;// 最大递归深度
        unsigned int samplesPerPixel; // 每像素采样数
        RenderOption()
            : width             (500)
            , height            (500)
            , depth             (4)
            , samplesPerPixel   (16)
        {}
    };

    // 环境光结构体
    // 定义了场景中的环境光属性
    struct Ambient
    {
        enum class Type
        {
            CONSTANT, ENVIROMENT_MAP
        };
        Type type;  // 环境光类型
        Vec3 constant = {}; // 常量环境光颜色
        Handle environmentMap = {}; // 环境光贴图句柄
    };

    struct Scene
    {
        Camera camera; // 场景相机

        RenderOption renderOption; // 渲染选项

        Ambient ambient;// 环境光

        // buffers
        vector<Material> materials; // 材质列表
        vector<Texture> textures; // 纹理列表

        vector<Model> models; // 模型列表
        vector<Node> nodes; // 节点列表
        // object buffer
        vector<Sphere> sphereBuffer; // 球体缓冲区
        vector<Triangle> triangleBuffer; // 三角形缓冲区
        vector<Plane> planeBuffer; // 平面缓冲区
        vector<Mesh> meshBuffer; // 网格缓冲区

        vector<Light> lights; // 光源列表
        // light buffer
        vector<PointLight> pointLightBuffer; // 点光源缓冲区
        vector<AreaLight> areaLightBuffer; // 面光源缓冲区
        vector<DirectionalLight> directionalLightBuffer; // 方向光源缓冲区
        vector<SpotLight> spotLightBuffer; // 聚光灯缓冲区
    };
    using SharedScene = shared_ptr<Scene>;
} // namespace NRenderer


#endif