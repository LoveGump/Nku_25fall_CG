// 模型系统定义
// 定义了场景中的各种几何实体和模型结构
#pragma once
#ifndef __NR_MODEL_HPP__
#define __NR_MODEL_HPP__

#include <string>
#include <vector>

#include "geometry/vec.hpp"

#include "Material.hpp"
#include "common/macros.hpp"

namespace NRenderer
{
    using namespace std;

    // 实体基类
    // 所有几何实体的基类，包含材质信息
    struct Entity {
        Handle material;    // 材质句柄
    };
    SHARE(Entity);

    // 球体实体
    // 定义了一个球体的位置、方向和半径
    struct Sphere : public Entity
    {
        Vec3 direction = {0, 0, 1};  // 球体方向
        Vec3 position = {0, 0, 0};   // 球心位置
        float radius = { 0 };        // 球体半径
    };
    SHARE(Sphere);
    
    // 三角形实体
    // 定义了一个三角形的三个顶点和法线
    struct Triangle : public Entity
    {
        union {
            struct {
                Vec3 v1;    // 第一个顶点
                Vec3 v2;    // 第二个顶点
                Vec3 v3;    // 第三个顶点
            };
            Vec3 v[3];     // 顶点数组形式
        };
    // 每个顶点的法线（用于平滑着色）
    // 默认使用零向量表示“未提供”，以便插值逻辑能正确检测到缺失状态。
    Vec3 vertexNormals[3] = { };
        Vec3 normal;       // 三角形法线

        // 默认构造函数
        Triangle()
            : v1            ()
            , v2            ()
            , v3            ()
            , normal         (0, 0, 1)
        {}

    };
    SHARE(Triangle);

    // 平面实体
    // 定义了一个平面的位置、法线和两个基向量
    struct Plane : public Entity
    {
        Vec3 normal = {0, 0, 1};  // 平面法线
        Vec3 position = {};       // 平面上的一点
        Vec3 u = {};             // 平面第一个基向量
        Vec3 v = {};             // 平面第二个基向量
    };
    SHARE(Plane);

    // 网格实体
    // 定义了一个由多个三角形组成的网格
    struct Mesh : public Entity
    {
        vector<Vec3> normals;         // 法线列表
        // 顶点法线（每个顶点一个法线，用于平滑着色）
        vector<Vec3> vertexNormals;
        vector<Vec3> positions;       // 顶点位置列表
        vector<Vec2> uvs;             // UV坐标列表
        vector<Index> normalIndices;   // 法线索引
        vector<Index> positionIndices; // 顶点索引
        vector<Index> uvIndices;       // UV索引

        // 检查是否有法线数据
        bool hasNormal() const {
            return normals.size() != 0;
        }

        // 检查是否有顶点法线数据
        bool hasVertexNormal() const {
            return vertexNormals.size() != 0;
        }

        // 检查是否有UV坐标数据
        bool hasUv() const {
            return uvs.size() != 0;
        }

        // 按算术平均计算顶点法线并写入 `vertexNormals`。
        // 算法：对每个三角形计算单位面法线 n，然后把 n 简单累加到三个顶点的法线累加器，
        // 最后对每个顶点累加向量归一化得到顶点法线（即相邻面法线的算术平均）。
        // 该函数会覆盖之前的 `vertexNormals` 数据。
        void computeVertexNormalsAreaWeighted(float eps = 1e-8f) {
            // 初始化顶点法线累加器
            vertexNormals.clear();
            vertexNormals.resize(positions.size(), Vec3(0.0f));

            // 计算网格质心（用于统一面法线朝向）
            Vec3 meshCenter(0.0f);
            if (!positions.empty()) {
                for (const auto& p : positions) meshCenter += p;
                meshCenter /= float(positions.size());
            }

            // 每三个索引为一个三角形
            for (size_t t = 0; t + 2 < positionIndices.size(); t += 3) {
                Index i0 = positionIndices[t + 0];
                Index i1 = positionIndices[t + 1];
                Index i2 = positionIndices[t + 2];

                // 边界校验
                if (i0 >= positions.size() || i1 >= positions.size() || i2 >= positions.size()) continue;

                // 三角形的三个顶点
                Vec3 v0 = positions[i0];
                Vec3 v1 = positions[i1];
                Vec3 v2 = positions[i2];

                Vec3 e1 = v1 - v0;
                Vec3 e2 = v2 - v0;
                Vec3 f = glm::cross(e1, e2); // 未归一化的面法线
                float flen = glm::length(f);
                if (flen <= eps) continue; // 退化三角形，跳过

                // 计算单位面法线（优先使用导入器提供的面法线方向）
                Vec3 faceNormal;
                size_t triIndex = t / 3;
                if (normals.size() == positionIndices.size() / 3) {
                    Vec3 nn = normals[triIndex];
                    float nlen = glm::length(nn);
                    if (nlen > eps) faceNormal = nn / nlen;
                    else faceNormal = f / flen;
                } else {
                    faceNormal = f / flen; // 单位面法线
                }

                // 统一面法线朝向：面法线应指向面心到网格质心的外侧
                Vec3 faceCenter = (v0 + v1 + v2) / 3.0f;
                Vec3 toOutside = faceCenter - meshCenter; // 指向外部的向量（近似）
                if (glm::dot(faceNormal, toOutside) < 0.0f) {
                    faceNormal = -faceNormal; // 翻转法线
                }

                // 算术平均：直接把单位面法线累加到三个顶点的累加器（不按面积加权）
                vertexNormals[i0] += faceNormal;
                vertexNormals[i1] += faceNormal;
                vertexNormals[i2] += faceNormal;
            }

            // 归一化每个顶点的法线，若长度不足则设置默认法线
            for (size_t i = 0; i < vertexNormals.size(); ++i) {
                float len = glm::length(vertexNormals[i]);
                if (len > eps) vertexNormals[i] = vertexNormals[i] / len;
                else vertexNormals[i] = Vec3(0.0f, 0.0f, 1.0f);
            }
        }
    };
    SHARE(Mesh);

    // 场景节点
    // 表示场景图中的一个节点，可以是不同类型的几何实体
    struct Node
    {
        // 节点类型枚举
        enum class Type
        {
            SPHERE = 0x0,    // 球体
            TRIANGLE = 0X1,   // 三角形
            PLANE = 0X2,      // 平面
            MESH = 0X3        // 网格
        };
        Type type = Type::SPHERE;  // 节点类型
        Index entity;              // 实体索引
        Index model;               // 所属模型索引
    };
    SHARE(Node);

    // 模型结构
    // 定义了一个完整的3D模型，包含多个节点和变换信息
    struct Model {
        vector<Index> nodes;           // 节点列表
        Vec3 translation = {0, 0, 0};  // 平移向量
        Vec3 scale = {1, 1, 1};       // 缩放向量
    };
    SHARE(Model);
}

#endif