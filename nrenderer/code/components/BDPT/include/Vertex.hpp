#pragma once
#ifndef __VERTEX_HPP__
#define __VERTEX_HPP__

#include "Ray.hpp"
#include "intersections/HitRecord.hpp"
#include "scene/Scene.hpp"

namespace BDPT
{
using namespace NRenderer;

enum LightType
{
    DeltaDirection,
    Other,
};

// 表示路径上的一个顶点
struct Vertex
{
    Ray ray;
    HitRecord hitRecord; // 交点，用于计算反射光线
    Vec3 radiance;       // 辐射度
    float pdf;           // 采样概率
    float n_dot_in;      // 入射光线与法线的夹角
    Vec3 attenuation;    // 表面反射率
    Vec3 beta;           // 路径上的累积辐射度
    LightType lightType; // 光源类型

  public:
    Vertex(const Ray &ray, const HitRecord &hitRecord, const Vec3 &radiance, float pdf, float n_dot_in,
           const Vec3 &attenuation, const Vec3 &beta, LightType lightType = LightType::Other)
        : ray(ray), hitRecord(hitRecord), radiance(radiance), pdf(pdf), n_dot_in(n_dot_in), attenuation(attenuation),
          beta(beta), lightType(lightType)
    {
    }
    Vertex() = default;
    ~Vertex() = default;

    auto getRay() const -> Ray
    {
        return ray;
    }
    auto getHitRecord() const -> HitRecord
    {
        return hitRecord;
    }
    auto getRadiance() const -> Vec3
    {
        return radiance;
    }
    auto getPdf() const -> float
    {
        return pdf;
    }
    auto getN_dot_in() const -> float
    {
        return n_dot_in;
    }
    auto getAttenuation() const -> Vec3
    {
        return attenuation;
    }
    auto getBeta() const -> Vec3
    {
        return beta;
    }
    auto getLightType() const -> LightType
    {
        return lightType;
    }
    void setBeta(const Vec3 &beta)
    {
        this->beta = beta;
    }
    void setAttanuation(const Vec3 &attenuation)
    {
        this->attenuation = attenuation;
    }
    void setLightType(LightType lightType)
    {
        this->lightType = lightType;
    }

    bool isConnectible() const
    {
        return lightType == LightType::Other || lightType == LightType::DeltaDirection;
    }
};
} // namespace BDPT

#endif