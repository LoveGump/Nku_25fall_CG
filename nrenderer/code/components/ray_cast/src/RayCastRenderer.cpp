// 光线投射渲染器实现
// 实现了基本的光线投射渲染算法，包括阴影计算
#include "RayCastRenderer.hpp"
#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"

namespace RayCast
{
    // 释放渲染结果
    // r: 渲染结果
    void RayCastRenderer::release(const RenderResult& r) {
        auto [p, w, h] = r;
        delete[] p;
    }

    // 伽马校正
    // rgb: 输入的RGB颜色
    // 返回校正后的颜色
    RGB RayCastRenderer::gamma(const RGB& rgb) {
        return glm::sqrt(rgb);
    }

    // 渲染场景
    // 返回渲染结果
    auto RayCastRenderer::render() -> RenderResult {
        // 渲染结果的宽度（横向像素数量）和高度（纵向像素数量）
        auto width = scene.renderOption.width;
        auto height = scene.renderOption.height;
        // 创建像素缓冲区，记录渲染结果
        auto pixels = new RGBA[width*height];

        // 执行顶点变换，将一个场景中的所有模型从局部坐标转换到世界坐标
        VertexTransformer vertexTransformer{};
        vertexTransformer.exec(spScene);

        // 创建着色器程序
        ShaderCreator shaderCreator{};
        for (auto& mtl : scene.materials) {
            shaderPrograms.push_back(shaderCreator.create(mtl, scene.textures));
        }

        // 对每个像素进行光线追踪
        for (int i=0; i<height; i++) {
            for (int j=0; j < width; j++) {
                // 生成光线，方向为相机指向视平面上对应像素的位置
                auto ray = camera.shoot(float(j)/float(width), float(i)/float(height));
                // 追踪光线获取颜色
                auto color = trace(ray);
                // 颜色后处理
                color = clamp(color); // 颜色夹紧到 [0,1]
                color = gamma(color); // 伽马校正   
                pixels[(height-i-1)*width+j] = {color, 1}; // 存储像素颜色，注意y轴翻转
            }
        }

        return {pixels, width, height};
    }
    
    // 光线追踪
    // r: 输入光线
    // 返回该光线对应的颜色
    // 这里的实现仅支持点光源和阴影计算
    RGB RayCastRenderer::trace(const Ray& r) {
        // 如果场景中没有点光源，返回黑色
        if (scene.pointLightBuffer.size() < 1) return {0, 0, 0};
        // 获取第一个点光源
        auto& l = scene.pointLightBuffer[0];
        // 计算与输入光线最近的相交物体
        auto closestHitObj = closestHit(r);
        
        if (closestHitObj) {
            // 如果有相交物体，获取相交记录
            auto& hitRec = *closestHitObj;
            // 计算光源方向，方向为从相交点指向光源
            auto out = glm::normalize(l.position - hitRec.hitPoint);
            // 检查光源是否在表面背面
            if (glm::dot(out, hitRec.normal) < 0) {
                return {0, 0, 0};
            }
            // 计算到光源的距离
            auto distance = glm::length(l.position - hitRec.hitPoint);
            // 生成阴影射线
            auto shadowRay = Ray{hitRec.hitPoint, out}; // 从相交点指向光源的射线
            auto shadowHit = closestHit(shadowRay); // 计算阴影射线的最近相交物体
            // 计算着色结果
            auto c = shaderPrograms[hitRec.material.index()]->shade(-r.direction, out, hitRec.normal);
            // 如果没有遮挡或遮挡物在光源后面，返回着色结果
            if ((!shadowHit) || (shadowHit && shadowHit->t > distance)) {
                return c * l.intensity;
            }
            // 否则在阴影中，返回黑色
            else {
                return Vec3{0};
            }
        }
        else {
            return {0, 0, 0};
        }
    }

    // 计算与输入光线最近的相交物体
    // r: 输入光线
    // 返回最近的相交记录
    HitRecord RayCastRenderer::closestHit(const Ray& r) {
        HitRecord closestHit = nullopt; // 最近的相交记录，初始为空
        float closest = FLOAT_INF;  // 最近的相交距离，初始为无穷大
        // 检查与球体的相交
        for (auto& s : scene.sphereBuffer) {
            auto hitRecord = Intersection::xSphere(r, s, 0.01, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        // 检查与三角形的相交
        for (auto& t : scene.triangleBuffer) {
            auto hitRecord = Intersection::xTriangle(r, t, 0.01, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        // 检查与平面的相交
        for (auto& p : scene.planeBuffer) {
            auto hitRecord = Intersection::xPlane(r, p, 0.01, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        return closestHit; 
    }
}