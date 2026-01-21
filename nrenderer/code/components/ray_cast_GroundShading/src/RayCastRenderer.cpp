// 光线投射渲染器实现
// 实现了基本的光线投射渲染算法，包括阴影计算
#include "RayCastRenderer.hpp"
#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"

namespace RayCast_GroundShading
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
        auto width = scene.renderOption.width;
        auto height = scene.renderOption.height;
        // 创建像素缓冲区
        auto pixels = new RGBA[width*height];

        // 执行顶点变换
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
                // 生成光线
                auto ray = camera.shoot(float(j)/float(width), float(i)/float(height));
                // 追踪光线获取颜色
                auto color = trace(ray);
                // 颜色后处理
                color = clamp(color);
                color = gamma(color);
                pixels[(height-i-1)*width+j] = {color, 1};
            }
        }

        return {pixels, width, height};
    }
    
    // 光线追踪
    // r: 输入光线
    // 返回该光线对应的颜色
    RGB RayCastRenderer::trace(const Ray& r) {
        // 如果场景中没有点光源，返回黑色
        if (scene.pointLightBuffer.size() < 1) return {0, 0, 0};
        // 获取第一个点光源
        auto& l = scene.pointLightBuffer[0];
        // 计算最近的相交物体
        auto closestHitObj = closestHit(r);
        
        if (closestHitObj) {
            auto& hitRec = *closestHitObj;
            // 计算光源方向（用于阴影检测）
            auto out = glm::normalize(l.position - hitRec.hitPoint);
            // 统一背面剔除（与原逻辑一致）
            if (glm::dot(out, hitRec.normal) < 0) {
                return {0, 0, 0};
            }
            // 计算到光源的距离
            auto distance = glm::length(l.position - hitRec.hitPoint);
            // 生成阴影射线
            auto shadowRay = Ray{hitRec.hitPoint, out};
            auto shadowHit = closestHit(shadowRay);
    
            // Gouraud 着色路径（仅三角形命中）
            RGB c;
            float bu = hitRec.bary.x, bv = hitRec.bary.y, bw = hitRec.bary.z;
            bool isTriangle = (bu >= 0.f && bv >= 0.f && bw >= 0.f) && (bu <= 1.f && bv <= 1.f && bw <= 1.f) && ((bu + bv + bw) > 0.999f);
            if (isTriangle) {
                const float eps = 1e-8f;
                Vec3 n0 = hitRec.vtxNormals[0];
                Vec3 n1 = hitRec.vtxNormals[1];
                Vec3 n2 = hitRec.vtxNormals[2];
                bool hasVN = (glm::length(n0) > eps) && (glm::length(n1) > eps) && (glm::length(n2) > eps);
                if (!hasVN) { n0 = hitRec.normal; n1 = hitRec.normal; n2 = hitRec.normal; }
    
                // 顶点处视线与光线方向
                Vec3 in0 = glm::normalize(scene.camera.position - hitRec.vtx[0]);
                Vec3 in1 = glm::normalize(scene.camera.position - hitRec.vtx[1]);
                Vec3 in2 = glm::normalize(scene.camera.position - hitRec.vtx[2]);
                Vec3 out0 = glm::normalize(l.position - hitRec.vtx[0]);
                Vec3 out1 = glm::normalize(l.position - hitRec.vtx[1]);
                Vec3 out2 = glm::normalize(l.position - hitRec.vtx[2]);
    
                // 顶点颜色
                auto c0 = shaderPrograms[hitRec.material.index()]->shade(in0, out0, glm::normalize(n0));
                auto c1 = shaderPrograms[hitRec.material.index()]->shade(in1, out1, glm::normalize(n1));
                auto c2 = shaderPrograms[hitRec.material.index()]->shade(in2, out2, glm::normalize(n2));
                c = bu * c0 + bv * c1 + bw * c2;
            }
            else {
                c = shaderPrograms[hitRec.material.index()]->shade(-r.direction, out, hitRec.normal);
            }
    
            // 阴影判断与返回颜色
            if ((!shadowHit) || (shadowHit && shadowHit->t > distance)) {
                return c * l.intensity;
            }
            else {
                return Vec3{0};
            }
        }
        else {
            return {0, 0, 0};
        }
    }

    // 计算最近的相交物体
    // r: 输入光线
    // 返回最近的相交记录
    HitRecord RayCastRenderer::closestHit(const Ray& r) {
        HitRecord closestHit = nullopt;
        float closest = FLOAT_INF;
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
        // 检查与网格的相交
        for (auto& m : scene.meshBuffer) {
            auto hitRecord = Intersection::xMesh(r, m, 0.01, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }  
        return closestHit; 
    }
}