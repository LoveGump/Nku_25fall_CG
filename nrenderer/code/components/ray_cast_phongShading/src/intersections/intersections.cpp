// 相交测试函数实现
// 实现了各种几何体与光线的相交测试算法
#include "intersections/intersections.hpp"

namespace RayCast_PhongShading::Intersection
{
    // 光线与三角形的相交?ller–Trumbore算法计算光线与三角形的相交
    // ray: 光线
    // t: 三角形
    // tMin, tMax: 相交距离范围
        HitRecord xTriangle(const Ray& ray, const Triangle& t, float tMin, float tMax) {
        const auto& v1 = t.v1;
        const auto& v2 = t.v2;
        const auto& v3 = t.v3;
    
        // 面法线（备用）
        Vec3 faceNormal = glm::normalize(t.normal);
    
        // 计算三角形的两个边向量
        auto e1 = v2 - v1;
        auto e2 = v3 - v1;
        // 计算叉积P
        auto P = glm::cross(ray.direction, e2);
        float det = glm::dot(e1, P);
        // 处理背面剔除
        Vec3 T;
        if (det > 0) T = ray.origin - v1;
        else { T = v1 - ray.origin; det = -det; }
        // 判断是否平行
        if (det < 0.000001f) return getMissRecord();
        // 计算重心坐标的分子项（尚未除以 det）
        float u = glm::dot(T, P);
        if (u > det || u < 0.f) return getMissRecord();
        Vec3 Q = glm::cross(T, e1);
        float v = glm::dot(ray.direction, Q);
        if (v < 0.f || v + u > det) return getMissRecord();
    
        // 命中距离 t
        float tHit = glm::dot(e2, Q);
        float invDet = 1.f / det;
        tHit *= invDet;
        // 检查相交距离是否在有效范围内
        if (tHit >= tMax || tHit <= tMin) return getMissRecord();
    
        // 计算归一化后的重心权重（用于法线插值）
        // 根据 M?ller–Trumbore 算法：u 对应 v2，v 对应 v3，(1-u-v) 对应 v1
        float bu = u * invDet;  // v2 的重心坐标
        float bv = v * invDet;  // v3 的重心坐标
        float bw = 1.0f - bu - bv;  // v1 的重心坐标
    
        // 顶点法线插值（若可用），否则退回面法线
        const float eps = 1e-8f;
        Vec3 n1 = t.vertexNormals[0];  // v1 的法线
        Vec3 n2 = t.vertexNormals[1];  // v2 的法线
        Vec3 n3 = t.vertexNormals[2];  // v3 的法线
        bool hasVtxNormals = (glm::length(n1) > eps) && (glm::length(n2) > eps) && (glm::length(n3) > eps);
        Vec3 shadingNormal = faceNormal;
        if (hasVtxNormals) {
            // 正确的重心插值：bw 对应 v1(n1)，bu 对应 v2(n2)，bv 对应 v3(n3)
            Vec3 interp = bw * n1 + bu * n2 + bv * n3;
            float nlen = glm::length(interp);
            if (nlen > eps) shadingNormal = interp / nlen;
        }
    
        return getHitRecord(tHit, ray.at(tHit), shadingNormal, t.material);
    }


    // 光线与球体的相交测试
    // 使用解析几何方法计算光线与球体的相交
    // ray: 光线
    // s: 球体
    // tMin, tMax: 相交距离范围
    HitRecord xSphere(const Ray& ray, const Sphere& s, float tMin, float tMax) {
        const auto& position = s.position;
        const auto& r = s.radius;
        // 计算二次方程系数
        Vec3 oc = ray.origin - position;
        float a = glm::dot(ray.direction, ray.direction);
        float b = glm::dot(oc, ray.direction);
        float c = glm::dot(oc, oc) - r*r;
        // 计算判别式
        float discriminant = b*b - a*c;
        float sqrtDiscriminant = sqrt(discriminant);
        // 如果有实根
        if (discriminant > 0) {
            // 尝试较近的交点
            float temp = (-b - sqrtDiscriminant) / a;
            if (temp < tMax && temp > tMin) {
                auto hitPoint = ray.at(temp);
                auto normal = (hitPoint - position)/r;
                return getHitRecord(temp, hitPoint, normal, s.material);
            }
            // 尝试较远的交点
            temp = (-b + sqrtDiscriminant) / a;
            if (temp < tMax && temp > tMin) {
                auto hitPoint = ray.at(temp);
                auto normal = (hitPoint - position)/r;
                return getHitRecord(temp, hitPoint, normal, s.material);
            }
        }
        return getMissRecord();
    }

    // 光线与平面的相交测试
    // ray: 光线
    // p: 平面
    // tMin, tMax: 相交距离范围
    HitRecord xPlane(const Ray& ray, const Plane& p, float tMin, float tMax) {
        // 计算平面法线
        Vec3 normal = glm::normalize(p.normal);
        // 计算光线方向与平面法线的点积
        auto Np_dot_d = glm::dot(ray.direction, normal);
        // 判断是否平行
        if (Np_dot_d < 0.0000001f && Np_dot_d > -0.00000001f) return getMissRecord();
        // 计算相交距离
        float dp = -glm::dot(p.position, normal);
        float t = (-dp - glm::dot(normal, ray.origin))/Np_dot_d;
        // 检查相交距离是否在有效范围内
        if (t >= tMax || t <= tMin) return getMissRecord();
        // 计算相交点
        Vec3 hitPoint = ray.at(t);
        // 检查相交点是否在平面范围内
        Mat3x3 d{p.u, p.v, glm::cross(p.u, p.v)};
        d = glm::inverse(d);
        auto res  = d * (hitPoint - p.position);
        auto u = res.x, v = res.y;
        if ((u<=1 && u>=0) && (v<=1 && v>=0)) {
            return getHitRecord(t, hitPoint, normal, p.material);
        }
        return getMissRecord();
    }

    // 光线与面光源的相交测试
    // ray: 光线
    // a: 面光源
    // tMin, tMax: 相交距离范围
    HitRecord xAreaLight(const Ray& ray, const AreaLight& a, float tMin, float tMax) {
        // 计算光源平面的法线
        Vec3 normal = glm::cross(a.u, a.v);
        Vec3 position = a.position;
        // 计算光线方向与平面法线的点积
        auto Np_dot_d = glm::dot(ray.direction, normal);
        // 判断是否平行
        if (Np_dot_d < 0.0000001f && Np_dot_d > -0.00000001f) return getMissRecord();
        // 计算相交距离
        float dp = -glm::dot(position, normal);
        float t = (-dp - glm::dot(normal, ray.origin))/Np_dot_d;
        // 检查相交距离是否在有效范围内
        if (t >= tMax || t <= tMin) return getMissRecord();
        // 计算相交点
        Vec3 hitPoint = ray.at(t);
        // 检查相交点是否在光源范围内
        Mat3x3 d{a.u, a.v, glm::cross(a.u, a.v)};
        d = glm::inverse(d);
        auto res  = d * (hitPoint - position);
        auto u = res.x, v = res.y;
        if ((u<=1 && u>=0) && (v<=1 && v>=0)) {
            return getHitRecord(t, hitPoint, normal, {});
        }
        return getMissRecord();
    }

    // // 光线与网格的相交测试
    // // ray: 光线
    // // m: 网格
    // // tMin, tMax: 相交距离范围
    // HitRecord xMesh(const Ray& ray, const Mesh& m, float tMin, float tMax) {
    //     HitRecord closestHit = nullopt;
    //     float closest = tMax;
    //     // 遍历网格的所有三角形面片
    //     for (size_t i = 0; i < m.positionIndices.size(); i += 3) {
    //         Triangle tri;
    //         tri.v1 = m.positions[m.positionIndices[i]];
    //         tri.v2 = m.positions[m.positionIndices[i + 1]];
    //         tri.v3 = m.positions[m.positionIndices[i + 2]];
    //         // 计算三角形法线
    //         tri.normal = glm::normalize(glm::cross(tri.v2 - tri.v1, tri.v3 - tri.v1));
    //         tri.material = m.material;
    //         auto hitRecord = xTriangle(ray, tri, tMin, closest);
    //         if (hitRecord && hitRecord->t < closest) {
    //             closest = hitRecord->t;
    //             closestHit = hitRecord;
    //         }
    //     }
    //     return closestHit;
    // }
    HitRecord xMesh(const Ray& ray, const Mesh& m, float tMin, float tMax) {
        HitRecord closestHit = nullopt;
        float closest = tMax;
        // 遍历网格的所有三角形面片
        for (size_t i = 0; i + 2 < m.positionIndices.size(); i += 3) {
            size_t idx0 = m.positionIndices[i + 0];
            size_t idx1 = m.positionIndices[i + 1];
            size_t idx2 = m.positionIndices[i + 2];
            // 边界检查
            if (idx0 >= m.positions.size() || idx1 >= m.positions.size() || idx2 >= m.positions.size()) continue;

            Triangle tri;
            tri.v1 = m.positions[idx0];
            tri.v2 = m.positions[idx1];
            tri.v3 = m.positions[idx2];

            // 备用面法线
            Vec3 f = glm::cross(tri.v2 - tri.v1, tri.v3 - tri.v1);
            float flen = glm::length(f);
            if (flen > 1e-8f) tri.normal = f / flen;
            else tri.normal = Vec3(0.f, 0.f, 1.f);

            // 传递材质句柄
            tri.material = m.material;

            // 拷贝顶点法线（如果 mesh 提供）
            if (!m.vertexNormals.empty()) {
                if (idx0 < m.vertexNormals.size() && idx1 < m.vertexNormals.size() && idx2 < m.vertexNormals.size()) {
                    tri.vertexNormals[0] = m.vertexNormals[idx0];
                    tri.vertexNormals[1] = m.vertexNormals[idx1];
                    tri.vertexNormals[2] = m.vertexNormals[idx2];
                }
            }

            auto hitRecord = xTriangle(ray, tri, tMin, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        return closestHit;
    }
}