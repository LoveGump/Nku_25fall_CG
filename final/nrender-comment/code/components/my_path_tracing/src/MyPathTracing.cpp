#include "server/Server.hpp"

#include "MyPathTracing.hpp"

#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"

#include "glm/gtc/matrix_transform.hpp"
#include <chrono>
#include <atomic>
using namespace std::chrono;
namespace
{
    constexpr float RAY_EPS = 0.000001f;
}

namespace MyPathTracing
{

    // 线程局部的计时器 (单位：纳秒)
    // thread_local long long time_intersect = 0; // 撞击检测耗时
    // thread_local long long time_sample = 0;    // 随机数生成耗时 (抢锁嫌疑人)
    // thread_local long long time_nee = 0;       // NEE 整体逻辑耗时
    // thread_local long long count_trace = 0;    // trace 调用次数
    /**
     * Gamma校正函数
     * 对颜色进行平方根校正，模拟人眼对亮度的感知
     * @param rgb 原始颜色
     * @return 校正后的颜色
     */
    RGB MyPathTracingRenderer::gamma(const RGB &rgb)
    {
        return glm::sqrt(rgb);
    }

    /**
     * 渲染任务函数（多线程）
     * 处理指定范围内的像素行，进行路径追踪计算
     * @param pixels 像素缓冲区
     * @param width 图像宽度
     * @param height 图像高度
     * @param off 起始行偏移
     * @param step 行步长（用于多线程分配）
     */
    void MyPathTracingRenderer::renderTask(RGBA *pixels, int width, int height, int off, int step)
    {
        for (int i = off; i < height; i += step)
        { // 按步长处理行
            for (int j = 0; j < width; j++)
            {                        // 处理每行的像素
                Vec3 color{0, 0, 0}; // 初始化像素颜色

                // 多重采样抗锯齿
                for (int k = 0; k < samples; k++)
                {
                    // 在像素内随机采样
                    auto r = defaultSamplerInstance<UniformInSquare>().sample2d();
                    float rx = r.x;
                    float ry = r.y;
                    float x = (float(j) + rx) / float(width);  // 归一化x坐标
                    float y = (float(i) + ry) / float(height); // 归一化y坐标

                    // 从相机发射光线
                    auto ray = camera.shoot(x, y);
                    color += trace(ray, 0); // 路径追踪
                }
                color /= samples;                                  // 平均采样结果
                color = gamma(color);                              // Gamma校正
                pixels[(height - i - 1) * width + j] = {color, 1}; // 存储像素（翻转y坐标）
            }
        }
        // 在函数最后添加：
        // 将纳秒转换为毫秒方便阅读
        // double ms_intersect = time_intersect / 1000000.0;
        // double ms_sample = time_sample / 1000000.0;
        // double ms_nee = time_nee / 1000000.0;

        // // 为了防止多线程打印乱码，简单拼接一下字符串
        // string logMsg = "Thread [" + to_string(off) + "] Report:\n";
        // logMsg += "  Calls: " + to_string(count_trace) + "\n";
        // logMsg += "  albedo Time: " + to_string(ms_intersect) + " ms\n";
        // logMsg += "  Sampler Time:   " + to_string(ms_sample) + " ms (Inside NEE)\n";
        // logMsg += "  Total NEE Time: " + to_string(ms_nee) + " ms\n";

        // // 输出日志 (使用 getServer().logger 或者 std::cout)
        // // 建议用 cout 因为 logger 可能会被过滤
        // std::cout << logMsg << std::endl;
    }

    /**
     * 主渲染函数
     * 初始化着色器，执行多线程渲染，返回渲染结果
     * @return 渲染结果（像素数据、宽度、高度）
     */
    auto MyPathTracingRenderer::render() -> RenderResult
    {
        // 初始化着色器程序
        shaderPrograms.clear();
        ShaderCreator shaderCreator{};
        for (auto &m : scene.materials)
        {
            shaderPrograms.push_back(shaderCreator.create(m, scene.textures));
        }

        // 分配像素缓冲区
        RGBA *pixels = new RGBA[width * height]{};

        // 将局部坐标转换成世界坐标
        VertexTransformer vertexTransformer{};
        vertexTransformer.exec(spScene);

        buildAcceleration();

        // 多线程渲染
        const auto taskNums = 16; // 使用16个线程
        thread t[taskNums];
        for (int i = 0; i < taskNums; i++)
        {
            t[i] = thread(&MyPathTracingRenderer::renderTask,
                          this, pixels, width, height, i, taskNums);
        }
        for (int i = 0; i < taskNums; i++)
        {
            t[i].join(); // 等待所有线程完成
        }
        getServer().logger.log("Done...");
        return {pixels, width, height};
    }

    void MyPathTracingRenderer::buildAcceleration()
    {
        if (!useKDTree || scene.triangleBuffer.empty())
        {
            kdTree.reset();
            return;
        }
        kdTree = std::make_unique<KDTree>(scene.triangleBuffer);
    }
    /**
     * 释放渲染结果内存
     * @param r 渲染结果
     */
    void MyPathTracingRenderer::release(const RenderResult &r)
    {
        auto [p, w, h] = r;
        delete[] p; // 释放像素缓冲区
    }

    /**
     * 查找光线与最近物体的相交
     * 若启用KD树则优先使用加速结构，否则回退为线性遍历
     */
    HitRecord MyPathTracingRenderer::closestHitObject(const Ray &r)
    {
        if (!(useKDTree && kdTree && !kdTree->empty()))
        {
            return closestHitLinear(r);
        }

        float closest = FLOAT_INF;
        HitRecord best = kdTree->intersect(r, RAY_EPS, closest);
        if (best)
        {
            closest = best->t;
        }

        auto tryUpdate = [&](const HitRecord &hit)
        {
            if (hit && hit->t < closest)
            {
                closest = hit->t;
                best = hit;
            }
        };

        for (auto &s : scene.sphereBuffer)
        {
            auto hitRecord = Intersection::xSphere(r, s, RAY_EPS, closest);
            tryUpdate(hitRecord);
        }
        for (auto &p : scene.planeBuffer)
        {
            auto hitRecord = Intersection::xPlane(r, p, RAY_EPS, closest);
            tryUpdate(hitRecord);
        }
        return best;
    }

    /**
     * 线性遍历所有几何体
     * 作为KD树禁用时的基准实现
     */
    HitRecord MyPathTracingRenderer::closestHitLinear(const Ray &r)
    {
        HitRecord closestHit = nullopt;
        float closest = FLOAT_INF;

        for (auto &s : scene.sphereBuffer)
        {
            auto hitRecord = Intersection::xSphere(r, s, RAY_EPS, closest);
            if (hitRecord && hitRecord->t < closest)
            {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        for (auto &t : scene.triangleBuffer)
        {
            auto hitRecord = Intersection::xTriangle(r, t, RAY_EPS, closest);
            if (hitRecord && hitRecord->t < closest)
            {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        for (auto &p : scene.planeBuffer)
        {
            auto hitRecord = Intersection::xPlane(r, p, RAY_EPS, closest);
            if (hitRecord && hitRecord->t < closest)
            {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        return closestHit;
    }

    /**
     * 查找光线与最近光源的相交
     * 遍历所有区域光源并返回最近距离和辐射度
     */
    tuple<float, Vec3> MyPathTracingRenderer::closestHitLight(const Ray &r)
    {
        Vec3 v = {};
        HitRecord closest = getHitRecord(FLOAT_INF, {}, {}, {});

        for (auto &a : scene.areaLightBuffer)
        {
            auto hitRecord = Intersection::xAreaLight(r, a, RAY_EPS, closest->t);
            if (hitRecord && closest->t > hitRecord->t)
            {
                closest = hitRecord;
                v = a.radiance;
            }
        }

        return {closest->t, v};
    }

    // =========================================================
    // 【修正后的 Trace 函数】
    // =========================================================
    RGB MyPathTracingRenderer::trace(const Ray &r, int currDepth, bool isSpecularBounce)
    {
        if (currDepth == depth)
            return scene.ambient.constant;

        auto hitObject = closestHitObject(r);
        auto [t, emitted] = closestHitLight(r);

        // --- 1. 光源命中处理 (Hybrid Strategy 核心判决) ---
        if (!hitObject || t < hitObject->t)
        {
            if (t == FLOAT_INF)
                return Vec3{0};

            // 只有当：
            // A. 摄像机直接看灯 (currDepth == 0)
            // B. 从镜面/玻璃反射过来看灯 (isSpecularBounce == true)
            // 才允许接受光源的自发光。
            // 如果是从漫反射面过来，NEE 已经算过光了，这里必须返回 0 防止过曝。
            if (currDepth == 0 || isSpecularBounce)
                return emitted;
            else
                return Vec3{0};
        }

        auto mtlHandle = hitObject->material;
        auto shader = shaderPrograms[mtlHandle.index()];
        Vec3 hitPoint = hitObject->hitPoint;
        Vec3 N = hitObject->normal;
        Vec3 V = -r.direction;

        RGB L_direct = {0, 0, 0};
        RGB L_indirect = {0, 0, 0};

        // --- 2. 直接光照采样 (NEE) ---
        for (const auto &light : scene.areaLightBuffer)
        {
            auto rand = defaultSamplerInstance<UniformInSquare>().sample2d();
            Vec3 lightPosSample = light.position + light.u * rand.x + light.v * rand.y;
            Vec3 L_vec = lightPosSample - hitPoint;
            float distSq = glm::dot(L_vec, L_vec);
            float dist = std::sqrt(distSq);
            Vec3 L_dir = L_vec / dist;

            float cosTheta = glm::dot(N, L_dir);
            Vec3 lightNormal = glm::normalize(glm::cross(light.u, light.v));
            float cosLight = glm::dot(lightNormal, -L_dir);

            if (cosTheta > 0 && cosLight > 0)
            {
                Vec3 brdf = shader->eval(L_dir, V, N);

                // 【优化】必须判断 BRDF 是否有值
                // 如果是金属或玻璃，eval() 返回 0，这里会直接跳过，省去了 ShadowRay 的开销
                if (glm::length(brdf) > 0.0f)
                {
                    Ray shadowRay(hitPoint + N * RAY_EPS, L_dir);
                    auto blocker = closestHitObject(shadowRay);
                    bool isBlocked = (blocker && blocker->t < dist - RAY_EPS);

                    // 兼容旧式玻璃的透明阴影逻辑
                    if (isBlocked)
                    {
                        auto &blockerMat = scene.materials[blocker->material.index()];
                        if (blockerMat.hasProperty("ior"))
                        {
                            bool looksLikeOldDielectric =
                                !blockerMat.hasProperty("roughness") &&
                                !blockerMat.hasProperty("transmission") &&
                                !blockerMat.hasProperty("diffuseColor");
                            if (looksLikeOldDielectric)
                                isBlocked = false;
                        }
                    }

                    if (!isBlocked)
                    {
                        float area = glm::length(glm::cross(light.u, light.v));
                        float pdf = 1.0f / area;
                        L_direct += light.radiance * brdf * cosTheta * cosLight / (distSq * pdf);
                    }
                }
            }
        }

        // --- 3. 间接光照采样 (BSDF) ---
        auto scattered = shader->shade(r, hitPoint, N);

        // 【关键】递归时传递 isSpecular 状态
        // 如果这次是镜面反射/折射 (scattered.isSpecular == true)，
        // 下一次 trace 就会允许光源发光。
        RGB Li = trace(scattered.ray, currDepth + 1, scattered.isSpecular);

        float n_dot_in = glm::abs(glm::dot(N, scattered.ray.direction));

        if (scattered.isSpecular)
        {
            L_indirect = Li * scattered.attenuation;
        }
        else
        {
            if (scattered.pdf > 0.0f)
                L_indirect = Li * scattered.attenuation * n_dot_in / scattered.pdf;
        }

        return L_direct + L_indirect;
    }
}
