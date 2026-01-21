/**
 * Solver.cpp: 2D拉格朗日流体求解器实现文件
 * 实现基于粒子的2D流体仿真算法 (SPH - Smoothed Particle Hydrodynamics)
 */

#include "Lagrangian/include/Solver.h"
#include "Global.h"
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <glm/gtc/constants.hpp>

namespace FluidSimulation
{

    namespace Lagrangian2d
    {
        /**
         * 构造函数，保存对粒子系统的引用
         * @param ps 粒子系统引用
         */
        Solver::Solver(ParticleSystem2d &ps) : mPs(ps)
        {
        }

        // ========================================================================
        // 核函数 (Kernel Functions)
        // SPH方法的核心，用于将离散粒子的属性平滑插值到连续空间
        // ========================================================================

        // Poly6核函数
        // 特点：在r=0处非零，但在r=0处梯度为0。
        // 用途：主要用于计算密度，因为它在中心处平滑，适合标量插值。
        // 公式：W(r, h) = (315 / (64 * pi * h^9)) * (h^2 - r^2)^3  (标准形式)
        // 这里使用的是简化优化版系数：(4 / (pi * h^8))
        float Poly6(float r2, float h2, float h8)
        {
            if (r2 > h2)
                return 0.0f;
            float term = h2 - r2;
            return (4.0f / (glm::pi<float>() * h8)) * term * term * term;
        }

        // Spiky核函数的梯度
        // 特点：在r=0处梯度不为0（实际上是常数斜率），随着r增加线性减小。
        // 用途：专门用于计算压力梯度力。如果使用Poly6计算压力力，当粒子重合时力会消失（因为梯度为0），导致粒子聚集（Clumping）。Spiky核解决了这个问题，提供强大的排斥力。
        // 公式：Grad(W) = -(45 / (pi * h^6)) * (h - r)^2 * (r / |r|)
        // 这里使用的是优化版系数：-(30 / (pi * h^5))
        glm::vec2 SpikyGrad(glm::vec2 r, float rLen, float h, float h5)
        {
            if (rLen > h || rLen <= 0.00001f)
                return glm::vec2(0.0f);
            float term = h - rLen;
            float coeff = -30.0f / (glm::pi<float>() * h5);
            return coeff * term * term * (r / rLen);
        }

        // 粘性核函数的拉普拉斯算子
        // 特点：用于计算二阶导数（拉普拉斯量）。
        // 用途：计算粘性力，模拟流体内部的摩擦。
        // 公式：Lap(W) = (45 / (pi * h^6)) * (h - r)
        // 这里使用的是优化版系数：(40 / (pi * h^5))
        float ViscosityLap(float rLen, float h, float h5)
        {
            if (rLen > h)
                return 0.0f;
            return (40.0f / (glm::pi<float>() * h5)) * (h - rLen);
        }

        // 旋转风车碰撞处理
        void ResolveWindmillCollision(ParticleInfo2d &p, Windmill2d &windmill, float particleRadius)
        {
            if (!windmill.enabled)
                return;

            float minDist = particleRadius + Lagrangian2dPara::eps;

            if (windmill.hubRadius > 0.0f)
            {
                float hubMinDist = windmill.hubRadius + minDist;
                glm::vec2 rel = p.position - windmill.center;
                float dist2 = glm::dot(rel, rel);
                if (dist2 < hubMinDist * hubMinDist)
                {
                    float dist = std::sqrt(dist2);
                    glm::vec2 normal = dist > 1e-6f ? rel / dist : glm::vec2(1.0f, 0.0f);

                    glm::vec2 contact = windmill.center + normal * windmill.hubRadius;
                    p.position = contact + normal * minDist;

                    glm::vec2 relToCenter = contact - windmill.center;
                    glm::vec2 bladeVel = windmill.angularVelocity * glm::vec2(-relToCenter.y, relToCenter.x);
                    glm::vec2 relVel = p.velocity - bladeVel;
                    float relNormal = glm::dot(relVel, normal);
                    if (relNormal < 0.0f)
                    {
                        glm::vec2 relVn = relNormal * normal;
                        glm::vec2 relVt = relVel - relVn;
                        relVn = -windmill.restitution * relVn;
                        relVt *= (1.0f - windmill.friction);
                        p.velocity = bladeVel + relVn + relVt;

                        // 限制碰撞后速度
                        float speed2 = glm::dot(p.velocity, p.velocity);
                        if (speed2 > Lagrangian2dPara::maxVelocity * Lagrangian2dPara::maxVelocity)
                        {
                            p.velocity *= Lagrangian2dPara::maxVelocity / std::sqrt(speed2);
                        }
                    }
                }
            }

            if (windmill.bladeCount <= 0 || windmill.bladeLength <= 0.0f || windmill.bladeWidth <= 0.0f)
                return;

            float halfLength = windmill.bladeLength * 0.5f;
            float halfWidth = windmill.bladeWidth * 0.5f;
            float minDist2 = minDist * minDist;
            float angleStep = glm::two_pi<float>() / static_cast<float>(windmill.bladeCount);
            float bladeOffset = windmill.hubRadius + halfLength;

            for (int blade = 0; blade < windmill.bladeCount; ++blade)
            {
                float angle = windmill.angle + angleStep * static_cast<float>(blade);
                glm::vec2 dir(std::cos(angle), std::sin(angle));
                glm::vec2 perp(-dir.y, dir.x);

                glm::vec2 bladeCenter = windmill.center + dir * bladeOffset;
                glm::vec2 rel = p.position - bladeCenter;
                float localX = glm::dot(rel, dir);
                float localY = glm::dot(rel, perp);
                float absX = std::fabs(localX);
                float absY = std::fabs(localY);

                bool inside = (absX <= halfLength && absY <= halfWidth);
                glm::vec2 closest;
                glm::vec2 normal;

                if (inside)
                {
                    float dx = halfLength - absX;
                    float dy = halfWidth - absY;
                    if (dx < dy)
                    {
                        normal = (localX >= 0.0f) ? dir : -dir;
                        closest = bladeCenter + dir * (localX >= 0.0f ? halfLength : -halfLength) + perp * localY;
                    }
                    else
                    {
                        normal = (localY >= 0.0f) ? perp : -perp;
                        closest = bladeCenter + dir * localX + perp * (localY >= 0.0f ? halfWidth : -halfWidth);
                    }
                }
                else
                {
                    float clampedX = (std::max)(-halfLength, (std::min)(halfLength, localX));
                    float clampedY = (std::max)(-halfWidth, (std::min)(halfWidth, localY));

                    closest = bladeCenter + dir * clampedX + perp * clampedY;
                    glm::vec2 delta = p.position - closest;
                    float dist2 = glm::dot(delta, delta);
                    if (dist2 >= minDist2)
                        continue;

                    if (dist2 > 1e-8f)
                        normal = delta / std::sqrt(dist2);
                    else
                        normal = (localY >= 0.0f) ? perp : -perp;
                }

                // 保存碰撞前速度用于力矩计算
                glm::vec2 oldVelocity = p.velocity;

                p.position = closest + normal * minDist;

                glm::vec2 relToCenter = closest - windmill.center;
                glm::vec2 bladeVel = windmill.angularVelocity * glm::vec2(-relToCenter.y, relToCenter.x);
                glm::vec2 relVel = p.velocity - bladeVel;
                float relNormal = glm::dot(relVel, normal);
                if (relNormal < 0.0f)
                {
                    glm::vec2 relVn = relNormal * normal;
                    glm::vec2 relVt = relVel - relVn;
                    relVn = -windmill.restitution * relVn;
                    relVt *= (1.0f - windmill.friction);
                    p.velocity = bladeVel + relVn + relVt;

                    // 限制碰撞后速度，避免数值爆炸
                    float speed2 = glm::dot(p.velocity, p.velocity);
                    if (speed2 > Lagrangian2dPara::maxVelocity * Lagrangian2dPara::maxVelocity)
                    {
                        p.velocity *= Lagrangian2dPara::maxVelocity / std::sqrt(speed2);
                    }

                    // 计算并施加力矩（流体驱动水车）
                    // 力矩 = r × F, 其中 F 是碰撞力
                    glm::vec2 impulse = (p.velocity - oldVelocity);
                    float torque = relToCenter.x * impulse.y - relToCenter.y * impulse.x;
                    // 大幅降低力矩缩放因子以提高稳定性
                    windmill.torque += torque * 0.002f;
                }
            }
        }

        /**
         * 求解流体方程
         * 实现一步粒子流体的仿真计算
         * 采用 SPH (Smoothed Particle Hydrodynamics) 方法
         */
        void Solver::solve()
        {
            // 获取仿真参数
            float dt = Lagrangian2dPara::dt / Lagrangian2dPara::substep; // 子步长
            float h = mPs.supportRadius;                                 // 支持半径 (平滑长度)
            float h2 = h * h;
            float h5 = pow(h, 5);
            float h8 = pow(h, 8);
            float mass = Lagrangian2dPara::density * mPs.particleVolume; // 粒子质量
            float restDensity = Lagrangian2dPara::density;               // 静止密度 (参考密度)
            float stiffness = Lagrangian2dPara::stiffness;               // 刚度系数 (气体常数 k)
            float exponent = Lagrangian2dPara::exponent;                 // 状态方程指数 (gamma)
            float viscosity = Lagrangian2dPara::viscosity;               // 粘度系数 (mu)

            // 修正重力方向：假设Y轴向上，重力应为负值
            glm::vec2 gravity = glm::vec2(Lagrangian2dPara::gravityX, -Lagrangian2dPara::gravityY);

            if (Lagrangian2dPara::enableIncompressible)
            {
                int iterations = (std::max)(1, Lagrangian2dPara::incompressibleIterations);
                float beta = 2.0f * dt * dt * mass * mass / (restDensity * restDensity);
                float lambdaEps = 1e-6f;
                const int particleCount = static_cast<int>(mPs.particles.size());
                std::vector<glm::vec2> pressureAccels(particleCount, glm::vec2(0.0f));

                for (int step = 0; step < Lagrangian2dPara::substep; ++step)
                {
                    mPs.updateMovingSolid(dt);
                    mPs.updateWindmill(dt);

#pragma omp parallel for
                    for (int i = 0; i < particleCount; ++i)
                    {
                        ParticleInfo2d &p = mPs.particles[i];
                        p.blockId = mPs.getBlockIdByPosition(p.position);
                    }

                    mPs.updateBlockInfo();

                    // 1) density for viscosity term
#pragma omp parallel for
                    for (int i = 0; i < particleCount; ++i)
                    {
                        ParticleInfo2d &p = mPs.particles[i];
                        float density = 0.0f;

                        int bRow = p.blockId / mPs.blockNum.x;
                        int bCol = p.blockId % mPs.blockNum.x;

                        for (int y = -1; y <= 1; ++y)
                        {
                            for (int x = -1; x <= 1; ++x)
                            {
                                int nRow = bRow + y;
                                int nCol = bCol + x;
                                if (nRow >= 0 && nRow < mPs.blockNum.y && nCol >= 0 && nCol < mPs.blockNum.x)
                                {
                                    uint32_t nBlockId = nRow * mPs.blockNum.x + nCol;
                                    glm::uvec2 range = mPs.blockExtens[nBlockId];
                                    for (uint32_t j = range.x; j < range.y; ++j)
                                    {
                                        const ParticleInfo2d &pj = mPs.particles[j];
                                        glm::vec2 r = p.position - pj.position;
                                        float r2 = glm::dot(r, r);
                                        if (r2 < h2)
                                        {
                                            density += mass * Poly6(r2, h2, h8);
                                        }
                                    }
                                }
                            }
                        }

                        p.density = (std::max)(density, restDensity);
                    }

                    // 2) external forces (gravity + viscosity)
#pragma omp parallel for
                    for (int i = 0; i < particleCount; ++i)
                    {
                        ParticleInfo2d &p = mPs.particles[i];
                        glm::vec2 viscosityForce(0.0f);

                        int bRow = p.blockId / mPs.blockNum.x;
                        int bCol = p.blockId % mPs.blockNum.x;

                        for (int y = -1; y <= 1; ++y)
                        {
                            for (int x = -1; x <= 1; ++x)
                            {
                                int nRow = bRow + y;
                                int nCol = bCol + x;
                                if (nRow >= 0 && nRow < mPs.blockNum.y && nCol >= 0 && nCol < mPs.blockNum.x)
                                {
                                    uint32_t nBlockId = nRow * mPs.blockNum.x + nCol;
                                    glm::uvec2 range = mPs.blockExtens[nBlockId];
                                    for (uint32_t j = range.x; j < range.y; ++j)
                                    {
                                        if (i == j)
                                            continue;
                                        const ParticleInfo2d &pj = mPs.particles[j];
                                        glm::vec2 r = p.position - pj.position;
                                        float rLen = glm::length(r);
                                        if (rLen < h)
                                        {
                                            viscosityForce += viscosity * mass * (pj.velocity - p.velocity) * (1.0f / pj.density) * ViscosityLap(rLen, h, h5);
                                        }
                                    }
                                }
                            }
                        }

                        p.accleration = viscosityForce / mass + gravity;
                    }

                    // 3) predict positions
#pragma omp parallel for
                    for (int i = 0; i < particleCount; ++i)
                    {
                        ParticleInfo2d &p = mPs.particles[i];
                        p.prevPosition = p.position;
                        p.velocity += p.accleration * dt;
                        p.position += p.velocity * dt;

                        if (p.position.x < mPs.lowerBound.x)
                            p.position.x = mPs.lowerBound.x + Lagrangian2dPara::eps;
                        if (p.position.x > mPs.upperBound.x)
                            p.position.x = mPs.upperBound.x - Lagrangian2dPara::eps;
                        if (p.position.y < mPs.lowerBound.y)
                            p.position.y = mPs.lowerBound.y + Lagrangian2dPara::eps;
                        if (p.position.y > mPs.upperBound.y)
                            p.position.y = mPs.upperBound.y - Lagrangian2dPara::eps;

                        if (mPs.movingSolid.enabled)
                        {
                            glm::vec2 toParticle = p.position - mPs.movingSolid.position;
                            float minDist = mPs.movingSolid.radius + mPs.particleRadius;
                            float dist2 = glm::dot(toParticle, toParticle);
                            if (dist2 < minDist * minDist)
                            {
                                float dist = sqrt(dist2);
                                glm::vec2 normal = dist > 1e-6f ? toParticle / dist : glm::vec2(1.0f, 0.0f);
                                p.position = mPs.movingSolid.position + normal * (minDist + Lagrangian2dPara::eps);
                            }
                        }

                        ResolveWindmillCollision(p, mPs.windmill, mPs.particleRadius);

                        p.pressure = 0.0f;
                        p.pressDivDens2 = 0.0f;
                        p.blockId = mPs.getBlockIdByPosition(p.position);
                    }

                    mPs.updateBlockInfo();

                    // 4) pressure correction iterations (PCISPH)
                    for (int iter = 0; iter < iterations; ++iter)
                    {
#pragma omp parallel for
                        for (int i = 0; i < particleCount; ++i)
                        {
                            ParticleInfo2d &p = mPs.particles[i];
                            float density = 0.0f;
                            float sumGrad2 = 0.0f;
                            glm::vec2 gradSum(0.0f);

                            int bRow = p.blockId / mPs.blockNum.x;
                            int bCol = p.blockId % mPs.blockNum.x;

                            for (int y = -1; y <= 1; ++y)
                            {
                                for (int x = -1; x <= 1; ++x)
                                {
                                    int nRow = bRow + y;
                                    int nCol = bCol + x;
                                    if (nRow >= 0 && nRow < mPs.blockNum.y && nCol >= 0 && nCol < mPs.blockNum.x)
                                    {
                                        uint32_t nBlockId = nRow * mPs.blockNum.x + nCol;
                                        glm::uvec2 range = mPs.blockExtens[nBlockId];
                                        for (uint32_t j = range.x; j < range.y; ++j)
                                        {
                                            const ParticleInfo2d &pj = mPs.particles[j];
                                            glm::vec2 r = p.position - pj.position;
                                            float r2 = glm::dot(r, r);
                                            if (r2 < h2)
                                            {
                                                density += mass * Poly6(r2, h2, h8);
                                                if (i != j)
                                                {
                                                    float rLen = sqrt(r2);
                                                    glm::vec2 grad = SpikyGrad(r, rLen, h, h5);
                                                    gradSum += grad;
                                                    sumGrad2 += glm::dot(grad, grad);
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            density = (std::max)(density, restDensity);
                            p.density = density;

                            sumGrad2 += glm::dot(gradSum, gradSum);
                            float delta = 1.0f / (beta * sumGrad2 + lambdaEps);
                            p.pressure = (std::max)(0.0f, p.pressure + delta * (density - restDensity));
                        }

#pragma omp parallel for
                        for (int i = 0; i < particleCount; ++i)
                        {
                            ParticleInfo2d &p = mPs.particles[i];
                            glm::vec2 pressureAccel(0.0f);

                            int bRow = p.blockId / mPs.blockNum.x;
                            int bCol = p.blockId % mPs.blockNum.x;

                            for (int y = -1; y <= 1; ++y)
                            {
                                for (int x = -1; x <= 1; ++x)
                                {
                                    int nRow = bRow + y;
                                    int nCol = bCol + x;
                                    if (nRow >= 0 && nRow < mPs.blockNum.y && nCol >= 0 && nCol < mPs.blockNum.x)
                                    {
                                        uint32_t nBlockId = nRow * mPs.blockNum.x + nCol;
                                        glm::uvec2 range = mPs.blockExtens[nBlockId];
                                        for (uint32_t j = range.x; j < range.y; ++j)
                                        {
                                            if (i == j)
                                                continue;
                                            const ParticleInfo2d &pj = mPs.particles[j];
                                            glm::vec2 r = p.position - pj.position;
                                            float rLen = glm::length(r);
                                            if (rLen < h)
                                            {
                                                pressureAccel -= mass * (p.pressure + pj.pressure) / (2.0f * restDensity * restDensity) * SpikyGrad(r, rLen, h, h5);
                                            }
                                        }
                                    }
                                }
                            }

                            pressureAccels[i] = pressureAccel;
                        }

#pragma omp parallel for
                        for (int i = 0; i < particleCount; ++i)
                        {
                            ParticleInfo2d &p = mPs.particles[i];
                            p.velocity += pressureAccels[i] * dt;
                            p.position += pressureAccels[i] * (dt * dt);

                            if (p.position.x < mPs.lowerBound.x)
                                p.position.x = mPs.lowerBound.x + Lagrangian2dPara::eps;
                            if (p.position.x > mPs.upperBound.x)
                                p.position.x = mPs.upperBound.x - Lagrangian2dPara::eps;
                            if (p.position.y < mPs.lowerBound.y)
                                p.position.y = mPs.lowerBound.y + Lagrangian2dPara::eps;
                            if (p.position.y > mPs.upperBound.y)
                                p.position.y = mPs.upperBound.y - Lagrangian2dPara::eps;

                            if (mPs.movingSolid.enabled)
                            {
                                glm::vec2 toParticle = p.position - mPs.movingSolid.position;
                                float minDist = mPs.movingSolid.radius + mPs.particleRadius;
                                float dist2 = glm::dot(toParticle, toParticle);
                                if (dist2 < minDist * minDist)
                                {
                                    float dist = sqrt(dist2);
                                    glm::vec2 normal = dist > 1e-6f ? toParticle / dist : glm::vec2(1.0f, 0.0f);
                                    p.position = mPs.movingSolid.position + normal * (minDist + Lagrangian2dPara::eps);
                                }
                            }
                            ResolveWindmillCollision(p, mPs.windmill, mPs.particleRadius);
                        }
                    }

                    // update block ids once after pressure iterations
#pragma omp parallel for
                    for (int i = 0; i < particleCount; ++i)
                    {
                        ParticleInfo2d &p = mPs.particles[i];
                        p.blockId = mPs.getBlockIdByPosition(p.position);
                    }

                    mPs.updateBlockInfo();

                    // 5) update velocity from displacement
#pragma omp parallel for
                    for (int i = 0; i < particleCount; ++i)
                    {
                        ParticleInfo2d &p = mPs.particles[i];
                        p.velocity = (p.position - p.prevPosition) / dt;
                    }

                    // 6) XSPH smoothing + boundary/solid response
#pragma omp parallel for
                    for (int i = 0; i < particleCount; ++i)
                    {
                        ParticleInfo2d &p = mPs.particles[i];
                        glm::vec2 v_xsph = p.velocity;
                        float xsphFactor = 0.1f;

                        int bRow = p.blockId / mPs.blockNum.x;
                        int bCol = p.blockId % mPs.blockNum.x;

                        for (int y = -1; y <= 1; ++y)
                        {
                            for (int x = -1; x <= 1; ++x)
                            {
                                int nRow = bRow + y;
                                int nCol = bCol + x;
                                if (nRow >= 0 && nRow < mPs.blockNum.y && nCol >= 0 && nCol < mPs.blockNum.x)
                                {
                                    uint32_t nBlockId = nRow * mPs.blockNum.x + nCol;
                                    glm::uvec2 range = mPs.blockExtens[nBlockId];
                                    for (uint32_t j = range.x; j < range.y; ++j)
                                    {
                                        if (i == j)
                                            continue;
                                        const ParticleInfo2d &pj = mPs.particles[j];
                                        glm::vec2 r = p.position - pj.position;
                                        float r2 = glm::dot(r, r);
                                        if (r2 < h2)
                                        {
                                            v_xsph += xsphFactor * (mass / pj.density) * (pj.velocity - p.velocity) * Poly6(r2, h2, h8);
                                        }
                                    }
                                }
                            }
                        }

                        p.velocity = v_xsph;

                        float vLen = glm::length(p.velocity);
                        if (vLen > Lagrangian2dPara::maxVelocity)
                            p.velocity = p.velocity / vLen * Lagrangian2dPara::maxVelocity;

                        if (p.position.x < mPs.lowerBound.x)
                        {
                            p.position.x = mPs.lowerBound.x + Lagrangian2dPara::eps;
                            p.velocity.x = -p.velocity.x * Lagrangian2dPara::velocityAttenuation;
                        }
                        if (p.position.x > mPs.upperBound.x)
                        {
                            p.position.x = mPs.upperBound.x - Lagrangian2dPara::eps;
                            p.velocity.x = -p.velocity.x * Lagrangian2dPara::velocityAttenuation;
                        }
                        if (p.position.y < mPs.lowerBound.y)
                        {
                            p.position.y = mPs.lowerBound.y + Lagrangian2dPara::eps;
                            p.velocity.y = -p.velocity.y * Lagrangian2dPara::velocityAttenuation;
                        }
                        if (p.position.y > mPs.upperBound.y)
                        {
                            p.position.y = mPs.upperBound.y - Lagrangian2dPara::eps;
                            p.velocity.y = -p.velocity.y * Lagrangian2dPara::velocityAttenuation;
                        }

                        if (mPs.movingSolid.enabled)
                        {
                            glm::vec2 toParticle = p.position - mPs.movingSolid.position;
                            float minDist = mPs.movingSolid.radius + mPs.particleRadius;
                            float dist2 = glm::dot(toParticle, toParticle);
                            if (dist2 < minDist * minDist)
                            {
                                float dist = sqrt(dist2);
                                glm::vec2 normal = dist > 1e-6f ? toParticle / dist : glm::vec2(1.0f, 0.0f);
                                p.position = mPs.movingSolid.position + normal * (minDist + Lagrangian2dPara::eps);

                                glm::vec2 relVel = p.velocity - mPs.movingSolid.velocity;
                                float relNormal = glm::dot(relVel, normal);
                                if (relNormal < 0.0f)
                                {
                                    glm::vec2 relVn = relNormal * normal;
                                    glm::vec2 relVt = relVel - relVn;
                                    relVn = -mPs.movingSolid.restitution * relVn;
                                    relVt *= (1.0f - mPs.movingSolid.friction);
                                    p.velocity = mPs.movingSolid.velocity + relVn + relVt;
                                }
                            }
                        }

                        ResolveWindmillCollision(p, mPs.windmill, mPs.particleRadius);

                        p.blockId = mPs.getBlockIdByPosition(p.position);
                    }
                }

                return;
            }
            // 子步迭代：为了保证数值稳定性，通常将一个时间步长拆分为多个小步
            for (int step = 0; step < Lagrangian2dPara::substep; ++step)
            {
                mPs.updateMovingSolid(dt);
                mPs.updateWindmill(dt);

                // 邻域搜索 (为计算密度做准备)
                mPs.updateBlockInfo();

                // 1. 计算密度
#pragma omp parallel for
                for (int i = 0; i < mPs.particles.size(); ++i)
                {
                    ParticleInfo2d &p = mPs.particles[i];
                    p.density = 0.0f;

                    // 获取当前粒子所在的网格坐标
                    int bRow = p.blockId / mPs.blockNum.x;
                    int bCol = p.blockId % mPs.blockNum.x;

                    // 遍历相邻的9个网格 (3x3区域)
                    for (int y = -1; y <= 1; ++y)
                    {
                        for (int x = -1; x <= 1; ++x)
                        {
                            int nRow = bRow + y;
                            int nCol = bCol + x;
                            // 边界检查
                            if (nRow >= 0 && nRow < mPs.blockNum.y && nCol >= 0 && nCol < mPs.blockNum.x)
                            {
                                uint32_t nBlockId = nRow * mPs.blockNum.x + nCol;
                                glm::uvec2 range = mPs.blockExtens[nBlockId];
                                // 遍历该网格内的所有粒子
                                for (uint32_t j = range.x; j < range.y; ++j)
                                {
                                    const ParticleInfo2d &pj = mPs.particles[j];
                                    glm::vec2 r = p.position - pj.position;
                                    float r2 = glm::dot(r, r);
                                    // 仅当距离小于支持半径时才计算贡献
                                    if (r2 < h2)
                                    {
                                        p.density += mass * Poly6(r2, h2, h8);
                                    }
                                }
                            }
                        }
                    }
                    // 避免后续除以零，并防止密度过低导致的数值问题
                    p.density = (std::max)(p.density, restDensity);
                }

                // 2. 计算压力
#pragma omp parallel for
                for (int i = 0; i < mPs.particles.size(); ++i)
                {
                    ParticleInfo2d &p = mPs.particles[i];

                    // 计算压力
                    p.pressure = (std::max)(0.0f, stiffness * (pow(p.density / restDensity, exponent) - 1.0f));

                    // 预计算 p / rho^2，这在压力梯度力公式中会用到
                    p.pressDivDens2 = p.pressure / (p.density * p.density);
                }

                // 3. 计算加速度
#pragma omp parallel for
                for (int i = 0; i < mPs.particles.size(); ++i)
                {
                    ParticleInfo2d &p = mPs.particles[i];
                    glm::vec2 pressureForce(0.0f);
                    glm::vec2 viscosityForce(0.0f);

                    int bRow = p.blockId / mPs.blockNum.x;
                    int bCol = p.blockId % mPs.blockNum.x;

                    // 再次遍历邻居粒子计算相互作用力
                    for (int y = -1; y <= 1; ++y)
                    {
                        for (int x = -1; x <= 1; ++x)
                        {
                            int nRow = bRow + y;
                            int nCol = bCol + x;
                            if (nRow >= 0 && nRow < mPs.blockNum.y && nCol >= 0 && nCol < mPs.blockNum.x)
                            {
                                uint32_t nBlockId = nRow * mPs.blockNum.x + nCol;
                                glm::uvec2 range = mPs.blockExtens[nBlockId];
                                for (uint32_t j = range.x; j < range.y; ++j)
                                {
                                    if (i == j)
                                        continue; // 跳过自己
                                    const ParticleInfo2d &pj = mPs.particles[j];
                                    glm::vec2 r = p.position - pj.position;
                                    float rLen = glm::length(r);

                                    if (rLen < h)
                                    {
                                        // 压力梯度力
                                        pressureForce -= mass * mass * (p.pressDivDens2 + pj.pressDivDens2) * SpikyGrad(r, rLen, h, h5);

                                        // 粘性力
                                        viscosityForce += viscosity * mass * mass * (1.0f / pj.density) * (pj.velocity - p.velocity) * ViscosityLap(rLen, h, h5);
                                    }
                                }
                            }
                        }
                    }

                    // 合力 = 压力力 + 粘性力
                    glm::vec2 totalForce = pressureForce + viscosityForce;
                    // 牛顿第二定律：F = ma => a = F/m + g
                    p.accleration = totalForce / mass + gravity;
                }

                // 4. 更新速度和位置
                const MovingSolid2d solid = mPs.movingSolid;
                const float particleRadius = mPs.particleRadius;
#pragma omp parallel for
                for (int i = 0; i < mPs.particles.size(); ++i)
                {
                    ParticleInfo2d &p = mPs.particles[i];

                    // 半隐式欧拉积分：先更新速度，再更新位置
                    p.velocity += p.accleration * dt;

                    // 限制最大速度 (CFL条件)
                    float vLen = glm::length(p.velocity);
                    if (vLen > Lagrangian2dPara::maxVelocity)
                    {
                        p.velocity = p.velocity / vLen * Lagrangian2dPara::maxVelocity;
                    }

                    // XSPH 人工粘性
                    glm::vec2 v_xsph = p.velocity;
                    float xsphFactor = 0.1f; // 混合系数 [0.0, 0.5]

                    int bRow = p.blockId / mPs.blockNum.x;
                    int bCol = p.blockId % mPs.blockNum.x;

                    // 计算 XSPH 修正项
                    for (int y = -1; y <= 1; ++y)
                    {
                        for (int x = -1; x <= 1; ++x)
                        {
                            int nRow = bRow + y;
                            int nCol = bCol + x;
                            if (nRow >= 0 && nRow < mPs.blockNum.y && nCol >= 0 && nCol < mPs.blockNum.x)
                            {
                                uint32_t nBlockId = nRow * mPs.blockNum.x + nCol;
                                glm::uvec2 range = mPs.blockExtens[nBlockId];
                                for (uint32_t j = range.x; j < range.y; ++j)
                                {
                                    if (i == j)
                                        continue;
                                    const ParticleInfo2d &pj = mPs.particles[j];
                                    glm::vec2 r = p.position - pj.position;
                                    float r2 = glm::dot(r, r);
                                    if (r2 < h2)
                                    {
                                        v_xsph += xsphFactor * (mass / pj.density) * (pj.velocity - p.velocity) * Poly6(r2, h2, h8);
                                    }
                                }
                            }
                        }
                    }

                    // 使用修正后的速度更新位置
                    p.position += v_xsph * dt;

                    // 5. 边界检查
                    if (p.position.x < mPs.lowerBound.x)
                    {
                        p.position.x = mPs.lowerBound.x + Lagrangian2dPara::eps;
                        p.velocity.x = -p.velocity.x * Lagrangian2dPara::velocityAttenuation;
                    }
                    if (p.position.x > mPs.upperBound.x)
                    {
                        p.position.x = mPs.upperBound.x - Lagrangian2dPara::eps;
                        p.velocity.x = -p.velocity.x * Lagrangian2dPara::velocityAttenuation;
                    }
                    if (p.position.y < mPs.lowerBound.y)
                    {
                        p.position.y = mPs.lowerBound.y + Lagrangian2dPara::eps;
                        p.velocity.y = -p.velocity.y * Lagrangian2dPara::velocityAttenuation;
                    }
                    if (p.position.y > mPs.upperBound.y)
                    {
                        p.position.y = mPs.upperBound.y - Lagrangian2dPara::eps;
                        p.velocity.y = -p.velocity.y * Lagrangian2dPara::velocityAttenuation;
                    }

                    // 运动固体碰撞
                    if (solid.enabled)
                    {
                        glm::vec2 toParticle = p.position - solid.position;
                        float minDist = solid.radius + particleRadius;
                        float dist2 = glm::dot(toParticle, toParticle);
                        if (dist2 < minDist * minDist)
                        {
                            float dist = sqrt(dist2);
                            glm::vec2 normal = dist > 1e-6f ? toParticle / dist : glm::vec2(1.0f, 0.0f);
                            p.position = solid.position + normal * (minDist + Lagrangian2dPara::eps);

                            glm::vec2 relVel = p.velocity - solid.velocity;
                            float relNormal = glm::dot(relVel, normal);
                            if (relNormal < 0.0f)
                            {
                                glm::vec2 relVn = relNormal * normal;
                                glm::vec2 relVt = relVel - relVn;
                                relVn = -solid.restitution * relVn;
                                relVt *= (1.0f - solid.friction);
                                p.velocity = solid.velocity + relVn + relVt;
                            }
                        }
                    }

                    ResolveWindmillCollision(p, mPs.windmill, mPs.particleRadius);

                    // 6. 更新块ID
                    p.blockId = mPs.getBlockIdByPosition(p.position);
                }
            }
        }
    }
}
