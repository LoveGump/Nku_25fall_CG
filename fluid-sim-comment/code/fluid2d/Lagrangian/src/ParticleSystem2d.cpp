#include "ParticleSystem2d.h"
#include <iostream>
#include "Global.h"
#include <unordered_set>
#include <cmath>
#include <glm/gtc/constants.hpp>

namespace FluidSimulation
{

    namespace Lagrangian2d
    {
        ParticleSystem2d::ParticleSystem2d()
        {
        }

        ParticleSystem2d::~ParticleSystem2d()
        {
        }

        // 设置流体容器的大小
        void ParticleSystem2d::setContainerSize(glm::vec2 lower = glm::vec2(-1.0f, -1.0f), glm::vec2 upper = glm::vec2(1.0f, 1.0f))
        {
            // 应用缩放
            lower *= Lagrangian2dPara::scale;
            upper *= Lagrangian2dPara::scale;

            // 设置边界,考虑支持半径和粒子直径
            lowerBound = lower - supportRadius + particleDiameter;
            upperBound = upper + supportRadius - particleDiameter;
            containerCenter = (lowerBound + upperBound) / 2.0f;

            glm::vec2 size = upperBound - lowerBound;

            // 计算块的数量和大小
            blockNum.x = floor(size.x / supportRadius);
            blockNum.y = floor(size.y / supportRadius);
            blockSize = glm::vec2(size.x / blockNum.x, size.y / blockNum.y);

            // 初始化块偏移数组
            blockIdOffs.resize(9);
            int p = 0;
            for (int j = -1; j <= 1; j++)
            {
                for (int i = -1; i <= 1; i++)
                {
                    blockIdOffs[p] = blockNum.x * j + i;
                    p++;
                }
            }

            // 清空粒子数组
            particles.clear();
        }

        // 添加流体块
        int ParticleSystem2d::addFluidBlock(glm::vec2 lowerCorner, glm::vec2 upperCorner, glm::vec2 v0, float particleSpace)
        {
            // 应用缩放
            lowerCorner *= Lagrangian2dPara::scale;
            upperCorner *= Lagrangian2dPara::scale;

            glm::vec2 size = upperCorner - lowerCorner;

            // 检查边界
            if (lowerCorner.x < lowerBound.x ||
                lowerCorner.y < lowerBound.y ||
                upperCorner.x > upperBound.x ||
                upperCorner.y > upperBound.y)
            {
                return 0;
            }

            // 计算粒子数量
            glm::uvec2 particleNum = glm::uvec2(size.x / particleSpace, size.y / particleSpace);
            std::vector<ParticleInfo2d> tempParticles(particleNum.x * particleNum.y);

            // 随机生成器,用于粒子位置的扰动
            Glb::RandomGenerator rand;
            int p = 0;
            // 在流体块中生成粒子
            for (int idX = 0; idX < particleNum.x; idX++)
            {
                for (int idY = 0; idY < particleNum.y; idY++)
                {
                    // 添加随机扰动避免规则排列
                    float x = (idX + rand.GetUniformRandom()) * particleSpace;
                    float y = (idY + rand.GetUniformRandom()) * particleSpace;

                    // 设置粒子属性
                    tempParticles[p].position = lowerCorner + glm::vec2(x, y);
                    tempParticles[p].prevPosition = tempParticles[p].position;
                    tempParticles[p].blockId = getBlockIdByPosition(tempParticles[p].position);
                    tempParticles[p].density = Lagrangian2dPara::density;
                    tempParticles[p].velocity = v0;
                    p++;
                }
            }

            // 将新粒子添加到系统中
            particles.insert(particles.end(), tempParticles.begin(), tempParticles.end());
            return particles.size();
        }

        // 根据位置获取块ID
        uint32_t ParticleSystem2d::getBlockIdByPosition(glm::vec2 position)
        {
            // 检查边界
            if (position.x < lowerBound.x ||
                position.y < lowerBound.y ||
                position.x > upperBound.x ||
                position.y > upperBound.y)
            {
                return -1;
            }

            // 计算块索引
            glm::vec2 deltePos = position - lowerBound;
            uint32_t c = floor(deltePos.x / blockSize.x);
            uint32_t r = floor(deltePos.y / blockSize.y);

            // Clamp indices to avoid out of bounds
            if (c >= blockNum.x)
                c = blockNum.x - 1;
            if (r >= blockNum.y)
                r = blockNum.y - 1;

            return r * blockNum.x + c;
        }

        // 更新块信息
        void ParticleSystem2d::updateBlockInfo()
        {
            if (particles.empty())
                return;

            // 根据块ID对粒子进行排序
            std::sort(particles.begin(), particles.end(),
                      [=](ParticleInfo2d &first, ParticleInfo2d &second)
                      {
                          return first.blockId < second.blockId;
                      });

            // 更新每个块中粒子的范围
            // Re-use memory if possible or just clear
            blockExtens.assign(blockNum.x * blockNum.y, glm::uvec2(0, 0));

            if (particles.empty())
                return;

            uint32_t curBlockId = particles[0].blockId;
            int left = 0;
            int right;
            for (right = 0; right < particles.size(); right++)
            {
                if (particles[right].blockId != curBlockId)
                {
                    if (curBlockId < blockExtens.size())
                    {
                        blockExtens[curBlockId] = glm::uvec2(left, right);
                    }
                    left = right;
                    curBlockId = particles[right].blockId;
                }
            }
            if (curBlockId < blockExtens.size())
            {
                blockExtens[curBlockId] = glm::uvec2(left, right);
            }
        }

        void ParticleSystem2d::setMovingSolid(glm::vec2 center, float radius, glm::vec2 velocity, float restitution, float friction)
        {
            movingSolid.enabled = true;
            movingSolid.position = center * Lagrangian2dPara::scale;
            movingSolid.radius = radius * Lagrangian2dPara::scale;
            movingSolid.velocity = velocity;
            movingSolid.restitution = restitution;
            movingSolid.friction = friction;
        }

        void ParticleSystem2d::disableMovingSolid()
        {
            movingSolid.enabled = false;
        }

        void ParticleSystem2d::updateMovingSolid(float dt)
        {
            if (!movingSolid.enabled)
                return;

            movingSolid.position += movingSolid.velocity * dt;

            float minX = lowerBound.x + movingSolid.radius;
            float maxX = upperBound.x - movingSolid.radius;
            float minY = lowerBound.y + movingSolid.radius;
            float maxY = upperBound.y - movingSolid.radius;

            if (movingSolid.position.x < minX)
            {
                movingSolid.position.x = minX + Lagrangian2dPara::eps;
                movingSolid.velocity.x = -movingSolid.velocity.x;
            }
            else if (movingSolid.position.x > maxX)
            {
                movingSolid.position.x = maxX - Lagrangian2dPara::eps;
                movingSolid.velocity.x = -movingSolid.velocity.x;
            }

            if (movingSolid.position.y < minY)
            {
                movingSolid.position.y = minY + Lagrangian2dPara::eps;
                movingSolid.velocity.y = -movingSolid.velocity.y;
            }
            else if (movingSolid.position.y > maxY)
            {
                movingSolid.position.y = maxY - Lagrangian2dPara::eps;
                movingSolid.velocity.y = -movingSolid.velocity.y;
            }
        }

        void ParticleSystem2d::setWindmill(glm::vec2 center, float bladeLength, float bladeWidth, float hubRadius, int bladeCount, float angularVelocity, float restitution, float friction)
        {
            windmill.enabled = true;
            windmill.center = center * Lagrangian2dPara::scale;
            windmill.bladeLength = bladeLength * Lagrangian2dPara::scale;
            windmill.bladeWidth = bladeWidth * Lagrangian2dPara::scale;
            windmill.hubRadius = (std::max)(0.0f, hubRadius * Lagrangian2dPara::scale);
            windmill.bladeCount = (std::max)(1, bladeCount);
            windmill.angularVelocity = angularVelocity;
            windmill.restitution = restitution;
            windmill.friction = friction;
            windmill.angle = 0.0f;
            windmill.torque = 0.0f;
            windmill.smoothedTorque = 0.0f;
            windmill.useFixedSpeed = true; // 使用固定角速度模式
            windmill.targetAngularVelocity = angularVelocity;

            // 计算转动惯量（近似为均匀分布的叶片）
            float bladeMass = 2.0f; // 每个叶片质量（增加以提高稳定性）
            windmill.momentOfInertia = bladeCount * bladeMass * bladeLength * bladeLength / 3.0f;
            windmill.angularDamping = 2.0f; // 空气阻力和摩擦造成的角速度衰减（增加阻尼）
        }

        void ParticleSystem2d::disableWindmill()
        {
            windmill.enabled = false;
        }

        void ParticleSystem2d::updateWindmill(float dt)
        {
            if (!windmill.enabled)
                return;

            if (windmill.useFixedSpeed)
            {
                // 固定角速度模式：直接使用目标角速度
                windmill.angularVelocity = windmill.targetAngularVelocity;
            }
            else
            {
                // 动态驱动模式：使用力矩平滑以获得更稳定的旋转
                const float torqueSmoothing = 0.15f; // 平滑系数
                windmill.smoothedTorque = windmill.smoothedTorque * (1.0f - torqueSmoothing) + windmill.torque * torqueSmoothing;

                // 应用平滑后的力矩产生角加速度
                if (windmill.momentOfInertia > 1e-6f)
                {
                    float angularAccel = windmill.smoothedTorque / windmill.momentOfInertia;
                    windmill.angularVelocity += angularAccel * dt;
                }

                // 应用阻尼
                windmill.angularVelocity *= (1.0f - windmill.angularDamping * dt);

                // 限制最大角速度以保持稳定性
                const float maxAngularVelocity = 8.0f;
                if (windmill.angularVelocity > maxAngularVelocity)
                    windmill.angularVelocity = maxAngularVelocity;
                else if (windmill.angularVelocity < -maxAngularVelocity)
                    windmill.angularVelocity = -maxAngularVelocity;
            }

            // 更新角度
            windmill.angle += windmill.angularVelocity * dt;
            const float twoPi = glm::two_pi<float>();
            if (windmill.angle > twoPi || windmill.angle < -twoPi)
            {
                windmill.angle = std::fmod(windmill.angle, twoPi);
            }

            // 重置力矩（在下一次碰撞检测中累积）
            windmill.torque = 0.0f;
        }
    }
}