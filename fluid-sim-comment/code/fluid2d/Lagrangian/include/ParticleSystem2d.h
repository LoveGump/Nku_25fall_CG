/**
 * ParticleSystem2d.h: 2D粒子系统头文件
 * 定义基于粒子的2D流体仿真数据结构
 */

#pragma once
#ifndef __PARTICAL_SYSTEM_2D_H__
#define __PARTICAL_SYSTEM_2D_H__

#include <vector>
#include <list>
#include <glm/glm.hpp>
#include "Global.h"

#include "Configure.h"

namespace FluidSimulation
{

    namespace Lagrangian2d
    {
        /**
         * 2D粒子信息结构体
         * 存储粒子的物理属性
         */
        struct ParticleInfo2d
        {
            alignas(8) glm::vec2 position;     // 位置
            alignas(8) glm::vec2 prevPosition; // previous position
            alignas(8) glm::vec2 velocity;     // 速度
            alignas(8) glm::vec2 accleration;  // 加速度
            alignas(4) float density;          // 密度
            alignas(4) float pressure;         // 压力
            alignas(4) float pressDivDens2;    // 压力除以密度的平方
            alignas(4) uint32_t blockId;       // 所属流体块ID
        };

        // Moving solid (kinematic obstacle)
        struct MovingSolid2d
        {
            bool enabled = false;
            glm::vec2 position = glm::vec2(0.0f);
            glm::vec2 velocity = glm::vec2(0.0f);
            float radius = 0.0f;
            float restitution = 0.0f;
            float friction = 0.0f;
        };

        // Rotating windmill obstacle
        struct Windmill2d
        {
            bool enabled = false;
            glm::vec2 center = glm::vec2(0.0f);
            float bladeLength = 0.0f;
            float bladeWidth = 0.0f;
            float hubRadius = 0.0f;
            int bladeCount = 4;
            float angularVelocity = 0.0f;
            float angle = 0.0f;
            float restitution = 0.0f;
            float friction = 0.0f;

            // 物理参数用于被流体驱动
            float momentOfInertia = 1.0f;       // 转动惯量
            float angularDamping = 0.5f;        // 角速度阻尼
            float torque = 0.0f;                // 当前力矩
            float smoothedTorque = 0.0f;        // 平滑后的力矩
            bool useFixedSpeed = false;         // 是否使用固定角速度
            float targetAngularVelocity = 2.0f; // 目标角速度
        };

        /**
         * 2D粒子系统类
         * 管理所有粒子的物理属性和空间索引
         */
        class ParticleSystem2d
        {
        public:
            ParticleSystem2d();
            ~ParticleSystem2d();

            void setContainerSize(glm::vec2 containerCorner, glm::vec2 containerSize);
            int32_t addFluidBlock(glm::vec2 corner, glm::vec2 size, glm::vec2 v0, float particleSpace);
            uint32_t getBlockIdByPosition(glm::vec2 position);
            void updateBlockInfo();
            void setMovingSolid(glm::vec2 center, float radius, glm::vec2 velocity, float restitution, float friction);
            void disableMovingSolid();
            void updateMovingSolid(float dt);
            void setWindmill(glm::vec2 center, float bladeLength, float bladeWidth, float hubRadius, int bladeCount, float angularVelocity, float restitution, float friction);
            void disableWindmill();
            void updateWindmill(float dt);

        public:
            // 粒子参数
            float supportRadius = Lagrangian2dPara::supportRadius;
            float supportRadius2 = supportRadius * supportRadius;
            float particleRadius = Lagrangian2dPara::particleRadius;
            float particleDiameter = Lagrangian2dPara::particleDiameter;
            float particleVolume = particleDiameter * particleDiameter;

            // 存储全部粒子信息
            std::vector<ParticleInfo2d> particles;

            // 容器参数
            glm::vec2 lowerBound = glm::vec2(FLT_MAX);
            glm::vec2 upperBound = glm::vec2(-FLT_MAX);
            glm::vec2 containerCenter = glm::vec2(0.0f);
            float scale = Lagrangian2dPara::scale;

            // Moving solid
            MovingSolid2d movingSolid;

            // Windmill
            Windmill2d windmill;

            // Block结构（加速临近搜索）
            glm::uvec2 blockNum = glm::uvec2(0);
            glm::vec2 blockSize = glm::vec2(0.0f);
            std::vector<glm::uvec2> blockExtens;
            std::vector<int32_t> blockIdOffs;
        };
    }
}

#endif // !PARTICAL_SYSTEM_H
