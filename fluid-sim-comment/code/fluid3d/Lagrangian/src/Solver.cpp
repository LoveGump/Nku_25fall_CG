/**
 * Solver.cpp: 3D拉格朗日流体求解器实现文件
 * 实现基于粒子的3D流体仿真算法 (SPH - Smoothed Particle Hydrodynamics)
 */

#include "fluid3d/Lagrangian/include/Solver.h"
#include "Global.h"
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <glm/gtc/constants.hpp>

namespace FluidSimulation
{

	namespace Lagrangian3d
	{
		// ========================================================================
		// 核函数 (Kernel Functions)
		// SPH方法的核心，用于将离散粒子的属性平滑插值到连续空间
		// 3D版本的核函数系数与2D不同
		// ========================================================================

		/**
		 * Poly6核函数 (3D版本)
		 * 特点：在r=0处非零，但在r=0处梯度为0。
		 * 用途：主要用于计算密度，因为它在中心处平滑，适合标量插值。
		 * 公式：W(r, h) = (315 / (64 * pi * h^9)) * (h^2 - r^2)^3
		 * @param r2 距离平方
		 * @param h2 支持半径平方
		 * @param h9 支持半径的9次方
		 * @return 核函数值
		 */
		float Poly6_3d(float r2, float h2, float h9)
		{
			if (r2 > h2)
				return 0.0f;
			float term = h2 - r2;
			return (315.0f / (64.0f * glm::pi<float>() * h9)) * term * term * term;
		}

		/**
		 * Spiky核函数的梯度 (3D版本)
		 * 特点：在r=0处梯度不为0，随着r增加线性减小。
		 * 用途：专门用于计算压力梯度力。
		 * 公式：Grad(W) = -(45 / (pi * h^6)) * (h - r)^2 * (r / |r|)
		 * @param r 位置向量差
		 * @param rLen 距离
		 * @param h 支持半径
		 * @param h6 支持半径的6次方
		 * @return 核函数梯度向量
		 */
		glm::vec3 SpikyGrad_3d(glm::vec3 r, float rLen, float h, float h6)
		{
			if (rLen > h || rLen <= 0.00001f)
				return glm::vec3(0.0f);
			float term = h - rLen;
			float coeff = -45.0f / (glm::pi<float>() * h6);
			return coeff * term * term * (r / rLen);
		}

		/**
		 * 粘性核函数的拉普拉斯算子 (3D版本)
		 * 特点：用于计算二阶导数（拉普拉斯量）。
		 * 用途：计算粘性力，模拟流体内部的摩擦。
		 * 公式：Lap(W) = (45 / (pi * h^6)) * (h - r)
		 * @param rLen 距离
		 * @param h 支持半径
		 * @param h6 支持半径的6次方
		 * @return 拉普拉斯值
		 */
		float ViscosityLap_3d(float rLen, float h, float h6)
		{
			if (rLen > h)
				return 0.0f;
			return (45.0f / (glm::pi<float>() * h6)) * (h - rLen);
		}

		/**
		 * 旋转木板碰撞检测
		 * 检测粒子与旋转木板的碰撞并处理响应
		 * @param p 粒子引用
		 * @param board 旋转木板引用
		 * @param particleRadius 粒子半径
		 */
		void ResolveRotatingBoardCollision(particle3d &p, RotatingBoard3d &board, float particleRadius)
		{
			if (!board.enabled)
				return;

			// 安全距离：粒子半径 + 小的容差
			float minDist = particleRadius + Lagrangian3dPara::eps;

			// 计算旋转矩阵（绕旋转轴）使用 Rodrigues 公式
			float cosA = std::cos(board.angle);
			float sinA = std::sin(board.angle);
			glm::vec3 axis = glm::normalize(board.rotationAxis);

			// 使用与渲染一致的旋转函数
			auto rotateVec = [&](const glm::vec3 &v) -> glm::vec3 {
				return v * cosA + glm::cross(axis, v) * sinA + axis * glm::dot(axis, v) * (1.0f - cosA);
			};

			// 计算旋转后的局部坐标轴
			glm::vec3 localX = rotateVec(glm::vec3(1, 0, 0));
			glm::vec3 localY = rotateVec(glm::vec3(0, 1, 0));
			glm::vec3 localZ = rotateVec(glm::vec3(0, 0, 1));

			// 将粒子位置转换到木板的局部坐标系
			glm::vec3 rel = p.position - board.center;
			// 投影到局部坐标轴上（相当于乘以旋转矩阵的逆/转置）
			glm::vec3 localPos;
			localPos.x = glm::dot(rel, localX);
			localPos.y = glm::dot(rel, localY);
			localPos.z = glm::dot(rel, localZ);

			// 木板的半尺寸（加上安全边距用于扩展检测范围）
			glm::vec3 halfSize = board.size * 0.5f;
			glm::vec3 expandedHalf = halfSize + glm::vec3(minDist);

			// 首先检查粒子是否在扩展的包围盒内
			if (std::abs(localPos.x) > expandedHalf.x ||
				std::abs(localPos.y) > expandedHalf.y ||
				std::abs(localPos.z) > expandedHalf.z)
			{
				return;  // 粒子距离木板太远，跳过
			}

			// 计算粒子到木板表面的最近点（在局部坐标系中）
			glm::vec3 closest;
			closest.x = glm::clamp(localPos.x, -halfSize.x, halfSize.x);
			closest.y = glm::clamp(localPos.y, -halfSize.y, halfSize.y);
			closest.z = glm::clamp(localPos.z, -halfSize.z, halfSize.z);

			glm::vec3 delta = localPos - closest;
			float dist2 = glm::dot(delta, delta);

			// 检测碰撞：粒子距离木板表面小于安全距离，或者粒子在木板内部
			bool isInside = (std::abs(localPos.x) <= halfSize.x &&
							 std::abs(localPos.y) <= halfSize.y &&
							 std::abs(localPos.z) <= halfSize.z);

			if (dist2 < minDist * minDist || isInside)
			{
				float dist = std::sqrt(dist2);
				glm::vec3 localNormal;
				
				if (dist > 1e-6f)
				{
					localNormal = delta / dist;
				}
				else
				{
					// 粒子在木板内部，找最近的面
					glm::vec3 penetration;
					penetration.x = halfSize.x - std::abs(localPos.x);
					penetration.y = halfSize.y - std::abs(localPos.y);
					penetration.z = halfSize.z - std::abs(localPos.z);

					if (penetration.x <= penetration.y && penetration.x <= penetration.z)
						localNormal = glm::vec3(localPos.x > 0 ? 1.0f : -1.0f, 0.0f, 0.0f);
					else if (penetration.y <= penetration.z)
						localNormal = glm::vec3(0.0f, localPos.y > 0 ? 1.0f : -1.0f, 0.0f);
					else
						localNormal = glm::vec3(0.0f, 0.0f, localPos.z > 0 ? 1.0f : -1.0f);
				}

				// 将法向量和最近点转换回世界坐标系
				glm::vec3 worldNormal = localNormal.x * localX + localNormal.y * localY + localNormal.z * localZ;
				glm::vec3 worldClosest = closest.x * localX + closest.y * localY + closest.z * localZ + board.center;

				// 计算木板在碰撞点的速度（由于旋转）
				glm::vec3 r_vec = worldClosest - board.center;
				glm::vec3 boardVel = board.angularVelocity * glm::cross(axis, r_vec);

				// 将粒子推出木板
				p.position = worldClosest + worldNormal * minDist;

				// 计算相对速度
				glm::vec3 relVel = p.velocity - boardVel;
				float relNormal = glm::dot(relVel, worldNormal);

				// 只在粒子朝向木板运动时处理碰撞
				if (relNormal < 0.0f)
				{
					glm::vec3 relVn = relNormal * worldNormal;
					glm::vec3 relVt = relVel - relVn;

					// 反弹（恢复系数）
					relVn = -board.restitution * relVn;
					// 摩擦
					relVt *= (1.0f - board.friction);

					p.velocity = boardVel + relVn + relVt;

					// 限制碰撞后速度
					float speed = glm::length(p.velocity);
					if (speed > Lagrangian3dPara::maxVelocity)
					{
						p.velocity = p.velocity / speed * Lagrangian3dPara::maxVelocity;
					}
				}
			}
		}

		/**
		 * 构造函数，保存粒子系统引用
		 * @param ps 粒子系统引用
		 */
		Solver::Solver(ParticleSystem3d &ps) : mPs(ps)
		{
			// 构造函数,保存粒子系统引用
		}

		/**
		 * 求解流体方程
		 * 实现一步3D粒子流体的仿真计算
		 * 采用 SPH (Smoothed Particle Hydrodynamics) 方法
		 */
		void Solver::solve()
		{
			// 获取仿真参数
			float dt = Lagrangian3dPara::dt / Lagrangian3dPara::substep; // 子步长
			float h = mPs.supportRadius;								 // 支持半径 (平滑长度)
			float h2 = h * h;
			float h6 = pow(h, 6);
			float h9 = pow(h, 9);
			float mass = Lagrangian3dPara::density * mPs.particleVolume; // 粒子质量
			float restDensity = Lagrangian3dPara::density;				 // 静止密度 (参考密度)
			float stiffness = Lagrangian3dPara::stiffness;				 // 刚度系数 (气体常数 k)
			float exponent = Lagrangian3dPara::exponent;				 // 状态方程指数 (gamma)
			float viscosity = Lagrangian3dPara::viscosity;				 // 粘度系数 (mu)

			// 重力方向：Z轴向下为负值
			glm::vec3 gravity = glm::vec3(
				Lagrangian3dPara::gravityX,
				Lagrangian3dPara::gravityY,
				-Lagrangian3dPara::gravityZ);

			// 子步迭代：为了保证数值稳定性，通常将一个时间步长拆分为多个小步
			for (int step = 0; step < Lagrangian3dPara::substep; ++step)
			{
				// 先更新木板角度，确保碰撞检测和渲染使用相同角度
				mPs.updateRotatingBoard(dt);

				// 邻域搜索 (为计算密度做准备)
				mPs.updateBlockInfo();

				// 1. 计算密度 - 使用SPH核函数计算每个粒子的密度
#pragma omp parallel for
				for (int i = 0; i < static_cast<int>(mPs.particles.size()); ++i)
				{
					particle3d &p = mPs.particles[i];
					p.density = 0.0f;

					// 获取当前粒子所在的网格坐标 (3D)
					int bZ = p.blockId / (mPs.blockNum.x * mPs.blockNum.y);
					int bRem = p.blockId % (mPs.blockNum.x * mPs.blockNum.y);
					int bY = bRem / mPs.blockNum.x;
					int bX = bRem % mPs.blockNum.x;

					// 遍历相邻的27个网格 (3x3x3区域)
					for (int z = -1; z <= 1; ++z)
					{
						for (int y = -1; y <= 1; ++y)
						{
							for (int x = -1; x <= 1; ++x)
							{
								int nZ = bZ + z;
								int nY = bY + y;
								int nX = bX + x;
								// 边界检查
								if (nX >= 0 && nX < static_cast<int>(mPs.blockNum.x) &&
									nY >= 0 && nY < static_cast<int>(mPs.blockNum.y) &&
									nZ >= 0 && nZ < static_cast<int>(mPs.blockNum.z))
								{
									uint32_t nBlockId = nZ * mPs.blockNum.x * mPs.blockNum.y + nY * mPs.blockNum.x + nX;
									glm::uvec2 range = mPs.blockExtens[nBlockId];
									// 遍历该网格内的所有粒子
									for (uint32_t j = range.x; j < range.y; ++j)
									{
										const particle3d &pj = mPs.particles[j];
										glm::vec3 r = p.position - pj.position;
										float r2 = glm::dot(r, r);
										// 仅当距离小于支持半径时才计算贡献
										if (r2 < h2)
										{
											p.density += mass * Poly6_3d(r2, h2, h9);
										}
									}
								}
							}
						}
					}
					// 避免后续除以零，并防止密度过低导致的数值问题
					p.density = (std::max)(p.density, restDensity);
				}

				// 2. 计算压力 - 根据密度计算压力
#pragma omp parallel for
				for (int i = 0; i < static_cast<int>(mPs.particles.size()); ++i)
				{
					particle3d &p = mPs.particles[i];

					// 使用Tait状态方程计算压力
					p.pressure = (std::max)(0.0f, stiffness * (pow(p.density / restDensity, exponent) - 1.0f));

					// 预计算 p / rho^2，这在压力梯度力公式中会用到
					p.pressDivDens2 = p.pressure / (p.density * p.density);
				}

				// 3. 计算加速度 - 计算压力梯度和粘性力
#pragma omp parallel for
				for (int i = 0; i < static_cast<int>(mPs.particles.size()); ++i)
				{
					particle3d &p = mPs.particles[i];
					glm::vec3 pressureForce(0.0f);
					glm::vec3 viscosityForce(0.0f);

					// 获取当前粒子所在的网格坐标 (3D)
					int bZ = p.blockId / (mPs.blockNum.x * mPs.blockNum.y);
					int bRem = p.blockId % (mPs.blockNum.x * mPs.blockNum.y);
					int bY = bRem / mPs.blockNum.x;
					int bX = bRem % mPs.blockNum.x;

					// 再次遍历邻居粒子计算相互作用力
					for (int z = -1; z <= 1; ++z)
					{
						for (int y = -1; y <= 1; ++y)
						{
							for (int x = -1; x <= 1; ++x)
							{
								int nZ = bZ + z;
								int nY = bY + y;
								int nX = bX + x;
								if (nX >= 0 && nX < static_cast<int>(mPs.blockNum.x) &&
									nY >= 0 && nY < static_cast<int>(mPs.blockNum.y) &&
									nZ >= 0 && nZ < static_cast<int>(mPs.blockNum.z))
								{
									uint32_t nBlockId = nZ * mPs.blockNum.x * mPs.blockNum.y + nY * mPs.blockNum.x + nX;
									glm::uvec2 range = mPs.blockExtens[nBlockId];
									for (uint32_t j = range.x; j < range.y; ++j)
									{
										if (static_cast<int>(j) == i)
											continue; // 跳过自己
										const particle3d &pj = mPs.particles[j];
										glm::vec3 r = p.position - pj.position;
										float rLen = glm::length(r);

										if (rLen < h)
										{
											// 压力梯度力
											// F_pressure = -sum_j m_j * (p_i/rho_i^2 + p_j/rho_j^2) * grad(W)
											pressureForce -= mass * mass * (p.pressDivDens2 + pj.pressDivDens2) * SpikyGrad_3d(r, rLen, h, h6);

											// 粘性力
											// F_viscosity = mu * sum_j m_j * (v_j - v_i) / rho_j * Lap(W)
											viscosityForce += viscosity * mass * mass * (1.0f / pj.density) * (pj.velocity - p.velocity) * ViscosityLap_3d(rLen, h, h6);
										}
									}
								}
							}
						}
					}

					// 合力 = 压力力 + 粘性力
					glm::vec3 totalForce = pressureForce + viscosityForce;
					// 牛顿第二定律：F = ma => a = F/m + g
					p.accleration = totalForce / mass + gravity;
				}

				// 4. 更新速度和位置 - 使用欧拉积分更新粒子状态
#pragma omp parallel for
				for (int i = 0; i < static_cast<int>(mPs.particles.size()); ++i)
				{
					particle3d &p = mPs.particles[i];

					// 半隐式欧拉积分：先更新速度，再更新位置
					p.velocity += p.accleration * dt;

					// 限制最大速度 (CFL条件)
					float vLen = glm::length(p.velocity);
					if (vLen > Lagrangian3dPara::maxVelocity)
					{
						p.velocity = p.velocity / vLen * Lagrangian3dPara::maxVelocity;
					}

					// XSPH 人工粘性 - 平滑速度场
					glm::vec3 v_xsph = p.velocity;
					float xsphFactor = 0.1f; // 混合系数 [0.0, 0.5]

					// 获取当前粒子所在的网格坐标 (3D)
					int bZ = p.blockId / (mPs.blockNum.x * mPs.blockNum.y);
					int bRem = p.blockId % (mPs.blockNum.x * mPs.blockNum.y);
					int bY = bRem / mPs.blockNum.x;
					int bX = bRem % mPs.blockNum.x;

					// 计算 XSPH 修正项
					for (int z = -1; z <= 1; ++z)
					{
						for (int y = -1; y <= 1; ++y)
						{
							for (int x = -1; x <= 1; ++x)
							{
								int nZ = bZ + z;
								int nY = bY + y;
								int nX = bX + x;
								if (nX >= 0 && nX < static_cast<int>(mPs.blockNum.x) &&
									nY >= 0 && nY < static_cast<int>(mPs.blockNum.y) &&
									nZ >= 0 && nZ < static_cast<int>(mPs.blockNum.z))
								{
									uint32_t nBlockId = nZ * mPs.blockNum.x * mPs.blockNum.y + nY * mPs.blockNum.x + nX;
									glm::uvec2 range = mPs.blockExtens[nBlockId];
									for (uint32_t j = range.x; j < range.y; ++j)
									{
										if (static_cast<int>(j) == i)
											continue;
										const particle3d &pj = mPs.particles[j];
										glm::vec3 r = p.position - pj.position;
										float r2 = glm::dot(r, r);
										if (r2 < h2)
										{
											v_xsph += xsphFactor * (mass / pj.density) * (pj.velocity - p.velocity) * Poly6_3d(r2, h2, h9);
										}
									}
								}
							}
						}
					}

					// 使用修正后的速度更新位置
					p.position += v_xsph * dt;

					// 5. 边界检查 - 处理粒子与边界的碰撞
					// 使用更柔和的边界处理，避免角落粒子爆炸
					float boundaryDamping = Lagrangian3dPara::velocityAttenuation;
					float boundaryEps = Lagrangian3dPara::eps;

					// X轴边界
					if (p.position.x < mPs.lowerBound.x + boundaryEps)
					{
						p.position.x = mPs.lowerBound.x + boundaryEps;
						if (p.velocity.x < 0.0f)
							p.velocity.x = -p.velocity.x * boundaryDamping;
					}
					else if (p.position.x > mPs.upperBound.x - boundaryEps)
					{
						p.position.x = mPs.upperBound.x - boundaryEps;
						if (p.velocity.x > 0.0f)
							p.velocity.x = -p.velocity.x * boundaryDamping;
					}

					// Y轴边界
					if (p.position.y < mPs.lowerBound.y + boundaryEps)
					{
						p.position.y = mPs.lowerBound.y + boundaryEps;
						if (p.velocity.y < 0.0f)
							p.velocity.y = -p.velocity.y * boundaryDamping;
					}
					else if (p.position.y > mPs.upperBound.y - boundaryEps)
					{
						p.position.y = mPs.upperBound.y - boundaryEps;
						if (p.velocity.y > 0.0f)
							p.velocity.y = -p.velocity.y * boundaryDamping;
					}

					// Z轴边界
					if (p.position.z < mPs.lowerBound.z + boundaryEps)
					{
						p.position.z = mPs.lowerBound.z + boundaryEps;
						if (p.velocity.z < 0.0f)
							p.velocity.z = -p.velocity.z * boundaryDamping;
					}
					else if (p.position.z > mPs.upperBound.z - boundaryEps)
					{
						p.position.z = mPs.upperBound.z - boundaryEps;
						if (p.velocity.z > 0.0f)
							p.velocity.z = -p.velocity.z * boundaryDamping;
					}

					// 限制边界处的最大速度，防止角落粒子积累过大能量
					float speed = glm::length(p.velocity);
					if (speed > Lagrangian3dPara::maxVelocity)
					{
						p.velocity = p.velocity / speed * Lagrangian3dPara::maxVelocity;
					}

					// 处理旋转木板碰撞 - 多次迭代确保不穿透
					for (int iter = 0; iter < 3; ++iter)
					{
						ResolveRotatingBoardCollision(p, mPs.rotatingBoard, mPs.particleRadius);
					}

					// 6. 更新块ID - 更新粒子的空间索引
					p.blockId = mPs.getBlockIdByPosition(p.position);
				}
			}
		}
	}
}