/**
 * Solver.cpp: 2D欧拉流体求解器实现文件
 * 实现流体仿真的核心算法
 */

#include "Eulerian/include/Solver.h"
#include "Configure.h"
#include <vector>
#include <cmath>
#include <algorithm>

namespace FluidSimulation
{
    namespace Eulerian2d
    {
        /**
         * 构造函数，初始化求解器并重置网格
         * @param grid MAC网格引用
         */
        Solver::Solver(MACGrid2d &grid) : mGrid(grid)
        {
            mGrid.reset();
        }

        /**
         * 求解流体方程
         * 实现一步流体仿真的主要步骤
         */
        void Solver::solve()
        {
            double dt = Eulerian2dPara::dt;
            double dx = mGrid.cellSize;
            double rho = Eulerian2dPara::airDensity;

            // 1. 平流(advection) - 将物理量沿速度场平流
            // 使用半拉格朗日方法(Semi-Lagrangian)求解平流方程: q_new(x) = q_old(x - u * dt)
            // 该方法无条件稳定，但会引入数值耗散
            Glb::GridData2dX targetU;
            targetU.initialize();
            Glb::GridData2dY targetV;
            targetV.initialize();
            Glb::CubicGridData2d targetD;
            targetD.initialize();
            Glb::CubicGridData2d targetT;
            targetT.initialize();

            // Advect U (水平速度分量)
            // U定义在垂直网格线的中心 (i, j+0.5)
            for (int j = 0; j < mGrid.dim[1]; j++)
            {
                for (int i = 0; i < mGrid.dim[0] + 1; i++)
                {
                    // 找到当前U分量的世界坐标位置
                    glm::vec2 pos(i * dx, (j + 0.5) * dx);
                    glm::vec2 oldPos = pos;
                    // 回溯寻找上一时刻的位置
                    pos = mGrid.semiLagrangian(pos, dt);
                    // 处理固体边界：如果回溯点在固体内部，则使用原位置（简单处理）
                    if (mGrid.inSolid(pos))
                        pos = oldPos;
                    // 插值获取上一时刻该位置的速度值
                    targetU(i, j) = mGrid.getVelocityX(pos);
                }
            }

            // Advect V (垂直速度分量)
            // V定义在水平网格线的中心 (i+0.5, j)
            for (int j = 0; j < mGrid.dim[1] + 1; j++)
            {
                for (int i = 0; i < mGrid.dim[0]; i++)
                {
                    glm::vec2 pos((i + 0.5) * dx, j * dx);
                    glm::vec2 oldPos = pos;
                    pos = mGrid.semiLagrangian(pos, dt);
                    if (mGrid.inSolid(pos))
                        pos = oldPos;
                    targetV(i, j) = mGrid.getVelocityY(pos);
                }
            }

            // Advect Density (密度场)
            // 密度定义在网格中心 (i+0.5, j+0.5)
            for (int j = 0; j < mGrid.dim[1]; j++)
            {
                for (int i = 0; i < mGrid.dim[0]; i++)
                {
                    glm::vec2 pos((i + 0.5) * dx, (j + 0.5) * dx);
                    glm::vec2 oldPos = pos;
                    pos = mGrid.semiLagrangian(pos, dt);
                    if (mGrid.inSolid(pos))
                        pos = oldPos;
                    targetD(i, j) = mGrid.getDensity(pos);
                }
            }

            // Advect Temperature (温度场)
            // 温度定义在网格中心 (i+0.5, j+0.5)
            for (int j = 0; j < mGrid.dim[1]; j++)
            {
                for (int i = 0; i < mGrid.dim[0]; i++)
                {
                    glm::vec2 pos((i + 0.5) * dx, (j + 0.5) * dx);
                    glm::vec2 oldPos = pos;
                    pos = mGrid.semiLagrangian(pos, dt);
                    if (mGrid.inSolid(pos))
                        pos = oldPos;
                    targetT(i, j) = mGrid.getTemperature(pos);
                }
            }

            // 更新网格数据
            mGrid.mU = targetU;
            mGrid.mV = targetV;
            mGrid.mD = targetD;
            mGrid.mT = targetT;

            // 2. 计算外力(external forces) - 添加重力、浮力等外力
            // 这里主要计算基于Boussinesq近似的浮力
            // 浮力与密度差和温度差有关
            for (int j = 0; j < mGrid.dim[1] + 1; j++)
            {
                for (int i = 0; i < mGrid.dim[0]; i++)
                {
                    if (mGrid.isSolidFace(i, j, MACGrid2d::Y))
                        continue;
                    glm::vec2 pos((i + 0.5) * dx, j * dx);
                    mGrid.mV(i, j) += dt * mGrid.getBoussinesqForce(pos);
                }
            }

            // 3. 投影(projection) - 求解压力场，使速度场无散(Divergence-free)
            // 求解压力泊松方程: Laplacian(p) = rho / dt * Divergence(u*)
            int width = mGrid.dim[0];
            int height = mGrid.dim[1];
            int n = width * height;
            std::vector<double> p(n, 0.0); // 压力解向量
            std::vector<double> b(n, 0.0); // 方程右端项
            std::vector<double> r(n, 0.0); // 残差向量
            std::vector<double> z(n, 0.0); // 预处理后的残差
            std::vector<double> s(n, 0.0); // 搜索方向

            double scale = rho * dx * dx / dt;

            // Build RHS (构建方程右端项 b)
            // b = -div(u*) * (rho * dx^2 / dt)
            for (int j = 0; j < height; j++)
            {
                for (int i = 0; i < width; i++)
                {
                    if (mGrid.isSolidCell(i, j))
                        continue;
                    double div = mGrid.getDivergence(i, j);
                    b[j * width + i] = -div * scale;
                }
            }

            // PCG Solver (预处理共轭梯度法求解线性方程组 Ap = b)
            r = b;
            z = r; // Identity preconditioner (单位预处理，即无预处理)
            s = z;

            double sigma = 0.0;
            for (double val : z)
                sigma += val * val; // dot(z, r)

            int maxIter = 200;
            double tol = 1e-5;

            for (int iter = 0; iter < maxIter; iter++)
            {
                if (sqrt(sigma) < tol)
                    break;

                // compute As (计算矩阵向量乘积 A * s)
                // A 是离散拉普拉斯算子矩阵
                std::vector<double> As(n, 0.0);
                for (int j = 0; j < height; j++)
                {
                    for (int i = 0; i < width; i++)
                    {
                        int idx = j * width + i;
                        if (mGrid.isSolidCell(i, j))
                            continue;

                        double val = 0.0;
                        // Diagonal (对角线元素)
                        val += mGrid.getPressureCoeffBetweenCells(i, j, i, j) * s[idx];

                        // Neighbors (邻居元素)
                        int neighbors[4][2] = {{i + 1, j}, {i - 1, j}, {i, j + 1}, {i, j - 1}};
                        for (auto &nb : neighbors)
                        {
                            int ni = nb[0];
                            int nj = nb[1];
                            if (ni >= 0 && ni < width && nj >= 0 && nj < height)
                            {
                                double coeff = mGrid.getPressureCoeffBetweenCells(i, j, ni, nj);
                                if (coeff != 0.0)
                                {
                                    val += coeff * s[nj * width + ni];
                                }
                            }
                        }
                        As[idx] = val;
                    }
                }

                double denom = 0.0;
                for (int k = 0; k < n; ++k)
                    denom += s[k] * As[k];
                if (fabs(denom) < 1e-10)
                    break;
                double alpha = sigma / denom;

                // 更新压力 p 和残差 r
                for (int k = 0; k < n; ++k)
                {
                    p[k] += alpha * s[k];
                    r[k] -= alpha * As[k];
                }

                z = r; // Identity preconditioner

                double sigmaNew = 0.0;
                for (int k = 0; k < n; ++k)
                    sigmaNew += z[k] * r[k];

                double beta = sigmaNew / sigma;
                sigma = sigmaNew;

                // 更新搜索方向 s
                for (int k = 0; k < n; ++k)
                {
                    s[k] = z[k] + beta * s[k];
                }
            }

            // Update Velocity (根据压力梯度修正速度)
            // u_new = u* - (dt / rho) * grad(p)

            // 更新 U 分量
            for (int j = 0; j < height; j++)
            {
                for (int i = 0; i < width + 1; i++)
                {
                    if (mGrid.isSolidFace(i, j, MACGrid2d::X))
                    {
                        mGrid.mU(i, j) = 0;
                        continue;
                    }
                    if (i > 0 && i < width)
                    {
                        double p_curr = p[j * width + i];
                        double p_prev = p[j * width + i - 1];
                        // 减去压力梯度
                        mGrid.mU(i, j) -= (dt / rho) * (p_curr - p_prev) / dx;
                    }
                }
            }

            // 更新 V 分量
            for (int j = 0; j < height + 1; j++)
            {
                for (int i = 0; i < width; i++)
                {
                    if (mGrid.isSolidFace(i, j, MACGrid2d::Y))
                    {
                        mGrid.mV(i, j) = 0;
                        continue;
                    }
                    if (j > 0 && j < height)
                    {
                        double p_curr = p[j * width + i];
                        double p_prev = p[(j - 1) * width + i];
                        // 减去压力梯度
                        mGrid.mV(i, j) -= (dt / rho) * (p_curr - p_prev) / dx;
                    }
                }
            }
        }
    }
}
