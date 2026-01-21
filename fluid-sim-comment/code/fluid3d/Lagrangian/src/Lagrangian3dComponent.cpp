/**
 * Lagrangian3dComponent.cpp: 3D拉格朗日流体组件实现文件
 * 实现组件的初始化、仿真和渲染功能
 */

#include "Lagrangian3dComponent.h"

namespace FluidSimulation
{
    namespace Lagrangian3d
    {
        /**
         * 关闭组件，释放资源
         */
        void Lagrangian3dComponent::shutDown()
        {
            delete renderer, solver, ps;
            renderer = NULL;
            solver = NULL;
            ps = NULL;
        }

        /**
         * 初始化组件
         * 创建渲染器、粒子系统和求解器
         */
        void Lagrangian3dComponent::init()
        {
            if (renderer != NULL || solver != NULL || ps != NULL)
            {
                shutDown();
            }
            Glb::Timer::getInstance().clear();

            renderer = new Renderer();
            renderer->init();

            ps = new ParticleSystem3d();
            ps->setContainerSize(glm::vec3(0.0, 0.0, 0.0), glm::vec3(1, 1, 1));
            
            for (int i = 0; i < Lagrangian3dPara::fluidBlocks.size(); i++) {
                ps->addFluidBlock(Lagrangian3dPara::fluidBlocks[i].lowerCorner, Lagrangian3dPara::fluidBlocks[i].upperCorner,
                    Lagrangian3dPara::fluidBlocks[i].initVel, Lagrangian3dPara::fluidBlocks[i].particleSpace);
            }

            // 设置旋转木板
            if (Lagrangian3dPara::enableRotatingBoard)
            {
                ps->setRotatingBoard(
                    Lagrangian3dPara::boardCenter,
                    Lagrangian3dPara::boardSize,
                    Lagrangian3dPara::boardRotationAxis,
                    Lagrangian3dPara::boardAngularVelocity,
                    Lagrangian3dPara::boardRestitution,
                    Lagrangian3dPara::boardFriction
                );
            }

            ps->updateBlockInfo();
            Glb::Logger::getInstance().addLog("3d Particle System initialized. particle num: " + std::to_string(ps->particles.size()));

            solver = new Solver(*ps);
        }

        void Lagrangian3dComponent::simulate()
        {
            // solve() 内部已经有 substep 循环，这里不需要重复
            ps->updateBlockInfo();
            solver->solve();
        }

        GLuint Lagrangian3dComponent::getRenderedTexture()
        {
            renderer->draw(*ps);
            return renderer->getRenderedTexture();
        }
    }
}
