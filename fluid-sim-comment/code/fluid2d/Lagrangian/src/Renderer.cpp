#include "Lagrangian/include/Renderer.h"

#include <iostream>
#include <fstream>
#include <cmath>
#include <glm/gtc/constants.hpp>
#include "Configure.h"

// 这是渲染器的主要实现,涉及OpenGL函数的使用
// 在阅读此文件之前,请确保您对OpenGL有基本了解

namespace FluidSimulation
{

    namespace Lagrangian2d
    {

        Renderer::Renderer()
        {
        }

        // 初始化渲染器
        int32_t Renderer::init()
        {
            extern std::string shaderPath;

            // 加载并编译粒子着色器
            std::string particleVertShaderPath = shaderPath + "/DrawParticles2d.vert";
            std::string particleFragShaderPath = shaderPath + "/DrawParticles2d.frag";
            shader = new Glb::Shader();
            shader->buildFromFile(particleVertShaderPath, particleFragShaderPath);

            std::string solidVertShaderPath = shaderPath + "/DrawSolid2d.vert";
            std::string solidFragShaderPath = shaderPath + "/DrawSolid2d.frag";
            solidShader = new Glb::Shader();
            solidShader->buildFromFile(solidVertShaderPath, solidFragShaderPath);

            // 生成顶点数组对象(VAO)
            glGenVertexArrays(1, &VAO);
            // 生成位置的顶点缓冲对象(VBO)
            glGenBuffers(1, &positionVBO);
            // 生成密度的顶点缓冲对象(VBO)
            glGenBuffers(1, &densityVBO);

            glGenVertexArrays(1, &solidVAO);
            glGenBuffers(1, &solidVBO);

            // 生成帧缓冲对象(FBO)
            glGenFramebuffers(1, &FBO);
            // 绑定帧缓冲
            glBindFramebuffer(GL_FRAMEBUFFER, FBO);

            // 生成并设置纹理
            glGenTextures(1, &textureID);
            glBindTexture(GL_TEXTURE_2D, textureID);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imageWidth, imageHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
            glBindTexture(GL_TEXTURE_2D, 0);

            // 将纹理附加到帧缓冲
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureID, 0);

            // 生成渲染缓冲对象(RBO)
            glGenRenderbuffers(1, &RBO);
            glBindRenderbuffer(GL_RENDERBUFFER, RBO);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, imageWidth, imageHeight);
            glBindRenderbuffer(GL_RENDERBUFFER, 0);

            // 将RBO附加到帧缓冲
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, RBO);
            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            {
                Glb::Logger::getInstance().addLog("Error: Framebuffer is not complete!");
            }
            glBindFramebuffer(GL_FRAMEBUFFER, 0);

            // 设置视口大小
            glViewport(0, 0, imageWidth, imageHeight);

            return 0;
        }

        // 绘制粒子系统
        void Renderer::draw(ParticleSystem2d &ps)
        {
            // 绑定VAO
            glBindVertexArray(VAO);

            // 绑定并更新位置VBO
            glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
            glBufferData(GL_ARRAY_BUFFER, ps.particles.size() * sizeof(ParticleInfo2d), ps.particles.data(), GL_STATIC_DRAW);

            // 设置位置属性
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(ParticleInfo2d), (void *)offsetof(ParticleInfo2d, position));
            glEnableVertexAttribArray(0);

            // 设置密度属性
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(ParticleInfo2d), (void *)offsetof(ParticleInfo2d, density));
            glEnableVertexAttribArray(1);

            glBindVertexArray(0);

            // 保存粒子数量
            particleNum = ps.particles.size();

            // 绑定帧缓冲开始渲染
            glBindFramebuffer(GL_FRAMEBUFFER, FBO);

            // 清空缓冲区
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glEnable(GL_DEPTH_TEST);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // 准备渲染
            glBindVertexArray(VAO);
            shader->use();
            shader->setFloat("scale", ps.scale);

            // 启用点精灵
            glEnable(GL_PROGRAM_POINT_SIZE);

            // 绘制所有粒子
            glDrawArrays(GL_POINTS, 0, particleNum);

            if (ps.movingSolid.enabled)
            {
                const int segments = 48;
                const float step = glm::two_pi<float>() / static_cast<float>(segments);
                std::vector<glm::vec2> vertices;
                vertices.reserve(segments + 2);
                vertices.push_back(ps.movingSolid.position);
                for (int i = 0; i <= segments; ++i)
                {
                    float angle = step * static_cast<float>(i);
                    glm::vec2 dir(std::cos(angle), std::sin(angle));
                    vertices.push_back(ps.movingSolid.position + dir * ps.movingSolid.radius);
                }

                solidVertexCount = vertices.size();

                glBindVertexArray(solidVAO);
                glBindBuffer(GL_ARRAY_BUFFER, solidVBO);
                glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec2), vertices.data(), GL_DYNAMIC_DRAW);
                glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void *)0);
                glEnableVertexAttribArray(0);

                glDisable(GL_DEPTH_TEST);
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

                solidShader->use();
                solidShader->setFloat("scale", ps.scale);
                solidShader->setVec4("color", glm::vec4(0.2f, 0.2f, 0.2f, 0.85f));

                glDrawArrays(GL_TRIANGLE_FAN, 0, static_cast<GLsizei>(solidVertexCount));

                glDisable(GL_BLEND);
                glEnable(GL_DEPTH_TEST);
                glBindVertexArray(0);
            }
            if (ps.windmill.enabled)
            {
                int bladeCount = ps.windmill.bladeCount;
                if (bladeCount < 1)
                    bladeCount = 1;

                float halfLength = ps.windmill.bladeLength * 0.5f;
                float halfWidth = ps.windmill.bladeWidth * 0.5f;
                float angleStep = glm::two_pi<float>() / static_cast<float>(bladeCount);
                // 叶片偏移量：从轮毂外缘开始
                float bladeOffset = ps.windmill.hubRadius + halfLength;

                std::vector<glm::vec2> vertices;
                vertices.reserve(bladeCount * 6 + 100);

                // 绘制叶片
                for (int blade = 0; blade < bladeCount; ++blade)
                {
                    float angle = ps.windmill.angle + angleStep * static_cast<float>(blade);
                    glm::vec2 dir(std::cos(angle), std::sin(angle));
                    glm::vec2 perp(-dir.y, dir.x);
                    glm::vec2 bladeCenter = ps.windmill.center + dir * bladeOffset;

                    glm::vec2 c0 = bladeCenter + dir * halfLength + perp * halfWidth;
                    glm::vec2 c1 = bladeCenter - dir * halfLength + perp * halfWidth;
                    glm::vec2 c2 = bladeCenter - dir * halfLength - perp * halfWidth;
                    glm::vec2 c3 = bladeCenter + dir * halfLength - perp * halfWidth;

                    vertices.push_back(c0);
                    vertices.push_back(c1);
                    vertices.push_back(c2);
                    vertices.push_back(c0);
                    vertices.push_back(c2);
                    vertices.push_back(c3);
                }

                glBindVertexArray(solidVAO);
                glBindBuffer(GL_ARRAY_BUFFER, solidVBO);
                glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec2), vertices.data(), GL_DYNAMIC_DRAW);
                glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void *)0);
                glEnableVertexAttribArray(0);

                glDisable(GL_DEPTH_TEST);
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

                solidShader->use();
                solidShader->setFloat("scale", ps.scale);

                // 绘制叶片，使用深灰色
                solidShader->setVec4("color", glm::vec4(0.2f, 0.2f, 0.22f, 0.9f));
                glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(bladeCount * 6));

                // 绘制支撑轴（从中心到底部的支柱）
                float supportWidth = 0.03f * ps.scale;
                float supportLength = (ps.windmill.center.y - (-1.0f * ps.scale)) * 0.95f; // 支柱长度
                if (supportLength > 0.0f)
                {
                    glm::vec2 supportBottom = ps.windmill.center - glm::vec2(0.0f, supportLength);
                    vertices.clear();
                    vertices.push_back(ps.windmill.center + glm::vec2(-supportWidth, 0.0f));
                    vertices.push_back(ps.windmill.center + glm::vec2(supportWidth, 0.0f));
                    vertices.push_back(supportBottom + glm::vec2(supportWidth, 0.0f));
                    vertices.push_back(ps.windmill.center + glm::vec2(-supportWidth, 0.0f));
                    vertices.push_back(supportBottom + glm::vec2(supportWidth, 0.0f));
                    vertices.push_back(supportBottom + glm::vec2(-supportWidth, 0.0f));

                    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec2), vertices.data(), GL_DYNAMIC_DRAW);
                    solidShader->setVec4("color", glm::vec4(0.15f, 0.15f, 0.17f, 0.95f));
                    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertices.size()));

                    // 绘制底座（支柱底部的加宽部分）
                    float baseWidth = supportWidth * 2.5f;
                    float baseHeight = 0.04f * ps.scale;
                    vertices.clear();
                    vertices.push_back(supportBottom + glm::vec2(-baseWidth, 0.0f));
                    vertices.push_back(supportBottom + glm::vec2(baseWidth, 0.0f));
                    vertices.push_back(supportBottom + glm::vec2(baseWidth, -baseHeight));
                    vertices.push_back(supportBottom + glm::vec2(-baseWidth, 0.0f));
                    vertices.push_back(supportBottom + glm::vec2(baseWidth, -baseHeight));
                    vertices.push_back(supportBottom + glm::vec2(-baseWidth, -baseHeight));

                    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec2), vertices.data(), GL_DYNAMIC_DRAW);
                    solidShader->setVec4("color", glm::vec4(0.12f, 0.12f, 0.14f, 0.95f));
                    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertices.size()));
                }

                // 绘制中心轮毂
                if (ps.windmill.hubRadius > 0.0f)
                {
                    const int segments = 40;
                    const float step = glm::two_pi<float>() / static_cast<float>(segments);
                    vertices.clear();
                    vertices.reserve(segments + 2);
                    vertices.push_back(ps.windmill.center);
                    for (int i = 0; i <= segments; ++i)
                    {
                        float angle = step * static_cast<float>(i);
                        glm::vec2 dir(std::cos(angle), std::sin(angle));
                        vertices.push_back(ps.windmill.center + dir * ps.windmill.hubRadius);
                    }

                    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec2), vertices.data(), GL_DYNAMIC_DRAW);
                    solidShader->setVec4("color", glm::vec4(0.1f, 0.1f, 0.12f, 0.95f));
                    glDrawArrays(GL_TRIANGLE_FAN, 0, static_cast<GLsizei>(vertices.size()));
                }

                glDisable(GL_BLEND);
                glEnable(GL_DEPTH_TEST);
                glBindVertexArray(0);
            }

            // 解绑帧缓冲
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }

        // 获取渲染结果的纹理ID
        GLuint Renderer::getRenderedTexture()
        {
            return textureID;
        }
    }

}
