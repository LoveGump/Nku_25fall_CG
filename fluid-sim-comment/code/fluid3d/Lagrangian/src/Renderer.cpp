#include "fluid3d/Lagrangian/include/Renderer.h"
#include <cmath>
#include <vector>
#include <glm/gtc/constants.hpp>

namespace FluidSimulation
{
    namespace Lagrangian3d
    {

        void Renderer::init()
        {
            container = new Glb::Container();
            container->resetSize(1, 1, 1);
            container->init();

            // Build Shaders

            shader = new Glb::Shader();
            std::string drawColorVertPath = shaderPath + "/DrawParticles3d.vert";
            std::string drawColorFragPath = shaderPath + "/DrawParticles3d.frag";
            shader->buildFromFile(drawColorVertPath, drawColorFragPath);

            // 固体着色器（用于水车）
            solidShader = new Glb::Shader();
            std::string lineVertPath = shaderPath + "/Line.vert";
            std::string lineFragPath = shaderPath + "/Line.frag";
            solidShader->buildFromFile(lineVertPath, lineFragPath);

            // Generate Frame Buffers
            // generate frame buffer object
            glGenFramebuffers(1, &FBO);
            // make it active
            // start fbo
            glBindFramebuffer(GL_FRAMEBUFFER, FBO);

            // generate textures
            glGenTextures(1, &textureID);
            glBindTexture(GL_TEXTURE_2D, textureID);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imageWidth, imageHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
            glBindTexture(GL_TEXTURE_2D, 0);

      
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureID, 0);

            // generate render buffer object (RBO)
            glGenRenderbuffers(1, &RBO);
            glBindRenderbuffer(GL_RENDERBUFFER, RBO);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, imageWidth, imageHeight);
            glBindRenderbuffer(GL_RENDERBUFFER, 0);

            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, RBO);
            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            {
                Glb::Logger::getInstance().addLog("Error: Framebuffer is not complete!");
            }
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glGenBuffers(1, &VBO);
            MakeVertexArrays();

            // 水车渲染缓冲
            glGenVertexArrays(1, &solidVAO);
            glGenBuffers(1, &solidVBO);

            glEnable(GL_MULTISAMPLE);

            glViewport(0, 0, imageWidth, imageHeight);
        }

        void Renderer::MakeVertexArrays()
        {
            glGenVertexArrays(1, &VAO);
            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(particle3d), (void *)offsetof(particle3d, position));
            glEnableVertexAttribArray(0); // location = 0
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(particle3d), (void *)offsetof(particle3d, density));
            glEnableVertexAttribArray(1); // location = 1
            glBindVertexArray(0);
        }

        void Renderer::draw(ParticleSystem3d &ps)
        {
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, VBO);
            glBufferData(GL_SHADER_STORAGE_BUFFER, ps.particles.size() * sizeof(particle3d), ps.particles.data(), GL_DYNAMIC_COPY);
            particleNum = ps.particles.size();

            glBindFramebuffer(GL_FRAMEBUFFER, FBO);
            glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LEQUAL);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glEnable(GL_PROGRAM_POINT_SIZE);

            shader->use();
            shader->setMat4("view", Glb::Camera::getInstance().GetView());
            shader->setMat4("projection", Glb::Camera::getInstance().GetProjection());
            shader->setFloat("scale", ps.scale);

            glBindVertexArray(VAO);
            glDrawArrays(GL_POINTS, 0, particleNum);
            shader->unUse();

            // 绘制旋转木板
            drawRotatingBoard(ps);

            container->draw();

            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }

        void Renderer::drawRotatingBoard(ParticleSystem3d &ps)
        {
            if (!ps.rotatingBoard.enabled)
                return;

            RotatingBoard3d &board = ps.rotatingBoard;
            
            // 应用旋转（使用 Rodrigues 公式）- 与碰撞检测保持一致
            glm::vec3 axis = glm::normalize(board.rotationAxis);
            float cosA = std::cos(board.angle);
            float sinA = std::sin(board.angle);
            
            auto rotateVec = [&](const glm::vec3 &v) -> glm::vec3 {
                return v * cosA + glm::cross(axis, v) * sinA + axis * glm::dot(axis, v) * (1.0f - cosA);
            };

            // 木板的局部坐标系（旋转后）
            glm::vec3 localX = rotateVec(glm::vec3(1, 0, 0));
            glm::vec3 localY = rotateVec(glm::vec3(0, 1, 0));
            glm::vec3 localZ = rotateVec(glm::vec3(0, 0, 1));

            float halfX = board.size.x * 0.5f;
            float halfY = board.size.y * 0.5f;
            float halfZ = board.size.z * 0.5f;

            std::vector<glm::vec3> vertices;

            // 8个角点
            glm::vec3 corners[8];
            for (int i = 0; i < 8; ++i)
            {
                float sx = (i & 1) ? halfX : -halfX;
                float sy = (i & 2) ? halfY : -halfY;
                float sz = (i & 4) ? halfZ : -halfZ;
                corners[i] = board.center + localX * sx + localY * sy + localZ * sz;
            }

            // 6个面，每个面2个三角形
            int faces[6][4] = {
                {0, 1, 3, 2}, // -Z
                {4, 6, 7, 5}, // +Z
                {0, 4, 5, 1}, // -X
                {2, 3, 7, 6}, // +X
                {0, 2, 6, 4}, // -Y
                {1, 5, 7, 3}  // +Y
            };

            for (int f = 0; f < 6; ++f)
            {
                vertices.push_back(corners[faces[f][0]]);
                vertices.push_back(corners[faces[f][1]]);
                vertices.push_back(corners[faces[f][2]]);
                vertices.push_back(corners[faces[f][0]]);
                vertices.push_back(corners[faces[f][2]]);
                vertices.push_back(corners[faces[f][3]]);
            }

            if (vertices.empty())
                return;

            glBindVertexArray(solidVAO);
            glBindBuffer(GL_ARRAY_BUFFER, solidVBO);
            glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void *)0);
            glEnableVertexAttribArray(0);

            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            solidShader->use();
            solidShader->setMat4("view", Glb::Camera::getInstance().GetView());
            solidShader->setMat4("projection", Glb::Camera::getInstance().GetProjection());
            solidShader->setVec4("color", glm::vec4(1.0f, 0.6f, 0.2f, 1.0f));  // 橙色，完全不透明

            glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertices.size()));

            solidShader->unUse();
            glDisable(GL_BLEND);
            glBindVertexArray(0);
        }

        GLuint Renderer::getRenderedTexture()
        {
            return textureID;
        }
    }
}