// 顶点变换器实现
// 实现了场景中模型顶点的坐标变换功能
#include "VertexTransformer.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/matrix_inverse.hpp"

namespace RayCastKD
{
    // 执行顶点变换
    // spScene: 场景指针
    // 将场景中所有模型从局部坐标转换到世界坐标
    void VertexTransformer::exec(SharedScene spScene)
    {
        auto &scene = *spScene;
        // 遍历场景中的所有节点
        for (auto &node : scene.nodes)
        {
            // 创建变换矩阵
            Mat4x4 t{1};
            auto &model = spScene->models[node.model];
            // 平移变换
            t = glm::translate(t, model.translation);
            // 法线矩阵（线性部分的逆转置）
            Mat3x3 normalMat = glm::transpose(glm::inverse(Mat3x3{t}));

            // 根据实体类型应用变换
            if (node.type == Node::Type::TRIANGLE)
            {
                // 顶点位置变换
                for (int i = 0; i < 3; i++)
                {
                    auto &v = scene.triangleBuffer[node.entity].v[i];
                    v = Vec3(t * Vec4{v, 1});
                }
                // 变换三角形的顶点法线与面法线
                for (int i = 0; i < 3; ++i)
                {
                    auto &n = scene.triangleBuffer[node.entity].vertexNormals[i];
                    n = glm::normalize(normalMat * n);
                }
                auto &fn = scene.triangleBuffer[node.entity].normal;
                fn = glm::normalize(normalMat * fn);
            }
            else if (node.type == Node::Type::SPHERE)
            {
                // 变换球体的中心位置（半径保持不变，避免非均匀缩放导致椭球）
                auto &v = scene.sphereBuffer[node.entity].position;
                v = Vec3(t * Vec4{v, 1});
            }
            else if (node.type == Node::Type::PLANE)
            {
                // 平面位置变换
                auto &v = scene.planeBuffer[node.entity].position;
                v = Vec3(t * Vec4{v, 1});
            }
            else if (node.type == Node::Type::MESH)
            {
                // 变换网格的所有顶点位置
                auto &mesh = scene.meshBuffer[node.entity];
                for (auto &pos : mesh.positions)
                {
                    pos = Vec3(t * Vec4{pos, 1});
                }
                // 变换网格的顶点法线（若存在）
                for (auto &n : mesh.vertexNormals)
                {
                    n = glm::normalize(normalMat * n);
                }
                // 变换网格的面法线（若存在）
                for (auto &n : mesh.normals)
                {
                    n = glm::normalize(normalMat * n);
                }
            }
        }
    }
}