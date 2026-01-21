#include "mesh_utils.hpp"
#include "glm/gtx/norm.hpp"

namespace RayCast {
namespace MeshUtils {

void computeVertexNormals(NRenderer::Mesh& mesh) {
    // 需要 positions 与 positionIndices
    if (mesh.positions.empty() || mesh.positionIndices.empty()) return;

    // 初始化累加缓冲
    std::vector<NRenderer::Vec3> normals(mesh.positions.size(), NRenderer::Vec3{0,0,0});

    // 遍历三角形索引，按三个一组
    for (size_t i = 0; i + 2 < mesh.positionIndices.size(); i += 3) {
        auto i0 = mesh.positionIndices[i];
        auto i1 = mesh.positionIndices[i+1];
        auto i2 = mesh.positionIndices[i+2];
        if (i0 >= mesh.positions.size() || i1 >= mesh.positions.size() || i2 >= mesh.positions.size()) continue;

        auto p0 = mesh.positions[i0];
        auto p1 = mesh.positions[i1];
        auto p2 = mesh.positions[i2];

        // 面法线（未归一化）——cross 的长度与三角形面积成正比
        auto faceVec = glm::cross(p1 - p0, p2 - p0);
        // 如果退化三角形，跳过
        if (glm::length2(faceVec) < 1e-12f) continue;

        // 用未归一化的面法线作为面积权重累加到每个顶点
        normals[i0] += faceVec;
        normals[i1] += faceVec;
        normals[i2] += faceVec;
    }

    // 归一化并写回 mesh.normals
    mesh.normals.clear();
    mesh.normals.resize(normals.size());
    for (size_t i = 0; i < normals.size(); ++i) {
        auto n = normals[i];
        if (glm::length2(n) < 1e-12f) {
            // fallback: 使用单位 z 轴
            mesh.normals[i] = NRenderer::Vec3{0,0,1};
        } else {
            mesh.normals[i] = glm::normalize(n);
        }
    }

    // normalIndices 与 positionIndices 一致，便于按三角形索引访问
    mesh.normalIndices = mesh.positionIndices;
}

} // namespace MeshUtils
} // namespace RayCast
