#pragma once
#ifndef __MESH_UTILS_HPP__
#define __MESH_UTILS_HPP__

// Mesh utility functions
// 提供从顶点和面索引计算顶点法线的功能

#include "scene/Model.hpp"

namespace RayCast {
namespace MeshUtils {

    // 根据 positionIndices(三角形索引) 和 positions 计算每个顶点法线
    // 算法：对每个面计算面法线（cross），用该向量作为面积权重累加到三角形的三个顶点，最后归一化。
    // 结果会写回 mesh.normals（大小为 positions.size()）并把 normalIndices 设为 positionIndices 的副本。
    void computeVertexNormals(NRenderer::Mesh& mesh);

} // namespace MeshUtils
} // namespace RayCast

#endif
