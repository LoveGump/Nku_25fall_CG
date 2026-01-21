#pragma once
#ifndef __VERTEX_TRANSFORM_HPP__
#define __VERTEX_TRANSFORM_HPP__

#include "scene/Scene.hpp"

namespace BDPT
{
    using namespace NRenderer;
    // 由局部坐标转换为世界坐标
    class VertexTransformer
    {
    private:
    public:
        void exec(SharedScene spScene);

        // 将相机坐标（世界坐标）转换为光栅坐标
        static Vec2 worldToRaster(const Vec3& worldCoord, const Camera& camera, int screenWidth, int screenHeight)
        {
            // Convert world coordinates to camera coordinates
            Vec3 cameraCoord = worldCoord; // TODO:暂时不考虑相机的旋转

            // Project camera coordinates to normalized device coordinates (NDC)
            Vec2 ndc;
            ndc.x = cameraCoord.x / cameraCoord.z;
            ndc.y = cameraCoord.y / cameraCoord.z;

            // Convert NDC to raster coordinates
            Vec2 rasterCoord;
            rasterCoord.x = (ndc.x + 1) * 0.5f * screenWidth;
            rasterCoord.y = (1 - ndc.y) * 0.5f * screenHeight;

            return rasterCoord;
        }
    };
}

#endif