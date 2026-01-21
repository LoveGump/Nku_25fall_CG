// 适配器：接入渲染器并复用实例，避免每帧重建 KD-Tree
#include "server/Server.hpp"
#include "component/RenderComponent.hpp"
#include "RayCastRenderer.hpp"

using namespace std;
using namespace NRenderer;

namespace RayCastKD {
    class Adapter : public RenderComponent {
    private:
        std::unique_ptr<RayCastRenderer> renderer; // 渲染器实例（跨帧复用）
    public:
        void render(SharedScene spScene) {
            // 延迟创建：首次调用时构建并持有渲染器
            if (!renderer)
            {
                renderer = std::make_unique<RayCastRenderer>(spScene);
            }
            // 渲染一帧
            auto result = renderer->render();
            
            // 将渲染结果写入屏幕缓冲
            auto [pixels, width, height] = result;
            getServer().screen.set(pixels, width, height);
            
            // 释放像素内存（由渲染器分配）
            renderer->release(result);
        }
    };
}

// 渲染器的文本描述（供注册/选择使用）
const static string description =
    "Ray Cast KD Renderer.\n"
    "Supported:\n"
    " - Lambertian and Phong\n"
    " - One Point Light\n"
    " - Triangle, Sphere, Plane\n"
    " - Simple Pinhole Camera\n\n"
    "Please use ray_cast.scn";

// ????????
REGISTER_RENDERER(RayCastKD, description, RayCastKD::Adapter);