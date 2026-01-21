#include "Configure.h"

// 系统配置参数
int imageWidth = 600;  // 渲染分辨率宽度
int imageHeight = 600; // 渲染分辨率高度

int windowWidth = 1080; // 窗口默认宽度
int windowHeight = 960; // 窗口默认高度

float fontSize = 16.0f; // 字体大小

bool simulating = false; // 模拟状态，用于切换暂停

// 2D欧拉方法相关参数
namespace Eulerian2dPara
{
    // MAC网格参数
    int theDim2d[2] = {100, 100}; // 网格维度
    float theCellSize2d = 0.5;    // 网格单元大小

    // 烟雾源的配置参数
    std::vector<SourceSmoke> source = {
        {// 源的位置                      // 初始速度           // 密度  // 温度
         glm::ivec2(theDim2d[0] / 3, 0), glm::vec2(0.0f, 1.0f), 1.0f, 1.0f}};

    bool addSolid = true; // 是否添加固体边界

    // 渲染器设置
    float contrast = 1;        // 对比度
    int drawModel = 0;         // 绘制模式
    int gridNum = theDim2d[0]; // 网格数量

    // 求解器设置
    float dt = 0.01;               // 时间步长
    float airDensity = 1.3;        // 空气密度
    float ambientTemp = 0.0;       // 环境温度
    float boussinesqAlpha = 500.0; // Boussinesq力公式中的alpha参数
    float boussinesqBeta = 2500.0; // Boussinesq力公式中的beta参数
}

// 3D欧拉方法相关参数
namespace Eulerian3dPara
{
    // MAC网格参数
    int theDim3d[3] = {12, 36, 36}; // 网格维度(确保 x <= y = z)
    float theCellSize3d = 0.5;      // 网格单元大小
    std::vector<SourceSmoke> source = {
        {glm::ivec3(theDim3d[0] / 2, theDim3d[1] / 2, 0), glm::vec3(0.0f, 0.0f, 1.0f), 1.0f, 1.0f}};
    bool addSolid = true; // 是否添加固体边界

    // 渲染器设置
    float contrast = 1;                                           // 对比度
    bool oneSheet = true;                                         // 是否只显示一个切片
    float distanceX = 0.51;                                       // X轴切片位置
    float distanceY = 0.51;                                       // Y轴切片位置
    float distanceZ = 0.985;                                      // Z轴切片位置
    bool xySheetsON = true;                                       // 是否显示XY平面切片
    bool yzSheetsON = true;                                       // 是否显示YZ平面切片
    bool xzSheetsON = true;                                       // 是否显示XZ平面切片
    int drawModel = 0;                                            // 绘制模式
    int gridNumX = (int)((float)theDim3d[0] / theDim3d[2] * 100); // X轴网格数
    int gridNumY = (int)((float)theDim3d[1] / theDim3d[2] * 100); // Y轴网格数
    int gridNumZ = 100;                                           // Z轴网格数
    int xySheetsNum = 3;                                          // XY平面切片数量
    int yzSheetsNum = 3;                                          // YZ平面切片数量
    int xzSheetsNum = 3;                                          // XZ平面切片数量

    // 求解器设置
    float dt = 0.01;               // 时间步长
    float airDensity = 1.3;        // 空气密度
    float ambientTemp = 0.0;       // 环境温度
    float boussinesqAlpha = 500.0; // Boussinesq力公式中的alpha参数
    float boussinesqBeta = 2500.0; // Boussinesq力公式中的beta参数
}

// 2D拉格朗日方法相关参数
namespace Lagrangian2dPara
{
    // 缩放系数
    float scale = 2;

    // 流体块初始配置（对撞/溃坝场景）
    std::vector<FluidBlock> fluidBlocks = {
        {glm::vec2(-0.9f, -0.95f), glm::vec2(-0.3f, -0.35f), glm::vec2(0.0f, -0.8f), 0.025f},
        {glm::vec2(0.3f, -0.95f), glm::vec2(0.9f, -0.35f), glm::vec2(0.0f, -0.8f), 0.025f},
        {glm::vec2(-0.4f, 0.6f), glm::vec2(0.4f, 0.95f), glm::vec2(0.0f, 0.0f), 0.025f}};

    bool enableMovingSolid = false;                  // Enable moving solid obstacle
    glm::vec2 solidCenter = glm::vec2(0.0f, -0.2f);  // Base center (unscaled)
    glm::vec2 solidVelocity = glm::vec2(0.0f, 0.0f); // Kinematic velocity
    float solidRadius = 0.2f;                        // Solid radius (unscaled)
    float solidRestitution = 0.2f;                   // Bounce against fluid
    float solidFriction = 0.05f;                     // Tangential damping

    bool enableWindmill = true;                        // Enable rotating windmill
    glm::vec2 windmillCenter = glm::vec2(0.0f, -0.2f); // Windmill center (unscaled)
    float windmillBladeLength = 0.6f;                  // Blade length (unscaled)
    float windmillBladeWidth = 0.08f;                  // Blade width (unscaled)
    float windmillHubRadius = 0.12f;                   // Hub radius (unscaled)
    int windmillBladeCount = 4;                        // Blade count
    float windmillAngularVelocity = 2.0f;              // Angular velocity (rad/s)
    float windmillRestitution = 0.2f;                  // Bounce against blades
    float windmillFriction = 0.08f;                    // Tangential damping

    bool enableIncompressible = true; // Use incompressible solver

    int incompressibleIterations = 4; // PCISPH iterations

    // 求解器设置
    float dt = 0.0016;               // 时间步长
    int substep = 1;                 // 子步数
    float maxVelocity = 10;          // 最大允许速度
    float velocityAttenuation = 0.7; // 碰撞后的速度衰减系数
    float eps = 1e-5;                // 一个很小的距离，用于边界处理，防止粒子穿过边界

    // 粒子系统参数
    float supportRadius = 0.04;                    // 在此半径内的相邻粒子会对当前粒子产生影响
    float particleRadius = 0.01;                   // 粒子的半径
    float particleDiameter = particleRadius * 2.0; // 粒子直径
    float gravityX = 0.0f;                         // x轴上的加速度
    float gravityY = 9.8f;                         // y轴上的加速度
    float density = 1000.0f;                       // 流体密度
    float stiffness = 70.0f;                       // 刚度
    float exponent = 7.0f;                         // 压力计算公式中的指数
    float viscosity = 0.03f;                       // 粘度

    /**
     * 应用场景预设
     * 根据选择的场景类型配置所有相关参数
     */
    void applyScenePreset(ScenePreset preset)
    {
        switch (preset)
        {
        case ScenePreset::COLLISION:
            // 对撞场景：两块流体从两侧对撞
            fluidBlocks.clear();
            fluidBlocks.push_back({glm::vec2(-0.9f, -0.95f), glm::vec2(-0.3f, -0.1f), glm::vec2(2.0f, -0.8f), 0.025f});
            fluidBlocks.push_back({glm::vec2(0.3f, -0.95f), glm::vec2(0.9f, -0.1f), glm::vec2(-2.0f, -0.8f), 0.025f});
            enableMovingSolid = false;
            enableWindmill = false;
            break;

        case ScenePreset::DAM_BREAK:
            // 溃坝场景：单侧大块流体坍塌
            fluidBlocks.clear();
            fluidBlocks.push_back({glm::vec2(-0.9f, -0.95f), glm::vec2(-0.1f, 0.5f), glm::vec2(0.0f, 0.0f), 0.025f});
            enableMovingSolid = false;
            enableWindmill = false;
            break;

        case ScenePreset::WATERWHEEL:
            // 水车场景：流体从上方落下驱动水车旋转
            fluidBlocks.clear();
            // 上方及底部的流体块
            fluidBlocks.push_back({glm::vec2(-0.9f, -0.95f), glm::vec2(-0.3f, -0.35f), glm::vec2(0.0f, -0.8f), 0.025f});
            fluidBlocks.push_back({glm::vec2(0.3f, -0.95f), glm::vec2(0.9f, -0.35f), glm::vec2(0.0f, -0.8f), 0.025f});
            fluidBlocks.push_back({glm::vec2(-0.4f, 0.6f), glm::vec2(0.4f, 0.95f), glm::vec2(0.0f, 0.0f), 0.025f});

            // 配置水车参数
            enableWindmill = true;
            windmillCenter = glm::vec2(0.0f, -0.2f);
            windmillBladeLength = 0.6f;
            windmillBladeWidth = 0.08f;
            windmillHubRadius = 0.12f;
            windmillBladeCount = 4;
            windmillAngularVelocity = 2.0f; // 初始速度
            windmillRestitution = 0.2f;
            windmillFriction = 0.08f;

            enableMovingSolid = false;
            break;

        case ScenePreset::CUSTOM:
        default:
            // 自定义场景：保持当前配置不变
            break;
        }
    }
}

// 3D拉格朗日方法相关参数
namespace Lagrangian3dPara
{
    // 缩放系数
    float scale = 1.2;

    // 流体块初始配置 - Z轴位置降低，更接近木板
    std::vector<FluidBlock> fluidBlocks = {
        {glm::vec3(0.05, 0.05, 0.15), glm::vec3(0.45, 0.45, 0.45), glm::vec3(0.0, 0.0, -1.0), 0.02f},
        {glm::vec3(0.45, 0.45, 0.15), glm::vec3(0.85, 0.85, 0.45), glm::vec3(0.0, 0.0, -1.0), 0.02f}};

    // 旋转木板参数
    bool enableRotatingBoard = true;                           // 启用旋转木板
    glm::vec3 boardCenter = glm::vec3(0.45f, 0.45f, 0.05f);    // 木板中心位置（场景最底部）
    glm::vec3 boardSize = glm::vec3(0.6f, 0.06f, 0.2f);        // 木板尺寸 (长, 厚, 高) - 不超过场景边界
    glm::vec3 boardRotationAxis = glm::vec3(0.0f, 0.0f, 1.0f); // 旋转轴 (Z轴 - 垂直旋转)
    float boardAngularVelocity = 10.0f;                        // 角速度 (rad/s) - 快速旋转
    float boardRestitution = 0.5f;                             // 恢复系数 - 增大弹性
    float boardFriction = 0.2f;                                // 摩擦系数

    // 求解器设置
    float dt = 0.002;                // 时间步长
    int substep = 1;                 // 子步数
    float maxVelocity = 10;          // 最大允许速度
    float velocityAttenuation = 0.7; // 碰撞后的速度衰减系数
    float eps = 1e-5;                // 一个很小的距离，用于边界处理

    // 粒子系统参数
    float supportRadius = 0.04;                    // 支持半径
    float particleRadius = 0.01;                   // 粒子半径
    float particleDiameter = particleRadius * 2.0; // 粒子直径

    float gravityX = 0.0f; // x轴重力
    float gravityY = 0.0f; // y轴重力
    float gravityZ = 9.8f; // z轴重力

    float density = 1000.0f; // 密度
    float stiffness = 70.0f; // 刚度（与2D一致）
    float exponent = 7.0f;   // 压力指数
    float viscosity = 0.03f; // 粘度（与2D一致）
}

// 存储系统的所有组件
std::vector<Glb::Component *> methodComponents;

// 资源路径
std::string shaderPath = "../../../../code/resources/shaders";
std::string picturePath = "../../../../code/resources/pictures";
