#pragma once
#ifndef __PHOTON_HPP__
#define __PHOTON_HPP__

#include "geometry/vec.hpp"

namespace SimplePathTracer {
    using namespace NRenderer;

    struct Photon {
        Vec3 position;     // 命中位置
        Vec3 incident;     // 入射方向（指向表面）
        Vec3 flux;         // 光子携带的能量/通量 (RGB)
    };
}

#endif

