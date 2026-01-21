#pragma once
#ifndef __SAMPLER_3D_HPP__
#define __SAMPLER_3D_HPP__

#include "Sampler.hpp"

#include <random>
#include "geometry/vec.hpp"

namespace MyPathTracing
{
    using NRenderer::Vec3;
    
    /**
     * ��ά����������
     * �ṩ��ά����������Ľӿ�
     */
    class Sampler3d : public Sampler
    {
        
    public:
        Sampler3d() = default;
        
        /**
         * ������ά�����
         * @return ��ά�������
         */
        virtual Vec3 sample3d() = 0;
    };
}

#endif
