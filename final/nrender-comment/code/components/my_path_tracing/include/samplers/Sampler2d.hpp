#pragma once
#ifndef __SAMPLER_2D_HPP__
#define __SAMPLER_2D_HPP__

#include "Sampler.hpp"

#include <random>
#include "geometry/vec.hpp"

namespace MyPathTracing
{
    using NRenderer::Vec2;
    
    /**
     * ��ά����������
     * �ṩ��ά����������Ľӿ�
     */
    class Sampler2d : public Sampler
    {
    public:
        Sampler2d() = default;
        
        /**
         * ���ɶ�ά�����
         * @return ��ά�������
         */
        virtual Vec2 sample2d() = 0;
    };
}

#endif
