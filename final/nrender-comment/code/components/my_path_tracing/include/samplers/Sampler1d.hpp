#pragma once
#ifndef __SAMPLER_1D_HPP__
#define __SAMPLER_1D_HPP__

#include "Sampler.hpp"

#include <random>

namespace MyPathTracing
{
    /**
     * һά����������
     * �ṩһά����������Ľӿ�
     */
    class Sampler1d : protected Sampler
    {
    public:
        Sampler1d() = default;
        
        /**
         * ����һά�����
         * @return �����
         */
        virtual float sample1d() = 0;
    };
}

#endif
