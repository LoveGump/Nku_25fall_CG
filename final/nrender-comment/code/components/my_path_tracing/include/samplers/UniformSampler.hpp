#pragma once
#ifndef __UNIFORM_SAMPLER_HPP__
#define __UNIFORM_SAMPLER_HPP__

#include "Sampler1d.hpp"
#include <ctime>

namespace MyPathTracing
{
    using namespace std;
    
    /**
     * ���ȷֲ�һά������
     * ��[0,1]���������ɾ��ȷֲ��������
     */
    class UniformSampler : public Sampler1d
    {
    private:
        default_random_engine e;                    // ���������
        uniform_real_distribution<float> u;         // ���ȷֲ�������
        
    public:
        /**
         * ���캯������ʼ�������������
         */
        UniformSampler()
            : e                 ((unsigned int)time(0) + insideSeed())
            , u                 (0, 1)
        {}
        
        /**
         * ����[0,1]�����ڵľ��ȷֲ������
         * @return �����
         */
        float sample1d() override {
            return u(e);
        }
    };
}

#endif
