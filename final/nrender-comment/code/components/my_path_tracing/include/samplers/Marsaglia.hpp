#pragma once
#ifndef __MARSAGLIA_HPP__
#define __MARSAGLIA_HPP__

#include "Sampler3d.hpp"
#include <ctime>

namespace MyPathTracing
{
    using namespace std;
    
    /**
     * Marsaglia������ȷֲ�������
     * �ڵ�λ���������ɾ��ȷֲ����������
     * ʹ��Marsaglia������ͨ���ܾ������ڵ�λԲ�����ɵ㣬Ȼ��ӳ�䵽����
     */
    class Marsaglia : public Sampler3d
    {
    private:
        default_random_engine e;                    // ���������
        uniform_real_distribution<float> u;         // ���ȷֲ�������
        
    public:
        /**
         * ���캯������ʼ�������������
         */
        Marsaglia()
            : e               ((unsigned int)time(0) + insideSeed())
            , u               (-1, 1)
        {}

        /**
         * ���ɵ�λ�����ϵľ��ȷֲ��������
         * ʹ��Marsaglia�������ڵ�λԲ�ڲ�����Ȼ��ӳ�䵽����
         * @return ��ά�����������λ������
         */
        Vec3 sample3d() override {
            float u_{0}, v_{0};
            float r2{0};
            do {
                u_ = u(e);
                v_ = u(e);
                r2 = u_*u_ + v_*v_;
            } while (r2 > 1);  // �ܾ�������ȷ�����ڵ�λԲ��
            
            // Marsaglia�任����Բ�ڵ�ӳ�䵽����
            float x = 2 * u_ * sqrt(1 - r2);
            float y = 2 * v_ * sqrt(1 - r2);
            float z = 1 - 2 * r2;
            return { x, y, z };
        }
    };
}

#endif
