#pragma once
#ifndef __SHADER_CREATOR_HPP__
#define __SHADER_CREATOR_HPP__

#include "Shader.hpp"
#include "Lambertian.hpp"
#include "Mirror.hpp"
#include "Dielectric.hpp"

namespace SimplePathTracer
{
    class ShaderCreator
    {
    public:
        ShaderCreator() = default;
        SharedShader create(Material &material, vector<Texture> &t)
        {
            auto getFloat = [&](const std::string &key)
            {
                auto prop = material.getProperty<Property::Wrapper::FloatType>(key);
                return prop ? (*prop).value : 0.0f;
            };

            float reflectivity = getFloat("reflectivity");
            float transparency = getFloat("transparency");

            SharedShader shader{nullptr};
            switch (material.type)
            {
            case 1:
                shader = make_shared<Mirror>(material, t);
                break;
            case 2:
                shader = make_shared<Dielectric>(material, t);
                break;
            default:
                if (transparency > 1e-3f)
                {
                    shader = make_shared<Dielectric>(material, t);
                }
                else if (reflectivity > 1e-3f)
                {
                    shader = make_shared<Mirror>(material, t);
                }
                else
                {
                    shader = make_shared<Lambertian>(material, t);
                }
                break;
            }
            if (!shader)
            {
                shader = make_shared<Lambertian>(material, t);
            }
            return shader;
        }
    };
}

#endif