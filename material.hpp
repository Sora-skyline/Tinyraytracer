#pragma once
#include <eigen3/Eigen/Eigen>
using namespace Eigen;
class Material {
public:
    Material(const float& ior, const Vector4f& alb, const Vector3f& color, const float& exp, const std::string& type) :refractive_index(ior), albedo(alb), diffuse_color(color), specular_exponent(exp), type(type) {}
    Material() : refractive_index(1), albedo(1, 0, 0, 0), diffuse_color(), specular_exponent(), type() {}
    float refractive_index;
    Vector4f albedo; //（漫反射率， 高光反射率，镜面反射率, 透光率）
    Vector3f diffuse_color;//  物体颜色
    float specular_exponent;// 高光的幂次
    std::string type;
};
