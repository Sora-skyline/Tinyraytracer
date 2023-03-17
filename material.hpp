#pragma once
#include <eigen3/Eigen/Eigen>
using namespace Eigen;
class Material {
public:
    Material(const float& ior, const Vector4f& alb, const Vector3f& color, const float& exp, const std::string& type) :refractive_index(ior), albedo(alb), diffuse_color(color), specular_exponent(exp), type(type) {}
    Material() : refractive_index(1), albedo(1, 0, 0, 0), diffuse_color(), specular_exponent(), type() {}
    float refractive_index;
    Vector4f albedo; //���������ʣ� �߹ⷴ���ʣ����淴����, ͸���ʣ�
    Vector3f diffuse_color;//  ������ɫ
    float specular_exponent;// �߹���ݴ�
    std::string type;
};
