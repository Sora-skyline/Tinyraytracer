#pragma once
#include <eigen3/Eigen/Eigen>
#include "material.hpp"
using namespace Eigen;
struct AABB {
    Vector3f min;
    Vector3f max;
};
class Model {
public:
    Model(const char* filename, Material m);
    Model() {};
    bool ray_intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t& index, Vector2f& uv, Vector3f& normal);
    bool rayIntersectWithAABB(const Vector3f& orig, const Vector3f& dir)const;
    void get_bbox(Vector3f& min, Vector3f& max);
    int get_verts() { return vec_num; }
    int get_tris() { return tri_num; }
    Material material;
private:
    AABB box;
    int box_num;
    int vec_num;
    int tri_num;
    std::vector<Vector3f>verts;
    std::vector<Vector3i>tris;
};