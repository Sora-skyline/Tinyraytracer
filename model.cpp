#pragma once
#include <iostream>
#include <cassert>
#include <fstream>
#include <sstream>
#include "model.hpp"
using namespace Eigen;
inline float dotProduct(Vector3f a, Vector3f b)
{
    return a.dot(b);
}
Model::Model(const char* filename, Material m) {
    material = m;
    std::ifstream in;
    in.open(filename, std::ifstream::in);
    if (in.fail()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }
    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line.c_str());
        char trash;
        if (!line.compare(0, 2, "v ")) {
            iss >> trash;
            Vector3f v;
            for (int i = 0; i < 3; i++) iss >> v[i];
            verts.push_back(v);
        }
        else if (!line.compare(0, 2, "f ")) {
            Vector3i f;
            int idx, cnt = 0;
            iss >> trash;
            while (iss >> idx) {
                idx--;
                f[cnt++] = idx;
            }
            if (3 == cnt) tris.push_back(f);
        }
    }
    tri_num = tris.size();
    vec_num = verts.size();
    std::cerr << "# v# " << verts.size() << " f# " << tris.size() << std::endl;
    get_bbox(box.min, box.max);
}
void Model::get_bbox(Vector3f& min, Vector3f& max) {
    min = max = verts[0];
    for (int i = 1; i < (int)verts.size(); ++i) {
        for (int j = 0; j < 3; j++) {
            min[j] = std::min(min[j], verts[i][j]);
            max[j] = std::max(max[j], verts[i][j]);
        }
    }
    std::cerr << "bbox: [" << min << " : " << max << "]" << std::endl;
}
bool Model::rayIntersectWithAABB(const Vector3f& orig, const Vector3f& d) const {//判断是否进入包围盒
    float t_min_x = (box.min.x() - orig.x()) / d.x();
    float t_min_y = (box.min.y() - orig.y()) / d.y();
    float t_min_z = (box.min.z() - orig.z()) / d.z();

    float t_max_x = (box.max.x() - orig.x()) / d.x();
    float t_max_y = (box.max.y() - orig.y()) / d.y();
    float t_max_z = (box.max.z() - orig.z()) / d.z();

    if (d.x() < 0) {
        std::swap(t_max_x, t_min_x);
    }
    if (d.y() < 0) {
        std::swap(t_max_y, t_min_y);
    }
    if (d.z() < 0) {
        std::swap(t_max_z, t_min_z);
    }

    float t_enter = std::max(t_min_x, std::max(t_min_y, t_min_z));
    float t_exit = std::min(t_max_x, std::min(t_max_y, t_max_z));
    return t_exit > t_enter && t_exit >= 0;
}
bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& orig, const Vector3f& dir, float& tnear, float& u, float& v)
{
    auto e1 = v1 - v0;
    auto e2 = v2 - v0;
    auto s = orig - v0;
    auto s1 = dir.cross(e2);
    auto s2 = s.cross(e1);
    tnear = dotProduct(s2, e2) / dotProduct(s1, e1);
    u = dotProduct(s1, s) / dotProduct(s1, e1);
    v = dotProduct(s2, dir) / dotProduct(s1, e1);
    double eps = -1e-6;
    if (tnear > eps && u > eps && v > eps && 1 - u - v > eps) {
        //std::cout << "cross"<<std::endl;
        return true;
    }

    return false;
}
bool Model::ray_intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t& index, Vector2f& uv, Vector3f &normal)
{
    bool intersect = false;
    if (rayIntersectWithAABB(orig, dir)) {
        //std::cout << "AABB" << std::endl;
        float t = std::numeric_limits<float>::max();
        for (int k = 0; k < tri_num; ++k)
        {
            const Vector3f& v0 = verts[tris[k].x()];
            const Vector3f& v1 = verts[tris[k].y()];
            const Vector3f& v2 = verts[tris[k].z()];
            float t, u, v;
            if (rayTriangleIntersect(v0, v1, v2, orig, dir, t, u, v) && t < tnear)
            {
                tnear = t;
                uv.x() = u;
                uv.y() = v;
                index = k;
                intersect |= true;
            }
        }
    
    }
    if (intersect) {
        Vector3f a = verts[tris[index].y()] - verts[tris[index].x()];
        Vector3f b = verts[tris[index].z()] - verts[tris[index].x()];
        normal = a.cross(b);
    }
    return intersect;
}