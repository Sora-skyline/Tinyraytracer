#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "Model.hpp"
#include "material.hpp"
#include <eigen3/Eigen/Eigen>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
using namespace Eigen;
#define M_PI 3.14159265358979323846
int envmap_width, envmap_height;

std::vector<Vector3f> envmap;
// 角度转弧度
inline float deg2rad(const float& deg)
{
    return deg * M_PI / 180.0;
}
// 反射光线
inline Vector3f reflect(const Vector3f& I, const Vector3f& N)
{
    return I - 2 * I.dot( N) * N;
}
// 折射光线
inline Vector3f refract(const Vector3f& I, const Vector3f& N, const float& ior)
{
    float cosi = std::max(-1.f, std::min(1.f, I.dot(N)));
    float etai = 1, etat = ior;
    Vector3f n = N;
    if (cosi < 0) { cosi = -cosi; }
    else { std::swap(etai, etat); n = -N; }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    return k < 0 ? Vector3f{0,0,0} : eta * I + (eta * cosi - sqrtf(k)) * n;
}

struct  Light {
    Light(const Vector3f& p, const float& i) : position(p), intensity(i) {}
    Vector3f position;
    float intensity;
};

struct Sphere {
    Vector3f center;
    float radius;
    Material material;
    Sphere() {};
    Sphere(const Vector3f& c, const float& r, const Material& mat) : center(c), radius(r), material(mat) {}

    inline bool ray_intersect(const Vector3f& orig, const Vector3f& dir, float& t0) const {
        Vector3f L = center - orig;
        float L2 = L.dot(L);
        float tca = L.dot(dir);
        float d2 = L.dot( L) - tca * tca;
        if (d2 > radius * radius) return false;
        if (L2 > radius * radius && tca < 0)return false;
        float thc = sqrtf(radius * radius - d2);
        t0 = tca - thc;// 球内钝角，球外
        float t1 = tca + thc; // 球内锐角
        if (t0 < 0) t0 = t1;
        return true;
    }
};
struct Plane {
    Vector3f v0, v1;
    Material material;
    Plane() {};
    Plane(const Vector3f& v0, const Vector3f& v1, const Material& mat) : v0(v0), v1(v1), material(mat){}
    inline bool ray_intersect(const Vector3f& orig, const Vector3f& dir, float& t0) const {
        if (std::fabs(dir.y()) > 1e-3) {
            float d = -(orig.y() - v0.y()) / dir.y();
            t0 = d;
            Vector3f pt = orig + dir * d;
            if (d > 0 && pt.x() > v0.x() && pt.z() < v0.z() && pt.x() < v1.x() && pt.z() > v1.z())
            return true;
        }
        return false;
    }
};

struct Object {
    Object(){};
    std::vector<Sphere> spheres;
    Plane plane;
    Model model;
};
//和所有物体判断相交
bool intersect(const Vector3f& orig, const Vector3f& dir, const Object&object, Vector3f &hit, Vector3f & normal, Material& material) {
    float tmin = std::numeric_limits<float>::max();
    for (size_t i = 0; i < object.spheres.size(); i++) {
        auto sphere = object.spheres[i];
        float t = 0;
        if (sphere.ray_intersect(orig, dir, t) && t < tmin) {
            tmin = t;
            hit = orig + dir * t;
            normal = (hit - sphere.center).normalized();
            material = sphere.material;
        }
    }
    // 棋盘平面
    auto plane = object.plane;
    float t  = tmin;
    if (plane.ray_intersect(orig, dir, t) && t < tmin) {
        tmin = t;
        hit = orig +dir * t;
        normal = Vector3f(0, 1, 0);
        //material = plane.material;
        material.diffuse_color = (int(.5 * hit.x() + 1000) + int(.5 * hit.z())) & 1 ? Vector3f(.3, .3, .3) : Vector3f(.3, .2, .1);
    }
     //obj模型
    Model model = object.model;
     t = tmin;
    uint32_t index = 0;
    Vector2f uv; 
    Vector3f n = { 0,0,0 };
    if (model.ray_intersect(orig, dir, t, index, uv, n) && t < tmin) {
        //std::cout << "ray_intersect";
        tmin = t;
        hit = orig + dir * t;
        normal = n.normalized();
        material = model.material;
    }
    return tmin < 1000;
}
Vector3f castRay(const Vector3f &orig,const  Vector3f &dir, const Object&object, const std::vector<Light> &lights, int depth) {
    //相交的数据
    Material material;
    Vector3f hit, normal;

    if (depth> 10 || !intersect(orig, dir, object, hit, normal, material)) {
        int x = std::max(0, std::min(envmap_width - 1, static_cast<int>((atan2(dir.z(), dir.x()) / (2 * M_PI) + .5) * envmap_width)));
        int y = std::max(0, std::min(envmap_height - 1, static_cast<int>(acos(dir.y()) / M_PI * envmap_height)));
        return envmap[x + y * envmap_width];
        //return Vector3f(0.7937, 0.7937, 0.7937);
    }
    Eigen::Vector3f kd = material.diffuse_color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);
    Vector3f result_color = { 0,0,0 };
    //反射
    Vector3f reflect_color = { 0,0,0 };
    if (material.type != "rubber") {
        auto reflect_dir = reflect(dir, normal).normalized();
        Vector3f reflect_orig;
        if (reflect_dir.dot(normal) < 0)reflect_orig = hit - normal * 1e-3;
        else reflect_orig = hit + normal * 1e-3;
        reflect_color = castRay(reflect_orig, reflect_dir, object, lights, depth + 1);
    }
    //折射
    Vector3f refract_color = { 0,0,0 };
    if (material.type == "glass") {
        auto refract_dir = refract(dir, normal, material.refractive_index).normalized();
        Vector3f refract_orig;
        if (refract_dir.dot(normal) < 0)refract_orig = hit - normal * 1e-3;
        else refract_orig = hit + normal * 1e-3;
        refract_color = castRay(refract_orig, refract_dir, object, lights, depth + 1);
    }
    // phong 着色
    Vector3f diffuse = {0, 0, 0};
    Vector3f specular = {0, 0, 0};
    for (auto& light : lights)
    {
        auto l = (light.position - hit).normalized();// l 方向的单位向量
        auto v = (orig - hit).normalized();
        auto r_square = (light.position - hit).squaredNorm();//r^2
        //判断在阴影里
        Vector3f shadow_orig = { 0, 0, 0 }, shadow_hit = { 0,0,0 }, shadow_normal = { 0,0,0 };
        Material tmp;
        float light_dist = (light.position - hit).norm();
        if (l.dot(normal) < 0)shadow_orig = hit - normal * 1e-3;
        else shadow_orig = hit + normal * 1e-3;
        if (intersect(shadow_orig, l, object, shadow_hit, shadow_normal, tmp) && (shadow_hit - shadow_orig).norm() < light_dist) 
            continue;

        auto h = (v + l).normalized();
        diffuse += kd*(light.intensity) * std::max(0.0f, l.dot(normal));
        specular += ks*(light.intensity) * pow((std::max(h.dot(normal), 0.0f)), material.specular_exponent);
    }
    result_color = diffuse * material.albedo[0] + specular * material.albedo[1] + reflect_color * material.albedo[2] + refract_color*material.albedo[3];
    return result_color;
}
inline void UpdateProgress(float progress)
{
    int barWidth = 70;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i)
    {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
}
void render(Object object, std::vector<Light> lights) {
    const int width = 1024, height = 760;
    const float fov = 90;
    const Vector3f eye_pos = {3,2,4};
    float scale = std::tan(deg2rad(fov * 0.5f));
    float imageAspectRatio = width / (float)height;
    std::vector<Vector3f> framebuffer(width * height);
    int cnt = 0;
    #pragma omp parallel for
    for (int j = 0; j < height;  j++) {
        for (int i = 0; i < width; i++) {
            float dir_x = (i + 0.5f) - width / 2.f;
            float dir_y = -(j + 0.5f) + height / 2.f; 
            float dir_z = -height / (2.0f * tan(fov / 2.f));

            Vector3f dir = Vector3f(dir_x, dir_y, dir_z);
            dir = dir.normalized();
            framebuffer[i + j * width] = castRay(eye_pos, dir, object, lights, 0);
        }
        std::cout <<cnt++/ (float)height<< '\n';
        //if (cnt > 500)break;
        //UpdateProgress(j / (float)height);
    }

    std::vector<unsigned char> pixmap(width * height * 3);
    for (size_t i = 0; i < height * width; ++i) {
        Vector3f& c = framebuffer[i];
        float max = std::max(c[0], std::max(c[1], c[2]));
        if (max > 1) c = c * (1. / max);
        for (size_t j = 0; j < 3; j++) {
            pixmap[i * 3 + j] = (unsigned char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    std::string fileName = "./output/out_.jpg";
    stbi_write_jpg(fileName.c_str(), width, height, 3, pixmap.data(), 100);
}

int main() {
    int n = -1;
    unsigned char* pixmap = stbi_load("envmap.jpg", &envmap_width, &envmap_height, &n, 0);
    if (!pixmap || 3 != n) {
        std::cerr << "Error: can not load the environment map" << std::endl;
        return -1;
    }
    envmap = std::vector<Vector3f>(envmap_width * envmap_height);
    #pragma omp parallel for
    for (int j = envmap_height - 1; j >= 0; j--) {
        for (int i = 0; i < envmap_width; i++) {
            envmap[i + j * envmap_width] = Vector3f(pixmap[(i + j * envmap_width) * 3 + 0], pixmap[(i + j * envmap_width) * 3 + 1], pixmap[(i + j * envmap_width) * 3 + 2]) * (1 / 255.);
        }
    }
    stbi_image_free(pixmap);
    Object object;
    std::vector<Light>lights;
    Material      glass          (1.5, {0.0, 0.5, 0.1, 0.8}, Vector3f(0.6, 0.7, 0.8 ), 125., "glass");
    Material      ivory          (1.0, {0.6, 0.3, 0.1, 0.0}, Vector3f(0.4, 0.4, 0.3), 200, "ivory");
    Material      rubber       (1.0, {0.9, 0.1, 0.0, 0.0}, Vector3f(0.3, 0.1, 0.1), 100, "rubber");
    Material      mirror        (1.0, {0.0, 10, 0.8, 0.0}, Vector3f(1.0, 1.0, 1.0), 1425, "mirror");
    std::vector<Sphere> spheres;
    object.spheres.push_back(Sphere(Vector3f(-3, 0, -16),        2, rubber));
    object.spheres.push_back(Sphere(Vector3f(-1.0, -1.5, -12), 2, glass));
    object.spheres.push_back(Sphere(Vector3f(1.5, -0.5, -18),  3, rubber));
    object.spheres.push_back(Sphere(Vector3f(7, 5, -18),         4, mirror));

    object.model = Model("tri.obj", glass);
    object.plane = Plane({ -10, -4, -10 }, { 10,-4, -30}, ivory);
    lights.push_back(Light(Vector3f(-20, 20, 20), 2));
    lights.push_back(Light(Vector3f(30, 50, -25), 1.8));
    lights.push_back(Light(Vector3f(30, 20, 30), 1.7));
    
    render(object, lights );
    return 0;
}