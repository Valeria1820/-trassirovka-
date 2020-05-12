#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include "geometry.h"

//--------------------Создание структуры, реализующей освещение---------------------------------------
struct Light {
    Light(const Vec3f& p, const float i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};
//--------------------Создание структуры, реализующей материал, из который состоят наши шарики--------
struct Material {
    Material(const Vec3f& color) :   diffuse_color(color) {}
    Material() :  diffuse_color() {}
    Vec3f diffuse_color;
   
};
//-------------------Структура, отвечающая за отрисовку сферы--------------------------------------------
struct Sphere {
    Vec3f center;
    float radius;
    Material material;

    Sphere(const Vec3f& c, const float r, const Material& m) : center(c), radius(r), material(m) {}

    bool ray_intersect( const Vec3f& dir, float& t0) const {
        Vec3f L = center ;
        float tca = L * dir;
        float d2 = L * L - tca * tca;
        if (d2 > radius* radius) return false;
        float thc = sqrtf(radius * radius - d2);
        t0 = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }
};


bool scene_intersect( const Vec3f& dir, const std::vector<Sphere>& spheres, Vec3f& hit, Vec3f& N, Material& material) {
    float spheres_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < spheres.size(); i++) {
        float dist_i;
        if (spheres[i].ray_intersect( dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = dir * dist_i;
            N = (hit - spheres[i].center).normalize();
            material = spheres[i].material;
        }
    }
    return spheres_dist < 1000;
}

Vec3f cast_ray(const Vec3f& orig, const Vec3f& dir, const std::vector<Sphere>& spheres, const std::vector<Light>& lights, size_t depth = 0) {
    Vec3f point, N;
    Material material;

    if (depth > 4 || !scene_intersect(dir, spheres, point, N, material)) {
        return Vec3f(0.3, 0, 0.3); // background color
    }


    float diffuse_light_intensity = 0, specular_light_intensity = 0;
    for (size_t i = 0; i < lights.size(); i++) {
        Vec3f light_dir = (lights[i].position - point).normalize();
        float light_distance = (lights[i].position - point).norm();

        Vec3f shadow_orig = light_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3; // checking if the point lies in the shadow of the lights[i]
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;
        if (scene_intersect( light_dir, spheres, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt - shadow_orig).norm() < light_distance)
            continue;

        diffuse_light_intensity += lights[i].intensity * std::max(0.f, light_dir * N);
      
    }
    return material.diffuse_color * diffuse_light_intensity + Vec3f(1., 1., 1.) * specular_light_intensity ;
}

void render(const std::vector<Sphere>& spheres, const std::vector<Light>& lights) {
    const int   width = 1024;
    const int   height = 768;
    const float fov = M_PI / 3.;
    std::vector<Vec3f> framebuffer(width * height);

#pragma omp parallel for
    for (size_t j = 0; j < height; j++) { // actual rendering loop
        for (size_t i = 0; i < width; i++) {
            float dir_x = (i + 0.5) - width / 2.;
            float dir_y = -(j + 0.5) + height / 2.;    // this flips the image at the same time
            float dir_z = -height / (2. * tan(fov / 2.));
            framebuffer[i + j * width] = cast_ray(Vec3f(0, 0, 0), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights);
        }
    }

    std::ofstream ofs; // Save file
    ofs.open("./out.ppm", std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (size_t i = 0; i < height * width; ++i) {
        Vec3f& c = framebuffer[i];
        float max = std::max(c[0], std::max(c[1], c[2]));
        if (max > 1) c = c * (1. / max);
        for (size_t j = 0; j < 3; j++) {
            ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    ofs.close();
}

int main() {
    Material      ivory( Vec3f(0.5, 0.1, 0.4));
  
    Material indigo( Vec3f(0.4, 0.4, 0.4));


    std::vector<Sphere> spheres;
    spheres.push_back(Sphere(Vec3f(-3, 0, -16), 2, ivory));
   
    spheres.push_back(Sphere(Vec3f(2, -0.5, -18), 3, indigo));
  

    std::vector<Light>  lights;
    lights.push_back(Light(Vec3f(-20, 20, 20), 1.5));
  // lights.push_back(Light(Vec3f(30, 50, -25), 1.8));
   // lights.push_back(Light(Vec3f(30, 20, 30), 1.7));

    render(spheres, lights);

    return 0;
}
