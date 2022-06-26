#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "geometry.h"

struct Material
{
    Vec4f albedo;
    Vec3f diffuse_color;
    float speular;
    float refractive_index;
    Material(const Vec3f& color) : diffuse_color(color) {};
    Material() :refractive_index(1),albedo(1,0,0,0),diffuse_color(),speular() {};
    Material(const float &refra,const Vec4f& a, const Vec3f& color, const float& spec) :refractive_index(refra),albedo(a), diffuse_color(color), speular(spec) {}
};

struct Light
{
    Vec3f position;
    float intensity;
    Light(const Vec3f& p, const float& i) :position(p), intensity(i) {};
};

struct Sphere {

    Vec3f center;
    float radius;
    Material material;
    Sphere(const Vec3f& c, const float& r,const Material &m) :center(c), radius(r), material(m){}

    bool Ray_Insert(const Vec3f& origion, const Vec3f& direction, float &t0) const//光线与球求交
    {
        Vec3f L= center - origion;
        float tca = L * direction;
        float d2 = L * L - tca * tca;
        if (d2 > radius * radius)return false;
        float thc =sqrtf(radius * radius - d2);
        t0 = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0)t0 = t1;
        if (t0 < 0)return false;//反向相交忽略不算
        return true;
    }
};

bool Scene_Intersect(const Vec3f &orig,const Vec3f &direction, const std::vector<Sphere>& spheres,Vec3f& hitpoint,  Vec3f& normal,Material &material)
{
    float sphere_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < spheres.size(); i++)
    {
        float dist_i;
        if (spheres[i].Ray_Insert(orig, direction, dist_i) && dist_i < sphere_dist)
        {
            sphere_dist = dist_i;
            hitpoint = orig + direction * dist_i;
            normal = (hitpoint - spheres[i].center).normalize();
            material = spheres[i].material;
        }
    }
   // return sphere_dist < 1000;
    float checkerboard_dist = std::numeric_limits<float>::max();
    if (fabs(direction.y) > 1e-3) {
        float d = -(orig.y + 4) / direction.y; // the checkerboard plane has equation y = -4
        Vec3f pt = orig + direction * d;
        if (d > 0 && fabs(pt.x) < 10 && pt.z<-10 && pt.z>-30 && d < sphere_dist) {
            checkerboard_dist = d;
            hitpoint = pt;
            normal = Vec3f(0, 1, 0);
            material.diffuse_color = (int(.5 * hitpoint.x + 1000) + int(.5 * hitpoint.z)) & 1 ? Vec3f(1, 1, 1) : Vec3f(1, .7, .3);
            material.diffuse_color = material.diffuse_color * .3;
        }
    }
    return std::min(sphere_dist, checkerboard_dist) < 1000;
}

Vec3f refract(const Vec3f& I, const Vec3f& N, const float& refractive_index)
{
    float cosi = -std::max(-1.f, std::min(1.f, I * N));
    float etai = 1, etat = refractive_index;
    Vec3f n = N;
    if (cosi < 0)
    {
        cosi = -cosi;
        std::swap(etai, etat); n = -N;
    }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    return k < 0 ? Vec3f(0, 0, 0) : I * eta + n * (eta * cosi - sqrtf(k));
}

Vec3f reflect(const Vec3f& I, const Vec3f& N) {
    return N * 2.f * (I * N)- I;
}
Vec3f Cast_Ray(const Vec3f& origion, const Vec3f& direction, const std::vector<Sphere> &spheres,const std::vector<Light> &lights,rsize_t depth=0)
{
    Vec3f hitpoint, normal;
    Material material;
    if (depth>4||!Scene_Intersect(origion, direction, spheres, hitpoint, normal, material))//不相交返回背景色
    {
        return Vec3f(0.2, 0.7, 0.8);
    }

    //reflect
    Vec3f reflect_dir = reflect(-direction, normal).normalize();
    Vec3f reflect_orig = reflect_dir * normal < 0 ? hitpoint - normal * 1e-3 : hitpoint + normal * 1e-3;//确保反射不与自身相交

     //refract
    Vec3f refract_dir = refract(direction, normal, material.refractive_index).normalize();
    Vec3f refract_orig = refract_dir * normal < 0 ? hitpoint - normal * 1e-3 : hitpoint + normal * 1e-3;//确保折射不与自身相交

    Vec3f reflect_color = Cast_Ray(reflect_orig, reflect_dir, spheres,lights, depth+1);
    Vec3f refract_color = Cast_Ray(refract_orig, refract_dir, spheres, lights, depth + 1);

    float diffuse_light_intensity = 0, specluar_light_intensity = 0;
    for (size_t i = 0; i < lights.size(); i++)
    {
        Vec3f light_dir = (lights[i].position - hitpoint).normalize();
        

        //shadow
        float lightTohitpointDistance= (lights[i].position - hitpoint).norm();
        Vec3f shaow_orig = light_dir * normal < 0 ? hitpoint - normal * 1e-3 : hitpoint + normal * 1e-3;//确保计算阴影的光线不与自身相交
        Vec3f shaowLineHitPoint, shaowLineHitPointNormal;
        Material materialTemp;
        if (Scene_Intersect(shaow_orig, light_dir, spheres, shaowLineHitPoint, shaowLineHitPointNormal, materialTemp) && (shaowLineHitPoint - shaow_orig).norm() < lightTohitpointDistance)continue;


        diffuse_light_intensity += lights[i].intensity * std::max(0.f, normal * light_dir);
        specluar_light_intensity += lights[i].intensity * powf(std::max(0.f, reflect(-light_dir, normal) * direction), material.speular);
    }

    //return material.diffuse_color * diffuse_light_intensity*material.albedo[0]+Vec3f(1.,1.,1.)* specluar_light_intensity*material.albedo[1];
    //return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.) * specluar_light_intensity * material.albedo[1]+ reflect_color*material.albedo[2];
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.) * specluar_light_intensity * material.albedo[1] + reflect_color * material.albedo[2] + refract_color * material.albedo[3];
  
}


void render( const std::vector<Sphere> &shperes,const std::vector<Light> & lights) {
    const int width = 1024;
    const int height = 768;
    const float PI = 4 * atan(1);
    float fov = PI /2 ;
 

    std::vector<Vec3f> framebuffer(width*height);

#pragma omp parallel for
    for (size_t j = 0; j < height; j++)
    {
        for (size_t i = 0; i < width; i++) 
        {
            //framebuffer[i * width + j] = Vec3f(1, j / float(width), i / float(height)); 
            float x = (2 * (i + 0.5) / (float)width - 1) * tan(fov / 2.) * width / (float)height;
            float y = -(2 * (j + 0.5) / (float)width - 1) * tan(fov / 2.);
            Vec3f dir = Vec3f(x, y, -1).normalize();
            framebuffer[i + width * j] = Cast_Ray(Vec3f(0,0,0), dir, shperes,lights);
        }
    }


    std::ofstream ofs;
    ofs.open("./out.ppm", std::ofstream::out | std::ofstream::binary);
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
    Material      ivory(1.0, Vec4f(0.6, 0.3, 0.1, 0.0), Vec3f(0.4, 0.4, 0.3), 50.);
    Material      glass(1.5, Vec4f(0.0, 0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8), 125.);
    Material red_rubber(1.0, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1), 10.);
    Material     mirror(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.);

    std::vector<Sphere> spheres;
    spheres.push_back(Sphere(Vec3f(-3, 0, -16), 2, ivory));
    spheres.push_back(Sphere(Vec3f(-1.0, -1.5, -12), 2, glass));
    spheres.push_back(Sphere(Vec3f(1.5, -0.5, -18), 3, red_rubber));
    spheres.push_back(Sphere(Vec3f(7, 5, -18), 4, mirror));

    std::vector<Light>  lights;
    lights.push_back(Light(Vec3f(-20, 20, 20), 1.5));
    lights.push_back(Light(Vec3f(30, 50, -25), 1.8));
    lights.push_back(Light(Vec3f(30, 20, 30), 1.7));

    render(spheres, lights);
    return 0;
}