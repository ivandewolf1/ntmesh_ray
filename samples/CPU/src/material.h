#ifndef CPU_MATERIAL_H
#define CPU_MATERIAL_H

struct hit_record;

#include "ray.h"
#include "hitable.h"
#include "rng.h"
#include <cmath>

inline float schlick(float cosine, float ref_idx) {
    float r0 = (1.0f-ref_idx) / (1.0f+ref_idx);
    r0 = r0*r0;
    return r0 + (1.0f-r0)*std::pow((1.0f - cosine),5.0f);
}

inline bool refract(const nvec3f& v, const nvec3f& n, float ni_over_nt, nvec3f& refracted) {
    nvec3f uv = v.normalized();
    float dt = uv.dot(n);
    float discriminant = 1.0f - ni_over_nt*ni_over_nt*(1-dt*dt);
    if (discriminant > 0) {
        refracted = ni_over_nt*(uv - n*dt) - n*std::sqrt(discriminant);
        return true;
    }
    else
        return false;
}

inline nvec3f random_in_unit_sphere(Rng& rng) {
    nvec3f p;
    do {
        p = 2.0f*nvec3f(rng.uniform(), rng.uniform(), rng.uniform()) - nvec3f(1,1,1);
    } while (p.length2() >= 1.0f);
    return p;
}

inline nvec3f reflect(const nvec3f& v, const nvec3f& n) {
     return v - 2.0f*v.dot(n)*n;
}

class material  {
    public:
        virtual ~material() {}
        virtual bool scatter(const ray& r_in, const hit_record& rec, nvec3f& attenuation, ray& scattered, Rng& rng) const = 0;
};

class lambertian : public material {
    public:
        lambertian(const nvec3f& a) : albedo(a) {}
        virtual bool scatter(const ray& r_in, const hit_record& rec, nvec3f& attenuation, ray& scattered, Rng& rng) const  {
             (void)r_in;
             nvec3f target = rec.p + rec.normal + random_in_unit_sphere(rng);
             scattered = ray(rec.p, target-rec.p);
             attenuation = albedo;
             return true;
        }

        nvec3f albedo;
};

class metal : public material {
    public:
        metal(const nvec3f& a, float f) : albedo(a) { if (f < 1) fuzz = f; else fuzz = 1; }
        virtual bool scatter(const ray& r_in, const hit_record& rec, nvec3f& attenuation, ray& scattered, Rng& rng) const  {
            nvec3f reflected = reflect(r_in.direction().normalized(), rec.normal);
            scattered = ray(rec.p, reflected + fuzz*random_in_unit_sphere(rng));
            attenuation = albedo;
            return (scattered.direction().dot(rec.normal) > 0.0f);
        }
        nvec3f albedo;
        float fuzz;
};

class dielectric : public material {
public:
    dielectric(float ri) : ref_idx(ri) {}
    virtual bool scatter(const ray& r_in,
                         const hit_record& rec,
                         nvec3f& attenuation,
                         ray& scattered,
                         Rng& rng) const  {
        nvec3f outward_normal;
        nvec3f reflected = reflect(r_in.direction(), rec.normal);
        float ni_over_nt;
        attenuation = nvec3f(1.0, 1.0, 1.0);
        nvec3f refracted;
        float reflect_prob;
        float cosine;
        if (r_in.direction().dot(rec.normal) > 0.0f) {
            outward_normal = -rec.normal;
            ni_over_nt = ref_idx;
            cosine = r_in.direction().dot(rec.normal) / r_in.direction().length();
            cosine = std::sqrt(1.0f - ref_idx*ref_idx*(1-cosine*cosine));
        }
        else {
            outward_normal = rec.normal;
            ni_over_nt = 1.0f / ref_idx;
            cosine = -r_in.direction().dot(rec.normal) / r_in.direction().length();
        }
        if (refract(r_in.direction(), outward_normal, ni_over_nt, refracted))
            reflect_prob = schlick(cosine, ref_idx);
        else
            reflect_prob = 1.0f;
        if (rng.uniform() < reflect_prob)
            scattered = ray(rec.p, reflected);
        else
            scattered = ray(rec.p, refracted);
        return true;
    }

    float ref_idx;
};
#endif
