#ifndef CPU_CAMERA_H
#define CPU_CAMERA_H

#include "ray.h"
#include "rng.h"
#include <cmath>

inline nvec3f random_in_unit_disk(Rng& rng) {
    nvec3f p;
    do {
        p = 2.0f*nvec3f(rng.uniform(), rng.uniform(), 0) - nvec3f(1,1,0);
    } while (p.dot(p) >= 1.0f);
    return p;
}

class camera {
public:
    camera(nvec3f lookfrom, nvec3f lookat, nvec3f vup, float vfov, float aspect, float aperture, float focus_dist) {
        lens_radius = aperture / 2.0f;
        float theta = vfov * 0.01745329251f;
        float half_height = std::tan(theta/2.0f);
        float half_width = aspect * half_height;
        origin = lookfrom;
        w = (lookfrom - lookat).normalized();
        u = (vup.cross(w)).normalized();
        v = w.cross(u);
        lower_left_corner = origin  - half_width*focus_dist*u -half_height*focus_dist*v - focus_dist*w;
        horizontal = 2.0f*half_width*focus_dist*u;
        vertical = 2.0f*half_height*focus_dist*v;
    }
    ray get_ray(float s, float t, Rng& rng) const {
        nvec3f rd = lens_radius*random_in_unit_disk(rng);
        nvec3f offset = u * rd[0] + v * rd[1];
        return ray(origin + offset, lower_left_corner + s*horizontal + t*vertical - origin - offset);
    }

    nvec3f origin;
    nvec3f lower_left_corner;
    nvec3f horizontal;
    nvec3f vertical;
    nvec3f u, v, w;
    float lens_radius;
};

#endif
