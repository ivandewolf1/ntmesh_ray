#ifndef CPU_HITABLE_H
#define CPU_HITABLE_H

#include "ray.h"

class material;

struct hit_record
{
    float t;
    nvec3f p;
    nvec3f normal;
    material *mat_ptr;
};

class hitable  {
    public:
        virtual ~hitable() {}
        virtual bool hit(const ray& r, float t_min, float t_max, hit_record& rec) const = 0;
};

#endif
