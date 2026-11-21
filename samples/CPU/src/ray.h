#ifndef CPU_RAY_H
#define CPU_RAY_H
#include "nvec3f.h"

class ray
{
    public:
        ray() {}
        ray(const nvec3f& a, const nvec3f& b) { A = a; B = b; }
        nvec3f origin() const       { return A; }
        nvec3f direction() const    { return B; }
        nvec3f point_at_parameter(float t) const { return A + t*B; }

        nvec3f A;
        nvec3f B;
};

#endif
