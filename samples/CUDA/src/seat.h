#pragma once
#include "bunny_load.h"
#include "nmat4.h"
#include "nvec3f.h"

// Host copy of one placed bunny. The CUDA and CPU samples share this seating.
// xform maps bunny space to world. mn and mx are the world box, padded.

struct MeshInst {
    ntv::nmat4<float> xform;
    float mn[3];
    float mx[3];
};

inline float unitRand(unsigned& seed) {
    seed = seed * 1664525u + 1013904223u;
    return (seed >> 8) * (1.f / 16777216.f);
}

// Center copy spins about the origin. The other two stay seated on the ground.
inline void seatInstances(MeshInst inst[3], const BunnyMesh& bunny, const nvec3f& axis, float angle) {
    float seat = -bunny.minY;
    float placeX[3] = {4.f, -4.f, 0.f};
    float placeY[3] = {seat + 0.25f, seat, seat};
    for (int k = 0; k < 3; ++k) {
        ntv::nmat4<float> M;
        if (k == 0) M.setRotation(axis, angle);
        else M.setRotation(nvec3f(0.f, 1.f, 0.f), angle);
        M.translate(nvec3f(placeX[k], placeY[k], 0.f));
        inst[k].xform.set(M);
        nvec3f p0 = M * bunny.points[0];
        nvec3f mn = p0;
        nvec3f mx = p0;
        for (int p = 1; p < bunny.npoints; ++p) {
            nvec3f wp = M * bunny.points[p];
            for (int a = 0; a < 3; ++a) {
                if (wp[a] < mn[a]) mn[a] = wp[a];
                if (wp[a] > mx[a]) mx[a] = wp[a];
            }
        }
        nvec3f pad(1e-3f, 1e-3f, 1e-3f);
        mn -= pad;
        mx += pad;
        for (int a = 0; a < 3; ++a) {
            inst[k].mn[a] = mn[a];
            inst[k].mx[a] = mx[a];
        }
    }
}
