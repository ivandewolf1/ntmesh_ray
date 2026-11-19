#ifndef MESH_INSTANCE_H
#define MESH_INSTANCE_H

#include "material.h"
#include "nmat4.h"
#include "ntmesh_ray_bvh.h"

// Rigid copy of a shared bunny tree. xform maps bunny space to world.
// The ray is turned into bunny space, and the hit normal is turned back.

struct MeshInst {
    ntv::nmat4<float> xform;
    float mn[3];
    float mx[3];
};

class mesh_instance : public hitable {
public:
    __device__ mesh_instance() {}
    __device__ virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const;
    ntv::nmat4<float> xform;
    material* mat;
    ntm::RayBVH<float> bvh;
    const nvec3f* pts;
    const nvec3f* normals;
    const int* tris;
    const int* quads;
};

__device__ bool mesh_instance::hit(const ray& r, float tmin, float tmax, hit_record& rec) const {
    nvec3f o = r.origin() - xform.getCol(3);
    nvec3f d = r.direction();
    nvec3f c0 = xform.getCol(0);
    nvec3f c1 = xform.getCol(1);
    nvec3f c2 = xform.getCol(2);
    nvec3f ol(c0.dot(o), c1.dot(o), c2.dot(o));
    nvec3f dl(c0.dot(d), c1.dot(d), c2.dot(d));
    ntm::RayHit<float> rh;
    if (!bvh.hit<ntm::RayQuadGarp>(pts, tris, quads, ol, dl, tmin, tmax, rh, normals)) return false;
    rec.t = rh.t;
    rec.p = r.point_at_parameter(rh.t);
    rec.normal = nvec3f(xform.getRow(0).dot(rh.n), xform.getRow(1).dot(rh.n), xform.getRow(2).dot(rh.n));
    rec.normal.normalize();
    rec.mat_ptr = mat;
    return true;
}

#endif
