#pragma once
#include "ntmesh_ray_bvh.h"

// Host copy of one triangulated bunny and its ray tree.
// points, tris, quads, and nodes are owned here. Call freeBunnyMesh when finished.

struct BunnyMesh {
    ntv::nvec3<float>* points = nullptr;
    ntv::nvec3<float>* normals = nullptr;
    int* tris = nullptr;
    int* quads = nullptr;
    ntm::RayNode<float>* nodes = nullptr;
    int npoints = 0;
    int ntris = 0;
    int nquads = 0;
    int nnodes = 0;
    float minY = 0.f;
};

int loadBunnyMesh(const char* path, BunnyMesh& out);
void freeBunnyMesh(BunnyMesh& out);
