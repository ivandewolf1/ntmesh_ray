#include "bunny_load.h"
#include "ntmesh.h"
#include "ntmesh_file.h"
#include <cstdlib>

static void takeBuf(int*& dst, int& n, ntm::Buf<int>& src) {
    dst = src.data;
    n = src.count;
    src.data = nullptr;
    src.count = 0;
    src.capacity = 0;
}

int loadBunnyMesh(const char* path, BunnyMesh& out) {
    out = {};
    ntm::ntmesh<float, ntm::WithAttribs> mesh;
    if (!ntm::readFile(mesh, path)) {
        mesh.clear();
        return 0;
    }
    mesh.triangulateNgons();
    if (mesh.pointCount() <= 0 || mesh.triangleCount() + mesh.quadCount() <= 0) {
        mesh.clear();
        return 0;
    }
    ntv::nvec3<float> mn, mx;
    mesh.bounds(mn, mx);
    out.minY = mn[1];
    out.points = mesh.points.data;
    out.npoints = mesh.points.count;
    mesh.points.data = nullptr;
    mesh.points.count = 0;
    mesh.points.capacity = 0;
    takeBuf(out.tris, out.ntris, mesh.triangleIndices);
    out.ntris /= 3;
    takeBuf(out.quads, out.nquads, mesh.quadIndices);
    out.nquads /= 4;
    const ntm::Attribute* N = mesh.findPointAttrib("N");
    if (N && N->tupleSize >= 3 && N->count >= out.npoints && N->data) {
        out.normals = static_cast<ntv::nvec3<float>*>(std::malloc(sizeof(ntv::nvec3<float>) * (size_t)out.npoints));
        if (out.normals) {
            for (int i = 0; i < out.npoints; ++i) {
                const float* s = N->floatAt(i);
                out.normals[i] = ntv::nvec3<float>(s[0], s[1], s[2]);
            }
        }
    }
    mesh.clear();

    ntm::RayBVH<float> bvh;
    bvh.build(out.points, out.tris, out.ntris, out.quads, out.nquads);
    if (bvh.count <= 0 || !bvh.nodes) {
        freeBunnyMesh(out);
        return 0;
    }
    out.nodes = bvh.nodes;
    out.nnodes = bvh.count;
    bvh.nodes = nullptr;
    bvh.count = 0;
    return 1;
}

void freeBunnyMesh(BunnyMesh& out) {
    std::free(out.points);
    std::free(out.normals);
    std::free(out.tris);
    std::free(out.quads);
    std::free(out.nodes);
    out = {};
}
