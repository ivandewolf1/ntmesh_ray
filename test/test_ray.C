#include <cmath>
#include <iostream>
#include "tests.h"
#include "ntmesh_file.h"
#include "ntmesh_ray_bvh.h"

using ntm::ntmesh;
using ntm::readFile;
using ntv::nvec3;

void rayTest()
{
  const int fails_before = g_fails;
  std::cout << "*************** test ntmesh_ray *********************\n";

  {
    ntm::RayBVH<float> empty;
    ntm::RayHit<float> miss;
    expect_true(!empty.hit<ntm::RayQuadSplit>(nullptr, nullptr, nullptr, nvec3<float>(0, 0, 0), nvec3<float>(0, 0, 1),
                                              0.f, 1e30f, miss),
                "empty bvh misses");
    nvec3<float> pts[4] = {
        nvec3<float>(0, 0, 0), nvec3<float>(1, 0, 0), nvec3<float>(0, 1, 0), nvec3<float>(1, 1, 0)};
    int tri[3] = {0, 1, 2};
    ntm::RayBVH<float> one;
    one.build(pts, tri, 1, nullptr, 0);
    expect_true(one.count == 1 && one.nodes && one.nodes[0].prim == 0 && one.nodes[0].quad == 0, "one triangle leaf");
    ntm::RayBVH<float> share = one;
    expect_true(share.nodes == one.nodes, "bvh copy shares nodes");
    ntm::RayHit<float> rec;
    expect_true(one.hit<ntm::RayQuadSplit>(pts, tri, nullptr, nvec3<float>(0.2f, 0.2f, 1.f), nvec3<float>(0, 0, -1),
                                           0.f, 1e30f, rec),
                "triangle hit");
    expect_true(std::fabs(rec.t - 1.f) < 1e-5f, "triangle distance");
    expect_vec(rec.p, nvec3<float>(0.2f, 0.2f, 0.f), 1e-5f, "triangle point");
    expect_vec(rec.n, nvec3<float>(0, 0, -1), 1e-5f, "triangle front normal");
    ntmesh<float> face;
    int f0 = face.addPoint(pts[0]);
    int f1 = face.addPoint(pts[1]);
    int f2 = face.addPoint(pts[2]);
    int corners[] = {f0, f1, f2};
    face.addFace(corners, 3);
    expect_vec(rec.n, face.faceNormal(0), 1e-5f, "hit normal matches faceNormal");
    nvec3<float> pn[3] = {
        nvec3<float>(0, 0, -1),
        nvec3<float>(1, 0, -1).normalized(),
        nvec3<float>(0, 1, -1).normalized()};
    expect_true(one.hit<ntm::RayQuadSplit>(pts, tri, nullptr, nvec3<float>(0.2f, 0.2f, 1.f), nvec3<float>(0, 0, -1),
                                           0.f, 1e30f, rec, pn),
                "triangle hit with point normals");
    nvec3<float> blended = pn[0] * 0.6f + pn[1] * 0.2f + pn[2] * 0.2f;
    blended.normalize();
    expect_vec(rec.n, blended, 1e-4f, "triangle blends point normals");
    nvec3<float> inward[3] = {nvec3<float>(0, 0, 1), nvec3<float>(0, 0, 1), nvec3<float>(0, 0, 1)};
    expect_true(one.hit<ntm::RayQuadSplit>(pts, tri, nullptr, nvec3<float>(0.2f, 0.2f, 1.f), nvec3<float>(0, 0, -1),
                                           0.f, 1e30f, rec, inward),
                "inward point normals still hit");
    expect_vec(rec.n, nvec3<float>(0, 0, -1), 1e-4f, "blended normal stays with the face");
    expect_true(one.hit<ntm::RayQuadSplit>(pts, tri, nullptr, nvec3<float>(0.2f, 0.2f, -1.f), nvec3<float>(0, 0, 1),
                                           0.f, 1e30f, rec),
                "triangle hit from the back");
    expect_vec(rec.n, nvec3<float>(0, 0, -1), 1e-5f, "back hit keeps the front normal");
    expect_true(!one.hit<ntm::RayQuadSplit>(pts, tri, nullptr, nvec3<float>(2.f, 2.f, 1.f), nvec3<float>(0, 0, -1),
                                            0.f, 1e30f, rec),
                "ray past the triangle misses");
    one.clear();
    face.clear();

    int quad[4] = {0, 1, 3, 2};
    nvec3<float> qpts[4] = {
        nvec3<float>(0, 0, 0), nvec3<float>(1, 0, 0), nvec3<float>(0, 1, 0), nvec3<float>(1, 1, 0)};
    ntm::RayBVH<float> qtree;
    qtree.build(qpts, nullptr, 0, quad, 1);
    nvec3<float> qo(0.2f, 0.7f, 2.f);
    nvec3<float> qd(0, 0, -1);
    expect_true(qtree.hit<ntm::RayQuadSplit>(qpts, nullptr, quad, qo, qd, 0.f, 1e30f, rec), "split quad hit");
    expect_true(std::fabs(rec.t - 2.f) < 1e-4f && rec.quad == 1 && rec.prim == 0, "split quad distance");
    expect_vec(rec.n, nvec3<float>(0, 0, -1), 1e-4f, "split quad normal");
    expect_true(qtree.hit<ntm::RayQuadGarp>(qpts, nullptr, quad, qo, qd, 0.f, 1e30f, rec), "patch quad hit");
    expect_true(std::fabs(rec.t - 2.f) < 1e-4f, "patch quad distance");
    expect_vec(rec.p, nvec3<float>(0.2f, 0.7f, 0.f), 1e-4f, "patch quad point");
    expect_vec(rec.n, nvec3<float>(0, 0, -1), 1e-4f, "patch quad normal");
    nvec3<float> qn[4] = {
        nvec3<float>(0, 0, -1), nvec3<float>(0, 0, -1),
        nvec3<float>(0, 0, -1), nvec3<float>(0, 1, -1).normalized()};
    expect_true(qtree.hit<ntm::RayQuadGarp>(qpts, nullptr, quad, qo, qd, 0.f, 1e30f, rec, qn), "patch blends normals");
    {
      float u = 0.2f, v = 0.7f;
      nvec3<float> s = qn[0] * ((1.f - u) * (1.f - v)) + qn[1] * (u * (1.f - v))
                     + qn[3] * (u * v) + qn[2] * ((1.f - u) * v);
      s.normalize();
      expect_vec(rec.n, s, 1e-4f, "patch normal is bilinear");
    }
    qtree.clear();

    nvec3<float> twist[4] = {
        nvec3<float>(0, 0, 0), nvec3<float>(1, 0, 0), nvec3<float>(1, 1, 1), nvec3<float>(0, 1, 0)};
    int tw[4] = {0, 1, 2, 3};
    ntm::RayBVH<float> twig;
    twig.build(twist, nullptr, 0, tw, 1);
    nvec3<float> to(0.5f, 0.5f, -1.f);
    nvec3<float> td(0, 0, 1);
    expect_true(twig.hit<ntm::RayQuadSplit>(twist, nullptr, tw, to, td, 0.f, 1e30f, rec), "split twisted quad");
    expect_true(std::fabs(rec.p[2] - 0.5f) < 1e-4f, "split follows the diagonal");
    expect_true(twig.hit<ntm::RayQuadGarp>(twist, nullptr, tw, to, td, 0.f, 1e30f, rec), "patch twisted quad");
    expect_true(std::fabs(rec.p[2] - 0.25f) < 1e-3f, "patch follows the bilinear");
    expect_vec(nvec3<float>(rec.p[0], rec.p[1], 0.f), nvec3<float>(0.5f, 0.5f, 0.f), 1e-4f, "patch stays over the quad");
    twig.clear();

    ntmesh<float> bunny;
    expect_true(readFile(bunny, BUNNY_GEO) == 1, "read bunny for rays");
    bunny.triangulateNgons();
    ntm::RayBVH<float> tree;
    tree.build(bunny.points.data, bunny.triangleIndices.data, bunny.triangleCount(),
               bunny.quadIndices.data, bunny.quadCount());
    expect_true(tree.count > bunny.triangleCount() + bunny.quadCount(), "bunny bvh has internal nodes");
    nvec3<float> bmn, bmx;
    bunny.bounds(bmn, bmx);
    nvec3<float> mid = (bmn + bmx) * 0.5f;
    nvec3<float> dirs[6] = {
        nvec3<float>(1, 0, 0), nvec3<float>(-1, 0, 0), nvec3<float>(0, 1, 0),
        nvec3<float>(0, -1, 0), nvec3<float>(0, 0, 1), nvec3<float>(0, 0, -1)};
    int matched = 0;
    for (int d = 0; d < 6; ++d) {
      nvec3<float> origin = mid - dirs[d] * 4.f;
      for (int method = 0; method < 2; ++method) {
        ntm::RayHit<float> fast, slow;
        bool hitFast = method == 0
            ? tree.hit<ntm::RayQuadSplit>(bunny.points.data, bunny.triangleIndices.data, bunny.quadIndices.data,
                                          origin, dirs[d], 0.f, 1e30f, fast)
            : tree.hit<ntm::RayQuadGarp>(bunny.points.data, bunny.triangleIndices.data, bunny.quadIndices.data,
                                         origin, dirs[d], 0.f, 1e30f, fast);
        slow.t = 1e30f;
        bool hitSlow = false;
        float tmax = 1e30f;
        for (int f = 0; f < bunny.triangleCount(); ++f) {
          int a = bunny.triangleVertex(f, 0), b = bunny.triangleVertex(f, 1), c = bunny.triangleVertex(f, 2);
          float t;
          nvec3<float> nrm;
          if (!ntm::rayTriangle(origin, dirs[d], bunny.points[a], bunny.points[b], bunny.points[c], 0.f, tmax, t, nrm))
            continue;
          hitSlow = true;
          tmax = t;
          slow.t = t;
          slow.p = origin + dirs[d] * t;
          slow.prim = f;
          slow.quad = 0;
        }
        for (int f = 0; f < bunny.quadCount(); ++f) {
          nvec3<float> qv[4];
          for (int k = 0; k < 4; ++k) qv[k] = bunny.points[bunny.quadVertex(f, k)];
          float t;
          nvec3<float> nrm;
          bool ok = method == 0
              ? ntm::rayQuadSplit(origin, dirs[d], qv[0], qv[1], qv[2], qv[3], 0.f, tmax, t, nrm)
              : ntm::rayQuadGarp(origin, dirs[d], qv[0], qv[1], qv[2], qv[3], 0.f, tmax, t, nrm);
          if (!ok) continue;
          hitSlow = true;
          tmax = t;
          slow.t = t;
          slow.p = origin + dirs[d] * t;
          slow.prim = f;
          slow.quad = 1;
        }
        if (hitFast == hitSlow && (!hitFast || (std::fabs(fast.t - slow.t) < 1e-4f && fast.quad == slow.quad && fast.prim == slow.prim)))
          ++matched;
      }
    }
    expect_true(matched == 12, "bunny rays match the leaf tests");
    nvec3<float> away = bmx + nvec3<float>(2, 2, 2);
    expect_true(!tree.hit<ntm::RayQuadSplit>(bunny.points.data, bunny.triangleIndices.data, bunny.quadIndices.data,
                                             away, nvec3<float>(1, 0, 0), 0.f, 1e30f, rec),
                "ray leaving the bunny misses");
    tree.clear();
    bunny.clear();
  }

  const int nfail = g_fails - fails_before;
  if (nfail == 0)
    std::cout << "ntmesh_ray: ok\n";
  else
    std::cout << "ntmesh_ray: " << nfail << " failed\n";
}
