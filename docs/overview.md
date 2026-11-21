# Overview

**ntmesh_ray** is a header-only bounding-volume hierarchy for rays against an [ntmesh](https://github.com/ivandewolf1/ntmesh). The scalar type is a template parameter. `hit` is marked with `NTL_HD`, so the same tree can be walked on the host or from a CUDA kernel. `build` stays on the host.

The two samples are the final scene from Ray Tracing in One Weekend with the three large spheres replaced by copies of one bunny. They exist to show the tree in a path tracer. The library itself is `src/ntmesh_ray_bvh.h`.

## What the tree stores

Leaves are one triangle or one quad. N-gons are left out. `loadBunnyMesh` calls `triangulateNgons()` before building, which is the usual way to get a mesh ready.

Triangles use Möller–Trumbore. A quad is either two triangles (`RayQuadSplit`) or one bilinear patch (`RayQuadGarp`). The choice is a template argument on `hit`, so one built tree can be asked either way. The samples use `RayQuadGarp`.

The hit normal matches `faceNormal`: clockwise when seen from the outside. A ray that strikes the back keeps that front normal. Pass a point-normal array and the hit blends those normals, then flips the blend if it faces against the polygon.

## Ownership

`build` allocates the node array with `malloc`. `clear` frees it. A plain copy shares the pointer. Call `clear` once for a set of copies.

`build` is host code. `hit` is host and device once `NTL_HD` is `__host__ __device__`. Left undefined, the macro is empty.

## Samples

The samples share the bunny load and the instance seating. Each one has its own ray, sphere, material, camera, and scene tree, because the CUDA versions of those types are `__device__` functions and take a `curandState*`.

| Piece | Where | Role |
|-------|--------|------|
| [`ntmesh_ray_bvh.h`](ray_bvh.md) | `src/` | Mesh tree. Built on the host, walked from either sample |
| `bunny_load.h`, `bunny_load.cpp` | `samples/CUDA/src/` | Read the geo, triangulate n-gons, build one `RayBVH` |
| `seat.h` | `samples/CUDA/src/` | Three world matrices and their world boxes |
| [CUDA sample](cuda.md) | `samples/CUDA/src/` | Device path tracer, 120 frames |
| [CPU sample](cpu.md) | `samples/CPU/src/` | Threaded host path tracer, one frame |

`ray.h`, `sphere.h`, `material.h`, `camera.h`, `nvec3f.h`, and `nanoexr.h` come from `NTV_ROOT/samples/CUDA/src` for the CUDA target. The CPU target has its own copies of the shading headers and still includes `nvec3f.h` and `nanoexr.h` from that directory. `samples/CPU/src` is first on the CPU include path so those local headers win.

Both programs write linear OpenEXR through Christopher Kulla’s `nanoexr.h`. There is no gamma pass.

## Scene the samples share

`samplegeo/mixed_manifold_bunny.geo` is one mixed bunny: 1176 points, 204 triangles, 1072 quads, and a point normal `N`. After n-gons are triangulated, one `RayBVH<float>` is built. Three `mesh_instance` objects share that tree.

| Copy | x | Y translation | Rotation | Material |
|------|---|--------------|----------|----------|
| 0 | +4 | `-minY` of the rest pose, then +0.25 | a fixed random axis, through a full turn across the frame range | glass, index 1.5 |
| 1 | −4 | `-minY` of the rest pose | yaw about Y by the same angle | clay lambertian `(0.4, 0.2, 0.1)` |
| 2 | 0 | `-minY` of the rest pose | yaw about Y by the same angle | metal `(0.7, 0.6, 0.5)`, no fuzz |

The rest of the world is the book's final scene in the CUDA port's layout. A lambertian ground sphere of radius 1000 sits at `(0, -1000, -1)`. The small spheres are a 22×22 grid of radius 0.2, centered near `(a, 0.2, b)` for `a` and `b` in `[-11, 11)`. Each small sphere is lambertian, metal, or glass from one random draw. The camera looks from `(13, 2, 3)` toward the origin, with a 15° vertical field of view, aperture 0.1, and focus distance 11. Rays bounce at most 50 times. The color is linear.

A second BVH, `bvh.h` in each sample, holds those 488 leaves: 485 spheres and 3 mesh instances. It is rebuilt when the instances move. The bunny tree is left alone. Each instance turns the ray into bunny space with the columns of its matrix and turns the hit normal back with the rows.

Sphere centers in the CUDA sample come from `curand` seeded at 1984. The CPU sample uses the same LCG as `seat.h`. The two pictures follow one recipe and land on different sphere positions.

See the [README](../README.md) for the CMake roots and the targets.
