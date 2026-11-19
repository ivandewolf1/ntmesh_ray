# ntmesh_ray
raytracing utilities for tracing ntmeshes

Polygon ray BVH and a CUDA path tracer. `src/ntmesh_ray_bvh.h` builds a tree over triangles and quads. Triangles use Möller–Trumbore. Quads are two triangles or one bilinear patch. A point-normal array, when passed to the hit, is blended at the hit and kept facing the polygon.

The sample loads `samplegeo/mixed_manifold_bunny.geo`, places three copies, and spins the center copy across one hundred twenty frames.

## Roots

`NTV_ROOT` is the ntv repository, the directory that contains `src/nvec3.h`. `NTMESH_ROOT` is the ntmesh repository, the directory that contains `src/ntmesh.h`.

A `-D` argument wins, then the environment variable of the same name, then a sibling checkout (`../ntv/ntv` and `../ntmesh`). Configuration stops and names the variable when a root is still missing.

```
cmake -S . -B build
cmake --build build
./build/ntmesh_ray_tests
```

`build/rtiow_bvh` writes `frame_00.exr` through `frame_119.exr` in the working directory. Run it from `build/`.
