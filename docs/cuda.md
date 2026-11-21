# RTIOW on CUDA

`rtiow_bvh` is the final scene from
*[Ray Tracing in One Weekend](https://raytracing.github.io/books/RayTracingInOneWeekend.html)*,
on CUDA, with the three large spheres replaced by copies of one bunny.

The device layout follows the ntv CUDA sample, which follows Roger Allen’s public-domain translation
([`ch12_where_next_cuda`](https://github.com/rogerallen/raytracinginoneweekendincuda/tree/ch12_where_next_cuda),
also the NVIDIA post
[Accelerated Ray Tracing in One Weekend in CUDA](https://developer.nvidia.com/blog/accelerated-ray-tracing-cuda/)).
Shading types (`ray`, `sphere`, `material`, `camera`, `nvec3f`, `nanoexr`) are included from `NTV_ROOT/samples/CUDA/src`. This repository adds the mesh tree, the instance, the scene BVH, and the frame loop.

The mesh tree is [`ntm::RayBVH`](ray_bvh.md). It is built on the host from `samplegeo/mixed_manifold_bunny.geo` and uploaded once. Each frame rebuilds only the tree over the spheres and the three instances.

## Citation

> Peter Shirley, Trevor David Black, Steve Hollasch.
> *[Ray Tracing in One Weekend](https://raytracing.github.io/books/RayTracingInOneWeekend.html)*.
> [RayTracing/raytracing.github.io](https://github.com/RayTracing/raytracing.github.io)

The book project is dedicated to the public domain under
[CC0 1.0 Universal](https://creativecommons.org/publicdomain/zero/1.0/).

> Roger Allen.
> *[Ray Tracing in One Weekend in CUDA](https://github.com/rogerallen/raytracinginoneweekendincuda/tree/ch12_where_next_cuda)*
> ([NVIDIA blog](https://developer.nvidia.com/blog/accelerated-ray-tracing-cuda/)).

OpenEXR output uses Christopher Kulla’s
[nanoexr](https://gist.github.com/fpsunflower/e5c99116ff94114d1cbe),
via the copy in the ntv CUDA sample.

> Chris Kulla.
> [Tiny OpenEXR output (nanoexr)](https://gist.github.com/fpsunflower/e5c99116ff94114d1cbe).

## Build and run

Configure from the repository root. The CUDA toolkit and CMake 3.18 or newer are required. Roots and architecture defaults are in the [README](../README.md).

```
cmake -S . -B build
cmake --build build --target rtiow_bvh
cd build
./rtiow_bvh
```

The executable writes `frame_00.exr` through `frame_119.exr` in the working directory. Run it from `build/`. Stderr prints the resolution, the bunny counts, a `scene` line, and one line per frame.

Defaults in `samples/CUDA/src/main.cu`:

| Setting | Value |
|---------|--------|
| Image | 1200×800 |
| Samples per pixel | 100 |
| Bounce limit | 50 |
| Frames | 120 |
| Thread block | 8×8 |
| Device stack | `cudaLimitStackSize` set to 65536 |

The `scene` line is the device time for `create_world`. Each frame line is the device time for that frame’s scene-tree rebuild, `curand` init, and trace. The EXR write happens after the timer stops.

## Scene

The shared layout is in the [overview](overview.md). On this target the angle advances `2π / 120` per frame, starting from a random angle drawn with the host LCG in `seat.h`. Copy 0 spins about the axis chosen at startup. Copies 1 and 2 yaw about Y and stay on the ground seat.

`create_world` runs as one thread and allocates spheres, materials, the three instances, and the camera with device `new`. Sphere materials use `curand` seeded at 1984. `update_frame` writes the three matrices, builds the scene BVH, and deletes the previous scene tree on later frames. `free_world` deletes the materials, the leaves, the scene tree, and the camera.

The framebuffer is managed memory, one `nvec3f` per pixel. `nanoexr` reads it as contiguous RGB floats. Sample `v` is flipped so row 0 of the file is the top of the view. Values are linear. There is no gamma pass.

## Mesh instance

`mesh_instance` is a `hitable`. It shares the uploaded points, normals, indices, and `RayBVH`. The hit subtracts the translation, dots the ray with the matrix columns to enter bunny space, calls `bvh.hit<ntm::RayQuadGarp>` with the point normals, and dots the resulting normal with the matrix rows to return to world space.

`MeshInst` in `seat.h` is the host record: the matrix and the world box. The box is the transformed point bounds, padded by `1e-3`, and it is what the scene BVH uses for that leaf.

## Scene BVH

`samples/CUDA/src/bvh.h` is a binary tree over scene leaves. A leaf is any `hitable` with its own box, here a sphere or a mesh instance. Internal nodes live in one pool allocated with device `new[]`. The walker keeps an explicit stack of 256 pointers and tells a node from a leaf by which pointers fall inside that pool. The split is a 12-bin surface-area cut. A cut whose smaller side is under an eighth of the range is rejected, unless the range is already 32 leaves or fewer, or one side is a single primitive that covers more than half the parent area. Otherwise the range is sorted on the widest centroid axis and cut at the midpoint.

## Layout

| File | Role |
|------|------|
| `src/main.cu` | World, frame loop, render kernels, EXR |
| `src/bvh.h` | Scene tree over spheres and instances |
| `src/mesh_instance.h` | One rigid copy of the bunny tree |
| `src/bunny_load.h`, `bunny_load.cpp` | Geo read, n-gon triangulation, host `RayBVH` |
| `src/seat.h` | `MeshInst` and `seatInstances`, shared with the CPU sample |
| `../../src/ntmesh_ray_bvh.h` | Mesh tree |
| ntv `samples/CUDA/src/` | `ray`, `sphere`, `material`, `camera`, `nvec3f`, `nanoexr` |

## What changed from the ntv CUDA sample

- The three large spheres are bunny instances. The small-sphere field, the ground, the camera, and the materials stay.
- The flat hittable list is a scene BVH, rebuilt each frame.
- The image is 120 frames. Copy 0 spins about a random axis. The other two yaw about Y.
- Quads are bilinear patches, and the bunny’s point normals are blended at the hit.
- The host LCG in `seat.h` places the instances. `curand` still places the small spheres and scatters the paths.

## License

- Book sources: CC0 1.0
- Roger Allen’s CUDA port: stated public domain on that project
- `nanoexr.h` (Chris Kulla): published on [gist](https://gist.github.com/fpsunflower/e5c99116ff94114d1cbe). The ntv sample notes that permission for that header is being confirmed with the author
- This repository: MIT, copyright 2026 Ivan DeWolf
