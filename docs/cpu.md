# RTIOW on the CPU

`rtiow_bvh_cpu` is the same scene as [`rtiow_bvh`](cuda.md), traced on the host. One thread band owns each strip of rows. The default image is a single 640×480 frame, and the program prints how long the trace took.

The shading types live in `samples/CPU/src` because the ntv CUDA headers mark every method `__device__` and pass a `curandState*` into `scatter` and `get_ray`. The bunny load and the instance seating are the CUDA sample’s files. The mesh walk is [`ntm::RayBVH::hit`](ray_bvh.md) on the host.

Sphere centers use the host LCG from `seat.h`, seeded at 1984. The CUDA sample places those spheres with `curand`. The two renders follow one scene recipe and are not a pixel match.

## Citation

The scene, the book, the CUDA port, and `nanoexr` are cited in the [CUDA sample](cuda.md). This target uses the same `nanoexr.h` from `NTV_ROOT/samples/CUDA/src`.

## Build and run

Configure from the repository root. The project still asks for the CUDA toolkit, because the CUDA sample is part of the same CMake list. Roots are in the [README](../README.md).

```
cmake -S . -B build
cmake --build build --target rtiow_bvh_cpu
cd build
./rtiow_bvh_cpu
```

The executable writes `frame_00.exr` in the working directory and prints two times. `scene` is the seating, the sphere list, and the first scene tree. The `frame_00.exr` line is the pixel loop only, from thread start through the last join. The file write sits outside that timer.

A 32-thread run of the default frame traced in about 4.3 seconds on the machine used to check this sample. The bunny in that run was 1176 points, 204 triangles, and 1072 quads.

Defaults in `samples/CPU/src/main.cpp`:

| Setting | Value |
|---------|--------|
| Image | 640×480 |
| Samples per pixel | 16 |
| Bounce limit | 50 |
| Frames | 1 |
| Threads | `std::thread::hardware_concurrency()`, at least 1 and at most the row count |

The constants at the top of `main.cpp` are the knobs. The CUDA run is `nx = 1200`, `ny = 800`, `ns = 100`, `nframes = 120`. With `nframes` above 1 the angle advances `2π / nframes` per frame, the scene tree is rebuilt, and the files are `frame_00.exr` onward. Those names are the same ones `rtiow_bvh` uses.

640×480 is 4:3. The camera keeps a 15° vertical field of view, so this frame is a slightly narrower horizontal slice than the 1200×800 CUDA frame.

## Tracing

Each thread renders a contiguous band of rows. The pixel seed is `1984` mixed with the pixel index, and that pixel’s `Rng` is private, so the bands do not share random state. The bunny tree and the scene tree are read-only during the trace.

`Rng` is the LCG in `seat.h`: `state * 1664525 + 1013904223`, with the top 24 bits scaled to `[0, 1)`.

`color` is the book’s loop with a fixed depth of 50. A scatter that fails returns black. A miss blends white toward `(0.5, 0.7, 1)` on the ray’s y. The sample is accumulated in linear `nvec3f` and divided by the sample count. `nanoexr` writes those floats as RGB. There is no gamma pass. Row 0 is the top of the view, matching the CUDA sample.

## Mesh instance and scene tree

`samples/CPU/src/mesh_instance.h` is the host copy of the CUDA instance. The ray is taken into bunny space with the matrix columns, `hit<ntm::RayQuadGarp>` blends the point normals, and the normal is taken back to world space with the matrix rows.

`samples/CPU/src/bvh.h` is the host copy of the scene tree. The walk is the same 256-entry stack and the same 12-bin surface-area build. A pointer is treated as an internal node when it falls inside the node pool. The comparison goes through `std::less`, which orders pointers from separate allocations. The CUDA walker compares device pointers directly.

`samples/CPU/src` is ahead of `NTV_ROOT/samples/CUDA/src` on the include path. `ray.h`, `sphere.h`, `material.h`, and `camera.h` in this directory are the ones the CPU target compiles. Moving that directory later on the path would pick up the `__device__` headers instead.

## Layout

| File | Role |
|------|------|
| `src/main.cpp` | Scene, thread bands, timer, EXR |
| `src/bvh.h` | Scene tree over spheres and instances |
| `src/mesh_instance.h` | One rigid copy of the bunny tree |
| `src/ray.h`, `hitable.h`, `sphere.h` | Host ray and spheres |
| `src/material.h` | Lambertian, metal, dielectric |
| `src/camera.h` | Thin-lens camera |
| `src/rng.h` | Per-pixel LCG |
| `samples/CUDA/src/bunny_load.*` | Geo read and the host mesh tree |
| `samples/CUDA/src/seat.h` | Instance placement, shared with the CUDA sample |
| ntv `samples/CUDA/src/nvec3f.h`, `nanoexr.h` | Float vector alias and the EXR writer |

## License

Same terms as the [CUDA sample](cuda.md). This target’s shading code is the host form of that scene. The repository license is MIT, copyright 2026 Ivan DeWolf.
