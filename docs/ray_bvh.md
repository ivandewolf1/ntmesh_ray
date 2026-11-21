# Ray BVH

Header: [`src/ntmesh_ray_bvh.h`](../src/ntmesh_ray_bvh.h). Namespace `ntm`.

A bounding-volume hierarchy over triangles and quads. One leaf is one triangle or one quad. N-gons are not stored. Build the tree from the point array and the index lists an `ntmesh` already keeps, after `triangulateNgons()` if the mesh has n-gons.

```cpp
#include "nvec3.h"
#include "ntmesh_ray_bvh.h"

ntm::RayBVH<float> tree;
tree.build(mesh.points.data, mesh.triangleIndices.data, mesh.triangleCount(),
           mesh.quadIndices.data, mesh.quadCount());

ntm::RayHit<float> rec;
bool hit = tree.hit<ntm::RayQuadGarp>(
    mesh.points.data, mesh.triangleIndices.data, mesh.quadIndices.data,
    origin, direction, tmin, tmax, rec, normals);

tree.clear();
```

`pts`, `tris`, and `quads` on `hit` are the arrays passed to `build`. The tree stores indices, not vertex positions.

## Types

### `RayHit<T>`

Filled on a hit. A miss leaves the record as the caller left it.

| Field | Meaning |
|-------|---------|
| `t` | Ray parameter of the closest hit inside `(tmin, tmax)` |
| `p` | Hit position, `origin + direction * t` |
| `n` | Unit normal, front face |
| `prim` | Triangle index or quad index inside its own list |
| `quad` | `0` for a triangle, `1` for a quad |

### `RayNode<T>`

One node in the array `RayBVH::nodes`.

| Field | Meaning |
|-------|---------|
| `mn`, `mx` | Axis-aligned box |
| `left`, `right` | Child indices. A leaf has `left < 0` |
| `prim` | Leaf primitive index. Internal nodes store `-1` |
| `quad` | `0` triangle leaf, `1` quad leaf |

### `RayBVH<T>`

| Field | Meaning |
|-------|---------|
| `nodes` | Node array, or null |
| `count` | Live node count |

### `RayQuadKind`

| Value | `hit` method | Quad test |
|-------|----------------|-----------|
| `RayQuadSplit` (`0`) | `hit<ntm::RayQuadSplit>` | Triangles `(0,1,2)` and `(0,2,3)`. The nearer hit wins |
| `RayQuadGarp` (`1`) | `hit<ntm::RayQuadGarp>` | Bilinear patch through the four corners. A ray may meet the patch twice. The nearer `t` wins |

Corner `0,1,2,3` is boundary order, the same order `ntmesh` stores in `quadIndices`.

## Build

```cpp
void build(const ntv::nvec3<T>* pts, const int* tris, int ntris,
           const int* quads, int nquads);
```

Host only. `build` calls `clear` first. A triangle count or a quad count of zero may pass a null index pointer. A missing point array, or a positive count with a null index pointer, leaves the tree empty. A failed allocation leaves the tree empty.

The split is a 12-bin surface-area cut on primitive centroids. A range that does not accept a bin cut, and any range past a depth of 48, falls back to a median split on the widest centroid axis. Each leaf holds one primitive. The node array is allocated for `2 * (ntris + nquads)` nodes. `count` is the number actually used.

Leaf boxes are padded by a fraction of their extent plus a small absolute margin, so a polygon that lies on a box face still reports a hit.

## Hit

```cpp
template <int Method>
NTL_HD bool hit(const ntv::nvec3<T>* pts, const int* tris, const int* quads,
                ntv::nvec3<T> origin, ntv::nvec3<T> direction,
                T tmin, T tmax, RayHit<T>& rec,
                const ntv::nvec3<T>* pointN = nullptr) const;
```

Closest intersection with `t` in `(tmin, tmax)`. The walk keeps an explicit stack of 128 node indices, so a device build does not depend on recursive calls.

`pointN`, when non-null, is one normal per point. The geometric normal is replaced by the blend at the hit barycentrics: triangle weights on a triangle, bilinear weights on a `RayQuadGarp` quad, and the triangle weights of the winning half on a `RayQuadSplit` quad. A blend that faces against the polygon normal is flipped. A degenerate blend keeps the geometric normal.

The geometric normal is the front normal, clockwise when seen from the outside, and it matches `ntmesh::faceNormal` for the same winding. A hit from the back returns that same direction.

An empty tree, a null node array, or a null point array misses.

## Ownership

`clear()` frees `nodes` and sets `count` to 0. The type has no destructor. A plain copy copies the pointer. `clear` one of them.

```cpp
ntm::RayBVH<float> share = tree;   // share.nodes == tree.nodes
tree.clear();                      // frees the array for both
```

## CUDA

Define the macro before the include, the same way ntv and ntmesh do:

```cpp
#define NTL_HD __host__ __device__
#include "ntmesh_ray_bvh.h"
```

The samples set it from CMake on CUDA translation units: `-DNTL_HD=__host__\ __device__`. `hit` and the ray–box and ray–polygon tests are marked. `build` and `clear` stay on the host. Upload the node array, the points, and the index lists, then point a device `RayBVH` at the device node pointer. That is what `rtiow_bvh` does.

See the [overview](overview.md) for how the samples share one tree across three instances.
