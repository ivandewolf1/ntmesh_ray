#ifndef BVHH
#define BVHH

#include "sphere.h"
#include <functional>

// Binary BVH over scene leaves. Each leaf brings its own box, so a leaf can be
// a sphere or any other hitable. Internal nodes sit in one pool so a ray walk
// can tell a node from a leaf by pointer range and keep an explicit stack.

class bvh_node : public hitable {
public:
    bvh_node() {}
    virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const;
    hitable* left;
    hitable* right;
    nvec3f mn, mx;
    nvec3f lmn, lmx;
    nvec3f rmn, rmx;
};

class bvh_world : public hitable {
public:
    bvh_world() {}
    virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const;
    bvh_node* pool;
    int poolN;
    hitable* root;
};

void sphereBox(sphere* s, nvec3f& mn, nvec3f& mx) {
    nvec3f e(s->radius, s->radius, s->radius);
    mn = s->center - e;
    mx = s->center + e;
}

nvec3f boxCenter(const nvec3f& mn, const nvec3f& mx) {
    return (mn + mx) * 0.5f;
}

void expandBox(nvec3f& mn, nvec3f& mx, const nvec3f& amn, const nvec3f& amx) {
    for (int a = 0; a < 3; ++a) {
        if (amn[a] < mn[a]) mn[a] = amn[a];
        if (amx[a] > mx[a]) mx[a] = amx[a];
    }
}

float boxArea(const nvec3f& mn, const nvec3f& mx) {
    float dx = mx[0] - mn[0];
    float dy = mx[1] - mn[1];
    float dz = mx[2] - mn[2];
    if (dx < 0.f) dx = 0.f;
    if (dy < 0.f) dy = 0.f;
    if (dz < 0.f) dz = 0.f;
    return 2.f * (dx * dy + dy * dz + dz * dx);
}

// Entry distance along the ray. A miss returns a value well past any hit.
float boxEntry(const nvec3f& mn, const nvec3f& mx, const ray& r, float tmin, float tmax) {
    nvec3f o = r.origin();
    nvec3f d = r.direction();
    for (int a = 0; a < 3; ++a) {
        float da = d[a];
        float oa = o[a];
        if (da == 0.f) {
            if (oa < mn[a] || oa > mx[a]) return 1.0e30f;
            continue;
        }
        float inv = 1.0f / da;
        float t0 = (mn[a] - oa) * inv;
        float t1 = (mx[a] - oa) * inv;
        if (inv < 0.f) {
            float tmp = t0;
            t0 = t1;
            t1 = tmp;
        }
        if (t0 > tmin) tmin = t0;
        if (t1 < tmax) tmax = t1;
        if (tmax <= tmin) return 1.0e30f;
    }
    return tmin;
}

bool bvh_node::hit(const ray& r, float tmin, float tmax, hit_record& rec) const {
    if (boxEntry(mn, mx, r, tmin, tmax) >= 1.0e30f) return false;
    bool any = false;
    if (left && left->hit(r, tmin, tmax, rec)) {
        any = true;
        tmax = rec.t;
    }
    if (right && right->hit(r, tmin, tmax, rec)) any = true;
    return any;
}

bool bvh_world::hit(const ray& r, float tmin, float tmax, hit_record& rec) const {
    if (!root) return false;
    if (poolN <= 0) return root->hit(r, tmin, tmax, rec);

    const hitable* poolBegin = pool;
    const hitable* poolEnd = pool + poolN;
    hitable* stack[256];
    int sp = 0;
    stack[sp++] = root;
    bool any = false;
    while (sp > 0) {
        hitable* h = stack[--sp];
        if (!std::less<const hitable*>{}(h, poolBegin) && std::less<const hitable*>{}(h, poolEnd)) {
            bvh_node* node = (bvh_node*)h;
            if (boxEntry(node->mn, node->mx, r, tmin, tmax) >= 1.0e30f) continue;
            float tl = boxEntry(node->lmn, node->lmx, r, tmin, tmax);
            float tr = boxEntry(node->rmn, node->rmx, r, tmin, tmax);
            hitable* first = node->left;
            hitable* second = node->right;
            float tf = tl;
            float ts = tr;
            if (tr < tl) {
                first = node->right;
                second = node->left;
                tf = tr;
                ts = tl;
            }
            if (ts < 1.0e30f && sp < 256) stack[sp++] = second;
            if (tf < 1.0e30f && sp < 256) stack[sp++] = first;
        } else if (h->hit(r, tmin, tmax, rec)) {
            any = true;
            tmax = rec.t;
        }
    }
    return any;
}

int centroidBin(float c, float c0, float span) {
    int b = (int)(((c - c0) / span) * 12.f);
    if (b < 0) b = 0;
    if (b > 11) b = 11;
    return b;
}

void sortRange(const nvec3f* bmn, const nvec3f* bmx, int* idx, int begin, int end, int axis) {
    for (int i = begin + 1; i < end; ++i) {
        int key = idx[i];
        float kc = boxCenter(bmn[key], bmx[key])[axis];
        int j = i;
        while (j > begin) {
            float jc = boxCenter(bmn[idx[j - 1]], bmx[idx[j - 1]])[axis];
            if (jc <= kc) break;
            idx[j] = idx[j - 1];
            --j;
        }
        idx[j] = key;
    }
}

// Binned surface-area split. A single huge primitive may peel off on its own.
// Otherwise the cut stays away from the ends so the tree cannot become a chain.
// bmn and bmx are indexed like list. outMn and outMx receive this node's box.
hitable* buildRange(hitable** list, int* idx, const nvec3f* bmn, const nvec3f* bmx,
                               int begin, int end, bvh_node* pool, int* used,
                               nvec3f& outMn, nvec3f& outMx) {
    int n = end - begin;
    if (n <= 0) return nullptr;
    if (n == 1) {
        outMn = bmn[idx[begin]];
        outMx = bmx[idx[begin]];
        return list[idx[begin]];
    }

    nvec3f mn = bmn[idx[begin]];
    nvec3f mx = bmx[idx[begin]];
    nvec3f cmn = boxCenter(mn, mx);
    nvec3f cmx = cmn;
    for (int i = begin + 1; i < end; ++i) {
        expandBox(mn, mx, bmn[idx[i]], bmx[idx[i]]);
        nvec3f c = boxCenter(bmn[idx[i]], bmx[idx[i]]);
        for (int a = 0; a < 3; ++a) {
            if (c[a] < cmn[a]) cmn[a] = c[a];
            if (c[a] > cmx[a]) cmx[a] = c[a];
        }
    }
    float parentArea = boxArea(mn, mx);

    int bestAxis = -1;
    int bestBin = -1;
    float bestCost = 1.0e30f;
    for (int axis = 0; axis < 3; ++axis) {
        float span = cmx[axis] - cmn[axis];
        if (!(span > 0.f)) continue;
        int counts[12];
        nvec3f binMn[12], binMx[12];
        int filled[12];
        for (int b = 0; b < 12; ++b) {
            counts[b] = 0;
            filled[b] = 0;
        }
        for (int i = begin; i < end; ++i) {
            int id = idx[i];
            int b = centroidBin(boxCenter(bmn[id], bmx[id])[axis], cmn[axis], span);
            if (!filled[b]) {
                binMn[b] = bmn[id];
                binMx[b] = bmx[id];
                filled[b] = 1;
            } else {
                expandBox(binMn[b], binMx[b], bmn[id], bmx[id]);
            }
            counts[b] += 1;
        }
        int leftCount = 0;
        nvec3f lmn, lmx;
        int leftFilled = 0;
        for (int b = 0; b < 11; ++b) {
            if (counts[b]) {
                if (!leftFilled) {
                    lmn = binMn[b];
                    lmx = binMx[b];
                    leftFilled = 1;
                } else {
                    expandBox(lmn, lmx, binMn[b], binMx[b]);
                }
                leftCount += counts[b];
            }
            int rightCount = n - leftCount;
            if (leftCount == 0 || rightCount == 0) continue;
            nvec3f rmn, rmx;
            int rightFilled = 0;
            for (int k = b + 1; k < 12; ++k) {
                if (!counts[k]) continue;
                if (!rightFilled) {
                    rmn = binMn[k];
                    rmx = binMx[k];
                    rightFilled = 1;
                } else {
                    expandBox(rmn, rmx, binMn[k], binMx[k]);
                }
            }
            if (!rightFilled) continue;
            int smaller = leftCount < rightCount ? leftCount : rightCount;
            float leftArea = boxArea(lmn, lmx);
            float rightArea = boxArea(rmn, rmx);
            float leafArea = leftCount == 1 ? leftArea : rightArea;
            int peel = (leftCount == 1 || rightCount == 1) && leafArea > 0.5f * parentArea;
            int wide = n <= 32 || smaller * 8 >= n;
            if (!peel && !wide) continue;
            float cost = leftCount * leftArea + rightCount * rightArea;
            if (cost < bestCost) {
                bestCost = cost;
                bestAxis = axis;
                bestBin = b;
            }
        }
    }

    int mid;
    if (bestAxis >= 0) {
        float span = cmx[bestAxis] - cmn[bestAxis];
        mid = begin;
        for (int i = begin; i < end; ++i) {
            int id = idx[i];
            int b = centroidBin(boxCenter(bmn[id], bmx[id])[bestAxis], cmn[bestAxis], span);
            if (b <= bestBin) {
                int tmp = idx[mid];
                idx[mid] = idx[i];
                idx[i] = tmp;
                ++mid;
            }
        }
        if (mid == begin || mid == end) bestAxis = -1;
    }
    if (bestAxis < 0) {
        int axis = 0;
        float span = cmx[0] - cmn[0];
        for (int a = 1; a < 3; ++a) {
            float s = cmx[a] - cmn[a];
            if (s > span) {
                span = s;
                axis = a;
            }
        }
        sortRange(bmn, bmx, idx, begin, end, axis);
        mid = begin + n / 2;
    }

    int id = *used;
    *used = id + 1;
    bvh_node* node = &pool[id];
    nvec3f lmn, lmx, rmn, rmx;
    node->left = buildRange(list, idx, bmn, bmx, begin, mid, pool, used, lmn, lmx);
    node->right = buildRange(list, idx, bmn, bmx, mid, end, pool, used, rmn, rmx);
    node->lmn = lmn;
    node->lmx = lmx;
    node->rmn = rmn;
    node->rmx = rmx;
    node->mn = lmn;
    node->mx = lmx;
    expandBox(node->mn, node->mx, rmn, rmx);
    outMn = node->mn;
    outMx = node->mx;
    return node;
}

bvh_world* buildBvh(hitable** list, const nvec3f* bmn, const nvec3f* bmx, int n) {
    bvh_world* world = new bvh_world();
    world->pool = nullptr;
    world->poolN = 0;
    world->root = nullptr;
    if (n <= 0) return world;
    if (n == 1) {
        world->root = list[0];
        return world;
    }
    int* order = new int[n];
    for (int i = 0; i < n; ++i) order[i] = i;
    bvh_node* pool = new bvh_node[n];
    int used = 0;
    nvec3f rootMn, rootMx;
    world->pool = pool;
    world->root = buildRange(list, order, bmn, bmx, 0, n, pool, &used, rootMn, rootMx);
    world->poolN = used;
    delete[] order;
    return world;
}

#endif
