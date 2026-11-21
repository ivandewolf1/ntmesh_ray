#include <iostream>
#include <cstdio>
#include <cmath>
#include <cfloat>
#include <chrono>
#include <thread>
#include <vector>
#include "nvec3f.h"
#include "ray.h"
#include "sphere.h"
#include "mesh_instance.h"
#include "bvh.h"
#include "camera.h"
#include "material.h"
#include "nanoexr.h"
#include "bunny_load.h"
#include "seat.h"

// The final scene from Ray Tracing in One Weekend, on the host.
// Same layout as the CUDA sample: one bunny, three copies, the small-sphere field.
// Default is one 640x480 frame. The CUDA sample is 1200x800, 100 samples, 120 frames.
// Sphere positions use the host LCG, so this picture is not a pixel match for the device run.

// Full CUDA run: nx = 1200, ny = 800, ns = 100, nframes = 120.
static const int nx = 640;
static const int ny = 480;
static const int ns = 16;
static const int nframes = 1;
static const int maxDepth = 50;

nvec3f color(const ray& r, hitable* world, Rng& rng) {
    ray cur_ray = r;
    nvec3f cur_attenuation = nvec3f(1.0f, 1.0f, 1.0f);
    for (int i = 0; i < maxDepth; i++) {
        hit_record rec;
        if (world->hit(cur_ray, 0.001f, FLT_MAX, rec)) {
            ray scattered;
            nvec3f attenuation;
            if (rec.mat_ptr->scatter(cur_ray, rec, attenuation, scattered, rng)) {
                cur_attenuation *= attenuation;
                cur_ray = scattered;
            }
            else {
                return nvec3f(0.0f, 0.0f, 0.0f);
            }
        }
        else {
            nvec3f unit_direction = cur_ray.direction().normalized();
            float t = 0.5f * (unit_direction[1] + 1.0f);
            nvec3f c = (1.0f - t) * nvec3f(1.0f, 1.0f, 1.0f) + t * nvec3f(0.5f, 0.7f, 1.0f);
            return cur_attenuation * c;
        }
    }
    return nvec3f(0.0f, 0.0f, 0.0f);
}

void renderRows(nvec3f* fb, int y0, int y1, const camera* cam, hitable* world) {
    for (int j = y0; j < y1; ++j) {
        for (int i = 0; i < nx; ++i) {
            int pixel_index = j * nx + i;
            Rng rng;
            rng.state = 1984u + (unsigned)pixel_index * 747796405u;
            nvec3f col(0, 0, 0);
            for (int s = 0; s < ns; s++) {
                float u = float(i + rng.uniform()) / float(nx);
                float v = float((ny - 1 - j) + rng.uniform()) / float(ny);
                ray r = cam->get_ray(u, v, rng);
                col += color(r, world, rng);
            }
            col /= float(ns);
            fb[pixel_index] = col;
        }
    }
}

void createWorld(hitable** list, camera*& cam, Rng& rng, const MeshInst* inst, const BunnyMesh& bunny) {
    list[0] = new sphere(nvec3f(0, -1000.0f, -1), 1000,
                         new lambertian(nvec3f(0.5f, 0.5f, 0.5f)));
    int i = 1;
    for (int a = -11; a < 11; a++) {
        for (int b = -11; b < 11; b++) {
            float choose_mat = rng.uniform();
            nvec3f center(a + rng.uniform(), 0.2f, b + rng.uniform());
            if (choose_mat < 0.8f) {
                list[i++] = new sphere(center, 0.2f,
                                       new lambertian(nvec3f(rng.uniform() * rng.uniform(),
                                                             rng.uniform() * rng.uniform(),
                                                             rng.uniform() * rng.uniform())));
            }
            else if (choose_mat < 0.95f) {
                list[i++] = new sphere(center, 0.2f,
                                       new metal(nvec3f(0.5f * (1.0f + rng.uniform()),
                                                        0.5f * (1.0f + rng.uniform()),
                                                        0.5f * (1.0f + rng.uniform())),
                                                 0.5f * rng.uniform()));
            }
            else {
                list[i++] = new sphere(center, 0.2f, new dielectric(1.5f));
            }
        }
    }

    ntm::RayBVH<float> tree;
    tree.nodes = bunny.nodes;
    tree.count = bunny.nnodes;
    for (int k = 0; k < 3; ++k) {
        mesh_instance* m = new mesh_instance();
        m->xform.set(inst[k].xform);
        m->bvh = tree;
        m->pts = bunny.points;
        m->normals = bunny.normals;
        m->tris = bunny.tris;
        m->quads = bunny.quads;
        if (k == 0) m->mat = new dielectric(1.5f);
        else if (k == 1) m->mat = new lambertian(nvec3f(0.4f, 0.2f, 0.1f));
        else m->mat = new metal(nvec3f(0.7f, 0.6f, 0.5f), 0.0f);
        list[i++] = m;
    }

    nvec3f lookfrom(13, 2, 3);
    nvec3f lookat(0, 0, 0);
    float dist_to_focus = 11.0f;
    float aperture = 0.1f;
    cam = new camera(lookfrom,
                     lookat,
                     nvec3f(0, 1, 0),
                     15.0f,
                     float(nx) / float(ny),
                     aperture,
                     dist_to_focus);
}

// New pose: write the three matrices, then rebuild the top tree. The bunny tree stays.
hitable* updateFrame(hitable** list, hitable* world, const MeshInst* inst, int replace) {
    const int nSphere = 22 * 22 + 1;
    if (replace) {
        bvh_world* old = (bvh_world*)world;
        delete[] old->pool;
        delete old;
    }
    for (int k = 0; k < 3; ++k) {
        mesh_instance* m = (mesh_instance*)list[nSphere + k];
        m->xform.set(inst[k].xform);
    }
    const int nLeaf = nSphere + 3;
    nvec3f* bmn = new nvec3f[nLeaf];
    nvec3f* bmx = new nvec3f[nLeaf];
    for (int s = 0; s < nSphere; ++s) sphereBox((sphere*)list[s], bmn[s], bmx[s]);
    for (int k = 0; k < 3; ++k) {
        bmn[nSphere + k] = nvec3f(inst[k].mn[0], inst[k].mn[1], inst[k].mn[2]);
        bmx[nSphere + k] = nvec3f(inst[k].mx[0], inst[k].mx[1], inst[k].mx[2]);
    }
    hitable* built = buildBvh(list, bmn, bmx, nLeaf);
    delete[] bmn;
    delete[] bmx;
    return built;
}

void freeWorld(hitable** list, hitable* world, camera* cam) {
    bvh_world* tree = (bvh_world*)world;
    const int nSphere = 22 * 22 + 1;
    for (int i = 0; i < nSphere; ++i) {
        delete ((sphere*)list[i])->mat_ptr;
        delete list[i];
    }
    for (int i = nSphere; i < nSphere + 3; ++i) {
        delete ((mesh_instance*)list[i])->mat;
        delete list[i];
    }
    delete[] tree->pool;
    delete tree;
    delete cam;
}

double secondsSince(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

int main() {
    int nthreads = (int)std::thread::hardware_concurrency();
    if (nthreads < 1) nthreads = 1;
    if (nthreads > ny) nthreads = ny;

    std::cerr << "Rendering a " << nx << "x" << ny << " image with " << ns << " samples per pixel";
    std::cerr << " on " << nthreads << " threads.\n";

    BunnyMesh bunny;
    if (!loadBunnyMesh(BUNNY_GEO, bunny)) {
        std::cerr << "could not read " << BUNNY_GEO << "\n";
        return 1;
    }
    std::cerr << "bunny " << bunny.npoints << " points, " << bunny.ntris << " triangles, "
              << bunny.nquads << " quads, minY " << bunny.minY
              << (bunny.normals ? ", point normals\n" : "\n");

    MeshInst inst[3];
    unsigned seed = 1984u;
    float zax = unitRand(seed) * 2.f - 1.f;
    float az = unitRand(seed) * 6.2831853f;
    float radial = std::sqrt(1.f - zax * zax);
    nvec3f axis(radial * std::cos(az), radial * std::sin(az), zax);
    float angle = unitRand(seed) * 3.14159265f;

    const int num_hitables = 22 * 22 + 1 + 3;
    hitable** list = new hitable*[num_hitables];
    camera* cam = nullptr;

    Rng sceneRng;
    sceneRng.state = 1984u;
    auto sceneT0 = std::chrono::steady_clock::now();
    seatInstances(inst, bunny, axis, angle);
    createWorld(list, cam, sceneRng, inst, bunny);
    hitable* world = updateFrame(list, nullptr, inst, 0);
    std::cerr << "scene " << secondsSince(sceneT0) << " seconds.\n";

    std::vector<nvec3f> fb((size_t)nx * (size_t)ny);
    for (int frame = 0; frame < nframes; ++frame) {
        if (frame > 0) {
            float frameAngle = angle + frame * (6.2831853f / float(nframes));
            seatInstances(inst, bunny, axis, frameAngle);
            world = updateFrame(list, world, inst, 1);
        }
        auto renderT0 = std::chrono::steady_clock::now();
        std::vector<std::thread> pool;
        pool.reserve((size_t)nthreads);
        for (int t = 0; t < nthreads; ++t) {
            int y0 = ny * t / nthreads;
            int y1 = ny * (t + 1) / nthreads;
            pool.emplace_back(renderRows, fb.data(), y0, y1, cam, world);
        }
        for (int t = 0; t < nthreads; ++t) pool[t].join();
        double renderSec = secondsSince(renderT0);

        char name[32];
        std::snprintf(name, sizeof(name), "frame_%02d.exr", frame);
        Exr(name, nx, ny, reinterpret_cast<const float*>(fb.data()));
        std::cerr << name << " " << renderSec << " seconds.\n";
    }

    freeWorld(list, world, cam);
    delete[] list;
    freeBunnyMesh(bunny);
    return 0;
}
