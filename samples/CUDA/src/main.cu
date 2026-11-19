#include <iostream>
#include <cstdio>
#include <cmath>
#include <time.h>
#include <float.h>
#include <curand_kernel.h>
#include "nvec3f.h"
#include "nmat4.h"
#include "ray.h"
#include "sphere.h"
#include "mesh_instance.h"
#include "bvh.h"
#include "camera.h"
#include "material.h"
#include "nanoexr.h"
#include "bunny_load.h"

// The final scene from Ray Tracing in One Weekend, on CUDA.
// The three large spheres are copies of one bunny. Two sit on the ground.
// The center copy spins about the origin, lifted 0.25, one pose per frame.

#define checkCudaErrors(val) check_cuda( (val), #val, __FILE__, __LINE__ )

void check_cuda(cudaError_t result, char const *const func, const char *const file, int const line) {
    if (result) {
        std::cerr << "CUDA error = " << static_cast<unsigned int>(result) << " at " <<
            file << ":" << line << " '" << func << "' \n";
        cudaDeviceReset();
        exit(99);
    }
}

__device__ nvec3f color(const ray& r, hitable **world, curandState *local_rand_state) {
    ray cur_ray = r;
    nvec3f cur_attenuation = nvec3f(1.0,1.0,1.0);
    for(int i = 0; i < 50; i++) {
        hit_record rec;
        if ((*world)->hit(cur_ray, 0.001f, FLT_MAX, rec)) {
            ray scattered;
            nvec3f attenuation;
            if(rec.mat_ptr->scatter(cur_ray, rec, attenuation, scattered, local_rand_state)) {
                cur_attenuation *= attenuation;
                cur_ray = scattered;
            }
            else {
                return nvec3f(0.0,0.0,0.0);
            }
        }
        else {
            nvec3f unit_direction = cur_ray.direction().normalized();
            float t = 0.5f*(unit_direction[1] + 1.0f);
            nvec3f c = (1.0f-t)*nvec3f(1.0, 1.0, 1.0) + t*nvec3f(0.5, 0.7, 1.0);
            return cur_attenuation * c;
        }
    }
    return nvec3f(0.0,0.0,0.0);
}

__global__ void rand_init(curandState *rand_state) {
    if (threadIdx.x == 0 && blockIdx.x == 0)
        curand_init(1984, 0, 0, rand_state);
}

__global__ void render_init(int max_x, int max_y, curandState *rand_state) {
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    int j = threadIdx.y + blockIdx.y * blockDim.y;
    if((i >= max_x) || (j >= max_y)) return;
    int pixel_index = j*max_x + i;
    curand_init(1984, pixel_index, 0, &rand_state[pixel_index]);
}

__global__ void render(nvec3f *fb, int max_x, int max_y, int ns, camera **cam, hitable **world, curandState *rand_state) {
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    int j = threadIdx.y + blockIdx.y * blockDim.y;
    if((i >= max_x) || (j >= max_y)) return;
    int pixel_index = j*max_x + i;
    curandState local_rand_state = rand_state[pixel_index];
    nvec3f col(0,0,0);
    for(int s=0; s < ns; s++) {
        float u = float(i + curand_uniform(&local_rand_state)) / float(max_x);
        float v = float((max_y - 1 - j) + curand_uniform(&local_rand_state)) / float(max_y);
        ray r = (*cam)->get_ray(u, v, &local_rand_state);
        col += color(r, world, &local_rand_state);
    }
    rand_state[pixel_index] = local_rand_state;
    col /= float(ns);
    fb[pixel_index] = col;
}

#define RND (curand_uniform(&local_rand_state))

__global__ void create_world(hitable **d_list, camera **d_camera, int nx, int ny,
                               curandState *rand_state, const MeshInst *inst, nvec3f *pts, nvec3f *normals,
                               int *tris, int *quads, ntm::RayNode<float> *nodes, int nnodes) {
    if (threadIdx.x == 0 && blockIdx.x == 0) {
        curandState local_rand_state = *rand_state;
        d_list[0] = new sphere(nvec3f(0,-1000.0,-1), 1000,
                               new lambertian(nvec3f(0.5, 0.5, 0.5)));
        int i = 1;
        for(int a = -11; a < 11; a++) {
            for(int b = -11; b < 11; b++) {
                float choose_mat = RND;
                nvec3f center(a+RND,0.2,b+RND);
                if(choose_mat < 0.8f) {
                    d_list[i++] = new sphere(center, 0.2,
                                             new lambertian(nvec3f(RND*RND, RND*RND, RND*RND)));
                }
                else if(choose_mat < 0.95f) {
                    d_list[i++] = new sphere(center, 0.2,
                                             new metal(nvec3f(0.5f*(1.0f+RND), 0.5f*(1.0f+RND), 0.5f*(1.0f+RND)), 0.5f*RND));
                }
                else {
                    d_list[i++] = new sphere(center, 0.2, new dielectric(1.5));
                }
            }
        }
        *rand_state = local_rand_state;

        ntm::RayBVH<float> bunny;
        bunny.nodes = nodes;
        bunny.count = nnodes;
        for (int k = 0; k < 3; ++k) {
            mesh_instance* m = new mesh_instance();
            m->xform.set(inst[k].xform);
            m->bvh = bunny;
            m->pts = pts;
            m->normals = normals;
            m->tris = tris;
            m->quads = quads;
            if (k == 0) m->mat = new dielectric(1.5f);
            else if (k == 1) m->mat = new lambertian(nvec3f(0.4f, 0.2f, 0.1f));
            else m->mat = new metal(nvec3f(0.7f, 0.6f, 0.5f), 0.0f);
            d_list[i++] = m;
        }

        nvec3f lookfrom(13,2,3);
        nvec3f lookat(0,0,0);
        float dist_to_focus = 11.0;// (lookfrom-lookat).length();
        float aperture = 0.1;
        *d_camera   = new camera(lookfrom,
                                 lookat,
                                 nvec3f(0,1,0),
                                 15.0,
                                 float(nx)/float(ny),
                                 aperture,
                                 dist_to_focus);
    }
}

// New pose: write the three matrices, then rebuild the top tree. The bunny tree stays.
__global__ void update_frame(hitable **d_list, hitable **d_world, const MeshInst *inst, int replace) {
    if (threadIdx.x != 0 || blockIdx.x != 0) return;
    const int nSphere = 22 * 22 + 1;
    if (replace) {
        bvh_world* old = (bvh_world*)(*d_world);
        delete[] old->pool;
        delete old;
    }
    for (int k = 0; k < 3; ++k) {
        mesh_instance* m = (mesh_instance*)d_list[nSphere + k];
        m->xform.set(inst[k].xform);
    }
    const int nLeaf = nSphere + 3;
    nvec3f* bmn = new nvec3f[nLeaf];
    nvec3f* bmx = new nvec3f[nLeaf];
    for (int s = 0; s < nSphere; ++s) sphereBox((sphere*)d_list[s], bmn[s], bmx[s]);
    for (int k = 0; k < 3; ++k) {
        bmn[nSphere + k] = nvec3f(inst[k].mn[0], inst[k].mn[1], inst[k].mn[2]);
        bmx[nSphere + k] = nvec3f(inst[k].mx[0], inst[k].mx[1], inst[k].mx[2]);
    }
    *d_world = buildBvh(d_list, bmn, bmx, nLeaf);
    delete[] bmn;
    delete[] bmx;
}

__global__ void free_world(hitable **d_list, hitable **d_world, camera **d_camera) {
    bvh_world* world = (bvh_world*)(*d_world);
    const int nSphere = 22 * 22 + 1;
    for (int i = 0; i < nSphere; ++i) {
        delete ((sphere*)d_list[i])->mat_ptr;
        delete d_list[i];
    }
    for (int i = nSphere; i < nSphere + 3; ++i) {
        delete ((mesh_instance*)d_list[i])->mat;
        delete d_list[i];
    }
    delete[] world->pool;
    delete world;
    delete *d_camera;
}

float unitRand(unsigned& seed) {
    seed = seed * 1664525u + 1013904223u;
    return (seed >> 8) * (1.f / 16777216.f);
}

// Center copy spins about the origin. The other two stay seated on the ground.
void seatInstances(MeshInst inst[3], const BunnyMesh& bunny, const nvec3f& axis, float angle) {
    float seat = -bunny.minY;
    float placeX[3] = {4.f, -4.f, 0.f};
    float placeY[3] = {seat + 0.25f, seat, seat};
    for (int k = 0; k < 3; ++k) {
        ntv::nmat4<float> M;
        if (k == 0) M.setRotation(axis, angle);
        else M.setRotation(nvec3f(0.f, 1.f, 0.f), angle);
        M.translate(nvec3f(placeX[k], placeY[k], 0.f));
        inst[k].xform.set(M);
        nvec3f p0 = M * bunny.points[0];
        nvec3f mn = p0;
        nvec3f mx = p0;
        for (int p = 1; p < bunny.npoints; ++p) {
            nvec3f wp = M * bunny.points[p];
            for (int a = 0; a < 3; ++a) {
                if (wp[a] < mn[a]) mn[a] = wp[a];
                if (wp[a] > mx[a]) mx[a] = wp[a];
            }
        }
        nvec3f pad(1e-3f, 1e-3f, 1e-3f);
        mn -= pad;
        mx += pad;
        for (int a = 0; a < 3; ++a) {
            inst[k].mn[a] = mn[a];
            inst[k].mx[a] = mx[a];
        }
    }
}

int main() {
    int nx = 1200;
    int ny = 800;
    int ns = 100;
    int tx = 8;
    int ty = 8;

    std::cerr << "Rendering a " << nx << "x" << ny << " image with " << ns << " samples per pixel ";
    std::cerr << "in " << tx << "x" << ty << " blocks.\n";

    checkCudaErrors(cudaDeviceSetLimit(cudaLimitStackSize, 65536));

    int num_pixels = nx*ny;
    size_t fb_size = num_pixels*sizeof(nvec3f);

    nvec3f *fb;
    checkCudaErrors(cudaMallocManaged((void **)&fb, fb_size));

    curandState *d_rand_state;
    checkCudaErrors(cudaMalloc((void **)&d_rand_state, num_pixels*sizeof(curandState)));
    curandState *d_rand_state2;
    checkCudaErrors(cudaMalloc((void **)&d_rand_state2, 1*sizeof(curandState)));

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
    const int nframes = 120;
    seatInstances(inst, bunny, axis, angle);

    nvec3f* d_pts = nullptr;
    nvec3f* d_nrm = nullptr;
    int* d_tris = nullptr;
    int* d_quads = nullptr;
    ntm::RayNode<float>* d_nodes = nullptr;
    MeshInst* d_inst = nullptr;
    checkCudaErrors(cudaMalloc((void**)&d_pts, bunny.npoints * sizeof(nvec3f)));
    checkCudaErrors(cudaMemcpy(d_pts, bunny.points, bunny.npoints * sizeof(nvec3f), cudaMemcpyHostToDevice));
    if (bunny.normals) {
        checkCudaErrors(cudaMalloc((void**)&d_nrm, bunny.npoints * sizeof(nvec3f)));
        checkCudaErrors(cudaMemcpy(d_nrm, bunny.normals, bunny.npoints * sizeof(nvec3f), cudaMemcpyHostToDevice));
    }
    if (bunny.ntris > 0) {
        checkCudaErrors(cudaMalloc((void**)&d_tris, bunny.ntris * 3 * sizeof(int)));
        checkCudaErrors(cudaMemcpy(d_tris, bunny.tris, bunny.ntris * 3 * sizeof(int), cudaMemcpyHostToDevice));
    }
    if (bunny.nquads > 0) {
        checkCudaErrors(cudaMalloc((void**)&d_quads, bunny.nquads * 4 * sizeof(int)));
        checkCudaErrors(cudaMemcpy(d_quads, bunny.quads, bunny.nquads * 4 * sizeof(int), cudaMemcpyHostToDevice));
    }
    checkCudaErrors(cudaMalloc((void**)&d_nodes, bunny.nnodes * sizeof(ntm::RayNode<float>)));
    checkCudaErrors(cudaMemcpy(d_nodes, bunny.nodes, bunny.nnodes * sizeof(ntm::RayNode<float>), cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMalloc((void**)&d_inst, 3 * sizeof(MeshInst)));
    checkCudaErrors(cudaMemcpy(d_inst, inst, 3 * sizeof(MeshInst), cudaMemcpyHostToDevice));

    const int num_hitables = 22*22+1+3;
    hitable **d_list;
    checkCudaErrors(cudaMalloc((void **)&d_list, num_hitables*sizeof(hitable *)));
    hitable **d_world;
    checkCudaErrors(cudaMalloc((void **)&d_world, sizeof(hitable *)));
    camera **d_camera;
    checkCudaErrors(cudaMalloc((void **)&d_camera, sizeof(camera *)));

    cudaEvent_t ev0, ev1, ev2;
    checkCudaErrors(cudaEventCreate(&ev0));
    checkCudaErrors(cudaEventCreate(&ev1));
    checkCudaErrors(cudaEventCreate(&ev2));

    rand_init<<<1,1>>>(d_rand_state2);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    checkCudaErrors(cudaEventRecord(ev0));
    create_world<<<1,1>>>(d_list, d_camera, nx, ny, d_rand_state2,
                          d_inst, d_pts, d_nrm, d_tris, d_quads, d_nodes, bunny.nnodes);
    checkCudaErrors(cudaEventRecord(ev1));
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaEventSynchronize(ev1));
    float build_ms = 0.f;
    checkCudaErrors(cudaEventElapsedTime(&build_ms, ev0, ev1));
    std::cerr << "scene " << build_ms / 1000.f << " seconds.\n";

    dim3 blocks(nx/tx+1,ny/ty+1);
    dim3 threads(tx,ty);
    for (int frame = 0; frame < nframes; ++frame) {
        if (frame > 0) {
            float frameAngle = angle + frame * (6.2831853f / float(nframes));
            seatInstances(inst, bunny, axis, frameAngle);
            checkCudaErrors(cudaMemcpy(d_inst, inst, 3 * sizeof(MeshInst), cudaMemcpyHostToDevice));
        }
        checkCudaErrors(cudaEventRecord(ev1));
        update_frame<<<1,1>>>(d_list, d_world, d_inst, frame > 0);
        checkCudaErrors(cudaGetLastError());
        checkCudaErrors(cudaDeviceSynchronize());
        render_init<<<blocks, threads>>>(nx, ny, d_rand_state);
        checkCudaErrors(cudaGetLastError());
        render<<<blocks, threads>>>(fb, nx, ny, ns, d_camera, d_world, d_rand_state);
        checkCudaErrors(cudaEventRecord(ev2));
        checkCudaErrors(cudaGetLastError());
        checkCudaErrors(cudaEventSynchronize(ev2));
        float frame_ms = 0.f;
        checkCudaErrors(cudaEventElapsedTime(&frame_ms, ev1, ev2));
        char name[32];
        std::snprintf(name, sizeof(name), "frame_%02d.exr", frame);
        Exr(name, nx, ny, reinterpret_cast<const float*>(fb));
        std::cerr << name << " " << frame_ms / 1000.f << " seconds.\n";
    }

    checkCudaErrors(cudaDeviceSynchronize());
    free_world<<<1,1>>>(d_list, d_world, d_camera);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaFree(d_camera));
    checkCudaErrors(cudaFree(d_world));
    checkCudaErrors(cudaFree(d_list));
    checkCudaErrors(cudaFree(d_rand_state));
    checkCudaErrors(cudaFree(d_rand_state2));
    checkCudaErrors(cudaFree(d_pts));
    checkCudaErrors(cudaFree(d_nrm));
    checkCudaErrors(cudaFree(d_tris));
    checkCudaErrors(cudaFree(d_quads));
    checkCudaErrors(cudaFree(d_nodes));
    checkCudaErrors(cudaFree(d_inst));
    checkCudaErrors(cudaFree(fb));
    freeBunnyMesh(bunny);
    cudaEventDestroy(ev0);
    cudaEventDestroy(ev1);
    cudaEventDestroy(ev2);

    cudaDeviceReset();
}
