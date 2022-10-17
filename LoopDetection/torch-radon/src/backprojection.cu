#include <iostream>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_fp16.h>

#include "utils.h"
#include "texture.h"
#include "backprojection.h"

template<bool parallel_beam, int channels, typename T>
__global__ void
radon_backward_kernel(T *__restrict__ output, cudaTextureObject_t texture, const float *__restrict__ angles,
                      const VolumeCfg vol_cfg, const ProjectionCfg proj_cfg) {
    // Calculate image coordinates
    const uint x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint y = blockIdx.y * blockDim.y + threadIdx.y;
    const uint tid = threadIdx.y * blockDim.x + threadIdx.x;

    const float cx = vol_cfg.width / 2.0f;
    const float cy = vol_cfg.height / 2.0f;
    const float cr = proj_cfg.det_count_u / 2.0f;

    const float dx = (float(x) - cx) * vol_cfg.sx + vol_cfg.dx + 0.5f;
    const float dy = (float(y) - cy) * vol_cfg.sy + vol_cfg.dy + 0.5f;

    const float ids = __fdividef(1.0f, proj_cfg.det_spacing_u);
    const float sdx = dx * ids;
    const float sdy = dy * ids;

    const int base = x + vol_cfg.width * (y + vol_cfg.height * blockIdx.z);
    const int pitch = vol_cfg.width * vol_cfg.height * blockDim.z * gridDim.z;

    // keep sin and cos packed toghether to save one memory load in the main loop
    __shared__ float2 sincos[4096];

    for (int i = tid; i < proj_cfg.n_angles; i += 256) {
        float2 tmp;
        tmp.x = -__sinf(angles[i]);
        tmp.y = __cosf(angles[i]);
        sincos[i] = tmp;
    }
    __syncthreads();

    if (x < vol_cfg.width && y < vol_cfg.height) {
        float accumulator[channels];
#pragma unroll
        for (int i = 0; i < channels; i++) accumulator[i] = 0.0f;

        if (parallel_beam) {
            const int n_angles = proj_cfg.n_angles;
            
            // keep a float version of i to avoid expensive int2float conversions inside the main loop
            float fi = 0.5f;
            #pragma unroll(16)
            for (int i = 0; i < n_angles; i++) {
                float j = sincos[i].y * sdx + sincos[i].x * sdy + cr;
                if (channels == 1) {
                    accumulator[0] += tex2DLayered<float>(texture, j, fi, blockIdx.z);
                } else {
                    // read 4 values at the given position and accumulate
                    float4 read = tex2DLayered<float4>(texture, j, fi, blockIdx.z);
                    accumulator[0] += read.x;
                    accumulator[1] += read.y;
                    accumulator[2] += read.z;
                    accumulator[3] += read.w;
                }
                fi += 1.0f;
            }
        } else {
            const float k = proj_cfg.s_dist + proj_cfg.d_dist;
            const int n_angles = proj_cfg.n_angles;
            
            // keep a float version of i to avoid expensive int2float conversions inside the main loop
            float fi = 0.5f;
            #pragma unroll(16)
            for (int i = 0; i < n_angles; i++) {
                float iden;
                float den = fmaf(sincos[i].y, -dy, sincos[i].x * dx + proj_cfg.s_dist);
                
                // iden = __fdividef(k, den);
                asm("div.approx.ftz.f32 %0, %1, %2;" : "=f"(iden) : "f"(k), "f"(den));

                float j = (sincos[i].y * sdx + sincos[i].x * sdy) * iden + cr;

                if (channels == 1) {
                    accumulator[0] += tex2DLayered<float>(texture, j, fi, blockIdx.z) * iden;
                } else {
                    // read 4 values at the given position and accumulate
                    float4 read = tex2DLayered<float4>(texture, j, fi, blockIdx.z);
                    accumulator[0] += read.x * iden;
                    accumulator[1] += read.y * iden;
                    accumulator[2] += read.z * iden;
                    accumulator[3] += read.w * iden;
                }
                fi += 1.0f;
            }
        }

#pragma unroll
        for (int b = 0; b < channels; b++) {
            output[base + b * pitch] = accumulator[b] * ids;
        }
    }
}


template<typename T>
void radon_backward_cuda(
        const T *x, const float *angles, T *y, TextureCache &tex_cache,
        const VolumeCfg &vol_cfg, const ProjectionCfg &proj_cfg, const ExecCfg &exec_cfg,
        const int batch_size, const int device
) {
    constexpr bool is_float = std::is_same<T, float>::value;
    constexpr int precision = is_float ? PRECISION_FLOAT : PRECISION_HALF;
    const int channels = exec_cfg.get_channels(batch_size);

    // copy x into CUDA Array (allocating it if needed) and bind to texture
    Texture *tex = tex_cache.get(
        {device, batch_size / channels, proj_cfg.n_angles, proj_cfg.det_count_u, true, channels, precision}
    );
    tex->put(x);

    // dim3 block_dim(16, 16);
    dim3 block_dim = exec_cfg.get_block_dim();
    dim3 grid_dim = exec_cfg.get_grid_size(vol_cfg.width, vol_cfg.height, batch_size / channels);

    // Invoke kernel
    if (proj_cfg.projection_type == FANBEAM) {
        if (channels == 1) {
            radon_backward_kernel<false, 1> << < grid_dim, block_dim >> >
                                                                      ((float*)y, tex->texture, angles, vol_cfg, proj_cfg);
        } else {
            if (is_float) {
                radon_backward_kernel<false, 4> << < grid_dim, block_dim >> >
                                                                          ((float*)y, tex->texture, angles, vol_cfg, proj_cfg);
            } else {
                radon_backward_kernel<false, 4> << < grid_dim, block_dim >> >
                                                                          ((__half *) y, tex->texture, angles, vol_cfg, proj_cfg);
            }
        }
    } else {
        if (channels == 1) {
            radon_backward_kernel<true, 1> << < grid_dim, block_dim >> >
                                                                     ((float*)y, tex->texture, angles, vol_cfg, proj_cfg);
        } else {
            if (is_float) {
                radon_backward_kernel<true, 4> << < grid_dim, block_dim >> >
                                                                         ((float*)y, tex->texture, angles, vol_cfg, proj_cfg);
            } else {
                radon_backward_kernel<true, 4> << < grid_dim, block_dim >> >
                                                                         ((__half *) y, tex->texture, angles, vol_cfg, proj_cfg);
            }
        }
    }
}

template void
radon_backward_cuda<float>(const float *x, const float *angles, float *y, TextureCache &tex_cache,
                           const VolumeCfg &vol_cfg, const ProjectionCfg &proj_cfg, const ExecCfg &exec_cfg,
                           const int batch_size, const int device);

template void radon_backward_cuda<unsigned short>(const unsigned short *x, const float *angles, unsigned short *y,
                                                  TextureCache &tex_cache,
                                                  const VolumeCfg &vol_cfg, const ProjectionCfg &proj_cfg,
                                                  const ExecCfg &exec_cfg,
                                                  const int batch_size, const int device);


template<int channels, typename T>
__global__ void
radon_backward_kernel_3d(T *__restrict__ output, cudaTextureObject_t texture, const float *__restrict__ angles,
                         const VolumeCfg vol_cfg, const ProjectionCfg proj_cfg) {
    // TODO consider pitch and initial_z
    // Calculate volume coordinates
    const uint x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint y = blockIdx.y * blockDim.y + threadIdx.y;
    const uint z = blockIdx.z * blockDim.z + threadIdx.z;
    const uint tid = (threadIdx.z * blockDim.y + threadIdx.y) * blockDim.x + threadIdx.x;

    const uint index = (z * vol_cfg.height + y) * vol_cfg.width + x;
    const uint pitch = vol_cfg.depth * vol_cfg.height * vol_cfg.width;

    const float cx = vol_cfg.width / 2.0f;
    const float cy = vol_cfg.height / 2.0f;
    const float cz = vol_cfg.depth / 2.0f;
    const float cu = proj_cfg.det_count_u / 2.0f;
    const float cv = proj_cfg.det_count_v / 2.0f;
    
    const float dx = (float(x) - cx) * vol_cfg.sx + vol_cfg.dx + 0.5f;
    const float dy = (float(y) - cy) * vol_cfg.sy + vol_cfg.dy + 0.5f;
    const float dz = (float(z) - cz) * vol_cfg.sz + vol_cfg.dz + 0.5f - proj_cfg.initial_z;

    const float inv_det_spacing_u = __fdividef(1.0f, proj_cfg.det_spacing_u);
    const float inv_det_spacing_v = __fdividef(1.0f, proj_cfg.det_spacing_v);
    const float ids = inv_det_spacing_u * inv_det_spacing_v;

    const float sdx = dx * inv_det_spacing_u;
    const float sdy = dy * inv_det_spacing_u;
    const float sdz = dz * inv_det_spacing_v;
    const float pitch_speed = -proj_cfg.pitch * 0.1591549f * inv_det_spacing_v;

    // using a single float3 array creates 3 memory loads, while float2+float ==> 2 loads
    __shared__ float2 sincos[4096];
    __shared__ float pitch_dz[4096];

    for (int i = tid; i < proj_cfg.n_angles; i += 256) {
        float2 tmp;
        tmp.x = __sinf(angles[i]);
        tmp.y = __cosf(angles[i]);
        pitch_dz[i] = angles[i] * pitch_speed;
        sincos[i] = tmp;
    }
    __syncthreads();

    if (x < vol_cfg.width && y < vol_cfg.height && z < vol_cfg.depth) {
        float accumulator[channels];
        
        #pragma unroll
        for (int i = 0; i < channels; i++) accumulator[i] = 0.0f;

        const float k = proj_cfg.s_dist + proj_cfg.d_dist;

        #pragma unroll(4)
        for (int i = 0; i < proj_cfg.n_angles; i++) {
            float alpha = fmaf(-dx, sincos[i].x, proj_cfg.s_dist) + sincos[i].y * dy;
            float beta = sincos[i].y * sdx + sincos[i].x * sdy;

            // float k_over_alpha = __fdividef(k, alpha);
            float k_over_alpha;
            asm("div.approx.ftz.f32 %0, %1, %2;" : "=f"(k_over_alpha) : "f"(k), "f"(alpha));

            float u = k_over_alpha * beta + cu;
            float v = k_over_alpha * (sdz + pitch_dz[i]) + cv;
            float scale = k_over_alpha * k_over_alpha;

            if (channels == 1) {
                accumulator[0] += tex2DLayered<float>(texture, u, v, i) * scale;
            } else {
                // read 4 values at the given position and accumulate
                float4 read = tex2DLayered<float4>(texture, u, v, i);
                accumulator[0] += read.x * scale;
                accumulator[1] += read.y * scale;
                accumulator[2] += read.z * scale;
                accumulator[3] += read.w * scale;
            }
        }

#pragma unroll
        for (int b = 0; b < channels; b++) {
            output[b * pitch + index] = accumulator[b] * ids;
        }
    }
}


template<typename T>
void radon_backward_cuda_3d(
        const T *x, const float *angles, T *y, TextureCache &tex_cache,
        const VolumeCfg &vol_cfg, const ProjectionCfg &proj_cfg, const ExecCfg &exec_cfg, const int batch_size,
        const int device
) {
    constexpr bool is_float = std::is_same<T, float>::value;
    constexpr int precision = is_float ? PRECISION_FLOAT : PRECISION_HALF;
    const int channels = exec_cfg.get_channels(batch_size);

    Texture *tex = tex_cache.get(
            {device, proj_cfg.n_angles, proj_cfg.det_count_v, proj_cfg.det_count_u, true, channels, precision});

    dim3 grid_dim = exec_cfg.get_grid_size(vol_cfg.width, vol_cfg.height, vol_cfg.depth);
    const dim3 block_dim = exec_cfg.get_block_dim();

    for (int i = 0; i < batch_size; i += channels) {
        T *local_y = &y[i * vol_cfg.depth * vol_cfg.height * vol_cfg.width];
        tex->put(&x[i * proj_cfg.n_angles * proj_cfg.det_count_v * proj_cfg.det_count_u]);

        // Invoke kernel
        if (channels == 1) {
            radon_backward_kernel_3d<1> << < grid_dim, block_dim >> >
                                                       (local_y, tex->texture, angles, vol_cfg, proj_cfg);
        } else {
            if (is_float) {
                radon_backward_kernel_3d<4> << < grid_dim, block_dim >> >
                                                           (local_y, tex->texture, angles, vol_cfg, proj_cfg);
            } else {
                radon_backward_kernel_3d<4> << < grid_dim, block_dim >> >
                                                           ((__half *) local_y, tex->texture, angles, vol_cfg, proj_cfg);
            }
        }
    }
}

template void radon_backward_cuda_3d<float>(const float *x, const float *angles, float *y, TextureCache &tex_cache,
                                            const VolumeCfg &vol_cfg, const ProjectionCfg &proj_cfg,
                                            const ExecCfg &exec_cfg,
                                            const int batch_size, const int device);

template void radon_backward_cuda_3d<unsigned short>(const unsigned short *x, const float *angles, unsigned short *y,
                                                     TextureCache &tex_cache,
                                                     const VolumeCfg &vol_cfg, const ProjectionCfg &proj_cfg,
                                                     const ExecCfg &exec_cfg,
                                                     const int batch_size, const int device);