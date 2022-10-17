#include "noise.h"
#include "rmath.h"
#include <iostream>

__global__ void initialize_random_states(curandState *state, const uint seed) {
    const uint sequence_id = threadIdx.x + blockIdx.x * blockDim.x;
    curand_init(seed, sequence_id, 0, &state[sequence_id]);
}

template<bool approximate>
__global__ void
radon_sinogram_noise(float *sinogram, curandState *state, const float signal, const float density_normalization,
                     const uint width, const uint height) {
    const uint x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint y = blockIdx.y * blockDim.y + threadIdx.y;
    const uint tid = y * blockDim.x * gridDim.x + x;
    const uint y_step = blockDim.y * gridDim.y;

    if (tid < 128 * 1024) {
        // load curand state in local memory
        curandState localState = state[tid];

        // loop down the sinogram adding noise
        for (uint yy = y; yy < height; yy += y_step) {
            if (x < width) {
                uint pos = yy * width + x;
                // measured signal = signal * exp(-sinogram[pos])
                // then apply poisson noise
                float mu = __expf(signal - sinogram[pos] / density_normalization);
                float reading;
                if (approximate) {
                    float var = __fsqrt_rn(mu);
                    reading = fmaxf(curand_normal(&localState) * var + mu, 1.0f);
                } else {
                    reading = fmaxf(curand_poisson(&localState, mu), 1.0f);
                }

                // convert back to sinogram scale
                sinogram[pos] = fmaxf((signal - __logf(reading)), 0.0f) * density_normalization;
            }
        }

        // save curand state back in global memory
        state[tid] = localState;
    }
}

__global__ void radon_emulate_readings(const float *sinogram, int *readings, curandState *state, const float signal,
                                       const float density_normalization, const uint width, const uint height) {
    const uint x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint y = blockIdx.y * blockDim.y + threadIdx.y;
    const uint tid = y * blockDim.x * gridDim.x + x;
    const uint y_step = blockDim.y * gridDim.y;

    // load curand state in local memory
    curandState localState = state[tid];

    // loop down the sinogram adding noise
    for (uint yy = y; yy < height; yy += y_step) {
        uint pos = yy * width + x;
        // measured signal = signal * exp(-sinogram[pos])
        // then apply poisson noise
        float mu = __expf(signal - sinogram[pos] / density_normalization);
        readings[pos] = curand_poisson(&localState, mu);
    }

    // save curand state back in global memory
    state[tid] = localState;
}

RadonNoiseGenerator::RadonNoiseGenerator(const uint _seed) : seed(_seed) {
    this->states = (curandState **) malloc(sizeof(curandState * ) * 8);
    for (int i = 0; i < 8; i++) this->states[i] = nullptr;
}


void RadonNoiseGenerator::set_seed(const uint seed, int device) {
    initialize_random_states << < 128, 1024 >> > (this->get(device), seed);
}

void RadonNoiseGenerator::set_seed(const uint seed) {
    this->seed = seed;
    for (int i = 0; i < 8; i++) {
        if (this->states[i] != nullptr) {
            this->set_seed(seed, i);
        }
    }
}

curandState *RadonNoiseGenerator::get(int device) {
    if (this->states[device] == nullptr) {
        checkCudaErrors(cudaSetDevice(device));
#ifdef VERBOSE
        std::cout << "[TORCH RADON] Allocating Random states on device " << device << std::endl;
#endif

        // allocate random states
        checkCudaErrors(cudaMalloc((void **) &states[device], 128 * 1024 * sizeof(curandState)));
        this->set_seed(seed, device);
    }
    return this->states[device];
}

void RadonNoiseGenerator::add_noise(float *sinogram, const float signal, const float density_normalization,
                                    const bool approximate, const uint width, const uint height, int device) {
    checkCudaErrors(cudaSetDevice(device));

    if (approximate) {
        radon_sinogram_noise<true> << < dim3(width / 16, 8 * 1024 / width), dim3(16, 16) >> >
                                                                            (sinogram, this->get(
                                                                                    device), signal, density_normalization, width, height);
    } else {
        radon_sinogram_noise<false> << < dim3(width / 16, 8 * 1024 / width), dim3(16, 16) >> >
                                                                             (sinogram, this->get(
                                                                                     device), signal, density_normalization, width, height);
    }
}

void RadonNoiseGenerator::emulate_readings(const float *sinogram, int *readings, const float signal,
                                           const float density_normalization, const uint width, const uint height,
                                           int device) {
    checkCudaErrors(cudaSetDevice(device));

    radon_emulate_readings << < dim3(width / 16, 8 * 1024 / width), dim3(16, 16) >> >
                                                                    (sinogram, readings, this->get(
                                                                            device), signal, density_normalization, width, height);
}

void RadonNoiseGenerator::free() {
    for (int i = 0; i < 8; i++) {
        if (this->states[i] != nullptr) {
#ifdef VERBOSE
            std::cout << "[TORCH RADON] Freeing Random states on device " << i << " " << this->states[i] << std::endl;
#endif
            checkCudaErrors(cudaSetDevice(i));
            checkCudaErrors(cudaFree(this->states[i]));
            this->states[i] = nullptr;
#ifdef VERBOSE
            std::cout << "[TORCH RADON] DONE Freeing Random states on device " << i << std::endl;
#endif
        }
    }
}

RadonNoiseGenerator::~RadonNoiseGenerator() {
    this->free();
}

template<bool use_shared>
__global__ void
lookup_kernel(const int *readings, float *result, const float *lookup_table, const uint lookup_size, const uint size) {
    const uint tid = blockIdx.x * blockDim.x + threadIdx.x;
    const uint step = blockDim.x * gridDim.x;

    const float *lookup = lookup_table;
    if (use_shared) {
        extern __shared__ float lookup_cache[];
        for (int i = threadIdx.x; i < lookup_size; i += 256) {
            lookup_cache[i] = lookup_table[i];
        }
        __syncthreads();
        lookup = lookup_cache;
    }

    for (uint pos = tid; pos < size; pos += step) {
        int index = min(readings[pos], lookup_size - 1);
        result[pos] = lookup[index];
    }
}

void readings_lookup_cuda(const int *x, float *y, const float *lookup_table, const uint lookup_size, const uint width,
                          const uint height, int device) {
    checkCudaErrors(cudaSetDevice(device));

    // don't exceed max shared memory size
    if (lookup_size * sizeof(float) < 64 * 1024) {
        lookup_kernel<true> << < dim3((width * height) / 256), dim3(256), lookup_size * sizeof(float) >> >
                                                                          (x, y, lookup_table, lookup_size,
                                                                                  width * height);
    } else {
        lookup_kernel<false> << < dim3((width * height) / 256), dim3(256), 0 >> >
                                                                           (x, y, lookup_table, lookup_size,
                                                                                   width * height);
    }

}

template<bool use_shared>
__global__ void
lookup_kernel_multilevel(const int *readings, float *result, const float *lookup_table, const int *levels,
                         const uint lookup_size,
                         const uint size) {
    const uint batch = blockIdx.x;
    extern __shared__ float lookup_cache[];
    const int level = levels[batch];

    const float *lookup = &lookup_table[level * lookup_size];
    if (use_shared) {
        for (int i = threadIdx.x; i < lookup_size; i += 256) {
            lookup_cache[i] = lookup[i];
        }
        __syncthreads();
        lookup = lookup_cache;
    }

    for (uint pos = batch * size + threadIdx.x; pos < (batch + 1) * size; pos += 256) {
        int index = min(readings[pos], lookup_size - 1);
        result[pos] = lookup[index];
    }
}

void readings_lookup_multilevel_cuda(const int *x, float *y, const float *lookup_table, const int *levels,
                                     const uint lookup_size, const uint batch, const uint width,
                                     const uint height, int device) {
    checkCudaErrors(cudaSetDevice(device));

    // don't exceed max shared memory size
    if (lookup_size * sizeof(float) < 64 * 1024) {
        lookup_kernel_multilevel<true> << < dim3(batch), dim3(256),
                lookup_size * sizeof(float) >> >
                (x, y, lookup_table, levels, lookup_size,
                        width * height);
    } else {
        lookup_kernel_multilevel<false> << < dim3((width * height) / 256, batch), dim3(256), 0 >> >
                                                                                             (x, y, lookup_table, levels,
                                                                                                     lookup_size,
                                                                                                     width * height);
    }
}


__inline__ __device__ void warpReduce(volatile float *sdata, const int tid) {
    sdata[tid] += sdata[tid + 32];
    sdata[tid] += sdata[tid + 16];
    sdata[tid] += sdata[tid + 8];
    sdata[tid] += sdata[tid + 4];
    sdata[tid] += sdata[tid + 2];
    sdata[tid] += sdata[tid + 1];
}

__inline__ __device__ void warpReduce(volatile double *sdata, const int tid) {
    sdata[tid] += sdata[tid + 32];
    sdata[tid] += sdata[tid + 16];
    sdata[tid] += sdata[tid + 8];
    sdata[tid] += sdata[tid + 4];
    sdata[tid] += sdata[tid + 2];
    sdata[tid] += sdata[tid + 1];
}

template<int n_threads, class dtype>
__inline__ __device__ void dualWarpReduce(dtype *sa, dtype *sb, const int tid) {
    if (n_threads >= 512) {
        if (tid < 256) {
            sa[tid] += sa[tid + 256];
            sb[tid] += sb[tid + 256];
        }
        __syncthreads();
    }
    if (n_threads >= 256) {
        if (tid < 128) {
            sa[tid] += sa[tid + 128];
            sb[tid] += sb[tid + 128];
        }
        __syncthreads();
    }
    if (n_threads >= 128) {
        if (tid < 64) {
            sa[tid] += sa[tid + 64];
            sb[tid] += sb[tid + 64];
        }
        __syncthreads();
    }

    if (tid < 32) {
        warpReduce(sa, tid);
        warpReduce(sb, tid);
    }
}

__global__ void
ab_kernel(const float *x, const int size, float *ab, const float signal, const float eps, const int k,
          const float nlog) {
    __shared__ float sa[512];
    __shared__ float sb[512];
    const int tid = threadIdx.x;

    float a = 0.0f;
    float b = 0.0f;
    for (int i = tid; i < size; i += 512) {
        float y = x[i];
        float v = exp(float(k * (signal - y) - nlog - exp(signal - y)));
        if (y <= eps) {
            a += v;
        } else {
            b += v;
        }
    }

    sa[tid] = a;
    sb[tid] = b;
    __syncthreads();

    dualWarpReduce<512>(sa, sb, tid);

    if (tid == 0) {
        ab[0] = sa[0];
        ab[1] = sb[0];
    }
}

std::pair<float, float> compute_ab(const float *x, const int size, const float signal, const float eps, const int k,
                                   const int device) {
    checkCudaErrors(cudaSetDevice(device));

    float ab_cpu[2];
    float *ab;
    checkCudaErrors(cudaMalloc(&ab, 2 * sizeof(float)));

    float nlog = k * (signal - eps) - rosh::exp(signal - eps);

    ab_kernel << < 1, 512 >> > (x, size, ab, signal, eps, k, nlog);

    checkCudaErrors(cudaMemcpy(ab_cpu, ab, 2 * sizeof(float), cudaMemcpyDeviceToHost));
    checkCudaErrors(cudaFree(ab));

    return std::make_pair(ab_cpu[0], ab_cpu[1]);
}

template<bool variance>
__global__ void
compute_lookup_table_kernel(const float *x, const float *g_weights, const float *mean_estimator, float *res,
                            const float *log_factorial, const float *border_w,
                            const int size, const int weights_size,
                            const float signal, const int scale) {
    constexpr int n_threads = 256;
    __shared__ double sp[n_threads];
    __shared__ double spv[n_threads];
    __shared__ double weights[256];
    //__shared__ float lgf[256];

    const int bin = blockIdx.x;
    const int tid = threadIdx.x;

    const int r = (weights_size - scale) / 2;
    const int start_index = max(r - bin * scale, 0);

    double estimated_mean;
    if (variance) {
        estimated_mean = mean_estimator[bin];
    }

    // load weights into shared memory
    if (tid < weights_size) {
        weights[tid] = g_weights[tid];

        // TODO last bin border
        // add border_w to weights (probability that a value in the first bin is made negative by Gaussian noise and than clamped back to zero)
        if(bin == 0 && tid < scale){
            weights[start_index + tid] += border_w[tid];
        }

        weights[tid] = log(weights[tid]) - log_factorial[max(bin*scale - r + tid, 0)];
    }
    __syncthreads();


    double p = 0.0f;
    double pv = 0.0f;
    for (int i = tid; i < size; i += n_threads) {
        // read sinogram value and precompute
        const double y = x[i];
        const double delta = signal - y;
        const double constant_part = bin*scale*delta - exp(delta);

        double prob = 0.0f;
        for (int j = start_index; j < weights_size; j += 1) {
            double tmp = (j - r) * delta + constant_part + weights[j];
            prob += exp(tmp);
        }
        p += prob;
        if (variance) {
            pv += prob * (y - estimated_mean) * (y - estimated_mean);
        } else {
            pv += prob * y;
        }
    }

    sp[tid] = p;
    spv[tid] = pv;
    __syncthreads();

    dualWarpReduce<n_threads>(sp, spv, tid);

    if (tid == 0) {
        if (variance) {
            res[bin] = sqrt(spv[0] / (sp[0] + 1e-9));
        } else {
            res[bin] = spv[0] / (sp[0] + 1e-9);
        }
    }
}

void compute_lookup_table(const float *x, const float *weights, float *y_mean, float *y_var, const float *log_factorial,
                          const float *border_w, const int size,
                          const int weights_size,
                          const float signal, const int bins, const int scale, const int device) {
    checkCudaErrors(cudaSetDevice(device));

    compute_lookup_table_kernel<false> << < bins, 256 >> >
                                                     (x, weights, NULL, y_mean, log_factorial, border_w, size, weights_size, signal, scale);
    compute_lookup_table_kernel<true> << < bins, 256 >> >
                                                    (x, weights, y_mean, y_var, log_factorial, border_w, size, weights_size, signal, scale);
}

__global__ void emulate_readings_kernel(const float *sinogram, int *readings, curandState *state, const float signal,
                                        const float normal_std, const int k, const int bins, const uint width,
                                        const uint height) {
    const uint x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint y = blockIdx.y * blockDim.y + threadIdx.y;
    const uint tid = y * blockDim.x * gridDim.x + x;
    const uint y_step = blockDim.y * gridDim.y;

    // load curand state in local memory
    curandState localState = state[tid];

    // loop down the sinogram adding noise
    for (uint yy = y; yy < height; yy += y_step) {
        uint pos = yy * width + x;
        // ideal measured signal = signal * exp(-sinogram[pos])
        float mu = __expf(signal - sinogram[pos]);
        // apply Poisson noise and add Gaussian noise
        int reading = int(curand_poisson(&localState, mu)) + __float2int_rn(curand_normal(&localState) * normal_std);
        // quantize reading clamping in range [0, bins - 1]
        readings[pos] = max(0, min(reading / k, bins - 1));
    }

    // save curand state back in global memory
    state[tid] = localState;
}


void RadonNoiseGenerator::emulate_readings_new(const float *sinogram, int *readings, const float signal,
                                               const float normal_std, const int k, const int bins,
                                               const uint width, const uint height, int device) {
    checkCudaErrors(cudaSetDevice(device));

    emulate_readings_kernel << < dim3(width / 16, 8 * 1024 / width), dim3(16, 16) >> >
                                                                     (sinogram, readings, this->get(
                                                                             device), signal, normal_std, k, bins, width, height);
}


__global__ void
emulate_readings_multilevel_kernel(const float *sinogram, int *readings, curandState *state, const int *levels,
                                   const float *signal,
                                   const float *normal_std, const int *ks, const int bins, const uint size) {
    const uint batch = blockIdx.x;
    const uint tid = blockIdx.x * blockDim.x + threadIdx.x;

    const int level = levels[batch];
    const float s = signal[level];
    const float ns = normal_std[level];
    const int k = ks[level];

    // load curand state in local memory
    curandState localState = state[tid];

    // loop down the sinogram adding noise
    for (uint pos = batch * size + threadIdx.x; pos < (batch + 1) * size; pos += 256) {
        // ideal measured signal = signal * exp(-sinogram[pos])
        float mu = __expf(s - sinogram[pos]);
        // apply Poisson noise and add Gaussian noise
        int reading = __float2int_rn((curand_poisson(&localState, mu) + curand_normal(&localState) * ns)) / k;
        // clamp in range [0, bins - 1]
        readings[pos] = max(0, min(reading, bins - 1));
    }

    // save curand state back in global memory
    state[tid] = localState;
}


void RadonNoiseGenerator::emulate_readings_multilevel(const float *sinogram, int *readings, const float *signal,
                                                      const float *normal_std, const int *k, const int *levels,
                                                      const uint bins, const uint batch, const uint width,
                                                      const uint height, const uint device) {
    checkCudaErrors(cudaSetDevice(device));

//TODO handle batch > 512
    emulate_readings_multilevel_kernel << < dim3(batch), dim3(256) >> > (sinogram, readings, this->get(device), levels,
            signal, normal_std, k, bins, width * height);
}