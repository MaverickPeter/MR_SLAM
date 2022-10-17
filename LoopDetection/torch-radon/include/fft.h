#ifndef TORCH_RADON_FFT_H
#define TORCH_RADON_FFT_H

#include <cuda.h>
#include <cuda_runtime.h>
#include "utils.h"
#include "cache.h"

class FFTConfig
{
public:
    int device;
    int rows;
    int cols;

    FFTConfig(int dv, int r, int c);

    bool operator==(const FFTConfig &o) const;
};

class FFTStructures
{
public:
    cufftHandle forward_plan, back_plan;
    FFTConfig cfg;

    FFTStructures(const FFTConfig &_cfg);

    bool matches(const FFTConfig &k) const;

    ~FFTStructures();
};

typedef Cache<FFTConfig, FFTStructures> FFTCache;

void FFT(FFTCache& fft_cache, const float *x, int device, int rows, int cols, float* y);

void iFFT(FFTCache& fft_cache, const float *x, int device, int rows, int cols, float* y);

#endif
