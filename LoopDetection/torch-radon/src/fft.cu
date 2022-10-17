#include <iostream>
#include <cufft.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <curand.h>

#include "utils.h"
#include "fft.h"
#include "log.h"

FFTConfig::FFTConfig(int dv, int r, int c):device(dv), rows(r), cols(c){}

bool FFTConfig::operator==(const FFTConfig &o) const{
    return device == o.device && rows == o.rows && cols == o.cols;
}

std::ostream &operator<<(std::ostream &os, FFTConfig const &cfg) {
    return os << "(device: " << cfg.device << ", rows: " << cfg.rows << ", cols: " << cfg.cols << ")";
}

FFTStructures::FFTStructures(const FFTConfig &_cfg) : cfg(_cfg) {
    LOG_INFO("Allocating FFT " << cfg);

    // create plans for FFT and iFFT
    cufftSafeCall(cufftPlan1d(&forward_plan, cfg.cols, CUFFT_R2C, cfg.rows));
    cufftSafeCall(cufftPlan1d(&back_plan, cfg.cols, CUFFT_C2R, cfg.rows));
}

bool FFTStructures::matches(const FFTConfig &k) const{
    return k == cfg;
}

FFTStructures::~FFTStructures() {
    // if (padded_data != nullptr) {
        LOG_DEBUG("Freeing FFT " << cfg);

        cufftSafeCall(cufftDestroy(forward_plan));
        cufftSafeCall(cufftDestroy(back_plan));
    // }
}


void FFT(FFTCache& fft_cache, const float *x, int device, int rows, int cols, float* y){
    FFTStructures* fft = fft_cache.get({device, rows, cols});
    checkCudaErrors(cudaSetDevice(device));

    cufftSafeCall(cufftExecR2C(fft->forward_plan, (cufftReal *)x, (cufftComplex *)y));
}

void iFFT(FFTCache& fft_cache, const float *x, int device, int rows, int cols, float* y){
    FFTStructures* fft = fft_cache.get({device, rows, cols});
    checkCudaErrors(cudaSetDevice(device));

    cufftSafeCall(cufftExecC2R(fft->back_plan, (cufftComplex *)x, (cufftReal *)y));
}