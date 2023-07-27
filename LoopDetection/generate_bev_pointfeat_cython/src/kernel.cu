#include <stdio.h>
#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

using namespace std;

void __global__ calculate_features(float* pointcloud, float* feature, int featsize, int* neighbors_indices, float* eigens, int size, int k) 
{
    float eps = 0.0;
    int gid = threadIdx.x + blockDim.x*blockIdx.x;
    if(gid >= size) return;
    
    float eig3d[3] = {0.0};
    float eig2d[2] = {0.0};
    float neighborhoodZ[30] = {0.0};
    float diffZ[30] = {0.0};
    float varZ[30] = {0.0};

    float eig3dsum, eig3dprod, eig2dsum;
    eig3dsum = 0.0;
    eig3dprod = 1.0;
    eig2dsum = 0.0;

    for(int i = 0; i < 5; i++)
    {
        if(i < 3){
            eig3d[i] = eigens[gid*5 + i];
            eig3dsum += eig3d[i];
            eig3dprod = eig3dprod * eig3d[i];
        }else{
            eig2d[i-3] = eigens[gid*5 + i];
            eig2dsum += eig2d[i-3];
        }
    }

    float C_, O_, L_, E_, P_, S_, A_, X_, D_, S_2, L_2, vZ_, dZ_, neighborMean;

    // 3d features
    C_ = eig3d[2] / (eig3dsum + eps);
    O_ = pow((eig3dprod / (pow(eig3dsum, 3)+ eps)), 1.0 / 3.0);
    L_ = (eig3d[0] - eig3d[1]) / (eig3d[0] + eps);

    E_ = 0.0;
    for(int i = 0; i < 3; i++){
        E_ += (eig3d[i] / (eig3dsum + eps)) * log(eig3d[i] / (eig3dsum + eps));
    }
    E_ = -E_;
    
    P_ = (eig3d[1] - eig3d[2]) / eig3d[0];
    S_ = eig3d[2] / eig3d[0];
    A_ = (eig3d[0] - eig3d[2]) / eig3d[0];
    X_ = eig3dsum;
    D_ = 3 * k / ((4 * M_PI * eig3dprod)+ eps);
    
    // 2d features
    S_2 = eig2dsum;
    L_2 = eig2d[1] / eig2d[0];

    // features using statistics data
    neighborMean = 0.0;
    float neighborMin = 10000.0;
    for(int i = 0; i < k; i++){
        int neighbor_ind = neighbors_indices[gid*k + i];
        neighborhoodZ[i] = pointcloud[neighbor_ind*3 + 2];        
        neighborMean += neighborhoodZ[i];
        if(neighborhoodZ[i] < neighborMin) neighborMin = neighborhoodZ[i];
    }

    neighborMean = neighborMean / k;
    dZ_ = -100000.0;
    vZ_ = 0.0;

    for(int i = 0; i < k; i++){
        diffZ[i] = neighborhoodZ[i] - neighborMin;
        vZ_ += pow(abs(neighborhoodZ[i] - neighborMean),2);
        if(diffZ[i] > dZ_) dZ_ = diffZ[i];
    }
    vZ_ = vZ_ / k;

    feature[gid*featsize + 0] = C_;
    feature[gid*featsize + 1] = O_;
    feature[gid*featsize + 2] = L_;
    feature[gid*featsize + 3] = E_;
    feature[gid*featsize + 4] = P_;
    feature[gid*featsize + 5] = S_;
    feature[gid*featsize + 6] = A_;
    feature[gid*featsize + 7] = X_;
    feature[gid*featsize + 8] = D_;
    feature[gid*featsize + 9] = S_2;
    feature[gid*featsize + 10] = L_2;
    feature[gid*featsize + 11] = dZ_;
    feature[gid*featsize + 12] = vZ_;
    __syncthreads();  

}

void __global__ point2gridmap(float* point, float* feat_map, float* max_h, int* x_vec, int* y_vec, int* height, int d_size, int max_length, int max_height, int num_x, int num_y, int num_height, int featsize) 
{
    int gid = threadIdx.x + blockDim.x*blockIdx.x;

    if(gid >= d_size) return;

    float gap_x, gap_y, gap_height;

    gap_x = 2.0 * (float)max_length / (float)num_x;
    gap_y = 2.0 * (float)max_length/(float)num_y;
    gap_height = 2.0 * (float)max_height/ (float)num_height;

    float x, y, z;
    x = point[gid];
    y = point[gid + d_size];
    z = point[gid + 2 * d_size];

    
    if(x == 0.0)
        x = 0.0001;
    if(y == 0.0)
        y = 0.0001;
    if(z == 0.0)
        z = 0.0001;

    if(x > 1.0)
        x = 0.9999;
    if(y > 1.0)
        y = 0.9999;
    if(z > 1.0)
        z = 0.9999;

    if(x < -1.0)
        x = -0.9999;
    if(y < -1.0)
        y = -0.9999;
    if(z < -1.0)
        z = -0.9999;

    int idx_ring = floor((x + 1.0) / gap_x);
    int idx_sector = floor((y + 1.0) / gap_y);
    int idx_height = floor((z + 1.0) / gap_height);

    height[gid] = idx_height;
    x_vec[gid] = idx_ring;
    y_vec[gid] = idx_sector;

    // max feature
    for (int j = 0; j < featsize; j++)
    {
        if(max_h[featsize*(idx_sector + idx_ring * num_y) + j] < point[gid + j*d_size])
        {
            feat_map[featsize*(idx_sector + idx_ring * num_y  + idx_height * num_y * num_x) + j] = point[gid + j*d_size];
            max_h[featsize*(idx_sector + idx_ring * num_y) + j] = point[gid + j*d_size];
        }
    }

    __syncthreads();  
}

