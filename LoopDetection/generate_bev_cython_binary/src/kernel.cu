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

void __global__ point2gridmap(float* point, int* x_vec, int* y_vec, int* height, int d_size, int max_length, int max_height, int num_x, int num_y, int num_height) 
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
    
    __syncthreads();  
}

