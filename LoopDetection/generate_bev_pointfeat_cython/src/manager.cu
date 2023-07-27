/*
This is the central piece of code. This file implements a class
(interface in manager.hh) that takes data in on the cpu side, copies
it to the gpu, and exposes functions (increment and retreive) that let
you perform actions with the GPU

This class will get translated into python via swig
*/

#include <kernel.cu>
#include <manager.hh>
#include <assert.h>
#include <iostream>
#include <chrono>
#include <thread>
using namespace std;


GPUTransformer::GPUTransformer (float* point_host_, int size_, int* x_, int* y_, int* height_, int max_length_, int max_height_, int num_x_, int num_y_, int num_height_, int featsize_) {
  h_max_length = max_length_;
  h_max_height_ = max_height_;
  h_num_height = num_height_;
  featsize = featsize_;

  // grid size in x and y dir
  h_num_x = num_x_;
  h_num_y = num_y_;

  size = size_* featsize * sizeof(float);
  d_size = size_;
  d_grid_size = num_x_ * num_y_ ;
  d_feat_size = d_grid_size * featsize * sizeof(float);

  float h_feature_map[d_grid_size * featsize] = {0.0};
  float h_max_handler[d_grid_size * featsize] = {0.0};

  cudaMalloc((void**) &point_device, size);
  cudaMalloc((void**) &x_ind, d_size * sizeof(int));
  cudaMalloc((void**) &y_ind, d_size * sizeof(int));
  cudaMalloc((void**) &z_ind, d_size * sizeof(int));
  cudaMalloc((void**) &d_feature_map, d_feat_size);
  cudaMalloc((void**) &d_max_handler, d_feat_size);

  cudaMemcpy(point_device, point_host_, size, cudaMemcpyHostToDevice);
  cudaMemcpy(d_max_handler, h_max_handler, d_feat_size, cudaMemcpyHostToDevice);
  cudaMemcpy(d_feature_map, h_feature_map, d_feat_size, cudaMemcpyHostToDevice);
  cudaMemcpy(y_ind, y_, d_size * sizeof(int), cudaMemcpyHostToDevice);
  cudaMemcpy(z_ind, height_, d_size * sizeof(int), cudaMemcpyHostToDevice);
  cudaMemcpy(x_ind, x_, d_size * sizeof(int), cudaMemcpyHostToDevice);
}


void GPUTransformer::transform() {
  dim3 blockSize(256);
  dim3 gridSize((d_size + blockSize.x - 1) / blockSize.x);
  point2gridmap<<<gridSize, blockSize>>>(point_device, d_feature_map, d_max_handler, x_ind, y_ind, z_ind, d_size, h_max_length, h_max_height_, h_num_x, h_num_y, h_num_height, featsize);
  cudaDeviceSynchronize();
}


void GPUTransformer::retreive(float* point_transformed) {
  // float max_h[d_grid_size*featsize] = {0.0};
  cudaMemcpy(point_transformed, d_feature_map, d_feat_size, cudaMemcpyDeviceToHost);
  // for(int i = 0; i < d_grid_size*featsize; i++) {
  //   point_transformed[i] = max_h[i];
  //   cout << "max_h[" << i << "] = " << max_h[i] << endl;
  // }
  // int x_h[d_size] = {0};
  // int y_h[d_size] = {0};
  // int height_h[d_size] = {0};
  // float max_h[d_grid_size*featsize] = {0.0};
  // int count_h[d_grid_size*featsize] = {0};
  // double max_h = -1000.0;

  // cudaMemcpy(x_h, x_ind, d_size * sizeof(int), cudaMemcpyDeviceToHost);
  // cudaMemcpy(y_h, y_ind, d_size * sizeof(int), cudaMemcpyDeviceToHost);
  // cudaMemcpy(height_h, z_ind, d_size * sizeof(int), cudaMemcpyDeviceToHost);

  // for (int i = 0; i < d_size; i++)
  // {
  //   for (int j = 0; j < featsize; j++)
  //   {
  //     if(max_h[featsize*(y_h[i] + x_h[i] * h_num_y) + j] < point_host[i + j*d_size]){
  //       point_transformed[featsize*(y_h[i] + x_h[i] * h_num_y + height_h[i] * h_num_y * h_num_x) + j] = point_host[i + j*d_size];
  //       max_h[featsize*(y_h[i] + x_h[i] * h_num_y) + j] = point_host[i + j*d_size];
  //     }
  //   }

  //   // average feature
  //   // for (int j = 0; j < featsize; j++)
  //   // {
  //   //   point_transformed[featsize*(y_h[i] + x_h[i] * h_num_y + height_h[i] * h_num_y * h_num_x) + j] = (count_h[featsize*(y_h[i] + x_h[i] * h_num_y) + j] * point_transformed[featsize*(y_h[i] + x_h[i] * h_num_y + height_h[i] * h_num_y * h_num_x) + j] + point_host[i + j*d_size]) / (count_h[featsize*(y_h[i] + x_h[i] * h_num_y) + j] + 1);
  //   //   count_h[featsize*(y_h[i] + x_h[i] * h_num_y) + j]++;
  //   // }

  //   // Max Intensity
  //   // if(max_h[y_h[i] + x_h[i] * h_num_y] < point_host[i + 2 * d_size]){
  //   //   point_transformed[3*(y_h[i] + x_h[i] * h_num_y + height_h[i] * h_num_y * h_num_x) + 2] = point_host[i + 2 * d_size];
  //   //   max_h[y_h[i] + x_h[i] * h_num_y] = point_host[i + 2 * d_size];
  //   // }

  //   // Density
  //   // point_transformed[3*(y_h[i] + x_h[i] * h_num_y + height_h[i] * h_num_y * h_num_x) + 2]++;

  //   // Occupancy
  //   // point_transformed[3*(y_h[i] + x_h[i] * h_num_y + height_h[i] * h_num_y * h_num_x) + 2] = 1;
    
  //   // Max height
  //   // if(max_h < point_host[i + 2 * d_size]){
  //   //   point_transformed[3*(y_h[i] + x_h[i] * h_num_y + height_h[i] * h_num_y * h_num_x) + 2] = point_host[i + 2 * d_size];
  //   //   max_h = point_host[i + 2 * d_size];
  //   // }
  // }

  cudaFree(point_device);
  cudaFree(z_ind);
  cudaFree(y_ind);
  cudaFree(x_ind);
  cudaFree(d_feature_map);
  cudaFree(d_max_handler);
}


GPUTransformer::~GPUTransformer() {

}



GPUFeatureExtractor::GPUFeatureExtractor(float* point_host_, int size_, int featsize_, int k_, int* neighbors_indices, float* eigens) {
  featsize = featsize_;
  size = size_;
  k = k_;

  float neighborhoodZ[k] = {0.0};
  float diffZ[k] = {0.0};
  float varZ[k] = {0.0};
  float h_feature[featsize * size] = {0.0};
  

  cudaMalloc((void**)&d_neighbors_indices, size * k * sizeof(int)); 
  cudaMalloc((void**)&d_pointcloud, size * 3 * sizeof(float)); 
  cudaMalloc((void**)&d_eigens, size * 5 * sizeof(float)); 
  cudaMalloc((void**)&d_neighborhoodZ, k * sizeof(float)); 
  cudaMalloc((void**)&d_diffZ, k * sizeof(float)); 
  cudaMalloc((void**)&d_varZ, k * sizeof(float)); 
  cudaMalloc((void**)&d_feature, size * featsize * sizeof(float)); 

  cudaMemcpy(d_feature, h_feature, size * featsize * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(d_neighbors_indices, neighbors_indices, size * k * sizeof(int), cudaMemcpyHostToDevice);
  cudaMemcpy(d_pointcloud, point_host_, size * 3 * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(d_eigens, eigens, size * 5 * sizeof(float), cudaMemcpyHostToDevice);

}


void GPUFeatureExtractor::get_features(float* feature) {
  dim3 blockSize(256);
  dim3 gridSize((size + blockSize.x - 1) / blockSize.x);

  calculate_features<<<gridSize, blockSize>>>(d_pointcloud, d_feature, featsize, d_neighbors_indices, d_eigens, size, k);

  cudaDeviceSynchronize();

  cudaMemcpy(feature, d_feature, size * featsize * sizeof(float), cudaMemcpyDeviceToHost); 

  cudaError_t cudaStatus = cudaGetLastError();
  if (cudaStatus != cudaSuccess) 
  {
    fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
  }
  cudaFree(d_neighbors_indices);
  cudaFree(d_pointcloud);
  cudaFree(d_feature);
  cudaFree(d_eigens);
}


GPUFeatureExtractor::~GPUFeatureExtractor() {

}
