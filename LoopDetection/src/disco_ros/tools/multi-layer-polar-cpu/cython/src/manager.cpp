/*
This is the central piece of code. This file implements a class
(interface in manager.hh) that takes data in on the cpu side, copies
it to the gpu, and exposes functions (increment and retreive) that let
you perform actions with the GPU

This class will get translated into python via swig
*/

#include <kernel.cpp>
#include <manager.hh>
#include <assert.h>
#include <iostream>
#include <chrono>
#include <thread>
using namespace std;

GPUTransformer::GPUTransformer (float* point_host_, int size_, int* ring_, int* sector_, int* height_, int max_length_, int max_height_, int num_ring_, int num_sector_, int num_height_, int enough_large_) {
  point_host = point_host_;
  h_max_length = max_length_;
  h_max_height = max_height_;
  h_num_ring = num_ring_;
  h_num_height = num_height_;
  h_num_sector = num_sector_;
  enough_large = enough_large_;

  d_size = size_;
  ring = (int*)malloc(sizeof(int)*(d_size));    
  sector = (int*)malloc(sizeof(int)*(d_size));
  height = (int*)malloc(sizeof(int)*(d_size));
  d_grid_size = num_ring_ * num_sector_ * num_height_ ;

}


void GPUTransformer::transform() {
  point2gridmap(point_host, ring, sector, height, d_size, h_max_length, h_max_height, h_num_ring, h_num_sector, h_num_height);
}


void GPUTransformer::retreive(float* point_transformed) {
  int pt_count = 0;
  int index = 0;
  int counter[d_grid_size] = {0};

  for (int i = 0; i < d_size; i++)
  {
    if(counter[sector[i] + ring[i] * h_num_sector + height[i] * h_num_sector * h_num_ring] < enough_large) //point_host[i + 2*d_size] >= (point_transformed[3*(ring[i] * h_num_sector + sector[i] + pt_count * h_num_sector * h_num_ring) + 2]) &&
    {
      pt_count = counter[sector[i] + ring[i] * h_num_sector + height[i] * h_num_sector * h_num_ring];
      point_transformed[3*(sector[i] + ring[i] * h_num_sector + height[i] * h_num_sector * h_num_ring + pt_count * h_num_sector * h_num_ring * h_num_height) + 0] = point_host[i];
      point_transformed[3*(sector[i] + ring[i] * h_num_sector + height[i] * h_num_sector * h_num_ring + pt_count * h_num_sector * h_num_ring * h_num_height) + 1] = point_host[i + d_size];
      point_transformed[3*(sector[i] + ring[i] * h_num_sector + height[i] * h_num_sector * h_num_ring + pt_count * h_num_sector * h_num_ring * h_num_height) + 2] = 1;
      counter[sector[i] + ring[i] * h_num_sector + height[i] * h_num_sector * h_num_ring] ++;
    }
  }

}


GPUTransformer::~GPUTransformer() {
  free(ring);
  free(sector);
  free(height);

}
