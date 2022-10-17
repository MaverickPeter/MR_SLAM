class GPUTransformer {
  float* point_device;
  float* point_host;

  int h_max_length;
  int h_max_height_;
  int h_num_x;
  int h_num_y;
  int h_num_height;
  int d_max_length;
  int d_num_x;
  int d_num_y;
  int enough_large;

  int* h_x;
  int* h_y;

  int* x;
  int* y;
  int* height;
  int size;
  int d_size;
  int d_grid_size;

public:

  GPUTransformer(float* point_host_, int size_, int* x_, int* y_, int* height, int max_length_, int max_height_, int num_x_, int num_y_, int num_height_, int enough_large_); // constructor (copies to GPU)

  ~GPUTransformer(); // destructor

  void transform(); // does operation inplace on the GPU

  void retreive(float* point_transformed); //gets results back from GPU, putting them in the memory that was passed in

};
