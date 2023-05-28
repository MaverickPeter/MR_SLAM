class GPUTransformer {
  // pointer to the GPU memory where the array is stored
  float* point_device;

  // pointer to the CPU memory where the array is stored
  float* point_host;

  // length of the array (number of elements)
  int h_max_length;
  int h_max_height;
  int h_num_ring;
  int h_num_sector;
  int h_num_height;

  int d_max_length;
  int d_max_height;
  int d_num_ring;
  int d_num_sector;
  int enough_large;

  int* h_ring;
  int* h_sector;

  int* ring;
  int* sector;
  int* height;
  int size;
  int d_size;
  int d_grid_size;

public:

  GPUTransformer(float* point_host_, int size_, int* ring_, int* sector_, int* height, int max_length_, int max_height_, int num_ring_, int num_sector_, int num_height_, int enough_large_); // constructor (copies to GPU)

  ~GPUTransformer(); // destructor

  void transform(); // does operation inplace on the GPU

  void retreive(float* point_transformed); //gets results back from GPU, putting them in the memory that was passed in
  // the constructor

};
