class GPUTransformer {
  float* point_device;

  int h_max_length;
  int h_max_height_;
  int h_num_x;
  int h_num_y;
  int h_num_height;
  int d_max_length;
  int d_feat_size;
  int d_num_x;
  int d_num_y;
  int featsize;

  float* d_feature_map;
  float* d_max_handler;

  int* x_ind;
  int* y_ind;
  int* z_ind;
  int size;
  int d_size;
  int d_grid_size;

public:

  GPUTransformer(float* point_host_, int size_, int* x_, int* y_, int* height, int max_length_, int max_height_, int num_x_, int num_y_, int num_height_, int featsize_); // constructor (copies to GPU)

  ~GPUTransformer(); // destructor

  void transform(); // does operation inplace on the GPU

  void retreive(float* point_transformed); //gets results back from GPU, putting them in the memory that was passed in

};

class GPUFeatureExtractor {
  int d_feat_size;
  int featsize;
  
  int* d_neighbors_indices;
  float* d_neighborhoodZ;
  float* d_pointcloud;
  float* d_feature;
  float* d_eigens;
  float* d_diffZ;
  float* d_varZ;
  float* d_eig2d;
  float* d_eig3d;


  float* neighborhoodZ;
  float* diffZ;
  float* varZ;
  float* eig2d;
  float* eig3d;

  int k;
  int size;

public:

  GPUFeatureExtractor(float* point_host_, int size_, int featsize_, int k_, int* neighbors_indices, float* eigens); // constructor (copies to GPU)

  ~GPUFeatureExtractor(); // destructor

  void get_features(float* feature);

};