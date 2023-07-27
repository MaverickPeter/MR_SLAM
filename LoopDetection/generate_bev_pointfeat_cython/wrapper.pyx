import numpy as np
import cython
cimport numpy as np

assert sizeof(int) == sizeof(np.int32_t)

cdef extern from "src/manager.hh":
    cdef cppclass C_GPUTransformer "GPUTransformer":
        C_GPUTransformer(float *point, int size, int *x, int *y, int *height, int max_length, int max_height, int num_x, int num_y, int num_height, int featsize)
        void transform()
        void retreive(float *point_trans)

    cdef cppclass C_GPUFeatureExtractor "GPUFeatureExtractor":
        C_GPUFeatureExtractor(float *point, int size, int featsize, int k, int* neighbors_indices, float* eigens)
        void get_features(float* feature)

cdef class GPUTransformer:
    cdef C_GPUTransformer* g
    cdef int size
    cdef int grid_size
    
    def __cinit__(self, np.ndarray[float, ndim=1, mode = "c"] point not None,
                    int size, int max_length, int max_height, int num_x, int num_y, int num_height, int featsize):

        self.size = size
        self.grid_size = num_x * num_y * num_height * featsize
        cdef np.ndarray[int, ndim=1, mode = "c"] y = np.zeros(self.size, dtype=np.int32)
        cdef np.ndarray[int, ndim=1, mode = "c"] x = np.zeros(self.size, dtype=np.int32)
        cdef np.ndarray[int, ndim=1, mode = "c"] height = np.zeros(self.size, dtype=np.int32)

        self.g = new C_GPUTransformer(&point[0], self.size, &x[0], &y[0], &height[0], max_length, max_height, num_x, num_y, num_height, featsize)

    def transform(self):
        self.g.transform()

    def retreive(self):
        cdef np.ndarray[float, ndim=1, mode = "c"] point_out = np.zeros(self.grid_size, dtype=np.float32)
        self.g.retreive(&point_out[0])
        return point_out


cdef class GPUFeatureExtractor:
    cdef C_GPUFeatureExtractor* gfe
    cdef int size
    cdef int featmapsize
    def __cinit__(self, np.ndarray[float, ndim=1, mode = "c"] point not None,
                    int size, int featsize, int k, 
                    np.ndarray[int, ndim=1, mode = "c"] neighbors_indices not None,
                    np.ndarray[float, ndim=1, mode = "c"] eigens not None,):

        self.size = size
        self.featmapsize = featsize * size
        self.gfe = new C_GPUFeatureExtractor(&point[0], size, featsize, k, &neighbors_indices[0], &eigens[0])

    def get_features(self):
        cdef np.ndarray[float, ndim=1, mode = "c"] feature = np.zeros(self.featmapsize, dtype=np.float32)
        self.gfe.get_features(&feature[0])
        
        return feature

