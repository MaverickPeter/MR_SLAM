import numpy as np
import cython
cimport numpy as np

assert sizeof(int) == sizeof(np.int32_t)

cdef extern from "src/manager.hh":
    cdef cppclass C_GPUTransformer "GPUTransformer":
        C_GPUTransformer(float *point, int size, int *x, int *y, int *height, int max_length, int max_height, int num_x, int num_y, int num_height, int enough_large)
        void transform()
        void retreive(float *point_trans)

cdef class GPUTransformer:
    cdef C_GPUTransformer* g
    cdef int size
    cdef int grid_size
    
    def __cinit__(self, np.ndarray[float, ndim=1, mode = "c"] point not None,
                    int size, int max_length, int max_height, int num_x, int num_y, int num_height, int enough_large):

        self.size = size
        self.grid_size = num_x * num_y * num_height * enough_large
        cdef np.ndarray[int, ndim=1, mode = "c"] y = np.zeros(self.size, dtype=np.int32)
        cdef np.ndarray[int, ndim=1, mode = "c"] x = np.zeros(self.size, dtype=np.int32)
        cdef np.ndarray[int, ndim=1, mode = "c"] height = np.zeros(self.size, dtype=np.int32)

        self.g = new C_GPUTransformer(&point[0], self.size, &x[0], &y[0], &height[0], max_length, max_height, num_x, num_y, num_height, enough_large)

    def transform(self):
        self.g.transform()

    def retreive(self):
        cdef np.ndarray[int, ndim=1, mode = "c"] grid_out = np.zeros(self.size, dtype=np.int32)
        cdef np.ndarray[int, ndim=1, mode = "c"] mask_out = np.zeros(self.size, dtype=np.int32)
        cdef np.ndarray[float, ndim=1, mode = "c"] point_out = np.zeros(self.grid_size * 3, dtype=np.float32)

        self.g.retreive(&point_out[0])
        
        return point_out
