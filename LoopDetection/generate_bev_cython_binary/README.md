## Voxelization_API

## Voxelization on point clouds using cython wrapped CUDA/C++
This code provides an API to voxelize input point clouds and outputs the occupied information of each voxel.

#### Requirements:
* cython (>=0.16)
* CUDA

YOU need to first ensure you have added environment settings in your ~/.bashrc:

`export PATH="/usr/local/cuda/bin:$PATH"`

`export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"`

#### Install:

To inplace install:

`$ python setup.py build_ext --inplace`
or install
`$ python setup.py install`

to test:

`$ python test.py`

#### More sampling methods can be found in branches

#### API Usage

* **An example**:

```
#### Settings
num_points = 4096
size = num_points
outsize = 2048
num_y = 120
num_x = 40
num_height = 20
max_height = 1  # max height
max_length = 1  # max_length in xy direction (same for x and y)

#### Usage for voxelization
test_data = test_data.transpose()
test_data = test_data.flatten()
voxelizer = voxelocc.GPUTransformer(test_data, size, max_length, max_height, num_x, num_y, num_height, outsize)
voxelizer.transform()
point_t = voxelizer.retreive()
point_t = point_t.reshape(-1,3)
```
