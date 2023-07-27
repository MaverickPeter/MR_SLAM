### Installation RING and RING++

#### 1. Use point cloud process module in cuda (cython wrapped)
In [genreate_bev_cython_binary](https://github.com/MaverickPeter/MR_SLAM/tree/main/LoopDetection/generate_bev_cython_binary) and [generate_bev_pointfeat_cython](https://github.com/MaverickPeter/MR_SLAM/tree/main/LoopDetection/generate_bev_pointfeat_cython):

```sh
# To install in python/site_packages (Suggested)
python setup.py install

# or inplace install the cython wrapped module:
python setup.py build_ext --inplace

# to test
python test.py

# If you meet segmentation fault error, you may have overlarge number of points to process e.g. 67w. To tackle this problem you may need to change your system stack size by 'ulimit -s 81920' in your bash
```
If you choose to build_ext: you will have a voxelocc.cpythonxxx.so and voxelfeat.cpythonxxx.so file and copy it to RING_ros where you can find a place holder. Note that the input of the wrapped point cloud process module you should scale the original point cloud to [-1~1] range for all axis. No extra process for point cloud such as downscale the points number.

#### 2. torch-radon (radon transform)
You can find the original README file in the torch-radon directory. You can simply follow the command below:
```sh
cd torch-radon
python setup.py install
```