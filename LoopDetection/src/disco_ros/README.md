# DiSCO-Pytorch
[![ral](https://img.shields.io/badge/ieee-ral2021-red.svg)](https://ieeexplore.ieee.org/document/9359460)
[![video](https://img.shields.io/badge/video-ral2021-blue.svg)](https://youtu.be/SludumGuLYo)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)


Code for **DiSCO: Differentiable Scan Context with Orientation** submitted in **IEEE Robotics and Automation Letters**  with **ICRA 2021**

Paper is available [here](https://ieeexplore.ieee.org/document/9359460).

Video is available [here](https://youtu.be/SludumGuLYo).

## Pre-Requisites
* PyTorch 1.4.0 (<= 1.6.0 fft module is modified after 1.6.0)
* tensorboardX
* Cython (for point cloud process [voxelization](https://github.com/ZJU-Robotics-Lab/Voxelization_API.git))

You can simply use 'pip install -r requirements.txt' to install required libraries.

If you want to integrate it into **ROS**. We have test it on **Kinetic and Melodic**

## How to use

### prepare training data

For NCLT dataset, if you want to use our code for convenience you have to form the data into this file structure. **(occ_xm is an empty folder)**

```
├── 2012-02-04
│   ├── ground_truth
│   ├── occ_0.5m
│   └── velodyne_data
│       └── velodyne_sync
├── 2012-03-17
│   ├── ground_truth
│   ├── occ_3m
│   └── velodyne_data
│       └── velodyne_sync
...
(You need at least 4 rounds)
```

In [generating_queries](https://github.com/MaverickPeter/DiSCO-pytorch/tree/main/generating_queries)/[nclt](https://github.com/MaverickPeter/DiSCO-pytorch/tree/main/generating_queries/nclt)/

```
python generate_training_tuples_baseline_with_pose.py
python generate_test_sets.py
```

### Use point cloud process module in cuda (cython wrapped)

In [multi-layer-polar-cython](https://github.com/MaverickPeter/DiSCO-pytorch/tree/main/multi-layer-polar-cython)/[cython](https://github.com/MaverickPeter/DiSCO-pytorch/tree/main/multi-layer-polar-cython/cython)

```
# To inplace install the cython wrapped module:
python setup.py build_ext --inplace

# or install in python/site_packages
python setup.py install

# to test
python test.py

(If you meet segmentation fault error, you may have overlarge number of points to process e.g. 67w. To tackle this problem you may need to change your system stack size by 'ulimit -s 81920' in your bash)
```

and now you will have a gputransform.cpythonxxx.so file and copy it to **[generating_queries](https://github.com/MaverickPeter/DiSCO-pytorch/tree/main/generating_queries)/[nclt](https://github.com/MaverickPeter/DiSCO-pytorch/tree/main/generating_queries/nclt) and main dir** where you can find a place holder. Note that the input of the wrapped point cloud process module you should scale the original point cloud to **[-1~1]** range for all axis. No extra process for point cloud such as downscale the points number.

## Train

```
python train_DiSCO.py (arguments please refer to the code in this python file)
```

## Evaluate
```
python evaluate.py (arguments please refer to the code in this python file)
```

## Infer

```
# simple inference
python inference.py

############################
# infer in ros
# create a workspace
mkdir -p ~/disco_ws/src
cd ~/disco_ws/src

# clone the repo
git clone https://github.com/MaverickPeter/DiSCO-pytorch.git
cd ..

# make
catkin_make
source devel/setup.bash

# run
rosrun disco_ros infer_ros.py
```

Take a look at train_DiSCO.py and evaluate.py for more parameters
**We found that our model also works well in cpu, only takes 50-80ms an inference.**

## Pretrained Model on NCLT dataset

[NCLT occupancy model](https://drive.google.com/file/d/1yGFtNUavJT0kxS_mtYAE8U3FzOTpt3c4/view?usp=sharing)


## Acknowledgements

Code references [PointNetVLAD](https://github.com/cattaneod/PointNetVlad-Pytorch)

### Citation

If you use our source code or inspired by our method, please consider citing the following:

```
@ARTICLE{9359460,
  author={X. {Xu} and H. {Yin} and Z. {Chen} and Y. {Li} and Y. {Wang} and R. {Xiong}},
  journal={IEEE Robotics and Automation Letters}, 
  title={DiSCO: Differentiable Scan Context With Orientation}, 
  year={2021},
  volume={6},
  number={2},
  pages={2791-2798},
  doi={10.1109/LRA.2021.3060741}}
```

