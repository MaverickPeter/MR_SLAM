import gputransform
import numpy as np
import numpy.testing as npt
import time
import os
import numpy.testing as npt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# load test point cloud util
def load_pc_file(filename):
    # returns Nx3 matrix
    pc = np.fromfile(os.path.join("./", filename), dtype=np.float64)
    
    if(pc.shape[0] != 4096*3):
        print("pc shape:", pc.shape)
        print("Error in pointcloud shape")
        return np.array([])

    pc = np.reshape(pc,(pc.shape[0]//3, 3))    
    return pc

# load test point cloud
print("start")

sim_data_orig = load_pc_file("2.bin")
print("load")
# visualize point cloud
x = sim_data_orig[...,0]
y = sim_data_orig[...,1]
z = sim_data_orig[...,2]
fig = plt.figure()
ax = Axes3D(fig)
ax.scatter(x, y, z)
plt.show()
plt.pause(0.1)
plt.close()


# prepare data for gpu process
sim_data_orig = sim_data_orig.astype(np.float32)
sim_data_orig = sim_data_orig[np.newaxis,:,...]

size = sim_data_orig.shape[1]
num_sector = 120
num_ring = 40
num_height = 20
max_length = 1
max_height = 1
num_in_voxel = 1

sim_data = sim_data_orig.transpose()
sim_data = sim_data.flatten()

# tic
time_start = time.time()

# gpu process

adder = gputransform.GPUTransformer(sim_data, size, max_length, max_height, num_ring, num_sector, num_height, num_in_voxel)
adder.transform()
print('tes cost', sim_data.shape)

point_t = adder.retreive()

# toc
time_end = time.time()

print('process cost',time_end - time_start,'s')

# visualize multi-layer scan context image
point_t = point_t.reshape(-1,3)
point_t = point_t[...,2]
point_t = point_t.reshape(20,40,120)
point_t = (point_t + 1.0) / 2.0 *255.0

for i in range(num_height):
    plt.imshow(point_t[i,:,:])
    plt.show()
    plt.pause(0.3)
