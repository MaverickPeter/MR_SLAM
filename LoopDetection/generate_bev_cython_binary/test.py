import voxelocc
import numpy as np
import numpy.testing as npt
import time
import os
import numpy.testing as npt
import matplotlib.pyplot as plt
from skimage.transform import radon
from mpl_toolkits.mplot3d import Axes3D

def load_pc_file(filename):
    # returns Nx3 matrix
    pc = np.fromfile(os.path.join("./", filename), dtype=np.float64)

    if(pc.shape[0] != 4096*3):
        print("Error in pointcloud shape")
        return np.array([])

    pc = np.reshape(pc,(pc.shape[0]//3, 3))    
    return pc

def project_2_bev(pointcloud, edge, range = 0):
    if range > 0:
        p_range = np.linalg.norm(pointcloud[:,:2], axis=1)
        inside = (p_range < range)
        pointcloud = pointcloud[inside,:]

    p_x = np.round(np.clip(pointcloud[:,0]*2, -edge, edge-1)+edge).astype(int)
    p_y = np.round(np.clip(pointcloud[:,1]*2, -edge, edge-1)+edge).astype(int)

    image = np.zeros((edge*2, edge*2)).astype(np.uint8)
    image[p_y, p_x] = 255
    return image

#### Test
test_data = load_pc_file("test.bin")
x = test_data[...,0]
y = test_data[...,1]
z = test_data[...,2]

fig = plt.figure()
ax = Axes3D(fig)
ax.scatter(x, y, z)
plt.show()
plt.pause(0.1)
plt.close()

sc_image = project_2_bev(test_data,40,120)
plt.imshow(sc_image)
plt.show()

test_data = test_data.astype(np.float32)
test_data = test_data[np.newaxis,:,...]

#### Settings
num_points = 4096
size = num_points
num_y = 120
num_x = 40
num_height = 20
max_height = 1  # max height
max_length = 1  # max_length in xy direction (same for x and y)
num_in_voxel = 1 # Num points in a voxel, no use for occupied information

#### Usage for voxelization
test_data = test_data.transpose()
test_data = test_data.flatten()
transer = voxelocc.GPUTransformer(test_data, size, max_length, max_height, num_x, num_y, num_height, num_in_voxel)
transer.transform()
point_t = transer.retreive()
point_t = point_t.reshape(-1,3)
point_t = point_t.reshape(num_height, num_x, num_y, 3)



#### Visualization
for i in range(num_height):
    plt.imshow(point_t[i,:,:,2])
    plt.show()
    radon_image = radon(point_t[i,:,:,2], theta=np.arange(0,180,1))
    plt.imshow(radon_image)
    plt.show()
    plt.pause(0.3)
