import voxelfeat
import numpy as np
import numpy.testing as npt
import time
import os
import numpy.testing as npt
import matplotlib.pyplot as plt
from skimage.transform import radon
from mpl_toolkits.mplot3d import Axes3D
from sklearn.neighbors import NearestNeighbors, KDTree
import math
from knn_cuda import KNN
import torch

def calculate_entropy_array(eigen):
    L_ = (eigen[:,0] - eigen[:,1]) / eigen[:,0]
    P_ = (eigen[:,1] - eigen[:,2]) / eigen[:,0]
    S_ = eigen[:,2] / eigen[:,0]
    Entropy = -L_*np.log(L_)-P_*np.log(P_)-S_*np.log(S_)
    return Entropy


def covariation_eigenvalue(pointcloud, neighborhood_index):
    ### calculate covariation and eigenvalue of 3D and 2D
    # prepare neighborhood
    neighborhoods = pointcloud[neighborhood_index]

    # 3D cov and eigen by matrix
    Ex = np.average(neighborhoods, axis=1)
    Ex = np.reshape(np.tile(Ex,[neighborhoods.shape[1]]), neighborhoods.shape)
    P = neighborhoods-Ex
    cov_ = np.matmul(P.transpose((0,2,1)),P)/(neighborhoods.shape[1]-1)
    eigen_, vec_ = np.linalg.eig(cov_)
    indices = np.argsort(eigen_)
    indices = indices[:,::-1]
    pcs_num_ = eigen_.shape[0]
    indx = np.reshape(np.arange(pcs_num_), [-1, 1])
    eig_ind = indices + indx*3
    vec_ind = np.reshape(eig_ind*3, [-1,1]) + np.full((pcs_num_*3,3), [0,1,2])
    vec_ind = np.reshape(vec_ind, [-1,3,3])
    eigen3d_ = np.take(eigen_, eig_ind)
    vectors_ = np.take(vec_, vec_ind)
    entropy_ = calculate_entropy_array(eigen3d_)

    # 2D cov and eigen
    cov2d_ = cov_[:,:2,:2]
    eigen2d, vec_2d = np.linalg.eig(cov2d_)
    indices = np.argsort(eigen2d)
    indices = indices[:, ::-1]
    pcs_num_ = eigen2d.shape[0]
    indx = np.reshape(np.arange(pcs_num_), [-1, 1])
    eig_ind = indices + indx * 2
    eigen2d_ = np.take(eigen2d, eig_ind)

    eigens_ = np.append(eigen3d_,eigen2d_,axis=1)

    return cov_, entropy_, eigens_.astype(np.float32), vectors_

def build_neighbors_NN(pointcloud, k):
	### using KNN NearestNeighbors cluster according k
    nbrs = NearestNeighbors(n_neighbors=k).fit(pointcloud)
    distances, indices = nbrs.kneighbors(pointcloud)
    print("knn index shape: ", indices.shape)
    covs, entropy, eigens_, vectors_ = covariation_eigenvalue(pointcloud, indices)

    return indices, covs, entropy, eigens_, vectors_

def calculate_features(pointcloud, nbrs_index, eigens_, vectors_):
    ### calculate handcraft feature with eigens and statistics data

    # features using eigens
    eig3d = eigens_[:3]
    eig2d = eigens_[3:5]

    # 3d
    # if eig3d.sum() == 0:
    #     print("eig3d.sum() 0 ", eig3d.sum())
    C_ = eig3d[2] / (eig3d.sum())
    O_ = np.power((eig3d.prod() / np.power(eig3d.sum(), 3)), 1.0 / 3)
    L_ = (eig3d[0] - eig3d[1]) / eig3d[0]
    E_ = -((eig3d / eig3d.sum()) * np.log(eig3d / eig3d.sum())).sum()
    P_ = (eig3d[1] - eig3d[2]) / eig3d[0]
    S_ = eig3d[2] / eig3d[0]
    A_ = (eig3d[0] - eig3d[2]) / eig3d[0]
    X_ = eig3d.sum()
    D_ = 3 * nbrs_index.shape[0] / (4 * math.pi * eig3d.prod())
    # 2d
    S_2 = eig2d.sum()
    L_2 = eig2d[1] / eig2d[0]
    # features using statistics data
    neighborhood = pointcloud[nbrs_index]
    nbr_dz = neighborhood[:, 2] - neighborhood[:, 2].min()
    dZ_ = nbr_dz.max()
    vZ_ = np.var(nbr_dz)
    V_ = vectors_[2][2]

    features = np.asarray([C_, O_, L_, E_, P_, S_, A_, X_, D_, S_2, L_2, dZ_, vZ_])#([C_,O_,L_,E_,D_,S_2,L_2,dZ_,vZ_,V_])
    return features, vZ_

def get_pointfeat(pointcloud):
    # prepare KNN cluster number k
    k = 30
    k_indices, k_covs, k_entropy, k_eigens_, k_vectors_  = build_neighbors_NN(pointcloud, k)
    
    ref = torch.from_numpy(pointcloud).unsqueeze(0).float().cuda()
    query = torch.from_numpy(pointcloud).unsqueeze(0).float().cuda()
    knn = KNN(k, transpose_mode=True)
    d, indx = knn(ref, query)
    print("indx.shape ",indx.shape)

    print("test torch knn ", indx==k_indices)
    points_feature = []
    for index in range(pointcloud.shape[0]):
        ### per point
        neighborhood = k_indices[index].astype(np.int32)
        eigens_ = k_eigens_[index].astype(np.float32)
        vectors_ = k_vectors_[index]

        # calculate point feature
        feature, vZ_ = calculate_features(pointcloud, neighborhood, eigens_, vectors_)
        points_feature.append(feature)

    points_feature = np.asarray(points_feature)

    return points_feature, indx.detach().cpu().numpy(), k_eigens_


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

# fig = plt.figure()
# ax = Axes3D(fig)
# ax.scatter(x, y, z)
# plt.show()
# plt.pause(0.1)
# plt.close()

sc_image = project_2_bev(test_data,40,120)
# plt.imshow(sc_image)
# plt.show()

test_data = test_data.astype(np.float32)
# test_data = test_data[np.newaxis,:,...]

#### Settings
num_points = 4096
size = num_points
num_y = 120
num_x = 120
num_height = 1
max_height = 1  # max height
max_length = 1  # max_length in xy direction (same for x and y)
num_in_voxel = 1 # Num points in a voxel, no use for occupied information
test_data = test_data.astype(np.float32)

#### Usage for voxelization
# print("test_data shape: ", test_data.shape)
pointfeat, k_indices, k_eigens_ = get_pointfeat(test_data)   # num_points * 10
CPUFeat = np.concatenate((test_data, pointfeat), axis=1)      # num_points * 13 

# #### trans CPU feat
CPUFeat_flatten = CPUFeat.transpose().flatten().astype(np.float32)
CPUtranser = voxelocc.GPUTransformer(CPUFeat_flatten, size, max_length, max_height, num_x, num_y, num_height, 16)
CPUtranser.transform()
CPUBEV = CPUtranser.retreive()
CPUBEV = CPUBEV.reshape(-1,16)
CPUBEV = CPUBEV[...,3:]
CPUBEV = CPUBEV.reshape(num_height, num_x, num_y, 13)
CPUBEV = CPUBEV.take([0,1,3,10,11,12], axis=3)

### Visualization
# for i in range(13):
#     plt.imshow(CPUBEV[0,:,:,i])
#     plt.show()
#     plt.pause(0.3)

print("test_data shape: ", test_data.shape)
#### GPU
test_data_flatten = test_data.flatten()
start = time.time()
k_indices = k_indices.flatten().astype(np.int32)
k_eigens_ = k_eigens_.flatten()

featureExtractor = voxelocc.GPUFeatureExtractor(test_data_flatten, size, 13, 30, k_indices, k_eigens_)
GPUfeat = featureExtractor.get_features()
GPUfeat = GPUfeat.reshape(-1,13)
GPUdataFeat = np.concatenate((test_data, GPUfeat), axis=1)      # num_points * 13 
end = time.time()
print("featureExtractor: ", end - start)

#### trans GPU feat
GPUdataFeat_flatten = GPUdataFeat.transpose().flatten()
GPUtranser = voxelocc.GPUTransformer(GPUdataFeat_flatten, size, max_length, max_height, num_x, num_y, num_height, 16)
GPUtranser.transform()
GPUBEV = GPUtranser.retreive()
print("GPUBEV shape: ", GPUBEV.shape)

GPUBEV = GPUBEV.reshape(-1,16)
GPUBEV = GPUBEV[...,3:]
GPUBEV = GPUBEV.reshape(num_height, num_x, num_y, 13)

GPUBEV = GPUBEV.take([0,1,3,10,11,12], axis=3)

print(np.all((CPUBEV - GPUBEV)==0))
#### Visualization
for i in range(6):
    plt.imshow(CPUBEV[0,:,:,i])
    plt.show()
    print(np.max(CPUBEV[0,:,:,i]))
    plt.imshow(GPUBEV[0,:,:,i])
    plt.show()
    print(np.max(GPUBEV[0,:,:,i]))
    plt.pause(0.3)
