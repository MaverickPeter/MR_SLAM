import os
import sys
import time
import copy
import torch
import random
import voxelocc
import voxelfeat
import config as cfg
import numpy as np
from math import sin, cos
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors, KDTree
import torchvision.transforms.functional as fn
from torch_radon import Radon, ParallelBeam, RadonFanbeam
from mpl_toolkits.mplot3d import Axes3D
from skimage import morphology
import multiprocessing as multiproc
from sys import getsizeof

np.seterr(divide='ignore',invalid='ignore')

# device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

# randomly downsample the pointcloud to a certain number of points
def random_sampling(orig_points, num_points):
    assert orig_points.shape[0] > num_points

    points_down_idx = random.sample(range(orig_points.shape[0]), num_points)
    down_points = orig_points[points_down_idx, :]

    return down_points


# get linear translation (x & y) and yaw angle from a SE3 rotation matrix
def se3_to_3dposes(R):
    # extract the translation and rotation matrix
    rot_mat = R[:3, :3]
    translation = R[:3, 3]
    # get the x and y translation
    x = translation[0]
    y = translation[1]
    # calculate the yaw angle
    yaw = np.arctan2(rot_mat[1, 0], rot_mat[0, 0])

    return x, y, yaw


# get the rotation matrix from euler angles
def euler2rot(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0],
                    [0, cos(roll), -sin(roll)],
                    [0, sin(roll), cos(roll)]])
    R_y = np.array([[cos(pitch), 0, sin(pitch)],
                    [0, 1, 0],
                    [-sin(pitch), 0, cos(pitch)]])
    R_z = np.array([[cos(yaw), -sin(yaw), 0],
                    [sin(yaw), cos(yaw), 0],
                    [0, 0, 1]])
    R = np.dot(R_z, np.dot(R_y, R_x))

    return R


# rotate the bev image using torch
def rotate_bev(bev, angle):
    # convert angle to degree
    angle = angle * 180. / np.pi
    return fn.rotate(bev, angle)


# get the SE3 rotation matrix from the x, y translation and yaw angle    
def getSE3(x, y, yaw):
    R = np.eye(4)
    R[:3, :3] = euler2rot(0, 0, yaw)
    R[0, 3] = x
    R[1, 3] = y
    R[2, 3] = 0

    return R

# transform rotation and translation to homogeneous matrix
def trans2hom(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T

def load_pc_infer(pc):
    # returns Nx3 matrix
    pc = np.array(pc, dtype=np.float32)

    x_condition = np.abs(pc[...,0]) < 70.
    y_condition = np.abs(pc[...,1]) < 70.
    z_condition1 = pc[...,2] < 30.
    z_condition2 = pc[...,2] > 0.
    # intensity_condition = pc[...,3] < 100.
    # print("pc[...,3] median: ", np.median(pc[...,3]))
    conditions_1 = np.bitwise_and(x_condition, y_condition)
    conditions_2 = np.bitwise_and(z_condition1, z_condition2)
    conditions_xy = np.bitwise_and(conditions_1, conditions_2)
    # conditions = np.bitwise_and(conditions_xy, intensity_condition)

    hits = pc[conditions_xy]

    hits[...,0] = hits[...,0] / 70.
    hits[...,1] = hits[...,1] / 70.
    hits[...,2] = (hits[...,2]) / 30.

    return hits


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

    ########### torch
    eigen_ = torch.linalg.eigvalsh(torch.from_numpy(cov_))
    eigen_ = eigen_.cpu().numpy()
    indices = np.argsort(eigen_)
    indices = indices[:,::-1]
    pcs_num_ = eigen_.shape[0]
    indx = np.reshape(np.arange(pcs_num_), [-1, 1])
    eig_ind = indices + indx*3
    vec_ind = np.reshape(eig_ind*3, [-1,1]) + np.full((pcs_num_*3,3), [0,1,2])
    vec_ind = np.reshape(vec_ind, [-1,3,3])
    eigen3d_ = np.take(eigen_, eig_ind)
    vectors_ = []
    entropy_ = calculate_entropy_array(eigen3d_)

    # 2d cov and eigen
    cov2d_ = cov_[:,:2,:2]
    eigen2d = torch.linalg.eigvalsh(torch.from_numpy(cov2d_))
    eigen2d = eigen2d.cpu().numpy()
    indices = np.argsort(eigen2d)
    indices = indices[:, ::-1]
    pcs_num_ = eigen2d.shape[0]
    indx = np.reshape(np.arange(pcs_num_), [-1, 1])
    eig_ind = indices + indx * 2
    eigen2d_ = np.take(eigen2d, eig_ind)
    eigens_ = np.append(eigen3d_,eigen2d_,axis=1)

    return cov_, entropy_, eigens_, vectors_


def build_neighbors_NN(pointcloud, k):
	### using KNN NearestNeighbors cluster according k
    ############ numpy KNN
    nbrs = NearestNeighbors(n_neighbors=k, algorithm='kd_tree', n_jobs=-1).fit(pointcloud)
    distances, indices = nbrs.kneighbors(pointcloud)
    covs, entropy, eigens_, vectors_ = covariation_eigenvalue(pointcloud, indices)

    return indices, covs, entropy, eigens_, vectors_


# genarate RING descriptor using CUDA accelerate 
def generate_RING(pc):
    size = pc.shape[0]
    pc = pc[:,0:3]
    pc = pc.transpose().flatten().astype(np.float32)
    
    ############# generate normal occ bev
    transer_bev = voxelocc.GPUTransformer(pc, size, cfg.max_length, cfg.max_height, cfg.num_ring, cfg.num_sector, cfg.num_height, 1)
    transer_bev.transform()
    point_t_bev = transer_bev.retreive()
    point_t_bev = point_t_bev.reshape(-1, 3)

    point_t_bev = point_t_bev[...,2]
    pc_bev = point_t_bev.reshape(cfg.num_height, cfg.num_ring, cfg.num_sector)

    ########### normal #############
    pc_bev_tensor = torch.from_numpy(pc_bev)

    # genearate RING
    angles = torch.FloatTensor(np.linspace(0, 2*np.pi, cfg.num_ring).astype(np.float32))
    radon = ParallelBeam(cfg.num_sector, angles)

    pc_RING = radon.forward(pc_bev_tensor.to(device))

    pc_RING_normalized = fn.normalize(pc_RING, mean=pc_RING.mean(), std=pc_RING.std())
    pc_TIRING = torch.fft.fft2(pc_RING_normalized, dim=-2,norm="ortho")

    return pc_bev, pc_RING.cpu(), pc_TIRING.cpu()


# genarate RING++ descriptor using CUDA accelerate 
def generate_RINGplusplus(pc):
    size = pc.shape[0]
    pc = pc[:,0:3]
    
    # ! extract local descriptors and then use RING to aggregate the descriptors
    ###### KNN CPU ######
    k = 30
    k_indices, k_covs, k_entropy, k_eigens_, k_vectors_  = build_neighbors_NN(pc, k)

    ###### lpd-feat GPU ######
    k_indices = k_indices.flatten().astype(np.int32)
    k_eigens_ = k_eigens_.flatten()
    pc_flatten = pc.flatten()

    featureExtractor = voxelfeat.GPUFeatureExtractor(pc_flatten, size, 13, 30, k_indices, k_eigens_)
    GPUfeat = featureExtractor.get_features()
    GPUfeat = GPUfeat.reshape(-1,13)
    GPUfeat = GPUfeat.take([0,1,3,10,11,12], axis=1)    # [0,1,3,10,11,12] x 5,7,8,9
    pc = np.concatenate((pc, GPUfeat), axis=1)    # num_points * 13 
    featsize = pc.shape[1]

    pc = pc.transpose().flatten().astype(np.float32)

    ###### generate lpd-feat BEV ######
    transer_bev = voxelfeat.GPUTransformer(pc, size, cfg.max_length, cfg.max_height, cfg.num_ring, cfg.num_sector, cfg.num_height, featsize)
    transer_bev.transform()
    point_t_bev = transer_bev.retreive()
    point_t_bev = point_t_bev.reshape(-1, featsize)
    point_t_bev = point_t_bev[...,3:]
    pc_bev = point_t_bev.reshape(cfg.num_height, cfg.num_ring, cfg.num_sector, featsize-3)
    
    # convert to tensor BEV with several feature channels
    pc_bev_tensor = torch.from_numpy(pc_bev).to(device)
    pc_bev_tensor = pc_bev_tensor.squeeze(0)
    pc_bev_tensor = pc_bev_tensor.permute(2,0,1)

    # genearate RING
    angles = torch.FloatTensor(np.linspace(0, 2*np.pi, cfg.num_ring).astype(np.float32))
    radon = ParallelBeam(cfg.num_sector, angles)

    # RING of lpd-feat BEV with several feature channels
    pc_RING = radon.forward(pc_bev_tensor)

    # TIRING with row-wise FFT on RING
    pc_TIRING, _ = forward_row_fft(pc_RING)
    
    return pc_bev_tensor, pc_RING.cpu(), pc_TIRING.cpu()


def robotid_to_key(robotid):
    char_a = 97
    keyBits = 64
    chrBits = 8
    indexBits = keyBits - chrBits
    outkey = char_a + robotid
    print("robotid: ", robotid, " outkey: ",outkey)
    return outkey << indexBits


# calculate the distance between two poses
def calculate_dist(pose1, pose2):
    dist = np.sqrt((pose1[0] - pose2[0])**2 + (pose1[1] - pose2[1])**2)
    return dist


# check if there is a loop (revisitness) in the map
def is_revisited(query_pose, map_poses, revisit_threshold):
    tree = KDTree(map_poses)
    # get the nearest neighbor
    dist, idx = tree.query(np.array([query_pose]), k=1)
    # dist, idx = tree.query(query_pose.reshape(1, -1) , k=1)
    if dist[0] < revisit_threshold:
        revisited = True
    else:
        revisited = False
    
    return revisited, dist, idx


# appy 2D fourier transform to the input data
def forward_fft(input):
    median_output = torch.fft.fft2(input, dim=(-2, -1),norm="ortho")
    median_output_r = median_output.real
    median_output_i = median_output.imag
    
    output = torch.sqrt(median_output_r ** 2 + median_output_i ** 2)
    output = fftshift2d(output)
    return output, median_output


# appy 1D fourier transform to the row of input data
def forward_row_fft(input):
    median_output = torch.fft.fft2(input, dim=-1,norm="ortho")
    median_output_r = median_output.real
    median_output_i = median_output.imag
    output = torch.sqrt(median_output_r ** 2 + median_output_i ** 2)
    return output, median_output


# appy 1D fourier transform to the colomn of input data
def forward_column_fft(input):
    median_output = torch.fft.fft2(input, dim=-2,norm="ortho")
    median_output_r = median_output.real
    median_output_i = median_output.imag
    output = torch.sqrt(median_output_r ** 2 + median_output_i ** 2)
    # output = fftshift2d(output)
    return output, median_output


# 1D shift the frequnecy spectrum to the center
def fftshift1d(x):
    return torch.fft.fftshift(x)


# 2D shift the frequnecy spectrum to the center
def fftshift2d(x):
    for dim in range(1, len(x.size())):
        n_shift = x.size(dim)//2
        if x.size(dim) % 2 != 0:
            n_shift = n_shift + 1  # for odd-sized images
        x = roll_n(x, axis=dim, n=n_shift)
    return x  # last dim=2 (real&imag)


def roll_n(X, axis, n):
    f_idx = tuple(slice(None, None, None) if i != axis else slice(0, n, None) for i in range(X.dim()))
    b_idx = tuple(slice(None, None, None) if i != axis else slice(n, None, None) for i in range(X.dim()))
    front = X[f_idx]
    back = X[b_idx]
    return torch.cat([back, front], axis)


# compute the max correlation value and the corresponding circular shift 
def fast_corr_RINGplusplus(a, b):

    a = fn.normalize(a, mean=a.mean(), std=a.std())
    b = fn.normalize(b, mean=b.mean(), std=b.std())

    a_fft = torch.fft.fft2(a, dim=-2,norm="ortho")
    b_fft = torch.fft.fft2(b, dim=-2,norm="ortho")

    corr = torch.fft.ifft2(a_fft*b_fft.conj(), dim=-2, norm="ortho")  
    corr = torch.sqrt(corr.real**2 + corr.imag**2)

    corr = torch.sum(corr,dim=0) # add this line for multi feature channels
    corr = torch.sum(corr,dim=-1).view(-1)
    # use the torch fftshift function to shift the correlation to the center (much faster)
    corr = torch.fft.fftshift(corr)

    angle = cfg.num_ring//2 - torch.argmax(corr)
    dist = 1 - torch.max(corr)/(0.15*a.shape[0]*cfg.num_ring*cfg.num_sector) # using any features
    dist = dist.cpu().numpy()
    angle = angle.cpu().numpy()

    return dist, angle


# compute the max correlation value and the corresponding circular shift 
def fast_corr(a, b):
    corr = torch.fft.ifft2(a*b.conj(), dim=-2, norm="ortho")
    corr = torch.sqrt(corr.real**2 + corr.imag**2)
    corr = torch.sum(corr,dim=0)

    corr = torch.sum(corr,dim=-1).view(-1)
    corr = fftshift1d(corr)
    dist = 1 - torch.max(corr)/(0.15*cfg.num_ring*cfg.num_sector)

    angle = -torch.argmax(corr) + cfg.num_ring//2
    dist = dist.cpu().numpy()
    angle = angle.cpu().numpy()
    return dist, angle


# calculate the circular shift to compensate the yaw difference between two RINGs
def calculate_row_shift(shift):
    if shift < cfg.num_ring // 2:
        shift = -shift 
    else:
        shift = shift - cfg.num_ring
    return shift


# solve the relative translation between query and positive
# calculate the relative translation between a and b (from a to b, point cloud transformation) using fft and svd method
def solve_translation(query, positive, rot_angle, device):
    B, H, W = query.shape
    # caculate the translation of b relative to a
    angles = torch.FloatTensor(np.linspace(0, 2*np.pi, H).astype(np.float32)).to(device)
    # compensate for the rotation of the query to calculate the translation
    angles = angles + rot_angle # in radians
    # # take one half of the spectrum for translation estimation
    # angles = angles[0:H//2]
    # query = query[0:H//2,:]
    # positive = positive[0:H//2,:]
    # matrices of the overdetermined linear system
    A = torch.stack([torch.cos(angles), torch.sin(angles)], dim=1)
    b = torch.FloatTensor(H).to(device)

    # calculate the condition number of A
    cond = torch.linalg.cond(A)

    for i in range(H):
        query_fft = torch.fft.fft2(query[:,i,:], dim=-1, norm="ortho")
        positive_fft = torch.fft.fft2(positive[:,i,:], dim=-1, norm="ortho")
        corr = torch.fft.ifft2(query_fft*positive_fft.conj(), dim=-1, norm="ortho")
        corr = torch.sqrt(corr.imag**2 + corr.real**2)
        # use the torch fftshift function to shift the correlation to the center (much faster)
        corr = torch.fft.fftshift(corr)  
        corr = torch.sum(corr,dim=0)
        shift = W//2 - torch.argmax(corr)
        b[i] = shift
    x, y = solve_overdetermined_linear_system(A, b, method='svd')
    # overdetermined linear system error
    error = torch.norm(torch.matmul(A, torch.cat([x, y], dim=0)) - b)
    x = x.cpu().numpy()
    y = y.cpu().numpy()
    error = error.cpu().numpy()
    # print('predicted x, y, error: ', x, y, error)
    
    return x, y, error


# exhaustive search for the best translation
def solve_translation_bev(a, b):
    # normalize the bev
    a = fn.normalize(a, mean=a.mean(), std=a.std())
    b = fn.normalize(b, mean=b.mean(), std=b.std())

    # 2D cross correlation
    a_fft = torch.fft.fft2(a, dim=(-2,-1), norm="ortho")
    b_fft = torch.fft.fft2(b, dim=(-2,-1), norm="ortho")
    corr = torch.fft.ifft2(a_fft*b_fft.conj(), dim=(-2,-1), norm="ortho")  
    corr = torch.sqrt(corr.real**2 + corr.imag**2)
    corr = torch.sum(corr,dim=0) # add this line for multi feature channels

    # fftshift2d function to shift the correlation to the center
    corr = torch.fft.fftshift(corr)

    # get the index of the max correlation
    idx_x = (corr==torch.max(corr)).nonzero()[0][0]
    idx_y = (corr==torch.max(corr)).nonzero()[0][1]

    # get the translation value
    x = idx_x - cfg.num_sector//2 
    y = cfg.num_ring//2 - idx_y 

    return y.cpu().numpy(), x.cpu().numpy(), -torch.max(corr).cpu().numpy()


# solve the relative translation between query and positive
def solve_multilayer_translation(query, positive, rot_angle, device):
    B, H, W = query.shape
    # caculate the translation of b relative to a
    angles = torch.FloatTensor(np.linspace(0, 2*np.pi, H).astype(np.float32)).to(device)
    angles = angles + rot_angle
    # matrices of the overdetermined linear system
    A = torch.stack([torch.cos(angles), torch.sin(angles)], dim=1)
    b = torch.FloatTensor(H).to(device)
    # calculate the condition number of A
    cond = torch.linalg.cond(A)
    print("cond: ", cond)

    for i in range(H):
        query_fft = torch.fft.fft2(query[:,i,:], dim=-1, norm="ortho")
        positive_fft = torch.fft.fft2(positive[:,i,:], dim=-1, norm="ortho")
        corr = torch.fft.ifft2(query_fft*positive_fft.conj(), dim=-1, norm="ortho")
        corr = torch.sqrt(corr.imag**2 + corr.real**2)
        corr = fftshift1d(corr)
        corr = torch.sum(corr,dim=0)
        shift = torch.argmax(corr) - W//2
        b[i] = shift

    x, y = solve_overdetermined_linear_system(A, b, method='svd')
    # overdetermined linear system error
    error = torch.norm(torch.matmul(A, torch.cat([x, y], dim=0)) - b)
    x = x.cpu().numpy()
    y = y.cpu().numpy()
    error = error.cpu().numpy()
    
    return x, y, error



# solve the overdetermined linear system using SVD by torch
def solve_overdetermined_linear_system(A, b, method='pinv'):
    # import pdb; pdb.set_trace()
    # A: [B, H, W, C]
    # b: [B, H, W]
    # method: 'pinv' or 'svd'
    assert method in ['pinv', 'svd']
    B = A.size(0)
    A = A.view(B, -1)
    b = b.view(B, -1)
    if method == 'pinv':
        return torch.pinverse(A) @ b
    else:
        u, s, v = torch.svd(A, some=False)
        s_new = torch.zeros(A.shape).to(device)
        for i in range(len(s)):
            s_new[i, i] = 1/s[i]
        s_inv = s_new.t()

        return v.t() @ s_inv @ u.t() @ b   


def imshow(tensor, title=None):
    image = tensor.cpu().clone()  # we clone the tensor to not do changes on it
    image = image.squeeze(0)  # remove the fake batch dimension
    plt.imshow(image, cmap='jet')
    # plt.colorbar()
    plt.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
    plt.show()