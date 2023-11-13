#!/usr/bin/python3
import os
import cv2
import sys
import math
import time
import rospy
import torch
import socket
import gputransform
import argparse
import importlib
import numpy as np
import config as cfg
import scipy.io as scio
import loss.loss_function
import pygicp
import torch.nn as nn
import open3d as o3d
import models.DiSCO as SC
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
from torch.backends import cudnn
from loading_pointclouds import *
from torchvision import transforms, utils
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from sklearn.neighbors import NearestNeighbors, KDTree
from tf.transformations import translation_matrix, quaternion_matrix, translation_from_matrix, quaternion_from_matrix
from dislam_msgs.msg import Loop, Loops, SubMap

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)


# main message information of all robots
Pose1 = []
Pose2 = []
Pose3 = []
Time1 = []
Time2 = []
Time3 = []
# main descriptors of all robots
PC1 = []
PC2 = []
PC3 = []
DiSCO1 = []
DiSCO2 = []
DiSCO3 = []
FFT1 = []
FFT2 = []
FFT3 = []
yaw_diff_pc = []

f = open("./loopinfo.txt", "w")

def imshow(tensor, title=None):
    unloader = transforms.ToPILImage()
    image = tensor.cpu().clone()  # we clone the tensor to not do changes on it
    image = image.squeeze(0)  # remove the fake batch dimension
    image = unloader(image)
    plt.imshow(image, cmap='jet')
    plt.show()


def infer_model(model, corr2soft, query):
    model.eval()
    corr2soft.eval()
    is_training = False
    
    with torch.no_grad():
        feed_tensor = torch.from_numpy(query).float()
        feed_tensor = feed_tensor.to(device)
        feed_tensor = feed_tensor.view((-1, cfg.num_height, cfg.num_ring, cfg.num_sector))
        out, outfft, fft_result, unet_out = model(feed_tensor)
        # imshow(torch.sum(feed_tensor, axis=1))
        # imshow(unet_out)
    model.train()
    return out, outfft, fft_result, unet_out


# generate DiSCO descriptor
def generate_DiSCO(input_cloud):
    # query = load_pc_infer(input_cloud)
    query = np.array(input_cloud, dtype=np.float32)

    out, _, fft_result, _ = infer_model(model, corr2soft, query)
    out = out.squeeze()
    return out.detach().cpu().numpy(), fft_result


# process pointcloud for inference
def load_pc_infer(pc):
    # returns Nx3 matrix
    pc = np.array(pc, dtype=np.float32)

    x_condition = np.abs(pc[...,0]) < 70.
    y_condition = np.abs(pc[...,1]) < 70.
    z_condition1 = pc[...,2] < 30.
    z_condition2 = pc[...,2] > 0.
    conditions_1 = np.bitwise_and(x_condition, y_condition)
    conditions_2 = np.bitwise_and(z_condition1, z_condition2)
    conditions = np.bitwise_and(conditions_1, conditions_2)
    hits = pc[conditions]

    hits[...,0] = hits[...,0] / 70.
    hits[...,1] = hits[...,1] / 70.
    hits[...,2] = (hits[...,2]) / 30.


    pc = np.array(hits, dtype=np.float32)
    size = pc.shape[0]

    pc_point = np.zeros([cfg.num_height * cfg.num_ring * cfg.num_sector])
    pc = pc.transpose().flatten().astype(np.float32)

    transer = gputransform.GPUTransformer(pc, size, cfg.max_length, cfg.max_height, cfg.num_ring, cfg.num_sector, cfg.num_height, 1)
    transer.transform()
    point_t = transer.retreive()
    point_t = point_t.reshape(-1, 3)
    point_t = point_t[...,2]
    pc_point = point_t.reshape(cfg.num_height, cfg.num_ring, cfg.num_sector)
    
    return pc_point


def robotid_to_key(robotid):
    char_a = 97
    keyBits = 64
    chrBits = 8
    indexBits = keyBits - chrBits
    outkey = char_a + robotid
    print("robotid: ", robotid, " outkey: ",outkey)
    return outkey << indexBits


# get the transformation matrix from the pose message
def get_homo_matrix_from_pose_msg(pose):
    trans = translation_matrix((pose.position.x,
                                                       pose.position.y,
                                                       pose.position.z))

    rot = quaternion_matrix((pose.orientation.x,
                                                    pose.orientation.y,
                                                    pose.orientation.z,
                                                    pose.orientation.w))

    se3 = np.dot(trans, rot)

    return se3


# get the pose message from the transformation matrix
def get_pose_msg_from_homo_matrix(se3):
    pose = Pose()
    trans = translation_from_matrix(se3)
    quat = quaternion_from_matrix(se3)
    
    # pose.position = trans
    # pose.orientation = quat

    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose

# apply icp using fast_gicp (https://github.com/SMRT-AIST/fast_gicp)
def fast_gicp(source, target, max_correspondence_distance=1.0, init_pose=np.eye(4)):
    # downsample the point cloud before registration

    source = pygicp.downsample(source, 0.2)
    target = pygicp.downsample(target, 0.2)

    # pygicp.FastGICP has more or less the same interfaces as the C++ version
    gicp = pygicp.FastGICP()
    gicp.set_input_target(target)
    gicp.set_input_source(source)

    # optional arguments
    gicp.set_num_threads(4)
    gicp.set_max_correspondence_distance(max_correspondence_distance)

    # align the point cloud using the initial pose calculated by DiSCO
    T_matrix = gicp.align(initial_guess=init_pose)

    # get the fitness score
    fitness = gicp.get_fitness_score(1.0)
    # get the transformation matrix
    T_matrix = gicp.get_final_transformation()

    return fitness, T_matrix

# achieve point-to-point icp with open3d
def o3d_icp(source, target, tolerance=0.2, init_pose=np.eye(4)):
    # apply outlier removal
    # source.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
    # target.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)

    # run icp
    result = o3d.pipelines.registration.registration_icp(source, target, tolerance, init_pose,
                    o3d.pipelines.registration.TransformationEstimationPointToPoint())

    # get the icp fitness score
    fitness = result.fitness

    # get the transformation matrix
    T_matrix = result.transformation

    return fitness, T_matrix


# get the rotation matrix from euler angles
def euler2rot(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    R = np.dot(R_z, np.dot(R_y, R_x))

    return R


# get the SE3 rotation matrix from the x, y translation and yaw angle    
def getSE3(x, y, yaw):
    R = np.eye(4)
    R[:3, :3] = euler2rot(0, 0, yaw)
    R[:3, 3] = np.array([x, y, 0])

    return R


def roll_n(X, axis, n):
    f_idx = tuple(slice(None, None, None) if i != axis else slice(0, n, None) for i in range(X.dim()))
    b_idx = tuple(slice(None, None, None) if i != axis else slice(n, None, None) for i in range(X.dim()))
    front = X[f_idx]
    back = X[b_idx]
    return torch.cat([back, front], axis)


def fftshift2d(x):
    for dim in range(1, len(x.size())):
        n_shift = x.size(dim)//2
        if x.size(dim) % 2 != 0:
            n_shift = n_shift + 1  # for odd-sized images
        x = roll_n(x, axis=dim, n=n_shift)
    return x  # last dim=2 (real&imag)


def phase_corr(a, b, device, corr2soft):
    # a: template; b: source
    eps = 1e-15
    corr = torch.fft.ifft2(a*b.conj(), norm="ortho")
    corr = torch.sqrt(corr.imag**2 + corr.real**2 + eps)
    corr = fftshift2d(corr)

    corr = corr.squeeze(1)

    angle = torch.argmax(corr)
    angle = angle % cfg.num_sector

    return angle.detach().cpu().numpy(), corr


# perform loop detection and apply icp
def detect_loop_icp(robotid_current, idx_current, pc_current, DiSCO_current, fft_current, \
                        robotid_candidate, pc_candidates, DiSCO_candidates, FFT_candidates):
    
    num_candidates = 1

    if len(DiSCO_candidates) <= 1:
        return

    kdtree_pc = KDTree(np.array(DiSCO_candidates))
    dists_pc, idxs_pc = kdtree_pc.query(DiSCO_current.reshape(1,-1), k=num_candidates)

    for i in range(num_candidates):
        idx_sc = idxs_pc[0][i]
        FFT_candidate = FFT_candidates[idx_sc] 
        yaw_pc, _ = phase_corr(FFT_candidate, fft_current, device, corr2soft)
        yaw_pc = (yaw_pc - cfg.num_sector//2) / float(cfg.num_sector) * 360.

        yaw_diff_pc.append(yaw_pc)
        idx_top1_pc = idxs_pc[0][0]

        pred_angle_deg = yaw_diff_pc[0] # in degree
        pred_angle_rad = pred_angle_deg * np.pi / 180.
        init_pose_pc = getSE3(0, 0, pred_angle_rad)

        pc_matched_pc = pc_candidates[idx_top1_pc]
        fitness_pc, loop_transform = fast_gicp(pc_current, pc_matched_pc, max_correspondence_distance=cfg.icp_max_distance, init_pose=init_pose_pc)
        
        print("fitness: ", fitness_pc)
        if fitness_pc < cfg.icp_fitness_score and robotid_current != robotid_candidate:
            print("ICP fitness score is less than threshold, accept the loop.")
            Loop_msgs = Loops()
            # publish the result of loop detection and icp
            Loop_msg = Loop()
            # id0 contain the current robot id and its RING index, id1 contain the matched robot id and its RING index       
            Loop_msg.id0 = robotid_to_key(robotid_current) + idx_current + 1
            Loop_msg.id1 = robotid_to_key(robotid_candidate) + idx_top1_pc + 1
            # !!! convert the pointcloud transformation matrix to the frame transformation matrix
            loop_transform = np.linalg.inv(loop_transform)
            # convert the transform matrix to the format of position and orientation
            pose = get_pose_msg_from_homo_matrix(loop_transform)
            Loop_msg.pose = pose
            line = [robotid_current, idx_current, robotid_candidate, idx_top1_pc, pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            line = ' '.join(str(i) for i in line)
            f.write(line)
            f.write("\n")
            Loop_msgs.Loops.append(Loop_msg)
            pub.publish(Loop_msgs)      
            print("DiSCO: Loop detected between id ", Loop_msg.id0, " and id ", Loop_msg.id1)          
        else:
            print("DiSCO: ICP fitness score is larger than threshold, reject the loop.")


def callback1(data):
    # current robot id
    robotid_current = 0
    # current robot index
    idx_current = len(PC1)
    # current robot timestamp
    timestamp = data.keyframePC.header.stamp
    # print(timestamp, timestamp.to_sec())

    # get the keyframe point cloud
    pc = pc2.read_points(data.keyframePC, skip_nans=True, field_names=("x", "y", "z"))
    pc_list = []
    for p in pc:
        pc_list.append([p[0],p[1],p[2]])

    # convert the point cloud to o3d point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_list)
    pcd = pcd.voxel_down_sample(voxel_size=0.2)

    pc_normalized = load_pc_infer(pcd.points)
    pc = np.asarray(pcd.points)
    
    # get the pose of the keyframe point cloud 
    # convert position and quaternion pose to se3 matrix
    se3 = get_homo_matrix_from_pose_msg(data.pose)

    # generate DiSCO and TIDiSCO descriptors
    times = time.time()
    pc_DiSCO, fft_result = generate_DiSCO(pc_normalized)
    timee = time.time()
    print("Descriptors generated time:", timee - times, 's')

    # detect the loop and apply icp
    # candidate robot id: 1
    # robotid_candidate = 0
    # detect_loop_icp(robotid_current, idx_current, pc, pc_DiSCO, robotid_candidate, PC1, DiSCO1)
    # candidate robot id: 2
    robotid_candidate = 1
    detect_loop_icp(robotid_current, idx_current, pc, pc_DiSCO, fft_result, robotid_candidate, PC2, DiSCO2, FFT2)
    # candidate robot id: 3
    robotid_candidate = 2
    detect_loop_icp(robotid_current, idx_current, pc, pc_DiSCO, fft_result, robotid_candidate, PC3, DiSCO3, FFT3)

    Pose1.append(se3)
    Time1.append(timestamp)
    PC1.append(pc)
    DiSCO1.append(pc_DiSCO)
    FFT1.append(fft_result)


def callback2(data):
    # current robot id
    robotid_current = 1
    # current robot index
    idx_current = len(PC2)
    # current robot timestamp
    timestamp = data.keyframePC.header.stamp

    # get the keyframe point cloud
    pc = pc2.read_points(data.keyframePC, skip_nans=True, field_names=("x", "y", "z"))
    pc_list = []
    for p in pc:
        pc_list.append([p[0],p[1],p[2]])
    
    # convert the point cloud to o3d point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_list)
    pcd = pcd.voxel_down_sample(voxel_size=0.2)

    pc_normalized = load_pc_infer(pcd.points)
    pc = np.asarray(pcd.points)

    # get the pose of the keyframe point cloud 
    # convert position and quaternion pose to se3 matrix
    se3 = get_homo_matrix_from_pose_msg(data.pose)

    # generate DiSCO and TIDiSCO descriptors
    times = time.time()
    pc_DiSCO, fft_result = generate_DiSCO(pc_normalized)
    timee = time.time()
    print("Descriptors generated time:", timee - times, 's')

    # detect the loop and apply icp
    # candidate robot id: 1
    robotid_candidate = 0
    detect_loop_icp(robotid_current, idx_current, pc, pc_DiSCO, fft_result, robotid_candidate, PC1, DiSCO1, FFT1)
    # candidate robot id: 2
    # robotid_candidate = 1
    # detect_loop_icp(robotid_current, idx_current, pc, pc_DiSCO, robotid_candidate, PC2, DiSCO2)
    # candidate robot id: 3
    robotid_candidate = 2
    detect_loop_icp(robotid_current, idx_current, pc, pc_DiSCO, fft_result, robotid_candidate, PC3, DiSCO3, FFT3)

    Pose2.append(se3)
    Time2.append(timestamp)
    PC2.append(pc)
    DiSCO2.append(pc_DiSCO)
    FFT2.append(fft_result)


def callback3(data):
    # current robot id
    robotid_current = 2
    # current robot index
    idx_current = len(PC3)
    # current robot timestamp
    timestamp = data.keyframePC.header.stamp

    # get the keyframe point cloud
    pc = pc2.read_points(data.keyframePC, skip_nans=True, field_names=("x", "y", "z"))
    pc_list = []
    for p in pc:
        pc_list.append([p[0],p[1],p[2]])

    # convert the point cloud to o3d point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_list)
    pcd = pcd.voxel_down_sample(voxel_size=0.2)

    pc_normalized = load_pc_infer(pcd.points)
    pc = np.asarray(pcd.points)

    # get the pose of the keyframe point cloud 
    # convert position and quaternion pose to se3 matrix
    se3 = get_homo_matrix_from_pose_msg(data.pose)

    # generate DiSCO and TIDiSCO descriptors
    times = time.time()
    pc_DiSCO, fft_result = generate_DiSCO(pc_normalized)
    timee = time.time()
    print("Descriptors generated time:", timee - times, 's')

    # detect the loop and apply icp
    # candidate robot id: 1
    robotid_candidate = 0
    detect_loop_icp(robotid_current, idx_current, pc, pc_DiSCO, fft_result, robotid_candidate, PC1, DiSCO1, FFT1)
    # candidate robot id: 2
    robotid_candidate = 1
    detect_loop_icp(robotid_current, idx_current, pc, pc_DiSCO, fft_result, robotid_candidate, PC2, DiSCO2, FFT2)
    # candidate robot id: 3
    # robotid_candidate = 2
    # detect_loop_icp(robotid_current, idx_current, pc, pc_DiSCO,, robotid_candidate, PC3, DiSCO3)

    Pose3.append(se3)
    Time3.append(timestamp)
    PC3.append(pc)
    DiSCO3.append(pc_DiSCO)
    FFT3.append(fft_result)


if __name__ == "__main__":
    #### load params
    parser = argparse.ArgumentParser(description='Loop Detection arguments')

    parser.add_argument('--num_ring', type=int, default=40) 
    parser.add_argument('--num_sector', type=int, default=120)
    parser.add_argument('--num_height', type=int, default=20) 
    parser.add_argument('--max_length', type=int, default=1)
    parser.add_argument('--max_height', type=int, default=1)
    parser.add_argument('--max_icp_iter', type=int, default=50) # 20 iterations is usually enough
    parser.add_argument('--icp_tolerance', type=float, default=0.001) 
    parser.add_argument('--icp_max_distance', type=float, default=5.0)
    parser.add_argument('--icp_fitness_score', type=float, default=0.10) # icp fitness score threshold

    args = parser.parse_args()

    #### load params
    cfg.num_ring = args.num_ring
    cfg.num_sector = args.num_sector
    cfg.num_height = args.max_height
    cfg.max_length = args.max_length
    cfg.max_height = args.max_height
    cfg.icp_max_distance = args.icp_max_distance
    cfg.max_icp_iter = args.max_icp_iter
    cfg.icp_tolerance = args.icp_tolerance
    cfg.icp_fitness_score = args.icp_fitness_score

    cfg.LOG_DIR = './log/polar_both_40_120_20/' # Change it !!
    cfg.MODEL_FILENAME = "model.ckpt"

    #### load model (this version can be used without model, refer to DiSCO.py line 325)
    model = SC.DiSCO(output_dim=cfg.FEATURE_OUTPUT_DIM)
    corr2soft = SC.Corr2Softmax(200., 0.)
    corr2soft = corr2soft.to(device)
    model = model.to(device)
    resume_filename = cfg.LOG_DIR + cfg.MODEL_FILENAME
    print("Resuming From ", resume_filename)
    # checkpoint = torch.load(resume_filename)
    # saved_state_dict = checkpoint['state_dict']
    # saved_corr2soft_dict = checkpoint['corr2soft']
    # model.load_state_dict(saved_state_dict)
    # corr2soft.load_state_dict(saved_corr2soft_dict)
    model = nn.DataParallel(model)

    #### ros
    rospy.init_node('LoopDetection', anonymous=True)
    print("Ready to publish detected loops")
    pub = rospy.Publisher('/loop_info', Loops, queue_size=10)
    rospy.Subscriber("/robot_1/submap", SubMap, callback1, queue_size=100)
    rospy.Subscriber("/robot_2/submap", SubMap, callback2, queue_size=100)
    rospy.Subscriber("/robot_3/submap", SubMap, callback3, queue_size=100)
    rospy.spin()
    f.close()
    
