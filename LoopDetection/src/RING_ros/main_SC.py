import os
import sys
import time
import torch
import rospy
import voxelocc
import argparse
from util import *
from icp import icp
import config as cfg
import numpy as np
import pygicp
import open3d as o3d
from geometry_msgs.msg import Pose
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from tf.transformations import translation_matrix, quaternion_matrix, translation_from_matrix, quaternion_from_matrix
from dislam_msgs.msg import Loop, Loops, SubMap
import pr_methods.ScanContext as SC

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

RING1 = []
RING2 = []
RING3 = []
TIRING1 = []
TIRING2 = []
TIRING3 = []

RingkeyPC1 = []
RingkeyPC2 = []
RingkeyPC3 = []
SC1 = []
SC2 = []
SC3 = []
d_pc = []
yaw_diff_pc = []

f = open("./loopinfo.txt", "w")

# generate scan context
def generate_scan_context(pc):
    size = pc.shape[0]
    pc = pc.transpose().flatten().astype(np.float32)

    # generate sc
    transer_sc = voxelocc.GPUTransformer(pc, size, cfg.max_length, cfg.max_height, cfg.num_ring, cfg.num_sector, cfg.num_height, 1)
    transer_sc.transform()
    point_t_sc = transer_sc.retreive()
    point_t_sc = point_t_sc.reshape(-1, 3)
    point_t_sc = point_t_sc[...,2]
    pc_sc = point_t_sc.reshape(cfg.num_height, cfg.num_ring, cfg.num_sector)
    
    return pc_sc


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

    # align the point cloud using the initial pose calculated by RING
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


# SC: perform loop detection and apply icp
def detect_loop_icp_SC(robotid_current, idx_current, pc_current, SC_current, Ringkey_current, \
                                           robotid_candidate, pc_candidates, SC_candidates, Ringkey_candidates):
    

    num_candidates = 1
    if len(Ringkey_candidates) < 1:
        return
    kdtree_pc = KDTree(np.array(Ringkey_candidates)) # ScanContext and ScanContext++ (PC)
    dists_pc, idxs_pc = kdtree_pc.query(np.array([Ringkey_current]), k=num_candidates)

    for i in range(num_candidates):
        idx_sc = idxs_pc[0][i]
        SC_candidate = SC_candidates[idx_sc] 
        # compute the cosine distance between the two scancontexts
        dist_pc, yaw_pc = SC.dist_align_sc(SC_candidate, SC_current, search_ratio=0.1)
        d_pc.append(dist_pc)
        yaw_diff_pc.append(yaw_pc)
        idx_top1_pc = idxs_pc[0][0]

        pred_angle_pc = yaw_diff_pc[0] # in grids
        pred_angle_pc = pred_angle_pc * 2 * np.pi / cfg.num_sector # in radians
        init_pose_pc = getSE3(0, 0, pred_angle_pc)

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
            print("Scan Context: Loop detected between id ", Loop_msg.id0, " and id ", Loop_msg.id1)          
        else:
            print("Scan Context: ICP fitness score is larger than threshold, reject the loop.")


# RING: perform loop detection and apply icp
def detect_loop_icp_RING(robotid_current, idx_current, pc_current, RING_current, TIRING_current, \
                                           robotid_candidate, pc_candidates, RING_candidates, TIRING_candidates):
    
    RING_idxs = []
    RING_dists = []
    RING_angles = []

    for idx in range(len(pc_candidates)):
        dist, angle = fast_corr(TIRING_current, TIRING_candidates[idx])
        # print("Distance bewteen two RING descriptors: ", dist)
        # print("Shifted angle bewteen two RING descriptors: ", angle)
        if dist < cfg.dist_threshold:
            RING_idxs.append(idx)
            RING_dists.append(dist)
            RING_angles.append(angle)

    if len(RING_dists) == 0:
        print("No loop detected.")

    else:
        dists_sorted = np.sort(RING_dists)
        idxs_sorted = np.argsort(RING_dists)
        # downsample the point cloud
        # pc_down_pts = random_sampling(pc_current, num_points=cfg.num_icp_points)
        idx_top1 = idxs_sorted[0]
        dist_top1 = RING_dists[idx_top1]

        # dist = RING_dists[idx]
        print("Top {} RING distance: ".format(1), dist)        
        # angle between the two matched RINGs in grids
        angle_matched = RING_angles[idx_top1] 
        angle_matched_extra = angle_matched - cfg.num_ring//2
        # convert the matched angle from grids to radians
        angle_matched_rad = angle_matched * 2 * np.pi / cfg.num_ring 
        angle_matched_extra_rad = angle_matched_extra * 2 * np.pi / cfg.num_ring         
        # row shift between the two matched RINGs to compensate the rotation
        row_shift = calculate_row_shift(angle_matched)
        row_shift_extra = calculate_row_shift(angle_matched_extra)
        # matched timestamp, pc and RING
        idx_matched = RING_idxs[idx_top1]
        pc_matched = pc_candidates[idx_matched]
        RING_matched = RING_candidates[idx_matched]                 
        # compensated matched RINGs
        RING_matched_shifted = torch.roll(RING_matched, row_shift, dims=1)
        RING_matched_shifted_extra = torch.roll(RING_matched, row_shift_extra, dims=1)

        # solve the translation between the two matched RINGs, x: right, y: forward, z: upward (in the RING coordinate)
        x, y, error = solve_translation(RING_current, RING_matched_shifted, angle_matched_rad, device)
        x_extra, y_extra, error_extra = solve_translation(RING_current, RING_matched_shifted_extra, angle_matched_extra_rad, device)
        if error < error_extra:
            trans_x = x / cfg.num_sector * 140.  # in meters
            trans_y = y / cfg.num_ring * 140.  # in meters
            rot_yaw = angle_matched_rad  # in radians
        else:
            trans_x = x_extra / cfg.num_sector * 140.  # in meters
            trans_y = y_extra / cfg.num_ring * 140.  # in meters 
            rot_yaw = angle_matched_extra_rad  # in radians
        
        # !!! convert the estimation results to the lidar coordinate
        # convert to the BEV coordinate (x: downward, y: right, z: upward)
        trans_x_bev = -trans_y
        trans_y_bev = trans_x

        # convert to the lidar coordinate (x: forward, y: left, z: upward)
        trans_x_lidar = -trans_x_bev
        trans_y_lidar = -trans_y_bev

        init_pose = np.linalg.inv(getSE3(trans_x_lidar, trans_y_lidar, rot_yaw))
        print("Loop detected.")
        print("Estimated translation: x: {}, y: {}, rotation: {}".format(trans_x_lidar, trans_y_lidar, rot_yaw))

        # apply ICP to the matched point clouds 
        # pc_matched_down_pts = random_sampling(pc_matched, num_points=cfg.num_icp_points)
        times = time.time()
        # loop_transform, distances, iterations = icp(pc_down_pts, pc_matched_down_pts, init_pose=init_pose, max_iterations=cfg.max_icp_iter, tolerance=cfg.icp_tolerance)
        # icp_fitness_score, loop_transform = o3d_icp(pc_current, pc_matched, tolerance=cfg.icp_tolerance, init_pose=init_pose)
        icp_fitness_score, loop_transform = fast_gicp(pc_current, pc_matched, max_correspondence_distance=cfg.icp_max_distance, init_pose=init_pose)
        # icp_fitness_score = distances.mean() 
        timee = time.time()
        # print("ICP iterations:", iterations)
        print("ICP fitness score:", icp_fitness_score)              
        print("ICP processed time:", timee - times, 's')

        if icp_fitness_score < cfg.icp_fitness_score and robotid_current != robotid_candidate:
            print("ICP fitness score is less than threshold, accept the loop.")
            Loop_msgs = Loops()
            # publish the result of loop detection and icp
            Loop_msg = Loop()
            # id0 contain the current robot id and its RING index, id1 contain the matched robot id and its RING index       
            Loop_msg.id0 = robotid_to_key(robotid_current) + idx_current + 1
            Loop_msg.id1 = robotid_to_key(robotid_candidate) + idx_matched + 1
            # !!! convert the pointcloud transformation matrix to the frame transformation matrix
            loop_transform = np.linalg.inv(loop_transform)
            # convert the transform matrix to the format of position and orientation
            pose = get_pose_msg_from_homo_matrix(loop_transform)
            Loop_msg.pose = pose
            line = [robotid_current, idx_current, robotid_candidate, idx_matched, pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            line = ' '.join(str(i) for i in line)
            f.write(line)
            f.write("\n")
            Loop_msgs.Loops.append(Loop_msg)
            pub.publish(Loop_msgs)      
            print("Loop detected between id ", Loop_msg.id0, " and id ", Loop_msg.id1)          
        else:
            print("ICP fitness score is larger than threshold, reject the loop.")


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
    
    pc_normalized = load_pc_infer(pc_list)
    pc = np.asarray(pcd.points)
    
    # get the pose of the keyframe point cloud 
    # convert position and quaternion pose to se3 matrix
    se3 = get_homo_matrix_from_pose_msg(data.pose)

    # generate RING and TIRING descriptors
    times = time.time()
    pc_sc = generate_scan_context(pc_normalized)
    ringkey_pc = SC.make_ringkey(pc_sc) 

    # pc_bev, pc_RING, pc_TIRING, _ = generate_RING(pc_normalized)
    timee = time.time()
    print("Descriptors generated time:", timee - times, 's')

    # detect the loop and apply icp
    # candidate robot id: 1
    # robotid_candidate = 0
    # detect_loop_icp_RING(robotid_current, idx_current, pc, pc_RING, pc_TIRING, robotid_candidate, PC1, RING1, TIRING1)
    # candidate robot id: 2
    robotid_candidate = 1
    detect_loop_icp_SC(robotid_current, idx_current, pc, pc_sc, ringkey_pc, robotid_candidate, PC2, SC2, RingkeyPC2)
    # detect_loop_icp_RING(robotid_current, idx_current, pc, pc_RING, pc_TIRING, robotid_candidate, PC2, RING2, TIRING2)
    # candidate robot id: 3
    robotid_candidate = 2
    detect_loop_icp_SC(robotid_current, idx_current, pc, pc_sc, ringkey_pc, robotid_candidate, PC3, SC3, RingkeyPC3)

    # detect_loop_icp_RING(robotid_current, idx_current, pc, pc_RING, pc_TIRING, robotid_candidate, PC3, RING3, TIRING3)

    Pose1.append(se3)
    Time1.append(timestamp)
    PC1.append(pc)
    SC1.append(pc_sc)
    RingkeyPC1.append(ringkey_pc)

    # RING1.append(pc_RING)
    # TIRING1.append(pc_TIRING)


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

    pc_normalized = load_pc_infer(pc_list)
    pc = np.asarray(pcd.points)

    # get the pose of the keyframe point cloud 
    # convert position and quaternion pose to se3 matrix
    se3 = get_homo_matrix_from_pose_msg(data.pose)

    # generate RING and TIRING descriptors
    times = time.time()
    pc_sc = generate_scan_context(pc_normalized)
    ringkey_pc = SC.make_ringkey(pc_sc) 
    # pc_bev, pc_RING, pc_TIRING, _ = generate_RING(pc_normalized)
    timee = time.time()
    print("Descriptors generated time:", timee - times, 's')

    # detect the loop and apply icp
    # candidate robot id: 1
    robotid_candidate = 0
    detect_loop_icp_SC(robotid_current, idx_current, pc, pc_sc, ringkey_pc, robotid_candidate, PC1, SC1, RingkeyPC1)
    # detect_loop_icp_RING(robotid_current, idx_current, pc, pc_RING, pc_TIRING, robotid_candidate, PC1, RING1, TIRING1)
    # candidate robot id: 2
    # robotid_candidate = 1
    # detect_loop_icp_RING(robotid_current, idx_current, pc, pc_RING, pc_TIRING, robotid_candidate, PC2, RING2, TIRING2)
    # candidate robot id: 3
    robotid_candidate = 2
    detect_loop_icp_SC(robotid_current, idx_current, pc, pc_sc, ringkey_pc, robotid_candidate, PC3, SC3, RingkeyPC3)
    # detect_loop_icp_RING(robotid_current, idx_current, pc, pc_RING, pc_TIRING, robotid_candidate, PC3, RING3, TIRING3)

    Pose2.append(se3)
    Time2.append(timestamp)
    PC2.append(pc)
    # RING2.append(pc_RING)
    # TIRING2.append(pc_TIRING)
    SC2.append(pc_sc)
    RingkeyPC2.append(ringkey_pc)


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

    pc_normalized = load_pc_infer(pc_list)
    pc = np.asarray(pcd.points)

    # get the pose of the keyframe point cloud 
    # convert position and quaternion pose to se3 matrix
    se3 = get_homo_matrix_from_pose_msg(data.pose)

    # generate RING and TIRING descriptors
    times = time.time()
    pc_sc = generate_scan_context(pc_normalized)
    ringkey_pc = SC.make_ringkey(pc_sc) 
    # pc_bev, pc_RING, pc_TIRING, _ = generate_RING(pc_normalized)
    timee = time.time()
    print("Descriptors generated time:", timee - times, 's')

    # detect the loop and apply icp
    # candidate robot id: 1
    robotid_candidate = 0
    detect_loop_icp_SC(robotid_current, idx_current, pc, pc_sc, ringkey_pc, robotid_candidate, PC1, SC1, RingkeyPC1)
    # detect_loop_icp_RING(robotid_current, idx_current, pc, pc_RING, pc_TIRING, robotid_candidate, PC1, RING1, TIRING1)
    # candidate robot id: 2
    robotid_candidate = 1
    detect_loop_icp_SC(robotid_current, idx_current, pc, pc_sc, ringkey_pc, robotid_candidate, PC2, SC2, RingkeyPC2)

    # detect_loop_icp_RING(robotid_current, idx_current, pc, pc_RING, pc_TIRING, robotid_candidate, PC2, RING2, TIRING2)
    # candidate robot id: 3
    # robotid_candidate = 2
    # detect_loop_icp_RING(robotid_current, idx_current, pc, pc_RING, pc_TIRING, robotid_candidate, PC3, RING3, TIRING3)

    Pose3.append(se3)
    Time3.append(timestamp)
    PC3.append(pc)
    SC3.append(pc_sc)
    RingkeyPC3.append(ringkey_pc)


if __name__ == "__main__":
    #### load params
    parser = argparse.ArgumentParser(description='PyICP SLAM arguments')
    parser.add_argument('--input_filename', default='./test.bin',
                        help='input file name [default: ./test.bin]')
    parser.add_argument('--input_type', default='point',
                        help='Input data type, can be [point] or scan [image], [default: point]')
    
    parser.add_argument('--num_ring', type=int, default=40) 
    parser.add_argument('--num_sector', type=int, default=120)
    parser.add_argument('--num_height', type=int, default=1) 
    parser.add_argument('--max_length', type=int, default=1)
    parser.add_argument('--max_height', type=int, default=1)
    parser.add_argument('--dist_threshold', type=float, default=0.48) # 0.48 is usually safe (for avoiding false loop closure)
    parser.add_argument('--max_icp_iter', type=int, default=20) # 20 iterations is usually enough
    parser.add_argument('--icp_tolerance', type=float, default=0.001) 
    parser.add_argument('--icp_max_distance', type=float, default=5.0)
    parser.add_argument('--num_icp_points', type=int, default=6000) # 6000 is enough for real time
    parser.add_argument('--icp_fitness_score', type=float, default=0.08) # icp fitness score threshold

    args = parser.parse_args()

    #### load params
    cfg.input_type = args.input_type
    cfg.num_ring = args.num_ring
    cfg.num_sector = args.num_sector
    cfg.num_height = args.max_height
    cfg.max_length = args.max_length
    cfg.max_height = args.max_height
    cfg.dist_threshold = args.dist_threshold
    cfg.max_icp_iter = args.max_icp_iter
    cfg.icp_tolerance = args.icp_tolerance
    cfg.num_icp_points = args.num_icp_points
    cfg.icp_fitness_score = args.icp_fitness_score
    

    #### ros
    rospy.init_node('LoopDetection', anonymous=True)
    print("Ready to publish detected loops")
    pub = rospy.Publisher('/loop_info', Loops, queue_size=10)
    rospy.Subscriber("/robot_1/submap", SubMap, callback1)
    rospy.Subscriber("/robot_2/submap", SubMap, callback2)
    rospy.Subscriber("/robot_3/submap", SubMap, callback3)
    rospy.spin()
    f.close()
    
