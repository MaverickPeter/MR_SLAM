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
BEV1 = []
BEV2 = []
BEV3 = []
TIRING1 = []
TIRING2 = []
TIRING3 = []

f = open("./loopinfo.txt", "w")

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


# perform loop detection and apply icp
def detect_loop_icp(robotid_current, idx_current, pc_current, bev_current, TIRING_current, \
                                           robotid_candidate, pc_candidates, bev_candidates, TIRING_candidates):
    
    TIRING_idxs = []
    TIRING_dists = []
    TIRING_angles = []

    for idx in range(len(pc_candidates)):
        dist, angle = fast_corr_RINGplusplus(TIRING_current, TIRING_candidates[idx])

        if dist < cfg.dist_threshold:
            TIRING_idxs.append(idx)
            TIRING_dists.append(dist)
            TIRING_angles.append(angle)

    if len(TIRING_dists) == 0:
        print("No loop detected.")

    else:
        idxs_sorted = np.argsort(TIRING_dists)
        idx_top1 = idxs_sorted[0]

        dist = TIRING_dists[idx_top1]
        print("Top {} TIRING distance: ".format(1), dist)   

        # angle between the two matched RINGs in grids
        angle_matched = TIRING_angles[idx_top1] 
        angle_matched_extra = angle_matched - cfg.num_ring//2

        # convert the matched angle from grids to radians
        angle_matched_rad = angle_matched * 2 * np.pi / cfg.num_ring 
        angle_matched_extra_rad = angle_matched_extra * 2 * np.pi / cfg.num_ring         

        # matched timestamp, pc and RING
        idx_matched = TIRING_idxs[idx_top1]
        pc_matched = pc_candidates[idx_matched]
        bev_matched = bev_candidates[idx_matched]

        # compensated matched RINGs
        bev_current_rotated = rotate_bev(bev_current, angle_matched_rad)
        bev_current_rotated_extra = rotate_bev(bev_current, angle_matched_extra_rad)

        # solve the translation between the two matched RINGs, x: right, y: forward, z: upward (in the RING coordinate)
        # !! change to solve translation using original BEV representation.
        x, y, error = solve_translation_bev(bev_current_rotated, bev_matched)
        x_extra, y_extra, error_extra = solve_translation_bev(bev_current_rotated_extra, bev_matched)
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
        timee = time.time()

        # print("ICP iterations:", iterations)
        print("ICP fitness score:", icp_fitness_score)              
        print("ICP processed time:", timee - times, 's')

        if icp_fitness_score < cfg.icp_fitness_score and robotid_current != robotid_candidate:
            print("\033[32mICP fitness score is less than threshold, accept the loop.\033[0m")
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
            print("\033[31mICP fitness score is larger than threshold, reject the loop.\033[0m")


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

    # generate RING and TIRING descriptors
    times = time.time()
    pc_bev, pc_RING, pc_TIRING = generate_RINGplusplus(pc_normalized)
    timee = time.time()
    print("Descriptors generated time:", timee - times, 's')

    # detect the loop and apply icp
    # candidate robot id: 1
    # robotid_candidate = 0
    # detect_loop_icp(robotid_current, idx_current, pc, pc_TIRING, robotid_candidate, PC1, BEV1, TIRING1)
    # candidate robot id: 2
    robotid_candidate = 1
    detect_loop_icp(robotid_current, idx_current, pc, pc_bev, pc_TIRING, robotid_candidate, PC2, BEV2, TIRING2)
    # candidate robot id: 3
    robotid_candidate = 2
    detect_loop_icp(robotid_current, idx_current, pc, pc_bev, pc_TIRING, robotid_candidate, PC3, BEV3, TIRING3)

    Pose1.append(se3)
    Time1.append(timestamp)
    PC1.append(pc)
    TIRING1.append(pc_TIRING)
    BEV1.append(pc_bev)

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

    # generate RING and TIRING descriptors
    times = time.time()
    pc_bev, pc_RING, pc_TIRING = generate_RINGplusplus(pc_normalized)
    timee = time.time()
    print("Descriptors generated time:", timee - times, 's')

    # detect the loop and apply icp
    # candidate robot id: 1
    robotid_candidate = 0
    detect_loop_icp(robotid_current, idx_current, pc, pc_bev, pc_TIRING, robotid_candidate, PC1, BEV1, TIRING1)
    # candidate robot id: 2
    # robotid_candidate = 1
    # detect_loop_icp(robotid_current, idx_current, pc, pc_TIRING, robotid_candidate, PC2, RING2, BEV2, TIRING2)
    # candidate robot id: 3
    robotid_candidate = 2
    detect_loop_icp(robotid_current, idx_current, pc, pc_bev, pc_TIRING, robotid_candidate, PC3, BEV3, TIRING3)

    Pose2.append(se3)
    Time2.append(timestamp)
    PC2.append(pc)
    TIRING2.append(pc_TIRING)
    BEV2.append(pc_bev)


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

    # generate RING and TIRING descriptors
    times = time.time()
    pc_bev, pc_RING, pc_TIRING = generate_RINGplusplus(pc_normalized)
    timee = time.time()
    print("Descriptors generated time:", timee - times, 's')

    # detect the loop and apply icp
    # candidate robot id: 1
    robotid_candidate = 0
    detect_loop_icp(robotid_current, idx_current, pc, pc_bev, pc_TIRING, robotid_candidate, PC1, BEV1, TIRING1)
    # candidate robot id: 2
    robotid_candidate = 1
    detect_loop_icp(robotid_current, idx_current, pc, pc_bev, pc_TIRING, robotid_candidate, PC2, BEV2, TIRING2)
    # candidate robot id: 3
    # robotid_candidate = 2
    # detect_loop_icp(robotid_current, idx_current, pc, pc_TIRING, robotid_candidate, PC3, BEV3, TIRING3)

    Pose3.append(se3)
    Time3.append(timestamp)
    PC3.append(pc)
    TIRING3.append(pc_TIRING)
    BEV3.append(pc_bev)


if __name__ == "__main__":
    #### load params
    parser = argparse.ArgumentParser(description='PyICP SLAM arguments')
    parser.add_argument('--input_filename', default='./test.bin',
                        help='input file name [default: ./test.bin]')
    parser.add_argument('--input_type', default='point',
                        help='Input data type, can be [point] or scan [image], [default: point]')
    
    parser.add_argument('--num_ring', type=int, default=120) 
    parser.add_argument('--num_sector', type=int, default=120)
    parser.add_argument('--num_height', type=int, default=1) 
    parser.add_argument('--max_length', type=int, default=1)
    parser.add_argument('--max_height', type=int, default=1)
    parser.add_argument('--dist_threshold', type=float, default=0.41) # 0.48 is usually safe (for avoiding false loop closure)
    parser.add_argument('--max_icp_iter', type=int, default=100) # 20 iterations is usually enough
    parser.add_argument('--icp_tolerance', type=float, default=0.001) 
    parser.add_argument('--icp_max_distance', type=float, default=5.0)
    parser.add_argument('--num_icp_points', type=int, default=6000) # 6000 is enough for real time
    parser.add_argument('--icp_fitness_score', type=float, default=0.13) # icp fitness score threshold

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
    
