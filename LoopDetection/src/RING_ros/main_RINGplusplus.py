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
import sensor_msgs.msg as sensor_msgs
import pcl

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)

historyKeyframeSearchNum = 15  # 新增的全局参数

# main message information of all robots
Pose1 = []
Time1 = []
# main descriptors of all robots
PC1 = []
BEV1 = []
TIRING1 = []

f = open("./my_loopinfo.txt", "w")

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
def detect_loop_icp(idx_current, pc_current, bev_current, TIRING_current, PC_history, BEV_history, TIRING_history):
    if idx_current < historyKeyframeSearchNum:
        print("No loop detected.")
    else:
        # 限制用于检测的历史帧范围
        Limit_idx = max(0, idx_current - historyKeyframeSearchNum)
        TIRING_candidates = TIRING_history[0:Limit_idx]

        print("PC_history size: ", len(PC_history))
        print("BEV_history size: ", len(BEV_history))
        print("TIRING_history size: ", len(TIRING_history))
        print("Limit_idx: ", Limit_idx)
        print("idx_current: ", idx_current)

        TIRING_idxs = []
        TIRING_dists = []
        TIRING_angles = []
        print("len(TIRING_candidates): %d" % len(TIRING_candidates))
        # 遍历候选帧，计算TIRING距离和角度
        for idx, TIRING_candidate in enumerate(TIRING_candidates):
            dist, angle = fast_corr_RINGplusplus(TIRING_current, TIRING_candidate)

            if dist < cfg.dist_threshold:
                TIRING_idxs.append(idx)  # 保存全局索引
                TIRING_dists.append(dist)
                TIRING_angles.append(angle)

        # 如果没有检测到满足条件的回环
        if len(TIRING_dists) == 0:
            print("No loop detected.")
        else:
            # 找到距离最近的回环候选帧
            idxs_sorted = np.argsort(TIRING_dists)
            idx_top1 = idxs_sorted[0]

            dist = TIRING_dists[idx_top1]
            print("Top 1 TIRING dis: ", dist)
            # 角度计算
            angle_matched = TIRING_angles[idx_top1]
            angle_matched_extra = angle_matched - cfg.num_ring // 2
            angle_matched_rad = angle_matched * 2 * np.pi / cfg.num_ring
            angle_matched_extra_rad = angle_matched_extra * 2 * np.pi / cfg.num_ring

            # 获取匹配帧数据
            idx_matched = TIRING_idxs[idx_top1]
            pc_matched = PC_history[idx_matched]
            bev_matched = BEV_history[idx_matched]

            # 旋转当前帧的BEV表示
            bev_current_rotated = rotate_bev(bev_current, angle_matched_rad)
            bev_current_rotated_extra = rotate_bev(bev_current, angle_matched_extra_rad)

            # 计算平移 (x, y) 和误差
            x, y, error = solve_translation_bev(bev_current_rotated, bev_matched)
            x_extra, y_extra, error_extra = solve_translation_bev(bev_current_rotated_extra, bev_matched)

            # 选择最优的平移和旋转
            if error < error_extra:
                trans_x = x / cfg.num_sector * 140.0
                trans_y = y / cfg.num_ring * 140.0
                rot_yaw = angle_matched_rad
            else:
                trans_x = x_extra / cfg.num_sector * 140.0
                trans_y = y_extra / cfg.num_ring * 140.0
                rot_yaw = angle_matched_extra_rad

            # convert to the BEV coordinate (x: downward, y: right, z: upward)
            trans_x_bev = -trans_y
            trans_y_bev = trans_x

            # convert to the lidar coordinate (x: forward, y: left, z: upward)
            trans_x_lidar = -trans_x_bev
            trans_y_lidar = -trans_y_bev

            # 初始位姿估计
            init_pose = np.linalg.inv(getSE3(trans_x_lidar, trans_y_lidar, rot_yaw))
            print("Estimated translation: x: {}, y: {}, rotation: {}".format(trans_x_lidar, trans_y_lidar, rot_yaw))

            # 执行ICP配准
            times = time.time()
            icp_fitness_score, loop_transform = fast_gicp(
                pc_current, pc_matched, max_correspondence_distance=cfg.icp_max_distance, init_pose=init_pose
            )
            timee = time.time()

            print("ICP fitness score:", icp_fitness_score)              
            print("ICP processed time:", timee - times, 's')


            # 判断ICP结果是否满足阈值条件
            if icp_fitness_score < cfg.icp_fitness_score:
                print("\033[32mICP fitness score is less than threshold, accept the loop.\033[0m")
                Loop_msgs = Loops()
                Loop_msg = Loop()

                # 保存回环检测结果
                # Loop_msg.id0 = idx_current + 1
                # Loop_msg.id1 = idx_matched + 1
                Loop_msg.id0 = robotid_to_key(0) + idx_current + 1
                Loop_msg.id1 = robotid_to_key(0) + idx_matched + 1

                loop_transform = np.linalg.inv(loop_transform)
                pose = get_pose_msg_from_homo_matrix(loop_transform)
                Loop_msg.pose = pose
                Loop_msgs.Loops.append(Loop_msg)

                pub_loop.publish(Loop_msgs)
                print("Loop detected between id ", Loop_msg.id0, " and id ", Loop_msg.id1)          
            else:
                print("\033[31mICP fitness score is larger than threshold, reject the loop.\033[0m")


def callback1(data):
    # 记录回调开始时间
    start_time = time.time()

    # 当前帧的索引
    idx_current = len(PC1)
    
    # 当前帧点云数据处理
    pc = pc2.read_points(data.keyframePC, skip_nans=True, field_names=("x", "y", "z"))
    pc_list = []
    for p in pc:
        pc_list.append([p[0], p[1], p[2]])

    # 转换为 Open3D 点云对象并下采样
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_list)
    pcd = pcd.voxel_down_sample(voxel_size=0.2)

    pc_normalized = load_pc_infer(pcd.points)
    pc = np.asarray(pcd.points)

    # 获取当前帧位姿
    se3 = get_homo_matrix_from_pose_msg(data.pose)

    # # 应用位姿变换到点云
    # pc_transformed = apply_pose_to_pointcloud(pc, se3)

    # # 将变换后的点云更新到 pcd 中
    # pcd.points = o3d.utility.Vector3dVector(pc_transformed)

    # # 将变换后的点云发布
    # pc_msg = pcl_to_ros(pcd)  # 使用变换后的Open3D点云
    # pub_cloud.publish(pc_msg)  # 发布点云消息

    # 生成当前帧的 RING 和 TIRING 描述子
    times = time.time()
    pc_bev, pc_RING, pc_TIRING = generate_RINGplusplus(pc_normalized)
    timee = time.time()
    print("Descriptors generated time:", timee - times, 's')
    
    
    # 如果历史帧数量不足，跳过回环检测
    print("loop detection")
    detect_loop_icp(idx_current, pc, pc_bev, pc_TIRING, PC1, BEV1, TIRING1)

    # 保存当前帧数据
    Pose1.append(se3)
    Time1.append(data.keyframePC.header.stamp)
    PC1.append(pc)
    TIRING1.append(pc_TIRING)
    BEV1.append(pc_bev)


    # 记录回调结束时间
    end_time = time.time()

    # 输出回调函数时长
    callback_duration = end_time - start_time
    print(f"Callback duration: {callback_duration:.4f} seconds")

def apply_pose_to_pointcloud(pc, se3):
    """
    将位姿应用到点云坐标系。
    
    pc: 点云数组 [N, 3]
    se3: 4x4 同质变换矩阵
    
    返回应用位姿后的点云
    """
    # 将点云转换为齐次坐标形式 [N, 4]
    ones = np.ones((pc.shape[0], 1))
    pc_homogeneous = np.hstack((pc, ones))

    # 使用矩阵相乘变换点云
    pc_transformed_homogeneous = pc_homogeneous @ se3.T  # 注意这里是 se3.T，因为 PCL 采用右乘方式
    pc_transformed = pc_transformed_homogeneous[:, :3]  # 去掉最后一列（齐次坐标）

    return pc_transformed


def pcl_to_ros(pcd):
    # 将Open3D点云转换为ROS PointCloud2消息
    pc_ros = sensor_msgs.PointCloud2()
    pc_ros.header.stamp = rospy.Time.now()
    pc_ros.header.frame_id = "base_link"  # 根据需要调整frame_id

    # 转换 Open3D 点云到 ROS PointCloud2 格式
    pc_data = np.asarray(pcd.points)
    pc_ros.height = 1
    pc_ros.width = len(pc_data)
    pc_ros.fields = [
        sensor_msgs.PointField(name="x", offset=0, datatype=sensor_msgs.PointField.FLOAT32, count=1),
        sensor_msgs.PointField(name="y", offset=4, datatype=sensor_msgs.PointField.FLOAT32, count=1),
        sensor_msgs.PointField(name="z", offset=8, datatype=sensor_msgs.PointField.FLOAT32, count=1),
    ]
    pc_ros.is_bigendian = False
    pc_ros.point_step = 12
    pc_ros.row_step = pc_ros.point_step * pc_ros.width
    pc_ros.is_dense = True
    pc_ros.data = pc_data.astype(np.float32).tobytes()

    return pc_ros


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
    pub_loop = rospy.Publisher('/loop_info', Loops, queue_size=10)
    pub_cloud = rospy.Publisher('/processed_point_cloud', sensor_msgs.PointCloud2, queue_size=10)
    rospy.Subscriber("/robot_1/submap", SubMap, callback1)
    rospy.spin()
    f.close()