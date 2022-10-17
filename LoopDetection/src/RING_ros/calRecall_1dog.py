'''
Evaluate the place recognition performance of M2DP, Fast Histogram, Scan Context, 
Scan Context++, and RING methods using the given point cloud of just one robot.
The ground truth point cloud dataset is in ../../data/GT_1dog folder.
'''
from cgitb import enable
import os
import sys
import pygicp
from util import *
import config as cfg
import numpy as np
import open3d as o3d
import seaborn as sns
import matplotlib.pyplot as plt
import pr_methods.M2DP as M2DP
import pr_methods.ScanContext as SC
import pr_methods.FastHistogram as FH
import pr_methods.RadonSinogram as RS
from sklearn.neighbors import KDTree
from tf.transformations import translation_matrix, quaternion_matrix
from sys import getsizeof

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)

np.set_printoptions(threshold=np.inf)

# find closest place timestamp with index returned
def find_closest_timestamp(A, target):
    # A must be sorted
    idx = A.searchsorted(target)
    idx = np.clip(idx, 1, len(A)-1)
    left = A[idx-1]
    right = A[idx]
    idx -= target - left < right - target
    return idx


# read pcd file and return the point cloud
def read_pcd(filename):
    # read the pcd file
    pcd = o3d.t.io.read_point_cloud(filename)
    # get the point cloud
    pc = np.asarray(pcd.point['positions'].numpy())
    intensity = np.asarray(pcd.point['intensity'].numpy())
    pc_intensity = np.concatenate((pc, intensity), axis=1)

    # pcd = o3d.io.read_point_cloud(filename)
    # # get the point cloud
    # pc = np.asarray(pcd.points)
    return pcd, pc, pc_intensity


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


def get_robotid(name):
    if name[:2] == "69":
        robotid = 2
    elif name[:2] == "70":
        robotid = 3
    return robotid


# get linear translation (x & y) and yaw angle from a SE3 rotation matrix
def se3_to_3dposes(R):
    # extract the translation and rotation matrix
    rot_mat = R[:3, :3]
    translation = R[:3, 3]
    # get the x and y translation
    x = translation[0]
    y = translation[1]
    # calculate the yaw angle from the rotation matrix
    yaw = np.arctan2(rot_mat[1, 0], rot_mat[0, 0])

    return x, y, yaw

# achieve point-to-point icp with open3d
def o3d_icp(source, target, max_correspondence_distance=1.0, init_pose=None):
    # convert the point cloud to o3d point cloud
    # pcd1 = o3d.geometry.PointCloud()
    # pcd1.points = o3d.utility.Vector3dVector(source)
    # pcd2 = o3d.geometry.PointCloud()
    # pcd2.points = o3d.utility.Vector3dVector(target)

    # apply outlier removal
    source.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
    target.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)

    # run icp
    result = o3d.pipelines.registration.registration_icp(source, target, max_correspondence_distance, init_pose,
                    o3d.pipelines.registration.TransformationEstimationPointToPoint())

    # get the icp fitness score
    fitness = result.fitness

    # get the transformation matrix
    T_matrix = result.transformation

    return fitness, T_matrix


# apply icp using fast_gicp (https://github.com/SMRT-AIST/fast_gicp)
def fast_gicp(source, target, max_correspondence_distance=1.0, init_pose=np.eye(4)):
    # downsample the point cloud before registration
    source = pygicp.downsample(source, 0.5)
    target = pygicp.downsample(target, 0.5)

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
    fitness = gicp.get_fitness_score(max_range=1.0)
    # get the transformation matrix
    T_matrix = gicp.get_final_transformation()

    return fitness, T_matrix


# visualize the icp result of open3d
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # set the color of the point cloud
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, 0])
    o3d.visualization.draw_geometries([coordinate_frame, source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])


# visualize the icp result of fast gicp
def draw_registration_result_fast_gicp(source, target, transformation):
    # convert the point cloud to o3d point cloud
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(source)
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(target)
    # set the color of the point cloud
    pcd1.paint_uniform_color([1, 0.706, 0])
    pcd2.paint_uniform_color([0, 0.651, 0.929])
    pcd1.transform(transformation)
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, 0])
    o3d.visualization.draw_geometries([coordinate_frame, pcd1, pcd2],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])


gt_vertex = open("./gt_pose.txt", "r")
gt_vertex = np.array([line.split() for line in gt_vertex])

pcd_path = "/home/client/Projects/SLAM/hdl_ws/dump/Keyframes/"
pcd_files = os.listdir(pcd_path)
pcd_files.sort()
pcd_filename = 'cloud.pcd' # each sub-folder contains one point cloud with the same name

# main descriptors of different pr methods
if cfg.enable_pc:
    RingkeyPC1 = []

if cfg.enable_m2dp:
    M2DP1 = []

if cfg.enable_fh:
    FH1 = []

if cfg.enable_sc:
    SC1 = []

if cfg.enable_cc:
    BEV1 = []
    RingkeyCC1 = []

# if cfg.enable_rs:
TIRING1 = []
RING1 = []
KDRING = []
PC1 = []
Pose1 = []
XYPose1 = []

# pcd files names in g2o file
filenames = np.array([int(i) for i in gt_vertex[:,0]])

for pcd_file in pcd_files:
    filename = int(pcd_file) + 1 # add 1 to the file name
    idx = find_closest_timestamp(filenames, filename)
    trans = translation_matrix([float(gt_vertex[idx, 1]), float(gt_vertex[idx, 2]), float(gt_vertex[idx, 3])])
    quat = quaternion_matrix([float(gt_vertex[idx, 4]), float(gt_vertex[idx, 5]), float(gt_vertex[idx, 6]), float(gt_vertex[idx, 7])])
    se3 = np.dot(trans, quat)      
    x, y, yaw = se3_to_3dposes(se3)
    pcd_file_path = pcd_path + pcd_file + '/' + pcd_filename

    # load the pcd file
    pcd, pc, pc_intensity = read_pcd(pcd_file_path)
    # normalize the point cloud to -1~1
    pc_normalized = load_pc_infer(pc_intensity)
    print("pc_normalized shape: ", pc_normalized.shape)

    # generate the RING and TIRING descriptors
    if cfg.enable_rs or cfg.enable_cc:
        pc_bev, pc_RING, pc_TIRING, pc_kdRing = generate_RING(pc_normalized)    
        start = time.time()
        ringkey_cc = SC.make_ringkey(pc_bev)
        end = time.time()
        print("generate cc cost: ", end - start)
        # print("pc_kdRing shape: ", np.asarray(pc_kdRing).shape)
        KDRING.append(pc_kdRing)
        # print("RING memory: ",round(getsizeof(pc_RING) / 1024.0,5))
        print("ring shape", pc_RING.shape)
        RING1.append(pc_RING)
        TIRING1.append(pc_TIRING)

    # extracting M2DP feature of the point cloud
    if cfg.enable_m2dp:
        start = time.time()
        desM2DP, A = M2DP.M2DP(pc)
        print("pc shape", pc.shape)
        print("M2DP memory: ",(desM2DP.nbytes) / 1024.0)
        end = time.time()
        print("generate M2DP cost: ", end - start)
        M2DP1.append(desM2DP)

    # extracting FastHistogram feature of the point cloud
    if cfg.enable_fh:
        start = time.time()
        desFH = FH.statisticsOnRange(pc_normalized, b=100)
        end = time.time()
        print("generate fasthistogram cost: ", end - start)
        print("fasth memory: ",(desFH.nbytes) / 1024.0)

        FH1.append(desFH)

    # extracting ScanContext feature of the point cloud
    if cfg.enable_sc or cfg.enable_pc:
        start = time.time()
        pc_sc = generate_scan_context(pc_normalized)
        end1 = time.time()
        print("generate sc cost: ", end1 - start)
        ringkey_pc = SC.make_ringkey(pc_sc) 
        end = time.time()
        print("generate pc cost: ", end - start)
        print("pc_sc memory: ",(ringkey_pc.nbytes) / 1024.0)
        print("sc memory: ",(pc_sc.nbytes) / 1024.0)

        SC1.append(pc_sc)
        RingkeyPC1.append(ringkey_pc)

    if cfg.enable_cc:
        BEV1.append(pc_bev)
        print("cc memory: ",ringkey_cc.nbytes / 1024.0)

        RingkeyCC1.append(ringkey_cc)

    PC1.append(pc)
    Pose1.append(se3)
    XYPose1.append([x,y])

# M2DP
if cfg.enable_m2dp:
    num_correct_m2dp = 0
    num_icp_correct_m2dp  = 0
    yaw_error_m2dp  = []
    x_lidar_error_m2dp  = []
    y_lidar_error_m2dp  = []
    M2DP_existed = []

# FastHistogram
if cfg.enable_fh:
    num_correct_fhist = 0
    num_icp_correct_fhist = 0
    yaw_error_fhist = []
    x_lidar_error_fhist = []
    y_lidar_error_fhist = []
    FH_existed = []

# ScanContext
if cfg.enable_sc:
    num_correct_sc = 0
    num_icp_correct_sc = 0
    yaw_error_sc = []
    x_lidar_error_sc = []
    y_lidar_error_sc = []
    SC_existed = []

# ScanContext++ - CC
if cfg.enable_cc:
    num_correct_cc = 0
    num_icp_correct_cc = 0
    yaw_error_cc = []
    x_lidar_error_cc = []
    y_lidar_error_cc = []
    BEV_existed = []
    RingkeyCC_existed = []

# ScanContext++ - PC
if cfg.enable_pc:
    num_correct_pc = 0
    num_icp_correct_pc = 0
    yaw_error_pc = []
    x_lidar_error_pc = []
    y_lidar_error_pc = []
    RingkeyPC_existed = []

# RadonSinogram
if cfg.enable_rs:
    num_correct_kdrs = 0
    num_correct_rs = 0
    num_icp_correct_rs = 0
    yaw_error_rs = []
    x_lidar_error_rs = []
    y_lidar_error_rs = []
    RING_existed = []
    TIRING_existed = []
    KDRING_existed = []

# Existed descriptors
PC_existed = []
Pose_existed = []
XYPose_existed = []

num_candidates_sc = 10 # number of candidates for SC and SC++
num_gt_loops = 0

# Single session loop closure
# Use the data collected as query and map simultaneously
for idx in range(len(PC1)):
    pc_query = PC1[idx]
    if cfg.enable_m2dp:
        M2DP_query = M2DP1[idx]
    if cfg.enable_fh:
        FH_query = FH1[idx]
    if cfg.enable_sc:
        SC_query = SC1[idx]
        RingkeyPC_query = RingkeyPC1[idx]
    if cfg.enable_cc:
        BEV_query = BEV1[idx]
        RingkeyCC_query = RingkeyCC1[idx]
    if cfg.enable_rs:
        RING_query = RING1[idx]
        TIRING_query = TIRING1[idx]
        KDRING_query = KDRING[idx]

    Pose_query = Pose1[idx]    
    XYPose_query = XYPose1[idx]
    
    # exclude the recent nodes for loop detection
    if idx > cfg.exclude_recent_nodes:
        idx_skip_recent = idx - cfg.exclude_recent_nodes
        PC_skip_recent = PC_existed[0:(idx_skip_recent+1)]
        if cfg.enable_pc:
            RingKeyPC_skip_recent = RingkeyPC_existed[0:(idx_skip_recent+1)]
        if cfg.enable_m2dp:
            M2DP_skip_recent = M2DP_existed[0:(idx_skip_recent+1)]
        if cfg.enable_fh:
            FH_skip_recent = FH_existed[0:(idx_skip_recent+1)]
        if cfg.enable_sc:
            SC_skip_recent = SC_existed[0:(idx_skip_recent+1)]
        if cfg.enable_cc:
            BEV_skip_recent = BEV_existed[0:(idx_skip_recent+1)]
            RingkeyCC_skip_recent = RingkeyCC_existed[0:(idx_skip_recent+1)]
        if cfg.enable_rs:
            RING_skip_recent = RING_existed[0:(idx_skip_recent+1)]
            TIRING_skip_recent = TIRING_existed[0:(idx_skip_recent+1)]
            KDRING_skip_recent = KDRING_existed[0:(idx_skip_recent+1)]
        Pose_skip_recent = Pose_existed[0:(idx_skip_recent+1)]    
        XYPose_skip_recent = XYPose_existed[0:(idx_skip_recent+1)]

        # build kd tree of the map descriptors
        if cfg.enable_m2dp:
            kdtree_m2dp = KDTree(np.array(M2DP_skip_recent)) # M2DP
        if cfg.enable_fh:
            kdtree_fhist = KDTree(np.array(FH_skip_recent)) # FastHistogram
        if cfg.enable_cc:
            kdtree_cc = KDTree(np.array(RingkeyCC_skip_recent)) # ScanContext++ (CC)
        if cfg.enable_pc:
            kdtree_pc = KDTree(np.array(RingKeyPC_skip_recent)) # ScanContext and ScanContext++ (PC)
        if cfg.enable_rs:
            kdtree_rs = KDTree(np.array(KDRING_skip_recent))

        kdtree_pose = KDTree(np.array(XYPose_skip_recent)) # Ground truth pose

        # print("Query pose x, y, yaw:", se3_to_3dposes(Pose_query))
        
        num_candidates = min(len(PC_skip_recent), num_candidates_sc)
        # find the closest descriptor in the map
        if cfg.enable_m2dp: 
            dists_m2dp, idxs_m2dp = kdtree_m2dp.query(np.array([M2DP_query]))
        if cfg.enable_fh:
            dists_fhist, idxs_fhist = kdtree_fhist.query(np.array([FH_query]))
        if cfg.enable_cc: 
            dists_cc, idxs_cc = kdtree_cc.query(np.array([RingkeyCC_query]), k=num_candidates)
        if cfg.enable_pc: 
            dists_pc, idxs_pc = kdtree_pc.query(np.array([RingkeyPC_query]), k=num_candidates)
        if cfg.enable_rs:
            dists_rs, idxs_rs = kdtree_rs.query(np.array([KDRING_query]), k=1)


        idxs_pose = kdtree_pose.query_radius(np.array([XYPose_query]), r=cfg.revisit_criteria)
        
        # num_gt_loops += len(idxs_pose)
        if len(idxs_pose[0]) > 0:
            num_gt_loops += 1
            isGTcorrect = True
        else:
            isGTcorrect = False
        
        # descriptor distance between query and map candidates
        if cfg.enable_sc: 
            d_sc = [] # original ScanContext
        if cfg.enable_cc:     
            d_cc = [] # ScanContext++ (CC)
        if cfg.enable_pc:     
            d_pc = [] # ScanContext++ (PC)

        # yaw difference between query and map candidates
        if cfg.enable_sc: 
            yaw_diff_sc = []
        if cfg.enable_cc: 
            lateral_diff_cc = []
        if cfg.enable_pc: 
            yaw_diff_pc = []
        if cfg.enable_rs: 
            yaw_diff_rs = []
            yaw_diff_rs_fft = []

        # ScanContext and ScanContext++ (PC and CC)
        if cfg.enable_sc or cfg.enable_cc or cfg.enable_pc: 
            for i in range(num_candidates):
                idx_cc = idxs_cc[0][i]
                idx_sc = idxs_pc[0][i]
                BEV_candidate = BEV_skip_recent[idx_cc]
                SC_candidate = SC_skip_recent[idx_sc] 
                # compute the cosine distance between the two scancontexts
                dist_sc, yaw_sc = SC.distance_sc(SC_query, SC_candidate)
                dist_cc, lateral_cc = SC.dist_align_sc(BEV_candidate, BEV_query, search_ratio=0.1)
                dist_pc, yaw_pc = SC.dist_align_sc(SC_candidate, SC_query, search_ratio=0.1)
                d_sc.append(dist_sc)
                yaw_diff_sc.append(yaw_sc)
                d_pc.append(dist_pc)
                yaw_diff_pc.append(yaw_pc)
                d_cc.append(dist_cc)
                lateral_diff_cc.append(lateral_cc)

        # RadonSinogram (RING)
        if cfg.enable_rs: 
            RING_dists = []
            RING_angles = []    
            for idx_map in range(len(PC_skip_recent)):
                RING_map = RING_skip_recent[idx_map]
                TIRING_map = TIRING_skip_recent[idx_map]
                # compute the correlation between the two RINGs
                dist, angle = fast_corr(TIRING_query, TIRING_map)
                RING_dists.append(dist)
                RING_angles.append(angle)

        # sort the similarity distance
        if cfg.enable_sc: 
            d_sc_sorted = np.sort(d_sc)
            idxs_sc_sorted = np.argsort(d_sc)
        if cfg.enable_cc: 
            d_cc_sorted = np.sort(d_cc)
            idxs_cc_sorted = np.argsort(d_cc)
        if cfg.enable_pc: 
            d_pc_sorted = np.sort(d_pc)
            idxs_pc_sorted = np.argsort(d_pc)
        if cfg.enable_rs: 
            d_rs_sorted = np.sort(RING_dists)
            idxs_rs_sorted = np.argsort(RING_dists)      

        # top 1 index of retrieved candidates
        if cfg.enable_m2dp: 
            idx_top1_m2dp = idxs_m2dp[0][0]
            Pose_m2dp = Pose_skip_recent[idx_top1_m2dp]
            T_m2dp = np.dot(np.linalg.inv(Pose_m2dp), Pose_query) 
            gt_x_m2dp, gt_y_m2dp, gt_yaw_m2dp = se3_to_3dposes(T_m2dp)
            dist_m2dp = np.linalg.norm(np.array([gt_x_m2dp, gt_y_m2dp]))
            if dist_m2dp < cfg.revisit_criteria:
                num_correct_m2dp += 1

        if cfg.enable_fh:
            idx_top1_fhist = idxs_fhist[0][0]
            Pose_fhist = Pose_skip_recent[idx_top1_fhist]
            T_fhist = np.dot(np.linalg.inv(Pose_fhist), Pose_query) 
            gt_x_fhist, gt_y_fhist, gt_y_fhkdrs = se3_to_3dposes(T_fhist)
            dist_fhist = np.linalg.norm(np.array([gt_x_fhist, gt_y_fhist]))
            if dist_fhist < cfg.revisit_criteria:
                num_correct_fhist += 1

        if cfg.enable_sc:
            idx_top1_sc = idxs_pc[0][idxs_sc_sorted[0]]
            Pose_sc = Pose_skip_recent[idx_top1_sc]
            T_sc = np.dot(np.linalg.inv(Pose_sc), Pose_query)
            gt_x_sc, gt_y_sc, gt_yaw_sc = se3_to_3dposes(T_sc)
            dist_sc = np.linalg.norm(np.array([gt_x_sc, gt_y_sc]))
            if dist_sc < cfg.revisit_criteria:
                num_correct_sc += 1

        if cfg.enable_cc:    
            idx_top1_cc = idxs_cc[0][idxs_cc_sorted[0]]
            Pose_cc = Pose_skip_recent[idx_top1_cc]
            T_cc = np.dot(np.linalg.inv(Pose_cc), Pose_query)
            gt_x_cc, gt_y_cc, gt_yaw_cc = se3_to_3dposes(T_cc)
            dist_cc = np.linalg.norm(np.array([gt_x_cc, gt_y_cc]))
            if dist_cc < cfg.revisit_criteria:
                num_correct_cc += 1

        if cfg.enable_pc:    
            idx_top1_pc = idxs_pc[0][idxs_pc_sorted[0]]
            Pose_pc = Pose_skip_recent[idx_top1_pc]
            T_pc = np.dot(np.linalg.inv(Pose_pc), Pose_query)
            gt_x_pc, gt_y_pc, gt_yaw_pc = se3_to_3dposes(T_pc)
            dist_pc = np.linalg.norm(np.array([gt_x_pc, gt_y_pc]))
            if dist_pc < cfg.revisit_criteria:
                num_correct_pc += 1

        if cfg.enable_rs:
            idx_top1_kdrs = idxs_rs[0][0]
            Pose_kdrs = Pose_skip_recent[idx_top1_kdrs]
            T_kdrs = np.dot(np.linalg.inv(Pose_kdrs), Pose_query) 
            gt_x_kdrs, gt_y_kdrs, gt_yaw_kdrs = se3_to_3dposes(T_kdrs)
            dist_kdrs = np.linalg.norm(np.array([gt_x_kdrs, gt_y_kdrs]))
            if dist_kdrs < cfg.revisit_criteria:
                num_correct_kdrs += 1

            idx_top1_rs = idxs_rs_sorted[0]
            Pose_rs = Pose_skip_recent[idx_top1_rs]
            T_rs = np.dot(np.linalg.inv(Pose_rs), Pose_query)
            gt_x_rs, gt_y_rs, gt_yaw_rs = se3_to_3dposes(T_rs)
            dist_rs = np.linalg.norm(np.array([gt_x_rs, gt_y_rs]))
            if dist_rs < cfg.revisit_criteria:
                num_correct_rs += 1
            # elif dist_rs > cfg.revisit_criteria and isGTcorrect:
            #     pc_normalized = load_pc_infer(pc_query)

            #     size = pc_normalized.shape[0]
            #     pc_normalized = pc_normalized[:,0:3]
            #     pc = pc_normalized.transpose().flatten().astype(np.float32)

            #     # generate bev
            #     transer_bev = voxelocc.GPUTransformer(pc, size, cfg.max_length, cfg.max_height, cfg.num_ring, cfg.num_sector, cfg.num_height, 1)
            #     transer_bev.transform()
            #     point_t_bev = transer_bev.retreive()
            #     point_t_bev = point_t_bev.reshape(-1, 3)
            #     point_t_bev = point_t_bev[...,2]
            #     pc_bev = point_t_bev.reshape(cfg.num_height, cfg.num_ring, cfg.num_sector)
            #     # plt.imshow(pc_bev[0,...])
            #     # plt.show()

            #     pc_normalized = load_pc_infer(PC1[idxs_pose[0][0]])

            #     size = pc_normalized.shape[0]
            #     pc_normalized = pc_normalized[:,0:3]
            #     pc = pc_normalized.transpose().flatten().astype(np.float32)

            #     # generate bev
            #     transer_bev = voxelocc.GPUTransformer(pc, size, cfg.max_length, cfg.max_height, cfg.num_ring, cfg.num_sector, cfg.num_height, 1)
            #     transer_bev.transform()
            #     point_t_bev = transer_bev.retreive()
            #     point_t_bev = point_t_bev.reshape(-1, 3)
            #     point_t_bev = point_t_bev[...,2]
            #     pc_bev = point_t_bev.reshape(cfg.num_height, cfg.num_ring, cfg.num_sector)
                # plt.imshow(pc_bev[0,...])
                # plt.show()


        ###### compute the relative pose error ######
        # estimate the relative pose of the top 1 candidate using SC and SC++ methods
        if cfg.enable_sc:
            pred_angle_sc = yaw_diff_sc[idxs_sc_sorted[0]] # in grids
            pred_angle_sc = pred_angle_sc * 2 * np.pi / cfg.num_sector # in radians
            init_pose_sc = getSE3(0, 0, pred_angle_sc)

        if cfg.enable_cc:
            pred_lateral_cc = lateral_diff_cc[idxs_cc_sorted[0]] # in grids
            pred_lateral_cc = pred_lateral_cc * 140. / cfg.num_sector # in meters
            pred_y_lidar_cc = pred_lateral_cc
            init_pose_cc = getSE3(0, pred_y_lidar_cc, 0)

        if cfg.enable_pc:
            pred_angle_pc = yaw_diff_pc[idxs_pc_sorted[0]] # in grids
            pred_angle_pc = pred_angle_pc * 2 * np.pi / cfg.num_sector # in radians
            init_pose_pc = getSE3(0, 0, pred_angle_pc)

        # estimate the relative pose of the top 1 candidate using RING method
        '''
        Note: 
        There may exist 180-degree difference between the ground truth and the estimated yaw angle due to the semi-symmetry property of the RING descriptor.
        To eliminate the effect, we use error of solving the overdetermined system of equations in the process of solving the relative translation to determine the final relative yaw angle.
        Theroretically, the error of rightly estimated yaw angle will be smaller than that of wrong yaw angle esimation.
        '''
        # angle between the two matched RINGs in grids
        if cfg.enable_rs:
            angle_matched = RING_angles[idx_top1_rs] 
            angle_matched_extra = angle_matched - cfg.num_ring//2
            # convert the matched angle from grids to radians
            angle_matched_rad = angle_matched * 2 * np.pi / cfg.num_ring 
            angle_matched_extra_rad = angle_matched_extra * 2 * np.pi / cfg.num_ring    
            # row shift between the two matched RINGs to compensate the rotation
            row_shift = calculate_row_shift(angle_matched)
            row_shift_extra = calculate_row_shift(angle_matched_extra)
            # matched point cloud and RING
            pc_matched = PC_skip_recent[idx_top1_rs]
            RING_matched = RING_skip_recent[idx_top1_rs]       

            # compensated matched RINGs
            RING_matched_shifted = torch.roll(RING_matched, row_shift, dims=1)
            RING_matched_shifted_extra = torch.roll(RING_matched, row_shift_extra, dims=1)
        
            # solve the translation between the two matched RINGs, x: right, y: forward, z: upward (in the RING coordinate)
            x, y, error = solve_translation(RING_query, RING_matched_shifted, angle_matched_rad, device)
            x_extra, y_extra, error_extra = solve_translation(RING_query, RING_matched_shifted_extra, angle_matched_extra_rad, device)
            if error < error_extra:
                trans_x = x / cfg.num_sector * 140.  # in meters
                trans_y = y / cfg.num_ring * 140.  # in meters
                rot_yaw = angle_matched_rad  # in radians
            else:
                trans_x = x_extra / cfg.num_sector * 140.  # in meters
                trans_y = y_extra / cfg.num_ring * 140.  # in meters 
                rot_yaw = angle_matched_extra_rad  # in radians
            
            # convert the estimated pose to lidar coordinate
            pred_x_lidar_rs = -trans_y
            pred_y_lidar_rs = trans_x
            pred_angle_rs = rot_yaw

            # get the initial pose guess for icp
            init_pose_rs = getSE3(pred_x_lidar_rs, pred_y_lidar_rs, pred_angle_rs)

        # get the top1 matched map scan of different methods
        # apply icp for transformation solving and geometry checking
        # transformation matrix calculated by ICP is from query (source) pointcloud to map (target) pointcloud    
        print("Apply point-to-point ICP")
        if cfg.enable_m2dp:
            pc_matched_m2dp = PC_skip_recent[idx_top1_m2dp]
            fitness_m2dp, T_matrix_m2dp = fast_gicp(pc_query, pc_matched_m2dp, max_correspondence_distance=cfg.icp_max_distance)
            if dist_m2dp < cfg.revisit_criteria and fitness_m2dp < cfg.icp_fitness_score:
                print("M2DP: ICP fitness score is less than threshold, accept the loop.")    
                num_icp_correct_m2dp += 1    
                icp_x, icp_y, icp_yaw = se3_to_3dposes(T_matrix_m2dp)
                # print("ICP translation: x: {}, y: {}, yaw: {}".format(icp_x, icp_y, icp_yaw))
            else:
                print("M2DP: ICP fitness score is larger than threshold, reject the loop.")

        if cfg.enable_fh:
            pc_matched_fhist = PC_skip_recent[idx_top1_fhist]
            fitness_fhist, T_matrix_fhist = fast_gicp(pc_query, pc_matched_fhist, max_correspondence_distance=cfg.icp_max_distance)
            if dist_fhist < cfg.revisit_criteria and fitness_fhist < cfg.icp_fitness_score:
                print("FastHistogram: ICP fitness score is less than threshold, accept the loop.")    
                num_icp_correct_fhist += 1    
                icp_x, icp_y, icp_yaw = se3_to_3dposes(T_matrix_fhist)
                # print("ICP translation: x: {}, y: {}, yaw: {}".format(icp_x, icp_y, icp_yaw))
            else:
                print("FastHistogram: ICP fitness score is larger than threshold, reject the loop.")

        if cfg.enable_sc:
            pc_matched_sc = PC_skip_recent[idx_top1_sc]
            fitness_sc, T_matrix_sc = fast_gicp(pc_query, pc_matched_sc, max_correspondence_distance=cfg.icp_max_distance, init_pose=init_pose_sc)
            if dist_sc < cfg.revisit_criteria and fitness_sc < cfg.icp_fitness_score:
                print("ScanContext: ICP fitness score is less than threshold, accept the loop.")    
                num_icp_correct_sc += 1    
                icp_x, icp_y, icp_yaw = se3_to_3dposes(T_matrix_sc)
                yaw_err = np.min([np.abs(pred_angle_sc - gt_yaw_sc), np.abs(np.abs(pred_angle_sc - gt_yaw_sc)-2*np.pi)]) * 180. / np.pi
                yaw_error_sc.append(yaw_err)
                # print("ICP translation: x: {}, y: {}, yaw: {}".format(icp_x, icp_y, icp_yaw))
            else:
                print("ScanContext: ICP fitness score is larger than threshold, reject the loop.")

        if cfg.enable_cc:
            pc_matched_cc = PC_skip_recent[idx_top1_cc]
            fitness_cc, T_matrix_cc = fast_gicp(pc_query, pc_matched_cc, max_correspondence_distance=cfg.icp_max_distance, init_pose=init_pose_cc)
            if dist_cc < cfg.revisit_criteria and fitness_cc < cfg.icp_fitness_score:
                print("ScanContext++ (CC): ICP fitness score is less than threshold, accept the loop.")    
                num_icp_correct_cc += 1    
                icp_x, icp_y, icp_yaw = se3_to_3dposes(T_matrix_cc)
                y_err = np.abs(pred_y_lidar_cc - gt_y_cc)
                y_lidar_error_cc.append(y_err)
                # print("ICP translation: x: {}, y: {}, yaw: {}".format(icp_x, icp_y, icp_yaw))
            else:
                print("ScanContext++ (CC): ICP fitness score is larger than threshold, reject the loop.")

        if cfg.enable_pc:
            pc_matched_pc = PC_skip_recent[idx_top1_pc]
            fitness_pc, T_matrix_pc = fast_gicp(pc_query, pc_matched_pc, max_correspondence_distance=cfg.icp_max_distance, init_pose=init_pose_pc)
            if dist_pc < cfg.revisit_criteria and fitness_pc < cfg.icp_fitness_score:
                print("ScanContext++ (PC): ICP fitness score is less than threshold, accept the loop.")    
                num_icp_correct_pc += 1    
                icp_x, icp_y, icp_yaw = se3_to_3dposes(T_matrix_pc)
                yaw_err = np.min([np.abs(pred_angle_pc - gt_yaw_pc), np.abs(np.abs(pred_angle_pc - gt_yaw_pc)-2*np.pi)]) * 180. / np.pi
                yaw_error_pc.append(yaw_err)
                # print("ICP translation: x: {}, y: {}, yaw: {}".format(icp_x, icp_y, icp_yaw))
            else:
                print("ScanContext++ (PC): ICP fitness score is larger than threshold, reject the loop.")

        if cfg.enable_rs:
            pc_matched_rs = PC_skip_recent[idx_top1_rs]
            fitness_rs, T_matrix_rs = fast_gicp(pc_query, pc_matched_rs, max_correspondence_distance=cfg.icp_max_distance, init_pose=init_pose_rs)
            if dist_rs < cfg.revisit_criteria and fitness_rs < cfg.icp_fitness_score:
                print("RadonSinogram: ICP fitness score is less than threshold, accept the loop.")    
                num_icp_correct_rs += 1    
                icp_x, icp_y, icp_yaw = se3_to_3dposes(T_matrix_rs)
                x_err = np.abs(pred_x_lidar_rs - gt_x_rs)
                y_err = np.abs(pred_y_lidar_rs - gt_y_rs)
                yaw_err = np.min([np.abs(pred_angle_rs - gt_yaw_rs), np.abs(np.abs(pred_angle_rs - gt_yaw_rs)-2*np.pi)]) * 180. / np.pi
                x_lidar_error_rs.append(x_err)
                y_lidar_error_rs.append(y_err)
                yaw_error_rs.append(yaw_err)
                # print("ICP translation: x: {}, y: {}, yaw: {}".format(icp_x, icp_y, icp_yaw))
            else:
                print("RadonSinogram: ICP fitness score is larger than threshold, reject the loop.")
    
        # print("Fitness Scores: ", fitness_m2dp, fitness_fhist, fitness_sc, fitness_cc, fitness_pc, fitness_rs)

    PC_existed.append(pc_query)
    if cfg.enable_m2dp:    
        M2DP_existed.append(M2DP_query)
    if cfg.enable_fh:
        FH_existed.append(FH_query)
    if cfg.enable_sc:
        SC_existed.append(SC_query)
    if cfg.enable_cc:
        BEV_existed.append(BEV_query)
        RingkeyCC_existed.append(RingkeyCC_query)
    if cfg.enable_pc:
        RingkeyPC_existed.append(RingkeyPC_query)
    if cfg.enable_rs:    
        RING_existed.append(RING_query)
        TIRING_existed.append(TIRING_query)
        KDRING_existed.append(KDRING_query)
    Pose_existed.append(Pose_query)
    XYPose_existed.append(XYPose_query)

# calculate the recall@1 and average pose error
print("Number of total loops:", num_gt_loops)
result_path = cfg.result_path
if cfg.enable_m2dp:
    recall1_m2dp = num_correct_m2dp / num_gt_loops
    recall1_icp_m2dp = num_icp_correct_m2dp / num_gt_loops
    print("Number of correct loops (M2DP):", num_correct_m2dp)
    print("Recall@1 of all methods (M2DP):", recall1_m2dp)
    print("ICP Recall@1 of all methods (M2DP):", recall1_icp_m2dp)

if cfg.enable_fh:
    recall1_fhist = num_correct_fhist / num_gt_loops
    recall1_icp_fhist = num_icp_correct_fhist / num_gt_loops
    print("Number of correct loops (FH):", num_correct_fhist)
    print("Recall@1 of all methods (FH):", recall1_fhist)
    print("ICP Recall@1 of all methods (FH):", recall1_icp_fhist)

if cfg.enable_sc:
    yaw_error_sc = np.array(yaw_error_sc)
    recall1_sc = num_correct_sc / num_gt_loops
    recall1_icp_sc = num_icp_correct_sc / num_gt_loops
    yaw_err_mean_sc = np.mean(yaw_error_sc)
    yaw_err_90_percentile_sc = np.percentile(yaw_error_sc, 90)
    yaw_err_99_percentile_sc = np.percentile(yaw_error_sc, 99)
    print("Number of correct loops (SC):", num_correct_sc)
    print("Recall@1 of all methods (SC):", recall1_sc)
    print("ICP Recall@1 of all methods (SC):", recall1_icp_sc)
    print("Average pose error of SC method: yaw: {}".format(yaw_err_mean_sc))
    print("90 percentile pose error of SC method: yaw: {}".format(yaw_err_90_percentile_sc))
    print("99 percentile pose error of SC method: yaw: {}".format(yaw_err_99_percentile_sc))

if cfg.enable_cc:
    y_lidar_error_cc = np.array(y_lidar_error_cc)
    recall1_cc = num_correct_cc / num_gt_loops
    recall1_icp_cc = num_icp_correct_cc / num_gt_loops
    y_err_mean_cc = np.mean(y_lidar_error_cc)
    y_err_90_percentile_cc = np.percentile(y_lidar_error_cc, 90)
    y_err_99_percentile_cc = np.percentile(y_lidar_error_cc, 99)
    print("Number of correct loops (CC):", num_correct_cc)
    print("Recall@1 of all methods (CC):", recall1_cc)
    print("ICP Recall@1 of all methods (CC):", recall1_icp_cc)
    print("Average pose error of CC method: y: {}".format(y_err_mean_cc))
    print("90 percentile pose error of CC method: y: {}".format(y_err_90_percentile_cc))
    print("99 percentile pose error of CC method: y: {}".format(y_err_99_percentile_cc))

if cfg.enable_pc:
    yaw_error_pc = np.array(yaw_error_pc)
    recall1_pc = num_correct_pc / num_gt_loops
    recall1_icp_pc = num_icp_correct_pc / num_gt_loops
    yaw_err_mean_pc = np.mean(yaw_error_pc)
    yaw_err_90_percentile_pc = np.percentile(yaw_error_pc, 90)
    yaw_err_99_percentile_pc = np.percentile(yaw_error_pc, 99)
    print("Number of correct loops (PC):", num_correct_pc)
    print("Recall@1 of all methods (PC):", recall1_pc)
    print("ICP Recall@1 of all methods (PC):", recall1_icp_pc)
    print("Average pose error of PC method: yaw: {}".format(yaw_err_mean_pc))
    print("90 percentile pose error of PC method: yaw: {}".format(yaw_err_90_percentile_pc))
    print("99 percentile pose error of PC method: yaw: {}".format(yaw_err_99_percentile_pc))

if cfg.enable_rs:
    x_lidar_error_rs = np.array(x_lidar_error_rs)
    y_lidar_error_rs = np.array(y_lidar_error_rs)
    yaw_error_rs = np.array(yaw_error_rs)
    recall1_rs = num_correct_rs / num_gt_loops
    recall1_kdrs = num_correct_kdrs / num_gt_loops
    recall1_icp_rs = num_icp_correct_rs / num_gt_loops
    x_err_mean_rs = np.mean(x_lidar_error_rs)
    y_err_mean_rs = np.mean(y_lidar_error_rs)
    yaw_err_mean_rs = np.mean(yaw_error_rs)
    x_err_90_percentile_rs = np.percentile(x_lidar_error_rs, 90)
    y_err_90_percentile_rs = np.percentile(y_lidar_error_rs, 90)
    yaw_err_90_percentile_rs = np.percentile(yaw_error_rs, 90)
    x_err_99_percentile_rs = np.percentile(x_lidar_error_rs, 99)
    y_err_99_percentile_rs = np.percentile(y_lidar_error_rs, 99)
    yaw_err_99_percentile_rs = np.percentile(yaw_error_rs, 99)
    np.save(result_path + 'x_'+ str(cfg.num_height) +'.npy',x_lidar_error_rs)
    np.save(result_path + 'y_'+ str(cfg.num_height) +'.npy',y_lidar_error_rs)
    np.save(result_path + 'yaw_'+ str(cfg.num_height) +'.npy',yaw_error_rs)
    print("Number of correct loops (RS):", num_correct_rs)
    print("Recall@1 of all methods (RS):", recall1_rs)
    print("Recall@1 of all methods (KDRS):", recall1_kdrs)
    print("ICP Recall@1 of all methods (RS):", recall1_icp_rs)
    print("Average pose error of RING method: x: {}, y: {}, yaw: {}".format(x_err_mean_rs, y_err_mean_rs, yaw_err_mean_rs))
    print("90 percentile pose error of RING method: x: {}, y: {}, yaw: {}".format(x_err_90_percentile_rs, y_err_90_percentile_rs, yaw_err_90_percentile_rs))
    print("99 percentile pose error of RING method: x: {}, y: {}, yaw: {}".format(x_err_99_percentile_rs, y_err_99_percentile_rs, yaw_err_99_percentile_rs))

if cfg.enable_sc:
    # SC and SC++
    # draw boxplot for yaw error
    fig, ax = plt.subplots()
    ax.set_yscale('log')
    ax.boxplot(yaw_error_sc)
    ax.set_title('SC: Boxplot of yaw error')
    plt.show()
    plt.close()

if cfg.enable_cc:
    # draw boxplot for y error
    fig, ax = plt.subplots()
    ax.set_yscale('log')
    ax.boxplot(y_lidar_error_cc)
    ax.set_title('SC++ (CC): Boxplot of y error')
    plt.show()
    plt.close()

if cfg.enable_pc:
    # draw boxplot for yaw error
    fig, ax = plt.subplots()
    ax.set_yscale('log')
    ax.boxplot(yaw_error_pc)
    ax.set_title('SC++ (PC): Boxplot of yaw error')
    plt.show()
    plt.close()

if cfg.enable_rs:
    # RING
    # draw boxplot for x error
    fig, ax = plt.subplots()
    ax.set_yscale('log')
    ax.boxplot(x_lidar_error_rs)
    ax.set_title('RING: Boxplot of x error')
    plt.show()
    plt.close()

    # draw boxplot for y error
    fig, ax = plt.subplots()
    ax.set_yscale('log')
    ax.boxplot(y_lidar_error_rs)
    ax.set_title('RING: Boxplot of y error')
    plt.show()
    plt.close()

    # draw boxplot for yaw error
    fig, ax = plt.subplots()
    ax.set_yscale('log')
    ax.boxplot(yaw_error_rs)
    ax.set_title('RING: Boxplot of yaw error')
    plt.show()
    plt.close()