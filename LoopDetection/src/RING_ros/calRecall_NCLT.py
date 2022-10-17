import os
import sys
import struct
import pygicp
import argparse
from util import *
import config as cfg
import numpy as np
import pandas as pd
import open3d as o3d
import seaborn as sns
from tqdm import tqdm
import matplotlib.pyplot as plt
from tf.transformations import translation_matrix, quaternion_matrix

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
    pcd = o3d.io.read_point_cloud(filename)
    # get the point cloud
    pc = np.asarray(pcd.points)
    return pcd, pc


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


# visualize the icp result
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, 0])
    o3d.visualization.draw_geometries([coordinate_frame, source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])


# nclt pointcloud utils
def convert(x_s, y_s, z_s):
    scaling = 0.005 # 5 mm
    offset = -100.0
    x = x_s * scaling + offset
    y = y_s * scaling + offset
    z = z_s * scaling + offset
    return x, y, z
    

# load lidar file in nclt dataset
def load_lidar_file_nclt(file_path):
    n_vec = 4
    hits = []
    with open(file_path,'rb') as f_bin:
        while True:
            x_str = f_bin.read(2)
            if x_str == b"": # eof
                break
            x = struct.unpack('<H', x_str)[0]
            y = struct.unpack('<H', f_bin.read(2))[0]
            z = struct.unpack('<H', f_bin.read(2))[0]
            i = struct.unpack('B', f_bin.read(1))[0]
            l = struct.unpack('B', f_bin.read(1))[0]
            x, y, z = convert(x, y, z)
            s = "%5.3f, %5.3f, %5.3f, %d, %d" % (x, y, z, i, l)
            hits += [[x, y, -z]]

        hits = np.asarray(hits)

    return hits


# load data of NCLT dataset
def load_nclt_data(data_path, gt_path, sampling_gap = cfg.sampling_gap):
    all_filenames = sorted(os.listdir(data_path))
    num_files = len(all_filenames)
    
    # main lists
    PCs = []
    RINGs = []
    TIRINGs = []
    xy_poses = []
    yaw_poses = []
    T_matrices = []

    # get groundtruth file and load it
    df_locations = pd.read_csv(gt_path, header=0, names = ['timestamp','northing','easting', 'height','roll','pitch','yaw'], low_memory=False)
    # convert data type
    df_locations['timestamp'] = df_locations['timestamp'].astype(int)
    first_flag = False

    # get the file name
    for idx in tqdm(range(num_files), desc='Loading'):
        filename = os.path.splitext(all_filenames[idx])[0]
        loc_idx = find_closest_timestamp(df_locations['timestamp'].values, int(filename))
        roll = df_locations['roll'].values[loc_idx]
        pitch = df_locations['pitch'].values[loc_idx]
        yaw = df_locations['yaw'].values[loc_idx]
        northing = df_locations['northing'].values[loc_idx]
        easting = df_locations['easting'].values[loc_idx]
        height = df_locations['height'].values[loc_idx]
        R = euler2rot(roll, pitch, yaw)
        T = trans2hom(R, [northing, easting, height])

        # for not nan value and very first ones (which often wrong)
        if np.isnan(float(northing)) or np.isnan(float(easting)):
            continue
        elif not first_flag :
            prev_northing, prev_easting = float(northing), float(easting)
            first_flag = True          

        if(sample_at_intervals(float(northing), float(easting), float(prev_northing), float(prev_easting), sampling_gap)):
            pc_path = os.path.join(data_path, filename + '.bin')
            pc = load_lidar_file_nclt(pc_path)
            pc_normalized = load_pc_infer(pc)
            if pc.shape[0] == 0:
                continue

            # generate RING and TIRING descriptors of the point cloud
            pc_bev, pc_RING, pc_TIRING, _ = generate_RING(pc_normalized)
            
            PCs.append(pc)
            RINGs.append(pc_RING)
            TIRINGs.append(pc_TIRING)
            xy_poses.append([northing, easting])
            yaw_poses.append(yaw)
            T_matrices.append(T)
            prev_northing, prev_easting = float(northing), float(easting)

    return PCs, RINGs, TIRINGs, xy_poses, yaw_poses, T_matrices


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--dataset", type=str, default="nclt", help="dataset name (nclt / mulran / oxford)")
    parser.add_argument("--map_folder", type=str, default="2012-02-04", help="map sequence for loop closure detection")
    parser.add_argument("--query_folder", type=str, default="2012-05-26", help="query sequence for loop closure detection")
    parser.add_argument("--map_sampling_gap", type=float, default=20, help="map sampling gap in meter")
    parser.add_argument("--query_sampling_gap", type=float, default=10, help="query sampling gap in meter")

    args = parser.parse_args()
    
    # args
    dataset = args.dataset # choose from "nclt", "mulran" and "oxford"
    map_folder = args.map_folder # choose one sequence folder from dataset as map
    query_folder = args.query_folder # choose one sequence folder from dataset as query
    map_sampling_gap = args.map_sampling_gap # map sampling gap in meter
    query_sampling_gap = args.query_sampling_gap # query sampling gap in meter
    revisit_criteria = map_sampling_gap / 2 # revisit criteria in meter

    # NCLT dataset path
    base_path = "/media/client/Mav/Datasets/NCLT/"
    pointcloud_fols = "/velodyne_data/velodyne_sync/"
    map_gt_filename = "/ground_truth/groundtruth_" + map_folder + ".csv"
    query_gt_filename = "/ground_truth/groundtruth_" + query_folder + ".csv"

    # load map and query data
    map_data_path = base_path + map_folder + pointcloud_fols
    map_gt_path = base_path + map_folder + map_gt_filename
    query_data_path = base_path + query_folder + pointcloud_fols
    query_gt_path = base_path + query_folder + query_gt_filename
    
    map_PCs, map_RINGs, map_TIRINGs, map_xy_poses, map_yaw_poses, map_T_matrices = load_nclt_data(map_data_path, map_gt_path, map_sampling_gap)
    query_PCs, query_RINGs, query_TIRINGs, query_xy_poses, query_yaw_poses, query_T_matrices = load_nclt_data(query_data_path, query_gt_path, query_sampling_gap)
    
    map_num_scans = len(map_RINGs)
    query_num_scans = len(query_RINGs)
    
    tree_xy_poses = KDTree(np.array(map_xy_poses))
    # Use the data collected by robot 2 as map, and the data collected by robot 3 as query
    num_correct = 0
    num_evaluated = 0
    yaw_error = []
    x_lidar_error = []
    y_lidar_error = []
    
    for query_idx in tqdm(range(query_num_scans), desc='Processing {}_{}_{}'.format(dataset.upper(), map_folder, query_folder)):
        # save to (online) DB
        query_PC = query_PCs[query_idx]
        query_RING = query_RINGs[query_idx]
        query_TIRING = query_TIRINGs[query_idx]
        query_xy_pose = query_xy_poses[query_idx]
        query_yaw_pose = query_yaw_poses[query_idx]
        query_T = query_T_matrices[query_idx]

        indices = tree_xy_poses.query_radius(np.array([query_xy_pose]), r=revisit_criteria)
        num_true_pos = len(indices[0])
        if num_true_pos > 0:
            num_evaluated += 1
            RING_idxs = []
            RING_dists = []
            RING_angles = []    
            
            for map_idx in range(map_num_scans):
                map_RING = map_RINGs[map_idx]
                map_TIRING = map_TIRINGs[map_idx]
                dist, angle = fast_corr(query_TIRING, map_TIRING)
                RING_idxs.append(map_idx)
                RING_dists.append(dist)
                RING_angles.append(angle)

            dists_sorted = np.sort(RING_dists)
            idxs_sorted = np.argsort(RING_dists)        
            idx_top1 = idxs_sorted[0]
            dist_top1 = RING_dists[idx_top1]    
            
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
            PC_matched = map_PCs[idx_matched]
            RING_matched = map_RINGs[idx_matched]                 
            T_matched = map_T_matrices[idx_matched]

            # ground truth translation and rotation
            gt_T = np.dot(np.linalg.inv(T_matched), query_T)
            gt_x, gt_y, gt_yaw = se3_to_3dposes(gt_T)                                   
            gt_d = np.sqrt(gt_x**2 + gt_y**2)   

            # calculate recall@1 
            if gt_d < revisit_criteria:
                num_correct += 1

                # compensated matched RINGs
                RING_matched_shifted = torch.roll(RING_matched, row_shift, dims=1)
                RING_matched_shifted_extra = torch.roll(RING_matched, row_shift_extra, dims=1)

                # solve the translation between the two matched RINGs, x: right, y: forward, z: upward (in the RING coordinate)
                x, y, error = solve_translation(query_RING, RING_matched_shifted, angle_matched_rad, device)
                x_extra, y_extra, error_extra = solve_translation(query_RING, RING_matched_shifted_extra, angle_matched_extra_rad, device)
                if error < error_extra:
                    trans_x = x / cfg.num_sector * 140.  # in meters
                    trans_y = y / cfg.num_ring * 140.  # in meters
                    rot_yaw = angle_matched_rad  # in radians
                else:
                    trans_x = x_extra / cfg.num_sector * 140.  # in meters
                    trans_y = y_extra / cfg.num_ring * 140.  # in meters 
                    rot_yaw = angle_matched_extra_rad  # in radians
                
                # convert to the BEV coordinate (x: downward, y: right, z: upward), which actually represents the lidar coordinate of point cloud
                trans_x_lidar = -trans_y
                trans_y_lidar = trans_x

                print("Loop detected.")
                print("Ground truth translation: x: {}, y: {}, yaw: {}".format(gt_x, gt_y, gt_yaw))
                print("Estimated translation: x: {}, y: {}, yaw: {}".format(trans_x_lidar, trans_y_lidar, rot_yaw))
                    
                x_err = np.abs(trans_x_lidar - gt_x)
                y_err = np.abs(trans_y_lidar - gt_y)
                yaw_err = np.min([np.abs(rot_yaw - gt_yaw), np.abs(np.abs(rot_yaw - gt_yaw)-2*np.pi)]) * 180. / np.pi
                x_lidar_error.append(x_err)
                y_lidar_error.append(y_err)
                yaw_error.append(yaw_err)

                # # apply ICP to the matched point clouds 
                # init_pose = getSE3(trans_x_lidar, trans_y_lidar, rot_yaw)
                # print("Apply point-to-point ICP")
                # times = time.time()
                # fitness, T_matrix = o3d_icp(query_PC, PC_matched, max_correspondence_distance=cfg.icp_max_distance, init_pose=np.linalg.inv(init_pose))
                # timee = time.time()

                # # print("Orignal point clouds")
                # # draw_registration_result(query_PC, PC_matched, np.eye(4))        
                # # print("Registrated point clouds with RING results")
                # # draw_registration_result(query_PC, PC_matched, np.linalg.inv(init_pose))        
                # # print("Registrated point clouds with ICP results")
                # # draw_registration_result(query_PC, PC_matched, T_matrix)   

                # T_matrix = np.linalg.inv(T_matrix)
                # print("ICP fitness score:", fitness)              
                # # print("ICP transformation matrix:", T_matrix)
                # print("ICP processed time:", timee - times, 's')

                # if fitness < cfg.icp_fitness_score:
                #     print("ICP fitness score is less than threshold, accept the loop.")        
                #     icp_x, icp_y, icp_yaw = se3_to_3dposes(T_matrix)
                #     x_err = abs(trans_x_lidar - icp_x)
                #     y_err = abs(trans_y_lidar - icp_y)
                #     yaw_err = abs(rot_yaw - icp_yaw) * 180. / np.pi
                #     x_lidar_error.append(x_err)
                #     y_lidar_error.append(y_err)
                #     yaw_error.append(yaw_err)
                #     print("ICP translation: x: {}, y: {}, yaw: {}".format(icp_x, icp_y, icp_yaw))
                # else:
                #     print("ICP fitness score is larger than threshold, reject the loop.")

    
x_lidar_error = np.array(x_lidar_error)
y_lidar_error = np.array(y_lidar_error)
yaw_error = np.array(yaw_error)

recall1 = num_correct / num_evaluated
x_err_mean = np.mean(x_lidar_error)
y_err_mean = np.mean(y_lidar_error)
yaw_err_mean = np.mean(yaw_error) 
np.save(cfg.result_path + 'x_'+ str(cfg.num_height) +'.npy',x_lidar_error)
np.save(cfg.result_path + 'y_'+ str(cfg.num_height) +'.npy',y_lidar_error)
np.save(cfg.result_path + 'yaw_'+ str(cfg.num_height) +'.npy',yaw_error)
print("Number of total loops:", num_evaluated)
print("Number of correct loops:", num_correct)
print("Recall@1:", recall1)
print("Average pose error: x: {}, y: {}, yaw: {}".format(x_err_mean, y_err_mean, yaw_err_mean))

# draw boxplot for x error
fig, ax = plt.subplots()
ax.set_yscale('log')
ax.boxplot(x_lidar_error)
ax.set_title(map_folder+' '+query_folder+': boxplot of x error')
plt.show()
plt.close()

# draw boxplot for y error
fig, ax = plt.subplots()
ax.set_yscale('log')
ax.boxplot(y_lidar_error)
ax.set_title(map_folder+' '+query_folder+': boxplot of y error')
plt.show()
plt.close()

# draw boxplot for yaw error
fig, ax = plt.subplots()
ax.set_yscale('log')
ax.boxplot(yaw_error)
ax.set_title(map_folder+' '+query_folder+': boxplot of yaw error')
plt.show()
plt.close()