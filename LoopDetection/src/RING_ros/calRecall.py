import os
import sys
from genpy import Duration
import pygicp
from util import *
import config as cfg
import numpy as np
import open3d as o3d
import seaborn as sns
import matplotlib.pyplot as plt
from tf.transformations import translation_matrix, quaternion_matrix
from sklearn.neighbors import KDTree

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


# apply icp using fast_gicp (https://github.com/SMRT-AIST/fast_gicp)
def fast_gicp(source, target, max_correspondence_distance=1.0, init_pose=None):
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

# main loop
PC2 = []
PC3 = []
RING2 = []
RING3 = []
TIRING2 = []
TIRING3 = []
KDRING2 = []
KDRING3 = []
Pose2 = []
Pose3 = []
Durations = []

# pcd files names
filenames = np.array([int(i) for i in gt_vertex[:,0]])

for pcd_file in pcd_files:
    filename = int(pcd_file.split(".")[0]) + 1 # add 1 to the file name
    # print("Processing file: ", filename)
    idx = find_closest_timestamp(filenames, filename)
    # print("Closest timestamp: ", filenames[idx])
    trans = translation_matrix([float(gt_vertex[idx, 1]), float(gt_vertex[idx, 2]), float(gt_vertex[idx, 3])])
    quat = quaternion_matrix([float(gt_vertex[idx, 4]), float(gt_vertex[idx, 5]), float(gt_vertex[idx, 6]), float(gt_vertex[idx, 7])])
    se3 = np.dot(trans, quat)      
    pcd_file_path = pcd_path + pcd_file + '/' + pcd_filename
    # load the pcd file
    pcd, pc = read_pcd(pcd_file_path)
    # normalize the point cloud to -1~1
    pc_normalized = load_pc_infer(pc)
    # generate the RING and TIRING descriptors
    times = time.time()    
    pc_bev, pc_RING, pc_TIRING, pc_kdRing = generate_RING(pc_normalized) 
    timee = time.time()
    print("generate_RING time:", timee - times, 's')
  
    if pcd_file[:2] == "69":
        robotid = 2
        PC2.append(pc)
        RING2.append(pc_RING)
        TIRING2.append(pc_TIRING)
        KDRING2.append(pc_kdRing.cpu().numpy())
        Pose2.append(se3)

    elif pcd_file[:2] == "70":
        robotid = 3
        PC3.append(pc)
        RING3.append(pc_RING)
        TIRING3.append(pc_TIRING)
        KDRING3.append(pc_kdRing.cpu().numpy())
        Pose3.append(se3)

# Use the data collected by robot 2 as map, and the data collected by robot 3 as query
num_total = len(PC3)
num_correct = 0
num_correct_kdtree = 0
num_icp_correct = 0
yaw_error = []
x_lidar_error = []
y_lidar_error = []

for idx_query in range(num_total):
    timestart = time.time()
    pc_query = PC3[idx_query]
    RING_query = RING3[idx_query]
    TIRING_query = TIRING3[idx_query]
    Pose_query = Pose3[idx_query]
    # print("Query pose x, y, yaw:", se3_to_3dposes(Pose_query))
    
    query_kd = KDRING3[idx_query]
    tree = KDTree(KDRING2)  
    dist, ind = tree.query(query_kd.reshape(1,-1), k=1)            

    RING_idxs = []
    RING_dists = []
    RING_angles = []    
    for idx_map in range(len(PC2)):
        pc_map = PC2[idx_map]
        RING_map = RING2[idx_map]
        TIRING_map = TIRING2[idx_map]
        Pose_map = Pose2[idx_map]
        # print("Map pose x, y, yaw:", se3_to_3dposes(Pose_map)) # ground truth transformation from query frame to map frame
        gt_T = np.dot(np.linalg.inv(Pose_query), Pose_map) # ground truth transformation from query point cloud to map point cloud
        gt_x, gt_y, gt_yaw = se3_to_3dposes(gt_T)
        gt_yaw_grid = GT_angle_convert(gt_yaw * 180 / np.pi, cfg.num_ring)
        # calculate the distance between the two poses
        gt_d = np.sqrt(gt_x**2 + gt_y**2)
        

        # !!!!!!!! Note !!!!!!!!
        '''
        Note: 
        The relative pose estimated by RING method is from the query point cloud to the map point cloud.
        So it needs to be inverted to get the transformation from query frame to map frame.
        '''
        dist, angle = fast_corr(TIRING_query, TIRING_map)
        # print("Distance bewteen two RING descriptors: ", dist)
        # print("Shifted angle bewteen two RING descriptors: ", angle)
        if gt_d < cfg.revisit_criteria:
            if idx_map == ind[0]:
                num_correct_kdtree += 1
            print("The distance between the two poses is: ", gt_d, "m.")
            # print("Relative pose x, y, yaw:", gt_x, gt_y, gt_yaw, gt_yaw_grid)
            # print("Distance bewteen two RING descriptors: ", dist)
            # print("Shifted angle bewteen two RING descriptors: ", angle)

        # print("dist: ",dist)
        if dist < cfg.dist_threshold:
            RING_idxs.append(idx_map)
            RING_dists.append(dist)
            RING_angles.append(angle)

    if len(RING_dists) == 0:
        print("No loop detected.")

    else:
        dists_sorted = np.sort(RING_dists)
        idxs_sorted = np.argsort(RING_dists)        
        idx_top1 = idxs_sorted[0]
        dist_top1 = RING_dists[idx_top1]
        print("Top 1 RING distance: ", dist_top1)        
        
        # angle between the two matched RINGs in grids
        angle_matched = RING_angles[idx_top1] 
        angle_matched_extra = angle_matched - cfg.num_ring//2
        print("angle_matched_extra: ", angle_matched_extra)
        print("angle matched: ", angle_matched)

        # convert the matched angle from grids to radians
        angle_matched_rad = angle_matched * 2 * np.pi / cfg.num_ring 
        angle_matched_extra_rad = angle_matched_extra * 2 * np.pi / cfg.num_ring    

        # row shift between the two matched RINGs to compensate the rotation
        row_shift = calculate_row_shift(angle_matched)
        row_shift_extra = calculate_row_shift(angle_matched_extra)

        # matched timestamp, pc and RING
        idx_matched = RING_idxs[idx_top1]
        pc_matched = PC2[idx_matched]
        RING_matched = RING2[idx_matched]                 
        Pose_matched = Pose2[idx_matched]

        # ground truth translation and rotation
        # gt_T = np.dot(np.linalg.inv(Pose_query), Pose_matched)
        gt_T = np.dot(np.linalg.inv(Pose_matched), Pose_query)
        gt_x, gt_y, gt_yaw = se3_to_3dposes(gt_T)                                   
        gt_d = np.sqrt(gt_x**2 + gt_y**2)   

        # calculate recall@1 
        if gt_d < cfg.revisit_criteria:
            num_correct += 1

        # compensated matched RINGs
        RING_matched_shifted = torch.roll(RING_matched, row_shift, dims=1)
        RING_matched_shifted_extra = torch.roll(RING_matched, row_shift_extra, dims=1)

        # solve the translation between the two matched RINGs, x: right, y: forward, z: upward (in the RING coordinate)
        # x, y, error = solve_translation(RING_query[0,...], RING_matched_shifted[0,...], angle_matched_rad, device)
        # x_extra, y_extra, error_extra = solve_translation(RING_query[0,...], RING_matched_shifted_extra[0,...], angle_matched_extra_rad, device)
        timess = time.time()
        x, y, error = solve_translation(torch.sum(RING_query,dim=0), torch.sum(RING_matched_shifted,dim=0), angle_matched_rad, device)
        x_extra, y_extra, error_extra = solve_translation(torch.sum(RING_query,dim=0), torch.sum(RING_matched_shifted_extra,dim=0), angle_matched_extra_rad, device)
        # x, y, error = solve_multilayer_translation(RING_query, RING_matched_shifted, angle_matched_rad, device)
        # x_extra, y_extra, error_extra = solve_multilayer_translation(RING_query, RING_matched_shifted_extra, angle_matched_extra_rad, device)
        timeee = time.time()
        print("solve_translation:", timeee - timess, 's')
        print("x, y, error:", x, y, error)
        print("x_extra, y_extra, error_extra:", x_extra, y_extra, error_extra)
        if error < error_extra:
            trans_x = x / cfg.num_sector * 140.  # in meters
            trans_y = y / cfg.num_ring * 140.  # in meters
            rot_yaw = angle_matched_rad  # in radians
        else:
            trans_x = x_extra / cfg.num_sector * 140.  # in meters
            trans_y = y_extra / cfg.num_ring * 140.  # in meters 
            rot_yaw = angle_matched_extra_rad  # in radians

        # convert to the BEV coordinate (x: downward, y: right, z: upward)
        trans_x_bev = -trans_y
        trans_y_bev = trans_x
        
        # ! Two possible relationships bewteen RING coordinate and Lidar corrdinate
        # ! Relationship 1 (has bigger translation error in this small test dataset)
        # convert to the lidar coordinate (x: forward, y: left, z: upward)
        trans_x_lidar = trans_y
        trans_y_lidar = -trans_x
        
        # ! Relationship 2 (has smaller translation error in this small test dataset)
        # convert to the lidar coordinate
        # trans_x_lidar = -trans_y
        # trans_y_lidar = trans_x
    
        print("Loop detected.")
        print("Ground truth translation: x: {}, y: {}, yaw: {}".format(gt_x, gt_y, gt_yaw))
        print("Estimated translation: x: {}, y: {}, yaw: {}".format(trans_x_lidar, trans_y_lidar, rot_yaw))

        # apply ICP to the matched point clouds 
        # transformation matrix calculated by ICP is from query (source) pointcloud to map (target) pointcloud
        init_pose = getSE3(trans_x_lidar, trans_y_lidar, rot_yaw)
        print("Apply point-to-point ICP")
        times = time.time()
        # fitness, T_matrix = o3d_icp(pc_query, pc_matched, max_correspondence_distance=1.0, init_pose=np.linalg.inv(init_pose))
        fitness, T_matrix = fast_gicp(pc_query, pc_matched, max_correspondence_distance=cfg.icp_max_distance, init_pose=init_pose)
        timee = time.time()
        
        # open3d icp visualization
        # print("Orignal point clouds")
        # draw_registration_result(pc_query, pc_matched, np.eye(4))        
        # print("Registrated point clouds with RING results")
        # draw_registration_result(pc_query, pc_matched, np.linalg.inv(init_pose))        
        # print("Registrated point clouds with ICP results")
        # draw_registration_result(pc_query, pc_matched, T_matrix) 
        
        # fast gicp visualization
        # print("Orignal point clouds")
        # draw_registration_result_fast_gicp(pc_query, pc_matched, np.eye(4))        
        # print("Registrated point clouds with RING results")
        # draw_registration_result_fast_gicp(pc_query, pc_matched, init_pose)        
        # print("Registrated point clouds with ICP results")
        # draw_registration_result_fast_gicp(pc_query, pc_matched, T_matrix)      

        # T_matrix = np.linalg.inv(T_matrix) # from query frame to map frame
        print("ICP fitness score:", fitness)              
        # print("ICP transformation matrix:", T_matrix)
        print("ICP processed time:", timee - times, 's')

        if fitness < cfg.icp_fitness_score:
            print("ICP fitness score is less than threshold, accept the loop.")    
            num_icp_correct += 1    
            icp_x, icp_y, icp_yaw = se3_to_3dposes(T_matrix)
            x_err = abs(trans_x_lidar - gt_x)
            y_err = abs(trans_y_lidar - gt_y)
            yaw_err = abs(rot_yaw - gt_yaw) * 180. / np.pi
            x_lidar_error.append(x_err)
            y_lidar_error.append(y_err)
            yaw_error.append(yaw_err)
            print("ICP translation: x: {}, y: {}, yaw: {}".format(icp_x, icp_y, icp_yaw))
        else:
            print("ICP fitness score is larger than threshold, reject the loop.")
    timeend = time.time()
    Durations.append(timeend - timestart)
    # print("one iter time:", timeend - timestart, 's')
  
x_lidar_error = np.array(x_lidar_error)
y_lidar_error = np.array(y_lidar_error)
yaw_error = np.array(yaw_error)
Durations = np.array(Durations)

recall1 = num_correct / num_total
recall1_icp = num_icp_correct / num_total
x_err_mean = np.mean(x_lidar_error)
y_err_mean = np.mean(y_lidar_error)
yaw_err_mean = np.median(yaw_error) 
duration_mean = np.mean(Durations)
print("Number of total loops:", num_total)
print("Number of correct loops:", num_correct)
print("Number of Kdtree correct loops:", num_correct_kdtree)
print("Mean Duration:", duration_mean)
print("Recall@1:", recall1)
print("ICP Recall@1:", recall1_icp)
print("Average pose error: x: {}, y: {}, yaw: {}".format(x_err_mean, y_err_mean, yaw_err_mean))

# draw boxplot for x error
fig, ax = plt.subplots()
ax.set_yscale('log')
ax.boxplot(x_lidar_error)
ax.set_title('Boxplot of x error')
plt.show()
plt.close()

# draw boxplot for y error
fig, ax = plt.subplots()
ax.set_yscale('log')
ax.boxplot(y_lidar_error)
ax.set_title('Boxplot of y error')
plt.show()
plt.close()

# draw boxplot for yaw error
fig, ax = plt.subplots()
ax.set_yscale('log')
ax.boxplot(yaw_error)
ax.set_title('Boxplot of yaw error')
plt.show()
plt.close()