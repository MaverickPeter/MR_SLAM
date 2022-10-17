import os
import time
import copy
import numpy as np
import open3d as o3d

# Load pcd point clouds
pcd_path1 = "/home/lusha/Datasets/Pseudo-GT/Keyframes/6989586621679009792.pcd"
pcd_path2 = "/home/lusha/Datasets/Pseudo-GT/Keyframes/6989586621679009793.pcd"

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

source = o3d.io.read_point_cloud(pcd_path1)
target = o3d.io.read_point_cloud(pcd_path2)

# apply outlier removal
source.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
target.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)

threshold = 0.2
trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                         [-0.139, 0.967, -0.215, 0.7],
                         [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
# trans_init = np.eye(4)
# draw_registration_result(source, target, trans_init)

treg = o3d.pipelines.registration
# Select the `Estimation Method`, and `Robust Kernel` (for outlier-rejection).
estimation = treg.TransformationEstimationPointToPoint()
# Search distance for Nearest Neighbour Search [Hybrid-Search is used].
max_correspondence_distance = 1.0

# Initial alignment or source to target transform.
init_source_to_target = trans_init

# Convergence-Criteria for Vanilla ICP
criteria = treg.ICPConvergenceCriteria(relative_fitness=0.0000001,
                                       relative_rmse=0.0000001,
                                       max_iteration=30)

# Down-sampling voxel-size. If voxel_size < 0, original scale is used.
voxel_size = -1

# Save iteration wise `fitness`, `inlier_rmse`, etc. to analyse and tune result.
save_loss_log = True
print("Apply Point-to-Point ICP")
s = time.time()

reg_point_to_point = treg.registration_icp(source, target, max_correspondence_distance,
                              init_source_to_target, estimation, criteria)

icp_time = time.time() - s
print("Time taken by Point-To-Point ICP: ", icp_time)
print("Fitness: ", reg_point_to_point.fitness)
print("Inlier RMSE: ", reg_point_to_point.inlier_rmse)

draw_registration_result(source, target, reg_point_to_point.transformation)


# Select the `Estimation Method`, and `Robust Kernel` (for outlier-rejection).
estimation = treg.TransformationEstimationPointToPlane()
# Search distance for Nearest Neighbour Search [Hybrid-Search is used].
max_correspondence_distance = 1.0
# Estimate normals to the source and target point cloud
source.estimate_normals(
    o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
target.estimate_normals(
    o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
estimate_normals = True

print("Apply Point-to-Plane ICP")
s = time.time()

reg_point_to_plane = treg.registration_icp(source, target, max_correspondence_distance,
                              init_source_to_target, estimation, criteria)

icp_time = time.time() - s
print("Time taken by Point-To-Plane ICP: ", icp_time)
print("Fitness: ", reg_point_to_plane.fitness)
print("Inlier RMSE: ", reg_point_to_plane.inlier_rmse)

draw_registration_result(source, target, reg_point_to_plane.transformation)

