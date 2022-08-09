import os
import pickle
import random
import time
import numpy as np
import pandas as pd
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from icp import *
import math

BASE_DIR = "/media/mav-lab/1T/Datasets/pointnetvlad_dataset/benchmark_datasets/"
base_path = "/media/mav-lab/1T/Datasets/pointnetvlad_dataset/benchmark_datasets/"

runs_folder = "oxford_test/"
filename = "pointcloud_locations_20m_10overlap.csv"
gps_filename = "ins.csv"

pointcloud_fols = "/pointcloud_20m_10overlap/"

all_folders = sorted(os.listdir(os.path.join(base_path,runs_folder)))

folders = []

# All runs are used for training (both full and partial)
index_list = range(len(all_folders)-1)
print("Number of runs: "+str(len(index_list)))
for index in index_list:
    folders.append(all_folders[index])
print(folders)

#####For training and test data split#####
x_width = 150
y_width = 150
p1 = [5735712.768124,620084.402381]
p2 = [5735611.299219,620540.270327]
p3 = [5735237.358209,620543.094379]
p4 = [5734749.303802,619932.693364]
p = [p1,p2,p3,p4]

icp_num_tests = 15

def isRotationMatrix(R) :

    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)

    return n < 1e-6

def rotationMatrixToEulerAngles(R) :
    # assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])


def load_pc_file(filename):
    # returns Nx3 matrix
    pc = np.fromfile(os.path.join("/media/mav-lab/1T/Datasets/pointnetvlad_dataset/benchmark_datasets/", filename), dtype=np.float64)

    if(pc.shape[0] != 4096*3):
        print("Error in pointcloud shape")
        return np.array([])

    pc = np.reshape(pc,(pc.shape[0]//3, 3))
    return pc


def load_pc_files(filenames):
    pcs = []
    for filename in filenames:
        # print(filename)
        pc = load_pc_file(filename)
        if(pc.shape[0] != 4096):
            continue
        pcs.append(pc)
    pcs = np.array(pcs)
    return pcs


def check_in_test_set(northing, easting, points, x_width, y_width):
    in_test_set = False
    for point in points:
        if(point[0]-x_width < northing and northing < point[0]+x_width and point[1]-y_width < easting and easting < point[1]+y_width):
            in_test_set = True
            break
    return in_test_set
##########################################


def construct_query_dict(df_centroids, filename):
    tree = KDTree(df_centroids[['northing','easting']])
    ind_nn = tree.query_radius(df_centroids[['northing','easting']],r=10)
    ind_r = tree.query_radius(df_centroids[['northing','easting']], r=50)
    queries = {}
    for i in range(len(ind_nn)):
        # if i != len(ind_nn)-1:
        #     q_heading = np.arctan2(df_centroids.iloc[i+1]["northing"]-df_centroids.iloc[i]["northing"],
        #                                             df_centroids.iloc[i+1]["easting"]-df_centroids.iloc[i]["easting"])
        # else:
        #     q_heading = np.arctan2(df_centroids.iloc[i]["northing"]-df_centroids.iloc[i-1]["northing"],
                                                    # df_centroids.iloc[i]["easting"]-df_centroids.iloc[i-1]["easting"])    
        query = df_centroids.iloc[i]["file"]
        # query_points = load_pc_file(query)
        query_yaw = df_centroids.iloc[i]["yaw"]
        heading = []
        positives = np.setdiff1d(ind_nn[i],[i]).tolist()
        # if len(positives) != 0:
        # for j in range(len(positives)):
        # # pos_pc = load_pc_file(df_centroids.iloc[positives[0]]["file"])
        #     pos_yaw = df_centroids.iloc[positives[j]]["yaw"]
        #     heading.append(query_yaw - pos_yaw)
            # for pos_file in positives:
            #     pos_point = load_pc_file(df_centroids.iloc[pos_file]["file"])
            #     pos_heading = np.arctan2(df_centroids.iloc[pos_file]["northing"]-df_centroids.iloc[pos_file-1]["northing"],
            #                                             df_centroids.iloc[pos_file]["easting"]-df_centroids.iloc[pos_file-1]["easting"])    

            #     total_time = 0
            #     pos_trans = np.ones((pos_point.shape[0], 4))
            #     q_trans = np.ones((query_points.shape[0], 4))
            #     C = np.ones((pos_point.shape[0], 4))

            #     initT = rotation_matrix(np.array([0.0, 0.0, 1.0]), pos_heading - q_heading)
                
            #     for i in range(icp_num_tests):
            #         start = time.time()
            #         q_trans[:,0:2] = np.copy(query_points[:,0:2])
            #         pos_trans[:,0:3] = np.dot(initT, pos_point.T).T
            #         pos_trans[:,3] = 0.

            #         T, distances, iterations = icp(pos_trans[:,0:2], q_trans[:,0:2], tolerance=0.0001)
            #         total_time += time.time() - start
                    
            #         C = np.copy(pos_point)
            #         # Transform C
            #         C = np.dot(T, C.T).T
                
            #     euler_angle = rotationMatrixToEulerAngles(T)
            #     yaw = q_heading - pos_heading + euler_angle[2]
            #     heading.append(yaw)
            #     print("yaw ", yaw)
            #     print('icp single time: {:.3}'.format(total_time/icp_num_tests))
            #     print('icp total time: {:.3}'.format(total_time))
            #     print("icp distances", distances.sum())
            
            # T = rotation_matrix(np.array([0.0, 0.0, 1.0]), query_yaw - pos_yaw)
            # pos_t = np.ones((pos_pc.shape[0], 4))
            # pos_t = np.copy(pos_pc)
            # pos_t = np.dot(T, pos_t.T).T
            # x = np.concatenate([query_points[...,0], pos_t[...,0]])
            # y = np.concatenate([query_points[...,1], pos_t[...,1]])
            # z = np.concatenate([query_points[...,2], pos_t[...,2]])
            # fig = plt.figure()
            # ax = Axes3D(fig)
            # ax.scatter(x, y, z)

            # x = query_points[...,0]
            # y = query_points[...,1]
            # z = query_points[...,2]
            # fig2 = plt.figure()
            # ax2 = Axes3D(fig2)
            # ax2.scatter(x, y, z)

            # x = pos_pc[...,0]
            # y = pos_pc[...,1]
            # z = pos_pc[...,2]
            # fig3 = plt.figure()
            # ax3 = Axes3D(fig3)
            # ax3.scatter(x, y, z)

            # x = pos_t[...,0]
            # y = pos_t[...,1]
            # z = pos_t[...,2]
            # fig4 = plt.figure()
            # ax4 = Axes3D(fig4)
            # ax4.scatter(x, y, z)
            # ax.set_xlabel("x")
            # ax.set_ylabel("y")
            # ax.set_zlabel("z")
            # ax2.set_xlabel("x")
            # ax2.set_ylabel("y")
            # ax2.set_zlabel("z")
            # ax3.set_xlabel("x")
            # ax3.set_ylabel("y")
            # ax3.set_zlabel("z")
            # ax4.set_xlabel("x")
            # ax4.set_ylabel("y")
            # ax4.set_zlabel("z")
            # plt.show()
            # plt.pause(0.1)
            # plt.close()
        # else:
        #     heading.append(0.0)
        negatives = np.setdiff1d(
            df_centroids.index.values.tolist(),ind_r[i]).tolist()
        random.shuffle(negatives)
        queries[i] = {"query":query, "heading":query_yaw,
                      "positives":positives,"negatives":negatives}
    # print(query)
    with open(filename, 'wb') as handle:
        pickle.dump(queries, handle, protocol=pickle.HIGHEST_PROTOCOL)

    print("Done ", filename)


# Initialize pandas DataFrame
df_train = pd.DataFrame(columns=['file','northing','easting','yaw'])
df_test = pd.DataFrame(columns=['file','northing','easting','yaw'])

for folder in folders:
    df_locations = pd.read_csv(os.path.join(
        base_path,runs_folder,folder,filename),sep=',')
    df_locations['timestamp'] = runs_folder+folder + \
        pointcloud_fols+df_locations['timestamp'].astype(str)+'.bin'
    df_locations = df_locations.rename(columns={'timestamp':'file'})

    for index, row in df_locations.iterrows():
        if(check_in_test_set(row['northing'], row['easting'], p, x_width, y_width)):
            df_test = df_test.append(row, ignore_index=True)
        else:
            df_train = df_train.append(row, ignore_index=True)

print("Number of training submaps: "+str(len(df_train['file'])))
print("Number of non-disjoint test submaps: "+str(len(df_test['file'])))
construct_query_dict(df_train,"./training_queries_baseline.pickle")
construct_query_dict(df_test,"./test_queries_baseline.pickle")
