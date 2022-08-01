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
gps_filename = "gps/ins.csv"

pointcloud_fols = "/pointcloud_20m_10overlap/"

all_folders = sorted(os.listdir(os.path.join(base_path,runs_folder)))

folders = []

# All runs are used for training (both full and partial)
index_list = range(len(all_folders)-1)
print("Number of runs: "+str(len(index_list)))
for index in index_list:
    folders.append(all_folders[index])
print(folders)

def find_closest_timestamp(A, target):
    #A must be sorted
    idx = A.searchsorted(target)
    idx = np.clip(idx, 1, len(A)-1)
    left = A[idx-1]
    right = A[idx]
    idx -= target - left < right - target
    return idx

# Initialize pandas DataFrame
ins = pd.DataFrame(columns=['timestamp','yaw'])

for folder in folders:
    df_gps = pd.read_csv(os.path.join(
        base_path,runs_folder,folder,gps_filename),sep=',')
    df_locations = pd.read_csv(os.path.join(
        base_path,runs_folder,folder,filename),sep=',')
    # df_locations['timestamp'] = runs_folder+folder + \
    #     pointcloud_fols+df_locations['timestamp'].astype(str)+'.bin'
    df_locations['timestamp'] = df_locations['timestamp'].astype(str)
    
    df_locations['yaw'] = 0.0
    for idx in range(len(df_locations)):
        loc_idx = find_closest_timestamp(df_gps['timestamp'].values, int(df_locations['timestamp'][idx]))
        df_locations['yaw'][idx] = df_gps['yaw'][loc_idx]
    df_locations.to_csv(os.path.join(base_path,runs_folder,folder,filename), index =False)
    # print(df_locations)
    # df_locations = df_locations.rename(columns={'timestamp':'file'})

