#FPFH.py
import open3d as o3d
import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
from sklearn.neighbors import KDTree # KDTree 进行搜索
import random
from ISS import *         #  iss feature detection
from pandas import DataFrame

def visual_feature_description(fpfh,keypoint_idx):
    for i in range(len(fpfh)):
        x = [i for i in range(len(fpfh[i]))]
        y = fpfh[i]
        plt.plot(x,y,label=keypoint_idx[i])
    #添加显示图例
    plt.title('Description Visualization for Keypoints')
    plt.legend(bbox_to_anchor=(1, 1),  # 图例边界框起始位置
               loc="upper right",  # 图例的位置
               ncol=1,  # 列数
               mode="None",  # 当值设置为“expend”时，图例会水平扩展至整个坐标轴区域
               borderaxespad=0,  # 坐标轴和图例边界之间的间距
               title="keypoints",  # 图例标题
               shadow=False,  # 是否为线框添加阴影
               fancybox=True)  # 线框圆角处理参数
    plt.xlabel("label")
    plt.ylabel("fpfh")
    plt.show()


def get_spfh(point_cloud, nearest_idx, keypoint_id, radius, B):   # single pfh
    points = np.asarray(point_cloud)
    keypoint = np.asarray(point_cloud)[keypoint_id]
    #remove query point 去除关键点  :
    key_nearest_idx = nearest_idx[keypoint_id]
    key_nearest_idx = list(set(nearest_idx[keypoint_id]) - set([keypoint_id]))
    key_nearest_idx = np.asarray(key_nearest_idx)
    ##step8 计算 u,v,w
    #向量 p2_p1
    diff = points[key_nearest_idx] - keypoint  # p2 - p1,shape: (k,3)  k为该点有多少个nearest points
    diff /= np.linalg.norm(diff,ord=2,axis=1)[:,None] #[:,None]的效果就是将二维数组按每行分割，最后形成一个三维数组 ,eg shape : (k,1)
    #compute n1 n2
    n1 = np.asarray(point_cloud_normals[keypoint_id])       #keypoint 邻近点的法向量
    n2 = np.asarray(point_cloud_normals[key_nearest_idx])   #keypoint 邻近点的邻近点的法向量
    #compute u v w
    u = n1
    v = np.cross(u,diff)
    w = np.cross(u,v)
    #compute alpha  phi theta 三元组
    alpha = np.multiply(v,n2).sum(axis=1)#alpha = (v*n2).sum(axis=1)
    phi = np.multiply(u,diff).sum(axis=1)#phi = (u * diff).sum(axis=1)
    theta = np.arctan2(np.multiply(w,n2).sum(axis=1), (u * n2).sum(axis=1))#theta = np.arctan2((w * n2).sum(axis=1), (u * n2).sum(axis=1))
    ##step9 计算直方图 histogram
    # get alpha histogram:
    alpha_histogram = np.histogram(alpha, bins=B, range=(-1.0, +1.0))[0]
    alpha_histogram = alpha_histogram / alpha_histogram.sum()
    # get phi histogram:
    phi_histogram = np.histogram(phi, bins=B, range=(-1.0, +1.0))[0]
    phi_histogram = phi_histogram / phi_histogram.sum()
    # get theta histogram:
    theta_histogram = np.histogram(theta, bins=B, range=(-np.pi, +np.pi))[0]
    theta_histogram = theta_histogram / theta_histogram.sum()
    ##step10 拼接直方图 histogram
    # build signature:
    signature = np.hstack(
        (
            # alpha:
            alpha_histogram,
            # phi:
            phi_histogram,
            # theta:
            theta_histogram
        )
    )
    return signature



def describe(point_cloud, nearest_idx, keypoint_id, radius, B):   # single pfh
    ##step5 寻找每个keypoint 的nearest points
    points = np.asarray(point_cloud)
    keypoint = np.asarray(point_cloud)[keypoint_id]
    #remove query point 去除关键点  :
    key_nearest_idx = nearest_idx[keypoint_id]
    key_nearest_idx = list(set(nearest_idx[keypoint_id]) - set([keypoint_id]))
    key_nearest_idx = np.asarray(key_nearest_idx)
    k = len(key_nearest_idx)             #keypoint的临近点个数
    ##step6 计算该关键点 群 权重 weights:
    W = 1.0 / np.linalg.norm(points[key_nearest_idx] - keypoint , ord=2, axis=1)
    ##step7 计算nearest points 的spfh
    X = np.asarray(
        [get_spfh(point_cloud,nearest_idx,i,radius,B) for i in key_nearest_idx]
    )
    ##step11 neighbor 的 spfh 权重和
    spfh_neighborhood = 1.0 / (k) * np.dot(W, X)
    ##step12 keypoints 的 spfh
    spfh_query = get_spfh(point_cloud,nearest_idx,keypoint_id,radius,B)
    ##step13 finally
    spfh = spfh_query + spfh_neighborhood
    # normalize again:
    spfh = spfh / np.linalg.norm(spfh)

    return spfh

if __name__ == '__main__':
    pc_path = '/media/client/Mav/Datasets/NCLT/2012-05-26/velodyne_data/velodyne_sync/1338076484263340.bin'

    point_cloud = load_lidar_file_nclt(pc_path)
    pc_normalized = load_pc_infer(point_cloud)    
    pc_normalized = pc_normalized[:, 0:3]  # 为 xyz的 N*3矩阵
    import time
    start = time.time()
    point_cloud_raw = point_cloud[:, 0:3]  # 为 xyz的 N*3矩阵
    ##step1 使用iss 找出所有关键点,并以label的形式输出
    keypoint_idx = iss(point_cloud_raw)
    print(keypoint_idx)
    feature_point = point_cloud[keypoint_idx]
    #visualization feature points
    #Point_Cloud_Show(point_cloud,feature_point)
    ##step2 求出所有点的法向量,在modelNet40数据集中,每个数据点的后三位为该点的法向量
    point_cloud_normals = point_cloud[:, 3:6]  # 为 point normal的法向量
    ##step3 构建radius nn tree
    leaf_size = 4
    radius = 0.05
    search_tree = KDTree(point_cloud_raw,leaf_size)  #构建 kd_tree
    ##step4 求解每个关键点的 spfh
    B = 5  # 每个 直方图 bin的个数
    nearest_idx = search_tree.query_radius(point_cloud_raw, radius)   #求解每个点的最邻近点
    #description the keypoints
    FPFH = np.asarray(
        [describe(point_cloud_raw, nearest_idx, keypoint_id, radius, B) for keypoint_id in keypoint_idx]
    )
    #visualization all feature description
    visual_feature_description(FPFH,keypoint_idx)
    #visualization test similar feature description ,相近的点 7847-3843 ； 6336-8605 ； 5508-7644
    test_keypoint_idx = [7847,3843] # [7847,3843] , [6336,8605] , [5508,7644]
    test_FPFH = np.asarray(
        [describe(point_cloud_raw, nearest_idx, keypoint_id, radius, B) for keypoint_id in test_keypoint_idx]
    )
    visual_feature_description(test_FPFH, test_keypoint_idx)
    # describe(point_cloud_raw, nearest_idx, keypoint_idx[0], radius, B)
    print(time.time() - start)