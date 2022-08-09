import os
import pickle
import numpy as np
import random
import config as cfg
import struct
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import gputransform
import time

def get_queries_dict(filename):
    # key:{'query':file,'positives':[files],'negatives:[files], 'neighbors':[keys]}
    with open(filename, 'rb') as handle:
        queries = pickle.load(handle)
        print("Queries Loaded.")
        return queries


def get_sets_dict(filename):
    #[key_dataset:{key_pointcloud:{'query':file,'northing':value,'easting':value}},key_dataset:{key_pointcloud:{'query':file,'northing':value,'easting':value}}, ...}
    with open(filename, 'rb') as handle:
        trajectories = pickle.load(handle)
        print("Trajectories Loaded.")
        return trajectories

def convert(x_s, y_s, z_s):

    scaling = 0.005 # 5 mm
    offset = -100.0

    x = x_s * scaling + offset
    y = y_s * scaling + offset
    z = z_s * scaling + offset

    return x, y, z

def load_lidar_file_nclt(file_path):
    n_vec = 4
    # dtype = np.float32
    # lidar_pc_raw = np.fromfile(file_path, dtype)
    f_bin = open(file_path,'rb')

    hits = []

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
        if np.abs(x) < 70. and z > -20. and z < -2. and np.abs(y) < 70. and not(np.abs(x) < 5. and np.abs(y) < 5.):
            hits += [[x/70., y/70., z/20.]]
        # hits += [[x, y, z]]

    f_bin.close()
    hits = np.asarray(hits)
    hits[:, 2] = -hits[:, 2]

    return hits


def load_pc_file_infer(filename):
    # returns Nx3 matrix
    pc = load_lidar_file_nclt(filename)  
    size = pc.shape[0]
    pc_point = np.zeros([cfg.num_height * cfg.num_ring * cfg.num_sector])
    pc = pc.transpose().flatten().astype(np.float32)

    transer = gputransform.GPUTransformer(pc, size, cfg.max_length, cfg.num_ring, cfg.num_sector, cfg.num_height, 1)
    transer.transform()
    point_t = transer.retreive()
    point_t = point_t.reshape(-1, 3)
    point_t = point_t[...,2]
    pc_point = point_t.reshape(cfg.num_height, cfg.num_ring, cfg.num_sector)
    
    return pc_point


def load_pc_infer(pc):
    # returns Nx3 matrix
    pc = np.array(pc, dtype=np.float32)

    x_condition = np.abs(pc[...,0]) < 70.
    y_condition = np.abs(pc[...,1]) < 70.
    z_condition1 = pc[...,2] < 30.
    z_condition2 = pc[...,2] > 0.
    conditions_1 = np.bitwise_and(x_condition, y_condition)
    conditions_2 = np.bitwise_and(z_condition1, z_condition2)
    conditions = np.bitwise_and(conditions_1, conditions_2)
    hits = pc[conditions]

    hits[...,0] = hits[...,0] / 70.
    hits[...,1] = hits[...,1] / 70.
    hits[...,2] = (hits[...,2]) / 30.
    
    # x = hits[...,0]
    # y = hits[...,1]
    # z = hits[...,2]
    # fig2 = plt.figure()
    # ax2 = Axes3D(fig2)
    # ax2.scatter(x, y, z)
    # plt.show()

    # hits = hits.transpose((1,0))

    pc = np.array(hits, dtype=np.float32)
    size = pc.shape[0]

    pc_point = np.zeros([cfg.num_height * cfg.num_ring * cfg.num_sector])
    pc = pc.transpose().flatten().astype(np.float32)

    transer = gputransform.GPUTransformer(pc, size, cfg.max_length, cfg.max_height, cfg.num_ring, cfg.num_sector, cfg.num_height, 1)
    transer.transform()
    point_t = transer.retreive()
    point_t = point_t.reshape(-1, 3)
    point_t = point_t[...,2]
    pc_point = point_t.reshape(cfg.num_height, cfg.num_ring, cfg.num_sector)
    
    return pc_point

def load_pc_file(filename):
    # returns Nx3 matrix
    #print("filename",filename)
    #time_start = time.time()
    # pc = np.fromfile(os.path.join("/media/mav-lab/1T/Datasets/NCLT/NCLT/", filename), dtype=np.float64)
    #pc = load_lidar_file_nclt(os.path.join("/media/mav-lab/1T/Datasets/NCLT/NCLT/", filename))
    #pc[:,0] = pc[:,0] / np.max(pc[:,0])
    #pc[:,1] = pc[:,1] / np.max(pc[:,1])
    #pc[:,2] = pc[:,2] / np.max(pc[:,2])
    # print("filename",filename)
    filename = filename.replace('.bin','.npy')
    filename = filename.replace('/velodyne_sync/','/sc_density_0.5m/')
    # print("filename",filename)
    pc_point = np.load(filename)
    

    # x = pc[...,0]
    # y = pc[...,1]
    # z = pc[...,2]
    # fig2 = plt.figure()
    # ax2 = Axes3D(fig2)
    # ax2.scatter(x, y, z)
    # plt.show()

    #size = pc.shape[0]
    #pc_point = np.zeros([cfg.num_height * cfg.num_ring * cfg.num_sector])
    #pc = pc.transpose().flatten().astype(np.float32)

    #transer = gputransform.GPUTransformer(pc, size, cfg.max_length, cfg.num_ring, cfg.num_sector, cfg.num_height, cfg.overlap_num)
    #transer.transform()
    #point_t = transer.retreive()
    #point_t = point_t.reshape(-1, 3)
    #point_t = point_t[...,2]
    #pc_point = point_t.reshape(cfg.num_height, cfg.num_ring, cfg.num_sector)
    #time_end = time.time()

    #print("loading time", time_end - time_start, 's')
    #pc = np.sum(pc_point, axis=0)
    #plt.imshow(pc)
    #plt.show()


    # if(pc.shape[0] != 4096*3):
    #     print("Error in pointcloud shape")
    #     return np.array([])

    # if(pc.shape[0] % 3):
    #     print("Error in pointcloud shape")
    #     return np.array([])

    # pc = np.reshape(pc,(pc.shape[0]//3, 3))

    # print("pc shape", pc.shape)
    return pc_point


def load_pc_files(filenames):
    pcs = []
    for filename in filenames:
        # print(filename)
        pc = load_pc_file(filename)
        # if(pc.shape[0] != 4096):
        #     continue
        pcs.append(pc)
    pcs = np.array(pcs)
    return pcs


def rotate_point_cloud(batch_data):
    """ Randomly rotate the point clouds to augument the dataset
        rotation is per shape based along up direction
        Input:
          BxNx3 array, original batch of point clouds
        Return:
          BxNx3 array, rotated batch of point clouds
    """
    rotated_data = np.zeros(batch_data.shape, dtype=np.float32)
    for k in range(batch_data.shape[0]):
        #rotation_angle = np.random.uniform() * 2 * np.pi
        #-90 to 90
        rotation_angle = (np.random.uniform()*np.pi) - np.pi/2.0
        cosval = np.cos(rotation_angle)
        sinval = np.sin(rotation_angle)
        rotation_matrix = np.array([[cosval, -sinval, 0],
                                    [sinval, cosval, 0],
                                    [0, 0, 1]])
        shape_pc = batch_data[k, ...]
        rotated_data[k, ...] = np.dot(
            shape_pc.reshape((-1, 3)), rotation_matrix)
    return rotated_data


def jitter_point_cloud(batch_data, sigma=0.005, clip=0.05):
    """ Randomly jitter points. jittering is per point.
        Input:
          BxNx3 array, original batch of point clouds
        Return:
          BxNx3 array, jittered batch of point clouds
    """
    B, N, C = batch_data.shape
    assert(clip > 0)
    jittered_data = np.clip(sigma * np.random.randn(B, N, C), -1*clip, clip)
    jittered_data += batch_data
    return jittered_data


def get_query_tuple(dict_value, num_pos, num_neg, QUERY_DICT, hard_neg=[], other_neg=False):
        # get query tuple for dictionary entry
        # return list [query,positives,negatives]
    heading = []
    query = load_pc_file(dict_value["query"])  # Nx3
    heading.append(dict_value["heading"])

    random.shuffle(dict_value["positives"])
    pos_files = []
    
    for i in range(num_pos):
        pos_files.append(QUERY_DICT[dict_value["positives"][i]]["query"])
        heading.append(QUERY_DICT[dict_value["positives"][i]]["heading"])
    #print("northing", QUERY_DICT[dict_value["positives"][i]]["query"])
    #positives= load_pc_files(dict_value["positives"][0:num_pos])
    positives = load_pc_files(pos_files)

    neg_files = []
    neg_indices = []
    if(len(hard_neg) == 0):
        random.shuffle(dict_value["negatives"])
        for i in range(num_neg):
            neg_files.append(QUERY_DICT[dict_value["negatives"][i]]["query"])
            neg_indices.append(dict_value["negatives"][i])
            heading.append(QUERY_DICT[dict_value["negatives"][i]]["heading"])


    else:
        random.shuffle(dict_value["negatives"])
        for i in hard_neg:
            neg_files.append(QUERY_DICT[i]["query"])
            heading.append(QUERY_DICT[i]["heading"])

            neg_indices.append(i)
        j = 0
        while(len(neg_files) < num_neg):

            if not dict_value["negatives"][j] in hard_neg:
                neg_files.append(
                    QUERY_DICT[dict_value["negatives"][j]]["query"])
                heading.append(QUERY_DICT[dict_value["negatives"][j]]["heading"])

                neg_indices.append(dict_value["negatives"][j])
            j += 1

    negatives = load_pc_files(neg_files)

    if other_neg is False:
        return [query, positives, negatives, heading]
    # For Quadruplet Loss
    else:
        # get neighbors of negatives and query
        neighbors = []
        for pos in dict_value["positives"]:
            neighbors.append(pos)
        for neg in neg_indices:
            for pos in QUERY_DICT[neg]["positives"]:
                neighbors.append(pos)
        possible_negs = list(set(QUERY_DICT.keys())-set(neighbors))
        random.shuffle(possible_negs)

        if(len(possible_negs) == 0):
            return [query, positives, negatives, np.array([]), heading]

        neg2 = load_pc_file(QUERY_DICT[possible_negs[0]]["query"])
        heading.append(QUERY_DICT[possible_negs[0]]["heading"])
        heading = np.array(heading)

        return [query, positives, negatives, neg2, heading]


def get_rotated_tuple(dict_value, num_pos, num_neg, QUERY_DICT, hard_neg=[], other_neg=False):
    query = load_pc_file(dict_value["query"])  # Nx3
    q_rot = rotate_point_cloud(np.expand_dims(query, axis=0))
    q_rot = np.squeeze(q_rot)

    random.shuffle(dict_value["positives"])
    pos_files = []
    for i in range(num_pos):
        pos_files.append(QUERY_DICT[dict_value["positives"][i]]["query"])
    #positives= load_pc_files(dict_value["positives"][0:num_pos])
    positives = load_pc_files(pos_files)
    p_rot = rotate_point_cloud(positives)

    neg_files = []
    neg_indices = []
    if(len(hard_neg) == 0):
        random.shuffle(dict_value["negatives"])
        for i in range(num_neg):
            neg_files.append(QUERY_DICT[dict_value["negatives"][i]]["query"])
            neg_indices.append(dict_value["negatives"][i])
    else:
        random.shuffle(dict_value["negatives"])
        for i in hard_neg:
            neg_files.append(QUERY_DICT[i]["query"])
            neg_indices.append(i)
        j = 0
        while(len(neg_files) < num_neg):
            if not dict_value["negatives"][j] in hard_neg:
                neg_files.append(
                    QUERY_DICT[dict_value["negatives"][j]]["query"])
                neg_indices.append(dict_value["negatives"][j])
            j += 1
    negatives = load_pc_files(neg_files)
    n_rot = rotate_point_cloud(negatives)

    if other_neg is False:
        return [q_rot, p_rot, n_rot]

    # For Quadruplet Loss
    else:
        # get neighbors of negatives and query
        neighbors = []
        for pos in dict_value["positives"]:
            neighbors.append(pos)
        for neg in neg_indices:
            for pos in QUERY_DICT[neg]["positives"]:
                neighbors.append(pos)
        possible_negs = list(set(QUERY_DICT.keys())-set(neighbors))
        random.shuffle(possible_negs)

        if(len(possible_negs) == 0):
            return [q_jit, p_jit, n_jit, np.array([])]

        neg2 = load_pc_file(QUERY_DICT[possible_negs[0]]["query"])
        n2_rot = rotate_point_cloud(np.expand_dims(neg2, axis=0))
        n2_rot = np.squeeze(n2_rot)

        return [q_rot, p_rot, n_rot, n2_rot]


def get_jittered_tuple(dict_value, num_pos, num_neg, QUERY_DICT, hard_neg=[], other_neg=False):
    query = load_pc_file(dict_value["query"])  # Nx3
    #q_rot= rotate_point_cloud(np.expand_dims(query, axis=0))
    q_jit = jitter_point_cloud(np.expand_dims(query, axis=0))
    q_jit = np.squeeze(q_jit)

    random.shuffle(dict_value["positives"])
    pos_files = []
    for i in range(num_pos):
        pos_files.append(QUERY_DICT[dict_value["positives"][i]]["query"])
    #positives= load_pc_files(dict_value["positives"][0:num_pos])
    positives = load_pc_files(pos_files)
    p_jit = jitter_point_cloud(positives)

    neg_files = []
    neg_indices = []
    if(len(hard_neg) == 0):
        random.shuffle(dict_value["negatives"])
        for i in range(num_neg):
            neg_files.append(QUERY_DICT[dict_value["negatives"][i]]["query"])
            neg_indices.append(dict_value["negatives"][i])
    else:
        random.shuffle(dict_value["negatives"])
        for i in hard_neg:
            neg_files.append(QUERY_DICT[i]["query"])
            neg_indices.append(i)
        j = 0
        while(len(neg_files) < num_neg):
            if not dict_value["negatives"][j] in hard_neg:
                neg_files.append(
                    QUERY_DICT[dict_value["negatives"][j]]["query"])
                neg_indices.append(dict_value["negatives"][j])
            j += 1
    negatives = load_pc_files(neg_files)
    n_jit = jitter_point_cloud(negatives)

    if other_neg is False:
        return [q_jit, p_jit, n_jit]

    # For Quadruplet Loss
    else:
        # get neighbors of negatives and query
        neighbors = []
        for pos in dict_value["positives"]:
            neighbors.append(pos)
        for neg in neg_indices:
            for pos in QUERY_DICT[neg]["positives"]:
                neighbors.append(pos)
        possible_negs = list(set(QUERY_DICT.keys())-set(neighbors))
        random.shuffle(possible_negs)

        if(len(possible_negs) == 0):
            return [q_jit, p_jit, n_jit, np.array([])]

        neg2 = load_pc_file(QUERY_DICT[possible_negs[0]]["query"])
        n2_jit = jitter_point_cloud(np.expand_dims(neg2, axis=0))
        n2_jit = np.squeeze(n2_jit)

        return [q_jit, p_jit, n_jit, n2_jit]
