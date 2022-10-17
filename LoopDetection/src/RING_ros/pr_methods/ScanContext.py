import os
import torch
from util import *
import config as cfg
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree

# np.set_printoptions(precision=4)

# Original SC step 1: place retrieval using a retrieval key
# SC++ step 1: place retrieval using a retrieval key
def make_ringkey(sc):
    row_idx = 0
    ring_key = np.zeros(cfg.num_ring)
    while row_idx < cfg.num_ring:
        curr_row = sc[:,row_idx,:]
        ring_key[row_idx] = np.mean(curr_row)
        row_idx += 1

    return ring_key 

def make_sectorkey(sc):
    col_idx = 0
    sector_key = np.zeros(cfg.num_sector)
    while col_idx < cfg.num_sector:
        curr_col = sc[...,col_idx]
        sector_key[col_idx] = np.mean(curr_col)
        col_idx += 1
    
    return sector_key

# Original SC step 2: compute the aligning distance in original scan context 
def distance_sc(sc1, sc2):
    sc1 = sc1.squeeze()
    sc2 = sc2.squeeze()
    num_sectors = cfg.num_sector 

    # repeate to move 1 column
    _one_step = 1 # const
    sim_for_each_cols = np.zeros(num_sectors)

    for i in range(num_sectors):
        # column shift
        sc1 = np.roll(sc1, _one_step, axis=-1)

        # compare
        sum_of_cossim = 0
        num_col_engaged = 0
        for j in range(num_sectors):
            col_j_1 = sc1[:, j]
            col_j_2 = sc2[:, j]
            if np.linalg.norm(col_j_1) > 0 and np.linalg.norm(col_j_2) > 0:
                # to avoid being divided by zero when calculating cosine similarity
                cos_sim = np.dot(col_j_1, col_j_2) / (np.linalg.norm(col_j_1) * np.linalg.norm(col_j_2))
                sum_of_cossim = sum_of_cossim + cos_sim
                num_col_engaged = num_col_engaged + 1

        # save 
        if num_col_engaged > 0:
            sim_for_each_cols[i] = sum_of_cossim / num_col_engaged
        else:
            sim_for_each_cols[i] = 0

    yaw_diff = np.argmax(sim_for_each_cols) + 1 # because python starts with 0 
    sim = np.max(sim_for_each_cols)
    dist = 1 - sim

    return dist, yaw_diff

# fast alignment of sc1 and sc2 using the difference of two scs
def fast_align(sc1, sc2):
    shift_idx = 0
    min_diff_norm = 1e8
    while shift_idx < cfg.num_sector:
        sc2_shifted = np.roll(sc2, shift_idx, axis=-1)
        sc_diff = sc1 - sc2_shifted
        curr_diff_norm = np.linalg.norm(sc_diff)
        if curr_diff_norm < min_diff_norm:
            min_shift = shift_idx
            min_diff_norm = curr_diff_norm
        shift_idx += 1

    return min_diff_norm, min_shift

# SC++ step 2: semimetric localization via prealignment using an aligning key
def fast_align_with_sectorkey(sector_key1, sector_key2):
    # sector_key1 = make_sectorkey(sc1)
    # sector_key2 = make_sectorkey(sc2)
    shift_idx = 0
    min_diff_norm = 1e8
    while shift_idx < cfg.num_sector:
        sector_key2_shifted = np.roll(sector_key2, shift_idx)
        sector_key_diff = sector_key1 - sector_key2_shifted
        curr_diff_norm = np.linalg.norm(sector_key_diff)
        if curr_diff_norm < min_diff_norm:
            min_shift = shift_idx
            min_diff_norm = curr_diff_norm
        shift_idx += 1

    return min_diff_norm, min_shift


# SC++ PC step 3: full SCD comparison for potential refinement and localization-quality assessment
def dist_direct_sc(sc1, sc2):
    sc1 = sc1.squeeze()
    sc2 = sc2.squeeze()
    num_eff_cols = 0
    sum_sector_similarity = 0

    # print(sc1.shape, sc2.shape)
    for col_idx in range(cfg.num_sector):
        col_sc1 = sc1[:,col_idx]
        col_sc2 = sc2[:,col_idx]
        # print(col_sc1.shape, col_sc2.shape)
        if np.linalg.norm(col_sc1) > 0 and np.linalg.norm(col_sc2) > 0:
            sector_similarity = np.dot(col_sc1,col_sc2) / (np.linalg.norm(col_sc1) *np.linalg.norm(col_sc2))
            sum_sector_similarity = sum_sector_similarity + sector_similarity
            num_eff_cols = num_eff_cols + 1
    if num_eff_cols > 0:
        sc_dist = 1.0 - sum_sector_similarity / num_eff_cols
    else:
        sc_dist = 1.0
        
    return sc_dist

# SC++ PC step 3: full SCD comparison for potential refinement and localization-quality assessment
def dist_align_sc(sc1, sc2, search_ratio=cfg.search_ratio):
    vkey_sc1 = make_sectorkey(sc1)
    vkey_sc2 = make_sectorkey(sc2)
    _, argmin_vkey_shift = fast_align_with_sectorkey(vkey_sc1, vkey_sc2)
    search_radius = round(0.5*search_ratio*cfg.num_sector)
    shift_idx_search_space = range(max(-cfg.num_sector, argmin_vkey_shift-search_radius), min(cfg.num_sector, argmin_vkey_shift+search_radius+1)) 
    min_sc_dist = 1e8
    for num_shift in shift_idx_search_space:
        sc2_shifted = np.roll(sc2, num_shift, axis=-1)
        cur_sc_dist = dist_direct_sc(sc1, sc2_shifted)
        if cur_sc_dist < min_sc_dist:
            min_sc_dist = cur_sc_dist
            min_shift = num_shift

    return min_sc_dist, min_shift

def dist_align_cc(sc1, sc2, search_ratio=cfg.search_ratio):
    vkey_sc1 = make_ringkey(sc1)
    vkey_sc2 = make_ringkey(sc2)
    _, argmin_vkey_shift = fast_align_with_sectorkey(vkey_sc1, vkey_sc2)
    search_radius = round(0.5*search_ratio*cfg.num_ring)
    shift_idx_search_space = range(max(-cfg.num_ring, argmin_vkey_shift-search_radius), min(cfg.num_ring, argmin_vkey_shift+search_radius+1)) 
    min_sc_dist = 1e8
    for num_shift in shift_idx_search_space:
        sc2_shifted = np.roll(sc2, num_shift, axis=-2)
        cur_sc_dist = dist_direct_sc(sc1, sc2_shifted)
        if cur_sc_dist < min_sc_dist:
            min_sc_dist = cur_sc_dist
            min_shift = num_shift

    return min_sc_dist, min_shift