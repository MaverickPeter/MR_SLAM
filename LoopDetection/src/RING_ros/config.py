
# Input Data Type
input_type = "point"  # "point" or "image"

# Sampling Gap
sampling_gap = 10 # in meters (5 meters)

# KNN params
k_num = 30

# Enable evaluation
enable_m2dp = True
enable_fh = True
enable_sc = True
enable_cc = True
enable_pc = True
enable_rs = False

result_path = "./results/NCLT/lpd_feat_dz/"

# GPU Process
# Normal Resolution
num_ring = 120
num_sector = 120
num_height = 1
max_length = 1
max_height = 1
# Low Resolution
num_ring_small = 40
num_sector_small = 40
num_height_small = 1
max_length_small = 1
max_height_small = 1
# High Resolution
num_ring_large = 200
num_sector_large = 200
num_height_large = 1
max_length_large = 1
max_height_large = 1

# Place Recognition Parameters
search_ratio = 0.1
num_candidates = 10
exclude_recent_nodes = 30
dist_threshold = 0.48 # descriptor distance threshold for place recognition
revisit_criteria = 5 # in meters

# ICP Parameters
max_icp_iter = 50
icp_tolerance = 0.001
icp_max_distance = 5.0
num_icp_points = 6000
icp_fitness_score = 0.22