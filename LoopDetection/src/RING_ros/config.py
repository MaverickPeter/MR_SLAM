
# KNN params
k_num = 30

# GPU Process
# Normal Resolution
num_ring = 120
num_sector = 120
num_height = 1
max_length = 1
max_height = 1

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