import os
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

pose_dir = "/home/uuleslie/SLAM/MR_SLAM_ws/orin/MR_SLAM/Mapping/src/global_manager/log/Keyframes"
pose_file = "/home/uuleslie/SLAM/MR_SLAM_ws/orin/MR_SLAM/Mapping/src/global_manager/log/full_graph.g2o"

def key_to_robotid(keys):
  keys = keys >> 56
  robotid = keys - 97
  return robotid

def robotid_to_key(robotid):
  char_a = 97
  indexBits = 56
  outkey = char_a + robotid
  return outkey << indexBits

# find closest place timestamp with index returned
def find_closest_timestamp(A, target):
    # A must be sorted
    idx = A.searchsorted(target)
    idx = np.clip(idx, 1, len(A)-1)
    left = A[idx-1]
    right = A[idx]
    idx -= target - left < right - target
    return idx

count = 0
loop_count = 0

first_timestamp = None

with open('./slam_results.txt', 'w') as wf:
    with open(pose_file, 'r') as rf:
        for line in rf:
            linevalue = line.split()
            type = linevalue[0]

            key = int(linevalue[1])
            timestamp = None
            timestamp2 = None
            key = str(key)
            if key == '0':
                continue

            try:
                with open(os.path.join(pose_dir, key, "data"), 'r') as rf:
                    for line in rf:
                        stamp = line.split()
                        if stamp[0] == 'stamp':
                            timestamp = str(stamp[1])[:10] + '.' + str(stamp[1])[10:]

                # If it's the first timestamp, initialize first_timestamp
                if first_timestamp is None:
                    first_timestamp = float(timestamp)

                # Calculate time difference (if first_timestamp is set)
                if first_timestamp is not None:
                    time_diff = float(timestamp) - first_timestamp

                if type == "VERTEX_SE3:QUAT":
                    x = linevalue[2]
                    y = linevalue[3]
                    z = linevalue[4]
                    qx = linevalue[5]
                    qy = linevalue[6]
                    qz = linevalue[7]
                    qw = linevalue[8]

                    # Use the time difference (time_diff)
                    line = [time_diff, x, y, z, qx, qy, qz, qw]  # Add time_diff to the line
                    line = ' '.join(str(i) for i in line)
                    wf.write(line)
                    wf.write("\n")
            except:
                print("No key")