import os
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

pose_dir = "/home/client/git-back/MR_ws/Mapping/src/global_manager/log/Keyframes"
pose_file = "/home/client/git-back/MR_ws/Mapping/src/global_manager/log/full_graph.g2o"

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

with open('./slam_results.txt', 'w') as wf:
  with open(pose_file,'r') as rf:
    for line in rf:
      linevalue = line.split()
      type = linevalue[0]

      key = int(linevalue[1])
      timestamp = None
      timestamp2 = None
      key = str(key - robotid_to_key(0))
      if key == '0':
        continue
      key = key.zfill(6)
      with open(os.path.join(pose_dir,key,"data"),'r') as rf:
        for line in rf:
          stamp = line.split()
          if stamp[0] == 'stamp':
            timestamp = str(stamp[1])[:10] + '.' + str(stamp[1])[10:]

      if type == "VERTEX_SE3:QUAT":

        x = linevalue[2]
        y = linevalue[3]
        z = linevalue[4]
        qx = linevalue[5]
        qy = linevalue[6]
        qz = linevalue[7]
        qw = linevalue[8]


        line = [timestamp, x, y, z, qx, qy, qz, qw]
        line = ' '.join(str(i) for i in line)
        wf.write(line)
        wf.write("\n")