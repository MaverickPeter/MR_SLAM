import os
import numpy as np
import matplotlib.pyplot as plt

# main parameters
# minDist = 0 # minimum range
# maxDist = 0.8 # maximum range for normalized pointcloud
# b = 100 # bucket count

def calculateRange(xyzs):
    # calculate the minimum and maximum range of a pointcloud
    minDist = 0.0 # initialize
    maxDist = 0.0 # initialize
    for i in range(len(xyzs)):
        xyz = xyzs[i]
        dist = (xyz[0]**2 + xyz[1]**2 + xyz[2]**2)**0.5
        if dist < minDist:
            minDist = dist
        if dist > maxDist:
            maxDist = dist
    return minDist, maxDist


def distanceJudge(distance, minDist, maxDist, b):
    # RANGEJUDGE
    distBucket = (maxDist - minDist) / b # delta_b
    
    if (distance > minDist and distance < maxDist):
        # from .cpp of laserScan_Similarity
       for i in range(b):
           if (distance >= (minDist + i*distBucket) and distance <= (minDist + (i+1)*distBucket)):
                bucket = i
    else:
        bucket = b - 1
        # bucket = -1
    
    return bucket

    
def statisticsOnRange(xyzs, b=100):
    # STATISTICSONRANGE
    minDist, maxDist = calculateRange(xyzs)
    vector = np.zeros(b)
    num_points = len(xyzs)

    for i in range(num_points):
        xyz = xyzs[i]
#         if xyz[2] < -5:   # some points filtered
#            continue
        dist = (xyz[0]**2 + xyz[1]**2 + xyz[2]**2)**0.5
        # print('distance', dist)
    
        # statistics
        bucket = distanceJudge(dist, minDist, maxDist, b)
        # if bucket  >=  0:
        #    vector[bucket] += 1
        vector[bucket] += 1
  
    # normalization
    for i in range(b):
       vector[i]  /= num_points

    return vector


def getEMD(vector1, vector2):
    # employ the discrete Wasserstein metric (Earth Mover's Distance (EMD)) between two histograms G and H
    result = 0.0
    for i in range(len(vector1)):
        result += abs(vector1[i] - vector2[i])
    result = result / len(vector1)
    return result


# employ the discrete Wasserstein metric between two histograms G and H
# Earth Mover's Distance (EMD)
def calculateW(H, G):
    # https://en.wikipedia.org/wiki/Wasserstein_metric
    # https://en.wikipedia.org/wiki/Earth_movers_distance
    result = 0.0
    for i in range(len(H)):
        result += abs(H[i] - G[i])
    result = result / len(H)
    return result