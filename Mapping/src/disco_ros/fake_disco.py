#!/home/mav-lab/miniconda3/envs/rostorch/bin/python3
import os
import cv2
import sys
import math
import time
import rospy
import torch
import socket
#import gpuadder
import argparse
import importlib
import numpy as np
import config as cfg
import torch.nn as nn
import scipy.io as scio
#import models.DiSCO as SC
import loss.loss_function
from dislam_msgs.msg import *
from torch.backends import cudnn
from torch.autograd import Variable
from sklearn.neighbors import KDTree
import sensor_msgs.point_cloud2 as pc2
from tensorboardX import SummaryWriter
from sensor_msgs.msg import PointCloud2
from torchvision import transforms, utils
from geometry_msgs.msg import Point, Point32
from sklearn.neighbors import NearestNeighbors


def callback(data):

    disco_msg = DiSCO()
    #disco_msg.signature = np.random.random(1024)
    #disco_msg.fftr = np.random.random(1024)
    #disco_msg.ffti = np.random.random(1024)
    disco_msg.pose = data.pose
    disco_msg.stamp = data.keyframePC.header.stamp
    pub.publish(disco_msg)


if __name__ == "__main__":

    rospy.init_node('disco_generator', anonymous=True)
    print("Ready to publish disco")
    pub = rospy.Publisher('/robot0/disco', DiSCO, queue_size=10)
    rospy.Subscriber("/robot0/submap", SubMap, callback)
    rospy.spin()
    
