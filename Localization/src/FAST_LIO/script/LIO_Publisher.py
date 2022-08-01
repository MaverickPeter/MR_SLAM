#!/usr/bin/env python2
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
import tf

''' Descriptions
parent /odom
child /velodyne

1. odometry to tf 

2. ros param -> 15m: 2.1 pub /Bool and 2.2 PC rename to "keyframe_PointCloud" 
'''

class LIO_Pub():
    def __init__(self):
        self.dis_th = rospy.get_param("/dis_th", 5) # [m]
        # assert self.dis_th == 15
        self.Signal = False
        self.signal_num = 0
        self.isFirstOdom = True
        self.distance = 0
        rospy.loginfo("Init well!")

        # ROS topics
        rospy.init_node("LIO_Pub_node")
        self.PC_pub = rospy.Publisher("/keyframe_PointCloud2",PointCloud2,queue_size=10)
        self.Dis_Sig_pub = rospy.Publisher("/Distance_Signal",Bool,queue_size=10)
        #TODO self.TF_pub = rospy.Publisher("/TF",Bool,queue_size=10)

        self.Odom_sub = rospy.Subscriber("/Odometry",Odometry,self.OdomCallback)
        self.PC_sub = rospy.Subscriber("/PointCloud2",PointCloud2,self.PCCallback)

    # My Callbacks
    def PCCallback(self,msg):
        self.pub_PC(msg)

    def OdomCallback(self,msg):
        # print(dir(msg.pose.pose))
        pose = msg.pose.pose
        if self.isFirstOdom:
            self.x0, self.y0, self.z0 = pose.position.x, pose.position.y, pose.position.z
            self.isFirstOdom = False
        self.x1, self.y1, self.z1 = pose.position.x, pose.position.y, pose.position.z
        
        # Compute distance
        self.distance = ((self.x1 - self.x0) ** 2 + (self.y1 - self.y0) ** 2 + (self.z1 - self.z0) ** 2) ** 0.5

        self.pub_Signal()
        self.pub_TF(msg)
        rospy.loginfo("Distance is %f", self.distance)

        if self.distance > self.dis_th:
            # New start point
            self.x0, self.y0, self.z0 = self.x1, self.y1, self.z1

            self.signal_num += 1
            self.Signal = True
            self.pub_Signal()
            self.Signal = False
            self.distance = 0
            # Publish three info
            

    # My Publishers
    def pub_Signal(self):
        rospy.loginfo("No. %d signal", self.signal_num)
        self.Dis_Sig_pub.publish(Bool(self.Signal)) # TODO
    
    def pub_PC(self,msg):
        self.PC_pub.publish(msg)
    
    def pub_TF(self,msg): # msg : nav_msgs.Odometry
        br = tf.TransformBroadcaster()
        pose = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        br.sendTransform((pose.x, pose.y, pose.z),
                     (orientation.x, orientation.y, orientation.z, orientation.w),
                     rospy.Time.now(),
                     "velodyne",
                     "world")

def main():
    lio_pub = LIO_Pub()
    rospy.spin()

if __name__ == "__main__":
    main()