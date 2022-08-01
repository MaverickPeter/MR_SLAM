#!/usr/bin/env python
#coding:utf-8

import rospy
import sys
sys.path.append('.')
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def pubImage():
    rospy.init_node('pubImageSim2',anonymous = True)
    pub = rospy.Publisher('robot_2/image_rect', Image, queue_size = 10)
    rate = rospy.Rate(10)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        image = np.zeros((640,480,3), np.uint8)
        msg = bridge.cv2_to_imgmsg(image,"bgr8")
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        #cv2.imshow("lala",image)
        #cv2.waitKey(0)
        rate.sleep()

if __name__ == '__main__':
    try:
        pubImage()
    except rospy.ROSInterruptException:
        pass
