#! /usr/bin/env python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of Willow Garage, Inc. nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#* 
#* Author: Eitan Marder-Eppstein
#***********************************************************
PKG = 'test_tf2'

import sys
import unittest

import tf2_py as tf2
import tf2_ros
import tf2_kdl
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import rospy
import PyKDL

class TestBufferClient(unittest.TestCase):
    def test_buffer_client(self):
        client = tf2_ros.BufferClient("tf_action")
        client.wait_for_server()

        p1 = PointStamped()
        p1.header.frame_id = "a"
        p1.header.stamp = rospy.Time(0.0)
        p1.point.x = 0.0
        p1.point.y = 0.0
        p1.point.z = 0.0

        try:
            p2 = client.transform(p1, "b")
            rospy.loginfo("p1: %s, p2: %s" % (p1, p2))
        except tf2.TransformException as e:
            rospy.logerr("%s" % e)

    def test_transform_type(self):
        client = tf2_ros.BufferClient("tf_action")
        client.wait_for_server()

        p1 = PointStamped()
        p1.header.frame_id = "a"
        p1.header.stamp = rospy.Time(0.0)
        p1.point.x = 0.0
        p1.point.y = 0.0
        p1.point.z = 0.0

        try:
            p2 = client.transform(p1, "b", new_type = PyKDL.Vector)
            rospy.loginfo("p1: %s, p2: %s" % (str(p1), str(p2)))
        except tf2.TransformException as e:
            rospy.logerr("%s" % e)

if __name__ == '__main__':
    rospy.init_node("test_buffer_client")
    import rostest
    rostest.rosrun(PKG, 'test_buffer_client', TestBufferClient)
