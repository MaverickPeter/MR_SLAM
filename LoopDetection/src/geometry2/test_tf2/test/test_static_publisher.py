#! /usr/bin/env python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2016, Felix Duvallet
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
#* Author: Felix Duvallet
#***********************************************************

import subprocess
import unittest

import rospy
PKG = 'test_tf2'


class TestStaticPublisher(unittest.TestCase):
    """
    These tests ensure the static transform publisher dies gracefully when
    provided with an invalid (or non-existent) transform parameter.

    These tests are started by the static_publisher.launch, which loads
    parameters into the param server.

    We check the output to make sure the correct error is occurring, since the
    return code is always -1 (255).

    Note that this *could* cause a problem if a valid TF is stored in the param
    server for one of the names; in this case the subprocess would never return
    and the test would run forever.
    """

    def test_publisher_no_args(self):
        # Start the publisher with no argument.
        cmd = 'rosrun tf2_ros static_transform_publisher'
        with self.assertRaises(subprocess.CalledProcessError) as cm:
            ret = subprocess.check_output(
                cmd.split(' '), stderr=subprocess.STDOUT, text=True)
        self.assertEqual(255, cm.exception.returncode)
        self.assertIn('not having the right number of arguments',
                      cm.exception.output)

    def test_publisher_nonexistent_param(self):
        # Here there is no paramater by that name.
        cmd = 'rosrun tf2_ros static_transform_publisher /test_tf2/tf_null'
        with self.assertRaises(subprocess.CalledProcessError) as cm:
            ret = subprocess.check_output(
                cmd.split(' '), stderr=subprocess.STDOUT, text=True)

        self.assertEqual(255, cm.exception.returncode)
        self.assertIn('Could not read TF', cm.exception.output)

    def test_publisher_invalid_param(self):
        # Here there is an invalid parameter stored in the parameter server.
        cmd = 'rosrun tf2_ros static_transform_publisher /test_tf2/tf_invalid'
        with self.assertRaises(subprocess.CalledProcessError) as cm:
            ret = subprocess.check_output(
                cmd.split(' '), stderr=subprocess.STDOUT, text=True)

        self.assertEqual(255, cm.exception.returncode)
        self.assertIn('Could not validate XmlRpcC', cm.exception.output)


if __name__ == '__main__':
    rospy.init_node("test_static_publisher_py")
    import rostest
    rostest.rosrun(PKG, 'test_static_publisher_py', TestStaticPublisher)
