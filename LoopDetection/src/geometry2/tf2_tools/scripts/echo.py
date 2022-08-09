#!/usr/bin/env python3

# tf2 echo code Copyright (c) 2018, Lucas Walter
# transformations.py code Copyright (c) 2006-2017, Christoph Gohlke
# transformations.py code Copyright (c) 2006-2017, The Regents of the University of California
# Produced at the Laboratory for Fluorescence Dynamics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the copyright holders nor the names of any
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function

import argparse
import math
import numpy
import rospy
import sys
import tf2_py as tf2
import tf2_ros

from geometry_msgs.msg import TransformStamped
# https://github.com/ros/geometry2/issues/222
# from tf import transformations

"""
The following euler conversion functions are from https://github.com/matthew-brett/transforms3d
which adapted it from transformations.py, it is needed here until transforms3d is available
as a dependency.

They are for internal use only.
"""

# epsilon for testing whether a number is close to zero
_EPS = numpy.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# TODO(lucasw) if sxyz works then eliminate the other possibilities
# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

def _euler_from_matrix(matrix, axes='sxyz'):
    """temporaray import from https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/_gohlketransforms.py for internal use only"""
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

def _quaternion_matrix(quaternion):
    """temporaray import from https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/_gohlketransforms.py for internal use only"""
    q = numpy.array(quaternion, dtype=numpy.float64, copy=True)
    n = numpy.dot(q, q)
    if n < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / n)
    q = numpy.outer(q, q)
    return numpy.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])

def _euler_from_quaternion(quaternion, axes='sxyz'):
    """temporaray import from https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/_gohlketransforms.py for internal use only"""
    return _euler_from_matrix(_quaternion_matrix(quaternion), axes)

def _euler_from_quaternion_msg(quaternion):
    # the above code is from transform3 which changed convention from old transformations.py
    # from xyzw to wxyz
    # return transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return _euler_from_quaternion([quaternion.w,
                                   quaternion.x,
                                   quaternion.y,
                                   quaternion.z])

class Echo():
    def __init__(self, args):
        self.tf_buffer = tf2_ros.Buffer(cache_time=args.cache_time)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.args = args

        self.count = 0
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.args.rate), self.lookup)

    def lookup(self, event):
        self.count += 1
        if self.args.limit:
            if self.count > self.args.limit:
                # TODO(lucasw) is there a better method to stop the spin()?
                rospy.signal_shutdown("tf echo finished")
                return

        cur_time = rospy.Time.now()
        # If the transform is from tf_static the ts.header.stamp will be 0.0
        # when offset == 0 or lookup_time is rospy.Time()
        if self.args.time:
            lookup_time = rospy.Time(self.args.time)
        elif self.args.offset:
            # If the transform is static this will always work
            lookup_time = cur_time + rospy.Duration(self.args.offset)
        else:
            # Get the most recent transform
            lookup_time = rospy.Time()

        try:
            ts = self.tf_buffer.lookup_transform(self.args.source_frame,
                                                 self.args.target_frame,
                                                 lookup_time)
        except tf2.LookupException as ex:
            msg = "At time {}, (current time {}) ".format(lookup_time.to_sec(), cur_time.to_sec())
            rospy.logerr(msg + str(ex))
            return
        except tf2.ExtrapolationException as ex:
            msg = "(current time {}) ".format(cur_time.to_sec())
            rospy.logerr(msg + str(ex))
            return

        # The old tf1 static_transform_publisher (which published into /tf, not /tf_static)
        # publishes transforms 0.5 seconds into future so the cur_time and header stamp
        # will be identical.
        msg = "At time {}, (current time {})".format(ts.header.stamp.to_sec(), cur_time.to_sec())
        xyz = ts.transform.translation
        msg += "\n- Translation: [{:.{p}f}, {:.{p}f}, {:.{p}f}]\n".format(xyz.x, xyz.y, xyz.z, p=self.args.precision)
        quat = ts.transform.rotation
        msg += "- Rotation: in Quaternion [{:.{p}f}, {:.{p}f}, {:.{p}f}, {:.{p}f}]\n".format(quat.x, quat.y, quat.z, quat.w, p=self.args.precision)
        # TODO(lucasw) need to get quaternion to euler from somewhere, but not tf1
        # or a dependency that isn't in Ubuntu or ros repos
        euler = _euler_from_quaternion_msg(quat)
        msg += "            in RPY (radian) "
        msg += "[{:.{p}f}, {:.{p}f}, {:.{p}f}]\n".format(euler[0], euler[1], euler[2], p=self.args.precision)
        msg += "            in RPY (degree) "
        msg += "[{:.{p}f}, {:.{p}f}, {:.{p}f}]".format(math.degrees(euler[0]),
                                                 math.degrees(euler[1]),
                                                 math.degrees(euler[2]), p=self.args.precision)
        print(msg)

def positive_float(x):
    x = float(x)
    if x <= 0.0:
        raise argparse.ArgumentTypeError("{} must be > 0.0".format(x))
    return x

def positive_int(x):
    x = int(x)
    if x <= 0:
        raise argparse.ArgumentTypeError("{} must be > 0".format(x))
    return x

if __name__ == '__main__':
    rospy.init_node("echo")

    other_args = rospy.myargv(argv=sys.argv)
    precision=3
    try:
        precision = rospy.get_param('~precision')
        rospy.loginfo("Precision default value was overriden, new value: %d", precision)
    except KeyError:
        pass

    parser = argparse.ArgumentParser()
    parser.add_argument("source_frame")  # parent
    parser.add_argument("target_frame")  # child
    parser.add_argument("-r", "--rate",
                        help="update rate, must be > 0.0",
                        default=1.0,
                        type=positive_float)
    parser.add_argument("-c", "--cache_time",
                        help="length of tf buffer cache in seconds",
                        type=positive_float)
    parser.add_argument("-o", "--offset",
                        help="offset the lookup from current time, ignored if using -t",
                        type=float)
    parser.add_argument("-t", "--time",
                        help="fixed time to do the lookup",
                        type=float)
    parser.add_argument("-l", "--limit",
                        help="lookup fixed number of times",
                        type=positive_int)
    parser.add_argument("-p", "--precision",
                        help="output precision",
                        default=precision,
                        type=positive_int)
    args = parser.parse_args(other_args[1:]) # Remove first arg
    echo = Echo(args)
    rospy.spin()
