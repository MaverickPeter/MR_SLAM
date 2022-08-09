# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# author: Wim Meeussen

from __future__ import print_function

import rospy
import tf2_py as tf2
import tf2_ros
from copy import deepcopy
from std_msgs.msg import Header

class BufferInterface:
    """
    Abstract interface for wrapping the Python bindings for the tf2 library in
    a ROS-based convenience API.
    Implementations include :class:tf2_ros.buffer.Buffer and
    :class:tf2_ros.buffer_client.BufferClient.
    """
    def __init__(self):
        self.registration = tf2_ros.TransformRegistration()

    # transform, simple api
    def transform(self, object_stamped, target_frame, timeout=rospy.Duration(0.0), new_type = None):
        """
        Transform an input into the target frame.

        The input must be a known transformable type (by way of the tf2 data type conversion interface).

        If new_type is not None, the type specified must have a valid conversion from the input type,
        else the function will raise an exception.

        :param object_stamped: The timestamped object the transform.
        :param target_frame: Name of the frame to transform the input into.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :param new_type: (Optional) Type to convert the object to.
        :return: The transformed, timestamped output, possibly converted to a new type.
        """
        do_transform = self.registration.get(type(object_stamped))
        res = do_transform(object_stamped, self.lookup_transform(target_frame, object_stamped.header.frame_id,
                                                                 object_stamped.header.stamp, timeout))
        if not new_type:
            return res

        return convert(res, new_type)

    # transform, advanced api
    def transform_full(self, object_stamped, target_frame, target_time, fixed_frame, timeout=rospy.Duration(0.0), new_type = None):
        """
        Transform an input into the target frame (advanced API).

        The input must be a known transformable type (by way of the tf2 data type conversion interface).

        If new_type is not None, the type specified must have a valid conversion from the input type,
        else the function will raise an exception.

        This function follows the advanced API, which allows tranforming between different time points,
        as well as specifying a frame to be considered fixed in time.

        :param object_stamped: The timestamped object the transform.
        :param target_frame: Name of the frame to transform the input into.
        :param target_time: Time to transform the input into.
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :param new_type: (Optional) Type to convert the object to.
        :return: The transformed, timestamped output, possibly converted to a new type.
        """
        do_transform = self.registration.get(type(object_stamped))
        res = do_transform(object_stamped, self.lookup_transform_full(target_frame, target_time,
                                                                     object_stamped.header.frame_id, object_stamped.header.stamp, 
                                                                     fixed_frame, timeout))
        if not new_type:
            return res

        return convert(res, new_type)

    def lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
        """
        Get the transform from the source frame to the target frame.

        Must be implemented by a subclass of BufferInterface.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest) 
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: The transform between the frames.
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        raise NotImplementedException()

    def lookup_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame, timeout=rospy.Duration(0.0)):
        """
        Get the transform from the source frame to the target frame using the advanced API.

        Must be implemented by a subclass of BufferInterface.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to. (0 will get the latest) 
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (0 will get the latest) 
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: The transform between the frames.
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        raise NotImplementedException()        

    # can, simple api
    def can_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
        """
        Check if a transform from the source frame to the target frame is possible.

        Must be implemented by a subclass of BufferInterface.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest) 
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: True if the transform is possible, false otherwise.
        :rtype: bool
        """
        raise NotImplementedException()        
    
    # can, advanced api
    def can_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame, timeout=rospy.Duration(0.0)):
        """
        Check if a transform from the source frame to the target frame is possible (advanced API).

        Must be implemented by a subclass of BufferInterface.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to. (0 will get the latest) 
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (0 will get the latest) 
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: True if the transform is possible, false otherwise.
        :rtype: bool
        """
        raise NotImplementedException()        


def Stamped(obj, stamp, frame_id):
    obj.header = Header(frame_id=frame_id, stamp=stamp)
    return obj



class TypeException(Exception):
    """
    Raised when an unexpected type is received while registering a transform
    in :class:`tf2_ros.buffer_interface.BufferInterface`.
    """
    def __init__(self, errstr):
        self.errstr = errstr

class NotImplementedException(Exception):
    """
    Raised when can_transform or lookup_transform is not implemented in a
    subclass of :class:`tf2_ros.buffer_interface.BufferInterface`.
    """
    def __init__(self):
        self.errstr = 'CanTransform or LookupTransform not implemented'


class TransformRegistration():
    __type_map = {}
    
    def print_me(self):
        print(TransformRegistration.__type_map)

    def add(self, key, callback):
        TransformRegistration.__type_map[key] = callback

    def get(self, key):
        if not key in TransformRegistration.__type_map:
            raise TypeException('Type %s if not loaded or supported'% str(key))
        else:
            return TransformRegistration.__type_map[key]

class ConvertRegistration():
    __to_msg_map = {}
    __from_msg_map = {}
    __convert_map = {}
    
    def add_from_msg(self, key, callback):
        ConvertRegistration.__from_msg_map[key] = callback

    def add_to_msg(self, key, callback):
        ConvertRegistration.__to_msg_map[key] = callback

    def add_convert(self, key, callback):
        ConvertRegistration.__convert_map[key] = callback

    def get_from_msg(self, key):
        if not key in ConvertRegistration.__from_msg_map:
            raise TypeException('Type %s if not loaded or supported'% str(key))
        else:
            return ConvertRegistration.__from_msg_map[key]

    def get_to_msg(self, key):
        if not key in ConvertRegistration.__to_msg_map:
            raise TypeException('Type %s if not loaded or supported'%str(key))
        else:
            return ConvertRegistration.__to_msg_map[key]

    def get_convert(self, key):
        if not key in ConvertRegistration.__convert_map:
            raise TypeException("Type %s if not loaded or supported" % str(key))
        else:
            return ConvertRegistration.__convert_map[key]

def convert(a, b_type):
    c = ConvertRegistration()
    #check if an efficient conversion function between the types exists
    try:
        f = c.get_convert((type(a), b_type))
        print("efficient copy")
        return f(a)
    except TypeException:
        if type(a) == b_type:
            print("deep copy")
            return deepcopy(a)

        f_to = c.get_to_msg(type(a))
        f_from = c.get_from_msg(b_type)
        print("message copy")
        return f_from(f_to(a))
