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

import rospy
import tf2_py as tf2
import tf2_ros
from tf2_msgs.srv import FrameGraph, FrameGraphResponse
import rosgraph.masterapi

class Buffer(tf2.BufferCore, tf2_ros.BufferInterface):
    """
    Standard implementation of the :class:`tf2_ros.BufferInterface` abstract data type.

    Inherits from :class:`tf2_ros.buffer_interface.BufferInterface` and :class:`tf2.BufferCore`.

    Stores known frames and optionally offers a ROS service, "tf2_frames", which responds to client requests
    with a response containing a :class:`tf2_msgs.FrameGraph` representing the relationship of
    known frames. 
    """

    def __init__(self, cache_time = None, debug = True):
        """
        .. function:: __init__(cache_time = None, debug = True)

            Constructor.

            :param cache_time: (Optional) How long to retain past information in BufferCore.
            :param debug: (Optional) If true, check if another tf2_frames service has been advertised.
        """
        if cache_time != None:
            tf2.BufferCore.__init__(self, cache_time)
        else:
            tf2.BufferCore.__init__(self)
        tf2_ros.BufferInterface.__init__(self)

        if debug:
            #Check to see if the service has already been advertised in this node
            try:
                m = rosgraph.masterapi.Master(rospy.get_name())
                m.lookupService('~tf2_frames')
            except (rosgraph.masterapi.Error, rosgraph.masterapi.Failure):   
                self.frame_server = rospy.Service('~tf2_frames', FrameGraph, self.__get_frames)

    def __get_frames(self, req):
       return FrameGraphResponse(self.all_frames_as_yaml()) 

    def lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
        """
        Get the transform from the source frame to the target frame.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest) 
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: The transform between the frames.
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """

        self.can_transform(target_frame, source_frame, time, timeout)
        return self.lookup_transform_core(target_frame, source_frame, time)

    def lookup_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame, timeout=rospy.Duration(0.0)):
        """
        Get the transform from the source frame to the target frame using the advanced API.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to. (0 will get the latest) 
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (0 will get the latest) 
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: The transform between the frames.
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        self.can_transform_full(target_frame, target_time, source_frame, source_time, fixed_frame, timeout)
        return self.lookup_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame)


    def can_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0), return_debug_tuple=False):
        """
        Check if a transform from the source frame to the target frame is possible.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest) 
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :param return_debug_type: (Optional) If true, return a tuple representing debug information.
        :return: True if the transform is possible, false otherwise.
        :rtype: bool
        """
        if timeout != rospy.Duration(0.0):
            start_time = rospy.Time.now()
            r= rospy.Rate(20)
            while (rospy.Time.now() < start_time + timeout and 
                   not self.can_transform_core(target_frame, source_frame, time)[0] and
                   (rospy.Time.now()+rospy.Duration(3.0)) >= start_time): # big jumps in time are likely bag loops, so break for them
                r.sleep()
        core_result = self.can_transform_core(target_frame, source_frame, time)
        if return_debug_tuple:
            return core_result
        return core_result[0]

    def can_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame, timeout=rospy.Duration(0.0),

                           return_debug_tuple=False):
        """
        Check if a transform from the source frame to the target frame is possible (advanced API).

        Must be implemented by a subclass of BufferInterface.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to. (0 will get the latest) 
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (0 will get the latest) 
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :param return_debug_type: (Optional) If true, return a tuple representing debug information.
        :return: True if the transform is possible, false otherwise.
        :rtype: bool
        """
        if timeout != rospy.Duration(0.0):
            start_time = rospy.Time.now()
            r= rospy.Rate(20)
            while (rospy.Time.now() < start_time + timeout and 
                   not self.can_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame)[0] and
                   (rospy.Time.now()+rospy.Duration(3.0)) >= start_time): # big jumps in time are likely bag loops, so break for them
                r.sleep()
        core_result = self.can_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame)
        if return_debug_tuple:
            return core_result
        return core_result[0]

