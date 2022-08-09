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

import threading

import rospy
from tf2_msgs.msg import TFMessage


class TransformListener:
    """
    :class:`TransformListener` is a convenient way to listen for coordinate frame transformation info.
    This class takes an object that instantiates the :class:`BufferInterface` interface, to which
    it propagates changes to the tf frame graph.
    """
    def __init__(self, buffer, queue_size=None, buff_size=65536, tcp_nodelay=False):
        """
        .. function:: __init__(buffer, queue_size=None, buff_size=65536, tcp_nodelay=False):

            Constructor.

            :param buffer: The buffer to propagate changes to when tf info updates.
            :param queue_size: Maximum number of messages to receive at a time. This will generally be 1 or None (infinite, default). buff_size should be increased if this parameter is set as incoming data still needs to sit in the incoming buffer before being discarded. Setting queue_size buff_size to a non-default value affects all subscribers to this topic in this process.
            :param buff_size: Incoming message buffer size in bytes. If queue_size is set, this should be set to a number greater than the queue_size times the average message size. Setting buff_size to a non-default value affects all subscribers to this topic in this process.
            :param tcp_nodelay: If True, request TCP_NODELAY from publisher. Use of this option is not generally recommended in most cases as it is better to rely on timestamps in message data. Setting tcp_nodelay to True enables TCP_NODELAY for all subscribers in the same python process.
        """
        self.buffer = buffer
        self.last_update = rospy.Time.now()
        self.last_update_lock = threading.Lock()
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.callback, queue_size=queue_size, buff_size=buff_size, tcp_nodelay=tcp_nodelay)
        self.tf_static_sub = rospy.Subscriber("/tf_static", TFMessage, self.static_callback, queue_size=queue_size, buff_size=buff_size, tcp_nodelay=tcp_nodelay)

    def __del__(self):
        self.unregister()

    def unregister(self):
        """
        Unregisters all tf subscribers.
        """
        self.tf_sub.unregister()
        self.tf_static_sub.unregister()

    def check_for_reset(self):
        # Lock to prevent different threads racing on this test and update.
        # https://github.com/ros/geometry2/issues/341
        with self.last_update_lock:
            now = rospy.Time.now()
            if now < self.last_update:
                rospy.logwarn("Detected jump back in time of %fs. Clearing TF buffer." % (self.last_update - now).to_sec())
                self.buffer.clear()
            self.last_update = now

    def callback(self, data):
        self.check_for_reset()
        who = data._connection_header.get('callerid', "default_authority")
        for transform in data.transforms:
            self.buffer.set_transform(transform, who)

    def static_callback(self, data):
        self.check_for_reset()
        who = data._connection_header.get('callerid', "default_authority")
        for transform in data.transforms:
            self.buffer.set_transform_static(transform, who)
