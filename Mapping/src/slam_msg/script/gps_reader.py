#!/usr/bin/env python
# license removed for brevity

import math
import rospy
import message_filters
from serial_buffer import SerialBuffer as SerialBuffer
from rtk_gps.msg import Position, Orientation
from geometry_msgs.msg import Pose
from gps_common.msg import GPSFix
from std_srvs.srv import Trigger, TriggerResponse
import tf


class GPSReader:

    def __init__(self):
        self.sb = SerialBuffer("/dev/ttyUSB0", 115200)
        if not self.sb.is_open():
            print "Cannot open port"
            exit()

        self.position_pub = rospy.Publisher('/rtk_gps/position', Position, queue_size=10)
        self.orientation_pub = rospy.Publisher('/rtk_gps/orientation', Orientation, queue_size=10)
        self.pose_pub = rospy.Publisher('/rtk_gps/pose', Pose, queue_size=10)
        self.reset_srv = rospy.Service('rtk_gps/reset', Trigger, self.resetCallback)

        self.position_sub = message_filters.Subscriber('/rtk_gps/position', Position)
        self.orientation_sub = message_filters.Subscriber('/rtk_gps/orientation', Orientation)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.position_sub, self.orientation_sub], 10, 0.1)
        self.sync.registerCallback(self.callback)

        # self.sb.write('JASC,GPGGA,5,PORTA')

    def spin_once(self):
        serial_msg = self.sb.spin_once()
        if serial_msg:
            self.process(serial_msg)

    def callback(self, position_msg, orientation_msg):
        # yaw(z), pitch(x), roll(y)
        msg = Pose()
        msg.position.x = position_msg.northing
        msg.position.y = -position_msg.easting

        quaternion = tf.transformations.quaternion_from_euler(orientation_msg.pitch, orientation_msg.roll, -orientation_msg.yaw)
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        self.pose_pub.publish(msg)

    def resetCallback(self, req):
        print 'Reset'
        return TriggerResponse(True, 'Reset')

    def process(self, msg):
        if msg[0] == '$PTNL':
            if len(msg) == 13:
                mm = Position()
                try:
                    mm.northing = float(msg[4])
                    mm.easting = float(msg[6])
                    mm.height = float(msg[11][3:])
                    mm.quality = int(msg[8])
                    mm.fixStatelliteNum = int(msg[9])
                    mm.fixDOP = float(msg[10])
                    self.position_pub.publish(mm)
                except:
                    rospy.logerr('Failed to convert')
                    return
            else:
                rospy.logerr('Wrong number of $PTNL message: 13 expected but %d given', len(msg))
                return
        elif msg[0] == '$PSAT':
            if len(msg) != 7:
                rospy.logerr('Wrong number of $PSAT message: 7 expected but %d given', len(msg))
                return
            try:
                mm = Orientation()
                mm.yaw = float(msg[3]) / 180.0 * math.pi
                mm.pitch = float(msg[4]) / 180.0 * math.pi
                mm.roll = float(msg[5]) / 180.0 * math.pi
                self.orientation_pub.publish(mm)
            except:
                rospy.logwarn('GPS orientation not ready')
                return

if __name__ == "__main__":

    rospy.init_node('rtk_gps')

    reader = GPSReader()
    while not rospy.is_shutdown():
        reader.spin_once()
