#!/usr/bin/env python

import math
import rospy
from serial_buffer import SerialBuffer as SerialBuffer
from gps_common.msg import GPSFix, GPSStatus
from std_srvs.srv import Trigger, TriggerResponse


def to_float(s):
    try:
        val = float(s)
    except ValueError:
        val = float('nan')
    return val


def to_int(s):
    try:
        val = int(s)
    except ValueError:
        val = -1
    return val


class GPSReader:

    def __init__(self):
        self.sb = SerialBuffer("/dev/ttyUSB0", 115200)
        if not self.sb.is_open():
            print "Cannot open port"
            exit()

        self.reset_srv = rospy.Service('rtk_gps/reset', Trigger, self.reset_callback)

        self.pub = rospy.Publisher('/rtk_gps/gps', GPSFix)
        self.msg = GPSFix()

    def spin_once(self):
        serial_msg = self.sb.spin_once()
        if serial_msg:
            self.process(serial_msg)

    def reset_callback(self, req):
        print 'Reset'
        return TriggerResponse(True, 'Reset')

    def process(self, msg):

        if msg[0] == '$PTNL':
            if len(msg) == 13:
                # mm.northing = float(msg[4])
                # mm.easting = float(msg[6])
                self.msg.err_vert = to_float(msg[4])
                self.msg.err_horz = to_float(msg[6])
                self.msg.altitude = to_float(msg[11][3:])
                quality = to_int(msg[8])
                self.msg.status.status = {0: GPSStatus.STATUS_NO_FIX,
                                          1: GPSStatus.STATUS_FIX,
                                          2: GPSStatus.STATUS_FIX,
                                          3: GPSStatus.STATUS_DGPS_FIX}[quality]
                self.msg.status.satellites_used = to_int(msg[9])
                self.msg.gdop = to_float(msg[10])
            else:
                rospy.logerr('Wrong number of $PTNL message: 13 expected but %d given', len(msg))
                return

        elif msg[0] == '$PSAT':
            if len(msg) != 7:
                rospy.logerr('Wrong number of $PSAT message: 7 expected but %d given', len(msg))
                return
            self.msg.track = to_float(msg[3]) / 180.0 * math.pi
            self.msg.pitch = to_float(msg[4]) / 180.0 * math.pi
            self.msg.roll = to_float(msg[5]) / 180.0 * math.pi

        elif msg[0] == '$GPRMC':
            if len(msg) != 14:
                rospy.logerr('Wrong number of $GPRMC message: 14 expected but %d given', len(msg))
                return
            if msg[2] == 'V':
                self.msg.status.status = GPSStatus.STATUS_NO_FIX
            else:
                self.msg.latitude = (to_float(msg[3]) if msg[4] == 'N' else -to_float(msg[3])) / 100
                self.msg.longitude = (to_float(msg[5]) if msg[6] == 'E' else -to_float(msg[5])) / 100
                self.msg.speed = to_float(msg[7])
                self.msg.track = to_float(msg[8])
                if msg[12] == 'D':
                    self.msg.status.status = GPSStatus.STATUS_DGPS_FIX
            self.publish()
        else:
            rospy.logwarn('Unknown msg:' + str(msg))
            return

    def publish(self):
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)


if __name__ == "__main__":

    rospy.init_node('rtk_gps')

    reader = GPSReader()
    while not rospy.is_shutdown():
        reader.spin_once()
