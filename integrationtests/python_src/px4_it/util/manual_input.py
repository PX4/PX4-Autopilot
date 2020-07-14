#!/usr/bin/env python
#***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#
import rospy

from px4.msg import manual_control_setpoint
from px4.msg import offboard_control_mode
from mav_msgs.msg import CommandAttitudeThrust
from std_msgs.msg import Header

#
# Manual input control helper
#
class ManualInput(object):

    def __init__(self):
        rospy.init_node('test_node', anonymous=True)
        self.pub_mcsp = rospy.Publisher('manual_control_setpoint', manual_control_setpoint, queue_size=10)
        self.pub_off = rospy.Publisher('offboard_control_mode', offboard_control_mode, queue_size=10)

    def arm(self):
        rate = rospy.Rate(10) # 10hz

        att = CommandAttitudeThrust()
        att.header = Header()

        pos = manual_control_setpoint()
        pos.x = 0
        pos.z = 0
        pos.y = 0
        pos.r = 0
        pos.mode_switch = 3
        pos.return_switch = 3
        pos.posctl_switch = 3
        pos.loiter_switch = 3
        pos.acro_switch = 0
        pos.offboard_switch = 3

        count = 0
        while not rospy.is_shutdown() and count < 5:
            rospy.loginfo("zeroing")
            time = rospy.get_rostime().now()
            pos.timestamp = time.secs * 1e6 + time.nsecs / 1000
            self.pub_mcsp.publish(pos)
            rate.sleep()
            count = count + 1

        pos.r = 1
        count = 0
        while not rospy.is_shutdown() and count < 5:
            rospy.loginfo("arming")
            time = rospy.get_rostime().now()
            pos.timestamp = time.secs * 1e6 + time.nsecs / 1000
            self.pub_mcsp.publish(pos)
            rate.sleep()
            count = count + 1

    def posctl(self):
        rate = rospy.Rate(10) # 10hz

        # triggers posctl
        pos = manual_control_setpoint()
        pos.x = 0
        pos.z = 0
        pos.y = 0
        pos.r = 0
        pos.mode_switch = 2
        pos.return_switch = 3
        pos.posctl_switch = 1
        pos.loiter_switch = 3
        pos.acro_switch = 0
        pos.offboard_switch = 3

        count = 0
        while not rospy.is_shutdown() and count < 5:
            rospy.loginfo("triggering posctl")
            time = rospy.get_rostime().now()
            pos.timestamp = time.secs * 1e6 + time.nsecs / 1000
            self.pub_mcsp.publish(pos)
            rate.sleep()
            count = count + 1


    def offboard_attctl(self):
        self.offboard(False, False, True, True, True, True)

    def offboard_posctl(self):
        self.offboard(False, False, True, False, True, True)

    # Trigger offboard and set offboard control mode before
    def offboard(self, ignore_thrust=False, ignore_attitude=False, ignore_bodyrate=True,
                        ignore_position=False, ignore_velocity=True, ignore_acceleration_force=True):
        rate = rospy.Rate(10) # 10hz

        mode = offboard_control_mode()
        mode.ignore_thrust = ignore_thrust
        mode.ignore_attitude = ignore_attitude
        mode.ignore_bodyrate_x = ignore_bodyrate
        mode.ignore_bodyrate_y = ignore_bodyrate
        mode.ignore_bodyrate_z = ignore_bodyrate
        mode.ignore_position = ignore_position
        mode.ignore_velocity = ignore_velocity
        mode.ignore_acceleration_force = ignore_acceleration_force

        count = 0
        while not rospy.is_shutdown() and count < 5:
            rospy.loginfo("setting offboard mode")
            time = rospy.get_rostime().now()
            mode.timestamp = time.secs * 1e6 + time.nsecs / 1000
            self.pub_off.publish(mode)
            rate.sleep()
            count = count + 1

        # triggers offboard
        pos = manual_control_setpoint()
        pos.x = 0
        pos.z = 0
        pos.y = 0
        pos.r = 0
        pos.mode_switch = 3
        pos.return_switch = 3
        pos.posctl_switch = 3
        pos.loiter_switch = 3
        pos.acro_switch = 0
        pos.offboard_switch = 1

        count = 0
        while not rospy.is_shutdown() and count < 5:
            rospy.loginfo("triggering offboard")
            time = rospy.get_rostime().now()
            pos.timestamp = time.secs * 1e6 + time.nsecs / 1000
            self.pub_mcsp.publish(pos)
            rate.sleep()
            count = count + 1

