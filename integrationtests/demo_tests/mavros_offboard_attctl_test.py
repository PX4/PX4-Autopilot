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
PKG = 'px4'

import sys
import unittest
import rospy
import math

from numpy import linalg
import numpy as np

from px4.msg import vehicle_control_mode
from std_msgs.msg import Header
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from mavros.srv import CommandBool

from manual_input import ManualInput

#
# Tests flying a path in offboard control by sending position setpoints
# over MAVROS.
#
# For the test to be successful it needs to reach all setpoints in a certain time.
# FIXME: add flight path assertion (needs transformation from ROS frame to NED)
#
class MavrosOffboardAttctlTest(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_node', anonymous=True)
        rospy.wait_for_service('mavros/cmd/arming', 30)
        rospy.Subscriber('px4_multicopter/vehicle_control_mode', vehicle_control_mode, self.vehicle_control_mode_callback)
        rospy.Subscriber("mavros/position/local", PoseStamped, self.position_callback)
        self.pubAtt = rospy.Publisher('mavros/setpoint/attitude', PoseStamped, queue_size=10)
        self.pubThr = rospy.Publisher('mavros/setpoint/att_throttle', Float64, queue_size=10)
        self.cmdArm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.rate = rospy.Rate(10) # 10hz
        self.rateSec = rospy.Rate(1)
        self.hasPos = False
        self.controlMode = vehicle_control_mode()

    #
    # General callback functions used in tests
    #
    def position_callback(self, data):
        self.hasPos = True
        self.localPosition = data

    def vehicle_control_mode_callback(self, data):
        self.controlMode = data


    #
    # Helper methods
    #
    def arm(self):
        return self.cmdArm(value=True)

    #
    # Test offboard position control
    #
    def test_attctl(self):
        # FIXME: this must go ASAP when arming is implemented
        manIn = ManualInput()
        manIn.arm()
        manIn.offboard_attctl()

        self.assertTrue(self.arm(), "Could not arm")
        self.rateSec.sleep()
        self.rateSec.sleep()
        self.assertTrue(self.controlMode.flag_armed, "flag_armed is not set after 2 seconds")

        # set some attitude and thrust
        att = PoseStamped()
        att.header = Header() 
        att.header.frame_id = "base_footprint"
        att.header.stamp = rospy.Time.now()
        quaternion = quaternion_from_euler(0.15, 0.15, 0)
        att.pose.orientation = Quaternion(*quaternion)

        throttle = Float64()
        throttle.data = 0.6

        # does it cross expected boundaries in X seconds?
        count = 0
        timeout = 120
        while(count < timeout):
            # update timestamp for each published SP
            att.header.stamp = rospy.Time.now()
            self.pubAtt.publish(att)
            self.pubThr.publish(throttle)
            
            if (self.localPosition.pose.position.x > 5
                and self.localPosition.pose.position.z > 5
                and self.localPosition.pose.position.y < -5):
                break
            count = count + 1
            self.rate.sleep()

        self.assertTrue(count < timeout, "took too long to cross boundaries")
    

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'mavros_offboard_attctl_test', MavrosOffboardAttctlTest)
    #unittest.main()
