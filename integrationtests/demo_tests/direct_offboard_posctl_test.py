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

from numpy import linalg
import numpy as np

from px4.msg import vehicle_local_position
from px4.msg import vehicle_control_mode
from px4.msg import actuator_armed
from px4.msg import position_setpoint_triplet
from px4.msg import position_setpoint
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

from manual_input import ManualInput
from flight_path_assertion import FlightPathAssertion

#
# Tests flying a path in offboard control by directly sending setpoints
# to the position controller (position_setpoint_triplet).
#
# For the test to be successful it needs to stay on the predefined path
# and reach all setpoints in a certain time.
#
class DirectOffboardPosctlTest(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_node', anonymous=True)
        rospy.Subscriber('px4_multicopter/vehicle_control_mode', vehicle_control_mode, self.vehicle_control_mode_callback)
        rospy.Subscriber("px4_multicopter/vehicle_local_position", vehicle_local_position, self.position_callback)
        self.pubSpt = rospy.Publisher('px4_multicopter/position_setpoint_triplet', position_setpoint_triplet, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

    def tearDown(self):
        if (self.fpa):
            self.fpa.stop()

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
    def is_at_position(self, x, y, z, offset):
        rospy.logdebug("current position %f, %f, %f" % (self.localPosition.x, self.localPosition.y, self.localPosition.z))
        desired = np.array((x, y, z))
        pos = np.array((self.localPosition.x, self.localPosition.y, self.localPosition.z))
        return linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        # set a position setpoint
        pos = position_setpoint()
        pos.valid = True
        pos.x = x
        pos.y = y
        pos.z = z
        pos.position_valid = True
        stp = position_setpoint_triplet()
        stp.current = pos
        self.pubSpt.publish(stp)

        # does it reach the position in X seconds?
        count = 0
        while(count < timeout):
            if(self.is_at_position(pos.x, pos.y, pos.z, 0.5)):
                break
            count = count + 1
            self.rate.sleep()

        self.assertTrue(count < timeout, "took too long to get to position")

    #
    # Test offboard position control
    #
    def test_posctl(self):
        manIn = ManualInput()

        # arm and go into offboard
        manIn.arm()
        manIn.offboard()
        self.assertTrue(self.controlMode.flag_armed, "flag_armed is not set")
        self.assertTrue(self.controlMode.flag_control_offboard_enabled, "flag_control_offboard_enabled is not set")
        self.assertTrue(self.controlMode.flag_control_position_enabled, "flag_control_position_enabled is not set")

        # prepare flight path
        positions = (
            (0,0,0),
            (2,2,-2),
            (2,-2,-2),
            (-2,-2,-2),
            (2,2,-2))

        # flight path assertion
        self.fpa = FlightPathAssertion(positions, 1, 0)
        self.fpa.start()

        for i in range(0, len(positions)):
            self.reach_position(positions[i][0], positions[i][1], positions[i][2], 120)
            self.assertFalse(self.fpa.failed, "breached flight path tunnel (%d)" % i)
        
        # does it hold the position for Y seconds?
        positionHeld = True
        count = 0
        timeout = 50
        while(count < timeout):
            if(not self.is_at_position(2, 2, -2, 0.5)):
                positionHeld = False
                break
            count = count + 1
            self.rate.sleep()

        self.assertTrue(count == timeout, "position could not be held")
        self.fpa.stop()
    

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'direct_offboard_posctl_test', DirectOffboardPosctlTest)
    #unittest.main()
