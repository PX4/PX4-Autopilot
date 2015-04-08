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

import unittest
import rospy

from px4.msg import actuator_armed
from px4.msg import vehicle_control_mode
from manual_input import ManualInput

#
# Tests if commander reacts to manual input and sets control flags accordingly
#
class ManualInputTest(unittest.TestCase):

    def setUp(self):
        self.actuator_status = actuator_armed()
        self.control_mode = vehicle_control_mode()

    #
    # General callback functions used in tests
    #
    def actuator_armed_callback(self, data):
        self.actuator_status = data

    def vehicle_control_mode_callback(self, data):
        self.control_mode = data
    
    #
    # Test arming
    #
    def test_manual_input(self):
        rospy.init_node('test_node', anonymous=True)
        rospy.Subscriber('actuator_armed', actuator_armed, self.actuator_armed_callback)
        rospy.Subscriber('vehicle_control_mode', vehicle_control_mode, self.vehicle_control_mode_callback)

        man_in = ManualInput()

        # Test arming
        man_in.arm()
        self.assertEquals(self.actuator_status.armed, True, "did not arm")

        # Test posctl
        man_in.posctl()
        self.assertTrue(self.control_mode.flag_control_position_enabled, "flag_control_position_enabled is not set")

        # Test offboard
        man_in.offboard()
        self.assertTrue(self.control_mode.flag_control_offboard_enabled, "flag_control_offboard_enabled is not set")
    

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'direct_manual_input_test', ManualInputTest)
