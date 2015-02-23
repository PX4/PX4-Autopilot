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

from px4.msg import actuator_armed
from manual_input import ManualInput

class ArmTest(unittest.TestCase):

    #
    # General callback functions used in tests
    #
    def actuator_armed_callback(self, data):
        self.actuatorStatus = data
    
    #
    # Test arming
    #
    def test_arm(self):
        rospy.init_node('test_node', anonymous=True)
        sub = rospy.Subscriber('px4_multicopter/actuator_armed', actuator_armed, self.actuator_armed_callback)

        # method to test
        arm = ManualInput()
        arm.arm()

        self.assertEquals(self.actuatorStatus.armed, True, "not armed")


    

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'arm_test', ArmTest)
