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
import rosbag

from px4.msg import vehicle_control_mode
from px4.msg import vehicle_local_position
from px4.msg import vehicle_attitude_setpoint
from px4.msg import vehicle_attitude
from std_msgs.msg import Header
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from px4_test_helper import PX4TestHelper

#
# Tests flying a path in offboard control by sending attitude and thrust setpoints
# over MAVROS.
#
# For the test to be successful it needs to cross a certain boundary in time.
#
class MavrosOffboardAttctlTest(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_node', anonymous=True)
        rospy.wait_for_service('mavros/cmd/arming', 30)
        self.helper = PX4TestHelper("mavros_offboard_attctl_test")
        self.helper.setUp()

        rospy.Subscriber('vehicle_control_mode', vehicle_control_mode, self.vehicle_control_mode_callback)
        rospy.Subscriber("mavros/local_position/local", PoseStamped, self.position_callback)
        self.pub_att = rospy.Publisher('mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        self.pub_thr = rospy.Publisher('mavros/setpoint_attitude/att_throttle', Float64, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        self.has_pos = False
        self.control_mode = vehicle_control_mode()
        self.local_position = PoseStamped()

    def tearDown(self):
        self.helper.tearDown()

    #
    # General callback functions used in tests
    #
    def position_callback(self, data):
        self.has_pos = True
        self.local_position = data

    def vehicle_control_mode_callback(self, data):
        self.control_mode = data

    #
    # Test offboard position control
    #
    def test_attctl(self):
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
        while count < timeout:
            # update timestamp for each published SP
            att.header.stamp = rospy.Time.now()

            self.pub_att.publish(att)
            self.helper.bag_write('mavros/setpoint_attitude/attitude', att)
            self.pub_thr.publish(throttle)
            self.helper.bag_write('mavros/setpoint_attitude/att_throttle', throttle)
            self.rate.sleep()

            if (self.local_position.pose.position.x > 5
                    and self.local_position.pose.position.z > 5
                    and self.local_position.pose.position.y < -5):
                break
            count = count + 1

        self.assertTrue(self.control_mode.flag_armed, "flag_armed is not set")
        self.assertTrue(self.control_mode.flag_control_attitude_enabled, "flag_control_attitude_enabled is not set")
        self.assertTrue(self.control_mode.flag_control_offboard_enabled, "flag_control_offboard_enabled is not set")
        self.assertTrue(count < timeout, "took too long to cross boundaries")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'mavros_offboard_attctl_test', MavrosOffboardAttctlTest)
    #unittest.main()
