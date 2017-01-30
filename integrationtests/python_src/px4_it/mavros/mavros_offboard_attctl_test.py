#!/usr/bin/env python2
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
# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.

PKG = 'px4'

import unittest
import rospy
import rosbag
import time

from std_msgs.msg import Header
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from mavros_msgs.srv import CommandLong
from sensor_msgs.msg import NavSatFix
#from px4_test_helper import PX4TestHelper

class MavrosOffboardAttctlTest(unittest.TestCase):
    """
    Tests flying in offboard control by sending attitude and thrust setpoints
    via MAVROS.

    For the test to be successful it needs to cross a certain boundary in time.
    """

    def setUp(self):
        rospy.init_node('test_node', anonymous=True)
        rospy.wait_for_service('mavros/cmd/arming', 30)
        #self.helper = PX4TestHelper("mavros_offboard_attctl_test")
        #self.helper.setUp()

        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.position_callback)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.global_position_callback)
        self.pub_att = rospy.Publisher('mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        self.pub_thr = rospy.Publisher('mavros/setpoint_attitude/att_throttle', Float64, queue_size=10)
        rospy.wait_for_service('mavros/cmd/command', 30)
        self._srv_cmd_long = rospy.ServiceProxy('mavros/cmd/command', CommandLong, persistent=True)
        self.rate = rospy.Rate(10) # 10hz
        self.has_global_pos = False
        self.local_position = PoseStamped()

    def tearDown(self):
        #self.helper.tearDown()
        pass

    #
    # General callback functions used in tests
    #
    def position_callback(self, data):
        self.local_position = data

    def global_position_callback(self, data):
        self.has_global_pos = True

    def test_attctl(self):
        """Test offboard attitude control"""

        # FIXME: hack to wait for simulation to be ready
        while not self.has_global_pos:
            self.rate.sleep()

        # set some attitude and thrust
        att = PoseStamped()
        att.header = Header()
        att.header.frame_id = "base_footprint"
        att.header.stamp = rospy.Time.now()
        quaternion = quaternion_from_euler(0.25, 0.25, 0)
        att.pose.orientation = Quaternion(*quaternion)

        throttle = Float64()
        throttle.data = 0.7
        armed = False

        # does it cross expected boundaries in X seconds?
        count = 0
        timeout = 120
        while count < timeout:
            # update timestamp for each published SP
            att.header.stamp = rospy.Time.now()

            self.pub_att.publish(att)
            #self.helper.bag_write('mavros/setpoint_attitude/attitude', att)
            self.pub_thr.publish(throttle)
            #self.helper.bag_write('mavros/setpoint_attitude/att_throttle', throttle)
            self.rate.sleep()

            # FIXME: arm and switch to offboard
            # (need to wait the first few rounds until PX4 has the offboard stream)
            if not armed and count > 5:
                self._srv_cmd_long(False, 176, False,
                                   1, 6, 0, 0, 0, 0, 0)
                # make sure the first command doesn't get lost
                time.sleep(1)

                self._srv_cmd_long(False, 400, False,
                                   # arm
                                   1, 0, 0, 0, 0, 0, 0)

                armed = True

            if (self.local_position.pose.position.x > 5
                    and self.local_position.pose.position.z > 5
                    and self.local_position.pose.position.y < -5):
                break
            count = count + 1

        self.assertTrue(count < timeout, "took too long to cross boundaries")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'mavros_offboard_attctl_test', MavrosOffboardAttctlTest)
    #unittest.main()
