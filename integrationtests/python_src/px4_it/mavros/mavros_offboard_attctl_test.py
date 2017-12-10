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
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header


class MavrosOffboardAttctlTest(unittest.TestCase):
    """
    Tests flying in offboard control by sending attitude and thrust setpoints
    via MAVROS.

    For the test to be successful it needs to cross a certain boundary in time.
    """

    def setUp(self):
        self.rate = rospy.Rate(10)  # 10hz
        self.has_global_pos = False
        self.local_position = PoseStamped()
        self.state = State()

        # setup ROS topics and services
        rospy.wait_for_service('mavros/cmd/arming', 30)
        rospy.wait_for_service('mavros/set_mode', 30)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming',
                                                 CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped,
                         self.position_callback)
        rospy.Subscriber('mavros/global_position/global', NavSatFix,
                         self.global_position_callback)
        rospy.Subscriber('mavros/state', State, self.state_callback)
        self.att_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

    def tearDown(self):
        pass

    #
    # General callback functions used in tests
    #
    def position_callback(self, data):
        self.local_position = data

    def global_position_callback(self, data):
        self.has_global_pos = True

    def state_callback(self, data):
        self.state = data

    #
    # Helper methods
    #
    def wait_until_ready(self):
        """FIXME: hack to wait for simulation to be ready"""
        rospy.loginfo("waiting for global position")
        while not self.has_global_pos:
            self.rate.sleep()

    #
    # Test method
    #
    def test_attctl(self):
        """Test offboard attitude control"""

        self.wait_until_ready()

        # set some attitude and thrust
        att = AttitudeTarget()
        att.header = Header()
        att.header.frame_id = "base_footprint"
        att.header.stamp = rospy.Time.now()
        att.orientation = Quaternion(*quaternion_from_euler(0.25, 0.25, 0))
        att.body_rate = Vector3()
        att.thrust = 0.7
        att.type_mask = 7

        # send some setpoints before starting
        for i in xrange(20):
            att.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(att)
            self.rate.sleep()

        rospy.loginfo("set mission mode and arm")
        while self.state.mode != "OFFBOARD" or not self.state.armed:
            if self.state.mode != "OFFBOARD":
                try:
                    self.set_mode_srv(0, 'OFFBOARD')
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            if not self.state.armed:
                try:
                    self.set_arming_srv(True)
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            rospy.sleep(2)

        rospy.loginfo("run mission")
        # does it cross expected boundaries in X seconds?
        timeout = 120
        crossed = False
        for count in xrange(timeout):
            # update timestamp for each published SP
            att.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(att)

            if (self.local_position.pose.position.x > 5 and
                    self.local_position.pose.position.z > 5 and
                    self.local_position.pose.position.y < -5):
                rospy.loginfo("boundary crossed | count {0}".format(count))
                crossed = True
                break

            self.rate.sleep()

        self.assertTrue(crossed, (
            "took too long to cross boundaries | x: {0}, y: {1}, z: {2}, timeout: {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))

        if self.state.armed:
            try:
                self.set_arming_srv(False)
            except rospy.ServiceException as e:
                rospy.logerr(e)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)

    rostest.rosrun(PKG, 'mavros_offboard_attctl_test',
                   MavrosOffboardAttctlTest)
