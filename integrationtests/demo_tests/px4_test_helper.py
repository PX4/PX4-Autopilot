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

from px4.msg import vehicle_local_position
from px4.msg import vehicle_attitude_setpoint
from px4.msg import vehicle_attitude
from px4.msg import vehicle_local_position_setpoint

from threading import Condition

#
# Test helper
#
class PX4TestHelper(object):

    def __init__(self, test_name):
        self.test_name = test_name

    def setUp(self):
        self.condition = Condition()
        self.closed = False

        rospy.init_node('test_node', anonymous=True)
        self.bag = rosbag.Bag(self.test_name + '.bag', 'w', compression="lz4")

        self.sub_vlp = rospy.Subscriber("iris/vehicle_local_position",
                vehicle_local_position, self.vehicle_position_callback)
        self.sub_vasp = rospy.Subscriber("iris/vehicle_attitude_setpoint",
                vehicle_attitude_setpoint, self.vehicle_attitude_setpoint_callback)
        self.sub_va = rospy.Subscriber("iris/vehicle_attitude",
                vehicle_attitude, self.vehicle_attitude_callback)
        self.sub_vlps = rospy.Subscriber("iris/vehicle_local_position_setpoint",
                vehicle_local_position_setpoint, self.vehicle_local_position_setpoint_callback)


    def tearDown(self):
        try:
            self.condition.acquire()
            self.closed = True

            self.sub_vlp.unregister()
            self.sub_vasp.unregister()
            self.sub_va.unregister()
            self.sub_vlps.unregister()
            self.bag.close()

        finally:
            self.condition.release()

    def vehicle_position_callback(self, data):
        self.bag_write('px4/vehicle_local_position', data)

    def vehicle_attitude_setpoint_callback(self, data):
        self.bag_write('px4/vehicle_attitude_setpoint', data)

    def vehicle_attitude_callback(self, data):
        self.bag_write('px4/vehicle_attitude', data)

    def vehicle_local_position_setpoint_callback(self, data):
        self.bag_write('px4/vehicle_local_position_setpoint', data)

    def bag_write(self, topic, data):
        try:
            self.condition.acquire()
            if not self.closed:
                self.bag.write(topic, data)
            else:
                rospy.logwarn("Trying to write to bag but it's already closed")
        finally:
            self.condition.release()

