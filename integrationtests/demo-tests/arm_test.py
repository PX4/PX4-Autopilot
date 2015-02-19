#!/usr/bin/env python
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
