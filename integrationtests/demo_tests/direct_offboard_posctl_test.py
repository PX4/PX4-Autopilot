#!/usr/bin/env python
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


class OffboardPosctlTest(unittest.TestCase):

	def setUp(self):
		rospy.init_node('test_node', anonymous=True)
		rospy.Subscriber('px4_multicopter/vehicle_control_mode', vehicle_control_mode, self.vehicle_control_mode_callback)
		rospy.Subscriber("px4_multicopter/vehicle_local_position", vehicle_local_position, self.position_callback)
		self.pubSpt = rospy.Publisher('px4_multicopter/position_setpoint_triplet', position_setpoint_triplet, queue_size=10)
		self.rate = rospy.Rate(10) # 10hz

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
	# Test offboard POSCTL
	#
	def test_posctl(self):
		manIn = ManualInput()

		# arm and go into offboard
		manIn.arm()
		manIn.offboard()
		self.assertTrue(self.controlMode.flag_armed, "flag_armed is not set")
		self.assertTrue(self.controlMode.flag_control_offboard_enabled, "flag_control_offboard_enabled is not set")
		self.assertTrue(self.controlMode.flag_control_position_enabled, "flag_control_position_enabled is not set")
	
		# prepare flight path assertion
		fpa = FlightPathAssertion(
			(
				(0,0,0),
				(2,2,-2),
				(2,-2,-2),
				(-2,-2,-2),
				(2,2,-2),
			), 0.5, 0)
		fpa.start()

		self.reach_position(2, 2, -2, 120)
		self.reach_position(2, -2, -2, 120)
		self.reach_position(-2, -2, -2, 120)
		self.reach_position(2, 2, -2, 120)
		
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
		fpa.stop()
	

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG, 'posctl_test', OffboardPosctlTest)
	#unittest.main()
