#!/usr/bin/env python
PKG = 'px4'

import sys
import unittest
import rospy

from px4.msg import vehicle_local_position
from px4.msg import vehicle_control_mode
from px4.msg import actuator_armed
from px4.msg import position_setpoint_triplet
from px4.msg import position_setpoint
from sensor_msgs.msg import Joy
from std_msgs.msg import Header


class DemoTest(unittest.TestCase):

	##
	## General callback functions used in tests
	##
	def position_callback(self, data):
		self.hasPos = True
		self.localPosition = data

	def actuator_armed_callback(self, data):
		self.actuatorStatus = data

	def vehicle_control_mode_callback(self, data):
		self.controlMode = data

	##
	## Testing start position
	##
	def test_position_zero(self):
		self.hasPos = False
		rospy.init_node('test_node', anonymous=True)
		rospy.Subscriber("px4_multicopter/vehicle_local_position", vehicle_local_position, self.position_callback)
		rate = rospy.Rate(1)

		count = 0
		while(count < 10):
			if(self.hasPos):
				break
			count = count + 1
			rate.sleep()

		self.assertTrue(count < 10, "took to long to get position")
		self.assertTrue(self.localPosition.x <= 0.1, "x position greater 0.1")
		self.assertTrue(self.localPosition.y <= 0.1, "y position greater 0.1")

	##
	## Test arming
	##
	def test_arm(self):
		rospy.init_node('test_node', anonymous=True)
		pub = rospy.Publisher('px4_multicopter/joy', Joy, queue_size=10)
		pub2 = rospy.Publisher('iris/joy', Joy, queue_size=10)
		sub = rospy.Subscriber('px4_multicopter/actuator_armed', actuator_armed, self.actuator_armed_callback)
		rate = rospy.Rate(10) # 10hz

		msg = Joy()
		msg.header = Header()
		msg.buttons = [0, 0, 0, 0, 0]
		msg.axes = [-0.0, -0.0, 1.0, -0.0, -0.0, 0.0, 0.0] 
		count = 0
		while not rospy.is_shutdown() and count < 10:
			rospy.loginfo("zeroing")
			pub.publish(msg)
			pub2.publish(msg)
			rate.sleep()
			count = count + 1 

		msg.buttons = [0, 0, 0, 0, 0]
		msg.axes = [-1.0, -0.0, 1.0, -0.0, -0.0, 0.0, 0.0] 
		count = 0
		while not rospy.is_shutdown() and count < 10:
			rospy.loginfo("arming")
			pub.publish(msg)
			pub2.publish(msg)
			rate.sleep()
			count = count + 1

		self.assertEquals(self.actuatorStatus.armed, True, "not armed")

	##
	## Test POSCTL
	##
	def test_posctl(self):
		rospy.init_node('test_node', anonymous=True)
		pub = rospy.Publisher('px4_multicopter/joy', Joy, queue_size=10)
		pub2 = rospy.Publisher('iris/joy', Joy, queue_size=10)
		rospy.Subscriber('px4_multicopter/vehicle_control_mode', vehicle_control_mode, self.vehicle_control_mode_callback)
		pubSpt = rospy.Publisher('px4_multicopter/position_setpoint_triplet', position_setpoint_triplet, queue_size=10)
		rospy.Subscriber("px4_multicopter/vehicle_local_position", vehicle_local_position, self.position_callback)
		rate = rospy.Rate(10) # 10hz

		# triggers posctl
		msg = Joy()
		msg.header = Header()
		msg.buttons = [0, 0, 1, 0, 0]
		msg.axes = [-0.0, -0.0, 1.0, -0.0, -0.0, 0.0, 0.0] 
		count = 0
		while not rospy.is_shutdown() and count < 10:
			rospy.loginfo("triggering posctl")
			pub.publish(msg)
			pub2.publish(msg)
			rate.sleep()
			count = count + 1

		self.assertTrue(self.controlMode.flag_armed, "flag_armed is not set")
		self.assertTrue(self.controlMode.flag_control_offboard_enabled, "flag_control_offboard_enabled is not set")
		self.assertTrue(self.controlMode.flag_control_position_enabled, "flag_control_position_enabled is not set")
	
		# set a position
		pos = position_setpoint()
		pos.valid = True
		pos.x = -5
		pos.z = -5
		pos.y = 5
		pos.position_valid = True
		stp = position_setpoint_triplet()
		stp.current = pos
		pubSpt.publish(stp)

		count = 0
		while(count < 30):
			if(self.localPosition.z > -6 and self.localPosition.z < -4):
				break
			count = count + 1
			rate.sleep()

		self.assertTrue(count < 30, "took to long to get to position")
		

	

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG, 'demo_test', DemoTest)
