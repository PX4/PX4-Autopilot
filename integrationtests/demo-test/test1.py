#!/usr/bin/env python
PKG = 'px4'

import sys
import unittest
import rospy

from px4.msg import vehicle_local_position


class DemoTest(unittest.TestCase):

	def position_callback(self, data):
		print data
		self.hasPos = True

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

	

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG, 'demo_test', DemoTest)
