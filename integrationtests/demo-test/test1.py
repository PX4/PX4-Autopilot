#!/usr/bin/env python
PKG = 'px4'

import sys
import unittest
import rospy

from px4.msg import vehicle_local_position


class DemoTest(unittest.TestCase):

	def test_position_zero(self):
		def position_callback(data):
			hasPos = True

		hasPos = False
		rospy.init_node('test_node', anonymous=True)
		rospy.Subscriber("px4_multicopter/vehicle_local_position", vehicle_local_position, position_callback)
		rate = rospy.Rate(1)

		count = 0
		while(count < 10):
			if(hasPos):
				break
			count = count + 1
			rate.sleep()

		self.assertTrue(count < 10, "took to long to get position")

	

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG, 'test_bare_bones', DemoTest)
