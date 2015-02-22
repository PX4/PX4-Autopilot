#!/usr/bin/env python
import sys
import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import Header


#
# Manual input control helper, fakes joystick input
# > needs to correspond to default mapping in manual_input node
#
class ManualInput:

	def __init__(self):
		rospy.init_node('test_node', anonymous=True)
		self.joyPx4 = rospy.Publisher('px4_multicopter/joy', Joy, queue_size=10)
		self.joyIris = rospy.Publisher('iris/joy', Joy, queue_size=10)

	def arm(self):
		rate = rospy.Rate(10) # 10hz

		msg = Joy()
		msg.header = Header()
		msg.buttons = [0, 0, 0, 0, 0]
		msg.axes = [-0.0, -0.0, 1.0, -0.0, -0.0, 0.0, 0.0] 
		count = 0
		while not rospy.is_shutdown() and count < 10:
			rospy.loginfo("zeroing")
			self.joyPx4.publish(msg)
			self.joyIris.publish(msg)
			rate.sleep()
			count = count + 1 

		msg.buttons = [0, 0, 0, 0, 0]
		msg.axes = [-1.0, -0.0, 1.0, -0.0, -0.0, 0.0, 0.0] 
		count = 0
		while not rospy.is_shutdown() and count < 10:
			rospy.loginfo("arming")
			self.joyPx4.publish(msg)
			self.joyIris.publish(msg)
			rate.sleep()
			count = count + 1

	def posctl(self):
		rate = rospy.Rate(10) # 10hz

		# triggers posctl
		msg = Joy()
		msg.header = Header()
		msg.buttons = [0, 0, 1, 0, 0]
		msg.axes = [-0.0, -0.0, 1.0, -0.0, -0.0, 0.0, 0.0] 
		count = 0
		while not rospy.is_shutdown() and count < 10:
			rospy.loginfo("triggering posctl")
			self.joyPx4.publish(msg)
			self.joyIris.publish(msg)
			rate.sleep()
			count = count + 1
