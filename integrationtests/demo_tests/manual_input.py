#!/usr/bin/env python
import sys
import rospy

from px4.msg import manual_control_setpoint
from mav_msgs.msg import CommandAttitudeThrust
from std_msgs.msg import Header

#
# Manual input control helper
#
# Note: this is not the way to do it. ATM it fakes input to iris/command/attitude because else
# the simulator does not instantiate our controller.
#
class ManualInput:

	def __init__(self):
		rospy.init_node('test_node', anonymous=True)
		self.pubMcsp = rospy.Publisher('px4_multicopter/manual_control_setpoint', manual_control_setpoint, queue_size=10)
		self.pubAtt = rospy.Publisher('iris/command/attitude', CommandAttitudeThrust, queue_size=10)

	def arm(self):
		rate = rospy.Rate(10) # 10hz

		att = CommandAttitudeThrust()
		att.header = Header()

		pos = manual_control_setpoint()
		pos.x = 0
		pos.z = 0
		pos.y = 0
		pos.r = 0
		pos.mode_switch = 3
		pos.return_switch = 3
		pos.posctl_switch = 3
		pos.loiter_switch = 3
		pos.acro_switch = 0
		pos.offboard_switch = 3

		count = 0
		while not rospy.is_shutdown() and count < 10:
			rospy.loginfo("zeroing")
			time = rospy.get_rostime().now()
			pos.timestamp = time.secs * 1e6 + time.nsecs / 1000
			self.pubMcsp.publish(pos)
			# Fake input to iris commander
			self.pubAtt.publish(att)
			rate.sleep()
			count = count + 1 

		pos.r = 1
		count = 0
		while not rospy.is_shutdown() and count < 10:
			rospy.loginfo("arming")
			time = rospy.get_rostime().now()
			pos.timestamp = time.secs * 1e6 + time.nsecs / 1000
			self.pubMcsp.publish(pos)
			rate.sleep()
			count = count + 1

	def posctl(self):
		rate = rospy.Rate(10) # 10hz

		# triggers posctl
		pos = manual_control_setpoint()
		pos.x = 0
		pos.z = 0
		pos.y = 0
		pos.r = 0
		pos.mode_switch = 2
		pos.return_switch = 3
		pos.posctl_switch = 1
		pos.loiter_switch = 3
		pos.acro_switch = 0
		pos.offboard_switch = 3

		count = 0
		while not rospy.is_shutdown() and count < 10:
			rospy.loginfo("triggering posctl")
			time = rospy.get_rostime().now()
			pos.timestamp = time.secs * 1e6 + time.nsecs / 1000
			self.pubMcsp.publish(pos)
			rate.sleep()
			count = count + 1

	def offboard(self):
		rate = rospy.Rate(10) # 10hz

		# triggers posctl
		pos = manual_control_setpoint()
		pos.x = 0
		pos.z = 0
		pos.y = 0
		pos.r = 0
		pos.mode_switch = 3
		pos.return_switch = 3
		pos.posctl_switch = 3
		pos.loiter_switch = 3
		pos.acro_switch = 0
		pos.offboard_switch = 1

		count = 0
		while not rospy.is_shutdown() and count < 10:
			rospy.loginfo("triggering posctl")
			time = rospy.get_rostime().now()
			pos.timestamp = time.secs * 1e6 + time.nsecs / 1000
			self.pubMcsp.publish(pos)
			rate.sleep()
			count = count + 1

