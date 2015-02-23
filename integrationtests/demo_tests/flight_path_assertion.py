#!/usr/bin/env python
import sys
import rospy
import threading

from px4.msg import vehicle_local_position
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from numpy import linalg
import numpy as np
import math

#
# Helper to test if vehicle stays in expected flight path.
#
class FlightPathAssertion(threading.Thread):

    def __init__(self, positions, tunnelRadius = 1, yawOffset = 0.2):
        threading.Thread.__init__(self)
        rospy.Subscriber("px4_multicopter/vehicle_local_position", vehicle_local_position, self.position_callback)
        self.spawn = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        self.setModelState = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        self.positions = positions
        self.tunnelRadius = tunnelRadius
        self.yawOffset = yawOffset
        self.hasPos = False
        self.shouldStop = False
        self.center = positions[0]
        self.endOfSegment = False

    def position_callback(self, data):
        self.hasPos = True
        self.localPosition = data

    def spawn_indicator(self):
        xml = "<?xml version='1.0'?><sdf version='1.4'><model name='indicator'><static>true</static><link name='link'><visual name='visual'><transparency>0.7</transparency><geometry><sphere><radius>%f</radius></sphere></geometry><material><ambient>1 0 0 0.5</ambient><diffuse>1 0 0 0.5</diffuse></material></visual></link></model></sdf>" % self.tunnelRadius
        self.spawn("indicator", xml, "", Pose(), "")

    def position_indicator(self):
        state = SetModelState()
        state.model_name = "indicator"
        pose = Pose()
        pose.position.x = self.center[0]
        pose.position.y = (-1) * self.center[1]
        pose.position.z = (-1) * self.center[2]
        state.pose = pose
        state.twist = Twist()
        state.reference_frame = ""
        self.setModelState(state)

    def distance_to_line(self, a, b, pos):
        v = b - a
        w = pos - a

        c1 = np.dot(w, v)
        if c1 <= 0: # before a
            self.center = a
            return linalg.norm(pos - a)

        c2 = np.dot(v, v)
        if c2 <= c1: # after b
            self.center = b
            self.endOfSegment = True
            return linalg.norm(pos - b)

        x = c1 / c2
        l = a + x * v
        self.center = l
        return linalg.norm(pos - l)
    
    def stop(self):
        self.shouldStop = True

    def run(self):
        rate = rospy.Rate(10) # 10hz
        self.spawn_indicator()

        current = 0

        while not self.shouldStop:
            if (self.hasPos):
                # calculate distance to line segment between first two points
                # if distances > tunnelRadius
                #   exit with error
                # advance current pos if not on the line anymore or distance to next point < tunnelRadius
                # exit if current pos is now the last position

                self.position_indicator()

                pos = np.array((self.localPosition.x,
                    self.localPosition.y, 
                    self.localPosition.z))
                aPos = np.array((self.positions[current][0],
                    self.positions[current][1], 
                    self.positions[current][2]))
                bPos = np.array((self.positions[current + 1][0],
                    self.positions[current + 1][1], 
                    self.positions[current + 1][2]))

                dist = self.distance_to_line(aPos, bPos, pos)
                bDist = linalg.norm(pos - bPos)

                rospy.loginfo("distance to line: %f, distance to end: %f" % (dist, bDist))

                if (dist > self.tunnelRadius):
                    rospy.logerr("left tunnel at position (%f, %f, %f)" % (self.localPosition.x, self.localPosition.y, self.localPosition.z))
                    # FIXME: assertion
                    break

                if (self.endOfSegment or bDist < self.tunnelRadius):
                    rospy.loginfo("next segment")
                    self.endOfSegment = False
                    current = current + 1

                if (current == len(self.positions) - 1):
                    rospy.loginfo("no more positions")
                    break

            rate.sleep()
