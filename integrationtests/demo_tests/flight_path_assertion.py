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
import rospy
import threading

from px4.msg import vehicle_local_position
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from numpy import linalg
import numpy as np

#
# Helper to test if vehicle stays on expected flight path.
#
class FlightPathAssertion(threading.Thread):

    #
    # Arguments
    # - positions: tuple of tuples in the form (x, y, z, heading)
    #
    # TODO: yaw validation
    # TODO: fail main test thread
    #
    def __init__(self, positions, tunnelRadius=1, yaw_offset=0.2):
        threading.Thread.__init__(self)
        rospy.Subscriber("vehicle_local_position", vehicle_local_position, self.position_callback)
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        self.positions = positions
        self.tunnel_radius = tunnelRadius
        self.yaw_offset = yaw_offset
        self.has_pos = False
        self.should_stop = False
        self.center = positions[0]
        self.end_of_segment = False
        self.failed = False
        self.local_position = vehicle_local_position

    def position_callback(self, data):
        self.has_pos = True
        self.local_position = data

    def spawn_indicator(self):
        self.delete_model("indicator")
        xml = (
            "<?xml version='1.0'?>" +
            "<sdf version='1.4'>" +
                "<model name='indicator'>" +
                    "<static>true</static>" +
                    "<link name='link'>" +
                        "<visual name='visual'>" +
                            "<transparency>0.7</transparency>" +
                            "<geometry>" +
                                "<sphere>" +
                                    "<radius>%f</radius>" +
                                "</sphere>" +
                            "</geometry>" +
                            "<material>" +
                                "<ambient>1 0 0 0.5</ambient>" +
                                "<diffuse>1 0 0 0.5</diffuse>" +
                            "</material>" +
                        "</visual>" +
                    "</link>" +
                "</model>" +
            "</sdf>") % self.tunnel_radius

        self.spawn_model("indicator", xml, "", Pose(), "")

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
        self.set_model_state(state)

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
            self.end_of_segment = True
            return linalg.norm(pos - b)

        x = c1 / c2
        l = a + x * v
        self.center = l
        return linalg.norm(pos - l)
    
    def stop(self):
        self.should_stop = True

    def run(self):
        rate = rospy.Rate(10) # 10hz
        self.spawn_indicator()

        current = 0
        count = 0
        while not self.should_stop:
            if self.has_pos:
                # calculate distance to line segment between first two points
                # if distances > tunnel_radius
                #   exit with error
                # advance current pos if not on the line anymore or distance to next point < tunnel_radius
                # exit if current pos is now the last position

                self.position_indicator()

                pos = np.array((self.local_position.x,
                                self.local_position.y,
                                self.local_position.z))
                a_pos = np.array((self.positions[current][0],
                                self.positions[current][1],
                                self.positions[current][2]))
                b_pos = np.array((self.positions[current + 1][0],
                                self.positions[current + 1][1],
                                self.positions[current + 1][2]))

                dist = self.distance_to_line(a_pos, b_pos, pos)
                b_dist = linalg.norm(pos - b_pos)

                rospy.logdebug("distance to line: %f, distance to end: %f" % (dist, b_dist))

                if dist > self.tunnel_radius:
                    msg = "left tunnel at position (%f, %f, %f)" % (self.local_position.x, self.local_position.y, self.local_position.z)
                    rospy.logerr(msg)
                    self.failed = True
                    break

                if self.end_of_segment or b_dist < self.tunnel_radius:
                    rospy.loginfo("next segment")
                    self.end_of_segment = False
                    current = current + 1

                if current == len(self.positions) - 1:
                    rospy.loginfo("no more positions")
                    break

            rate.sleep()
            count = count + 1

            if count > 10 and not self.has_pos: # no position after 1 sec
                rospy.logerr("no position")
                self.failed = True
                break
