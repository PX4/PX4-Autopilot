#!/usr/bin/env python2
#***************************************************************************
#
#   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
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

# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.

PKG = 'px4'

import unittest
import rospy
import glob
import json
import math
import os
import px4tools
import sys
from mavros import mavlink
from mavros.mission import QGroundControlWP
from pymavlink import mavutil
from threading import Thread
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, ExtendedState, Mavlink, State, Waypoint
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush
from sensor_msgs.msg import NavSatFix


def get_last_log():
    try:
        log_path = os.environ['PX4_LOG_DIR']
    except KeyError:
        log_path = os.path.join(os.environ['HOME'],
                                '.ros/rootfs/fs/microsd/log')
    last_log_dir = sorted(glob.glob(os.path.join(log_path, '*')))[-1]
    last_log = sorted(glob.glob(os.path.join(last_log_dir, '*.ulg')))[-1]
    return last_log


def read_new_mission(f):
    d = json.load(f)
    current = True
    for wp in d['items']:
        yield Waypoint(
            is_current=current,
            frame=int(wp['frame']),
            command=int(wp['command']),
            param1=float(wp['param1']),
            param2=float(wp['param2']),
            param3=float(wp['param3']),
            param4=float(wp['param4']),
            x_lat=float(wp['coordinate'][0]),
            y_long=float(wp['coordinate'][1]),
            z_alt=float(wp['coordinate'][2]),
            autocontinue=bool(wp['autoContinue']))
        if current:
            current = False


class MavrosMissionTest(unittest.TestCase):
    """
    Run a mission
    """

    def setUp(self):
        self.rate = rospy.Rate(10)  # 10hz
        self.has_global_pos = False
        self.local_position = PoseStamped()
        self.global_position = NavSatFix()
        self.extended_state = ExtendedState()
        self.altitude = Altitude()
        self.state = State()
        self.mc_rad = 5
        self.fw_rad = 60
        self.fw_alt_rad = 10
        self.last_alt_d = None
        self.last_pos_d = None
        self.mission_name = ""

        # setup ROS topics and services
        rospy.wait_for_service('mavros/mission/push', 30)
        rospy.wait_for_service('mavros/cmd/arming', 30)
        rospy.wait_for_service('mavros/set_mode', 30)
        self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push',
                                              WaypointPush)
        self.set_arming_srv = rospy.ServiceProxy('/mavros/cmd/arming',
                                                 CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped,
                         self.position_callback)
        rospy.Subscriber('mavros/global_position/global', NavSatFix,
                         self.global_position_callback)
        rospy.Subscriber('mavros/extended_state', ExtendedState,
                         self.extended_state_callback)
        rospy.Subscriber('mavros/altitude', Altitude, self.altitude_callback)
        rospy.Subscriber('mavros/state', State, self.state_callback)
        self.mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)

        # need to simulate heartbeat to prevent datalink loss detection
        self.hb_mav_msg = mavutil.mavlink.MAVLink_heartbeat_message(
            mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
        self.hb_mav_msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
        self.hb_ros_msg = mavlink.convert_to_rosmsg(self.hb_mav_msg)
        self.hb_thread = Thread(target=self.send_heartbeat, args=())
        self.hb_thread.daemon = True
        self.hb_thread.start()

    def tearDown(self):
        pass

    #
    # General callback functions used in tests
    #
    def position_callback(self, data):
        self.local_position = data

    def global_position_callback(self, data):
        self.global_position = data

        if not self.has_global_pos:
            self.has_global_pos = True

    def extended_state_callback(self, data):
        prev_state = self.extended_state.vtol_state
        self.extended_state = data

        if (prev_state != self.extended_state.vtol_state):
            rospy.loginfo("VTOL state change: {0}".format(
                self.extended_state.vtol_state))

    def state_callback(self, data):
        self.state = data

    def altitude_callback(self, data):
        self.altitude = data

    #
    # Helper methods
    #
    def send_heartbeat(self):
        while not rospy.is_shutdown():
            self.mavlink_pub.publish(self.hb_ros_msg)
            try:
                rospy.sleep(0.5)
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, lat, lon, alt, xy_offset, z_offset):
        R = 6371000  # metres
        rlat1 = math.radians(lat)
        rlat2 = math.radians(self.global_position.latitude)

        rlat_d = math.radians(self.global_position.latitude - lat)
        rlon_d = math.radians(self.global_position.longitude - lon)

        a = (math.sin(rlat_d / 2) * math.sin(rlat_d / 2) + math.cos(rlat1) *
             math.cos(rlat2) * math.sin(rlon_d / 2) * math.sin(rlon_d / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        d = R * c
        alt_d = abs(alt - self.altitude.amsl)

        # remember best distances
        if not self.last_pos_d or self.last_pos_d > d:
            self.last_pos_d = d
        if not self.last_alt_d or self.last_alt_d > alt_d:
            self.last_alt_d = alt_d

        rospy.logdebug("d: {0}, alt_d: {1}".format(d, alt_d))
        return d < xy_offset and alt_d < z_offset

    def reach_position(self, lat, lon, alt, timeout, index):
        # reset best distances
        self.last_alt_d = None
        self.last_pos_d = None

        rospy.loginfo(
            "trying to reach waypoint | " +
            "lat: {0:13.9f}, lon: {1:13.9f}, alt: {2:6.2f}, timeout: {3}, index: {4}".
            format(lat, lon, alt, timeout, index))

        # does it reach the position in X seconds?
        reached = False
        for count in xrange(timeout):
            # use MC radius by default
            # FIXME: also check MAV_TYPE from system status, otherwise pure fixed-wing won't work
            xy_radius = self.mc_rad
            z_radius = self.mc_rad

            # use FW radius if in FW or in transition
            if (self.extended_state.vtol_state == ExtendedState.VTOL_STATE_FW
                    or self.extended_state.vtol_state ==
                    ExtendedState.VTOL_STATE_TRANSITION_TO_MC or
                    self.extended_state.vtol_state ==
                    ExtendedState.VTOL_STATE_TRANSITION_TO_FW):
                xy_radius = self.fw_rad
                z_radius = self.fw_alt_rad

            if self.is_at_position(lat, lon, alt, xy_radius, z_radius):
                reached = True
                rospy.loginfo(
                    "position reached | index: {0}, count: {1}, pos_d: {2}, alt_d: {3}".
                    format(index, count, self.last_pos_d, self.last_alt_d))
                break

            self.rate.sleep()

        vtol_state_string = "VTOL undefined"

        if (self.extended_state.vtol_state == ExtendedState.VTOL_STATE_MC):
            vtol_state_string = "VTOL MC"
        if (self.extended_state.vtol_state == ExtendedState.VTOL_STATE_FW):
            vtol_state_string = "VTOL FW"
        if (self.extended_state.vtol_state ==
                ExtendedState.VTOL_STATE_TRANSITION_TO_MC):
            vtol_state_string = "VTOL FW->MC"
        if (self.extended_state.vtol_state ==
                ExtendedState.VTOL_STATE_TRANSITION_TO_FW):
            vtol_state_string = "VTOL MC->FW"

        self.assertTrue(reached, (
            "({0}) took too long to get to position | lat: {1:13.9f}, lon: {2:13.9f}, alt: {3:6.2f}, xy off: {4}, z off: {5}, timeout: {6}, index: {7}, pos_d: {8}, alt_d: {9}, VTOL state: {10}".
            format(self.mission_name, lat, lon, alt, xy_radius, z_radius,
                   timeout, index, self.last_pos_d, self.last_alt_d,
                   vtol_state_string)))

    def wait_until_ready(self):
        """FIXME: hack to wait for simulation to be ready"""
        rospy.loginfo("waiting for global position")
        while not self.has_global_pos or math.isnan(
                self.altitude.amsl) or math.isnan(self.altitude.relative):
            self.rate.sleep()

    def wait_on_landing(self, timeout, index):
        """Wait for landed state"""
        rospy.loginfo("waiting for landing | timeout: {0}, index: {1}".format(
            timeout, index))
        landed = False
        for count in xrange(timeout):
            if self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                rospy.loginfo(
                    "landed | index: {0}, count: {1}".format(index, count))
                landed = True
                break

            self.rate.sleep()

        self.assertTrue(landed, (
            "({0}) landing not detected after landing WP | timeout: {1}, index: {2}".
            format(self.mission_name, timeout, index)))

    def wait_on_transition(self, transition, timeout, index):
        """Wait for VTOL transition"""

        rospy.loginfo(
            "waiting for VTOL transition | transition: {0}, timeout: {1}, index: {2}".
            format(transition, timeout, index))
        transitioned = False
        for count in xrange(timeout):
            # transition to MC
            if (transition == ExtendedState.VTOL_STATE_MC and
                    self.extended_state.vtol_state ==
                    ExtendedState.VTOL_STATE_MC):
                rospy.loginfo("transitioned | index: {0}, count: {1}".format(
                    index, count))
                transitioned = True
                break

            # transition to FW
            if (transition == ExtendedState.VTOL_STATE_FW and
                    self.extended_state.vtol_state ==
                    ExtendedState.VTOL_STATE_FW):
                rospy.loginfo("transitioned | index: {0}, count: {1}".format(
                    index, count))
                transitioned = True
                break

            self.rate.sleep()

        self.assertTrue(transitioned, (
            "({0}) transition not detected | timeout: {1}, index: {2}".format(
                self.mission_name, timeout, index)))

    #
    # Test method
    #
    def test_mission(self):
        """Test mission"""

        if len(sys.argv) < 2:
            self.fail("usage: mission_test.py mission_file")
            return

        self.mission_name = sys.argv[1]
        mission_file = os.path.dirname(
            os.path.realpath(__file__)) + "/" + sys.argv[1]

        rospy.loginfo("reading mission {0}".format(mission_file))
        wps = []
        with open(mission_file, 'r') as f:
            mission_ext = os.path.splitext(mission_file)[1]
            if mission_ext == '.mission':
                rospy.loginfo("new style mission file detected")
                for waypoint in read_new_mission(f):
                    wps.append(waypoint)
                    rospy.logdebug(waypoint)
            elif mission_ext == '.txt':
                rospy.loginfo("old style mission file detected")
                mission = QGroundControlWP()
                for waypoint in mission.read(f):
                    wps.append(waypoint)
                    rospy.logdebug(waypoint)
            else:
                raise IOError('unknown mission file extension', mission_ext)

        self.wait_until_ready()

        rospy.loginfo("send mission")
        result = False
        try:
            res = self.wp_push_srv(start_index=0, waypoints=wps)
            result = res.success
        except rospy.ServiceException as e:
            rospy.logerr(e)
        self.assertTrue(
            result,
            "({0}) mission could not be transfered".format(self.mission_name))

        rospy.loginfo("set mission mode and arm")
        while self.state.mode != "AUTO.MISSION" or not self.state.armed:
            if self.state.mode != "AUTO.MISSION":
                try:
                    res = self.set_mode_srv(0, 'AUTO.MISSION')
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            if not self.state.armed:
                try:
                    self.set_arming_srv(True)
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            rospy.sleep(2)

        rospy.loginfo("run mission")
        for index, waypoint in enumerate(wps):
            # only check position for waypoints where this makes sense
            if waypoint.frame == Waypoint.FRAME_GLOBAL_REL_ALT or waypoint.frame == Waypoint.FRAME_GLOBAL:
                alt = waypoint.z_alt
                if waypoint.frame == Waypoint.FRAME_GLOBAL_REL_ALT:
                    alt += self.altitude.amsl - self.altitude.relative

                self.reach_position(waypoint.x_lat, waypoint.y_long, alt, 600,
                                    index)

            # check if VTOL transition happens if applicable
            if waypoint.command == 84 or waypoint.command == 85 or waypoint.command == 3000:
                transition = waypoint.param1

                if waypoint.command == 84:  # VTOL takeoff implies transition to FW
                    transition = ExtendedState.VTOL_STATE_FW

                if waypoint.command == 85:  # VTOL takeoff implies transition to MC
                    transition = ExtendedState.VTOL_STATE_MC

                self.wait_on_transition(transition, 600, index)

            # after reaching position, wait for landing detection if applicable
            if waypoint.command == 85 or waypoint.command == 21:
                self.wait_on_landing(600, index)

        if self.state.armed:
            try:
                self.set_arming_srv(False)
            except rospy.ServiceException as e:
                rospy.logerr(e)

        rospy.loginfo("mission done, calculating performance metrics")
        last_log = get_last_log()
        rospy.loginfo("log file {0}".format(last_log))
        data = px4tools.ulog.read_ulog(last_log).concat(dt=0.1)
        data = px4tools.ulog.compute_data(data)
        res = px4tools.estimator_analysis(data, False)

        # enforce performance
        self.assertTrue(abs(res['roll_error_mean']) < 5.0, str(res))
        self.assertTrue(abs(res['pitch_error_mean']) < 5.0, str(res))
        self.assertTrue(abs(res['yaw_error_mean']) < 5.0, str(res))
        self.assertTrue(res['roll_error_std'] < 5.0, str(res))
        self.assertTrue(res['pitch_error_std'] < 5.0, str(res))
        self.assertTrue(res['yaw_error_std'] < 5.0, str(res))


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)

    name = "mavros_mission_test"
    if len(sys.argv) > 1:
        name += "-%s" % sys.argv[1]
    rostest.rosrun(PKG, name, MavrosMissionTest)
