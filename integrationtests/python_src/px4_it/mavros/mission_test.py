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
from __future__ import division

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
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, Mavlink, \
                            State, Waypoint
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
    # dictionaries correspond to mavros ExtendedState msg
    LAND_STATES = {
        0: 'UNDEFINED',
        1: 'ON_GROUND',
        2: 'IN_AIR',
        3: 'TAKEOFF',
        4: 'LANDING'
    }
    VTOL_STATES = {
        0: 'VTOL UNDEFINED',
        1: 'VTOL MC->FW',
        2: 'VTOL FW->MC',
        3: 'VTOL MC',
        4: 'VTOL FW'
    }

    def setUp(self):
        self.rate = rospy.Rate(10)  # 10hz
        self.has_global_pos = False
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
        self.sub_topics_ready = {
            key: False
            for key in ['global_pos', 'home_pos', 'ext_state', 'alt', 'state']
        }

        # setup ROS topics and services
        try:
            rospy.wait_for_service('mavros/mission/push', 30)
            rospy.wait_for_service('mavros/cmd/arming', 30)
            rospy.wait_for_service('mavros/set_mode', 30)
        except rospy.ROSException:
            self.fail("failed to connect to mavros services")

        self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push',
                                              WaypointPush)
        self.set_arming_srv = rospy.ServiceProxy('/mavros/cmd/arming',
                                                 CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global',
                                               NavSatFix,
                                               self.global_position_callback)
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home',
                                             HomePosition,
                                             self.home_position_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state',
                                              ExtendedState,
                                              self.extended_state_callback)
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude,
                                        self.altitude_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.state_callback)
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
    # Callback functions
    #
    def global_position_callback(self, data):
        self.global_position = data

        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True

    def home_position_callback(self, data):
        # this topic publishing seems to be a better indicator that the sim
        # is ready, it's not actually needed
        self.home_pos_sub.unregister()

        if not self.sub_topics_ready['home_pos']:
            self.sub_topics_ready['home_pos'] = True

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                self.VTOL_STATES.get(self.extended_state.vtol_state),
                self.VTOL_STATES.get(data.vtol_state)))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                self.LAND_STATES.get(self.extended_state.landed_state),
                self.LAND_STATES.get(data.landed_state)))

        self.extended_state = data

        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    def altitude_callback(self, data):
        self.altitude = data

        # amsl has been observed to be nan while other fields are valid
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    #
    # Helper methods
    #
    def send_heartbeat(self):
        rate = rospy.Rate(2)  # Hz
        while not rospy.is_shutdown():
            self.mavlink_pub.publish(self.hb_ros_msg)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        old_mode = self.state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo(
                    "set mode success | new mode: {0}, old mode: {1} | seconds: {2} of {3}".
                    format(mode, old_mode, i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            rate.sleep()

        self.assertTrue(mode_set, (
            "failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}".
            format(mode, old_mode, timeout)))

    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        old_arm = self.state.armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                rospy.loginfo(
                    "set arm success | new arm: {0}, old arm: {1} | seconds: {2} of {3}".
                    format(arm, old_arm, i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            rate.sleep()

        self.assertTrue(arm_set, (
            "failed to set arm | new arm: {0}, old arm: {1} | timeout(seconds): {2}".
            format(arm, old_arm, timeout)))

    def is_at_position(self, lat, lon, alt, xy_offset, z_offset):
        """alt(amsl), xy_offset, z_offset: meters"""
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
        """alt(amsl): meters, timeout(int): seconds"""
        # reset best distances
        self.last_alt_d = None
        self.last_pos_d = None

        rospy.loginfo(
            "trying to reach waypoint | lat: {0:13.9f}, lon: {1:13.9f}, alt: {2:6.2f}, index: {3}".
            format(lat, lon, alt, index))

        # does it reach the position in 'timeout' seconds?
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
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
                    "position reached | pos_d: {0:.2f}, alt_d: {1:.2f}, index: {2} | seconds: {3} of {4}".
                    format(self.last_pos_d, self.last_alt_d, index, i /
                           loop_freq, timeout))
                break

            rate.sleep()

        self.assertTrue(reached, (
            "({0}) took too long to get to position | lat: {1:13.9f}, lon: {2:13.9f}, alt: {3:6.2f}, xy off: {4}, z off: {5}, pos_d: {6:.2f}, alt_d: {7:.2f}, VTOL state: {8}, index: {9} | timeout(seconds): {10}".
            format(self.mission_name, lat, lon, alt, xy_radius, z_radius,
                   self.last_pos_d, self.last_alt_d,
                   self.VTOL_STATES.get(self.extended_state.vtol_state), index,
                   timeout)))

    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for simulation topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo("simulation topics ready | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            rate.sleep()

        self.assertTrue(simulation_ready, (
            "failed to hear from all subscribed simulation topics | topic ready flags: {0} | timeout(seconds): {1}".
            format(self.sub_topics_ready, timeout)))

    def wait_on_landed_state(self, desired_landed_state, timeout, index):
        rospy.loginfo(
            "waiting for landed state | state: {0}, index: {1}".format(
                self.LAND_STATES.get(desired_landed_state), index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in xrange(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo(
                    "landed state confirmed | state: {0}, index: {1}".format(
                        self.LAND_STATES.get(desired_landed_state), index))
                break

            rate.sleep()

        self.assertTrue(landed_state_confirmed, (
            "({0}) landed state not detected | desired: {1}, current: {2} | index: {3}, timeout(seconds): {4}".
            format(self.mission_name,
                   self.LAND_STATES.get(desired_landed_state),
                   self.LAND_STATES.get(self.extended_state.landed_state),
                   index, timeout)))

    def wait_on_transition(self, transition, timeout, index):
        """Wait for VTOL transition, timeout(int): seconds"""
        rospy.loginfo(
            "waiting for VTOL transition | transition: {0}, index: {1}".format(
                self.VTOL_STATES.get(transition), index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        transitioned = False
        for i in xrange(timeout * loop_freq):
            if transition == self.extended_state.vtol_state:
                rospy.loginfo(
                    "transitioned | index: {0} | seconds: {1} of {2}".format(
                        index, i / loop_freq, timeout))
                transitioned = True
                break

            rate.sleep()

        self.assertTrue(transitioned, (
            "({0}) transition not detected | index: {1} | timeout(seconds): {2}, ".
            format(self.mission_name, index, timeout)))

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

        # delay starting the mission
        self.wait_for_topics(30)

        # make sure the simulation is ready to start the mission
        self.wait_on_landed_state(ExtendedState.LANDED_STATE_ON_GROUND, 10, -1)

        rospy.loginfo("seting mission mode")
        self.set_mode("AUTO.MISSION", 5)
        rospy.loginfo("arming")
        self.set_arm(True, 5)

        rospy.loginfo("run mission")
        for index, waypoint in enumerate(wps):
            # only check position for waypoints where this makes sense
            if waypoint.frame == Waypoint.FRAME_GLOBAL_REL_ALT or waypoint.frame == Waypoint.FRAME_GLOBAL:
                alt = waypoint.z_alt
                if waypoint.frame == Waypoint.FRAME_GLOBAL_REL_ALT:
                    alt += self.altitude.amsl - self.altitude.relative

                self.reach_position(waypoint.x_lat, waypoint.y_long, alt, 60,
                                    index)

            # check if VTOL transition happens if applicable
            if waypoint.command == 84 or waypoint.command == 85 or waypoint.command == 3000:
                transition = waypoint.param1

                if waypoint.command == 84:  # VTOL takeoff implies transition to FW
                    transition = ExtendedState.VTOL_STATE_FW

                if waypoint.command == 85:  # VTOL takeoff implies transition to MC
                    transition = ExtendedState.VTOL_STATE_MC

                self.wait_on_transition(transition, 60, index)

            # after reaching position, wait for landing detection if applicable
            if waypoint.command == 85 or waypoint.command == 21:
                self.wait_on_landed_state(ExtendedState.LANDED_STATE_ON_GROUND,
                                          60, index)

        rospy.loginfo("disarming")
        self.set_arm(False, 5)

        rospy.loginfo("mission done, calculating performance metrics")
        last_log = get_last_log()
        rospy.loginfo("log file {0}".format(last_log))
        data = px4tools.read_ulog(last_log).concat(dt=0.1)
        data = px4tools.compute_data(data)
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
