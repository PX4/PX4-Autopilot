#!/usr/bin/env python2
from __future__ import division

import unittest
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, \
                            WaypointList
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, \
                            WaypointPush
from sensor_msgs.msg import NavSatFix


class MavrosTestCommon(unittest.TestCase):
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
        1: 'VTOL MC-to-FW',
        2: 'VTOL FW-to-MC',
        3: 'VTOL MC',
        4: 'VTOL FW'
    }

    def __init__(self, *args):
        super(MavrosTestCommon, self).__init__(*args)

    def setUp(self):
        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.mission_wp = WaypointList()
        self.state = State()
        self.mav_type = None

        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
                'mission_wp', 'state'
            ]
        }

        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")
        self.get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
        self.set_arming_srv = rospy.ServiceProxy('/mavros/cmd/arming',
                                                 CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.wp_clear_srv = rospy.ServiceProxy('mavros/mission/clear',
                                               WaypointClear)
        self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push',
                                              WaypointPush)

        # ROS subscribers
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude,
                                        self.altitude_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state',
                                              ExtendedState,
                                              self.extended_state_callback)
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global',
                                               NavSatFix,
                                               self.global_position_callback)
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home',
                                             HomePosition,
                                             self.home_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)
        self.mission_wp_sub = rospy.Subscriber(
            'mavros/mission/waypoints', WaypointList, self.mission_wp_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.state_callback)

    #
    # Callback functions
    #
    def altitude_callback(self, data):
        self.altitude = data

        # amsl has been observed to be nan while other fields are valid
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

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

    def global_position_callback(self, data):
        self.global_position = data

        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True

    def home_position_callback(self, data):
        self.home_position = data

        if not self.sub_topics_ready['home_pos']:
            self.sub_topics_ready['home_pos'] = True

    def local_position_callback(self, data):
        self.local_position = data

        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def mission_wp_callback(self, data):
        if self.mission_wp.current_seq != data.current_seq:
            rospy.loginfo("current mission waypoint sequence updated: {0}".
                          format(data.current_seq))

        self.mission_wp = data

        if not self.sub_topics_ready['mission_wp']:
            self.sub_topics_ready['mission_wp'] = True

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

    #
    # Helper methods
    #
    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
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

    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
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

    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
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
                    "landed state confirmed | state: {0}, index: {1} | seconds: {2} of {3}".
                    format(
                        self.LAND_STATES.get(desired_landed_state), index, i /
                        loop_freq, timeout))
                break

            rate.sleep()

        self.assertTrue(landed_state_confirmed, (
            "landed state not detected | desired: {0}, current: {1} | index: {2}, timeout(seconds): {3}".
            format(
                self.LAND_STATES.get(desired_landed_state),
                self.LAND_STATES.get(self.extended_state.landed_state), index,
                timeout)))

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
            "transition not detected | index: {0} | timeout(seconds): {1}".
            format(index, timeout)))

    def clear_wps(self, timeout):
        """timeout(int): seconds"""
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_cleared = False
        for i in xrange(timeout * loop_freq):
            if not self.mission_wp.waypoints:
                wps_cleared = True
                rospy.loginfo("clear waypoints success | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.wp_clear_srv()
                    if not res.success:
                        rospy.logerr("failed to send waypoint clear command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            rate.sleep()

        self.assertTrue(wps_cleared, (
            "failed to clear waypoints | timeout(seconds): {0}".format(timeout)
        ))

    def send_wps(self, waypoints, timeout):
        """waypoints, timeout(int): seconds"""
        rospy.loginfo("sending mission waypoints")
        if self.mission_wp.waypoints:
            rospy.loginfo("FCU already has mission waypoints")

        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_sent = False
        wps_verified = False
        for i in xrange(timeout * loop_freq):
            if not wps_sent:
                try:
                    res = self.wp_push_srv(start_index=0, waypoints=waypoints)
                    wps_sent = res.success
                    if wps_sent:
                        rospy.loginfo("waypoints successfully transferred")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            else:
                if len(waypoints) == len(self.mission_wp.waypoints):
                    rospy.loginfo("number of waypoints transferred: {0}".
                                  format(len(waypoints)))
                    wps_verified = True

            if wps_sent and wps_verified:
                rospy.loginfo("send waypoints success | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            rate.sleep()

        self.assertTrue((
            wps_sent and wps_verified
        ), "mission could not be transferred and verified | timeout(seconds): {0}".
                        format(timeout))

    def wait_for_mav_type(self, timeout):
        """Wait for MAV_TYPE parameter, timeout(int): seconds"""
        rospy.loginfo("waiting for MAV_TYPE")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        res = False
        for i in xrange(timeout * loop_freq):
            try:
                res = self.get_param_srv('MAV_TYPE')
                if res.success:
                    self.mav_type = res.value.integer
                    rospy.loginfo(
                        "MAV_TYPE received | value: {0} | seconds: {1} of {2}".
                        format(self.mav_type, i / loop_freq, timeout))
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            rate.sleep()

        self.assertTrue(res.success, (
            "MAV_TYPE param get failed | timeout(seconds): {0}".format(timeout)
        ))
