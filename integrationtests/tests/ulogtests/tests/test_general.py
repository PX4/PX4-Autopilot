"""
Tests that can be applied to real test-flights as well
as simulated test-flights
"""

from uloganalysis import attitudeanalysis as attanl
from uloganalysis import positionanalysis as posanl
from uloganalysis import ulogconv
from uloganalysis import loginfo
import pyulog
import os
import numpy as np
import pytest


def setup_dataframe(self, filepath, topics):
    # Check if any of the topics exist in the topics exists in the log file
    try:
        self.ulog = pyulog.ULog(filepath, topics)
    except:
        print("Not a single topic that is needed for this test exists in the provided ulog file. Abort test")
        assert False

    # Check for every topic separately if it exists in the log file
    for i in range(len(self.ulog.data_list)):
        if self.ulog.data_list[i].name in topics:
            idx = topics.index(self.ulog.data_list[i].name)
            topics.pop(idx)

    if len(topics) > 0:
        print("\033[93m" + "The following topics do not exist in the provided ulog file: " + "\033[0m")
        print(topics)
        pytest.skip("Skip this test because topics are missing")
    else:
        self.df = ulogconv.merge(ulogconv.createPandaDict(self.ulog))
    
    # return self


class TestAttitude:
    """
    Test Attitude related constraints
    """
    def test_tilt_desired(self, filepath):

        topics = [
            "vehicle_attitude",
            "vehicle_attitude_setpoint",
            "vehicle_status",
        ]

        setup_dataframe(self, filepath, topics)
    
        # During Manual / Stabilized and Altitude, the tilt threshdol should not exceed
        # MPC_MAN_TILT_MAX

        attanl.add_desired_tilt(self.df)
        man_tilt = (
            loginfo.get_param(self.ulog, "MPC_MAN_TILT_MAX", 0) * np.pi / 180
        )
        assert self.df[
            (
                (self.df.T_vehicle_status_0__F_nav_state == 0)
                | (self.df.T_vehicle_status_0__F_nav_state == 1)
            )
            & (
                self.df.T_vehicle_attitude_setpoint_0__NF_tilt_desired
                > man_tilt
            )
        ].empty


class TestRTLHeight:
    # The return to home height changes with the distance from home
    # mode was triggered
    # check the height above ground while the drone returns to home. compare it with 
    # the allowed maximum or minimum heights, until the drone has reached home and motors have been turned off

    def test_rtl(self, filepath):

        topics = [
            "vehicle_local_position",
            "vehicle_status",
        ]

        setup_dataframe(self, filepath, topics)

        # drone parameters: below rtl_min_dist, the drone follows different rules than outside of it.
        rtl_min_dist = (
            loginfo.get_param(self.ulog, "RTL_MIN_DIST", 0)
        )
        rtl_return_alt = (
            loginfo.get_param(self.ulog, "RTL_RETURN_ALT", 0)
        )
        rtl_cone_dist = (
            loginfo.get_param(self.ulog, "RTL_CONE_DIST", 0)
        )

        NAVIGATION_STATE_AUTO_RTL = 5           # see https://github.com/PX4/Firmware/blob/master/msg/vehicle_status.msg
        thresh = 1                              # Threshold for position inaccuracies, in meters

        posanl.add_horizontal_distance(self.df)

        # Run the test every time that RTL was triggered
        self.df['T_vehicle_status_0__F_nav_state_group2'] = (self.df.T_vehicle_status_0__F_nav_state != self.df.T_vehicle_status_0__F_nav_state.shift()).cumsum()
        state_group = self.df.groupby(['T_vehicle_status_0__F_nav_state_group2'])
        for g, d in state_group:
            # Check that RTL was actually triggered 
            # at least two consecutive T_vehicle_status_0__F_nav_state values have to 
            # be equal to NAVIGATION_STATE_AUTO_RTL in order to confirm that RTL has been triggered
            if d.T_vehicle_status_0__F_nav_state.count() > 1 and d.T_vehicle_status_0__F_nav_state[0] == NAVIGATION_STATE_AUTO_RTL:
                height_at_RTL = abs(d.T_vehicle_local_position_0__F_z[0])
                distance_at_RTL = d.T_vehicle_local_position_0__NF_abs_horizontal_dist[0]
                max_height_during_RTL = abs(max(d.T_vehicle_local_position_0__F_z))

                if rtl_cone_dist > 0:
                    # Drone should not rise higher than height defined by a cone (definition taken from rtl.cpp file in firmware)
                    max_height_within_RTL_MIN_DIST = 2 * distance_at_RTL
                else:
                    # If no cone is defined, drone should not rise at all within certain radius around home
                    max_height_within_RTL_MIN_DIST = height_at_RTL
                
                # check if a value of the z position after triggering RTL is larger than allowed value
                if (distance_at_RTL < rtl_min_dist) & (height_at_RTL < max_height_within_RTL_MIN_DIST):
                    assert max_height_during_RTL < max_height_within_RTL_MIN_DIST + thresh

                elif (distance_at_RTL < rtl_min_dist) & (height_at_RTL >= max_height_within_RTL_MIN_DIST): 
                    assert max_height_during_RTL < height_at_RTL + thresh

                elif (distance_at_RTL >= rtl_min_dist) & (height_at_RTL < rtl_return_alt): 
                    assert max_height_during_RTL < rtl_return_alt + thresh

                elif (distance_at_RTL >= rtl_min_dist) & (height_at_RTL > rtl_return_alt): 
                    assert max_height_during_RTL < height_at_RTL + thresh
                        


# class TestSomething:
#
    # def test_1(self, filepath):
    #     topics = [
    #         "topic1",
    #         "topic2",
    #     ]
    #    setup_dataframe(self, filepath)
#        assert True
#    def test_2(self, filepath):
    #     topics = [
    #         "topic1",
    #         "topic2",
    #     ]
        # setup_dataframe(self, filepath)
#        assert True