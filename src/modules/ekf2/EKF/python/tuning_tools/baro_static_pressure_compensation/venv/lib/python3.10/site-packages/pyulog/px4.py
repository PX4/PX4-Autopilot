"""
PX4-specific ULog helper
"""
from __future__ import print_function
import numpy as np

__author__ = "Beat Kueng"


class PX4ULog(object):
    """
    This class contains PX4-specific ULog things (field names, etc.)
    """

    def __init__(self, ulog_object):
        """
        @param ulog_object: ULog instance
        """
        self._ulog = ulog_object

    def get_mav_type(self):
        """ return the MAV type as string from initial parameters """

        mav_type = self._ulog.initial_parameters.get('MAV_TYPE', None)
        return {0: 'Generic',
                1: 'Fixed Wing',
                2: 'Quadrotor',
                3: 'Coaxial helicopter',
                4: 'Normal helicopter with tail rotor',
                5: 'Ground installation',
                6: 'Ground Control Station',
                7: 'Airship, controlled',
                8: 'Free balloon, uncontrolled',
                9: 'Rocket',
                10: 'Ground Rover',
                11: 'Surface Vessel, Boat, Ship',
                12: 'Submarine',
                13: 'Hexarotor',
                14: 'Octorotor',
                15: 'Tricopter',
                16: 'Flapping wing',
                17: 'Kite',
                18: 'Onboard Companion Controller',
                19: 'Two-rotor VTOL (Tailsitter)',
                20: 'Quad-rotor VTOL (Tailsitter)',
                21: 'Tiltrotor VTOL',
                22: 'VTOL Standard', #VTOL reserved 2
                23: 'VTOL reserved 3',
                24: 'VTOL reserved 4',
                25: 'VTOL reserved 5',
                26: 'Onboard Gimbal',
                27: 'Onboard ADSB Peripheral'}.get(mav_type, 'unknown type')

    def get_estimator(self):
        """return the configured estimator as string from initial parameters"""

        mav_type = self._ulog.initial_parameters.get('MAV_TYPE', None)
        if mav_type == 1: # fixed wing always uses EKF2
            return 'EKF2'

        mc_est_group = self._ulog.initial_parameters.get('SYS_MC_EST_GROUP', 2)
        return {0: 'INAV',
                1: 'LPE',
                2: 'EKF2',
                3: 'Q'}.get(mc_est_group, 'unknown ({})'.format(mc_est_group))


    def add_roll_pitch_yaw(self):
        """ convenience method to add the fields 'roll', 'pitch', 'yaw' to the
        loaded data using the quaternion fields (does not update field_data).

        Messages are: 'vehicle_attitude.q' and 'vehicle_attitude_setpoint.q_d',
        'vehicle_attitude_groundtruth.q' and 'vehicle_vision_attitude.q' """

        self._add_roll_pitch_yaw_to_message('vehicle_attitude')
        self._add_roll_pitch_yaw_to_message('vehicle_vision_attitude')
        self._add_roll_pitch_yaw_to_message('vehicle_attitude_groundtruth')
        self._add_roll_pitch_yaw_to_message('vehicle_attitude_setpoint', '_d')


    def _add_roll_pitch_yaw_to_message(self, message_name, field_name_suffix=''):

        message_data_all = [elem for elem in self._ulog.data_list if elem.name == message_name]
        for message_data in message_data_all:
            q = [message_data.data['q'+field_name_suffix+'['+str(i)+']'] for i in range(4)]
            roll = np.arctan2(2.0 * (q[0] * q[1] + q[2] * q[3]),
                              1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))
            pitch = np.arcsin(2.0 * (q[0] * q[2] - q[3] * q[1]))
            yaw = np.arctan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                             1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))
            message_data.data['roll'+field_name_suffix] = roll
            message_data.data['pitch'+field_name_suffix] = pitch
            message_data.data['yaw'+field_name_suffix] = yaw


    def get_configured_rc_input_names(self, channel):
        """
        find all RC mappings to a given channel and return their names

        :param channel: input channel (0=first)
        :return: list of strings or None
        """
        ret_val = []
        for key in self._ulog.initial_parameters:
            param_val = self._ulog.initial_parameters[key]
            if key.startswith('RC_MAP_') and param_val == channel + 1:
                ret_val.append(key[7:].capitalize())

        if len(ret_val) > 0:
            return ret_val
        return None
