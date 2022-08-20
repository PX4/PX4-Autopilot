#! /usr/bin/env python3
"""
function collection for post-processing of ulog data.
"""

from typing import Tuple

import numpy as np

def get_gps_check_fail_flags(estimator_status: dict) -> dict:
    """
    :param estimator_status:
    :return:
    """
    gps_fail_flags = dict()

    # 0 : insufficient fix type (no 3D solution)
    # 1 : minimum required sat count fail
    # 2 : maximum allowed PDOP fail
    # 3 : maximum allowed horizontal position error fail
    # 4 : maximum allowed vertical position error fail
    # 5 : maximum allowed speed error fail
    # 6 : maximum allowed horizontal position drift fail
    # 7 : maximum allowed vertical position drift fail
    # 8 : maximum allowed horizontal speed fail
    # 9 : maximum allowed vertical velocity discrepancy fail
    gps_fail_flags['gfix_fail'] = ((2 ** 0 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['nsat_fail'] = ((2 ** 1 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['pdop_fail'] = ((2 ** 2 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['herr_fail'] = ((2 ** 3 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['verr_fail'] = ((2 ** 4 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['serr_fail'] = ((2 ** 5 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['hdrift_fail'] = ((2 ** 6 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['vdrift_fail'] = ((2 ** 7 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['hspd_fail'] = ((2 ** 8 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['veld_diff_fail'] = ((2 ** 9 & estimator_status['gps_check_fail_flags']) > 0) * 1
    return gps_fail_flags


def magnetic_field_estimates_from_states(estimator_states: dict) -> Tuple[float, float, float]:
    """

    :param estimator_states:
    :return:
    """
    rad2deg = 57.2958
    field_strength = np.sqrt(
        estimator_states['states[16]'] ** 2 + estimator_states['states[17]'] ** 2 +
        estimator_states['states[18]'] ** 2)
    declination = rad2deg * np.arctan2(estimator_states['states[17]'],
                                       estimator_states['states[16]'])
    inclination = rad2deg * np.arcsin(
        estimator_states['states[18]'] / np.maximum(field_strength, np.finfo(np.float32).eps))
    return declination, field_strength, inclination
