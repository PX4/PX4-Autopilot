#! /usr/bin/env python3
"""
function collection for post-processing of ulog data.
"""

from typing import Tuple

import numpy as np

def get_gnss_failed_checks(vehicle_gnss: dict) -> dict:
    """
    :param vehicle_gnss: the selected receiver's samples
    :return: one array per check, 1 where the check failed
    """
    # vehicle_gnss CHECK_* bits
    check_bits = {
        'nsat_fail': 0,
        'pdop_fail': 1,
        'herr_fail': 2,
        'verr_fail': 3,
        'serr_fail': 4,
        'hdrift_fail': 5,
        'vdrift_fail': 6,
        'hspd_fail': 7,
        'veld_diff_fail': 8,
        'gfix_fail': 10,
    }

    return {name: ((2 ** bit & vehicle_gnss['failed_checks']) > 0) * 1 for name, bit in check_bits.items()}


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
