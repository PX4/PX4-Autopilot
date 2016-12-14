"""
Contains some defintions and helper functions
"""

# pylint: disable=invalid-name

import numpy as np

BETA_TABLE = [
    0,
    8.82050518214,
    12.094592431,
    13.9876612368,
    16.0875642296,
    17.8797700658,
    19.6465647819,
    21.3802576894,
    23.0806434845,
    24.6673803845,
    26.1487953661,
    27.6350821245,
    29.6565383703,
    31.2211113844,
    32.7673547211,
    34.2967756977,
    35.6906782236,
    37.0724753352,
    38.4549693067,
    39.836592699,
    ]

params = {
    'GYRO': 5e-3,  # rad/s / sqrt(Hz)
    'BARO_Z': 1.0,  # m / sqrt(Hz)
    'GPS_XY': 0.3,  # m / sqrt(Hz)
    'GPS_Z': 0.3,  # m / sqrt(Hz)
    'GPS_VXY': 0.03,  # m/s / sqrt(Hz)
    'GPS_VZ': 0.03,  # m/s / sqrt(Hz)
    'LDR_Z': 0.02,  # m / sqrt(Hz)
    'ACC': 5e-2,  # m/s^2 / sqrt(Hz)
    'ACC_C': 1,  # m/s^2 / sqrt(Hz), weight for correction of acc dir
    'MAG_NE': 1e-1,  # 1 / sqrt(Hz)
    'MAG_D': 1e6,  # 1 / sqrt(Hz), large value prevents roll/pitch correction
}


class X(object):
    """
    State enumeration, doesn't line up with state space due to
    using quaternion instead of infinitesimal euler angles like error state
    """
    q_nb_0 = 0
    q_nb_1 = 1
    q_nb_2 = 2
    q_nb_3 = 3
    vel_N = 4
    vel_E = 5
    vel_D = 6
    gyro_bias_bx = 7
    gyro_bias_by = 8
    gyro_bias_bz = 9
    accel_scale = 10
    pos_N = 11
    pos_E = 12
    pos_D = 13
    terrain_alt = 14
    baro_bias = 15
    n = 16


class Xe(object):
    """
    State error enum, used for state-space for kalman filter
    """
    rot_bx = 0
    rot_by = 1
    rot_bz = 2
    vel_N = 3
    vel_E = 4
    vel_D = 5
    gyro_bias_bx = 6
    gyro_bias_by = 7
    gyro_bias_bz = 8
    accel_scale = 9
    pos_N = 10
    pos_E = 11
    pos_D = 12
    terrain_alt = 13
    baro_bias = 14
    n = 15


class U(object):
    """
    Input (accel and gyro measurements)
    """
    omega_nb_bx = 0
    omega_nb_by = 1
    omega_nb_bz = 2
    a_bx = 3
    a_by = 4
    a_bz = 5
    n = 6


class Y_accel(object):
    """
    Acceleratoin measurement in body frame
    """
    bx = 0
    by = 1
    bz = 2
    n = 3


class Y_gps(object):
    """
    GPS measurement
    """
    pos_N = 0
    pos_E = 1
    pos_D = 2
    vel_N = 3
    vel_E = 4
    vel_D = 5
    n = 6


class Y_baro(object):
    """
    Y baro
    """
    asl = 0
    n = 1


class Y_mag(object):
    """
    Magnetometer measurement
    """
    bx = 0
    by = 1
    bz = 2
    n = 3


class Y_lidar(object):
    """
    Lidar measurement
    """
    d = 0
    n = 1


class Timer(object):
    """
    Event timer
    """

    def __init__(self, t0, period):
        self.t0 = t0
        self.period = period

    def ready(self, t):
        """
        Returns if timer is ready
        """
        if t - self.t0 > self.period:
            self.t0 = t
            return True
        else:
            return False


def skew(v):
    """
    Return skew (cross-product) matrix
    """
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]])


def vect2quat(v):
    """
    Converts a vector to a quaternion with q0 = 0
    """
    return np.hstack([[0], v])

#  vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 :
