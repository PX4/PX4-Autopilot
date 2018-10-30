###############################################################################
#
# Copyright (c) 2017 Estimation and Control Library (ECL). All rights reserved.
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
# 3. Neither the name ECL nor the names of its contributors may be
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
###############################################################################

"""Test the sensor data sampling of the ecl's EKF

@author: Peter Dürr <Peter.Duerr@sony.com>
"""
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import pytest
from hypothesis import given
from hypothesis import strategies as st

from test_utils import ecl_EKF
from test_utils import float_array


@pytest.mark.parametrize("dt_usec, downsampling_factor", [
    (ecl_EKF.Ekf.FILTER_UPDATE_PERIOD_MS * 1000 // 3, 3),
    (ecl_EKF.Ekf.FILTER_UPDATE_PERIOD_MS * 1000 // 2, 2),
    (ecl_EKF.Ekf.FILTER_UPDATE_PERIOD_MS * 1000, 1),
])
@given(accel_x=st.floats(-5, 5),
       accel_y=st.floats(-5, 5),
       accel_z=st.floats(-5, 5))
def test_imu_input(dt_usec, downsampling_factor, accel_x, accel_y, accel_z):
    """Make sure the acceleration is updated correctly when there is no angular
    velocity (test with and without downsampling)

    Tests random accelerations in x, y, z directions (using the hypothesis
    decorator) with different update frequencies (using pytest's parametrize
    decorator)

    """
    time_usec = 100
    delta_ang = float_array([0, 0, 0])
    delta_vel = float_array([accel_x,
                             accel_y,
                             accel_z]) * dt_usec / 1e6

    ekf = ecl_EKF.Ekf()
    # Run to accumulate buffer (choose sample after downsampling)
    for _ in range(20 * downsampling_factor):
        time_usec += dt_usec
        ekf.set_imu_data(time_usec,
                         dt_usec,
                         dt_usec,
                         delta_ang,
                         delta_vel)

    imu_sample = ekf.get_imu_sample_delayed()
    assert imu_sample.delta_ang_x == pytest.approx(0.0, abs=1e-3)
    assert imu_sample.delta_ang_y == pytest.approx(0.0, abs=1e-3)
    assert imu_sample.delta_ang_z == pytest.approx(0.0, abs=1e-3)
    assert imu_sample.delta_vel_x == pytest.approx(
        accel_x * dt_usec * downsampling_factor / 1e6, abs=1e-3)
    assert imu_sample.delta_vel_y == pytest.approx(
        accel_y * dt_usec * downsampling_factor / 1e6, abs=1e-3)
    assert imu_sample.delta_vel_z == pytest.approx(
        accel_z * dt_usec * downsampling_factor / 1e6, abs=1e-3)


@pytest.mark.parametrize("dt_usec, expected_dt_usec", [
    (ecl_EKF.Ekf.FILTER_UPDATE_PERIOD_MS * 1000 // 3,
     ecl_EKF.Ekf.FILTER_UPDATE_PERIOD_MS * 1000),
    (ecl_EKF.Ekf.FILTER_UPDATE_PERIOD_MS * 1000 // 2,
     ecl_EKF.Ekf.FILTER_UPDATE_PERIOD_MS * 1000),
    (ecl_EKF.Ekf.FILTER_UPDATE_PERIOD_MS * 1000,
     ecl_EKF.Ekf.FILTER_UPDATE_PERIOD_MS * 1000),
    (ecl_EKF.Ekf.FILTER_UPDATE_PERIOD_MS * 1000 * 2,
     ecl_EKF.Ekf.FILTER_UPDATE_PERIOD_MS * 1000 * 2),
    (500 * 1000,
     500 * 1000)
])
def test_imu_sampling(dt_usec, expected_dt_usec):
    """Make sure the timing is updated correctly

    If the imu update is faster than the filter period, it should be
    downsampled, otherwise used as is.

    """
    time_usec = 0
    delta_ang = float_array([0, 0, 0])
    delta_vel = float_array([0, 0, 0])
    ekf = ecl_EKF.Ekf()
    for _ in range(100):
        time_usec += dt_usec
        ekf.set_imu_data(time_usec,
                         dt_usec,
                         dt_usec,
                         delta_ang,
                         delta_vel)

    imu_sample = ekf.get_imu_sample_delayed()
    assert imu_sample.delta_ang_dt == pytest.approx(
        expected_dt_usec / 1e6, abs=1e-5)
    assert imu_sample.delta_vel_dt == pytest.approx(
        expected_dt_usec / 1e6, abs=1e-5)
    # Make sure the timestamp of the last sample is a small positive multiple
    # of the period away from now
    assert (time_usec - imu_sample.time_us) >= 0
    assert (time_usec - imu_sample.time_us) / expected_dt_usec < 20
