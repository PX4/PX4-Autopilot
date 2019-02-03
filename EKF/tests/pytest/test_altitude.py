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

"""Test the altitude estimation of the ecl's EKF

@author: Peter Dürr <Peter.Duerr@sony.com>
"""
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

from collections import namedtuple

import numpy as np

import pytest
from hypothesis import given
from hypothesis import example
from hypothesis import strategies as st

from test_utils import ecl_EKF
from test_utils import update_sensors
from test_utils import float_array
from test_utils import initialized_ekf


@given(z_bias=st.floats(-0.2, 0.2))
@example(0.1)
@example(0.2)
def test_accel_z_bias_converges(z_bias):
    """Make sure the accelerometer bias in z-direction is estimated correctly
    """
    ekf = ecl_EKF.Ekf()

    time_usec = 1000
    dt_usec = ecl_EKF.Ekf.FILTER_UPDATE_PERIOD_MS * 1000

    # Run for a while
    n_samples = 10000

    # Prepare data collection for plotting
    if pytest.ENABLE_PLOTTING:  # pylint: disable=no-member
        plot_data = namedtuple('PlotData', ['time', 'accel_bias', 'altitude'])
        plot_data.time = np.array([i * dt_usec * 1e-6
                                   for i in range(n_samples)])
        plot_data.accel_bias = np.zeros(n_samples)
        plot_data.altitude = np.zeros(n_samples)

    # Simulation
    for i in range(n_samples):
        update_sensors(ekf,
                       time_usec,
                       dt_usec,
                       accel=float_array([0.0,
                                          0.0,
                                          -ecl_EKF.one_g + z_bias]))
        ekf.update()
        time_usec += dt_usec

        if pytest.ENABLE_PLOTTING:  # pylint: disable=no-member
            plot_data.altitude[i] = ekf.get_position()[2]
            plot_data.accel_bias[i] = ekf.get_accel_bias()[2]

    # Plot if necessary
    if pytest.ENABLE_PLOTTING:  # pylint: disable=no-member
        from plot_utils import figure
        with figure(params={'z_bias': z_bias},
                    subplots=(2, 1)) as (_, (ax1, ax2)):
            ax1.plot(plot_data.time,
                     plot_data.altitude,
                     label='Altitude Estimate')
            ax1.legend(loc='best')
            ax1.set_ylabel('Altitude $[m]$')
            ax2.plot(plot_data.time,
                     plot_data.accel_bias,
                     label='Accel Z Bias Estimate')
            ax2.axhline(z_bias, color='k', label='Accel Bias Value')
            ax2.legend(loc='best')
            ax2.set_ylabel('Bias $[m / s^2]$')
            ax2.set_xlabel('Time $[s]$')

    # Check assumptions
    converged_pos = ekf.get_position()
    converged_vel = ekf.get_velocity()
    converged_accel_bias = ekf.get_accel_bias()
    for i in range(3):
        assert converged_pos[i] == pytest.approx(0.0, abs=1e-2)
        assert converged_vel[i] == pytest.approx(0.0, abs=1e-2)
    for i in range(2):
        assert converged_accel_bias[i] == pytest.approx(0.0, abs=1e-3)
    assert converged_accel_bias[2] == pytest.approx(z_bias, abs=1e-2)


@given(altitude=st.floats(0.0, 10.0))
@example(0.1)
@example(1.0)
@example(5.0)
@example(10.0)
def test_converges_to_baro_altitude(altitude):
    """Make sure that the ekf converges to (arbitrary) baro altitudes

    Increase the altitude with a bang-bang acceleration profile to target
    altitude, then wait there for a while and make sure it converges
    """
    ekf = ecl_EKF.Ekf()

    time_usec = 1000
    dt_usec = ecl_EKF.Ekf.FILTER_UPDATE_PERIOD_MS * 1000

    # Run for a while
    n_samples = 10000

    # Compute smooth acceleration profile
    rampup_accel = altitude / (((n_samples // 2 // 2) * (dt_usec / 1e6))**2)

    # Prepare data collection for plotting
    if pytest.ENABLE_PLOTTING:  # pylint: disable=no-member
        plot_data = namedtuple('PlotData', ['time',
                                            'baro',
                                            'altitude',
                                            'accel_z_bias'])
        plot_data.time = np.array([i * dt_usec * 1e-6
                                   for i in range(n_samples)])
        plot_data.baro = np.zeros(n_samples)
        plot_data.accel_z_bias = np.zeros(n_samples)
        plot_data.altitude = np.zeros(n_samples)

    # Simulate
    current_state = namedtuple('State', ['alt', 'vel'])
    current_state.alt = 0.0
    current_state.vel = 0.0
    for i in range(n_samples // 2):
        update_sensors(ekf,
                       time_usec,
                       dt_usec,
                       accel=float_array([0,
                                          0,
                                          -ecl_EKF.one_g - rampup_accel
                                          if i < n_samples // 4
                                          else -ecl_EKF.one_g + rampup_accel]),
                       baro_data=current_state.alt)
        ekf.update()

        if pytest.ENABLE_PLOTTING:  # pylint: disable=no-member
            plot_data.baro[i] = current_state.alt
            plot_data.altitude[i] = -ekf.get_position()[2]
            plot_data.accel_z_bias[i] = ekf.get_accel_bias()[2]

        # Euler step
        current_state.vel += (dt_usec / 1e6) * (rampup_accel
                                                if i < n_samples // 4
                                                else -rampup_accel)
        current_state.alt += current_state.vel * dt_usec / 1e6
        time_usec += dt_usec

    # Stay at altitude
    for i in range(n_samples // 2):
        update_sensors(ekf, time_usec, dt_usec, baro_data=altitude)
        ekf.update()
        time_usec += dt_usec

        if pytest.ENABLE_PLOTTING:  # pylint: disable=no-member
            plot_data.baro[n_samples // 2 + i] = altitude
            plot_data.altitude[n_samples // 2 + i] = -ekf.get_position()[2]
            plot_data.accel_z_bias[
                n_samples // 2 + i] = ekf.get_accel_bias()[2]

    # Plot if necessary
    if pytest.ENABLE_PLOTTING:  # pylint: disable=no-member
        from plot_utils import figure
        with figure(params={'altitude': altitude},
                    subplots=(2, 1)) as (_, (ax1, ax2)):
            ax1.plot(plot_data.time,
                     plot_data.altitude,
                     label='Altitude Estimate')
            ax1.plot(plot_data.time,
                     plot_data.altitude,
                     label='Baro Input')
            ax1.legend(loc='best')
            ax1.set_ylabel('Altitude $[m]$')
            ax2.plot(plot_data.time,
                     plot_data.accel_z_bias,
                     label='Accel Z Bias Estimate')
            ax2.legend(loc='best')
            ax2.set_ylabel('Bias $[m / s^2]$')
            ax2.set_xlabel('Time $[s]$')
    # plt.plot(np.array(baro_vs_pos))
    # plt.show()
    # print(ekf.get_accel_bias())

    converged_pos = ekf.get_position()
    converged_vel = ekf.get_velocity()
    assert converged_pos[0] == pytest.approx(0.0, abs=1e-6)
    assert converged_pos[1] == pytest.approx(0.0, abs=1e-6)
    # Check for 10cm level accuracy
    assert converged_pos[2] == pytest.approx(-altitude, abs=1e-1)
    assert converged_vel[0] == pytest.approx(0.0, abs=1e-3)
    assert converged_vel[1] == pytest.approx(0.0, abs=1e-3)
    assert converged_vel[2] == pytest.approx(0.0, abs=1e-1)
