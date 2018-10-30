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

"""Utils for the Python-based tests for the ecl library

@author: Peter Dürr <Peter.Duerr@sony.com>
"""
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import numpy as np
import pytest

try:
    import ecl_EKF  # pylint: disable=import-error
except ImportError:
    print("ImportError: ecl library cannot be found."
          " Make sure to compile ecl with Python bindings "
          "(add -DPythonTests=1 to cmake invocation), "
          "and set the PYTHONPATH to your build directory.")
    exit(1)


def float_array(inp):
    """Convert input to an array of 32 bit floats
    """
    return np.array(inp, dtype=np.float32)


def update_sensors(ekf,  # pylint: disable=too-many-arguments
                   time_usec,
                   dt_usec,
                   accel=float_array([0.0, 0.0, -ecl_EKF.one_g]),
                   ang_vel=float_array([0.0, 0.0, 0.0]),
                   mag_data=float_array([1.0, 0.0, 0.0]),
                   baro_data=0.0):
    """Updates the sensors with inputs
    """
    ekf.set_imu_data(time_usec,
                     dt_usec,
                     dt_usec,
                     ang_vel * dt_usec / 1e6,
                     accel * dt_usec / 1e6)
    ekf.set_mag_data(time_usec,
                     mag_data)
    ekf.set_baro_data(time_usec,
                      baro_data)


@pytest.fixture
def initialized_ekf():
    """Provides an initialized ekf, ready to go
    """
    ekf = ecl_EKF.Ekf()

    time_usec = 1000
    dt_usec = 5000

    # Provide a few samples
    for _ in range(1000):
        update_sensors(ekf, time_usec, dt_usec)
        ekf.update()
        time_usec += dt_usec

    # Make sure the ekf updates as expected
    return ekf, time_usec
