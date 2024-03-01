#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2022 PX4 Development Team
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
    3. Neither the name PX4 nor the names of its contributors may be
    used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

File: baro_static_pressure_compensation_tuning.py
Author: Mathieu Bresciani <mathieu@auterion.com>
License: BSD 3-Clause
Description:
    Tune the coefficients used to compensate for
    dynamic pressure disturbances on the barometer
    NOTE: this script currently assumes no wind.
"""

import matplotlib.pylab as plt
from pyulog import ULog
from pyulog.px4 import PX4ULog
import numpy as np
import quaternion
from scipy import optimize
from scipy.signal import detrend

def getAllData(logfile):
    log = ULog(logfile)

    v_local = np.matrix([getData(log, 'vehicle_local_position', 'vx'),
              getData(log, 'vehicle_local_position', 'vy'),
              getData(log, 'vehicle_local_position', 'vz')])

    t_local = ms2s(getData(log, 'vehicle_local_position', 'timestamp'))

    dist_bottom = getData(log, 'vehicle_local_position', 'dist_bottom')
    baro = getData(log, 'vehicle_air_data', 'baro_alt_meter')
    t_baro = ms2s(getData(log, 'vehicle_air_data', 'timestamp'))

    baro_bias = getData(log, 'estimator_baro_bias', 'bias')
    t_baro_bias = ms2s(getData(log, 'estimator_baro_bias', 'timestamp'))

    q = np.matrix([getData(log, 'vehicle_attitude', 'q[0]'),
              getData(log, 'vehicle_attitude', 'q[1]'),
              getData(log, 'vehicle_attitude', 'q[2]'),
              getData(log, 'vehicle_attitude', 'q[3]')])
    t_q = ms2s(getData(log, 'vehicle_attitude', 'timestamp'))

    gnss_h = getData(log, 'vehicle_gps_position', 'altitude_msl_m')
    t_gnss = ms2s(getData(log, 'vehicle_gps_position', 'timestamp'))

    (t_aligned, v_body_aligned, baro_aligned, v_local_z_aligned, gnss_h_aligned, baro_bias_aligned) = alignData(t_local, v_local, dist_bottom, t_q, q, baro, t_baro, t_gnss, gnss_h, t_baro_bias, baro_bias)

    t_aligned -= t_aligned[0]

    return (t_aligned, v_body_aligned, baro_aligned, v_local_z_aligned, gnss_h_aligned, baro_bias_aligned)

def alignData(t_local, v_local, dist_bottom, t_q, q, baro, t_baro, t_gnss, gnss_h, t_baro_bias, baro_bias):
    #TODO: use resample?
    len_q = len(t_q)
    len_l = len(t_local)
    len_g = len(t_gnss)
    len_bb = len(t_baro_bias)
    i_q = 0
    i_l = 0
    i_g = 0
    i_bb = 0
    v_body_aligned = np.empty((3,0))
    baro_aligned = []
    gnss_h_aligned = []
    v_local_z_aligned = []
    baro_bias_aligned = []
    t_aligned = []

    for i_b in range(len(t_baro)):
        t = t_baro[i_b]
        while t_local[i_l] < t and i_l < len_l-1:
            i_l += 1
        while t_q[i_q] < t and i_q < len_q-1:
            i_q += 1
        while t_gnss[i_g] < t and i_g < len_g-1:
            i_g += 1
        while t_baro_bias[i_bb] < t and i_bb < len_bb-1:
            i_bb += 1

        # Only use in air data
        if dist_bottom[i_l] < 1.0:
            continue

        qk = np.quaternion(q[0, i_q],q[1, i_q],q[2, i_q],q[3, i_q])
        q_vl = np.quaternion(0, v_local[0, i_l], v_local[1, i_l], v_local[2, i_l])
        q_vb = qk.conjugate() * q_vl * qk # Get velocity in body frame
        vb = quaternion.as_float_array(q_vb)[1:4]

        v_body_aligned = np.append(v_body_aligned, [[vb[0]], [vb[1]], [vb[2]]], axis=1)
        baro_aligned = np.append(baro_aligned, baro[i_b])
        v_local_z_aligned = np.append(v_local_z_aligned, v_local[2, i_l])
        gnss_h_aligned = np.append(gnss_h_aligned, gnss_h[i_g])
        baro_bias_aligned = np.append(baro_bias_aligned, baro_bias[i_bb])
        t_aligned.append(t)

    return (t_aligned, v_body_aligned, baro_aligned, v_local_z_aligned, gnss_h_aligned, baro_bias_aligned)

def getData(log, topic_name, variable_name, instance=0):
    variable_data = np.array([])
    for elem in log.data_list:
        if elem.name == topic_name:
            if instance == elem.multi_id:
                variable_data = elem.data[variable_name]
                break

    return variable_data

def ms2s(time_ms):
    return time_ms * 1e-6

def baroCorrected(x, v_body, baro):
    return baro + baroCorrection(x, v_body)

def baroCorrection(x, v_body):
    correction = []
    for i in range(len(v_body[0])):
        if v_body[0,i] < 0.0:
            kx = x[0]
        else:
            kx = x[1]

        if v_body[1,i] < 0.0:
            ky = x[2]
        else:
            ky = x[3]

        kz = x[4]

        correction.append((kx * v_body[0,i]**2 + ky * v_body[1,i]**2 + kz * v_body[2,i]**2) / 2.0)

    return correction

def run(logfile):
    (t, v_body, baro, v_local_z, gnss_h, baro_bias) = getAllData(logfile)

    # x[0]: pcoef_xn / g
    # x[1]: pcoef_xp / g
    # x[2]: pcoef_yn / g
    # x[3]: pcoef_yp / g
    # x[4]: pcoef_z / g
    baro -= baro_bias
    baro_error = detrend(gnss_h - baro)

    J = lambda x: np.sum(np.power(baro_error - baroCorrection(x, v_body), 2.0)) # cost function

    x0 = [0.0, 0.0, 0.0, 0.0, 0.0] # initial conditions
    res = optimize.minimize(J, x0, method='nelder-mead', options={'disp': True})

    # Convert results to parameters
    g = 9.80665
    pcoef_xn = res.x[0] * g
    pcoef_xp = res.x[1] * g
    pcoef_yn = res.x[2] * g
    pcoef_yp = res.x[3] * g
    pcoef_z = res.x[4] * g

    print(f"param set EKF2_PCOEF_XN {pcoef_xn:.3f}")
    print(f"param set EKF2_PCOEF_XP {pcoef_xp:.3f}")
    print(f"param set EKF2_PCOEF_YN {pcoef_yn:.3f}")
    print(f"param set EKF2_PCOEF_YP {pcoef_yp:.3f}")
    print(f"param set EKF2_PCOEF_Z {pcoef_z:.3f}")

    # Plot data
    plt.figure(1)
    plt.suptitle(f"Report of baro_static_pressure_compensation.py {logfile.split('/')[-1]}")
    ax1 = plt.subplot(3, 1, 1)
    ax1.set_title(f"PCoef_xn = {pcoef_xn:.3f}, PCoef_xp = {pcoef_xp:.3f}\nPCoef_yn = {pcoef_yn:.3f}, PCoef_yp = {pcoef_yp:.3f}, PCoef_z = {pcoef_z:.3f}")
    ax1.plot(t, baro-baro[0])
    ax1.plot(t, baroCorrected(res.x, v_body, baro)-baro[0])
    ax1.plot(t, gnss_h-gnss_h[0])
    ax1.set_ylabel("height (m)")
    ax1.legend(["baro_raw", "baro_corrected", "GNSS"])

    ax2 = plt.subplot(3, 1, 2, sharex=ax1)
    ax2.plot(t, baro_error)
    ax2.plot(t, baroCorrection(res.x, v_body))
    ax2.set_ylabel("height error (m)")
    ax2.legend(["GNSS-baro", "fitted model"])

    ax3 = plt.subplot(3, 1, 3, sharex=ax1)
    ax3.plot(t, v_body[0])
    ax3.plot(t, v_body[1])
    ax3.plot(t, v_body[2])
    ax3.set_xlabel("time (s)")
    ax3.set_ylabel("body velocity (m/s)")
    ax3.legend(["forward", "right", "down"])

    plt.show()

if __name__ == '__main__':
    import os
    import argparse

    # Get the path of this script (without file name)
    script_path = os.path.split(os.path.realpath(__file__))[0]

    # Parse arguments
    parser = argparse.ArgumentParser(
        description='Estimate the baro static pressure compensation coefficients using a ULog file')

    # Provide parameter file path and name
    parser.add_argument('logfile', help='Full ulog file path, name and extension', type=str)
    args = parser.parse_args()

    logfile = os.path.abspath(args.logfile) # Convert to absolute path

    run(logfile)
