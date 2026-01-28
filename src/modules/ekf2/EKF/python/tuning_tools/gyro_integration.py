#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2024 PX4 Development Team
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
"""

import matplotlib.pylab as plt
from pyulog import ULog
from pyulog.px4 import PX4ULog
import numpy as np
import sym
import symforce.symbolic as sf

def getAllData(logfile):
    log = ULog(logfile)

    gyro = np.matrix([getData(log, 'sensor_combined', 'gyro_rad[0]'),
              getData(log, 'sensor_combined', 'gyro_rad[1]'),
              getData(log, 'sensor_combined', 'gyro_rad[2]')])
    t_gyro = getTimestampsSeconds(log, 'sensor_combined')

    t_gyro -= t_gyro[0]

    q = np.matrix([getData(log, 'vehicle_attitude', 'q[0]'),
              getData(log, 'vehicle_attitude', 'q[1]'),
              getData(log, 'vehicle_attitude', 'q[2]'),
              getData(log, 'vehicle_attitude', 'q[3]')])
    t_q = getTimestampsSeconds(log, 'vehicle_attitude')
    t_q -= t_q[0]

    return (t_gyro, gyro, t_q, q)

def getData(log, topic_name, variable_name, instance=0):
    variable_data = np.array([])
    for elem in log.data_list:
        if elem.name == topic_name:
            if instance == elem.multi_id:
                variable_data = elem.data[variable_name]
                break

    return variable_data

def us2s(time_us):
    return time_us * 1e-6

def getTimestampsSeconds(log, topic_name, instance=0):
    return us2s(getData(log, topic_name, 'timestamp', instance))

def integrateAngularRate(t, angular_rate, rot_init=sym.Rot3()):
    R = rot_init
    roll = []
    pitch = []
    yaw = []
    t_prev = 0

    for i in range(len(t)):
        dt = t[i] - t_prev
        R = R * sym.Rot3.from_tangent(angular_rate[:, i] * dt)
        att = R.to_yaw_pitch_roll()
        yaw = np.append(yaw, att[0])
        pitch = np.append(pitch, att[1])
        roll = np.append(roll, att[2])

        t_prev = t[i]

    return (roll, pitch, yaw)

def quat2RollPitchYaw(t, q):
    roll = []
    pitch = []
    yaw = []

    for i in range(len(t)):
        vect = sf.V3(float(q[1, i]), float(q[2, i]), float(q[3, i]))
        quat = sf.Quaternion(w=q[0, i], xyz=vect)
        R = sf.Rot3(quat)
        att = R.to_yaw_pitch_roll()
        yaw = np.append(yaw, float(att[0].evalf()))
        pitch = np.append(pitch, float(att[1].evalf()))
        roll = np.append(roll, float(att[2].evalf()))

    return (roll, pitch, yaw)


def run(logfile):
    (t, gyro, t_q, q) = getAllData(logfile)

    (roll, pitch, yaw) = quat2RollPitchYaw(t_q, q)
    (roll_raw, pitch_raw, yaw_raw) = integrateAngularRate(t, gyro, rot_init=sym.Rot3.from_yaw_pitch_roll(yaw[0], pitch[0], roll[0]))

    # Plot data
    plt.figure(1)
    plt.suptitle(logfile.split('/')[-1])

    ax1 = plt.subplot(3, 1, 1)
    ax1.plot(t_q, np.rad2deg(roll), '-')
    ax1.plot(t, np.rad2deg(roll_raw), '--')
    ax1.set_ylabel("roll (deg)")
    ax1.legend(["estimated", "integrated"])
    ax1.grid()

    ax2 = plt.subplot(3, 1, 2, sharex=ax1)
    ax2.plot(t_q, np.rad2deg(pitch))
    ax2.plot(t, np.rad2deg(pitch_raw), '--')
    ax2.set_ylabel("pitch (deg)")
    ax2.legend(["estimated", "integrated"])
    ax2.grid()

    ax3 = plt.subplot(3, 1, 3, sharex=ax1)
    ax3.plot(t_q, np.rad2deg(yaw))
    ax3.plot(t, np.rad2deg(yaw_raw), '--')
    ax3.set_xlabel("time (s)")
    ax3.set_ylabel("yaw (deg)")
    ax3.legend(["estimated", "integrated"])
    ax3.grid()
    plt.show()

if __name__ == '__main__':
    import os
    import argparse

    script_path = os.path.split(os.path.realpath(__file__))[0]

    parser = argparse.ArgumentParser(
        description='Integrate angular velocity to attitude and compare it with attitude estimate')

    parser.add_argument('logfile', help='Full ulog file path, name and extension', type=str)
    args = parser.parse_args()

    logfile = os.path.abspath(args.logfile)

    run(logfile)
