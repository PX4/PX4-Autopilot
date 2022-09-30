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

File: derivation.py
Description:
"""

import symforce.symbolic as sf
from derivation_utils import *

class State:
    qw = 0
    qx = 1
    qy = 2
    qz = 3
    vx = 4
    vy = 5
    vz = 6
    px = 7
    py = 8
    pz = 9
    d_ang_bx = 10
    d_ang_by = 11
    d_ang_bz = 12
    d_vel_bx = 13
    d_vel_by = 14
    d_vel_bz = 15
    ix = 16
    iy = 17
    iz = 18
    ibx = 19
    iby = 20
    ibz = 21
    wx = 22
    wy = 23
    n_states = 24

class VState(sf.Matrix):
    SHAPE = (State.n_states, 1)

class MState(sf.Matrix):
    SHAPE = (State.n_states, State.n_states)

def compute_airspeed_innov_and_innov_var(
        state: VState,
        P: MState,
        airspeed: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar):

    vel_rel = sf.V3(state[State.vx] - state[State.wx], state[State.vy] - state[State.wy], state[State.vz])
    airspeed_pred = vel_rel.norm(epsilon=epsilon)

    innov = airspeed_pred - airspeed

    H = sf.V1(airspeed_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov, innov_var)

def compute_airspeed_h_and_k(
        state: VState,
        P: MState,
        innov_var: sf.Scalar,
        epsilon: sf.Scalar
) -> (VState, VState):

    vel_rel = sf.V3(state[State.vx] - state[State.wx], state[State.vy] - state[State.wy], state[State.vz])
    airspeed_pred = vel_rel.norm(epsilon=epsilon)
    H = sf.V1(airspeed_pred).jacobian(state)

    K = P * H.T / sm.Max(innov_var, epsilon)

    return (H.T, K)

generate_px4_function(compute_airspeed_innov_and_innov_var, output_names=["innov", "innov_var"])
generate_px4_function(compute_airspeed_h_and_k, output_names=["H", "K"])
