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

File: derivation_yaw_estimator.py
Description:
"""

import symforce.symbolic as sf
from derivation_utils import *

class State:
    vx = 0
    vy = 1
    yaw = 2
    n_states = 3

class VState(sf.Matrix):
    SHAPE = (State.n_states, 1)

class MState(sf.Matrix):
    SHAPE = (State.n_states, State.n_states)

def yaw_est_predict_covariance(
        state: VState,
        P: MState,
        d_vel: sf.V2,
        d_vel_var: sf.Scalar,
        d_ang_var: sf.Scalar
):
    d_ang = sf.Symbol("d_ang") # does not appear in the jacobians

    # derive the body to nav direction transformation matrix
    Tbn = sf.Matrix([[sf.cos(state[State.yaw]) , -sf.sin(state[State.yaw])],
                [sf.sin(state[State.yaw]) ,  sf.cos(state[State.yaw])]])

    # attitude update equation
    yaw_new = state[State.yaw] + d_ang

    # velocity update equations
    v_new = sf.V2(state[State.vx], state[State.vy]) + Tbn * d_vel

    # Define vector of process equations
    state_new = VState.block_matrix([[v_new], [sf.V1(yaw_new)]])

    # Calculate state transition matrix
    F = state_new.jacobian(state)

    # Derive the covariance prediction equations
    # Error growth in the inertial solution is assumed to be driven by 'noise' in the delta angles and
    # velocities, after bias effects have been removed.

    # derive the control(disturbance) influence matrix from IMU noise to state noise
    G = state_new.jacobian(sf.V3.block_matrix([[d_vel], [sf.V1(d_ang)]]))

    # derive the state error matrix
    var_u = sf.Matrix.diag([d_vel_var, d_vel_var, d_ang_var])

    Q = G * var_u * G.T

    P_new = F * P * F.T + Q

    # Generate the equations for the upper triangular matrix and the diagonal only
    # Since the matrix is symmetric, the lower triangle does not need to be derived
    # and can simply be copied in the implementation
    for index in range(State.n_states):
        for j in range(State.n_states):
            if index > j:
                P_new[index,j] = 0

    return P_new

def yaw_est_compute_measurement_update(
        P: MState,
        vel_obs_var: sf.Scalar,
        epsilon : sf.Scalar
):
    H = sf.Matrix([[1,0,0],
                   [0,1,0]])

    R = sf.Matrix([[vel_obs_var , 0],
                   [0 , vel_obs_var]])

    S = H * P * H.T + R
    S_det = S[0, 0] * S[1, 1] - S[1, 0] * S[0, 1]
    S_det_inv = add_epsilon_sign(1 / S_det, S_det, epsilon)

    # Compute inverse using simple formula for 2x2 matrix and using protected division
    S_inv = sf.M22([[S[1, 1], -S[0, 1]], [-S[1, 0], S[0, 0]]]) * S_det_inv
    K = (P * H.T) * S_inv
    P_new = P - K * H * P

    # Generate the equations for the upper triangular matrix and the diagonal only
    # Since the matrix is symmetric, the lower triangle does not need to be derived
    # and can simply be copied in the implementation
    for index in range(State.n_states):
        for j in range(State.n_states):
            if index > j:
                P_new[index,j] = 0

    return (S_inv, S_det_inv, K, P_new)

print("Derive yaw estimator equations...")
generate_px4_function(yaw_est_predict_covariance, output_names=["P_new"])
generate_px4_function(yaw_est_compute_measurement_update, output_names=["S_inv", "S_det_inv", "K", "P_new"])
