#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2022-2024 PX4 Development Team
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

import symforce
symforce.set_epsilon_to_symbol()

import symforce.symbolic as sf
from symforce.values import Values

# generate_px4_function from derivation_utils in EKF/ekf_derivation/utils
import os, sys
derivation_utils_dir = os.path.dirname(os.path.abspath(__file__)) + "/../../python/ekf_derivation/utils"
sys.path.append(derivation_utils_dir)
import derivation_utils

State = Values(
    vel = sf.V2(),
    R = sf.Rot2() # 2D rotation to handle angle wrap
)

class VTangent(sf.Matrix):
    SHAPE = (State.tangent_dim(), 1)

class MTangent(sf.Matrix):
    SHAPE = (State.tangent_dim(), State.tangent_dim())

def rot2_small_angle(angle: sf.V1):
    # Approximation for small "delta angles" to avoid trigonometric functions
    return sf.Rot2(sf.Complex(1, angle[0]))

def yaw_est_predict_covariance(
        state: VTangent,
        P: MTangent,
        d_vel: sf.V2,
        d_vel_var: sf.Scalar,
        d_ang: sf.Scalar,
        d_ang_var: sf.Scalar,
):
    state = State.from_tangent(state)
    d_ang = sf.V1(d_ang) # cast to vector to gain group properties (e.g.: to_tangent)

    state_error = Values(
        vel = sf.V2.symbolic("delta_vel"),
        yaw = sf.V1.symbolic("delta_yaw")
    )

    # True state kinematics
    state_t = Values(
        vel = state["vel"] + state_error["vel"],
        R = state["R"] * rot2_small_angle(state_error["yaw"])
    )

    noise = Values(
        d_vel = sf.V2.symbolic("a_n"),
        d_ang = sf.V1.symbolic("w_n"),
    )

    input_t = Values(
        d_vel = d_vel - noise["d_vel"],
        d_ang = d_ang - noise["d_ang"]
    )

    state_t_pred = Values(
        vel = state_t["vel"] + state_t["R"] * input_t["d_vel"],
        R = state_t["R"] * rot2_small_angle(input_t["d_ang"])
    )

    # Nominal state kinematics
    state_pred = Values(
        vel = state["vel"] + state["R"] * d_vel,
        R = state["R"] * rot2_small_angle(d_ang)
    )

    # Error state kinematics
    delta_rot = (state_pred["R"].inverse() * state_t_pred["R"])
    state_error_pred = Values(
        vel = state_t_pred["vel"] - state_pred["vel"],
        yaw = sf.simplify(delta_rot.z.imag) # small angle appriximation; use simplify to cancel R.T*R
    )

    zero_state_error = {state_error[key]: state_error[key].zero() for key in state_error.keys()}
    zero_noise = {noise[key]: noise[key].zero() for key in noise.keys()}

    # Calculate state transition matrix
    F = VTangent(state_error_pred.to_storage()).jacobian(state_error).subs(zero_state_error).subs(zero_noise)

    # Derive the covariance prediction equations
    # Error growth in the inertial solution is assumed to be driven by 'noise' in the delta angles and
    # velocities, after bias effects have been removed.

    # derive the control(disturbance) influence matrix from IMU noise to error-state noise
    G = VTangent(state_error_pred.to_storage()).jacobian(noise).subs(zero_state_error).subs(zero_noise)

    # derive the state error matrix
    var_u = sf.Matrix.diag([d_vel_var, d_vel_var, d_ang_var])

    Q = G * var_u * G.T

    P_new = F * P * F.T + Q

    # Generate the equations for the upper triangular matrix and the diagonal only
    # Since the matrix is symmetric, the lower triangle does not need to be derived
    # and can simply be copied in the implementation
    for index in range(State.tangent_dim()):
        for j in range(State.tangent_dim()):
            if index > j:
                P_new[index,j] = 0

    return P_new

def yaw_est_compute_measurement_update(
        P: MTangent,
        vel_obs_var: sf.Scalar,
        epsilon : sf.Scalar
):
    H = sf.Matrix([[1,0,0],
                   [0,1,0]])

    R = sf.Matrix([[vel_obs_var , 0],
                   [0 , vel_obs_var]])

    S = H * P * H.T + R
    S_det = S[0, 0] * S[1, 1] - S[1, 0] * S[0, 1]
    S_det_inv = derivation_utils.add_epsilon_sign(1 / S_det, S_det, epsilon)

    # Compute inverse using simple formula for 2x2 matrix and using protected division
    S_inv = sf.M22([[S[1, 1], -S[0, 1]], [-S[1, 0], S[0, 0]]]) * S_det_inv
    K = (P * H.T) * S_inv
    P_new = P - K * H * P

    # Generate the equations for the upper triangular matrix and the diagonal only
    # Since the matrix is symmetric, the lower triangle does not need to be derived
    # and can simply be copied in the implementation
    for index in range(State.tangent_dim()):
        for j in range(State.tangent_dim()):
            if index > j:
                P_new[index,j] = 0

    return (S_inv, S_det_inv, K, P_new)

print("Derive yaw estimator equations...")
derivation_utils.generate_px4_function(yaw_est_predict_covariance, output_names=["P_new"])
derivation_utils.generate_px4_function(yaw_est_compute_measurement_update, output_names=["S_inv", "S_det_inv", "K", "P_new"])
