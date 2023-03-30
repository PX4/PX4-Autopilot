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

def predict_covariance(
        state: VState,
        P: MState,
        d_vel: sf.V3,
        d_vel_var: sf.V3,
        d_ang: sf.V3,
        d_ang_var: sf.Scalar,
        dt: sf.Scalar
):
    g = sf.Symbol("g") # does not appear in the jacobians

    d_vel_b = sf.V3(state[State.d_vel_bx], state[State.d_vel_by], state[State.d_vel_bz])
    d_vel_true = d_vel - d_vel_b

    d_ang_b = sf.V3(state[State.d_ang_bx], state[State.d_ang_by], state[State.d_ang_bz])
    d_ang_true = d_ang - d_ang_b

    q = sf.V4(state[State.qw], state[State.qx], state[State.qy], state[State.qz])
    R_to_earth = quat_to_rot_simplified(q)
    v = sf.V3(state[State.vx], state[State.vy], state[State.vz])
    p = sf.V3(state[State.px], state[State.py], state[State.pz])

    q_new = quat_mult(q, sf.V4(1, 0.5 * d_ang_true[0],  0.5 * d_ang_true[1],  0.5 * d_ang_true[2]))
    v_new = v + R_to_earth * d_vel_true + sf.V3(0 ,0 ,g) * dt
    p_new = p + v * dt

    # Predicted state vector at time t + dt
    state_new = VState.block_matrix([[q_new], [v_new], [p_new], [sf.Matrix(state[State.d_ang_bx:State.n_states])]])

    # State propagation jacobian
    A = state_new.jacobian(state)
    G = state_new.jacobian(sf.V6.block_matrix([[d_vel], [d_ang]]))

    # Covariance propagation
    var_u = sf.Matrix.diag([d_vel_var[0], d_vel_var[1], d_vel_var[2], d_ang_var, d_ang_var, d_ang_var])
    P_new = A * P * A.T + G * var_u * G.T

    # Generate the equations for the lower triangular matrix and the diagonal only
    # Since the matrix is symmetric, the upper triangle does not need to be derived
    # and can simply be copied in the implementation
    for index in range(State.n_states):
        for j in range(State.n_states):
            if index > j:
                P_new[index,j] = 0

    return P_new

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

    K = P * H.T / sf.Max(innov_var, epsilon)

    return (H.T, K)

def predict_sideslip(
        state: VState,
        epsilon: sf.Scalar
) -> (sf.Scalar):

    vel_rel = sf.V3(state[State.vx] - state[State.wx], state[State.vy] - state[State.wy], state[State.vz])
    q_att = sf.V4(state[State.qw], state[State.qx], state[State.qy], state[State.qz])
    relative_wind_body = quat_to_rot(q_att).T * vel_rel

    # Small angle approximation of side slip model
    # Protect division by zero using epsilon
    sideslip_pred = add_epsilon_sign(relative_wind_body[1] / relative_wind_body[0], relative_wind_body[0], epsilon)

    return sideslip_pred

def compute_sideslip_innov_and_innov_var(
        state: VState,
        P: MState,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar, sf.Scalar):

    sideslip_pred = predict_sideslip(state, epsilon);

    innov = sideslip_pred - 0.0

    H = sf.V1(sideslip_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov, innov_var)

def compute_sideslip_h_and_k(
        state: VState,
        P: MState,
        innov_var: sf.Scalar,
        epsilon: sf.Scalar
) -> (VState, VState):

    sideslip_pred = predict_sideslip(state, epsilon);

    H = sf.V1(sideslip_pred).jacobian(state)

    K = P * H.T / sf.Max(innov_var, epsilon)

    return (H.T, K)

def predict_mag_body(state) -> sf.V3:
    q_att = sf.V4(state[State.qw], state[State.qx], state[State.qy], state[State.qz])
    mag_field_earth = sf.V3(state[State.ix], state[State.iy], state[State.iz])
    mag_bias_body = sf.V3(state[State.ibx], state[State.iby], state[State.ibz])

    mag_body = quat_to_rot(q_att).T * mag_field_earth + mag_bias_body
    return mag_body

def compute_mag_innov_innov_var_and_hx(
        state: VState,
        P: MState,
        meas: sf.V3,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.V3, sf.V3, VState):

    meas_pred = predict_mag_body(state);

    innov = meas_pred - meas

    innov_var = sf.V3()
    Hx = sf.V1(meas_pred[0]).jacobian(state)
    innov_var[0] = (Hx * P * Hx.T + R)[0,0]
    Hy = sf.V1(meas_pred[1]).jacobian(state)
    innov_var[1] = (Hy * P * Hy.T + R)[0,0]
    Hz = sf.V1(meas_pred[2]).jacobian(state)
    innov_var[2] = (Hz * P * Hz.T + R)[0,0]

    return (innov, innov_var, Hx.T)

def compute_mag_y_innov_var_and_h(
        state: VState,
        P: MState,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VState):

    meas_pred = predict_mag_body(state);

    H = sf.V1(meas_pred[1]).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_mag_z_innov_var_and_h(
        state: VState,
        P: MState,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VState):

    meas_pred = predict_mag_body(state);

    H = sf.V1(meas_pred[2]).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_yaw_321_innov_var_and_h(
        state: VState,
        P: MState,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VState):

    q_att = sf.V4(state[State.qw], state[State.qx], state[State.qy], state[State.qz])
    R_to_earth = quat_to_rot(q_att)
    # Fix the singularity at pi/2 by inserting epsilon
    meas_pred = sf.atan2(R_to_earth[1,0], R_to_earth[0,0], epsilon=epsilon)

    H = sf.V1(meas_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_yaw_321_innov_var_and_h_alternate(
        state: VState,
        P: MState,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VState):

    q_att = sf.V4(state[State.qw], state[State.qx], state[State.qy], state[State.qz])
    R_to_earth = quat_to_rot(q_att)
    # Alternate form that has a singularity at yaw 0 instead of pi/2
    meas_pred = sf.pi/2 - sf.atan2(R_to_earth[0,0], R_to_earth[1,0], epsilon=epsilon)

    H = sf.V1(meas_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_yaw_312_innov_var_and_h(
        state: VState,
        P: MState,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VState):

    q_att = sf.V4(state[State.qw], state[State.qx], state[State.qy], state[State.qz])
    R_to_earth = quat_to_rot(q_att)
    # Alternate form to be used when close to pitch +-pi/2
    meas_pred = sf.atan2(-R_to_earth[0,1], R_to_earth[1,1], epsilon=epsilon)

    H = sf.V1(meas_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_yaw_312_innov_var_and_h_alternate(
        state: VState,
        P: MState,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VState):

    q_att = sf.V4(state[State.qw], state[State.qx], state[State.qy], state[State.qz])
    R_to_earth = quat_to_rot(q_att)
    # Alternate form to be used when close to pitch +-pi/2
    meas_pred = sf.pi/2 - sf.atan2(-R_to_earth[1,1], R_to_earth[0,1], epsilon=epsilon)

    H = sf.V1(meas_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_mag_declination_pred_innov_var_and_h(
        state: VState,
        P: MState,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar, VState):

    meas_pred = sf.atan2(state[State.iy], state[State.ix], epsilon=epsilon)

    H = sf.V1(meas_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (meas_pred, innov_var, H.T)

def predict_opt_flow(state, distance, epsilon):
    q_att = sf.V4(state[State.qw], state[State.qx], state[State.qy], state[State.qz])
    R_to_earth = quat_to_rot(q_att)
    R_to_body = R_to_earth.T

    # Calculate earth relative velocity in a non-rotating sensor frame
    v = sf.V3(state[State.vx], state[State.vy], state[State.vz])
    rel_vel_sensor = R_to_body * v

    # Divide by range to get predicted angular LOS rates relative to X and Y
    # axes. Note these are rates in a non-rotating sensor frame
    flow_pred = sf.V2()
    flow_pred[0] =  rel_vel_sensor[1] / distance
    flow_pred[1] = -rel_vel_sensor[0] / distance
    flow_pred = add_epsilon_sign(flow_pred, distance, epsilon)

    return flow_pred


def compute_flow_xy_innov_var_and_hx(
        state: VState,
        P: MState,
        distance: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.V2, VState):
    meas_pred = predict_opt_flow(state, distance, epsilon);

    innov_var = sf.V2()
    Hx = sf.V1(meas_pred[0]).jacobian(state)
    innov_var[0] = (Hx * P * Hx.T + R)[0,0]
    Hy = sf.V1(meas_pred[1]).jacobian(state)
    innov_var[1] = (Hy * P * Hy.T + R)[0,0]

    return (innov_var, Hx.T)

def compute_flow_y_innov_var_and_h(
        state: VState,
        P: MState,
        distance: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VState):
    meas_pred = predict_opt_flow(state, distance, epsilon);

    Hy = sf.V1(meas_pred[1]).jacobian(state)
    innov_var = (Hy * P * Hy.T + R)[0,0]

    return (innov_var, Hy.T)

def compute_gnss_yaw_pred_innov_var_and_h(
        state: VState,
        P: MState,
        antenna_yaw_offset: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar, VState):

    q_att = sf.V4(state[State.qw], state[State.qx], state[State.qy], state[State.qz])
    R_to_earth = quat_to_rot(q_att)

    # define antenna vector in body frame
    ant_vec_bf = sf.V3(sf.cos(antenna_yaw_offset), sf.sin(antenna_yaw_offset), 0)

    # rotate into earth frame
    ant_vec_ef = R_to_earth * ant_vec_bf

    # Calculate the yaw angle from the projection
    meas_pred = sf.atan2(ant_vec_ef[1], ant_vec_ef[0], epsilon=epsilon)

    H = sf.V1(meas_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (meas_pred, innov_var, H.T)

def predict_drag(
        state: VState,
        rho: sf.Scalar,
        cd: sf.Scalar,
        cm: sf.Scalar,
        epsilon: sf.Scalar
        ):
    q_att = sf.V4(state[State.qw], state[State.qx], state[State.qy], state[State.qz])
    R_to_earth = quat_to_rot(q_att)
    R_to_body = R_to_earth.T

    vel_rel = sf.V3(state[State.vx] - state[State.wx],
                    state[State.vy] - state[State.wy],
                    state[State.vz])
    vel_rel_body = R_to_body * vel_rel

    bluff_body_drag = -0.5 * rho * cd * sf.V2(vel_rel_body) * vel_rel_body.norm(epsilon=epsilon)
    momentum_drag = -cm * sf.V2(vel_rel_body)

    return bluff_body_drag + momentum_drag


def compute_drag_x_innov_var_and_k(
        state: VState,
        P: MState,
        rho: sf.Scalar,
        cd: sf.Scalar,
        cm: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar, VState):

    meas_pred = predict_drag(state, rho, cd, cm, epsilon)
    Hx = sf.V1(meas_pred[0]).jacobian(state)
    innov_var = (Hx * P * Hx.T + R)[0,0]
    Ktotal = P * Hx.T / sf.Max(innov_var, epsilon)
    K = VState()
    K[State.wx] = Ktotal[State.wx]
    K[State.wy] = Ktotal[State.wy]

    return (innov_var, K)

def compute_drag_y_innov_var_and_k(
        state: VState,
        P: MState,
        rho: sf.Scalar,
        cd: sf.Scalar,
        cm: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar, VState):

    meas_pred = predict_drag(state, rho, cd, cm, epsilon)
    Hy = sf.V1(meas_pred[1]).jacobian(state)
    innov_var = (Hy * P * Hy.T + R)[0,0]
    Ktotal = P * Hy.T / sf.Max(innov_var, epsilon)
    K = VState()
    K[State.wx] = Ktotal[State.wx]
    K[State.wy] = Ktotal[State.wy]

    return (innov_var, K)

def compute_gravity_innov_var_and_k_and_h(
        state: VState,
        P: MState,
        meas: sf.V3,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.V3, sf.V3, VState, VState, VState):

    # get transform from earth to body frame
    q_att = sf.V4(state[State.qw], state[State.qx], state[State.qy], state[State.qz])
    R_to_body = quat_to_rot(q_att).T

    # the innovation is the error between measured acceleration
    #  and predicted (body frame), assuming no body acceleration
    meas_pred = R_to_body * sf.Matrix([0,0,-9.80665])
    innov = meas_pred - 9.80665 * meas.normalized(epsilon=epsilon)

    # initialize outputs
    innov_var = sf.V3()
    K = [None] * 3

    # calculate observation jacobian (H), kalman gain (K), and innovation variance (S)
    #  for each axis
    for i in range(3):
        H = sf.V1(meas_pred[i]).jacobian(state)
        innov_var[i] = (H * P * H.T + R)[0,0]
        K[i] = P * H.T / innov_var[i]

    return (innov, innov_var, K[0], K[1], K[2])

print("Derive EKF2 equations...")
generate_px4_function(compute_airspeed_innov_and_innov_var, output_names=["innov", "innov_var"])
generate_px4_function(compute_airspeed_h_and_k, output_names=["H", "K"])

generate_px4_function(compute_sideslip_innov_and_innov_var, output_names=["innov", "innov_var"])
generate_px4_function(compute_sideslip_h_and_k, output_names=["H", "K"])
generate_px4_function(predict_covariance, output_names=["P_new"])
generate_px4_function(compute_mag_innov_innov_var_and_hx, output_names=["innov", "innov_var", "Hx"])
generate_px4_function(compute_mag_y_innov_var_and_h, output_names=["innov_var", "H"])
generate_px4_function(compute_mag_z_innov_var_and_h, output_names=["innov_var", "H"])
generate_px4_function(compute_yaw_321_innov_var_and_h, output_names=["innov_var", "H"])
generate_px4_function(compute_yaw_321_innov_var_and_h_alternate, output_names=["innov_var", "H"])
generate_px4_function(compute_yaw_312_innov_var_and_h, output_names=["innov_var", "H"])
generate_px4_function(compute_yaw_312_innov_var_and_h_alternate, output_names=["innov_var", "H"])
generate_px4_function(compute_mag_declination_pred_innov_var_and_h, output_names=["pred", "innov_var", "H"])
generate_px4_function(compute_flow_xy_innov_var_and_hx, output_names=["innov_var", "H"])
generate_px4_function(compute_flow_y_innov_var_and_h, output_names=["innov_var", "H"])
generate_px4_function(compute_gnss_yaw_pred_innov_var_and_h, output_names=["meas_pred", "innov_var", "H"])
generate_px4_function(compute_drag_x_innov_var_and_k, output_names=["innov_var", "K"])
generate_px4_function(compute_drag_y_innov_var_and_k, output_names=["innov_var", "K"])
generate_px4_function(compute_gravity_innov_var_and_k_and_h, output_names=["innov", "innov_var", "Kx", "Ky", "Kz"])
