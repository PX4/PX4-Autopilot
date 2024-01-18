#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2022-2023 PX4 Development Team
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
    Derivation of an error-state EKF based on
    Sola, Joan. "Quaternion kinematics for the error-state Kalman filter." arXiv preprint arXiv:1711.02508 (2017).
    The derivation is directly done in discrete-time as this allows us to define the desired type of discretization
    for each element while defining the equations (easier than a continuous-time derivation followed by a block-wise discretization).
"""

import argparse

import symforce
symforce.set_epsilon_to_symbol()

import symforce.symbolic as sf
from symforce import typing as T
from symforce import ops
from symforce.values import Values

import sympy as sp
from derivation_utils import *

# Initialize parser
parser = argparse.ArgumentParser()

parser.add_argument("--disable_mag", action='store_true', help="disable mag")
parser.add_argument("--disable_wind", action='store_true', help="disable wind")

# Read arguments from command line
args = parser.parse_args()

# The state vector is organized in an ordered dictionary
State = Values(
    quat_nominal = sf.Rot3(),
    vel = sf.V3(),
    pos = sf.V3(),
    gyro_bias = sf.V3(),
    accel_bias = sf.V3(),
    mag_I = sf.V3(),
    mag_B = sf.V3(),
    wind_vel = sf.V2()
)

if args.disable_mag:
    del State["mag_I"]
    del State["mag_B"]

if args.disable_wind:
    del State["wind_vel"]

class IdxDof():
    def __init__(self, idx, dof):
        self.idx = idx
        self.dof = dof

def BuildTangentStateIndex():
    # Build a dictionary that can be used to access elements of vectors
    # and matrices defined in the state tangent space (e.g.: P, K and H)
    tangent_state_index = {}
    idx = 0
    for key in State.keys_recursive():
        dof = State[key].tangent_dim()
        tangent_state_index[key] = IdxDof(idx, dof)
        idx += dof
    return tangent_state_index

tangent_idx = BuildTangentStateIndex()

class VState(sf.Matrix):
    SHAPE = (State.storage_dim(), 1)

class VTangent(sf.Matrix):
    SHAPE = (State.tangent_dim(), 1)

class MTangent(sf.Matrix):
    SHAPE = (State.tangent_dim(), State.tangent_dim())

def vstate_to_state(v: VState):
    state = State.from_storage(v)
    q_px4 = state["quat_nominal"].to_storage()
    state["quat_nominal"] = sf.Rot3(sf.Quaternion(xyz=sf.V3(q_px4[1], q_px4[2], q_px4[3]), w=q_px4[0]))
    return state

def predict_covariance(
    state: VState,
    P: MTangent,
    accel: sf.V3,
    accel_var: sf.V3,
    gyro: sf.V3,
    gyro_var: sf.Scalar,
    dt: sf.Scalar
) -> MTangent:

    state = vstate_to_state(state)
    g = sf.Symbol("g") # does not appear in the jacobians

    state_error = Values(
        theta = sf.V3.symbolic("delta_theta"),
        vel = sf.V3.symbolic("delta_v"),
        pos = sf.V3.symbolic("delta_p"),
        gyro_bias = sf.V3.symbolic("delta_w_b"),
        accel_bias = sf.V3.symbolic("delta_a_b"),
        mag_I = sf.V3.symbolic("mag_I"),
        mag_B = sf.V3.symbolic("mag_B"),
        wind_vel = sf.V2.symbolic("wind_vel")
    )

    if args.disable_mag:
        del state_error["mag_I"]
        del state_error["mag_B"]

    if args.disable_wind:
        del state_error["wind_vel"]

    # True state kinematics
    state_t = Values()

    for key in state.keys():
        if key == "quat_nominal":
            # Create true quaternion using small angle approximation of the error rotation
            state_t["quat_nominal"] = state["quat_nominal"] * sf.Rot3(sf.Quaternion(xyz=(state_error["theta"] / 2), w=1))
        else:
            state_t[key] = state[key] + state_error[key]

    noise = Values(
        accel = sf.V3.symbolic("a_n"),
        gyro = sf.V3.symbolic("w_n"),
    )

    input_t = Values(
        accel = accel - state_t["accel_bias"] - noise["accel"],
        gyro = gyro - state_t["gyro_bias"] - noise["gyro"]
    )

    R_t = state_t["quat_nominal"]
    state_t_pred = state_t.copy()
    state_t_pred["quat_nominal"] = state_t["quat_nominal"] * sf.Rot3(sf.Quaternion(xyz=(input_t["gyro"] * dt / 2), w=1))
    state_t_pred["vel"] = state_t["vel"] + (R_t * input_t["accel"] + sf.V3(0, 0, g)) * dt
    state_t_pred["pos"] = state_t["pos"] + state_t["vel"] * dt

    # Nominal state kinematics
    input = Values(
        accel = accel - state["accel_bias"],
        gyro = gyro - state["gyro_bias"]
    )

    R = state["quat_nominal"]
    state_pred = state.copy()
    state_pred["quat_nominal"] = state["quat_nominal"] * sf.Rot3(sf.Quaternion(xyz=(input["gyro"] * dt / 2), w=1))
    state_pred["vel"] = state["vel"] + (R * input["accel"] + sf.V3(0, 0, g)) * dt
    state_pred["pos"] = state["pos"] + state["vel"] * dt

    # Error state kinematics
    state_error_pred = Values()
    for key in state_error.keys():
        if key == "theta":
            delta_q = sf.Quaternion.from_storage(state_pred["quat_nominal"].to_storage()).conj() * sf.Quaternion.from_storage(state_t_pred["quat_nominal"].to_storage())
            state_error_pred["theta"] = 2 * sf.V3(delta_q.x, delta_q.y, delta_q.z) # Use small angle approximation to obtain a simpler jacobian
        else:
            state_error_pred[key] = state_t_pred[key] - state_pred[key]

    # Simplify angular error state prediction
    for i in range(state_error_pred["theta"].storage_dim()):
        state_error_pred["theta"][i] = sp.expand(state_error_pred["theta"][i]).subs(dt**2, 0) # do not consider dt**2 effects in the derivation
        q_est = sf.Quaternion.from_storage(state["quat_nominal"].to_storage())
        state_error_pred["theta"][i] = sp.factor(state_error_pred["theta"][i]).subs(q_est.w**2 + q_est.x**2 + q_est.y**2 + q_est.z**2, 1) # unit norm quaternion

    zero_state_error = {state_error[key]: state_error[key].zero() for key in state_error.keys()}
    zero_noise = {noise[key]: noise[key].zero() for key in noise.keys()}

    # State propagation jacobian
    A = VTangent(state_error_pred.to_storage()).jacobian(state_error).subs(zero_state_error).subs(zero_noise)
    G = VTangent(state_error_pred.to_storage()).jacobian(noise).subs(zero_state_error).subs(zero_noise)

    # Covariance propagation
    var_u = sf.Matrix.diag([accel_var[0], accel_var[1], accel_var[2], gyro_var, gyro_var, gyro_var])
    P_new = A * P * A.T + G * var_u * G.T

    # Generate the equations for the upper triangular matrix and the diagonal only
    # Since the matrix is symmetric, the lower triangle does not need to be derived
    # and can simply be copied in the implementation
    for index in range(state.tangent_dim()):
        for j in range(state.tangent_dim()):
            if index > j:
                P_new[index,j] = 0

    return P_new

def compute_airspeed_innov_and_innov_var(
        state: VState,
        P: MTangent,
        airspeed: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar):

    state = vstate_to_state(state)
    wind = sf.V3(state["wind_vel"][0], state["wind_vel"][1], 0.0)
    vel_rel = state["vel"] - wind
    airspeed_pred = vel_rel.norm(epsilon=epsilon)

    innov = airspeed_pred - airspeed

    H = sf.V1(airspeed_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov, innov_var)

def compute_airspeed_h_and_k(
        state: VState,
        P: MTangent,
        innov_var: sf.Scalar,
        epsilon: sf.Scalar
) -> (VTangent, VTangent):

    state = vstate_to_state(state)
    wind = sf.V3(state["wind_vel"][0], state["wind_vel"][1], 0.0)
    vel_rel = state["vel"] - wind
    airspeed_pred = vel_rel.norm(epsilon=epsilon)
    H = sf.V1(airspeed_pred).jacobian(state)

    K = P * H.T / sf.Max(innov_var, epsilon)

    return (H.T, K)

def compute_wind_init_and_cov_from_airspeed(
        v_local: sf.V3,
        heading: sf.Scalar,
        airspeed: sf.Scalar,
        v_var: sf.V3,
        heading_var: sf.Scalar,
        sideslip_var: sf.Scalar,
        airspeed_var: sf.Scalar,
) -> (sf.V2, sf.M22):

    # Initialise wind states assuming horizontal flight
    sideslip = sf.Symbol("beta")
    wind = sf.V2(v_local[0] - airspeed * sf.cos(heading + sideslip), v_local[1] - airspeed * sf.sin(heading + sideslip))
    J = wind.jacobian([v_local[0], v_local[1], heading, sideslip, airspeed])

    R = sf.M55()
    R[0,0] = v_var[0]
    R[1,1] = v_var[1]
    R[2,2] = heading_var
    R[3,3] = sideslip_var
    R[4,4] = airspeed_var

    P = J * R * J.T

    # Assume zero sideslip
    P = P.subs({sideslip: 0.0})
    wind = wind.subs({sideslip: 0.0})
    return (wind, P)

def predict_sideslip(
        state: State,
        epsilon: sf.Scalar
) -> (sf.Scalar):

    wind = sf.V3(state["wind_vel"][0], state["wind_vel"][1], 0.0)
    vel_rel = state["vel"] - wind
    relative_wind_body = state["quat_nominal"].inverse() * vel_rel

    # Small angle approximation of side slip model
    # Protect division by zero using epsilon
    sideslip_pred = add_epsilon_sign(relative_wind_body[1] / relative_wind_body[0], relative_wind_body[0], epsilon)

    return sideslip_pred

def compute_sideslip_innov_and_innov_var(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar, sf.Scalar):

    state = vstate_to_state(state)
    sideslip_pred = predict_sideslip(state, epsilon);

    innov = sideslip_pred - 0.0

    H = sf.V1(sideslip_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov, innov_var)

def compute_sideslip_h_and_k(
        state: VState,
        P: MTangent,
        innov_var: sf.Scalar,
        epsilon: sf.Scalar
) -> (VTangent, VTangent):

    state = vstate_to_state(state)
    sideslip_pred = predict_sideslip(state, epsilon);

    H = sf.V1(sideslip_pred).jacobian(state)

    K = P * H.T / sf.Max(innov_var, epsilon)

    return (H.T, K)

def predict_mag_body(state) -> sf.V3:
    mag_field_earth = state["mag_I"]
    mag_bias_body = state["mag_B"]

    mag_body = state["quat_nominal"].inverse() * mag_field_earth + mag_bias_body
    return mag_body

def compute_mag_innov_innov_var_and_hx(
        state: VState,
        P: MTangent,
        meas: sf.V3,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.V3, sf.V3, VTangent):

    state = vstate_to_state(state)
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
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):

    state = vstate_to_state(state)
    meas_pred = predict_mag_body(state);

    H = sf.V1(meas_pred[1]).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_mag_z_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):

    state = vstate_to_state(state)
    meas_pred = predict_mag_body(state);

    H = sf.V1(meas_pred[2]).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_yaw_321_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):

    state = vstate_to_state(state)
    R_to_earth = state["quat_nominal"].to_rotation_matrix()
    # Fix the singularity at pi/2 by inserting epsilon
    meas_pred = sf.atan2(R_to_earth[1,0], R_to_earth[0,0], epsilon=epsilon)

    H = sf.V1(meas_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_yaw_321_innov_var_and_h_alternate(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):

    state = vstate_to_state(state)
    R_to_earth = state["quat_nominal"].to_rotation_matrix()
    # Alternate form that has a singularity at yaw 0 instead of pi/2
    meas_pred = sf.pi/2 - sf.atan2(R_to_earth[0,0], R_to_earth[1,0], epsilon=epsilon)

    H = sf.V1(meas_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_yaw_312_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):

    state = vstate_to_state(state)
    R_to_earth = state["quat_nominal"].to_rotation_matrix()
    # Alternate form to be used when close to pitch +-pi/2
    meas_pred = sf.atan2(-R_to_earth[0,1], R_to_earth[1,1], epsilon=epsilon)

    H = sf.V1(meas_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_yaw_312_innov_var_and_h_alternate(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):

    state = vstate_to_state(state)
    R_to_earth = state["quat_nominal"].to_rotation_matrix()
    # Alternate form to be used when close to pitch +-pi/2
    meas_pred = sf.pi/2 - sf.atan2(-R_to_earth[1,1], R_to_earth[0,1], epsilon=epsilon)

    H = sf.V1(meas_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_mag_declination_pred_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar, VTangent):

    state = vstate_to_state(state)
    meas_pred = sf.atan2(state["mag_I"][1], state["mag_I"][0], epsilon=epsilon)

    H = sf.V1(meas_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (meas_pred, innov_var, H.T)

def predict_opt_flow(state, distance, epsilon):
    R_to_body = state["quat_nominal"].inverse()

    # Calculate earth relative velocity in a non-rotating sensor frame
    rel_vel_sensor = R_to_body * state["vel"]

    # Divide by range to get predicted angular LOS rates relative to X and Y
    # axes. Note these are rates in a non-rotating sensor frame
    flow_pred = sf.V2()
    flow_pred[0] =  rel_vel_sensor[1] / distance
    flow_pred[1] = -rel_vel_sensor[0] / distance
    flow_pred = add_epsilon_sign(flow_pred, distance, epsilon)

    return flow_pred


def compute_flow_xy_innov_var_and_hx(
        state: VState,
        P: MTangent,
        distance: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.V2, VTangent):
    state = vstate_to_state(state)
    meas_pred = predict_opt_flow(state, distance, epsilon);

    innov_var = sf.V2()
    Hx = sf.V1(meas_pred[0]).jacobian(state)
    innov_var[0] = (Hx * P * Hx.T + R)[0,0]
    Hy = sf.V1(meas_pred[1]).jacobian(state)
    innov_var[1] = (Hy * P * Hy.T + R)[0,0]

    return (innov_var, Hx.T)

def compute_flow_y_innov_var_and_h(
        state: VState,
        P: MTangent,
        distance: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):
    state = vstate_to_state(state)
    meas_pred = predict_opt_flow(state, distance, epsilon);

    Hy = sf.V1(meas_pred[1]).jacobian(state)
    innov_var = (Hy * P * Hy.T + R)[0,0]

    return (innov_var, Hy.T)

def compute_gnss_yaw_pred_innov_var_and_h(
        state: VState,
        P: MTangent,
        antenna_yaw_offset: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar, VTangent):

    state = vstate_to_state(state)
    R_to_earth = state["quat_nominal"]

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
        state: State,
        rho: sf.Scalar,
        cd: sf.Scalar,
        cm: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar):
    R_to_body = state["quat_nominal"].inverse()

    wind = sf.V3(state["wind_vel"][0], state["wind_vel"][1], 0.0)
    vel_rel = state["vel"] - wind
    vel_rel_body = R_to_body * vel_rel
    vel_rel_body_xy = sf.V2(vel_rel_body[0], vel_rel_body[1])

    bluff_body_drag = -0.5 * rho * cd * vel_rel_body_xy * vel_rel_body.norm(epsilon=epsilon)
    momentum_drag = -cm * vel_rel_body_xy

    return bluff_body_drag + momentum_drag


def compute_drag_x_innov_var_and_k(
        state: VState,
        P: MTangent,
        rho: sf.Scalar,
        cd: sf.Scalar,
        cm: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):

    state = vstate_to_state(state)
    meas_pred = predict_drag(state, rho, cd, cm, epsilon)
    Hx = sf.V1(meas_pred[0]).jacobian(state)
    innov_var = (Hx * P * Hx.T + R)[0,0]
    Ktotal = P * Hx.T / sf.Max(innov_var, epsilon)
    K = VTangent()
    K[tangent_idx["wind_vel"].idx: tangent_idx["wind_vel"].idx + tangent_idx["wind_vel"].dof] = Ktotal[tangent_idx["wind_vel"].idx: tangent_idx["wind_vel"].idx + tangent_idx["wind_vel"].dof]

    return (innov_var, K)

def compute_drag_y_innov_var_and_k(
        state: VState,
        P: MTangent,
        rho: sf.Scalar,
        cd: sf.Scalar,
        cm: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):

    state = vstate_to_state(state)
    meas_pred = predict_drag(state, rho, cd, cm, epsilon)
    Hy = sf.V1(meas_pred[1]).jacobian(state)
    innov_var = (Hy * P * Hy.T + R)[0,0]
    Ktotal = P * Hy.T / sf.Max(innov_var, epsilon)
    K = VTangent()
    K[tangent_idx["wind_vel"].idx: tangent_idx["wind_vel"].idx + tangent_idx["wind_vel"].dof] = Ktotal[tangent_idx["wind_vel"].idx: tangent_idx["wind_vel"].idx + tangent_idx["wind_vel"].dof]

    return (innov_var, K)

def predict_gravity_direction(state: State):
    # get transform from earth to body frame
    R_to_body = state["quat_nominal"].inverse()

    # the innovation is the error between measured acceleration
    #  and predicted (body frame), assuming no body acceleration
    return R_to_body * sf.Matrix([0,0,-1])

def compute_gravity_xyz_innov_var_and_hx(
        state: VState,
        P: MTangent,
        R: sf.Scalar
) -> (sf.V3, VTangent):

    state = vstate_to_state(state)
    meas_pred = predict_gravity_direction(state)

    # initialize outputs
    innov_var = sf.V3()
    H = [None] * 3

    # calculate observation jacobian (H), kalman gain (K), and innovation variance (S)
    #  for each axis
    for i in range(3):
        H[i] = sf.V1(meas_pred[i]).jacobian(state)
        innov_var[i] = (H[i] * P * H[i].T + R)[0,0]

    return (innov_var, H[0].T)

def compute_gravity_y_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar
) -> (sf.V3, VTangent, VTangent, VTangent):

    state = vstate_to_state(state)
    meas_pred = predict_gravity_direction(state)

    # calculate observation jacobian (H), kalman gain (K), and innovation variance (S)
    H = sf.V1(meas_pred[1]).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_gravity_z_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar
) -> (sf.V3, VTangent, VTangent, VTangent):

    state = vstate_to_state(state)
    meas_pred = predict_gravity_direction(state)

    # calculate observation jacobian (H), kalman gain (K), and innovation variance (S)
    H = sf.V1(meas_pred[2]).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

print("Derive EKF2 equations...")
generate_px4_function(predict_covariance, output_names=None)

if not args.disable_mag:
    generate_px4_function(compute_mag_declination_pred_innov_var_and_h, output_names=["pred", "innov_var", "H"])
    generate_px4_function(compute_mag_innov_innov_var_and_hx, output_names=["innov", "innov_var", "Hx"])
    generate_px4_function(compute_mag_y_innov_var_and_h, output_names=["innov_var", "H"])
    generate_px4_function(compute_mag_z_innov_var_and_h, output_names=["innov_var", "H"])

if not args.disable_wind:
    generate_px4_function(compute_airspeed_h_and_k, output_names=["H", "K"])
    generate_px4_function(compute_airspeed_innov_and_innov_var, output_names=["innov", "innov_var"])
    generate_px4_function(compute_drag_x_innov_var_and_k, output_names=["innov_var", "K"])
    generate_px4_function(compute_drag_y_innov_var_and_k, output_names=["innov_var", "K"])
    generate_px4_function(compute_sideslip_h_and_k, output_names=["H", "K"])
    generate_px4_function(compute_sideslip_innov_and_innov_var, output_names=["innov", "innov_var"])
    generate_px4_function(compute_wind_init_and_cov_from_airspeed, output_names=["wind", "P_wind"])

generate_px4_function(compute_yaw_312_innov_var_and_h, output_names=["innov_var", "H"])
generate_px4_function(compute_yaw_312_innov_var_and_h_alternate, output_names=["innov_var", "H"])
generate_px4_function(compute_yaw_321_innov_var_and_h, output_names=["innov_var", "H"])
generate_px4_function(compute_yaw_321_innov_var_and_h_alternate, output_names=["innov_var", "H"])
generate_px4_function(compute_flow_xy_innov_var_and_hx, output_names=["innov_var", "H"])
generate_px4_function(compute_flow_y_innov_var_and_h, output_names=["innov_var", "H"])
generate_px4_function(compute_gnss_yaw_pred_innov_var_and_h, output_names=["meas_pred", "innov_var", "H"])
generate_px4_function(compute_gravity_xyz_innov_var_and_hx, output_names=["innov_var", "Hx"])
generate_px4_function(compute_gravity_y_innov_var_and_h, output_names=["innov_var", "Hy"])
generate_px4_function(compute_gravity_z_innov_var_and_h, output_names=["innov_var", "Hz"])

generate_px4_state(State, tangent_idx)
