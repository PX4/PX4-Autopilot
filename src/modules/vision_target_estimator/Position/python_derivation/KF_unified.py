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
from utils.derivation_utils import *

# Initialize parser
parser = argparse.ArgumentParser()

parser.add_argument("--moving", action='store_true', help="generate equations for moving filter")

# Read arguments from command line
args = parser.parse_args()
moving = args.moving

# The state vector is organized in an ordered dictionary
State = Values(
    pos_rel=sf.V1(),
    vel_uav=sf.V1(),
    bias=sf.V1(),
    acc_target=sf.V1(),
    vel_target=sf.V1()
)

if not moving:
    del State["acc_target"]
    del State["vel_target"]

class IdxDof():
    def __init__(self, idx, dof):
        self.idx = idx
        self.dof = dof

def BuildTangentStateIndex():
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

class MState(sf.Matrix):
    SHAPE = (State.storage_dim(), State.storage_dim())

class VMeas(sf.Matrix):
    SHAPE = (1, State.storage_dim())


def get_Phi(dt: sf.Scalar) -> sf.Matrix:
    n = State.storage_dim()
    Phi = sf.Matrix.zeros(State.storage_dim(), State.storage_dim())

    # Helper functions to set blocks in Phi matrix
    def set_Phi_block(row_key, col_key, value):
        idx_row = tangent_idx[row_key].idx
        dof_row = tangent_idx[row_key].dof
        idx_col = tangent_idx[col_key].idx
        dof_col = tangent_idx[col_key].dof
        if isinstance(value, sf.Matrix):
            # Ensure the value matrix has the correct shape
            assert value.shape == (dof_row, dof_col), "Value matrix shape mismatch"
            Phi[idx_row:idx_row + dof_row, idx_col:idx_col + dof_col] = value
        else:
            # Scalar value; create a block matrix
            block = value * sf.Matrix.eye(dof_row, dof_col)
            Phi[idx_row:idx_row + dof_row, idx_col:idx_col + dof_col] = block

    # Set the diagonal elements of Phi to 1
    for key in State.keys_recursive():
        idx = tangent_idx[key].idx
        dof = tangent_idx[key].dof
        for i in range(dof):
            Phi[idx + i, idx + i] = 1

    set_Phi_block('pos_rel', 'vel_uav', -dt)

    if moving:
        # Update Phi with off-diagonal elements for the moving filter
        set_Phi_block('pos_rel', 'acc_target', 0.5 * dt * dt)
        set_Phi_block('pos_rel', 'vel_target', dt)
        set_Phi_block('vel_target', 'acc_target', dt)

    return Phi


def get_G(dt: sf.Scalar) -> sf.Matrix:
    G = sf.Matrix.zeros(State.storage_dim(), 1)

    # Helper functions to set blocks in G matrix
    def set_G_block(key, value):
        idx = tangent_idx[key].idx
        dof = tangent_idx[key].dof
        if isinstance(value, sf.Matrix):
            # Ensure the value vector has the correct shape
            assert value.shape == (dof, 1), "Value vector shape mismatch"
            G[idx:idx + dof, 0] = value
        else:
            # Scalar value; create a vector
            block = sf.Matrix([value] * dof)
            G[idx:idx + dof, 0] = block

    # Update G with the process noise terms
    set_G_block('pos_rel', -0.5 * dt * dt)
    set_G_block('vel_uav', dt)

    return G

def predictState(dt: sf.Scalar, state: VState, acc: sf.Scalar) -> VState:
    Phi = get_Phi(dt)
    G = get_G(dt)
    return Phi * state + G * acc

def syncState(dt: sf.Scalar, state: VState, acc: sf.Scalar) -> VState:
    Phi = get_Phi(dt)
    G = get_G(dt)
    return Phi.inv() * (state - G * acc)

def predictCov(dt: sf.Scalar, input_var: sf.Scalar, bias_var: sf.Scalar, acc_var: sf.Scalar, covariance: MState) -> MState:
    Phi = get_Phi(dt)
    G = get_G(dt)
    Q = sf.Matrix.zeros(State.storage_dim(), State.storage_dim())

    idx_bias = tangent_idx['bias'].idx
    Q[idx_bias, idx_bias] = bias_var

    if moving:
        idx_a_target = tangent_idx['acc_target'].idx
        Q[idx_a_target, idx_a_target] = acc_var

    return G * input_var * G.T + Q + Phi * covariance * Phi.T

def computeInnovCov(meas_unc: sf.Scalar, covariance: MState, meas_matrix: VMeas) -> sf.Scalar:
    return (meas_matrix * covariance * meas_matrix.T)[0, 0] + meas_unc

# Generate functions
print(f"Derive VTEST equations for {'moving' if moving else 'static'} targets...")
generate_px4_function(predictState, output_names=["predict_state"])
generate_px4_function(syncState, output_names=["sync_state"])
generate_px4_function(predictCov, output_names=["cov_updated"])
generate_px4_function(computeInnovCov, output_names=["innov_cov_updated"])

generate_px4_state(State, tangent_idx)
