#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2022-2025 PX4 Development Team
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
    Derivation of KF
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

def symbolic_state(state_structure):
    symbolic_state = Values()
    for key in state_structure.keys_recursive():
        var_type = type(state_structure[key])
        if hasattr(var_type, 'symbolic'):
            symbolic_state[key] = var_type.symbolic(key)

    return symbolic_state

def get_Phi_and_G(dt: sf.Scalar,
) -> MState:

    state = symbolic_state(State)

    # Input acceleration (symbolic)
    acc = sf.Symbol('acc')

    state_pred = state.copy()
    state_pred["pos_rel"] = state["pos_rel"] - dt * state["vel_uav"] - 0.5 * dt**2 * acc
    state_pred["vel_uav"] = state["vel_uav"] + dt * acc
    state_pred["bias"] = state["bias"]

    if moving:
        state_pred["pos_rel"] += dt * state["vel_target"] + 0.5 * dt**2 * state["acc_target"]
        state_pred["vel_target"] = state["vel_target"] + dt * state["acc_target"]
        state_pred["acc_target"] = state["acc_target"]

    # State vector
    state_pred_matrix = sf.Matrix([
        state_pred[key] for key in state.keys_recursive()
    ])

    # Compute Phi = ∂f/∂x
    Phi = state_pred_matrix.jacobian([state])
    # Compute G = ∂f/∂w
    G = state_pred_matrix.jacobian(acc)

    return Phi, G

def predictState(dt: sf.Scalar, state: VState, acc: sf.Scalar) -> VState:
    Phi, G = get_Phi_and_G(dt)
    return Phi * state + G * acc

def syncState(dt: sf.Scalar, state: VState, acc: sf.Scalar) -> VState:
    Phi, G = get_Phi_and_G(dt)
    return Phi.inv() * (state - G * acc)

def predictCov(dt: sf.Scalar, input_var: sf.Scalar, bias_var: sf.Scalar, acc_var: sf.Scalar, covariance: MState) -> MState:
    Phi, G = get_Phi_and_G(dt)
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
