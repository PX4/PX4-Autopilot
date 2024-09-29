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
    yaw=sf.V1(),
    yaw_rate=sf.V1()
)

if not moving:
    del State["yaw_rate"]

class IdxDof():
    def __init__(self, idx, dof):
        self.idx = idx
        self.dof = dof

# TODO: remove dof, all dofs are one here.
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

def get_Phi(dt: sf.Scalar) -> T.Tuple[sf.Matrix, sf.Matrix]:
    if moving:
        Phi = sf.Matrix([
            [1, dt],
            [0, 1]])
    else:
        Phi = sf.Matrix([[1]])
    return Phi

def predictState(dt: sf.Scalar, state: VState) -> VState:
    Phi = get_Phi(dt)
    return Phi * state

def syncState(dt: sf.Scalar, state: VState) -> VState:
    Phi = get_Phi(dt)
    return Phi.inv() * (state)

def predictCov(dt: sf.Scalar, covariance: MState) -> MState:
    Phi = get_Phi(dt)

    return Phi * covariance * Phi.T

def computeInnovCov(meas_unc: sf.Scalar, covariance: MState, meas_matrix: VMeas) -> sf.Scalar:
    return (meas_matrix * covariance * meas_matrix.T)[0, 0] + meas_unc

# Generate functions
print(f"Derive VTEST equations for {'moving' if moving else 'static'} targets...")
generate_px4_function(predictState, output_names=["predict_state"])
generate_px4_function(syncState, output_names=["sync_state"])
generate_px4_function(predictCov, output_names=["cov_updated"])
generate_px4_function(computeInnovCov, output_names=["innov_cov_updated"])

generate_px4_state(State, tangent_idx)
