#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2022-2026 PX4 Development Team
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
    Orientation covariance prediction derivation.
"""

import re

import symforce
symforce.set_epsilon_to_symbol()

import symforce.symbolic as sf
from symforce.values import Values
import sympy as sp


def generate_px4_function(function_name, output_names):
    from symforce.codegen import Codegen, CppConfig
    import fileinput
    import os

    codegen = Codegen.function(
        function_name,
        output_names=output_names,
        config=CppConfig(zero_initialization_sparsity_threshold=1),
    )

    metadata = codegen.generate_function(
        output_dir="generated",
        skip_directory_nesting=True,
    )

    for generated_file in metadata.generated_files:
        print(f"  |- {os.path.relpath(generated_file, metadata.output_dir)}")

    with fileinput.FileInput(os.path.abspath(metadata.generated_files[0]), inplace=True) as file:
        for line in file:
            line = line.replace("std::max", "math::max")
            line = line.replace("std::min", "math::min")
            line = line.replace("Eigen", "matrix")
            line = line.replace("matrix/Dense", "matrix/math.hpp")

            # Avoid reserved naming pattern: underscore + uppercase identifier.
            line = re.sub(r'_([A-Z])', lambda match: '_' + match.group(1).lower(), line)

            print(line, end='')


class MState(sf.Matrix):
    SHAPE = (2, 2)


State = Values(
    yaw=sf.V1(),
    yaw_rate=sf.V1(),
)


def symbolic_state(state_structure):
    symbolic = Values()
    for key in state_structure.keys_recursive():
        var_type = type(state_structure[key])
        if hasattr(var_type, "symbolic"):
            symbolic[key] = var_type.symbolic(key)
    return symbolic


def get_Phi_and_G(dt: sf.Scalar):
    state = symbolic_state(State)
    yaw_acc = sf.Symbol("yaw_acc")

    state_pred = state.copy()
    state_pred["yaw"] = state["yaw"] + dt * state["yaw_rate"] + sf.S(1) / 2 * dt**2 * yaw_acc
    state_pred["yaw_rate"] = state["yaw_rate"] + dt * yaw_acc

    state_pred_matrix = sf.Matrix([
        state_pred[key] for key in state.keys_recursive()
    ])

    # Compute Phi = ∂f/∂x
    Phi = state_pred_matrix.jacobian([state])
    # Compute G = ∂f/∂w
    G = state_pred_matrix.jacobian(yaw_acc)
    return Phi, G


def derive_discrete_process_noise(dt: sf.Scalar, yaw_acc_var: sf.Scalar) -> MState:
    tau = sf.Symbol("tau")
    _, G_tau = get_Phi_and_G(tau)

    # Continuous-time white yaw acceleration enters through d/dtau of the discrete G(tau).
    G_continuous = G_tau.jacobian(tau)

    Q = sf.Matrix.zeros(State.storage_dim(), State.storage_dim())

    for row in range(State.storage_dim()):
        for col in range(State.storage_dim()):
            integrand = G_continuous[row, 0] * G_continuous[col, 0] * yaw_acc_var
            Q[row, col] = sp.integrate(integrand, (tau, 0, dt))

    return Q


def predictCov(dt: sf.Scalar, yaw_acc_var: sf.Scalar, covariance: MState) -> MState:
    Phi, _ = get_Phi_and_G(dt)
    Q = derive_discrete_process_noise(dt, yaw_acc_var)
    return Phi * covariance * Phi.T + Q


print("Derive VTEST orientation equations...")
generate_px4_function(predictCov, output_names=["cov_updated"])
