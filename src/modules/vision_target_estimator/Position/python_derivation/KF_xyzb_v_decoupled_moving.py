#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2023 PX4 Development Team
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

File: KF_xyzb_v_decoupled_moving.py
Description: Derive the Kalman filter equations for moving targets with decoupled dynamics using symforce (augmented state)
Author: Jonas Perolini <jonspero@me.com>
"""

import symforce.symbolic as sf

def generate_px4_function(function_name, output_names):
    from symforce.codegen import Codegen, CppConfig
    import os
    import fileinput

    codegen = Codegen.function(
            function_name,
            output_names=output_names,
            config=CppConfig())
    metadata = codegen.generate_function(
            output_dir="src/modules/vision_target_estimator/python_derivation/generated/decoupled_moving_xyzb_v",
            skip_directory_nesting=True)

    print("Files generated in {}:\n".format(metadata.output_dir))
    for f in metadata.generated_files:
        print("  |- {}".format(os.path.relpath(f, metadata.output_dir)))

    # Replace cstdlib and Eigen functions by PX4 equivalents
    with fileinput.FileInput(os.path.abspath(metadata.generated_files[0]), inplace=True) as file:
        for line in file:
            line = line.replace("std::max", "math::max")
            line = line.replace("std::min", "math::min")
            line = line.replace("Eigen", "matrix")
            line = line.replace("matrix/Dense", "matrix/math.hpp")
            print(line, end='')

#-------------------------------- DECOUPLED DYNAMICS, MOVING TARGET ------------------------------------------ #

class State:
    r = 0
    rx_dot = 1
    bx = 2
    atx = 3,
    vtx = 4,
    n_states = 5

class Input:
    ax = 0
    n_inputs = 1

class Directions:
    x = 0,
    nb_directions = 1

class VState(sf.Matrix):
    SHAPE = (State.n_states, 1)

class MState(sf.Matrix):
    SHAPE = (State.n_states, State.n_states)

class VInput(sf.Matrix):
    SHAPE = (Input.n_inputs, 1)

class VMeas(sf.Matrix):
    SHAPE = (1, State.n_states)

class MDirections(sf.Matrix):
    SHAPE = (Directions.nb_directions, Directions.nb_directions)


def predictState(dt: sf.Scalar, state: VState, acc: VInput) -> (VState):

    Phi = sf.Matrix([   [1, -dt, 0 , 0.5*dt*dt, dt],
                        [0, 1,  0, 0, 0],
                        [0, 0,  1, 0, 0],
                        [0, 0,  0, 1, 0],
                        [0, 0,  0, dt, 1],])


    G = sf.Matrix([ [-0.5*dt*dt], [dt], [0], [0], [0]])

    return (Phi * state + G*acc).simplify()


def syncState(dt: sf.Scalar, state: VState, acc: VInput) -> (VState):

    Phi = sf.Matrix([   [1, -dt, 0 , 0.5*dt*dt, dt],
                        [0, 1,  0, 0, 0],
                        [0, 0,  1, 0, 0],
                        [0, 0,  0, 1, 0],
                        [0, 0,  0, dt, 1],])


    G = sf.Matrix([ [-0.5*dt*dt], [dt], [0], [0], [0]])

    return (Phi.inv() * (state - G*acc)).simplify()

def predictCov(dt: sf.Scalar, input_var: sf.Scalar, bias_var: sf.Scalar, acc_var: sf.Scalar, covariance: MState) -> (MState):

    Phi = sf.Matrix([   [1, -dt, 0 , 0.5*dt*dt, dt],
                        [0, 1,  0, 0, 0],
                        [0, 0,  1, 0, 0],
                        [0, 0,  0, 1, 0],
                        [0, 0,  0, dt, 1],])


    G = sf.Matrix([ [-0.5*dt*dt], [dt], [0], [0], [0]])

    Q_acc = sf.Matrix([  [0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0],
                                [0, 0, 0, acc_var, 0],
                                [0, 0, 0, 0, 0]])

    Q_bias = sf.Matrix([  [0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0],
                            [0, 0, bias_var, 0, 0],
                            [0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0]])

    return G*input_var*G.T + Q_bias + Q_acc + Phi*covariance*Phi.T


def computeInnovCov(meas_unc: sf.Scalar, covariance: MState, meas_matrix: VMeas) -> (sf.Scalar):
    return (meas_matrix*covariance*meas_matrix.T)[0,0] + meas_unc

# generate_px4_function(predictState, output_names=["predict_state"])
# generate_px4_function(syncState, output_names=["sync_state"])
generate_px4_function(predictCov, output_names=["cov_updated"])
generate_px4_function(computeInnovCov, output_names=["innov_cov_updated"])