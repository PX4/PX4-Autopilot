#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2023-2024 PX4 Development Team
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

File: derivation_terrain_estimator.py
Description:
"""

import symforce
symforce.set_epsilon_to_symbol()

import symforce.symbolic as sf

# generate_px4_function from derivation_utils in EKF/ekf_derivation/utils
import os, sys
derivation_utils_dir = os.path.dirname(os.path.abspath(__file__)) + "/../../python/ekf_derivation/utils"
sys.path.append(derivation_utils_dir)
import derivation_utils

def predict_opt_flow(
        terrain_vpos: sf.Scalar,
        q_att: sf.V4,
        v: sf.V3,
        pos_z: sf.Scalar,
        epsilon : sf.Scalar
):
    R_to_earth = sf.Rot3(sf.Quaternion(xyz=q_att[1:4], w=q_att[0])).to_rotation_matrix()
    flow_pred = sf.V2()
    dist = - (pos_z - terrain_vpos)
    dist = derivation_utils.add_epsilon_sign(dist, dist, epsilon)
    flow_pred[0] = -v[1] / dist * R_to_earth[2, 2]
    flow_pred[1] = v[0] / dist * R_to_earth[2, 2]
    return flow_pred

def terr_est_compute_flow_xy_innov_var_and_hx(
        terrain_vpos: sf.Scalar,
        P: sf.Scalar,
        q_att: sf.V4,
        v: sf.V3,
        pos_z: sf.Scalar,
        R: sf.Scalar,
        epsilon : sf.Scalar
):
    flow_pred = predict_opt_flow(terrain_vpos, q_att, v, pos_z, epsilon)
    Hx = sf.V1(flow_pred[0]).jacobian(terrain_vpos)
    Hy = sf.V1(flow_pred[1]).jacobian(terrain_vpos)

    innov_var = sf.V2()
    innov_var[0] = (Hx * P * Hx.T + R)[0,0]
    innov_var[1] = (Hy * P * Hy.T + R)[0,0]

    return (innov_var, Hx[0, 0])

def terr_est_compute_flow_y_innov_var_and_h(
        terrain_vpos: sf.Scalar,
        P: sf.Scalar,
        q_att: sf.V4,
        v: sf.V3,
        pos_z: sf.Scalar,
        R: sf.Scalar,
        epsilon : sf.Scalar
):
    flow_pred = predict_opt_flow(terrain_vpos, q_att, v, pos_z, epsilon)
    Hy = sf.V1(flow_pred[1]).jacobian(terrain_vpos)

    innov_var = (Hy * P * Hy.T + R)[0,0]

    return (innov_var, Hy[0, 0])

print("Derive terrain estimator equations...")
derivation_utils.generate_px4_function(terr_est_compute_flow_xy_innov_var_and_hx, output_names=["innov_var", "H"])
derivation_utils.generate_px4_function(terr_est_compute_flow_y_innov_var_and_h, output_names=["innov_var", "H"])
