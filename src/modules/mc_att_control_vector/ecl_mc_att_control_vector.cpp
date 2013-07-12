/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author:    Tobias Naegeli
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ecl_mc_att_control_vector.cpp
 *
 * Multirotor attitude controller based on concepts in:
 *
 * Minimum Snap Trajectory Generation and Control for Quadrotors
 * http://www.seas.upenn.edu/~dmel/mellingerICRA11.pdf
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 */

#include <mathlib/mathlib.h>
#include "ecl_mc_att_control_vector.h"

ECL_MCAttControlVector::ECL_MCAttControlVector() :
    _integral_error(0.0f, 0.0f),
    _integral_max(1000.0f),
    _integral_lock(false)
    {

    }

void ECL_MCAttControlVector::control(float dt, const math::Dcm &R_nb, float yaw, const math::Vector &F_des_in,
                                float Kp, float Kd, float Ki, const math::Vector &angular_rates,
                                math::Vector &rates_des, float &thrust)
{
    // XXX
    bool earth = true;
    bool integral_reset = false;
     
    math::Matrix R_bn = R_nb.transpose();
     
    float cy = cosf(yaw);
    float sy = sinf(yaw);
     
    math::Matrix RYaw = math::Dcm(cy,-sy,0.0f,sy,cy,0.0f,0.0f,0.0f,1.0f);
    math::Vector z_b = math::Vector3(R_bn(0,2), R_bn(1,2), R_bn(2,2));

    math::Vector3 F_des = F_des_in;
     
    // desired thrust in body frame
    // avoid division by zero
    if (F_des(2) >= 0.1f) {
        thrust = F_des(2) / R_bn(2, 2);
    } else {
        F_des(2) = 0.1f;
        thrust= F_des(2) / R_bn(2, 2);
    }

    math::Vector3 x_B_des;
    math::Vector3 y_B_des;
    math::Vector3 z_B_des;
     
    // desired body z axis
    if (earth) {
        z_B_des = (F_des / F_des.norm());
    } else {
        z_B_des = RYaw * (F_des / F_des.norm());
    }

    // desired direction in world coordinates (yaw angle)
    math::Vector3 x_C(cy, sy, 0.0f);
    // desired body y axis
    y_B_des = z_B_des.cross(x_C) / (z_B_des.cross(x_C)).norm();
    // desired body x axis
    x_B_des = y_B_des.cross(z_B_des);
    // desired Rotation Matrix
    math::Matrix R_des = math::Dcm(x_B_des(0), x_B_des(1), x_B_des(2),
                                   y_B_des(0), y_B_des(1), y_B_des(2),
                                   z_B_des(0), z_B_des(1), z_B_des(2));
     
     
    // Attitude Controller
    // P controller
     
    // error rotation matrix
    math::Matrix e_R = (R_des.transpose() * R_bn - R_bn.transpose() * R_des) * 0.5f;
     
    // error rotation vector
    math::Vector e_R_v(3);
    e_R_v(0) = e_R(1,2);
    e_R_v(1) = e_R(0,2);
    e_R_v(2) = e_R(0,1);
     
     
    // include an integral part
    math::Vector intError = math::Vector3(0.0f, 0.0f, 0.0f);
    if (!_integral_lock) {
        if (thrust > 0.3f && !integral_reset) {
            if (fabsf(_integral_error(0)) < _integral_max(0)) {
                _integral_error(0) = _integral_error(0) + e_R_v(0) * dt;
            }

            if (fabsf(_integral_error(1)) < _integral_max(1)) {
                _integral_error(1) = _integral_error(1) + e_R_v(1) * dt;
            }
           
        } else {
            _integral_error(0) = 0.0f;
            _integral_error(1) = 0.0f;
        }
        
        intError(0) = _integral_error(0);
        intError(1) = _integral_error(1);
        intError(2) = 0.0f;
    }

    rates_des = (e_R_v * Kp + angular_rates * Kd + intError * Ki);
}
