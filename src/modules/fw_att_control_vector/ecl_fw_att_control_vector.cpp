/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file ecl_fw_att_control_vector.cpp
 *
 * Fixed wing attitude controller
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 *
 */

#include <mathlib/mathlib.h>
#include <systemlib/geo/geo.h>
#include "ecl_fw_att_control_vector.h"

ECL_FWAttControlVector::ECL_FWAttControlVector() :
    _integral_error(0.0f, 0.0f),
    _integral_max(1000.0f, 1000.0f),
    _rates_demanded(0.0f, 0.0f, 0.0f),
    _k_p(1.0f, 1.0f, 1.0f),
    _k_d(1.0f, 1.0f, 1.0f),
    _k_i(1.0f, 1.0f, 1.0f),
    _integral_lock(false),
    _p_airspeed_min(12.0f),
    _p_airspeed_max(24.0f),
    _p_tconst(0.1f),
    _p_roll_ffd(1.0f),
    _airspeed_enabled(false)
    {

    }

/**
 *
 * @param F_des_in  Desired force vector in body frame (NED). Straight flight is (0 0 -1)',
 *                  banking hard right (1 0 -1)' and pitching down (1 0 -1)'.
 */
void ECL_FWAttControlVector::control(float dt, float airspeed, float airspeed_scaling, const math::Dcm &R_nb, float roll, float pitch, float yaw, const math::Vector &F_des_in,
                                const math::Vector &angular_rates,
                                math::Vector &moment_des, float &thrust)
{
    if (!isfinite(airspeed) || !airspeed_enabled()) {
        // If airspeed is not available or Inf/NaN, use the center
        // of min / max 
        airspeed = 0.5f * (_p_airspeed_min + _p_airspeed_max);
    }
     
    math::Dcm R_bn(R_nb.transpose());
    math::Matrix R_yaw_bn = math::Dcm(math::EulerAngles(0.0f, 0.0f, yaw)).transpose();

    // Establish actuator signs and lift compensation
    float lift_sign = (R_bn(3, 3) >= 0) ? 1.0f : -1.0f;

    float lift_comp = CONSTANTS_ONE_G / math::max(airspeed , float(_p_airspeed_min) * (R_nb(2,3) * R_nb(2,3)) / (R_nb(3,3) * sqrtf((R_nb(2,3) * R_nb(2,3)) + (R_nb(3,3) * R_nb(3,3)));
    //float lift_comp = fabsf((CONSTANTS_ONE_G / math::max(airspeed , float(_p_airspeed_min)) * tanf(roll) * sinf(roll))) * _p_roll_ffd;
    
    float cy = cosf(yaw);
    float sy = sinf(yaw);
     
    //math::Matrix RYaw = math::Dcm(cy,-sy,0.0f,sy,cy,0.0f,0.0f,0.0f,1.0f);
    math::Vector z_b = math::Vector3(R_bn(0,2), R_bn(1,2), R_bn(2,2));

    math::Vector3 F_des = R_yaw_bn * F_des_in;
     
    // desired thrust in body frame
    // avoid division by zero
    // compensates for thrust loss due to roll/pitch
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
    z_B_des = (F_des / F_des.norm());

    // desired direction in world coordinates (yaw angle)
    math::Vector3 x_C(cy, sy, 0.0f);
    // desired body y axis
    y_B_des = z_B_des.cross(x_C) / (z_B_des.cross(x_C)).norm();
    // desired body x axis
    x_B_des = y_B_des.cross(z_B_des);
    // desired Rotation Matrix
    math::Dcm R_des(x_B_des(0), x_B_des(1), x_B_des(2),
                                   y_B_des(0), y_B_des(1), y_B_des(2),
                                   z_B_des(0), z_B_des(1), z_B_des(2));
     
     
    // Attitude Controller
    // P controller
     
    // error rotation matrix
    // operation executed in quaternion space to allow large differences
    // XXX switch between operations based on difference,
    // benchmark both options
    math::Quaternion e_q = math::Quaternion(R_des) - math::Quaternion(R_bn);
    // Renormalize
    e_q = e_q / e_q.norm();
    math::Matrix e_R = math::Dcm(e_q);
    //small angles: math::Matrix e_R = (R_des.transpose() * R_bn - R_bn.transpose() * R_des) * 0.5f;
     
    // error rotation vector
    math::Vector e_R_v(3);
    e_R_v(0) = e_R(1,2);
    e_R_v(1) = e_R(0,2);
    e_R_v(2) = e_R(0,1);
     
     
    // attitude integral error
    math::Vector intError = math::Vector3(0.0f, 0.0f, 0.0f);
    if (!_integral_lock) {

        if (fabsf(_integral_error(0)) < _integral_max(0)) {
            _integral_error(0) = _integral_error(0) + e_R_v(0) * dt;
        }

        if (fabsf(_integral_error(1)) < _integral_max(1)) {
            _integral_error(1) = _integral_error(1) + e_R_v(1) * dt;
        }
        
        intError(0) = _integral_error(0);
        intError(1) = _integral_error(1);
        intError(2) = 0.0f;
    }

    _rates_demanded = (e_R_v * _k_p(0) + angular_rates * _k_d(0) + intError * _k_i(0));
    moment_des = _rates_demanded;
}
