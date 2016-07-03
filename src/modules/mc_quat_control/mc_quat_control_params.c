/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file mc_quat_control_params.c
 * Parameters for the quaternion based quadrotor controller for manual and trajectory tracking.
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Moses Bangura <moses.bangura@anu.edu.au, dnovichman@hotmail.com>
 */

#include <systemlib/param/param.h>


/**
 * Quaternion Roll and Pitch P gains i.e. q1 and q2 respectively
 *
 * @max 20
 * @min 0.0
 * @group Quadrotor Quaternion Control
 */
PARAM_DEFINE_FLOAT(MC_QUAT_ROLL_P, 6.0f);
PARAM_DEFINE_FLOAT(MC_QUAT_PITCH_P, 6.0f);

/**
 * Quaternion ptch and roll rates P gain or K_\Omega in paper
 *
 * @min 0.0
 * @group Quadrotor Quaternion Control
 */
PARAM_DEFINE_FLOAT(MC_QUAT_ROLLR_P, 0.05f);
PARAM_DEFINE_FLOAT(MC_QUAT_PITCHR_P, 0.05f);


/**
 * Quaternion Pitch and roll rates D gain feedforward term. This is the moment of inertia
 *
 * @min 0.0
 * @group Quadrotor Quaternion Control
 */
PARAM_DEFINE_FLOAT(MC_QUAT_PITCHR_D, 0.0f);
PARAM_DEFINE_FLOAT(MC_QUAT_ROLLR_D, 0.0f);

/**
 * Yaw proportional gain
 *
 * @min 0.0
 * @group Quadrotor Quaternion Control
 */
PARAM_DEFINE_FLOAT(MC_QUAT_YAW_P, 10.0f);

/**
 * Yaw rate proportional gain
 *
 * @min 0.0
 * @group Quadrotor Quaternion Control
 */
PARAM_DEFINE_FLOAT(MC_QUAT_YAWR_P, 0.1f);

/**
 * Yaw rate integral gain
 *
 * @min 0.0
 * @group Quadrotor Quaternion Control
 */
PARAM_DEFINE_FLOAT(MC_QUAT_YAWR_I, 0.1f);

/**
 * Yaw rate integral max
 *
 * @min 0.0
 * @group Quadrotor Quaternion Control
 */
PARAM_DEFINE_FLOAT(MC_QUAT_YR_IMAX, 30.0f);

/**
 * Battery 
 *
 * @min 0.0
 * @group Quadrotor Quaternion Control
 */
PARAM_DEFINE_INT32(MC_QUAT_BAT_V, 0);

/**
 * Max manual roll
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group Quadrotor Quaternion Control
 */
PARAM_DEFINE_FLOAT(MC_MAN_R_MAX, 35.0f);

/**
 * Max manual pitch
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group Quadrotor Quaternion Control
 */
PARAM_DEFINE_FLOAT(MC_MAN_P_MAX, 35.0f);

/**
 * Max manual yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @group Quadrotor Quaternion Control
 */
PARAM_DEFINE_FLOAT(MC_MAN_Y_MAX, 120.0f);