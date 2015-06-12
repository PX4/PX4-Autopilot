/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Anton Babushkin <anton.babushkin@me.com>
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
 * OF USE) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_pos_control_params.c
 * Multicopter position controller parameters.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_defines.h>
#include "mc_pos_control_params.h"
#include <systemlib/param/param.h>

/**
 * Minimum thrust
 *
 * Minimum vertical thrust. It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 1.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_THR_MIN);

/**
 * Maximum thrust
 *
 * Limit max allowed thrust.
 *
 * @min 0.0
 * @max 1.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_THR_MAX);

/**
 * Proportional gain for vertical position error
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_Z_P);

/**
 * Proportional gain for vertical velocity error
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_Z_VEL_P);

/**
 * Integral gain for vertical velocity error
 *
 * Non zero value allows hovering thrust estimation on stabilized or autonomous takeoff.
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_Z_VEL_I);

/**
 * Differential gain for vertical velocity error
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_Z_VEL_D);

/**
 * Maximum vertical velocity
 *
 * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL).
 *
 * @unit m/s
 * @min 0.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_Z_VEL_MAX);

/**
 * Vertical velocity feed forward
 *
 * Feed forward weight for altitude control in stabilized modes (ALTCTRL). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_Z_FF);

/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_XY_P);

/**
 * Proportional gain for horizontal velocity error
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_XY_VEL_P);

/**
 * Integral gain for horizontal velocity error
 *
 * Non-zero value allows to resist wind.
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_XY_VEL_I);

/**
 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_XY_VEL_D);

/**
 * Maximum horizontal velocity
 *
 * Maximum horizontal velocity in AUTO mode and endpoint for position stabilized mode (POSCTRL).
 *
 * @unit m/s
 * @min 0.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_XY_VEL_MAX);

/**
 * Horizontal velocity feed forward
 *
 * Feed forward weight for position control in position control mode (POSCTRL). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_XY_FF);

/**
 * Maximum tilt angle in air
 *
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_TILTMAX_AIR);

/**
 * Maximum tilt during landing
 *
 * Limits maximum tilt angle on landing.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_TILTMAX_LND);

/**
 * Landing descend rate
 *
 * @unit m/s
 * @min 0.0
 * @group Multicopter Position Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_LAND_SPEED);

/**
 * Max manual roll
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_MAN_R_MAX);

/**
 * Max manual pitch
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_MAN_P_MAX);

/**
 * Max manual yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MPP_MAN_Y_MAX);

