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
 * @file mc_att_control_params.c
 * Parameters for multicopter attitude controller.
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_defines.h>
#include "mc_att_control_params.h"
#include <systemlib/param/param.h>

/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_ROLL_P);

/**
 * Roll rate P gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_ROLLRATE_P);

/**
 * Roll rate I gain
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_ROLLRATE_I);

/**
 * Roll rate D gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_ROLLRATE_D);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_PITCH_P);

/**
 * Pitch rate P gain
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_PITCHRATE_P);

/**
 * Pitch rate I gain
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_PITCHRATE_I);

/**
 * Pitch rate D gain
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_PITCHRATE_D);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_YAW_P);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_YAWRATE_P);

/**
 * Yaw rate I gain
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_YAWRATE_I);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_YAWRATE_D);

/**
 * Yaw feed forward
 *
 * Feed forward weight for manual yaw control. 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_YAW_FF);

/**
 * Max yaw rate
 *
 * Limit for yaw rate, has effect for large rotations in autonomous mode, to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_YAWRATE_MAX);

/**
 * Max manual roll
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_MAN_R_MAX);

/**
 * Max manual pitch
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_MAN_P_MAX);

/**
 * Max manual yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_MAN_Y_MAX);

/**
 * Max acro roll rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_ACRO_R_MAX);

/**
 * Max acro pitch rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_ACRO_P_MAX);

/**
 * Max acro yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_ACRO_Y_MAX);
