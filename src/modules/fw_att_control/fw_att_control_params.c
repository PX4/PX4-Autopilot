/****************************************************************************
 *
f *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file fw_att_control_params.c
 *
 * Parameters defined by the fixed-wing attitude control task
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>


/*
 * Controller parameters, accessible via MAVLink
 *
 */

/**
 * Attitude Time Constant
 *
 * This defines the latency between a step input and the achieved setpoint
 * (inverse to a P gain). Half a second is a good start value and fits for
 * most average systems. Smaller systems may require smaller values, but as
 * this will wear out servos faster, the value should only be decreased as
 * needed.
 *
 * @unit seconds
 * @min 0.4
 * @max 1.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_ATT_TC, 0.5f);

/**
 * Pitch rate proportional gain.
 *
 * This defines how much the elevator input will be commanded depending on the
 * current body angular rate error.
 *
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_PR_P, 0.05f);

/**
 * Pitch rate integrator gain.
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @min 0.0
 * @max 50.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_PR_I, 0.0f);

/**
 * Maximum positive / up pitch rate.
 *
 * This limits the maximum pitch up angular rate the controller will output (in
 * degrees per second). Setting a value of zero disables the limit.
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_P_RMAX_POS, 0.0f);

/**
 * Maximum negative / down pitch rate.
 *
 * This limits the maximum pitch down up angular rate the controller will
 * output (in degrees per second). Setting a value of zero disables the limit.
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_P_RMAX_NEG, 0.0f);

/**
 * Pitch rate integrator limit
 *
 * The portion of the integrator part in the control surface deflection is
 * limited to this value
 *
 * @min 0.0
 * @max 1.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_PR_IMAX, 0.2f);

/**
 * Roll to Pitch feedforward gain.
 *
 * This compensates during turns and ensures the nose stays level.
 *
 * @min 0.0
 * @max 2.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_P_ROLLFF, 0.0f); //xxx: set to 0 as default, see comment in ECL_PitchController::control_attitude (float turn_offset = ...)

/**
 * Roll rate proportional Gain
 *
 * This defines how much the aileron input will be commanded depending on the
 * current body angular rate error.
 *
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_RR_P, 0.05f);

/**
 * Roll rate integrator Gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @min 0.0
 * @max 100.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_RR_I, 0.0f);

/**
 * Roll Integrator Anti-Windup
 *
 * The portion of the integrator part in the control surface deflection is limited to this value.
 *
 * @min 0.0
 * @max 1.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_RR_IMAX, 0.2f);

/**
 * Maximum Roll Rate
 *
 * This limits the maximum roll rate the controller will output (in degrees per
 * second). Setting a value of zero disables the limit.
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_R_RMAX, 0.0f);

/**
 * Yaw rate proportional gain
 *
 * This defines how much the rudder input will be commanded depending on the
 * current body angular rate error.
 *
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_YR_P, 0.05f);

/**
 * Yaw rate integrator gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @min 0.0
 * @max 50.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_YR_I, 0.0f);

/**
 * Yaw rate integrator limit
 *
 * The portion of the integrator part in the control surface deflection is
 * limited to this value
 *
 * @min 0.0
 * @max 1.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_YR_IMAX, 0.2f);

/**
 * Maximum Yaw Rate
 *
 * This limits the maximum yaw rate the controller will output (in degrees per
 * second). Setting a value of zero disables the limit.
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_Y_RMAX, 0.0f);

/**
 * Roll rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @min 0.0
 * @max 10.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_RR_FF, 0.3f);

/**
 * Pitch rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @min 0.0
 * @max 10.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_PR_FF, 0.4f);

/**
 * Yaw rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @min 0.0
 * @max 10.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_YR_FF, 0.3f);

/**
 * Minimal speed for yaw coordination
 *
 * For airspeeds above this value, the yaw rate is calculated for a coordinated
 * turn. Set to a very high value to disable.
 *
 * @unit m/s
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_YCO_VMIN, 1000.0f);

/* Airspeed parameters:
 * The following parameters about airspeed are used by the attitude and the
 * position controller.
 * */

/**
 * Minimum Airspeed
 *
 * If the airspeed falls below this value, the TECS controller will try to
 * increase airspeed more aggressively.
 *
 * @unit m/s
 * @min 0.0
 * @max 30.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_MIN, 13.0f);

/**
 * Trim Airspeed
 *
 * The TECS controller tries to fly at this airspeed.
 *
 * @unit m/s
 * @min 0.0
 * @max 30.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_TRIM, 20.0f);

/**
 * Maximum Airspeed
 *
 * If the airspeed is above this value, the TECS controller will try to decrease
 * airspeed more aggressively.
 *
 * @unit m/s
 * @min 0.0
 * @max 30.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_MAX, 50.0f);

/**
 * Roll Setpoint Offset
 *
 * An airframe specific offset of the roll setpoint in degrees, the value is
 * added to the roll setpoint and should correspond to the typical cruise speed
 * of the airframe.
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_RSP_OFF, 0.0f);

/**
 * Pitch Setpoint Offset
 *
 * An airframe specific offset of the pitch setpoint in degrees, the value is
 * added to the pitch setpoint and should correspond to the typical cruise
 * speed of the airframe.
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_PSP_OFF, 0.0f);

/**
 * Max Manual Roll
 *
 * Max roll for manual control in attitude stabilized mode
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_R_MAX, 45.0f);

/**
 * Max Manual Pitch
 *
 * Max pitch for manual control in attitude stabilized mode
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_P_MAX, 45.0f);
