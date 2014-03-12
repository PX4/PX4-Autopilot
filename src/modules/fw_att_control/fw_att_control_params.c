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
// @DisplayName		Attitude Time Constant
// @Description		This defines the latency between a step input and the achieved setpoint (inverse to a P gain). Half a second is a good start value and fits for most average systems. Smaller systems may require smaller values, but as this will wear out servos faster, the value should only be decreased as needed.
// @Range		0.4 to 1.0 seconds, in tens of seconds
PARAM_DEFINE_FLOAT(FW_ATT_TC, 0.5f);

// @DisplayName		Pitch rate proportional gain.
// @Description		This defines how much the elevator input will be commanded depending on the current body angular rate error.
// @Range		10 to 200, 1 increments
PARAM_DEFINE_FLOAT(FW_PR_P, 0.05f);

// @DisplayName		Pitch rate integrator gain.
// @Description		This gain defines how much control response will result out of a steady state error. It trims any constant error.
// @Range		0 to 50.0
PARAM_DEFINE_FLOAT(FW_PR_I, 0.0f);

// @DisplayName		Maximum positive / up pitch rate.
// @Description		This limits the maximum pitch up angular rate the controller will output (in degrees per second). Setting a value of zero disables the limit.
// @Range		0 to 90.0 degrees per seconds, in 1 increments
PARAM_DEFINE_FLOAT(FW_P_RMAX_POS, 0.0f);

// @DisplayName		Maximum negative / down pitch rate.
// @Description		This limits the maximum pitch down up angular rate the controller will output (in degrees per second). Setting a value of zero disables the limit.
// @Range		0 to 90.0 degrees per seconds, in 1 increments
PARAM_DEFINE_FLOAT(FW_P_RMAX_NEG, 0.0f);

// @DisplayName		Pitch rate integrator limit
// @Description		The portion of the integrator part in the control surface deflection is limited to this value
// @Range		0.0 to 1
// @Increment		0.1
PARAM_DEFINE_FLOAT(FW_PR_IMAX, 0.2f);

// @DisplayName		Roll to Pitch feedforward gain.
// @Description		This compensates during turns and ensures the nose stays level.
// @Range		0.5 2.0
// @Increment		0.05
// @User		User
PARAM_DEFINE_FLOAT(FW_P_ROLLFF, 0.0f); //xxx: set to 0 as default, see comment in ECL_PitchController::control_attitude (float turn_offset = ...)

// @DisplayName		Roll rate proportional Gain.
// @Description		This defines how much the aileron input will be commanded depending on the current body angular rate error.
// @Range		10.0 200.0
// @Increment		10.0
// @User		User
PARAM_DEFINE_FLOAT(FW_RR_P, 0.05f);

// @DisplayName		Roll rate integrator Gain
// @Description		This gain defines how much control response will result out of a steady state error. It trims any constant error.
// @Range		0.0 100.0
// @Increment		5.0
// @User		User
PARAM_DEFINE_FLOAT(FW_RR_I, 0.0f);

// @DisplayName		Roll Integrator Anti-Windup
// @Description		The portion of the integrator part in the control surface deflection is limited to this value.
// @Range		0.0 to 1.0
// @Increment		0.1
PARAM_DEFINE_FLOAT(FW_RR_IMAX, 0.2f);

// @DisplayName		Maximum Roll Rate
// @Description		This limits the maximum roll rate the controller will output (in degrees per second). Setting a value of zero disables the limit.
// @Range		0 to 90.0 degrees per seconds
// @Increment		1.0
PARAM_DEFINE_FLOAT(FW_R_RMAX, 0);

// @DisplayName		Yaw rate proportional gain.
// @Description		This defines how much the rudder input will be commanded depending on the current body angular rate error.
// @Range		10 to 200, 1 increments
PARAM_DEFINE_FLOAT(FW_YR_P, 0.05);

// @DisplayName		Yaw rate integrator gain.
// @Description		This gain defines how much control response will result out of a steady state error. It trims any constant error.
// @Range		0 to 50.0
PARAM_DEFINE_FLOAT(FW_YR_I, 0.0f);

// @DisplayName		Yaw rate integrator limit
// @Description		The portion of the integrator part in the control surface deflection is limited to this value
// @Range		0.0 to 1
// @Increment		0.1
PARAM_DEFINE_FLOAT(FW_YR_IMAX, 0.2f);

// @DisplayName		Maximum Yaw Rate
// @Description		This limits the maximum yaw rate the controller will output (in degrees per second). Setting a value of zero disables the limit.
// @Range		0 to 90.0 degrees per seconds
// @Increment		1.0
PARAM_DEFINE_FLOAT(FW_Y_RMAX, 0);

// @DisplayName		Roll rate feed forward
// @Description		Direct feed forward from rate setpoint to control surface output
// @Range		0 to 10
// @Increment		0.1
PARAM_DEFINE_FLOAT(FW_RR_FF, 0.3f);

// @DisplayName		Pitch rate feed forward
// @Description		Direct feed forward from rate setpoint to control surface output
// @Range		0 to 10
// @Increment		0.1
PARAM_DEFINE_FLOAT(FW_PR_FF, 0.4f);

// @DisplayName		Yaw rate feed forward
// @Description		Direct feed forward from rate setpoint to control surface output
// @Range		0 to 10
// @Increment		0.1
PARAM_DEFINE_FLOAT(FW_YR_FF, 0.3f);

// @DisplayName		Minimal speed for yaw coordination
// @Description		For airspeeds above this value the yaw rate is calculated for a coordinated turn. Set to a very high value to disable.
// @Range		0 to 90.0 degrees per seconds
// @Increment		1.0
PARAM_DEFINE_FLOAT(FW_YCO_VMIN, 1000.0f);

/* Airspeed parameters: the following parameters about airspeed are used by the attitude and the positon controller */

// @DisplayName		Minimum Airspeed
// @Description		If the airspeed falls below this value the TECS controller will try to increase airspeed more aggressively
// @Range		0.0 to 30
PARAM_DEFINE_FLOAT(FW_AIRSPD_MIN, 13.0f);

// @DisplayName		Trim Airspeed
// @Description		The TECS controller tries to fly at this airspeed
// @Range		0.0 to 30
PARAM_DEFINE_FLOAT(FW_AIRSPD_TRIM, 20.0f);

// @DisplayName		Maximum Airspeed
// @Description		If the airspeed is above this value the TECS controller will try to decrease airspeed more aggressively
// @Range		0.0 to 30
PARAM_DEFINE_FLOAT(FW_AIRSPD_MAX, 50.0f);

// @DisplayName		Roll Setpoint Offset
// @Description		An airframe specific offset of the roll setpoint in degrees, the value is added to the roll setpoint and should correspond to the typical cruise speed of the airframe
// @Range		-90.0 to 90.0
PARAM_DEFINE_FLOAT(FW_RSP_OFF, 0.0f);

// @DisplayName		Pitch Setpoint Offset
// @Description		An airframe specific offset of the pitch setpoint in degrees, the value is added to the pitch setpoint and should correspond to the typical cruise speed of the airframe
// @Range		-90.0 to 90.0
PARAM_DEFINE_FLOAT(FW_PSP_OFF, 0.0f);
