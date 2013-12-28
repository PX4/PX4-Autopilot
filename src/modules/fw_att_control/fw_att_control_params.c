/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>

/*
 * Controller parameters, accessible via MAVLink
 *
 */

// @DisplayName		Attitude Time Constant
// @Description		This defines the latency between a step input and the achieved setpoint. Half a second is a good start value and fits for most average systems. Smaller systems may require smaller values, but as this will wear out servos faster, the value should only be decreased as needed.
// @Range		0.4 to 1.0 seconds, in tens of seconds
PARAM_DEFINE_FLOAT(FW_ATT_TC, 0.5f);

// @DisplayName		Proportional gain.
// @Description		This defines how much the elevator input will be commanded dependend on the current pitch error.
// @Range		10 to 200, 1 increments
PARAM_DEFINE_FLOAT(FW_P_P, 40.0f);

// @DisplayName		Damping gain.
// @Description		This gain damps the airframe pitch rate. In particular relevant for flying wings.
// @Range		0.0 to 10.0, 0.1 increments
PARAM_DEFINE_FLOAT(FW_P_D, 0.0f);

// @DisplayName		Integrator gain.
// @Description		This gain defines how much control response will result out of a steady state error. It trims any constant error.
// @Range		0 to 50.0
PARAM_DEFINE_FLOAT(FW_P_I, 0.0f);

// @DisplayName		Maximum positive / up pitch rate.
// @Description		This limits the maximum pitch up angular rate the controller will output (in degrees per second). Setting a value of zero disables the limit.
// @Range		0 to 90.0 degrees per seconds, in 1 increments
PARAM_DEFINE_FLOAT(FW_P_RMAX_POS, 0.0f);

// @DisplayName		Maximum negative / down pitch rate.
// @Description		This limits the maximum pitch down up angular rate the controller will output (in degrees per second). Setting a value of zero disables the limit.
// @Range		0 to 90.0 degrees per seconds, in 1 increments
PARAM_DEFINE_FLOAT(FW_P_RMAX_NEG, 0.0f);

// @DisplayName		Pitch Integrator Anti-Windup
// @Description		This limits the range in degrees the integrator can wind up to.
// @Range		0.0 to 45.0
// @Increment		1.0
PARAM_DEFINE_FLOAT(FW_P_IMAX, 15.0f);

// @DisplayName		Roll feedforward gain.
// @Description		This compensates during turns and ensures the nose stays level.
// @Range		0.5 2.0
// @Increment		0.05
// @User		User
PARAM_DEFINE_FLOAT(FW_P_ROLLFF, 1.0f);

// @DisplayName		Proportional Gain.
// @Description		This gain controls the roll angle to roll actuator output.
// @Range		10.0 200.0
// @Increment		10.0
// @User		User
PARAM_DEFINE_FLOAT(FW_R_P, 40.0f);

// @DisplayName		Damping Gain
// @Description		Controls the roll rate to roll actuator output. It helps to reduce motions in turbulence.
// @Range		0.0 10.0
// @Increment		1.0
// @User		User
PARAM_DEFINE_FLOAT(FW_R_D, 0.0f);

// @DisplayName		Integrator Gain
// @Description		This gain controls the contribution of the integral to roll actuator outputs. It trims out steady state errors.
// @Range		0.0 100.0
// @Increment		5.0
// @User		User
PARAM_DEFINE_FLOAT(FW_R_I, 0.0f);

// @DisplayName		Roll Integrator Anti-Windup
// @Description		This limits the range in degrees the integrator can wind up to.
// @Range		0.0 to 45.0
// @Increment		1.0
PARAM_DEFINE_FLOAT(FW_R_IMAX, 15.0f);

// @DisplayName		Maximum Roll Rate
// @Description		This limits the maximum roll rate the controller will output (in degrees per second). Setting a value of zero disables the limit.
// @Range		0 to 90.0 degrees per seconds
// @Increment		1.0
PARAM_DEFINE_FLOAT(FW_R_RMAX, 60);


PARAM_DEFINE_FLOAT(FW_Y_P, 0);
PARAM_DEFINE_FLOAT(FW_Y_I, 0);
PARAM_DEFINE_FLOAT(FW_Y_IMAX, 15.0f);
PARAM_DEFINE_FLOAT(FW_Y_D, 0);
PARAM_DEFINE_FLOAT(FW_Y_ROLLFF, 1);
PARAM_DEFINE_FLOAT(FW_AIRSPD_MIN, 9.0f);
PARAM_DEFINE_FLOAT(FW_AIRSPD_TRIM, 12.0f);
PARAM_DEFINE_FLOAT(FW_AIRSPD_MAX, 18.0f);
