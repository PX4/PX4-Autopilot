/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: 	@author Thomas Gubler <thomasgubler@gmail.com>
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
 * @file mTecs_params.c
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include <nuttx/config.h>
#include <systemlib/param/param.h>

/*
 * Controller parameters, accessible via MAVLink
 */

/**
 * mTECS enabled
 *
 * Set to 1 to enable mTECS
 *
 * @min 0
 * @max 1
 * @group mTECS
 */
PARAM_DEFINE_INT32(MT_ENABLED, 0);

/**
 * Total Energy Rate Control Feedforward
 * Maps the total energy rate setpoint to the throttle setpoint
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_THR_FF, 0.7f);

/**
 * Total Energy Rate Control P
 * Maps the total energy rate error to the throttle setpoint
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_THR_P, 0.1f);

/**
 * Total Energy Rate Control I
 * Maps the integrated total energy rate to the throttle setpoint
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_THR_I, 0.25f);

/**
 * Total Energy Rate Control Offset (Cruise throttle sp)
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_THR_OFF, 0.7f);

/**
 * Energy Distribution Rate Control Feedforward
 * Maps the energy distribution rate setpoint to the pitch setpoint
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_PIT_FF, 0.4f);

/**
 * Energy Distribution Rate Control P
 * Maps the energy distribution rate error to the pitch setpoint
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_PIT_P, 0.03f);

/**
 * Energy Distribution Rate Control I
 * Maps the integrated energy distribution rate error to the pitch setpoint
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_PIT_I, 0.03f);


/**
 * Total Energy Distribution Offset (Cruise pitch sp)
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_PIT_OFF, 0.0f);

/**
 * Minimal Throttle Setpoint
 *
 * @min 0.0
 * @max 1.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_THR_MIN, 0.0f);

/**
 * Maximal Throttle Setpoint
 *
 * @min 0.0
 * @max 1.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_THR_MAX, 1.0f);

/**
 * Minimal Pitch Setpoint in Degrees
 *
 * @min -90.0
 * @max 90.0
 * @unit deg
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_PIT_MIN, -45.0f);

/**
 * Maximal Pitch Setpoint in Degrees
 *
 * @min -90.0
 * @max 90.0
 * @unit deg
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_PIT_MAX, 20.0f);

/**
 * Lowpass (cutoff freq.) for altitude
 *
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_ALT_LP, 1.0f);

/**
 * Lowpass (cutoff freq.) for the flight path angle
 *
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_FPA_LP, 1.0f);

/**
 * P gain for the altitude control
 * Maps the altitude error to the flight path angle setpoint
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_FPA_P, 0.3f);

/**
 * D gain for the altitude control
 * Maps the change of altitude error to the flight path angle setpoint
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_FPA_D, 0.0f);

/**
 * Lowpass for FPA error derivative calculation (see MT_FPA_D)
 *
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_FPA_D_LP, 1.0f);


/**
 * Minimal flight path angle setpoint
 *
 * @min -90.0
 * @max 90.0
 * @unit deg
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_FPA_MIN, -20.0f);

/**
 * Maximal flight path angle setpoint
 *
 * @min -90.0
 * @max 90.0
 * @unit deg
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_FPA_MAX, 30.0f);

/**
 * Lowpass (cutoff freq.) for airspeed
 *
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_A_LP, 0.5f);

/**
 * Airspeed derivative calculation lowpass
 *
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_AD_LP, 0.5f);

/**
 * P gain for the airspeed control
 * Maps the airspeed error to the acceleration setpoint
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_ACC_P, 0.3f);

/**
 * D gain for the airspeed control
 * Maps the change of airspeed error to the acceleration setpoint
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_ACC_D, 0.0f);

/**
 * Lowpass for ACC error derivative calculation (see MT_ACC_D)
 *
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_ACC_D_LP, 0.5f);

/**
 * Minimal acceleration (air)
 *
 * @unit m/s^2
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_ACC_MIN, -40.0f);

/**
 * Maximal acceleration (air)
 *
* @unit m/s^2
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_ACC_MAX, 40.0f);

/**
 * Minimal throttle during takeoff
 *
 * @min 0.0
 * @max 1.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_TKF_THR_MIN, 1.0f);

/**
 * Maximal throttle during takeoff
 *
 * @min 0.0
 * @max 1.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_TKF_THR_MAX, 1.0f);

/**
 * Minimal pitch during takeoff
 *
 * @min -90.0
 * @max 90.0
 * @unit deg
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_TKF_PIT_MIN, 0.0f);

/**
 * Maximal pitch during takeoff
 *
 * @min -90.0
 * @max 90.0
 * @unit deg
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_TKF_PIT_MAX, 45.0f);

/**
 * Minimal throttle in underspeed mode
 *
 * @min 0.0
 * @max 1.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_USP_THR_MIN, 1.0f);

/**
 * Maximal throttle in underspeed mode
 *
 * @min 0.0
 * @max 1.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_USP_THR_MAX, 1.0f);

/**
 * Minimal pitch in underspeed mode
 *
 * @min -90.0
 * @max 90.0
 * @unit deg
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_USP_PIT_MIN, -45.0f);

/**
 * Maximal pitch in underspeed mode
 *
 * @min -90.0
 * @max 90.0
 * @unit deg
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_USP_PIT_MAX, 0.0f);

/**
 * Minimal throttle in landing mode (only last phase of landing)
 *
 * @min 0.0
 * @max 1.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_LND_THR_MIN, 0.0f);

/**
 * Maximal throttle in landing mode (only last phase of landing)
 *
 * @min 0.0
 * @max 1.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_LND_THR_MAX, 0.0f);

/**
 * Minimal pitch in landing mode
 *
 * @min -90.0
 * @max 90.0
 * @unit deg
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_LND_PIT_MIN, -5.0f);

/**
 * Maximal pitch in landing mode
 *
 * @min -90.0
 * @max 90.0
 * @unit deg
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_LND_PIT_MAX, 15.0f);

/**
 * Integrator Limit for Total Energy Rate Control
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_THR_I_MAX, 10.0f);

/**
 * Integrator Limit for Energy Distribution Rate Control
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_PIT_I_MAX, 10.0f);
