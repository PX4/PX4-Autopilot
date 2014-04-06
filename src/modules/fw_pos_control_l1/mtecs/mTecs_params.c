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
PARAM_DEFINE_INT32(MT_ENABLED, 1);

/**
 * Total Energy Rate Control FF
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_THR_FF, 0.2f);

/**
 * Total Energy Rate Control P
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_THR_P, 0.03f);

/**
 * Total Energy Rate Control I
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_THR_I, 0.1f);

/**
 * Total Energy Rate Control OFF (Cruise throttle)
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_THR_OFF, 0.7f);

/**
 * Energy Distribution Rate Control FF
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_PIT_FF, 0.1f);

/**
 * Energy Distribution Rate Control P
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_PIT_P, 0.03f);

/**
 * Energy Distribution Rate Control I
 *
 * @min 0.0
 * @max 10.0
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_PIT_I, 0.03f);


/**
 * Total Energy Distribution OFF (Cruise pitch sp)
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
 * @min -90.0f
 * @max 90.0f
 * @unit deg
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_PIT_MIN, -45.0f);

/**
 * Maximal Pitch Setpoint in Degrees
 *
 * @min -90.0f
 * @max 90.0f
 * @unit deg
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_PIT_MAX, 20.0f);

/**
 * P gain for the altitude control
 *
 * @min 0.0f
 * @max 10.0f
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_FPA_P, 0.2f);

/**
 * D gain for the altitude control
 *
 * @min 0.0f
 * @max 10.0f
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_FPA_D, 0.0f);

/**
 * Lowpass for FPA error derivative (see MT_FPA_D)
 *
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_FPA_D_LP, 1.0f);


/**
 * Minimal flight path angle setpoint
 *
 * @min -90.0f
 * @max 90.0f
 * @unit deg
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_FPA_MIN, -10.0f);

/**
 * Maximal flight path angle setpoint
 *
 * @min -90.0f
 * @max 90.0f
 * @unit deg
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_FPA_MAX, 30.0f);


/**
 * P gain for the airspeed control
 *
 * @min 0.0f
 * @max 10.0f
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_ACC_P, 1.5f);

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
 * Airspeed derivative lowpass
 *
 * @group mTECS
 */
PARAM_DEFINE_FLOAT(MT_AD_LP, 1.0f);
