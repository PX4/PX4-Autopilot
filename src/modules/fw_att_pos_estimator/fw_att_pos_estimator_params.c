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
 * @file fw_att_pos_estimator_params.c
 *
 * Parameters defined by the attitude and position estimator task
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>

/*
 * Estimator parameters, accessible via MAVLink
 *
 */

/**
 * Velocity estimate delay
 *
 * The delay in milliseconds of the velocity estimate from GPS.
 *
 * @min 0
 * @max 1000
 * @group Position Estimator
 */
PARAM_DEFINE_INT32(PE_VEL_DELAY_MS, 230);

/**
 * Position estimate delay
 *
 * The delay in milliseconds of the position estimate from GPS.
 *
 * @min 0
 * @max 1000
 * @group Position Estimator
 */
PARAM_DEFINE_INT32(PE_POS_DELAY_MS, 210);

/**
 * Height estimate delay
 *
 * The delay in milliseconds of the height estimate from the barometer.
 *
 * @min 0
 * @max 1000
 * @group Position Estimator
 */
PARAM_DEFINE_INT32(PE_HGT_DELAY_MS, 350);

/**
 * Mag estimate delay
 *
 * The delay in milliseconds of the magnetic field estimate from
 * the magnetometer.
 *
 * @min 0
 * @max 1000
 * @group Position Estimator
 */
PARAM_DEFINE_INT32(PE_MAG_DELAY_MS, 30);

/**
 * True airspeeed estimate delay
 *
 * The delay in milliseconds of the airspeed estimate.
 *
 * @min 0
 * @max 1000
 * @group Position Estimator
 */
PARAM_DEFINE_INT32(PE_TAS_DELAY_MS, 210);

/**
 * GPS vs. barometric altitude update weight
 *
 * RE-CHECK this.
 *
 * @min 0.0
 * @max 1.0
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_GPS_ALT_WGT, 0.9f);

