/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file parameters.c
 * Parameter definition for ekf2.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include <systemlib/param/param.h>

/**
* Magnetometer measurement delay
*
* @group EKF2
* @min 0
* @max 300
* @unit ms
*/
PARAM_DEFINE_FLOAT(EKF2_MAG_DELAY, 0);

/**
 * Barometer measurement delay
 *
 * @group EKF2
 * @min 0
 * @max 300
 * @unit ms
 */
PARAM_DEFINE_FLOAT(EKF2_BARO_DELAY, 0);

/**
 * GPS measurement delay
 *
 * @group EKF2
 * @min 0
 * @max 300
 * @unit ms
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_DELAY, 200);

/**
 * Airspeed measurement delay
 *
 * @group EKF2
 * @min 0
 * @max 300
 * @unit ms
 */
PARAM_DEFINE_FLOAT(EKF2_ASP_DELAY, 200);

/**
 * Required EPH to use GPS.
 *
 * @group EKF2
 * @min 2
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_EPH, 10);

/**
 * Required EPV to use GPS.
 *
 * @group EKF2
 * @min 2
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_EPV, 20);

/**
 * Gyro noise.
 *
 * @group EKF2
 * @min 0.0001
 * @max 0.05
 */
PARAM_DEFINE_FLOAT(EKF2_G_NOISE, 0.001f);

/**
 * Process noise for delta velocity prediction.
 *
 * @group EKF2
 * @min 0.01
 * @max 1
 * @unit
 */
PARAM_DEFINE_FLOAT(EKF2_ACC_NOISE, 0.1f);

/**
 * Process noise for delta angle bias prediction.
 *
 * @group EKF2
 * @min 0
 * @max 0.0001
 * @unit
 */
PARAM_DEFINE_FLOAT(EKF2_GB_NOISE, 1e-5f);

/**
 * Process noise for delta velocity z bias prediction.
 *
 * @group EKF2
 * @min 0.000001
 * @max 0.01
 * @unit
 */
PARAM_DEFINE_FLOAT(EKF2_ACCB_NOISE, 1e-3f);

/**
 * Process noise for delta angle scale prediction.
 *
 * @group EKF2
 * @min 0.000001
 * @max 0.01
 * @unit
 */
PARAM_DEFINE_FLOAT(EKF2_GS_NOISE, 1e-4f);

/**
 * Process noise for earth magnetic field and bias prediction.
 *
 * @group EKF2
 * @min 0.0001
 * @max 0.1
 * @unit
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_NOISE, 1e-2f);

/**
 * Process noise for wind velocity prediction.
 *
 * @group EKF2
 * @min 0.01
 * @max 1
 * @unit
 */
PARAM_DEFINE_FLOAT(EKF2_WIND_NOISE, 0.05f);

/**
 * Measurement noise for gps velocity.
 *
 * @group EKF2
 * @min 0.001
 * @max 0.5
 * @unit
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_V_NOISE, 0.05f);

/**
 * Measurement noise for gps position.
 *
 * @group EKF2
 * @min 0.01
 * @max 5
 * @unit
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_P_NOISE, 1.0f);

/**
 * Measurement noise for barometer.
 *
 * @group EKF2
 * @min 0.001
 * @max 1
 * @unit
 */
PARAM_DEFINE_FLOAT(EKF2_BARO_NOISE, 0.1f);

/**
 * Measurement noise for mag heading fusion.
 *
 * @group EKF2
 * @min 0.0001
 * @max 0.1
 * @unit
 */
PARAM_DEFINE_FLOAT(EKF2_HEAD_NOISE, 3e-2f);

/**
 * Magnetic declination
 *
 * @group EKF2
 * @unit degrees
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_DECL, 0);

/**
 * Gate for maginetic heading fusion
 *
 * @group EKF2
 */
PARAM_DEFINE_FLOAT(EKF2_H_INOV_GATE, 0.5f);
