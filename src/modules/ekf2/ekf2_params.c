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
* Magnetometer measurement delay relative to IMU measurements
*
* @group EKF2
* @min 0
* @max 300
* @unit ms
*/
PARAM_DEFINE_FLOAT(EKF2_MAG_DELAY, 0);

/**
 * Barometer measurement delay relative to IMU measurements
 *
 * @group EKF2
 * @min 0
 * @max 300
 * @unit ms
 */
PARAM_DEFINE_FLOAT(EKF2_BARO_DELAY, 0);

/**
 * GPS measurement delay relative to IMU measurements
 *
 * @group EKF2
 * @min 0
 * @max 300
 * @unit ms
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_DELAY, 200);

/**
 * Airspeed measurement delay relative to IMU measurements
 *
 * @group EKF2
 * @min 0
 * @max 300
 * @unit ms
 */
PARAM_DEFINE_FLOAT(EKF2_ASP_DELAY, 200);

/**
 * Integer bitmask controlling GPS checks. Set bits to 1 to enable checks. Checks enabled by the following bit positions
 * 0 : Minimum required sat count set by EKF2_REQ_NSATS
 * 1 : Minimum required GDoP set by EKF2_REQ_GDOP
 * 2 : Maximum allowed horizontal position error set by EKF2_REQ_EPH
 * 3 : Maximum allowed vertical position error set by EKF2_REQ_EPV
 * 4 : Maximum allowed speed error set by EKF2_REQ_SACC
 * 5 : Maximum allowed horizontal position rate set by EKF2_REQ_HDRIFT. This check can only be used if the vehciel is stationary during alignment.
 * 6 : Maximum allowed vertical position rate set by EKF2_REQ_VDRIFT. This check can only be used if the vehciel is stationary during alignment.
 * 7 : Maximum allowed horizontal speed set by EKF2_REQ_HDRIFT. This check can only be used if the vehciel is stationary during alignment.
 * 8 : Maximum allowed vertical velocity discrepancy set by EKF2_REQ_VDRIFT
 *
 * @group EKF2
 * @min 0
 * @max 511
 * @unit
 */
PARAM_DEFINE_INT32(EKF2_GPS_CHECK, 21);

/**
 * Required EPH to use GPS.
 *
 * @group EKF2
 * @min 2
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_EPH, 5.0f);

/**
 * Required EPV to use GPS.
 *
 * @group EKF2
 * @min 2
 * @max 100
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_EPV, 8.0f);

/**
 * Required speed accuracy to use GPS.
 *
 * @group EKF2
 * @min 0.5
 * @max 5.0
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_SACC, 1.0f);

/**
 * Required satellite count to use GPS.
 *
 * @group EKF2
 * @min 4
 * @max 12
 * @unit
 */
PARAM_DEFINE_INT32(EKF2_REQ_NSATS, 6);

/**
 * Required GDoP to use GPS.
 *
 * @group EKF2
 * @min 1.5
 * @max 5.0
 * @unit
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_GDOP, 2.5f);

/**
 * Maximum horizontal drift speed to use GPS.
 *
 * @group EKF2
 * @min 0.1
 * @max 1.0
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_HDRIFT, 0.3f);

/**
 * Maximum vertical drift speed to use GPS.
 *
 * @group EKF2
 * @min 0.1
 * @max 1.5
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_VDRIFT, 0.5f);

/**
 * Rate gyro noise for covariance prediction.
 *
 * @group EKF2
 * @min 0.0001
 * @max 0.01
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(EKF2_GYR_NOISE, 1.0e-3f);

/**
 * Accelerometer noise for covariance prediction.
 *
 * @group EKF2
 * @min 0.01
 * @max 1.0
 * @unit m/s/s
 */
PARAM_DEFINE_FLOAT(EKF2_ACC_NOISE, 0.25f);

/**
 * Process noise for delta angle bias prediction.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.0001
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(EKF2_GYR_B_NOISE, 7.0e-5f);

/**
 * Process noise for delta velocity z bias prediction.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.01
 * @unit m/s/s
 */
PARAM_DEFINE_FLOAT(EKF2_ACC_B_NOISE, 1.0e-4f);

/**
 * Process noise for delta angle scale factor prediction.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.01
 * @unit None
 */
PARAM_DEFINE_FLOAT(EKF2_GYR_S_NOISE, 3.0e-3f);

/**
 * Process noise for sensor bias and earth magnetic field prediction.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.1
 * @unit Gauss/s
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_B_NOISE, 2.5e-2f);

/**
 * Process noise for wind velocity prediction.
 *
 * @group EKF2
 * @min 0.0
 * @max 1.0
 * @unit m/s/s
 */
PARAM_DEFINE_FLOAT(EKF2_WIND_NOISE, 1.0e-1f);

/**
 * Measurement noise for gps horizontal velocity.
 *
 * @group EKF2
 * @min 0.01
 * @max 5.0
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_V_NOISE, 0.5f);

/**
 * Measurement noise for gps position.
 *
 * @group EKF2
 * @min 0.01
 * @max 10.0
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_P_NOISE, 1.0f);

/**
 * Measurement noise for non-aiding position hold.
 *
 * @group EKF2
 * @min 0.5
 * @max 50.0
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_NOAID_NOISE, 10.0f);

/**
 * Measurement noise for barometric altitude.
 *
 * @group EKF2
 * @min 0.01
 * @max 15.0
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_BARO_NOISE, 3.0f);

/**
 * Measurement noise for magnetic heading fusion.
 *
 * @group EKF2
 * @min 0.01
 * @max 1.0
 * @unit rad
 */
PARAM_DEFINE_FLOAT(EKF2_HEAD_NOISE, 0.17f);

/**
 * Measurement noise for magnetometer 3-axis fusion.
 *
 * @group EKF2
 * @min 0.001
 * @max 1.0
 * @unit Gauss
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_NOISE, 5.0e-2f);

/**
 * Magnetic declination
 *
 * @group EKF2
 * @unit degrees
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_DECL, 0);

/**
 * Gate size for magnetic heading fusion
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_HDG_GATE, 3.0f);

/**
 * Gate size for magnetometer XYZ component fusion
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_GATE, 3.0f);

/**
 * Gate size for barometric height fusion
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_BARO_GATE, 3.0f);

/**
 * Gate size for GPS horizontal position fusion
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_P_GATE, 3.0f);

/**
 * Gate size for GPS velocity fusion
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_V_GATE, 3.0f);
