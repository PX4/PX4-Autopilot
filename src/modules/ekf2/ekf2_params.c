/****************************************************************************
 *
 *   Copyright (c) 2015-2016 Estimation and Control Library (ECL). All rights reserved.
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
 * Integer bitmask controlling GPS checks.
 * 
 * Set bits to 1 to enable checks. Checks enabled by the following bit positions
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
 */
PARAM_DEFINE_INT32(EKF2_REQ_NSATS, 6);

/**
 * Required GDoP to use GPS.
 *
 * @group EKF2
 * @min 1.5
 * @max 5.0
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
 * @max 0.1
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(EKF2_GYR_NOISE, 6.0e-2f);

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
PARAM_DEFINE_FLOAT(EKF2_GYR_B_NOISE, 2.5e-6f);

/**
 * Process noise for delta velocity z bias prediction.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.01
 * @unit m/s/s
 */
PARAM_DEFINE_FLOAT(EKF2_ACC_B_NOISE, 3.0e-5f);

/**
 * Process noise for delta angle scale factor prediction.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.01
 */
PARAM_DEFINE_FLOAT(EKF2_GYR_S_NOISE, 3.0e-4f);

/**
 * Process noise for body magnetic field prediction.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.1
 * @unit Gauss/s
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_B_NOISE, 5.0e-4f);

/**
 * Process noise for earth magnetic field prediction.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.1
 * @unit Gauss/s
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_E_NOISE, 2.5e-3f);

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
PARAM_DEFINE_FLOAT(EKF2_GPS_P_NOISE, 0.5f);

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
PARAM_DEFINE_FLOAT(EKF2_HEAD_NOISE, 0.3f);

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
 * Measurement noise for airspeed fusion.
 *
 * @group EKF2
 * @min 0.5
 * @max 5.0
 * @unit m/s
 */
 PARAM_DEFINE_FLOAT(EKF2_EAS_NOISE, 1.4f);

/**
 * Magnetic declination
 *
 * @group EKF2
 * @unit deg
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_DECL, 0);

/**
 * Gate size for magnetic heading fusion
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_HDG_GATE, 2.6f);

/**
 * Gate size for magnetometer XYZ component fusion
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_GATE, 3.0f);

/**
 * Integer bitmask controlling handling of magnetic declination.
 *
 * Set bits in the following positions to enable functions.
 * 0 : Set to true to use the declination from the geo_lookup library when the GPS position becomes available, set to false to always use the EKF2_MAG_DECL value.
 * 1 : Set to true to save the EKF2_MAG_DECL parameter to the value returned by the EKF when the vehicle disarms.
 * 2 : Set to true to always use the declination as an observaton when 3-axis magnetometer fusion is being used.
 *
 * @group EKF2
 * @min 0
 * @max 7
 */
PARAM_DEFINE_INT32(EKF2_DECL_TYPE, 7);

/**
 * Type of magnetometer fusion
 *
 * Integer controlling the type of magnetometer fusion used - magnetic heading or 3-axis magnetometer.
 * If set to automatic: heading fusion on-ground and 3-axis fusion in-flight
 * 
 * @group EKF2
 * @value 0 Automatic
 * @value 1 Magnetic heading
 * @value 2 3-axis fusion
 * @value 3 2-D projection
 * @value 4 None
 */
PARAM_DEFINE_INT32(EKF2_MAG_TYPE, 0);

/**
 * Gate size for barometric height fusion
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_BARO_GATE, 5.0f);

/**
 * Gate size for GPS horizontal position fusion
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_P_GATE, 5.0f);

/**
 * Gate size for GPS velocity fusion
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_V_GATE, 5.0f);

/**
 * Replay mode
 *
 * A value of 1 indicates that the ekf2 module will publish
 * replay messages for logging.
 *
 * @group EKF2
 * @boolean
 */
PARAM_DEFINE_INT32(EKF2_REC_RPL, 0);

/**
 * Integer bitmask controlling which external aiding sources will be used.
 *
 * Set bits in the following positions to enable:
 * 0 : Set to true to use GPS data if available
 * 1 : Set to true to use optical flow data if available
 *
 * @group EKF2
 * @min 0
 * @max 3
 */
PARAM_DEFINE_INT32(EKF2_AID_MASK, 1);

/**
 * Determines the primary source of height data used by the EKF.
 *
 * The range sensor option should only be used when for operation over a flat surface as the local NED origin will move up and down with ground level.
 *
 * @group EKF2
 * @value 0 Barometric pressure
 * @value 1 Reserved (GPS)
 * @value 2 Range sensor
 */
PARAM_DEFINE_INT32(EKF2_HGT_MODE, 0);

/**
 * Measurement noise for range finder fusion
 *
 * @group EKF2
 * @min 0.01
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_NOISE, 0.1f);

/**
 * Gate size for range finder fusion
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_GATE, 5.0f);

/**
 * Minimum valid range for the range finder
 *
 * @group EKF2
 * @min 0.01
 * @unit m
 */
PARAM_DEFINE_FLOAT(EKF2_MIN_RNG, 0.1f);

/**
 * Measurement noise for the optical flow sensor when it's reported quality metric is at the maximum
 *
 * @group EKF2
 * @min 0.05
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(EKF2_OF_N_MIN, 0.15f);

/**
 * Measurement noise for the optical flow sensor.
 *
 * (when it's reported quality metric is at the minimum set by EKF2_OF_QMIN).
 * The following condition must be met: EKF2_OF_N_MAXN >= EKF2_OF_N_MIN
 *
 * @group EKF2
 * @min 0.05
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(EKF2_OF_N_MAX, 0.5f);

/**
 * Optical Flow data will only be used if the sensor reports a quality metric >= EKF2_OF_QMIN.
 *
 * @group EKF2
 * @min 0
 * @max 255
 */
PARAM_DEFINE_INT32(EKF2_OF_QMIN, 1);

/**
 * Gate size for optical flow fusion
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 */
PARAM_DEFINE_FLOAT(EKF2_OF_GATE, 3.0f);

/**
 * Optical Flow data will not fused if the magnitude of the flow rate > EKF2_OF_RMAX
 *
 * @group EKF2
 * @min 1.0
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(EKF2_OF_RMAX, 2.5f);

/**
 * Terrain altitude process noise - accounts for instability in vehicle height estimate
 *
 * @group EKF2
 * @min 0.5
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_TERR_NOISE, 5.0f);

/**
 * Magnitude of terrain gradient
 *
 * @group EKF2
 * @min 0.0
 * @unit m/m
 */
PARAM_DEFINE_FLOAT(EKF2_TERR_GRAD, 0.5f);
