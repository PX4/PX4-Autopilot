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
 * @file ekf_att_pos_estimator_params.c
 *
 * Parameters defined by the attitude and position estimator task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

/*
 * Estimator parameters, accessible via MAVLink
 *
 */

/**
 * Velocity estimate delay
 *
 * The delay in milliseconds of the velocity estimate from GPS.
 *
 * @unit ms
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
 * @unit ms
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
 * @unit ms
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
 * @unit ms
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
 * @unit ms
 * @min 0
 * @max 1000
 * @group Position Estimator
 */
PARAM_DEFINE_INT32(PE_TAS_DELAY_MS, 210);

/**
 * GPS vs. barometric altitude update weight
 *
 * RE-CHECK this.
 * @min 0.0
 * @max 1.0
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_GPS_ALT_WGT, 0.9f);

/**
 * Airspeed measurement noise.
 *
 * Increasing this value will make the filter trust this sensor
 * less and trust other sensors more.
 *
 * @min 0.5
 * @max 5.0
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_EAS_NOISE, 1.4f);

/**
 * Velocity measurement noise in north-east (horizontal) direction.
 *
 * Generic default: 0.3, multicopters: 0.5, ground vehicles: 0.5
 *
 * @min 0.05
 * @max 5.0
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_VELNE_NOISE, 0.3f);

/**
 * Velocity noise in down (vertical) direction
 *
 * Generic default: 0.3, multicopters: 0.4, ground vehicles: 0.7
 *
 * @min 0.2
 * @max 3.0
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_VELD_NOISE, 0.3f);

/**
 * Position noise in north-east (horizontal) direction
 *
 * Generic defaults: 0.5, multicopters: 0.5, ground vehicles: 0.5
 *
 * @min 0.1
 * @max 10.0
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_POSNE_NOISE, 0.5f);

/**
 * Position noise in down (vertical) direction
 *
 * Generic defaults: 1.25, multicopters: 1.0, ground vehicles: 1.0
 *
 * @min 0.5
 * @max 3.0
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_POSD_NOISE, 1.25f);

/**
 * Magnetometer measurement noise
 *
 * Generic defaults: 0.05, multicopters: 0.05, ground vehicles: 0.05
 *
 * @min 0.01
 * @max 1.0
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_MAG_NOISE, 0.05f);

/**
 * Gyro process noise
 *
 * Generic defaults: 0.015, multicopters: 0.015, ground vehicles: 0.015.
 * This noise controls how much the filter trusts the gyro measurements.
 * Increasing it makes the filter trust the gyro less and other sensors more.
 *
 * @min 0.001
 * @max 0.05
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_GYRO_PNOISE, 0.015f);

/**
 * Accelerometer process noise
 *
 * Generic defaults: 0.25, multicopters: 0.25, ground vehicles: 0.25.
 * Increasing this value makes the filter trust the accelerometer less
 * and other sensors more.
 *
 * @min 0.05
 * @max 1.0
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_ACC_PNOISE, 0.125f);

/**
 * Gyro bias estimate process noise
 *
 * Generic defaults: 1e-07f, multicopters: 1e-07f, ground vehicles: 1e-07f.
 * Increasing this value will make the gyro bias converge faster but noisier.
 *
 * @min 0.00000005
 * @max 0.00001
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_GBIAS_PNOISE, 1e-07f);

/**
 * Accelerometer bias estimate process noise
 *
 * Generic defaults: 0.00001f, multicopters: 0.00001f, ground vehicles: 0.00001f.
 * Increasing this value makes the bias estimation faster and noisier.
 *
 * @min 0.00001
 * @max 0.001
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_ABIAS_PNOISE, 1e-05f);

/**
 * Magnetometer earth frame offsets process noise
 *
 * Generic defaults: 0.0001, multicopters: 0.0001, ground vehicles: 0.0001.
 * Increasing this value makes the magnetometer earth bias estimate converge
 * faster but also noisier.
 *
 * @min 0.0001
 * @max 0.01
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_MAGE_PNOISE, 0.0003f);

/**
 * Magnetometer body frame offsets process noise
 *
 * Generic defaults: 0.0003, multicopters: 0.0003, ground vehicles: 0.0003.
 * Increasing this value makes the magnetometer body bias estimate converge faster
 * but also noisier.
 *
 * @min 0.0001
 * @max 0.01
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_MAGB_PNOISE, 0.0003f);

/**
 * Magnetometer X bias
 *
 * The magnetometer bias. This bias is learnt by the filter over time and
 * persists between boots.
 *
 * @min -0.6
 * @max 0.6
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_MAGB_X, 0.0f);

/**
 * Magnetometer Y bias
 *
 * The magnetometer bias. This bias is learnt by the filter over time and
 * persists between boots.
 *
 * @min -0.6
 * @max 0.6
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_MAGB_Y, 0.0f);

/**
 * Magnetometer Z bias
 *
 * The magnetometer bias. This bias is learnt by the filter over time and
 * persists between boots.
 *
 * @min -0.6
 * @max 0.6
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_MAGB_Z, 0.0f);

/**
 * Threshold for filter initialization.
 *
 * If the standard deviation of the GPS position estimate is below this threshold
 * in meters, the filter will initialize.
 *
 * @min 0.3
 * @max 10.0
 * @group Position Estimator
 */
PARAM_DEFINE_FLOAT(PE_POSDEV_INIT, 5.0f);
