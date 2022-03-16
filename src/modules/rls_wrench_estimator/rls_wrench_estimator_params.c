/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file rls_wrench_estimator_params.c
 *
 * Parameters used by the RLS Identification and Wrench estimator
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 */

/**
 * Vehicle Total Mass [kg]
 *
 * Used in to estimate actuator forces applied
 * to vehicle based on accelerometer/gyro data.
 * Set to 1 if unknown.
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @unit kg
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_MASS, 1.04f);

/**
 * Rotor Tilt Angle [deg]
 *
 * Used in to estimate actuator forces applied
 * to vehicle based on accelerometer/gyro data.
 * Set to 0 for regular quadcopters.
 *
 * @decimal 5
 * @min 0.0
 * @max 45.0
 * @unit deg
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_TILT, 31.0f);

/**
 * Motor Low-Pass Filter Time Constant [sec]
 *
 * Define first-order motor dynamics
 * Input: PWM values
 * Output: Motor Speed
 *
 * @decimal 5
 * @min 0.01
 * @max 2.0
 * @unit s
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_LPF_M, 0.1f);

/**
 * Initial thrust constant guess (k_f*1e6)
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_KF_INIT, 1.5f);

/**
 * Initial thrust reduction guess (k_r*1e6)
 *
 * @decimal 5
 * @min 0.0
 * @max 50.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_KR_INIT, 0.1f);

/**
 * Confidence in thrust constant initial guess
 *
 * Set to 0 if perfect knowledge
 *
 * @decimal 5
 * @min 0.0
 * @max 100.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_KF_CONF, 0.001f);

/**
 * Confidence in thrust reduction initial guess
 *
 * Set to 0 if perfect knowledge
 *
 * @decimal 5
 * @min 0.0
 * @max 100.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_KR_CONF, 0.01f);

/**
 * Accelerometer xy-noise for RLS parameter identification
 *
 * @decimal 5
 * @min 0.01
 * @max 100.0
 * @unit m/s^2
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_XY_NOISE, 1.f);

/**
 * Accelerometer z-noise for RLS parameter identification
 *
 * @decimal 5
 * @min 0.01
 * @max 100.0
 * @unit m/s^2
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_Z_NOISE, 10.f);

/**
 * Define number of rotors for RLS. Use 4 (Quad) or 8 (Octo)
 *
 * @group RLS Wrench Estimator
 * @min 4
 * @max 8
 */
PARAM_DEFINE_INT32(RLS_EST_N_ROTORS, 8);

/**
 * PWM to speed (P1)
 *
 * (PWM*P1 - P2) = SPEED
 * This is rotor dependant.
 *
 * @decimal 6
 * @min 0.01
 * @max 10.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_SPE_P1, 1.823f);

/**
 * PWM to speed (P2)
 *
 * (PWM*P1 - P2) = SPEED
 * This is rotor dependant.
 *
 * @decimal 6
 * @min 0.01
 * @max 10000.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_SPE_P2, 1673.7f);

/**
 * PWM to speed (Voltage Correction)
 *
 * @decimal 6
 * @min 0.01
 * @max 10000.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_SPE_V1, 11.7f);

/**
 * PWM to speed (Voltage Correction)
 *
 * @decimal 6
 * @min 0.01
 * @max 10000.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_SPE_V2, 1.5f);

/**
 * Force Estimator Time Constant [sec]
 *
 *
 * @decimal 5
 * @min 0.01
 * @max 5.0
 * @unit s
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_TAU_F, 1.0f);

/**
 * Moment Estimator Time Constant [sec]
 *
 *
 * @decimal 5
 * @min 0.01
 * @max 5.0
 * @unit s
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_TAU_M, 1.0f);

/**
 * CoM x-offset initial guess [mm]
 *
 * @decimal 5
 * @min 0.0
 * @max 1000.0
 * @unit mm
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_XO_INIT, 0.f);

/**
 * CoM y-offset initial guess [mm]
 *
 * @decimal 5
 * @min 0.0
 * @max 1000.0
 * @unit mm
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_YO_INIT, 0.f);

/**
 * Confidence in x-offset initial guess
 *
 * Set to 0 if perfect knowledge
 *
 * @decimal 5
 * @min 0.001
 * @max 10000.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_XO_CONF, 1000.0f);

/**
 * Confidence in y-offset initial guess
 *
 * Set to 0 if perfect knowledge
 *
 * @decimal 5
 * @min 0.001
 * @max 10000.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_YO_CONF, 1000.0f);

/**
 * Motor output noise for RLS parameter identification
 *
 * @decimal 5
 * @min 0.001
 * @max 100.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_F_NOISE, 1.0f);

/**
 * Motor Torque Constant (k_m*1e8)
 *
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_KM, 1.4f);

/**
 * Vehicle Diameter [m]
 *
 *
 * @decimal 5
 * @min 0.01
 * @max 2.0
 * @unit m
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_D, 0.25f);

/**
 * Top Motors distance from CoM [mm]
 *
 *
 * @decimal 1
 * @min 0.0
 * @max 500.0
 * @unit mm
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_TOP_H, 35.0f);

/**
 * Bottom motors distance from CoM [mm]
 *
 *
 * @decimal 1
 * @min 0.0
 * @max 500.0
 * @unit mm
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_BOT_H, 20.0f);

/**
 * Vehicle moment of inertia about x-axis [kg m^2]*1e3
 *
 *
 * @decimal 5
 * @min 0.1
 * @max 500.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_IXX, 2.5513f);

/**
 * Vehicle moment of inertia about y-axis [kg m^2]*1e3
 *
 *
 * @decimal 5
 * @min 0.1
 * @max 500.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_IYY, 2.8425f);

/**
 * Vehicle moment of inertia about z-axis [kg m^2]*1e3
 *
 *
 * @decimal 5
 * @min 0.1
 * @max 500.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_IZZ, 4.5935f);
