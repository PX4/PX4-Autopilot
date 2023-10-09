/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file mc_acro_params.c
 *
 * Parameters for Acro mode behavior
 */

/**
 * Acro mode maximum roll rate
 *
 * Full stick deflection leads to this rate.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Acro Mode
 */
PARAM_DEFINE_FLOAT(MC_ACRO_R_MAX, 100.f);

/**
 * Acro mode maximum pitch rate
 *
 * Full stick deflection leads to this rate.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Acro Mode
 */
PARAM_DEFINE_FLOAT(MC_ACRO_P_MAX, 100.f);

/**
 * Acro mode maximum yaw rate
 *
 * Full stick deflection leads to this rate.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Acro Mode
 */
PARAM_DEFINE_FLOAT(MC_ACRO_Y_MAX, 100.f);

/**
 * Acro mode roll, pitch expo factor
 *
 * Exponential factor for tuning the input curve shape.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter Acro Mode
 */
PARAM_DEFINE_FLOAT(MC_ACRO_EXPO, 0.f);

/**
 * Acro mode yaw expo factor
 *
 * Exponential factor for tuning the input curve shape.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter Acro Mode
 */
PARAM_DEFINE_FLOAT(MC_ACRO_EXPO_Y, 0.f);

/**
 * Acro mode roll, pitch super expo factor
 *
 * "Superexponential" factor for refining the input curve shape tuned using MC_ACRO_EXPO.
 *
 * 0 Pure Expo function
 * 0.7 reasonable shape enhancement for intuitive stick feel
 * 0.95 very strong bent input curve only near maxima have effect
 *
 * @min 0
 * @max 0.95
 * @decimal 2
 * @group Multicopter Acro Mode
 */
PARAM_DEFINE_FLOAT(MC_ACRO_SUPEXPO, 0.f);

/**
 * Acro mode yaw super expo factor
 *
 * "Superexponential" factor for refining the input curve shape tuned using MC_ACRO_EXPO_Y.
 *
 * 0 Pure Expo function
 * 0.7 reasonable shape enhancement for intuitive stick feel
 * 0.95 very strong bent input curve only near maxima have effect
 *
 * @min 0
 * @max 0.95
 * @decimal 2
 * @group Multicopter Acro Mode
 */
PARAM_DEFINE_FLOAT(MC_ACRO_SUPEXPOY, 0.f);
