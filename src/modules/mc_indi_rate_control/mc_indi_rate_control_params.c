/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file mc_indi_rate_control_params.c
 *
 * Parameters for INDI rate controller
 * Note many params defined in the Multicopter Params file are used within INDI rate control
 */

/**
 * INDI Rate Control Enable
 *
 * This parameter enables the INDI rate control.
 *
 * @min 0
 * @max 1
 * @decimal 0
 * @increment 1
 */
PARAM_DEFINE_INT32(MC_INDI_EN, 1);

/**
 * Incremental Nonlinear Dynamic Inversion (INDI) Adaptation Enable
 *
 * This parameter enables the adaptation of the INDI effectiveness matrix.
 *
 * @min 0
 * @max 1
 * @decimal 0
 * @increment 1
 */
PARAM_DEFINE_INT32(MC_INDI_ADAPT_EN, 0);

/**
 * Incremental Nonlinear Dynamic Inversion (INDI) G1 Roll Gain
 *
 * This gain is applied to the squared rotor speeds to calculate the aerodynamic torque contribution for the roll axis.
 *
 * @min 0.0
 * @max 0.1
 * @decimal 6
 * @increment 0.000001
 * @group Multicopter INDI Rate Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_G1_ROLL, 0.00002f);

/**
 * Incremental Nonlinear Dynamic Inversion (INDI) G1 Pitch Gain
 *
 * This gain is applied to the squared rotor speeds to calculate the aerodynamic torque contribution for the pitch axis.
 *
 * @min 0.0
 * @max 0.1
 * @decimal 6
 * @increment 0.000001
 * @group Multicopter INDI Rate Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_G1_PITCH, 0.00002f);

/**
 * Incremental Nonlinear Dynamic Inversion (INDI) G1 Yaw Gain
 *
 * This gain is applied to the squared rotor speeds to calculate the aerodynamic torque contribution for the yaw axis.
 *
 * @min 0.0
 * @max 0.1
 * @decimal 6
 * @increment 0.000001
 * @group Multicopter INDI Rate Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_G1_YAW, 0.00002f);

/**
 * Incremental Nonlinear Dynamic Inversion (INDI) G2 Roll Gain
 *
 * This gain is applied to the rotor acceleration to calculate the inertial torque contribution for the roll axis.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 6
 * @increment 0.000001
 * @group Multicopter INDI Rate Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_G2_ROLL, 1.0f);

/**
 * Incremental Nonlinear Dynamic Inversion (INDI) G2 Pitch Gain
 *
 * This gain is applied to the rotor acceleration to calculate the inertial torque contribution for the pitch axis.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 6
 * @increment 0.000001
 * @group Multicopter INDI Rate Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_G2_PITCH, 1.0f);

/**
 * Incremental Nonlinear Dynamic Inversion (INDI) G2 Yaw Gain
 *
 * This gain is applied to the rotor acceleration to calculate the inertial torque contribution for the yaw axis.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 6
 * @increment 0.000001
 * @group Multicopter INDI Rate Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_G2_YAW, 0.00001f);
