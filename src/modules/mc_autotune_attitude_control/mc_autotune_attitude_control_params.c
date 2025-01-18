/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file mc_autotune_attitude_control_params.c
 *
 * Parameters used by the attitude auto-tuner
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

/**
 * Multicopter autotune module enable
 *
 * @boolean
 * @group Autotune
 */
PARAM_DEFINE_INT32(MC_AT_EN, 0);

/**
 * Start the autotuning sequence
 *
 * WARNING: this will inject steps to the rate controller
 * and can be dangerous. Only activate if you know what you
 * are doing, and in a safe environment.
 *
 * Any motion of the remote stick will abort the signal
 * injection and reset this parameter
 * Best is to perform the identification in position or
 * hold mode.
 * Increase the amplitude of the injected signal using
 * MC_AT_SYSID_AMP for more signal/noise ratio
 *
 * @boolean
 * @group Autotune
 */
PARAM_DEFINE_INT32(MC_AT_START, 0);

/**
 * Amplitude of the injected signal
 *
 * @min 0.1
 * @max 6.0
 * @decimal 1
 * @group Autotune
 */
PARAM_DEFINE_FLOAT(MC_AT_SYSID_AMP, 0.7);

/**
 * Controls when to apply the new gains
 *
 * After the auto-tuning sequence is completed,
 * a new set of gains is available and can be applied
 * immediately or after landing.
 *
 * WARNING Applying the gains in air is dangerous as there is no
 * guarantee that those new gains will be able to stabilize
 * the drone properly.
 *
 * @value 0 Do not apply the new gains (logging only)
 * @value 1 Apply the new gains after disarm
 * @value 2 WARNING Apply the new gains in air
 * @group Autotune
 */
PARAM_DEFINE_INT32(MC_AT_APPLY, 1);

/**
 * Desired angular rate closed-loop rise time
 *
 * @min 0.01
 * @max 0.5
 * @decimal 2
 * @unit s
 * @group Autotune
 */
PARAM_DEFINE_FLOAT(MC_AT_RISE_TIME, 0.14);
