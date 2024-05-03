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
 * Vertical thrust required to hover
 *
 * Mapped to center throttle stick in Stabilized mode (see MPC_THR_CURVE).
 * Used for initialization of the hover thrust estimator (see MPC_USE_HTE).
 * The estimated hover thrust is used as base for zero vertical acceleration in altitude control.
 * The hover thrust is important for land detection to work correctly.
 *
 * @unit norm
 * @min 0.1
 * @max 0.8
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_HOVER, 0.5f);

/**
 * Hover thrust estimator
 *
 * Disable to use the fixed parameter MPC_THR_HOVER
 * Enable to use the hover thrust estimator
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_USE_HTE, 1);

/**
 * Horizontal thrust margin
 *
 * Margin that is kept for horizontal control when higher priority vertical thrust is saturated.
 * To avoid completely starving horizontal control with high vertical error.
 *
 * @unit norm
 * @min 0
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_XY_MARG, 0.3f);

/**
 * Velocity low pass cutoff frequency
 *
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_LP, 0.0f);

/**
 * Velocity notch filter frequency
 *
 * The center frequency for the 2nd order notch filter on the velocity.
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_NF_FRQ, 0.0f);

/**
 * Velocity notch filter bandwidth
 *
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_NF_BW, 5.0f);

/**
 * Velocity derivative low pass cutoff frequency
 *
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VELD_LP, 5.0f);
