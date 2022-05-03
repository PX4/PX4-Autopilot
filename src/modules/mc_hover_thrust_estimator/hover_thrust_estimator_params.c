/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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
 * @file hover_thrust_estimator_params.c
 *
 * Parameters used by the hover thrust estimator
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 */

/**
 * Hover thrust process noise
 *
 * Reduce to make the hover thrust estimate
 * more stable, increase if the real hover thrust
 * is expected to change quickly over time.
 *
 * @decimal 4
 * @min 0.0001
 * @max 1.0
 * @unit normalized_thrust/s
 * @group Hover Thrust Estimator
 */
PARAM_DEFINE_FLOAT(HTE_HT_NOISE, 0.0036);

/**
 * Gate size for acceleration fusion
 *
 * Sets the number of standard deviations used
 * by the innovation consistency test.
 *
 * @decimal 1
 * @min 1.0
 * @max 10.0
 * @unit SD
 * @group Hover Thrust Estimator
 */
PARAM_DEFINE_FLOAT(HTE_ACC_GATE, 3.0);

/**
 * 1-sigma initial hover thrust uncertainty
 *
 * Sets the number of standard deviations used
 * by the innovation consistency test.
 *
 * @decimal 3
 * @min 0.0
 * @max 1.0
 * @unit normalized_thrust
 * @group Hover Thrust Estimator
 */
PARAM_DEFINE_FLOAT(HTE_HT_ERR_INIT, 0.1);

/**
 * Horizontal velocity threshold for sensitivity reduction
 *
 * Above this speed, the measurement noise is linearly increased
 * to reduce the sensitivity of the estimator from biased measurement.
 *
 * Set to a low value on vehicles with large lifting surfaces.
 *
 * @decimal 1
 * @min 1.0
 * @max 20.0
 * @unit m/s
 * @group Hover Thrust Estimator
 */
PARAM_DEFINE_FLOAT(HTE_VXY_THR, 10.0);

/**
 * Vertical velocity threshold for sensitivity reduction
 *
 * Above this speed, the measurement noise is linearly increased
 * to reduce the sensitivity of the estimator from biased measurement.
 *
 * Set to a low value on vehicles affected by air drag when climbing or descending.
 *
 * @decimal 1
 * @min 1.0
 * @max 10.0
 * @unit m/s
 * @group Hover Thrust Estimator
 */
PARAM_DEFINE_FLOAT(HTE_VZ_THR, 2.0);

/**
 * Max deviation from MPC_THR_HOVER
 *
 * Defines the range of the hover thrust estimate around MPC_THR_HOVER.
 * A value of 0.2 with MPC_THR_HOVER at 0.5 results in a range of [0.3, 0.7].
 *
 * Set to a large value if the vehicle operates in varying physical conditions that
 * affect the required hover thrust strongly (e.g. differently sized payloads).
 *
 * @decimal 2
 * @min 0.01
 * @max 0.4
 * @unit normalized_thrust
 * @group Hover Thrust Estimator
 */
PARAM_DEFINE_FLOAT(HTE_THR_RANGE, 0.2);
