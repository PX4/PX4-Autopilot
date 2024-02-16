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
 * Maximal tilt angle in Stabilized or Altitude mode
 *
 * @unit deg
 * @min 1
 * @max 70
 * @decimal 0
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_TILT_MAX, 35.f);

/**
 * Max manual yaw rate for Stabilized, Altitude, Position mode
 *
 * @unit deg/s
 * @min 0
 * @max 400
 * @decimal 0
 * @increment 10
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_Y_MAX, 150.f);

/**
 * Minimum collective thrust in Stabilized mode
 *
 * The value is mapped to the lowest throttle stick position in Stabilized mode.
 *
 * Too low collective thrust leads to loss of roll/pitch/yaw torque control authority.
 * Airmode is used to keep torque authority with zero thrust (see MC_AIRMODE).
 *
 * @unit norm
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MANTHR_MIN, 0.08f);

/**
 * Thrust curve mapping in Stabilized Mode
 *
 * This parameter defines how the throttle stick input is mapped to collective thrust
 * in Stabilized mode.
 *
 * In case the default is used ('Rescale to hover thrust'), the stick input is linearly
 * rescaled, such that a centered stick corresponds to the hover throttle (see MPC_THR_HOVER).
 *
 * Select 'No Rescale' to directly map the stick 1:1 to the output. This can be useful
 * in case the hover thrust is very low and the default would lead to too much distortion
 * (e.g. if hover thrust is set to 20%, then 80% of the upper thrust range is squeezed into the
 * upper half of the stick range).
 *
 * Note: In case MPC_THR_HOVER is set to 50%, the modes 0 and 1 are the same.
 *
 * @value 0 Rescale to hover thrust
 * @value 1 No Rescale
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_THR_CURVE, 0);
