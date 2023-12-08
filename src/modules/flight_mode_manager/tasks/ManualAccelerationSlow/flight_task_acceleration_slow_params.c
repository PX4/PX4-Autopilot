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
 * Manual input mapped to scale horizontal velocity in position slow mode
 *
 * @value 0 No rescaling
 * @value 1 AUX1
 * @value 2 AUX2
 * @value 3 AUX3
 * @value 4 AUX4
 * @value 5 AUX5
 * @value 6 AUX6
 * @group Multicopter Position Slow Mode
 */
PARAM_DEFINE_INT32(MC_SLOW_MAP_HVEL, 0);

/**
 * Manual input mapped to scale vertical velocity in position slow mode
 *
 * @value 0 No rescaling
 * @value 1 AUX1
 * @value 2 AUX2
 * @value 3 AUX3
 * @value 4 AUX4
 * @value 5 AUX5
 * @value 6 AUX6
 * @group Multicopter Position Slow Mode
 */
PARAM_DEFINE_INT32(MC_SLOW_MAP_VVEL, 0);

/**
 * Manual input mapped to scale yaw rate in position slow mode
 *
 * @value 0 No rescaling
 * @value 1 AUX1
 * @value 2 AUX2
 * @value 3 AUX3
 * @value 4 AUX4
 * @value 5 AUX5
 * @value 6 AUX6
 * @group Multicopter Position Slow Mode
 */
PARAM_DEFINE_INT32(MC_SLOW_MAP_YAWR, 0);

/**
 * Horizontal velocity lower limit
 *
 * The lowest input maps and is clamped to this velocity.
 *
 * @unit m/s
 * @min 0.1
 * @increment 0.1
 * @decimal 2
 * @group Multicopter Position Slow Mode
 */
PARAM_DEFINE_FLOAT(MC_SLOW_MIN_HVEL, .3f);

/**
 * Vertical velocity lower limit
 *
 * The lowest input maps and is clamped to this velocity.
 *
 * @unit m/s
 * @min 0.1
 * @increment 0.1
 * @decimal 2
 * @group Multicopter Position Slow Mode
 */
PARAM_DEFINE_FLOAT(MC_SLOW_MIN_VVEL, .3f);

/**
 * Yaw rate lower limit
 *
 * The lowest input maps and is clamped to this rate.
 *
 * @unit deg/s
 * @min 1
 * @increment 0.1
 * @decimal 0
 * @group Multicopter Position Slow Mode
 */
PARAM_DEFINE_FLOAT(MC_SLOW_MIN_YAWR, 3.f);

/**
 * Default horizontal velocity limit
 *
 * This value is used in slow mode if
 * no aux channel is mapped and
 * no limit is commanded through MAVLink.
 *
 * @unit m/s
 * @min 0.1
 * @increment 0.1
 * @decimal 2
 * @group Multicopter Position Slow Mode
 */
PARAM_DEFINE_FLOAT(MC_SLOW_DEF_HVEL, 3.f);

/**
 * Default vertical velocity limit
 *
 * This value is used in slow mode if
 * no aux channel is mapped and
 * no limit is commanded through MAVLink.
 *
 * @unit m/s
 * @min 0.1
 * @increment 0.1
 * @decimal 2
 * @group Multicopter Position Slow Mode
 */
PARAM_DEFINE_FLOAT(MC_SLOW_DEF_VVEL, 1.f);

/**
 * Default yaw rate limit
 *
 * This value is used in slow mode if
 * no aux channel is mapped and
 * no limit is commanded through MAVLink.
 *
 * @unit deg/s
 * @min 1
 * @increment 0.1
 * @decimal 0
 * @group Multicopter Position Slow Mode
 */
PARAM_DEFINE_FLOAT(MC_SLOW_DEF_YAWR, 45.f);
