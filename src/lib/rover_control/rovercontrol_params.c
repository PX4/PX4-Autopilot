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
 * @file rovercontrol_params.c
 *
 * Parameters defined by the rover control lib.
 */

/**
 * Yaw stick deadzone
 *
 * Percentage of stick input range that will be interpreted as zero around the stick centered value.
 *
 * @min 0
 * @max 1
 * @increment 0.01
 * @decimal 2
 * @group Rover Rate Control
 */
PARAM_DEFINE_FLOAT(RO_YAW_STICK_DZ, 0.1f);

/**
 * Yaw rate measurement threshold
 *
 * The minimum threshold for the yaw rate measurement not to be interpreted as zero.
 *
 * @unit deg/s
 * @min 0
 * @max 100
 * @increment 0.01
 * @decimal 2
 * @group Rover Rate Control
 */
PARAM_DEFINE_FLOAT(RO_YAW_RATE_TH, 3.f);

/**
 * Proportional gain for closed loop yaw rate controller
 *
 * @min 0
 * @max 100
 * @increment 0.01
 * @decimal 3
 * @group Rover Rate Control
 */
PARAM_DEFINE_FLOAT(RO_YAW_RATE_P, 0.f);

/**
 * Integral gain for closed loop yaw rate controller
 *
 * @min 0
 * @max 100
 * @increment 0.01
 * @decimal 3
 * @group Rover Rate Control
 */
PARAM_DEFINE_FLOAT(RO_YAW_RATE_I, 0.f);

/**
 * Yaw rate limit
 *
 * Used to cap yaw rate setpoints and map controller inputs to yaw rate setpoints
 * in Acro, Stabilized and Position mode.
 *
 * @unit deg/s
 * @min 0
 * @max 10000
 * @increment 0.01
 * @decimal 2
 * @group Rover Rate Control
 */
PARAM_DEFINE_FLOAT(RO_YAW_RATE_LIM, 0.f);

/**
 * Yaw acceleration limit
 *
 * Used to cap how quickly the magnitude of yaw rate setpoints can increase.
 * Set to -1 to disable.
 *
 * @unit deg/s^2
 * @min -1
 * @max 10000
 * @increment 0.01
 * @decimal 2
 * @group Rover Rate Control
 */
PARAM_DEFINE_FLOAT(RO_YAW_ACCEL_LIM, -1.f);

/**
 * Yaw deceleration limit
 *
 * Used to cap how quickly the magnitude of yaw rate setpoints can decrease.
 * Set to -1 to disable.
 *
 * @unit deg/s^2
 * @min -1
 * @max 10000
 * @increment 0.01
 * @decimal 2
 * @group Rover Rate Control
 */
PARAM_DEFINE_FLOAT(RO_YAW_DECEL_LIM, -1.f);

/**
 * Proportional gain for closed loop yaw controller
 *
 * @min 0
 * @max 100
 * @increment 0.01
 * @decimal 3
 * @group Rover Attitude Control
 */
PARAM_DEFINE_FLOAT(RO_YAW_P, 0.f);

/**
 * Speed the rover drives at maximum throttle
 *
 * Used to linearly map speeds [m/s] to throttle values [-1. 1].
 *
 * @min 0
 * @max 100
 * @unit m/s
 * @increment 0.01
 * @decimal 2
 * @group Rover Velocity Control
 */
PARAM_DEFINE_FLOAT(RO_MAX_THR_SPEED, 0.f);

/**
 * Proportional gain for ground speed controller
 *
 * @min 0
 * @max 100
 * @increment 0.01
 * @decimal 2
 * @group Rover Velocity Control
 */
PARAM_DEFINE_FLOAT(RO_SPEED_P, 0.f);

/**
 * Integral gain for ground speed controller
 *
 * @min 0
 * @max 100
 * @increment 0.001
 * @decimal 3
 * @group Rover Velocity Control
 */
PARAM_DEFINE_FLOAT(RO_SPEED_I, 0.f);

/**
 * Speed limit
 *
 * Used to cap speed setpoints and map controller inputs to speed setpoints in Position mode.
 *
 * @unit m/s
 * @min -1
 * @max 100
 * @increment 0.01
 * @decimal 2
 * @group Rover Velocity Control
 */
PARAM_DEFINE_FLOAT(RO_SPEED_LIM, -1.f);

/**
 * Acceleration limit
 *
 * Set to -1 to disable.
 * For mecanum rovers this limit is used for longitudinal and lateral acceleration.
 *
 * @unit m/s^2
 * @min -1
 * @max 100
 * @increment 0.01
 * @decimal 2
 * @group Rover Velocity Control
 */
PARAM_DEFINE_FLOAT(RO_ACCEL_LIM, -1.f);

/**
 * Deceleration limit
 *
 * Set to -1 to disable.
 * Note that if it is disabled the rover will not slow down when approaching waypoints in auto modes.
 * For mecanum rovers this limit is used for longitudinal and lateral deceleration.
 *
 * @unit m/s^2
 * @min -1
 * @max 100
 * @increment 0.01
 * @decimal 2
 * @group Rover Velocity Control
 */
PARAM_DEFINE_FLOAT(RO_DECEL_LIM, -1.f);

/**
 * Jerk limit
 *
 * Set to -1 to disable.
 * Note that if it is disabled the rover will not slow down when approaching waypoints in auto modes.
 * For mecanum rovers this limit is used for longitudinal and lateral jerk.
 *
 * @unit m/s^3
 * @min -1
 * @max 100
 * @increment 0.01
 * @decimal 2
 * @group Rover Velocity Control
 */
PARAM_DEFINE_FLOAT(RO_JERK_LIM, -1.f);

/**
 * Speed measurement threshold
 *
 * Set to -1 to disable.
 * The minimum threshold for the speed measurement not to be interpreted as zero.
 *
 * @unit m/s
 * @min 0
 * @max 100
 * @increment 0.01
 * @decimal 2
 * @group Rover Velocity Control
 */
PARAM_DEFINE_FLOAT(RO_SPEED_TH, 0.1f);
