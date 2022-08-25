/****************************************************************************
 *
 *   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file follow_target_params.c
 *
 * Parameters for follow target mode
 *
 * @author Jimmy Johnson <catch22@fastmail.net>
 * @author Junwoo Hwang <junwoo091400@gmail.com>
 */

/**
 * Responsiveness to target movement in Target Estimator
 *
 * lower values increase the responsiveness to changing position, but also ignore less noise
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Follow target
 */
PARAM_DEFINE_FLOAT(FLW_TGT_RS, 0.1f);

/**
 * Follow target height
 *
 * Following height above the target
 *
 * @unit m
 * @min 8.0
 * @group Follow target
 */
PARAM_DEFINE_FLOAT(FLW_TGT_HT, 8.0f);

/**
 * Distance to follow target from
 *
 * The distance in meters to follow the target at
 *
 * @unit m
 * @min 1.0
 * @group Follow target
 */
PARAM_DEFINE_FLOAT(FLW_TGT_DST, 8.0f);

/**
 * Follow Angle setting in degrees
 *
 * Angle to follow the target from. 0.0 Equals straight in front of the target's
 * course (direction of motion) and the angle increases in clockwise direction,
 * meaning Right-side would be 90.0 degrees while Left-side is -90.0 degrees
 *
 * Note: When the user force sets the angle out of the min/max range, it will be
 * wrapped (e.g. 480 -> 120) in the range to gracefully handle the out of range.
 *
 * @min -180.0
 * @max 180.0
 * @group Follow target
 */
PARAM_DEFINE_FLOAT(FLW_TGT_FA, 180.0f);

/**
 * Altitude control mode
 *
 * Maintain altitude or track target's altitude. When maintaining the altitude,
 * the drone can crash into terrain when the target moves uphill. When tracking
 * the target's altitude, the follow altitude FLW_TGT_HT should be high enough
 * to prevent terrain collisions due to GPS inaccuracies of the target.
 *
 * @value 0 2D Tracking: Maintain constant altitude relative to home and track XY position only
 * @value 1 2D + Terrain: Maintain constant altitude relative to terrain below and track XY position
 * @value 2 3D Tracking: Track target's altitude (be aware that GPS altitude bias usually makes this useless)
 * @group Follow target
 */
PARAM_DEFINE_INT32(FLW_TGT_ALT_M, 0);

/**
 * Maximum tangential velocity setting for generating the follow orbit trajectory
 *
 * This is the maximum tangential velocity the drone will circle around the target whenever
 * an orbit angle setpoint changes. Higher value means more aggressive follow behavior.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @group Follow target
 */
PARAM_DEFINE_FLOAT(FLW_TGT_MAX_VEL, 5.0f);
