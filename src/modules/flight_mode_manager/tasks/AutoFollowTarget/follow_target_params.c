/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 */

/*
 * Follow target parameters
 */

/**
 * Minimum follow target altitude
 *
 * The minimum height in meters relative to home for following a target
 *
 * @unit m
 * @min 8.0
 * @group Follow target
 */
PARAM_DEFINE_FLOAT(NAV_FT_MIN_HT, 8.0f);

/**
 * Distance to follow target from
 *
 * The distance in meters to follow the target at
 *
 * @unit m
 * @min 1.0
 * @group Follow target
 */
PARAM_DEFINE_FLOAT(NAV_FT_DST, 8.0f);

/**
 * Side to follow target from
 *
 * The side to follow the target from
 *
 * @value 0 None (default, Behind)
 * @value 1 Behind
 * @value 2 front
 * @value 3 front right
 * @value 4 front left
 * @value 5 mid right
 * @value 6 mid left
 * @value 7 behind right
 * @value 8 behind left
 *
 * @min 0
 * @max 8
 * @group Follow target
 */
PARAM_DEFINE_INT32(NAV_FT_FS, 1);

/**
 * Dynamic filtering algorithm responsiveness to target movement in Target Estimator
 *
 * lower values increase the responsiveness to changing long lat
 * but also ignore less noise
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Follow target
 */
PARAM_DEFINE_FLOAT(NAV_FT_RS, 0.1f);

/**
 * Altitude control mode
 *
 * Maintain altitude or track target's altitude. When maintaining the altitude,
 * the drone can crash into terrain when the target moves uphill. When tracking
 * the target's altitude, the follow altitude NAV_MIN_FT_HT should be high enough
 * to prevent terrain collisions due to GPS inaccuracies of the target.
 *
 * @value 0 2D Tracking : Maintain constant altitude relative to home and track XY position only
 * @value 1 2D + Terrain : Mantain constant altitude relative to terrain below and track XY position
 * @value 2 3D Tracking : Track target's altitude (be aware that GPS altitude bias usually makes this useless)
 * @group Follow target
 */
PARAM_DEFINE_INT32(NAV_FT_ALT_M, 0);

/**
 * Gimbal tracking mode
 *
 * @value 0 2D tracking: Point at target XY coordinates, and at ground Z coordinate
 * @value 1 2D tracking with terrain: Point at target XY coordinates, and at terrain Z coordinate
 * @value 2 3D tracking: Point at target XYZ coordinates
 * @group Follow target
 */
PARAM_DEFINE_INT32(NAV_FT_GMB_M, 0);

/**
 * Yaw Setpoint Filtering Enable
 *
 * @boolean
 * @group Follow target
 */
PARAM_DEFINE_INT32(NAV_FT_YAW_FT, 0);

/**
 * Target Pose filter's natural angular rate setting [rad/s]
 *
 * The output of the Target estimator will be filtered via 2nd order filter with this natural angular rate
 *
 * @unit rad/s
 * @min 0.1
 * @decimal 1
 * @group Follow target
 */
PARAM_DEFINE_FLOAT(NAV_FT_FILT_R, 1.0f);
