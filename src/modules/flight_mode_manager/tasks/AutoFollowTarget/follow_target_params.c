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
PARAM_DEFINE_FLOAT(NAV_MIN_FT_HT, 8.0f);

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
 * none         = 0
 * behind       = 1
 * front        = 2
 * front right  = 3
 * front left   = 4
 * mid right    = 5
 * mid left     = 6
 * behind right = 7
 * behind left  = 8
 *
 * @min 0
 * @max 8
 * @group Follow target
 */
PARAM_DEFINE_INT32(NAV_FT_FS, 1);

/**
 * Dynamic filtering algorithm responsiveness to target movement
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
 * @value 0 Maintain constant altitude and track XY position only (2D tracking)
 * @value 1 Track target's altitude as well (3D tracking)
 * @group Follow target
 */
PARAM_DEFINE_INT32(NAV_FT_ALT_M, 0);

/**
 * Compensate filter delay
 *
 * The follow flight mode is estimating and filtering the target's position
 * and velocity in order to achieve a smoother tracking behavior. The filtering
 * introduces some delay which is noticable when the target is moving at higher
 * speeds, such as the case for bicycles and cars. For this high-speed scenario
 * the delay compensation can be enabled, which extrapolates the filtered position
 * in order to compensate for the filter delay.
 * This setting should not be used when the target is moving slowly, for example
 * when it's a pedestrian, as it will then emphasize the measurement noise instead
 * and cause unnecessary movement for the drone.
 *
 * @value 0 disabled
 * @value 1 enabled
 * @group Follow target
 */
PARAM_DEFINE_INT32(NAV_FT_DELC, 0);
