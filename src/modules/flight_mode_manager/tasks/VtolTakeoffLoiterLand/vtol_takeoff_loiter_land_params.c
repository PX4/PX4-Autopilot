/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file vtol_takeoff_loiter_land_params.c
 *
 * Parameters for the VTOL takeoff, loiter, land flight mode.
 *
 */

/**
 * VTOL Loiter Radius
 *
 * Radius of the loiter circle for VTOL takeoff, loiter, land mode.
 *
 * @unit m
 * @min 10
 * @max 500
 * @decimal 1
 * @increment 5
 * @group VTOL Takeoff Loiter Land
 */
PARAM_DEFINE_FLOAT(VT_LOITER_RAD, 50.0f);

/**
 * VTOL Loiter Duration
 *
 * Duration of loiter phase in VTOL takeoff, loiter, land mode.
 *
 * @unit s
 * @min 10
 * @max 600
 * @decimal 1
 * @increment 10
 * @group VTOL Takeoff Loiter Land
 */
PARAM_DEFINE_FLOAT(VT_LOITER_TIME, 60.0f);

/**
 * VTOL Landing Altitude
 *
 * Altitude above ground for landing phase in VTOL takeoff, loiter, land mode.
 *
 * @unit m
 * @min 1
 * @max 50
 * @decimal 1
 * @increment 1
 * @group VTOL Takeoff Loiter Land
 */
PARAM_DEFINE_FLOAT(VT_LAND_ALT, 5.0f);
