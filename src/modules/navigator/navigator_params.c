/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file navigator_params.c
 *
 * Parameters for navigator in general
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>

/**
 * Loiter radius (FW only)
 *
 * Default value of loiter radius for missions, loiter, RTL, etc. (fixedwing only).
 *
 * @unit meters
 * @min 0.0
 * @group Mission
 */
PARAM_DEFINE_FLOAT(NAV_LOITER_RAD, 50.0f);

/**
 * Acceptance Radius
 *
 * Default acceptance radius, overridden by acceptance radius of waypoint if set.
 *
 * @unit meters
 * @min 1.0
 * @group Mission
 */
PARAM_DEFINE_FLOAT(NAV_ACC_RAD, 25.0f);

/**
 * Set OBC mode for data link loss
 *
 * If set to 1 the behaviour on data link loss is set to a mode according to the OBC rules
 *
 * @min 0
 * @group Mission
 */
PARAM_DEFINE_INT32(NAV_DLL_OBC, 0);

/**
 * Set OBC mode for rc loss
 *
 * If set to 1 the behaviour on data link loss is set to a mode according to the OBC rules
 *
 * @min 0
 * @group Mission
 */
PARAM_DEFINE_INT32(NAV_RCL_OBC, 0);

/**
 * Airfield home Lat
 *
 * Latitude of airfield home waypoint
 *
 * @unit degrees * 1e7
 * @min 0.0
 * @group DLL
 */
PARAM_DEFINE_INT32(NAV_AH_LAT, -265847810);

/**
 * Airfield home Lon
 *
 * Longitude of airfield home waypoint
 *
 * @unit degrees * 1e7
 * @min 0.0
 * @group DLL
 */
PARAM_DEFINE_INT32(NAV_AH_LON, 1518423250);

/**
 * Airfield home alt
 *
 * Altitude of airfield home waypoint
 *
 * @unit m
 * @min 0.0
 * @group DLL
 */
PARAM_DEFINE_FLOAT(NAV_AH_ALT, 600.0f);
