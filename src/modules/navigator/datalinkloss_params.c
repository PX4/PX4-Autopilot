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
 * @file datalinkloss_params.c
 *
 * Parameters for DLL
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>

/*
 * Data Link Loss parameters, accessible via MAVLink
 */

/**
 * Comms hold wait time
 *
 * The amount of time in seconds the system should wait at the comms hold waypoint
 *
 * @unit seconds
 * @min 0.0
 * @group DLL
 */
PARAM_DEFINE_FLOAT(NAV_DLL_CH_T, 120.0f);

/**
 * Comms hold Lat
 *
 * Latitude of comms hold waypoint
 *
 * @unit degrees * 1e7
 * @min 0.0
 * @group DLL
 */
PARAM_DEFINE_INT32(NAV_DLL_CH_LAT, -266072120);

/**
 * Comms hold Lon
 *
 * Longitude of comms hold waypoint
 *
 * @unit degrees * 1e7
 * @min 0.0
 * @group DLL
 */
PARAM_DEFINE_INT32(NAV_DLL_CH_LON, 1518453890);

/**
 * Comms hold alt
 *
 * Altitude of comms hold waypoint
 *
 * @unit m
 * @min 0.0
 * @group DLL
 */
PARAM_DEFINE_FLOAT(NAV_DLL_CH_ALT, 600.0f);

/**
 * Aifield hole wait time
 *
 * The amount of time in seconds the system should wait at the airfield home waypoint
 *
 * @unit seconds
 * @min 0.0
 * @group DLL
 */
PARAM_DEFINE_FLOAT(NAV_DLL_AH_T, 120.0f);

/**
 * Number of allowed Datalink timeouts
 *
 * After more than this number of data link timeouts the aircraft returns home directly
 *
 * @group DLL
 * @min 0
 * @max 1000
 */
PARAM_DEFINE_INT32(NAV_DLL_N, 2);

/**
 * Skip comms hold wp
 *
 * If set to 1 the system will skip the comms hold wp on data link loss and will directly fly to
 * airfield home
 *
 * @group DLL
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(NAV_DLL_CHSK, 0);
