/****************************************************************************
 *
 *   Copyright (c) 2014-2016 PX4 Development Team. All rights reserved.
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
 * @file mission_params.c
 *
 * Parameters for mission.
 *
 * @author Julian Oes <julian@oes.ch>
 */

/*
 * Mission parameters, accessible via MAVLink
 */

/**
 * Take-off altitude
 *
 * This is the minimum altitude the system will take off to.
 *
 * @unit m
 * @min 0
 * @max 80
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_TAKEOFF_ALT, 10.0f);

/**
 * Minimum Loiter altitude
 *
 * This is the minimum altitude the system will always obey. The intent is to stay out of ground effect.
 *
 * @unit m
 * @min 0
 * @max 80
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_LTRMIN_ALT, 1.2f);

/**
 * Enable persistent onboard mission storage
 *
 * When enabled, missions that have been uploaded by the GCS are stored
 * and reloaded after reboot persistently.
 *
 * @boolean
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_ONBOARD_EN, 1);

/**
 * Maximal horizontal distance from home to first waypoint
 *
 * Failsafe check to prevent running mission stored from previous flight at a new takeoff location.
 * Set a value of zero or less to disable. The mission will not be started if the current
 * waypoint is more distant than MIS_DIS_1WP from the current position.
 *
 * @unit m
 * @min 0
 * @max 1000
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_DIST_1WP, 900);

/**
 * Altitude setpoint mode
 *
 * 0: the system will follow a zero order hold altitude setpoint
 * 1: the system will follow a first order hold altitude setpoint
 * values follow the definition in enum mission_altitude_mode
 *
 * @min 0
 * @max 1
 * @value 0 Zero Order Hold
 * @value 1 First Order Hold
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_ALTMODE, 1);

/**
 * Multirotor only. Yaw setpoint mode.
 *
 * The values are defined in the enum mission_altitude_mode
 *
 * @min 0
 * @max 3
 * @value 0 Heading as set by waypoint
 * @value 1 Heading towards waypoint
 * @value 2 Heading towards home
 * @value 3 Heading away from home
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_YAWMODE, 1);

/**
 * Time in seconds we wait on reaching target heading at a waypoint if it is forced.
 *
 * If set > 0 it will ignore the target heading for normal waypoint acceptance. If the
 * waypoint forces the heading the timeout will matter. For example on VTOL forwards transiton.
 * Mainly useful for VTOLs that have less yaw authority and might not reach target
 * yaw in wind. Disabled by default.
 *
 * @unit s
 * @min -1
 * @max 20
 * @increment 1
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_YAW_TMT, -1.0f);

/**
 * Max yaw error in degree needed for waypoint heading acceptance.
 *
 * @unit deg
 * @min 0
 * @max 90
 * @increment 1
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_YAW_ERR, 12.0f);
