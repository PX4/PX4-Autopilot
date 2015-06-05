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
 * @author Julian Oes <joes@student.ethz.ch>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>

/*
 * Mission parameters, accessible via MAVLink
 */

/**
 * Take-off altitude
 *
 * Even if first waypoint has altitude less then MIS_TAKEOFF_ALT above home position, system will climb to
 * MIS_TAKEOFF_ALT on takeoff, then go to waypoint.
 *
 * @unit meters
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_TAKEOFF_ALT, 10.0f);

/**
 * Enable persistent onboard mission storage
 *
 * When enabled, missions that have been uploaded by the GCS are stored
 * and reloaded after reboot persistently.
 *
 * @min 0
 * @max 1
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
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_ALTMODE, 1);

/**
 * Multirotor only. Yaw setpoint mode.
 *
 * 0: Set the yaw heading to the yaw value specified for the destination waypoint.
 * 1: Maintain a yaw heading pointing towards the next waypoint.
 * 2: Maintain a yaw heading that always points to the home location.
 * 3: Maintain a yaw heading that always points away from the home location (ie: back always faces home).
 *
 * The values are defined in the enum mission_altitude_mode
 *
 * @min 0
 * @max 3
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_YAWMODE, 1);
