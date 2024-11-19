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
 * Default take-off altitude
 *
 * This is the relative altitude the system will take off to
 * if not otherwise specified.
 *
 * @unit m
 * @min 0
 * @decimal 1
 * @increment 0.5
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_TAKEOFF_ALT, 2.5f);

/**
 * Mission takeoff/landing required
 *
 * Specifies if a mission has to contain a takeoff and/or mission landing.
 * Validity of configured takeoffs/landings is checked independently of the setting here.
 *
 * @value 0 No requirements
 * @value 1 Require a takeoff
 * @value 2 Require a landing
 * @value 3 Require a takeoff and a landing
 * @value 4 Require both a takeoff and a landing, or neither
 * @value 5 Same as previous when landed, in-air require landing only if no valid VTOL approach is present
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_TKO_LAND_REQ, 0);

/**
 * Maximal horizontal distance from Home to first waypoint
 *
 * There will be a warning message if the current waypoint is more distant than MIS_DIST_1WP from Home.
 * Has no effect on mission validity.
 * Set a value of zero or less to disable.
 *
 * @unit m
 * @min -1
 * @max 100000
 * @decimal 1
 * @increment 100
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_DIST_1WP, 10000);

/**
* Enable yaw control of the mount. (Only affects multicopters and ROI mission items)
*
* If enabled, yaw commands will be sent to the mount and the vehicle will follow its heading towards the flight direction.
* If disabled, the vehicle will yaw towards the ROI.
*
* @value 0 Disable
* @value 1 Enable
* @min 0
* @max 1
* @group Mission
*/
PARAM_DEFINE_INT32(MIS_MNT_YAW_CTL, 0);

/**
 * Time in seconds we wait on reaching target heading at a waypoint if it is forced.
 *
 * If set > 0 it will ignore the target heading for normal waypoint acceptance. If the
 * waypoint forces the heading the timeout will matter. For example on VTOL forwards transition.
 * Mainly useful for VTOLs that have less yaw authority and might not reach target
 * yaw in wind. Disabled by default.
 *
 * @unit s
 * @min -1
 * @max 20
 * @decimal 1
 * @increment 1
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_YAW_TMT, -1.0f);

/**
 * Max yaw error in degrees needed for waypoint heading acceptance.
 *
 * @unit deg
 * @min 0
 * @max 90
 * @decimal 1
 * @increment 1
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_YAW_ERR, 12.0f);

/**
 * Timeout for a successful payload deployment acknowledgement
 *
 * @unit s
 * @min 0
 * @decimal 1
 * @increment 1
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_PD_TO, 5.0f);

/**
 * Landing abort min altitude
 *
 * Minimum altitude above landing point that the vehicle will climb to after an aborted landing.
 * Then vehicle will loiter in this altitude until further command is received.
 * Only applies to fixed-wing vehicles.
 *
 * @unit m
 * @min 0
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_LND_ABRT_ALT, 30);
