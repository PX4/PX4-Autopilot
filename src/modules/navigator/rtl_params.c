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
 * @file rtl_params.c
 *
 * Parameters for return mode
 *
 * @author Julian Oes <julian@oes.ch>
 */

/*
 * Return mode parameters, accessible via MAVLink
 */

/**
 * Return mode return altitude
 *
 * Default minimum altitude above destination (e.g. home, safe point, landing pattern) for return flight.
 * The vehicle will climb to this altitude when Return mode is enganged, unless it currently is flying higher already.
 * This is affected by RTL_MIN_DIST and RTL_CONE_ANG.
 *
 * @unit m
 * @min 0
 * @decimal 1
 * @increment 0.5
 * @group Return Mode
 */
PARAM_DEFINE_FLOAT(RTL_RETURN_ALT, 60.f);


/**
 * Return mode loiter altitude
 *
 * Descend to this altitude (above destination position) after return, and wait for time defined in RTL_LAND_DELAY.
 * Land (i.e. slowly descend) from this altitude if autolanding allowed.
 * VTOLs do transition to hover in this altitdue above the landing point.
 *
 * @unit m
 * @min 0
 * @decimal 1
 * @increment 0.5
 * @group Return Mode
 */
PARAM_DEFINE_FLOAT(RTL_DESCEND_ALT, 30.f);

/**
 * Return mode delay
 *
 * Delay before landing (after initial descent) in Return mode.
 * If set to -1 the system will not land but loiter at RTL_DESCEND_ALT.
 *
 * @unit s
 * @min -1
 * @decimal 1
 * @increment 0.5
 * @group Return Mode
 */
PARAM_DEFINE_FLOAT(RTL_LAND_DELAY, 0.0f);

/**
 * Horizontal radius from return point within which special rules for return mode apply.
 *
 * The return altitude will be calculated based on RTL_CONE_ANG parameter.
 * The yaw setpoint will switch to the one defined by corresponding waypoint.
 *
 *
 * @unit m
 * @min 0.5
 * @decimal 1
 * @increment 0.5
 * @group Return Mode
 */
PARAM_DEFINE_FLOAT(RTL_MIN_DIST, 10.0f);

/**
 * Return type
 *
 * Return mode destination and flight path (home location, rally point, mission landing pattern, reverse mission)
 *
 * @value 0 Return to closest safe point (home or rally point) via direct path.
 * @value 1 Return to closest safe point other than home (mission landing pattern or rally point), via direct path. If no mission landing or rally points are defined return home via direct path. Always chose closest safe landing point if vehicle is a VTOL in hover mode.
 * @value 2 Return to a planned mission landing, if available, using the mission path, else return to home via the reverse mission path. Do not consider rally points.
 * @value 3 Return via direct path to closest destination: home, start of mission landing pattern or safe point. If the destination is a mission landing pattern, follow the pattern to land.
 * @value 4 Return to the planned mission landing, or to home via the reverse mission path, whichever is closer by counting waypoints. Do not consider rally points.
 * @group Return Mode
 */
PARAM_DEFINE_INT32(RTL_TYPE, 0);

/**
 * Half-angle of the return mode altitude cone
 *
 * Defines the half-angle of a cone centered around the destination position that
 * affects the altitude at which the vehicle returns.
 *
 * @unit deg
 * @min 0
 * @max 90
 * @value 0 No cone, always climb to RTL_RETURN_ALT above destination.
 * @value 25 25 degrees half cone angle.
 * @value 45 45 degrees half cone angle.
 * @value 65 65 degrees half cone angle.
 * @value 80 80 degrees half cone angle.
 * @value 90 Only climb to at least RTL_DESCEND_ALT above destination.
 * @group Return Mode
 */
PARAM_DEFINE_INT32(RTL_CONE_ANG, 45);

/**
 * RTL precision land mode
 *
 * Use precision landing when doing an RTL landing phase.
 * This setting does not apply for RTL destinations planned as part of a mission.
 *
 * @value 0 No precision landing
 * @value 1 Opportunistic precision landing
 * @value 2 Required precision landing
 * @group Return Mode
 */
PARAM_DEFINE_INT32(RTL_PLD_MD, 0);

/**
 * Loiter radius for rtl descend
 *
 * Set the radius for loitering to a safe altitude for VTOL transition.
 *
 * @unit m
 * @min 25
 * @decimal 1
 * @increment 0.5
 * @group Return Mode
 */
PARAM_DEFINE_FLOAT(RTL_LOITER_RAD, 80.0f);

/**
 * RTL time estimate safety margin factor
 *
 * Safety factor that is used to scale the actual RTL time estimate.
 * Time with margin = RTL_TIME_FACTOR * time + RTL_TIME_MARGIN
 *
 * @min 1.0
 * @max 2.0
 * @decimal 1
 * @increment 0.1
 * @group Return To Land
 */
PARAM_DEFINE_FLOAT(RTL_TIME_FACTOR, 1.1f);

/**
 * RTL time estimate safety margin offset
 *
 * Margin that is added to the time estimate, after it has already been scaled
 * Time with margin = RTL_TIME_FACTOR * time + RTL_TIME_MARGIN
 *
 * @unit s
 * @min 0
 * @max 3600
 * @decimal 1
 * @increment 1
 * @group Return To Land
 */
PARAM_DEFINE_INT32(RTL_TIME_MARGIN, 100);

/**
 * RTL force approach landing
 *
 * Only consider RTL point, if it has an approach defined.
 *
 * @boolean
 * @group Return To Land
 */
PARAM_DEFINE_INT32(RTL_APPR_FORCE, 0);
