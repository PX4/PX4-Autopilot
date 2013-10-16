/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@student.ethz.ch>
 *           @author Julian Oes <joes@student.ethz.ch>
 *           @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file mission_item.h
 * Definition of one mission item.
 */

#ifndef TOPIC_MISSION_H_
#define TOPIC_MISSION_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

enum NAV_CMD {
	NAV_CMD_WAYPOINT = 0,
	NAV_CMD_LOITER_TURN_COUNT,
	NAV_CMD_LOITER_TIME_LIMIT,
	NAV_CMD_LOITER_UNLIMITED,
	NAV_CMD_RETURN_TO_LAUNCH,
	NAV_CMD_LAND,
	NAV_CMD_TAKEOFF
};

/**
 * @addtogroup topics
 * @{
 */

#define MAX_MISSION_ITEMS 200

/**
 * Global position setpoint in WGS84 coordinates.
 *
 * This is the position the MAV is heading towards. If it of type loiter,
 * the MAV is circling around it with the given loiter radius in meters.
 */
typedef struct mission_item_s {
	float aceptable_radius; /* For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters */
	float acceptable_time; /* For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds */
	float orbit; /* For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise,
		        if negative the orbit direction should be counter-clockwise. */
	float yaw; /* For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH */
	float lat; /* local: x position, global: latitude */
	float lon; /* y position: global: longitude */
	float alt; /* z position: global: altitude */
	uint16_t seq; /* Sequence */
	uint16_t command; /* The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs */
	uint8_t frame; /* The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h */
	uint8_t current; /* false:0, true:1 */
	uint8_t autocontinue; /* autocontinue to next wp */
} mission_item_t;


/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(mission);

#endif

