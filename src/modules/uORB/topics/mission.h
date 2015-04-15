/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file mission.h
 * Definition of a mission consisting of mission items.
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#ifndef TOPIC_MISSION_H_
#define TOPIC_MISSION_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

#define NUM_MISSIONS_SUPPORTED 256

/* compatible to mavlink MAV_CMD */
enum NAV_CMD {
	NAV_CMD_IDLE=0,
	NAV_CMD_WAYPOINT=16,
	NAV_CMD_LOITER_UNLIMITED=17,
	NAV_CMD_LOITER_TURN_COUNT=18,
	NAV_CMD_LOITER_TIME_LIMIT=19,
	NAV_CMD_RETURN_TO_LAUNCH=20,
	NAV_CMD_LAND=21,
	NAV_CMD_TAKEOFF=22,
	NAV_CMD_ROI=80,
	NAV_CMD_PATHPLANNING=81,
	NAV_CMD_DO_JUMP=177
};

enum ORIGIN {
	ORIGIN_MAVLINK = 0,
	ORIGIN_ONBOARD
};

/**
 * @addtogroup topics
 * @{
 */

/**
 * Global position setpoint in WGS84 coordinates.
 *
 * This is the position the MAV is heading towards. If it of type loiter,
 * the MAV is circling around it with the given loiter radius in meters.
 */
struct mission_item_s {
	bool altitude_is_relative;	/**< true if altitude is relative from start point	*/
	double lat;			/**< latitude in degrees				*/
	double lon;			/**< longitude in degrees				*/
	float altitude;			/**< altitude in meters	(AMSL)			*/
	float yaw;			/**< in radians NED -PI..+PI, NAN means don't change yaw		*/
	float loiter_radius;		/**< loiter radius in meters, 0 for a VTOL to hover     */
	int8_t loiter_direction;	/**< 1: positive / clockwise, -1, negative.		*/
	enum NAV_CMD nav_cmd;		/**< navigation command					*/
	float acceptance_radius;	/**< default radius in which the mission is accepted as reached in meters */
	float time_inside;		/**< time that the MAV should stay inside the radius before advancing in seconds */
	float pitch_min;		/**< minimal pitch angle for fixed wing takeoff waypoints */
	bool autocontinue;		/**< true if next waypoint should follow after this one */
	enum ORIGIN origin;		/**< where the waypoint has been generated		*/
	int do_jump_mission_index;	/**< index where the do jump will go to                 */
	unsigned do_jump_repeat_count;	/**< how many times do jump needs to be done            */
	unsigned do_jump_current_count;	/**< count how many times the jump has been done	*/
};

/**
 * This topic used to notify navigator about mission changes, mission itself and new mission state
 * must be stored in dataman before publication.
 */
struct mission_s
{
	int dataman_id;			/**< default 0, there are two offboard storage places in the dataman: 0 or 1 */
	unsigned count;			/**< count of the missions stored in the dataman */
	int current_seq;				/**< default -1, start at the one changed latest */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(offboard_mission);
ORB_DECLARE(onboard_mission);

#endif
