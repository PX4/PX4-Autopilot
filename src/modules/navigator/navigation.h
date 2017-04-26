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
 * @file navigation.h
 * Definition of a mission consisting of mission items.
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <stdint.h>
#include <stdbool.h>

#if defined(MEMORY_CONSTRAINED_SYSTEM)
#  define NUM_MISSIONS_SUPPORTED 50
#elif defined(__PX4_POSIX)
#  define NUM_MISSIONS_SUPPORTED (UINT16_MAX-1) // This is allocated as needed.
#else
#  define NUM_MISSIONS_SUPPORTED 2000 // This allocates a file of around 181 kB on the SD card.
#endif

#define NAV_EPSILON_POSITION	0.001f	/**< Anything smaller than this is considered zero */

/* compatible to mavlink MAV_CMD */
enum NAV_CMD {
	NAV_CMD_IDLE = 0,
	NAV_CMD_WAYPOINT = 16,
	NAV_CMD_LOITER_UNLIMITED = 17,
	NAV_CMD_LOITER_TIME_LIMIT = 19,
	NAV_CMD_RETURN_TO_LAUNCH = 20,
	NAV_CMD_LAND = 21,
	NAV_CMD_TAKEOFF = 22,
	NAV_CMD_LOITER_TO_ALT = 31,
	NAV_CMD_DO_FOLLOW_REPOSITION = 33,
	NAV_CMD_ROI = 80,
	NAV_CMD_VTOL_TAKEOFF = 84,
	NAV_CMD_VTOL_LAND = 85,
	NAV_CMD_DELAY = 93,
	NAV_CMD_DO_JUMP = 177,
	NAV_CMD_DO_CHANGE_SPEED = 178,
	NAV_CMD_DO_SET_SERVO = 183,
	NAV_CMD_DO_LAND_START = 189,
	NAV_CMD_DO_SET_ROI = 201,
	NAV_CMD_DO_DIGICAM_CONTROL = 203,
	NAV_CMD_DO_MOUNT_CONFIGURE = 204,
	NAV_CMD_DO_MOUNT_CONTROL = 205,
	NAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,
	NAV_CMD_IMAGE_START_CAPTURE = 2000,
	NAV_CMD_IMAGE_STOP_CAPTURE = 2001,
	NAV_CMD_VIDEO_START_CAPTURE = 2500,
	NAV_CMD_VIDEO_STOP_CAPTURE = 2501,
	NAV_CMD_DO_VTOL_TRANSITION = 3000,
	NAV_CMD_INVALID = UINT16_MAX /* ensure that casting a large number results in a specific error */
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
#pragma pack(push, 1)
struct mission_item_s {
	double lat;			/**< latitude in degrees				*/
	double lon;			/**< longitude in degrees				*/
	union {
		struct {
			union {
				float time_inside;		/**< time that the MAV should stay inside the radius before advancing in seconds */
				float pitch_min;		/**< minimal pitch angle for fixed wing takeoff waypoints */
			};
			float acceptance_radius;	/**< default radius in which the mission is accepted as reached in meters */
			float loiter_radius;		/**< loiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise */
			float yaw;					/**< in radians NED -PI..+PI, NAN means don't change yaw		*/
			float ___lat_float;			/**< padding */
			float ___lon_float;			/**< padding */
			float altitude;				/**< altitude in meters	(AMSL)			*/
		};
		float params[7];				/**< array to store mission command values for MAV_FRAME_MISSION ***/
	};
	uint16_t nav_cmd;					/**< navigation command					*/
	int16_t do_jump_mission_index;		/**< index where the do jump will go to                 */
	uint16_t do_jump_repeat_count;		/**< how many times do jump needs to be done            */
	uint16_t do_jump_current_count;		/**< count how many times the jump has been done	*/
	struct {
		uint16_t frame : 4,					/**< mission frame ***/
			 origin : 3,						/**< how the mission item was generated */
			 loiter_exit_xtrack : 1,			/**< exit xtrack location: 0 for center of loiter wp, 1 for exit location */
			 force_heading : 1,				/**< heading needs to be reached ***/
			 altitude_is_relative : 1,		/**< true if altitude is relative from start point	*/
			 autocontinue : 1,				/**< true if next waypoint should follow after this one */
			 disable_mc_yaw : 1,				/**< weathervane mode */
			 vtol_back_transition : 1;		/**< part of the vtol back transition sequence */
	};
};
#pragma pack(pop)
#include <uORB/topics/mission.h>

/**
 * @}
 */


#endif
