/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file mission_item_triplet.h
 * Definition of the global WGS84 position setpoint uORB topic.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#ifndef TOPIC_MISSION_ITEM_TRIPLET_H_
#define TOPIC_MISSION_ITEM_TRIPLET_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

enum SETPOINT_TYPE
{
	SETPOINT_TYPE_POSITION = 0,	/**< position setpoint */
	SETPOINT_TYPE_VELOCITY,		/**< velocity setpoint */
	SETPOINT_TYPE_LOITER,		/**< loiter setpoint */
	SETPOINT_TYPE_TAKEOFF,		/**< takeoff setpoint */
	SETPOINT_TYPE_LAND,		/**< land setpoint, altitude must be ignored, descend until landing */
	SETPOINT_TYPE_IDLE,		/**< do nothing, switch off motors or keep at idle speed (MC) */
	SETPOINT_TYPE_OFFBOARD, 	/**< setpoint in NED frame (x, y, z, vx, vy, vz) set by offboard */
};

struct position_setpoint_s
{
	bool valid;			/**< true if setpoint is valid */
	enum SETPOINT_TYPE type;	/**< setpoint type to adjust behavior of position controller */
	float x;			/**< local position setpoint in m in NED */
	float y;			/**< local position setpoint in m in NED */
	float z;			/**< local position setpoint in m in NED */
	bool position_valid;	/**< true if local position setpoint valid */
	float vx;			/**< local velocity setpoint in m/s in NED */
	float vy;			/**< local velocity setpoint in m/s in NED */
	float vz;			/**< local velocity setpoint in m/s in NED */
	bool velocity_valid;		/**< true if local velocity setpoint valid */
	double lat;			/**< latitude, in deg */
	double lon;			/**< longitude, in deg */
	float alt;			/**< altitude AMSL, in m */
	float yaw;			/**< yaw (only for multirotors), in rad [-PI..PI), NaN = hold current yaw */
	float yawspeed;			/**< yawspeed (only for multirotors, in rad/s) */
	float loiter_radius;		/**< loiter radius (only for fixed wing), in m */
	int8_t loiter_direction;	/**< loiter direction: 1 = CW, -1 = CCW */
	float pitch_min;		/**< minimal pitch angle for fixed wing takeoff waypoints */
};

/**
 * Global position setpoint triplet in WGS84 coordinates.
 *
 * This are the three next waypoints (or just the next two or one).
 */
struct position_setpoint_triplet_s
{
	struct position_setpoint_s previous;
	struct position_setpoint_s current;
	struct position_setpoint_s next;

	unsigned nav_state;				/**< report the navigation state */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(position_setpoint_triplet);

#endif
