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
 * @file vehicle_global_position_setpoint.h
 * Definition of the global WGS84 position setpoint uORB topic.
 */

#ifndef TOPIC_VEHICLE_GLOBAL_POSITION_SETPOINT_H_
#define TOPIC_VEHICLE_GLOBAL_POSITION_SETPOINT_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"
#include "mission.h"

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
struct vehicle_global_position_setpoint_s
{
	bool altitude_is_relative;	/**< true if altitude is relative from start point	*/
	int32_t lat;			/**< latitude in degrees * 1E7				*/
	int32_t lon;			/**< longitude in degrees * 1E7				*/
	float altitude;			/**< altitude in meters					*/
	float yaw;			/**< in radians NED -PI..+PI 				*/
	float loiter_radius;		/**< loiter radius in meters, 0 for a VTOL to hover     */
	int8_t loiter_direction;	/**< 1: positive / clockwise, -1, negative.		*/
	enum NAV_CMD nav_cmd;		/**< true if loitering is enabled			*/
	float param1;
	float param2;
	float param3;
	float param4;
	float turn_distance_xy;		/**< The distance on the plane which will mark this as reached */
	float turn_distance_z;		/**< The distance in Z direction which will mark this as reached */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_global_position_setpoint);

#endif
