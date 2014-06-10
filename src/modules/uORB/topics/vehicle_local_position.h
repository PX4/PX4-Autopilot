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
 * @file vehicle_local_position.h
 * Definition of the local fused NED position uORB topic.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef TOPIC_VEHICLE_LOCAL_POSITION_H_
#define TOPIC_VEHICLE_LOCAL_POSITION_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * Fused local position in NED.
 */
struct vehicle_local_position_s {
	uint64_t timestamp;		/**< Time of this estimate, in microseconds since system start */
	bool xy_valid;			/**< true if x and y are valid */
	bool z_valid;			/**< true if z is valid */
	bool v_xy_valid;		/**< true if vy and vy are valid */
	bool v_z_valid;			/**< true if vz is valid */
	/* Position in local NED frame */
	float x;				/**< X position in meters in NED earth-fixed frame */
	float y;				/**< X position in meters in NED earth-fixed frame */
	float z;				/**< Z position in meters in NED earth-fixed frame (negative altitude) */
	/* Velocity in NED frame */
	float vx; 				/**< Ground X Speed (Latitude), m/s in NED */
	float vy;				/**< Ground Y Speed (Longitude), m/s in NED */
	float vz;				/**< Ground Z Speed (Altitude), m/s	in NED */
	/* Heading */
	float yaw;
	/* Reference position in GPS / WGS84 frame */
	bool xy_global;			/**< true if position (x, y) is valid and has valid global reference (ref_lat, ref_lon) */
	bool z_global;			/**< true if z is valid and has valid global reference (ref_alt) */
	uint64_t ref_timestamp;	/**< Time when reference position was set */
	double ref_lat;		/**< Reference point latitude in degrees */
	double ref_lon;		/**< Reference point longitude in degrees */
	float ref_alt;			/**< Reference altitude AMSL in meters, MUST be set to current (not at reference point!) ground level */
	bool landed;			/**< true if vehicle is landed */
	/* Distance to surface */
	float dist_bottom;		/**< Distance to bottom surface (ground) */
	float dist_bottom_rate;		/**< Distance to bottom surface (ground) change rate */
	uint64_t surface_bottom_timestamp;		/**< Time when new bottom surface found */
	bool dist_bottom_valid;	/**< true if distance to bottom surface is valid */
	float eph;
	float epv;
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_local_position);

#endif
