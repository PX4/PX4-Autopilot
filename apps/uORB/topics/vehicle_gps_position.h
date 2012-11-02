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
 * @file vehicle_gps_position.h
 * Definition of the GPS WGS84 uORB topic.
 */

#ifndef TOPIC_VEHICLE_GPS_H_
#define TOPIC_VEHICLE_GPS_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * GPS position in WGS84 coordinates.
 */
struct vehicle_gps_position_s
{
	uint64_t timestamp;			/**< Timestamp (microseconds since system boot)   */
	uint32_t counter;			/**< Count of GPS messages */
	uint64_t time_gps_usec;			/**< Timestamp (microseconds in GPS format), this is the timestamp which comes from the gps module   */
	
	int32_t lat;				/**< Latitude in 1E7 degrees //LOGME   */
	int32_t lon;				/**< Longitude in 1E7 degrees //LOGME   */
	int32_t alt;				/**< Altitude in 1E3 meters (millimeters) above MSL //LOGME   */
	uint16_t counter_pos_valid;		/**< is only increased when new lat/lon/alt information was added   */
	uint16_t eph;				/**< GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535 //LOGME   */
	uint16_t epv;				/**< GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535   */
	uint16_t vel;				/**< GPS ground speed (m/s * 100). If unknown, set to: 65535   */
	float vel_n;				/**< GPS ground speed in m/s */
	float vel_e;				/**< GPS ground speed in m/s */
	float vel_d;				/**< GPS ground speed in m/s */
	uint16_t cog;				/**< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535   */
	uint8_t fix_type; 			/**< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.   */
	uint8_t satellites_visible;		/**< Number of satellites visible. If unknown, set to 255   */

	uint8_t satellite_prn[20]; 		/**< Global satellite ID   */
	uint8_t satellite_used[20];		/**< 0: Satellite not used, 1: used for localization   */
	uint8_t satellite_elevation[20]; 	/**< Elevation (0: right on top of receiver, 90: on the horizon) of satellite   */
	uint8_t satellite_azimuth[20];		/**< Direction of satellite, 0: 0 deg, 255: 360 deg.   */
	uint8_t satellite_snr[20];		/**< Signal to noise ratio of satellite   */
	uint8_t satellite_info_available;	/**< 0 for no info, 1 for info available   */

	/* flags */
	float vel_ned_valid;			/**< Flag to indicate if NED speed is valid */

};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_gps_position);

#endif
