/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file vehicle_global_position.h
 * Definition of the global fused WGS84 position uORB topic.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#ifndef VEHICLE_GLOBAL_POSITION_T_H_
#define VEHICLE_GLOBAL_POSITION_T_H_

#include <stdint.h>
#include <stdbool.h>
#include <platforms/px4_defines.h>

/**
 * @addtogroup topics
 * @{
 */

 /**
 * Fused global position in WGS84.
 *
 * This struct contains global position estimation. It is not the raw GPS
 * measurement (@see vehicle_gps_position). This topic is usually published by the position
 * estimator, which will take more sources of information into account than just GPS,
 * e.g. control inputs of the vehicle in a Kalman-filter implementation.
 */
struct vehicle_global_position_s {
	uint64_t timestamp;		/**< Time of this estimate, in microseconds since system start		*/
	uint64_t time_utc_usec;		/**< GPS UTC timestamp in microseconds					   */
	double lat;			/**< Latitude in degrees							 	   */
	double lon;			/**< Longitude in degrees							 	   */
	float alt;			/**< Altitude AMSL in meters						 	   */
	float vel_n; 			/**< Ground north velocity, m/s				 			   */
	float vel_e;			/**< Ground east velocity, m/s							   */
	float vel_d;			/**< Ground downside velocity, m/s						   */
	float yaw; 			/**< Yaw in radians -PI..+PI.							   */
	float eph;			/**< Standard deviation of position estimate horizontally */
	float epv;			/**< Standard deviation of position vertically */
	float terrain_alt;		/**< Terrain altitude in m, WGS84 */
	bool terrain_alt_valid;		/**< Terrain altitude estimate is valid */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_global_position);

#endif
