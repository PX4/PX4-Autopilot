/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file optical_flow.h
 * Definition of the optical flow uORB topic.
 */

#ifndef TOPIC_OPTICAL_FLOW_H_
#define TOPIC_OPTICAL_FLOW_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 */

/**
 * Optical flow in NED body frame in SI units.
 *
 * @see http://en.wikipedia.org/wiki/International_System_of_Units
 */
struct optical_flow_s {

	uint64_t timestamp;		/**< in microseconds since system start  */
	uint8_t sensor_id;		/**< id of the sensor emitting the flow value */
	float pixel_flow_x_integral; /**< accumulated optical flow in radians around x axis */
	float pixel_flow_y_integral; /**< accumulated optical flow in radians around y axis */
	float gyro_x_rate_integral;	 /**< accumulated gyro value in radians around x axis */
	float gyro_y_rate_integral;  /**< accumulated gyro value in radians around y axis */
	float gyro_z_rate_integral;   /**< accumulated gyro value in radians around z axis */
	float ground_distance_m;	 /**< Altitude / distance to ground in meters */
	uint32_t integration_timespan; /**<accumulation timespan in microseconds     */
	uint32_t time_since_last_sonar_update;/**< time since last sonar update in microseconds */
	uint16_t frame_count_since_last_readout;/**< number of accumulated frames in timespan */
	int16_t gyro_temperature;/**< 	Temperature * 100 in centi-degrees Celsius */
	uint8_t	quality;		/**< Average of quality of accumulated frames, 0: bad quality, 255: maximum quality */




};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(optical_flow);

#endif
