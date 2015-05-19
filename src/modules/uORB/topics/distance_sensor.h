/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *   Author: @author Nuno Marques <n.marques21@hotmail.com>
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
 * @file distance_sensor.h
 * Definition of the distance sensor uORB topic.
 */

#ifndef TOPIC_DISTANCE_SENSOR_H_
#define TOPIC_DISTANCE_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 */

/**
 * Distance sensor data, with range in centimeters (not in SI).
 *
 * @see http://en.wikipedia.org/wiki/International_System_of_Units
 */
struct distance_sensor_s {

	uint32_t time_boot_ms; /**< Time since system boot */
	uint16_t min_distance; /**< Minimum distance the sensor can measure in centimeters */
	uint16_t max_distance; /**< Maximum distance the sensor can measure in centimeters */
	uint16_t current_distance; /**< Current distance reading */
	uint8_t type; /**< Type from MAV_DISTANCE_SENSOR enum */
	uint8_t id; /**< Onboard ID of the sensor */
	uint8_t orientation; /**< Direction the sensor faces from MAV_SENSOR_ORIENTATION enum */
	uint8_t covariance; /**< Measurement covariance in centimeters, 0 for unknown / invalid readings */

};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(distance_sensor);

#endif