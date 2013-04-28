/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
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
 * @file differential_pressure.h
 *
 * Definition of differential pressure topic
 */

#ifndef TOPIC_DIFFERENTIAL_PRESSURE_H_
#define TOPIC_DIFFERENTIAL_PRESSURE_H_

#include "../uORB.h"
#include <stdint.h>

/**
 * @addtogroup topics
 * @{
 */

/**
 * Differential pressure and airspeed
 */
struct differential_pressure_s {
	uint64_t	timestamp;					/**< microseconds since system boot, needed to integrate */
	float   	static_pressure_mbar;		/**< Static / environment pressure */
	float		differential_pressure_mbar;	/**< Differential pressure reading */
	float		temperature_celcius;		/**< ambient temperature in celcius, -1 if unknown */
	float		indicated_airspeed_m_s;		/**< indicated airspeed in meters per second, -1 if unknown	 */
	float		true_airspeed_m_s;			/**< true airspeed in meters per second, -1 if unknown */
	float		voltage;					/**< Voltage from the airspeed sensor (voltage divider already compensated) */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(differential_pressure);

#endif
