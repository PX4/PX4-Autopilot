/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
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
 * @file home_position.h
 * Definition of the home position uORB topic.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#ifndef TOPIC_HOME_POSITION_H_
#define TOPIC_HOME_POSITION_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * GPS home position in WGS84 coordinates.
 */
struct home_position_s
{
	uint64_t timestamp;			/**< Timestamp (microseconds since system boot)	*/

	double lat;				/**< Latitude in degrees 			*/
	double lon;				/**< Longitude in degrees			*/
	float alt;				/**< Altitude in meters	(AMSL)		*/

	float x;				/**< X coordinate in meters			*/
	float y;				/**< Y coordinate in meters			*/
	float z;				/**< Z coordinate in meters			*/
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(home_position);

#endif
