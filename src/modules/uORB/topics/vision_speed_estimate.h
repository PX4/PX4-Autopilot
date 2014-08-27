/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file vision_speed_estimate.h
 * Vision based speed estimate
 */

#ifndef TOPIC_VISION_SPEED_ESTIMATE_H_
#define TOPIC_VISION_SPEED_ESTIMATE_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * Vision based speed estimate in NED frame
 */
struct vision_speed_estimate_s {

	unsigned id;				/**< ID of the estimator, commonly the component ID of the incoming message */

	uint64_t timestamp_boot;		/**< time of this estimate, in microseconds since system start */
	uint64_t timestamp_computer;		/**< timestamp provided by the companion computer, in us */

	float vx;				/**< X velocity in meters per second in NED earth-fixed frame */
	float vy;				/**< Y velocity in meters per second in NED earth-fixed frame */
	float vz;				/**< Z velocity in meters per second in NED earth-fixed frame */

	// XXX Add covariances here
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vision_speed_estimate);

#endif /* TOPIC_VISION_SPEED_ESTIMATE_H_ */