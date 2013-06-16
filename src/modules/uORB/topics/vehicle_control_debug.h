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
 * @file vehicle_control_debug.h
 * For debugging purposes to log PID parts of controller
 */

#ifndef TOPIC_VEHICLE_CONTROL_DEBUG_H_
#define TOPIC_VEHICLE_CONTROL_DEBUG_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */
struct vehicle_control_debug_s
{
	uint64_t timestamp; /**< in microseconds since system start */

	float roll_p;		/**< roll P control part		*/
	float roll_i;		/**< roll I control part 		*/
	float roll_d;		/**< roll D control part 		*/

	float roll_rate_p;	/**< roll rate P control part		*/
	float roll_rate_i;	/**< roll rate I control part 		*/
	float roll_rate_d;	/**< roll rate D control part 		*/

	float pitch_p;		/**< pitch P control part		*/
	float pitch_i;		/**< pitch I control part 		*/
	float pitch_d;		/**< pitch D control part 		*/

	float pitch_rate_p;	/**< pitch rate P control part		*/
	float pitch_rate_i;	/**< pitch rate I control part 		*/
	float pitch_rate_d;	/**< pitch rate D control part 		*/

	float yaw_p;		/**< yaw P control part			*/
	float yaw_i;		/**< yaw I control part 		*/
	float yaw_d;		/**< yaw D control part 		*/

	float yaw_rate_p;	/**< yaw rate P control part		*/
	float yaw_rate_i;	/**< yaw rate I control part 		*/
	float yaw_rate_d;	/**< yaw rate D control part 		*/

}; /**< vehicle_control_debug */

 /**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_control_debug);

#endif
