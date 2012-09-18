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
 * @file ardrone_motors_setpoint.h
 * Definition of the ardrone_motors_setpoint uORB topic.
 */

#ifndef TOPIC_ARDRONE_MOTORS_SETPOINT_H_
#define TOPIC_ARDRONE_MOTORS_SETPOINT_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

struct ardrone_motors_setpoint_s
{
	uint64_t timestamp; //in microseconds since system start, is set whenever the writing thread stores new data

	uint8_t group;	/**< quadrotor group */
	uint8_t mode;	/**< requested flight mode XXX define */
	float p1;		/**< parameter 1: rate control: roll rate, att control: roll angle (in radians, NED) */
	float p2;		/**< parameter 2: rate control: pitch rate, att control: pitch angle (in radians, NED) */
	float p3;		/**< parameter 3: rate control: yaw rate, att control: yaw angle (in radians, NED) */
	float p4;		/**< parameter 4: thrust, [0..1] */
}; /**< AR.Drone low level motors */

 /**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(ardrone_motors_setpoint);

#endif
