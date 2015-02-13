/****************************************************************************
 *
 *	 Copyright (C) 2008-2015 PX4 Development Team. All rights reserved.
 *	 Author: @author Lorenz Meier <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *		notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *		notice, this list of conditions and the following disclaimer in
 *		the documentation and/or other materials provided with the
 *		distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *		used to endorse or promote products derived from this software
 *		without specific prior written permission.
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
 * @file offboard_control_mode.h
 * Definition of the manual_control_setpoint uORB topic.
 */

#ifndef TOPIC_OFFBOARD_CONTROL_MODE_H_
#define TOPIC_OFFBOARD_CONTROL_MODE_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * Off-board control mode
 */

/**
 * @addtogroup topics
 * @{
 */

struct offboard_control_mode_s {
	uint64_t timestamp;

	bool ignore_thrust;
	bool ignore_attitude;
	bool ignore_bodyrate;
	bool ignore_position;
	bool ignore_velocity;
	bool ignore_acceleration_force;

}; /**< offboard control inputs */
/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(offboard_control_mode);

#endif
