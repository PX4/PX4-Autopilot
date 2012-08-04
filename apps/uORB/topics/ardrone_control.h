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

/*
 * @file ardrone_control.c
 * Definition of the ardrone_control uORB topic.
 */

#ifndef TOPIC_ARDRONE_CONTROL_H_
#define TOPIC_ARDRONE_CONTROL_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

struct ardrone_control_s
{
	uint16_t counter;                    /**< incremented by the publishing thread everytime new data is stored. */
	uint64_t timestamp;                  /**< in microseconds since system start, is set whenever the writing thread stores new data. */

	float setpoint_rate_cast[3];
	float setpoint_thrust_cast;          /**< LOGME */
	float setpoint_attitude_rate[3];
	float setpoint_attitude[3];          /**< LOGME */
	float attitude_control_output[3];    /**< roll, pitch, yaw. */
	float position_control_output[3];    /**< x, y, z.  */
	float attitude_setpoint_navigationframe_from_positioncontroller[3]; /**< LOGME */
	float gyro_scaled[3];
	float gyro_filtered[3];
	float gyro_filtered_offset[3];
	float zcompensation;
	uint16_t motor_front_nw;
	uint16_t motor_right_ne;
	uint16_t motor_back_se;
	uint16_t motor_left_sw;

}; /**< ardrone control status */

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(ardrone_control);

#endif /* TOPIC_ARDRONE_CONTROL_H_ */
