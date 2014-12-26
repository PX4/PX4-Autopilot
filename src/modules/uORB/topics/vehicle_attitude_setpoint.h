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
 * @file vehicle_attitude_setpoint.h
 * Definition of the vehicle attitude setpoint uORB topic.
 */

#ifndef TOPIC_VEHICLE_ATTITUDE_SETPOINT_H_
#define TOPIC_VEHICLE_ATTITUDE_SETPOINT_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * vehicle attitude setpoint.
 */
struct vehicle_attitude_setpoint_s {
	uint64_t timestamp;		/**< in microseconds since system start, is set whenever the writing thread stores new data */

	float roll_body;				/**< body angle in NED frame		*/
	float pitch_body;				/**< body angle in NED frame		*/
	float yaw_body;					/**< body angle in NED frame		*/
	//float body_valid;				/**< Set to true if body angles are valid */

	float R_body[3][3];				/**< Rotation matrix describing the setpoint as rotation from the current body frame */
	bool R_valid;					/**< Set to true if rotation matrix is valid */

	//! For quaternion-based attitude control
	float q_d[4];				/** Desired quaternion for quaternion control */
	bool q_d_valid;					/**< Set to true if quaternion vector is valid */
	float q_e[4];				/** Attitude error in quaternion */
	bool q_e_valid;					/**< Set to true if quaternion error vector is valid */

	float thrust;					/**< Thrust in Newton the power system should generate */

	bool	roll_reset_integral;			/**< Reset roll integral part (navigation logic change) */
	bool	pitch_reset_integral;			/**< Reset pitch integral part (navigation logic change) */
	bool	yaw_reset_integral;			/**< Reset yaw integral part (navigation logic change) */

};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_attitude_setpoint);
ORB_DECLARE(mc_virtual_attitude_setpoint);
ORB_DECLARE(fw_virtual_attitude_setpoint);

#endif /* TOPIC_ARDRONE_CONTROL_H_ */
