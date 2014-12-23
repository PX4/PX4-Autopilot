/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
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

/**
 * @file vehicle_attitude.h
 * Definition of the attitude uORB topic.
 */

#ifndef VEHICLE_ATTITUDE_H_
#define VEHICLE_ATTITUDE_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * Attitude in NED body frame in SI units.
 *
 * @see http://en.wikipedia.org/wiki/International_System_of_Units
 */
struct vehicle_attitude_s {

	uint64_t timestamp;	/**< in microseconds since system start          */

	/* This is similar to the mavlink message ATTITUDE, but for onboard use */

	/** @warning roll, pitch and yaw have always to be valid, the rotation matrix and quaternion are optional */

	float roll;		/**< Roll angle (rad, Tait-Bryan, NED)				*/
	float pitch;		/**< Pitch angle (rad, Tait-Bryan, NED)				*/
	float yaw;		/**< Yaw angle (rad, Tait-Bryan, NED)				*/
	float rollspeed;	/**< Roll angular speed (rad/s, Tait-Bryan, NED)		*/
	float pitchspeed;	/**< Pitch angular speed (rad/s, Tait-Bryan, NED)		*/
	float yawspeed;		/**< Yaw angular speed (rad/s, Tait-Bryan, NED)			*/
	float rollacc;		/**< Roll angular accelration (rad/s, Tait-Bryan, NED)		*/
	float pitchacc;		/**< Pitch angular acceleration (rad/s, Tait-Bryan, NED)	*/
	float yawacc;		/**< Yaw angular acceleration (rad/s, Tait-Bryan, NED)		*/
	float rate_offsets[3];	/**< Offsets of the body angular rates from zero		*/
	float R[3][3];		/**< Rotation matrix body to world, (Tait-Bryan, NED)		*/
	float q[4];		/**< Quaternion (NED)						*/
	float g_comp[3];	/**< Compensated gravity vector					*/
	bool R_valid;		/**< Rotation matrix valid					*/
	bool q_valid;		/**< Quaternion valid						*/

	// secondary attitude, use for VTOL
	float roll_sec;		/**< Roll angle (rad, Tait-Bryan, NED)				*/
	float pitch_sec;		/**< Pitch angle (rad, Tait-Bryan, NED)				*/
	float yaw_sec;		/**< Yaw angle (rad, Tait-Bryan, NED)				*/
	float rollspeed_sec;	/**< Roll angular speed (rad/s, Tait-Bryan, NED)		*/
	float pitchspeed_sec;	/**< Pitch angular speed (rad/s, Tait-Bryan, NED)		*/
	float yawspeed_sec;		/**< Yaw angular speed (rad/s, Tait-Bryan, NED)			*/
	float rollacc_sec;		/**< Roll angular accelration (rad/s, Tait-Bryan, NED)		*/
	float pitchacc_sec;		/**< Pitch angular acceleration (rad/s, Tait-Bryan, NED)	*/
	float yawacc_sec;		/**< Yaw angular acceleration (rad/s, Tait-Bryan, NED)		*/
	float rate_offsets_sec[3];	/**< Offsets of the body angular rates from zero		*/
	float R_sec[3][3];		/**< Rotation matrix body to world, (Tait-Bryan, NED)		*/
	float q_sec[4];		/**< Quaternion (NED)						*/
	float g_comp_sec[3];	/**< Compensated gravity vector					*/
	bool R_valid_sec;		/**< Rotation matrix valid					*/
	bool q_valid_sec;		/**< Quaternion valid						*/

};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_attitude);

#endif
