/****************************************************************************
 *
 *	 Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
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
 * @file offboard_control_setpoint.h
 * Definition of the manual_control_setpoint uORB topic.
 */

#ifndef TOPIC_OFFBOARD_CONTROL_SETPOINT_H_
#define TOPIC_OFFBOARD_CONTROL_SETPOINT_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * Off-board control inputs.
 *
 * Typically sent by a ground control station / joystick or by
 * some off-board controller via C or SIMULINK.
 */
enum OFFBOARD_CONTROL_MODE {
	OFFBOARD_CONTROL_MODE_DIRECT = 0,
	OFFBOARD_CONTROL_MODE_DIRECT_RATES = 1,
	OFFBOARD_CONTROL_MODE_DIRECT_ATTITUDE = 2,
	OFFBOARD_CONTROL_MODE_DIRECT_LOCAL_NED = 3,
	OFFBOARD_CONTROL_MODE_DIRECT_LOCAL_OFFSET_NED = 4,
	OFFBOARD_CONTROL_MODE_DIRECT_BODY_NED = 5,
	OFFBOARD_CONTROL_MODE_DIRECT_BODY_OFFSET_NED = 6,
	OFFBOARD_CONTROL_MODE_DIRECT_GLOBAL = 7,
	OFFBOARD_CONTROL_MODE_ATT_YAW_RATE = 8,
	OFFBOARD_CONTROL_MODE_ATT_YAW_POS = 9,
	OFFBOARD_CONTROL_MODE_MULTIROTOR_SIMPLE = 10, /**< roll / pitch rotated aligned to the takeoff orientation, throttle stabilized, yaw pos */
};

enum OFFBOARD_CONTROL_FRAME {
	OFFBOARD_CONTROL_FRAME_LOCAL_NED = 0,
	OFFBOARD_CONTROL_FRAME_LOCAL_OFFSET_NED = 1,
	OFFBOARD_CONTROL_FRAME_BODY_NED = 2,
	OFFBOARD_CONTROL_FRAME_BODY_OFFSET_NED = 3,
	OFFBOARD_CONTROL_FRAME_GLOBAL = 4
};

/**
 * @addtogroup topics
 * @{
 */

struct offboard_control_setpoint_s {
	uint64_t timestamp;

	enum OFFBOARD_CONTROL_MODE mode;		 /**< The current control inputs mode */

	double position[3];	/**< lat, lon, alt / x, y, z */
	float velocity[3];	/**< x vel, y vel, z vel */
	float acceleration[3];	/**< x acc, y acc, z acc */
	float attitude[4];	/**< attitude of vehicle (quaternion) */
	float attitude_rate[3];	/**< body angular rates (x, y, z) */

	uint16_t ignore; /**< if field i is set to true, pi should be ignored */
	bool isForceSetpoint; /**< the acceleration vector should be interpreted as force */

	float override_mode_switch;

	float aux1_cam_pan_flaps;
	float aux2_cam_tilt;
	float aux3_cam_zoom;
	float aux4_cam_roll;

}; /**< offboard control inputs */
/**
 * @}
 */

inline bool offboard_control_sp_ignore_position(const struct offboard_control_setpoint_s &offboard_control_sp, int index) {
	return (bool)(offboard_control_sp.ignore & (1 << index));
}

inline bool offboard_control_sp_ignore_velocity(const struct offboard_control_setpoint_s &offboard_control_sp, int index) {
	return (bool)(offboard_control_sp.ignore & (1 << (3 + index)));
}

inline bool offboard_control_sp_ignore_acceleration(const struct offboard_control_setpoint_s &offboard_control_sp, int index) {
	return (bool)(offboard_control_sp.ignore & (1 << (6 + index)));
}

inline bool offboard_control_sp_ignore_bodyrates(const struct offboard_control_setpoint_s &offboard_control_sp, int index) {
	return (bool)(offboard_control_sp.ignore & (1 << (9 + index)));
}

/* register this as object request broker structure */
ORB_DECLARE(offboard_control_setpoint);

#endif
