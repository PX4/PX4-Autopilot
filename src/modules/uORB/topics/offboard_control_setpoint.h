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
enum OFFBOARD_CONTROL_MODE
{
	OFFBOARD_CONTROL_MODE_DIRECT = 0,
	OFFBOARD_CONTROL_MODE_DIRECT_RATES = 1,
	OFFBOARD_CONTROL_MODE_DIRECT_ATTITUDE = 2,
	OFFBOARD_CONTROL_MODE_DIRECT_VELOCITY = 3,
	OFFBOARD_CONTROL_MODE_DIRECT_POSITION = 4,
	OFFBOARD_CONTROL_MODE_ATT_YAW_RATE = 5,
	OFFBOARD_CONTROL_MODE_ATT_YAW_POS = 6,
	OFFBOARD_CONTROL_MODE_MULTIROTOR_SIMPLE = 7, /**< roll / pitch rotated aligned to the takeoff orientation, throttle stabilized, yaw pos */
};

/**
 * @addtogroup topics
 * @{
 */

struct offboard_control_setpoint_s {
	uint64_t timestamp;

	enum OFFBOARD_CONTROL_MODE mode;		 /**< The current control inputs mode */
	bool armed;	/**< Armed flag set, yes / no */
	float p1;	/**< ailerons roll / roll rate input */
	float p2;	/**< elevator / pitch / pitch rate */
	float p3;	/**< rudder / yaw rate / yaw */
	float p4;	/**< throttle / collective thrust / altitude */

	float override_mode_switch;

	float aux1_cam_pan_flaps;
	float aux2_cam_tilt;
	float aux3_cam_zoom;
	float aux4_cam_roll;

}; /**< offboard control inputs */
/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(offboard_control_setpoint);

#endif
