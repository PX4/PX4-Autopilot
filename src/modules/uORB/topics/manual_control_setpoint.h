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
 * @file manual_control_setpoint.h
 * Definition of the manual_control_setpoint uORB topic.
 */

#ifndef TOPIC_MANUAL_CONTROL_SETPOINT_H_
#define TOPIC_MANUAL_CONTROL_SETPOINT_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

struct manual_control_setpoint_s {
	uint64_t timestamp;

	float roll;			 	/**< ailerons roll / roll rate input */
	float pitch;				/**< elevator / pitch / pitch rate */
	float yaw;				/**< rudder / yaw rate / yaw */
	float throttle;				/**< throttle / collective thrust / altitude */

	float mode_switch;			/**< mode 3 position switch (mandatory): manual, assisted, auto */
	float return_switch;			/**< land 2 position switch (mandatory): land, no effect */
	float assisted_switch;			/**< assisted 2 position switch (optional): seatbelt, simple */
	float mission_switch;		/**< mission 2 position switch (optional): mission, loiter */

	/**
	 * Any of the channels below may not be available and be set to NaN
	 * to indicate that it does not contain valid data.
	 */

	// XXX needed or parameter?
	//float auto_offboard_input_switch;	/**< controller setpoint source (0 = onboard, 1 = offboard) */

	float flaps;				/**< flap position */

	float aux1;				/**< default function: camera yaw / azimuth */
	float aux2;				/**< default function: camera pitch / tilt */
	float aux3;				/**< default function: camera trigger */
	float aux4;				/**< default function: camera roll */
	float aux5;				/**< default function: payload drop */

}; /**< manual control inputs */

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(manual_control_setpoint);

#endif
