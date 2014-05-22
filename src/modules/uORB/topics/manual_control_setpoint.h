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
 * Switch position
 */
typedef enum {
	SWITCH_POS_NONE = 0,	/**< switch is not mapped */
	SWITCH_POS_ON,			/**< switch activated (value = 1) */
	SWITCH_POS_MIDDLE,		/**< middle position (value = 0) */
	SWITCH_POS_OFF			/**< switch not activated (value = -1) */
} switch_pos_t;

/**
 * @addtogroup topics
 * @{
 */

struct manual_control_setpoint_s {
	uint64_t timestamp;

	/**
	 * Any of the channels may not be available and be set to NaN
	 * to indicate that it does not contain valid data.
	 * The variable names follow the definition of the
	 * MANUAL_CONTROL mavlink message.
	 * The default range is from -1 to 1 (mavlink message -1000 to 1000)
	 * The range for the z variable is defined from 0 to 1. (The z field of
	 * the MANUAL_CONTROL mavlink message is defined from -1000 to 1000)
	 */
	float x;			/**< stick position in x direction -1..1
					  in general corresponds to forward/back motion or pitch of vehicle,
					  in general a positive value means forward or negative pitch and
					  a negative value means backward or positive pitch */
	float y;			/**< stick position in y direction -1..1
					  in general corresponds to right/left motion or roll of vehicle,
					  in general a positive value means right or positive roll and
					  a negative value means left or negative roll */
	float z;			/**< throttle stick position 0..1
					  in general corresponds to up/down motion or thrust of vehicle,
					  in general the value corresponds to the demanded throttle by the user,
					  if the input is used for setting the setpoint of a vertical position
					  controller any value > 0.5 means up and any value < 0.5 means down */
	float r;			/**< yaw stick/twist positon, -1..1
					  in general corresponds to the righthand rotation around the vertical
					  (downwards) axis of the vehicle */
	float flaps;			/**< flap position */
	float aux1;			/**< default function: camera yaw / azimuth */
	float aux2;			/**< default function: camera pitch / tilt */
	float aux3;			/**< default function: camera trigger */
	float aux4;			/**< default function: camera roll */
	float aux5;			/**< default function: payload drop */

	switch_pos_t mode_switch;			/**< main mode 3 position switch (mandatory): _MANUAL_, ASSIST, AUTO */
	switch_pos_t return_switch;			/**< return to launch 2 position switch (mandatory): _NORMAL_, RTL */
	switch_pos_t posctl_switch;			/**< position control 2 position switch (optional): _ALTCTL_, POSCTL */
	switch_pos_t loiter_switch;			/**< loiter 2 position switch (optional): _MISSION_, LOITER */
	switch_pos_t acro_switch;			/**< acro 2 position switch (optional): _MANUAL_, ACRO */
}; /**< manual control inputs */

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(manual_control_setpoint);

#endif
