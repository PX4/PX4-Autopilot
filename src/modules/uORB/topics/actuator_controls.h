/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file actuator_controls.h
 *
 * Actuator control topics - mixer inputs.
 *
 * Values published to these topics are the outputs of the vehicle control
 * system, and are expected to be mixed and used to drive the actuators
 * (servos, speed controls, etc.) that operate the vehicle.
 *
 * Each topic can be published by a single controller
 */

#ifndef TOPIC_ACTUATOR_CONTROLS_H
#define TOPIC_ACTUATOR_CONTROLS_H

#include <stdint.h>
#include "../uORB.h"

#define NUM_ACTUATOR_CONTROLS		8
#define NUM_ACTUATOR_CONTROL_GROUPS	4	/**< for sanity checking */

/* control sets with pre-defined applications */
#define ORB_ID_VEHICLE_ATTITUDE_CONTROLS	ORB_ID(actuator_controls_0)

/**
 * @addtogroup topics
 * @{
 */

struct actuator_controls_s {
	uint64_t timestamp;
	float	control[NUM_ACTUATOR_CONTROLS];
};

/**
 * @}
 */

/* actuator control sets; this list can be expanded as more controllers emerge */
ORB_DECLARE(actuator_controls_0);
ORB_DECLARE(actuator_controls_1);
ORB_DECLARE(actuator_controls_2);
ORB_DECLARE(actuator_controls_3);
ORB_DECLARE(actuator_controls_virtual_mc);
ORB_DECLARE(actuator_controls_virtual_fw);


#endif
