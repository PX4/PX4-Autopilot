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
 * @file actuator_controls_effective.h
 *
 * Actuator control topics - mixer inputs.
 *
 * Values published to these topics are the outputs of the vehicle control
 * system and mixing process; they are the control-scale values that are
 * then fed to the actual actuator driver.
 *
 * Each topic can be published by a single controller
 */

#ifndef TOPIC_ACTUATOR_CONTROLS_EFFECTIVE_H
#define TOPIC_ACTUATOR_CONTROLS_EFFECTIVE_H

#include <stdint.h>
#include "../uORB.h"
#include "actuator_controls.h"

#define NUM_ACTUATOR_CONTROLS_EFFECTIVE		NUM_ACTUATOR_CONTROLS
#define NUM_ACTUATOR_CONTROL_GROUPS_EFFECTIVE	NUM_ACTUATOR_CONTROL_GROUPS	/**< for sanity checking */

/**
 * @addtogroup topics
 * @{
 */

struct actuator_controls_effective_s {
	uint64_t timestamp;
	float	control_effective[NUM_ACTUATOR_CONTROLS_EFFECTIVE];
};

/**
 * @}
 */

/* actuator control sets; this list can be expanded as more controllers emerge */
ORB_DECLARE(actuator_controls_effective_0);
ORB_DECLARE(actuator_controls_effective_1);
ORB_DECLARE(actuator_controls_effective_2);
ORB_DECLARE(actuator_controls_effective_3);

/* control sets with pre-defined applications */
#define ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE	ORB_ID(actuator_controls_effective_0)

#endif /* TOPIC_ACTUATOR_CONTROLS_EFFECTIVE_H */