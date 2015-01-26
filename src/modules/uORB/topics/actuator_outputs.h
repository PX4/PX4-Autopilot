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
 * @file actuator_outputs.h
 *
 * Actuator output values.
 *
 * Values published to these topics are the outputs of the control mixing
 * system as sent to the actuators (servos, motors, etc.) that operate
 * the vehicle.
 *
 * Each topic can be published by a single output driver.
 */

#ifndef TOPIC_ACTUATOR_OUTPUTS_H
#define TOPIC_ACTUATOR_OUTPUTS_H

#include <stdint.h>
#include "../uORB.h"

#define NUM_ACTUATOR_OUTPUTS		16
#define NUM_ACTUATOR_OUTPUT_GROUPS	4	/**< for sanity checking */

/**
 * @addtogroup topics
 * @{
 */

struct actuator_outputs_s {
	uint64_t timestamp;				/**< output timestamp in us since system boot */
	float	output[NUM_ACTUATOR_OUTPUTS];		/**< output data, in natural output units */
	unsigned noutputs;					/**< valid outputs */
};

/**
 * @}
 */

/* actuator output sets; this list can be expanded as more drivers emerge */
ORB_DECLARE(actuator_outputs);

#endif
