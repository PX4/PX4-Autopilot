/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file motor_outputs.hpp
 *
 * Motor assignment of the flight controller's own outputs, for code that takes over the ESC
 * signal pins and has no MixingOutput. Needs the output_functions_header target.
 */

#pragma once

#include <mixer_module/output_functions.hpp>
#include <parameters/param.h>
#include <px4_platform_common/px4_config.h>

#include <stdio.h>

/**
 * Motor assigned to an output by its function parameter
 *
 * @param output Output channel, starting at 0
 * @return Motor index, starting at 0, or -1 if the output is not assigned to a motor
 */
static inline int motor_index_of_output(unsigned output)
{
	// The outputs of a board with an IO processor are the AUX outputs
#if defined(CONFIG_BOARD_IO)
	static constexpr char prefix[] = "PWM_AUX";
#else
	static constexpr char prefix[] = "PWM_MAIN";
#endif

	char name[17];
	snprintf(name, sizeof(name), "%s_FUNC%u", prefix, output + 1);
	const param_t param = param_find(name);
	int32_t function = 0;

	if (param == PARAM_INVALID || param_get(param, &function) != 0
	    || function < (int32_t)OutputFunction::Motor1 || function > (int32_t)OutputFunction::MotorMax) {
		return -1;
	}

	return function - (int32_t)OutputFunction::Motor1;
}
