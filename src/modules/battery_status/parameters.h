/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#pragma once

/**
 * @file parameters.h
 *
 * defines the list of parameters that are used within the sensors module
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */
#include <px4_platform_common/px4_config.h>
#include <drivers/drv_rc_input.h>

#include <parameters/param.h>
#include <mathlib/mathlib.h>

namespace battery_status
{

struct Parameters {
	float battery_voltage_scaling;
	float battery_current_scaling;
	float battery_current_offset;
	float battery_v_div;
	float battery_a_per_v;
	int32_t battery_source;
	int32_t battery_adc_channel;
};

struct ParameterHandles {
	param_t battery_voltage_scaling;
	param_t battery_current_scaling;
	param_t battery_current_offset;
	param_t battery_v_div;
	param_t battery_a_per_v;
	param_t battery_source;
	param_t battery_adc_channel;
};

/**
 * initialize ParameterHandles struct
 */
void initialize_parameter_handles(ParameterHandles &parameter_handles);


/**
 * Read out the parameters using the handles into the parameters struct.
 * @return 0 on success, <0 on error
 */
int update_parameters(const ParameterHandles &parameter_handles, Parameters &parameters);

} /* namespace sensors */
