/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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
 * @file parameters.cpp
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "parameters.h"

#include <px4_platform_common/log.h>

namespace battery_status
{

void initialize_parameter_handles(ParameterHandles &parameter_handles)
{
	parameter_handles.battery_voltage_scaling = param_find("BAT_CNT_V_VOLT");
	parameter_handles.battery_current_scaling = param_find("BAT_CNT_V_CURR");
	parameter_handles.battery_current_offset = param_find("BAT_V_OFFS_CURR");
	parameter_handles.battery_v_div = param_find("BAT_V_DIV");
	parameter_handles.battery_a_per_v = param_find("BAT_A_PER_V");
	parameter_handles.battery_source = param_find("BAT_SOURCE");
	parameter_handles.battery_adc_channel = param_find("BAT_ADC_CHANNEL");
}

int update_parameters(const ParameterHandles &parameter_handles, Parameters &parameters)
{
	int ret = PX4_OK;

	const char *paramerr = "FAIL PARM LOAD";

	/* scaling of ADC ticks to battery voltage */
	if (param_get(parameter_handles.battery_voltage_scaling, &(parameters.battery_voltage_scaling)) != OK) {
		PX4_WARN("%s", paramerr);

	} else if (parameters.battery_voltage_scaling < 0.0f) {
		/* apply scaling according to defaults if set to default */
		parameters.battery_voltage_scaling = (3.3f / 4096);
		param_set_no_notification(parameter_handles.battery_voltage_scaling, &parameters.battery_voltage_scaling);
	}

	/* scaling of ADC ticks to battery current */
	if (param_get(parameter_handles.battery_current_scaling, &(parameters.battery_current_scaling)) != OK) {
		PX4_WARN("%s", paramerr);

	} else if (parameters.battery_current_scaling < 0.0f) {
		/* apply scaling according to defaults if set to default */
		parameters.battery_current_scaling = (3.3f / 4096);
		param_set_no_notification(parameter_handles.battery_current_scaling, &parameters.battery_current_scaling);
	}

	if (param_get(parameter_handles.battery_current_offset, &(parameters.battery_current_offset)) != OK) {
		PX4_WARN("%s", paramerr);

	}

	if (param_get(parameter_handles.battery_v_div, &(parameters.battery_v_div)) != OK) {
		PX4_WARN("%s", paramerr);
		parameters.battery_v_div = 0.0f;

	} else if (parameters.battery_v_div <= 0.0f) {
		/* apply scaling according to defaults if set to default */

		parameters.battery_v_div = BOARD_BATTERY1_V_DIV;
		param_set_no_notification(parameter_handles.battery_v_div, &parameters.battery_v_div);
	}

	if (param_get(parameter_handles.battery_a_per_v, &(parameters.battery_a_per_v)) != OK) {
		PX4_WARN("%s", paramerr);
		parameters.battery_a_per_v = 0.0f;

	} else if (parameters.battery_a_per_v <= 0.0f) {
		/* apply scaling according to defaults if set to default */

		parameters.battery_a_per_v = BOARD_BATTERY1_A_PER_V;
		param_set_no_notification(parameter_handles.battery_a_per_v, &parameters.battery_a_per_v);
	}

	param_get(parameter_handles.battery_source,      &(parameters.battery_source));
	param_get(parameter_handles.battery_adc_channel, &(parameters.battery_adc_channel));

	return ret;
}

} /* namespace battery_status */
