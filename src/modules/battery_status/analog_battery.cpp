/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include <stdio.h>
#include <lib/battery/battery.h>
#include "analog_battery.h"

AnalogBattery::AnalogBattery(int index, ModuleParams *parent, const int sample_interval_us) :
	Battery(index, parent, sample_interval_us)
{
	char param_name[17];

	_analog_param_handles.v_offs_cur = param_find("BAT_V_OFFS_CURR");

	snprintf(param_name, sizeof(param_name), "BAT%d_V_DIV", index);
	_analog_param_handles.v_div = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_A_PER_V", index);
	_analog_param_handles.a_per_v = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_V_CHANNEL", index);
	_analog_param_handles.v_channel = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_I_CHANNEL", index);
	_analog_param_handles.i_channel = param_find(param_name);

	_analog_param_handles.v_div_old = param_find("BAT_V_DIV");
	_analog_param_handles.a_per_v_old = param_find("BAT_A_PER_V");
	_analog_param_handles.adc_channel_old = param_find("BAT_ADC_CHANNEL");
}

void
AnalogBattery::updateBatteryStatusADC(hrt_abstime timestamp, float voltage_raw, float current_raw,
				      int source, int priority, float throttle_normalized)
{
	float voltage_v = voltage_raw * _analog_params.v_div;
	float current_a = (current_raw - _analog_params.v_offs_cur) * _analog_params.a_per_v;

	bool connected = (voltage_v > BOARD_ADC_OPEN_CIRCUIT_V);// && (BOARD_ADC_OPEN_CIRCUIT_V <= BOARD_VALID_UV);

	// only publish if connected
	if (connected) {
		Battery::updateBatteryStatus(timestamp, voltage_v, current_a, connected, source, priority, throttle_normalized);
	}
}

int AnalogBattery::get_voltage_channel()
{
	if (_analog_params.v_channel >= 0) {
		return _analog_params.v_channel;

	} else {
#if defined(BOARD_BATT_V_LIST)
		static constexpr int batt_v_list[BOARD_NUMBER_BRICKS] = BOARD_BATT_V_LIST;
		return batt_v_list[_index - 1];
#else
		return 0;
#endif // BOARD_BATT_V_LIST
	}
}

int AnalogBattery::get_current_channel()
{
	if (_analog_params.i_channel >= 0) {
		return _analog_params.i_channel;

	} else {
#if defined(BOARD_BATT_I_LIST)
		static constexpr int batt_i_list[BOARD_NUMBER_BRICKS] = BOARD_BATT_I_LIST;
		return batt_i_list[_index - 1];
#else
		return 0;
#endif // BOARD_BATT_I_LIST
	}
}

void
AnalogBattery::updateParams()
{
	if (_index == 1) {
		migrateParam<float>(_analog_param_handles.v_div_old, _analog_param_handles.v_div, &_analog_params.v_div_old,
				    &_analog_params.v_div, _first_parameter_update);
		migrateParam<float>(_analog_param_handles.a_per_v_old, _analog_param_handles.a_per_v, &_analog_params.a_per_v_old,
				    &_analog_params.a_per_v, _first_parameter_update);
		migrateParam<int>(_analog_param_handles.adc_channel_old, _analog_param_handles.v_channel,
				  &_analog_params.adc_channel_old, &_analog_params.v_channel, _first_parameter_update);

	} else {
		param_get(_analog_param_handles.v_div, &_analog_params.v_div);
		param_get(_analog_param_handles.a_per_v, &_analog_params.a_per_v);
		param_get(_analog_param_handles.v_channel, &_analog_params.v_channel);
	}

	param_get(_analog_param_handles.i_channel, &_analog_params.i_channel);
	param_get(_analog_param_handles.v_offs_cur, &_analog_params.v_offs_cur);

	if (_analog_params.v_div < 0.0f) {
		/* apply scaling according to defaults if set to default */
		_analog_params.v_div = BOARD_BATTERY1_V_DIV;
		param_set_no_notification(_analog_param_handles.v_div, &_analog_params.v_div);

		if (_index == 1) {
			_analog_params.v_div_old = BOARD_BATTERY1_V_DIV;
			param_set_no_notification(_analog_param_handles.v_div_old, &_analog_params.v_div_old);
		}
	}

	if (_analog_params.a_per_v < 0.0f) {
		/* apply scaling according to defaults if set to default */

		_analog_params.a_per_v = BOARD_BATTERY1_A_PER_V;
		param_set_no_notification(_analog_param_handles.a_per_v, &_analog_params.a_per_v);

		if (_index == 1) {
			_analog_params.a_per_v_old = BOARD_BATTERY1_A_PER_V;
			param_set_no_notification(_analog_param_handles.a_per_v_old, &_analog_params.a_per_v_old);
		}
	}

	Battery::updateParams();
}
