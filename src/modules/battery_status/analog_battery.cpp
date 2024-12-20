/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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

// Defaults to use if the parameters are not set
#if BOARD_NUMBER_BRICKS > 0
#if defined(BOARD_BATT_V_LIST) && defined(BOARD_BATT_I_LIST)
static constexpr int   DEFAULT_V_CHANNEL[BOARD_NUMBER_BRICKS] = BOARD_BATT_V_LIST;
static constexpr int   DEFAULT_I_CHANNEL[BOARD_NUMBER_BRICKS] = BOARD_BATT_I_LIST;
#else
#error  "BOARD_BATT_V_LIST and BOARD_BATT_I_LIST need to be defined"
#endif
#else
static constexpr int DEFAULT_V_CHANNEL[1] = {-1};
static constexpr int DEFAULT_I_CHANNEL[1] = {-1};
#endif

AnalogBattery::AnalogBattery(int index, ModuleParams *parent, const int sample_interval_us, const uint8_t source,
			     const uint8_t priority) :
	Battery(index, parent, sample_interval_us, source)
{
	Battery::setPriority(priority);
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

	snprintf(param_name, sizeof(param_name), "BAT%d_I_OVERWRITE", index);
	_analog_param_handles.i_overwrite = param_find(param_name);
}

void
AnalogBattery::updateBatteryStatusADC(hrt_abstime timestamp, float voltage_raw, float current_raw)
{
	float voltage_v = voltage_raw * _analog_params.v_div;
	const bool connected = voltage_v > BOARD_ADC_OPEN_CIRCUIT_V &&
			       (BOARD_ADC_OPEN_CIRCUIT_V <= BOARD_VALID_UV || is_valid());
	float current_a = (current_raw - _analog_params.v_offs_cur) * _analog_params.a_per_v;

#if defined(BOARD_BATTERY_ADC_VOLTAGE_FILTER_S) || defined(BOARD_BATTERY_ADC_CURRENT_FILTER_S)

	if (_last_timestamp == 0) {
		_last_timestamp = timestamp;
	}

	const float dt = (timestamp - _last_timestamp) / 1e6f;
	_last_timestamp = timestamp;
#endif

#ifdef BOARD_BATTERY_ADC_VOLTAGE_FILTER_S
	voltage_v = _voltage_filter.update(fmaxf(voltage_v, 0.f), dt);
#endif

#ifdef BOARD_BATTERY_ADC_CURRENT_FILTER_S
	current_a = _current_filter.update(fmaxf(current_a, 0.f), dt);
#endif

	// Overwrite the measured current if current overwrite is defined and vehicle is unarmed
	if (_analog_params.i_overwrite > 0) {
		updateTopics();

		if (_arming_state == vehicle_status_s::ARMING_STATE_DISARMED) {
			current_a = _analog_params.i_overwrite;
		}
	}

	Battery::setConnected(connected);
	Battery::updateVoltage(voltage_v);
	Battery::updateCurrent(current_a);
	Battery::updateAndPublishBatteryStatus(timestamp);
}

bool AnalogBattery::is_valid()
{
#ifdef BOARD_BRICK_VALID_LIST
	bool valid[BOARD_NUMBER_BRICKS] = BOARD_BRICK_VALID_LIST;
	return valid[_index - 1];
#else
	// TODO: Maybe return false instead?
	return true;
#endif
}

int AnalogBattery::get_voltage_channel()
{
	if (_analog_params.v_channel >= 0) {
		return _analog_params.v_channel;

	} else {
		return DEFAULT_V_CHANNEL[_index - 1];
	}
}

int AnalogBattery::get_current_channel()
{
	if (_analog_params.i_channel >= 0) {
		return _analog_params.i_channel;

	} else {
		return DEFAULT_I_CHANNEL[_index - 1];
	}
}

void
AnalogBattery::updateParams()
{
	param_get(_analog_param_handles.v_div, &_analog_params.v_div);
	param_get(_analog_param_handles.a_per_v, &_analog_params.a_per_v);
	param_get(_analog_param_handles.v_channel, &_analog_params.v_channel);
	param_get(_analog_param_handles.i_channel, &_analog_params.i_channel);
	param_get(_analog_param_handles.i_overwrite, &_analog_params.i_overwrite);
	param_get(_analog_param_handles.v_offs_cur, &_analog_params.v_offs_cur);

	Battery::updateParams();
}

void AnalogBattery::updateTopics()
{
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_arming_state = vehicle_status.arming_state;
	}
}
