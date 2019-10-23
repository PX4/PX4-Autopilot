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

/**
 * @file battery.cpp
 *
 * Library calls for battery functionality.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include "battery.h"
#include <mathlib/mathlib.h>
#include <cstring>
#include <px4_defines.h>

Battery::Battery() :
	ModuleParams(nullptr),
	_warning(battery_status_s::BATTERY_WARNING_NONE),
	_last_timestamp(0)
{
}

void
Battery::reset(battery_status_s *battery_status)
{
	memset(battery_status, 0, sizeof(*battery_status));
	battery_status->current_a = -1.f;
	battery_status->remaining = 1.f;
	battery_status->scale = 1.f;
	battery_status->cell_count = _param_bat_n_cells.get();
	// TODO: check if it is sane to reset warning to NONE
	battery_status->warning = battery_status_s::BATTERY_WARNING_NONE;
	battery_status->connected = false;
}

void
Battery::updateBatteryStatus(hrt_abstime timestamp, float voltage_v, float current_a,
			     bool connected, bool selected_source, int priority,
			     float throttle_normalized,
			     bool armed, battery_status_s *battery_status)
{
	reset(battery_status);
	battery_status->timestamp = timestamp;
	filterVoltage(voltage_v);
	filterThrottle(throttle_normalized);
	filterCurrent(current_a);
	sumDischarged(timestamp, current_a);
	estimateRemaining(_voltage_filtered_v, _current_filtered_a, _throttle_filtered, armed);
	computeScale();

	if (_battery_initialized) {
		determineWarning(connected);
	}

	if (_voltage_filtered_v > 2.1f) {
		_battery_initialized = true;
		battery_status->voltage_v = voltage_v;
		battery_status->voltage_filtered_v = _voltage_filtered_v;
		battery_status->scale = _scale;
		battery_status->current_a = current_a;
		battery_status->current_filtered_a = _current_filtered_a;
		battery_status->discharged_mah = _discharged_mah;
		battery_status->warning = _warning;
		battery_status->remaining = _remaining;
		battery_status->connected = connected;
		battery_status->system_source = selected_source;
		battery_status->priority = priority;
	}
}

void
Battery::filterVoltage(float voltage_v)
{
	if (!_battery_initialized) {
		_voltage_filtered_v = voltage_v;
	}

	// TODO: inspect that filter performance
	const float filtered_next = _voltage_filtered_v * 0.99f + voltage_v * 0.01f;

	if (PX4_ISFINITE(filtered_next)) {
		_voltage_filtered_v = filtered_next;
	}
}

void
Battery::filterCurrent(float current_a)
{
	if (!_battery_initialized) {
		_current_filtered_a = current_a;
	}

	// ADC poll is at 100Hz, this will perform a low pass over approx 500ms
	const float filtered_next = _current_filtered_a * 0.98f + current_a * 0.02f;

	if (PX4_ISFINITE(filtered_next)) {
		_current_filtered_a = filtered_next;
	}
}

void Battery::filterThrottle(float throttle)
{
	if (!_battery_initialized) {
		_throttle_filtered = throttle;
	}

	const float filtered_next = _throttle_filtered * 0.99f + throttle * 0.01f;

	if (PX4_ISFINITE(filtered_next)) {
		_throttle_filtered = filtered_next;
	}
}

void
Battery::sumDischarged(hrt_abstime timestamp, float current_a)
{
	// Not a valid measurement
	if (current_a < 0.f) {
		// Because the measurement was invalid we need to stop integration
		// and re-initialize with the next valid measurement
		_last_timestamp = 0;
		return;
	}

	// Ignore first update because we don't know dt.
	if (_last_timestamp != 0) {
		const float dt = (timestamp - _last_timestamp) / 1e6;
		// mAh since last loop: (current[A] * 1000 = [mA]) * (dt[s] / 3600 = [h])
		_discharged_mah_loop = (current_a * 1e3f) * (dt / 3600.f);
		_discharged_mah += _discharged_mah_loop;
	}

	_last_timestamp = timestamp;
}

void
Battery::estimateRemaining(float voltage_v, float current_a, float throttle, bool armed)
{
	// remaining battery capacity based on voltage
	float cell_voltage = voltage_v / _param_bat_n_cells.get();

	// correct battery voltage locally for load drop to avoid estimation fluctuations
	if (_param_bat_r_internal.get() >= 0.f) {
		cell_voltage += _param_bat_r_internal.get() * current_a;

	} else {
		// assume linear relation between throttle and voltage drop
		cell_voltage += throttle * _param_bat_v_load_drop.get();
	}

	_remaining_voltage = math::gradual(cell_voltage, _param_bat_v_empty.get(), _param_bat_v_charged.get(), 0.f, 1.f);

	// choose which quantity we're using for final reporting
	if (_param_bat_capacity.get() > 0.f) {
		// if battery capacity is known, fuse voltage measurement with used capacity
		if (!_battery_initialized) {
			// initialization of the estimation state
			_remaining = _remaining_voltage;

		} else {
			// The lower the voltage the more adjust the estimate with it to avoid deep discharge
			const float weight_v = 3e-4f * (1 - _remaining_voltage);
			_remaining = (1 - weight_v) * _remaining + weight_v * _remaining_voltage;
			// directly apply current capacity slope calculated using current
			_remaining -= _discharged_mah_loop / _param_bat_capacity.get();
			_remaining = math::max(_remaining, 0.f);
		}

	} else {
		// else use voltage
		_remaining = _remaining_voltage;
	}
}

void
Battery::determineWarning(bool connected)
{
	if (connected) {
		// propagate warning state only if the state is higher, otherwise remain in current warning state
		if (_remaining < _param_bat_emergen_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_EMERGENCY)) {
			_warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

		} else if (_remaining < _param_bat_crit_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_CRITICAL)) {
			_warning = battery_status_s::BATTERY_WARNING_CRITICAL;

		} else if (_remaining < _param_bat_low_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_LOW)) {
			_warning = battery_status_s::BATTERY_WARNING_LOW;
		}
	}
}

void
Battery::computeScale()
{
	const float voltage_range = (_param_bat_v_charged.get() - _param_bat_v_empty.get());

	// reusing capacity calculation to get single cell voltage before drop
	const float bat_v = _param_bat_v_empty.get() + (voltage_range * _remaining_voltage);

	_scale = _param_bat_v_charged.get() / bat_v;

	if (_scale > 1.3f) { // Allow at most 30% compensation
		_scale = 1.3f;

	} else if (!PX4_ISFINITE(_scale) || _scale < 1.f) { // Shouldn't ever be more than the power at full battery
		_scale = 1.f;
	}
}
