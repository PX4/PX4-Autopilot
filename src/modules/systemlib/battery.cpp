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

Battery::Battery() :
	SuperBlock(nullptr, "BAT"),
	_param_v_empty(this, "V_EMPTY"),
	_param_v_full(this, "V_CHARGED"),
	_param_n_cells(this, "N_CELLS"),
	_param_capacity(this, "CAPACITY"),
	_param_v_load_drop(this, "V_LOAD_DROP"),
	_param_r_internal(this, "R_INTERNAL"),
	_param_low_thr(this, "LOW_THR"),
	_param_crit_thr(this, "CRIT_THR"),
	_voltage_filtered_v(-1.0f),
	_discharged_mah(0.0f),
	_remaining_voltage(1.0f),
	_remaining_capacity(1.0f),
	_remaining(1.0f),
	_scale(1.0f),
	_warning(battery_status_s::BATTERY_WARNING_NONE),
	_last_timestamp(0)
{
	/* load initial params */
	updateParams();
}

Battery::~Battery()
{
}

void
Battery::reset(battery_status_s *battery_status)
{
	memset(battery_status, 0, sizeof(*battery_status));
	battery_status->current_a = -1.0f;
	battery_status->remaining = 1.0f;
	battery_status->scale = 1.0f;
	battery_status->cell_count = _param_n_cells.get();
	// TODO: check if it is sane to reset warning to NONE
	battery_status->warning = battery_status_s::BATTERY_WARNING_NONE;
	battery_status->connected = false;
}

void
Battery::updateBatteryStatus(hrt_abstime timestamp, float voltage_v, float current_a, float throttle_normalized,
			     bool armed, battery_status_s *battery_status)
{
	reset(battery_status);
	battery_status->timestamp = timestamp;
	filterVoltage(voltage_v);
	filterCurrent(current_a);
	sumDischarged(timestamp, current_a);
	estimateRemaining(voltage_v, current_a, throttle_normalized, armed);
	determineWarning();
	computeScale();

	if (_voltage_filtered_v > 2.1f) {
		battery_status->voltage_v = voltage_v;
		battery_status->voltage_filtered_v = _voltage_filtered_v;
		battery_status->scale = _scale;
		battery_status->current_a = current_a;
		battery_status->current_filtered_a = _current_filtered_a;
		battery_status->discharged_mah = _discharged_mah;
		battery_status->warning = _warning;
		battery_status->remaining = _remaining;
		battery_status->connected = true;
	}
}

void
Battery::filterVoltage(float voltage_v)
{
	if (_voltage_filtered_v < 0.0f) {
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
	if (_current_filtered_a < 0.0f) {
		_current_filtered_a = current_a;
	}

	// ADC poll is at 100Hz, this will perform a low pass over approx 500ms
	const float filtered_next = _current_filtered_a * 0.98f + current_a * 0.02f;

	if (PX4_ISFINITE(filtered_next)) {
		_current_filtered_a = filtered_next;
	}
}


void
Battery::sumDischarged(hrt_abstime timestamp, float current_a)
{
	// Not a valid measurement
	if (current_a < 0.0f) {
		// Because the measurement was invalid we need to stop integration
		// and re-initialize with the next valid measurement
		_last_timestamp = 0;
		return;
	}

	// Ignore first update because we don't know dT.
	if (_last_timestamp != 0) {
		_discharged_mah += current_a * ((float)(timestamp - _last_timestamp)) / 1e3f / 3600.0f;
	}

	_last_timestamp = timestamp;
}

void
Battery::estimateRemaining(float voltage_v, float current_a, float throttle_normalized, bool armed)
{
	const float bat_r = _param_r_internal.get();

	// remaining charge estimate based on voltage and internal resistance (drop under load)
	float bat_v_empty_dynamic = _param_v_empty.get();

	if (bat_r >= 0.0f) {
		bat_v_empty_dynamic -= current_a * bat_r;

	} else {
		// assume 10% voltage drop of the full drop range with motors idle
		const float thr = (armed) ? ((fabsf(throttle_normalized) + 0.1f) / 1.1f) : 0.0f;

		bat_v_empty_dynamic -= _param_v_load_drop.get() * thr;
	}

	// the range from full to empty is the same for batteries under load and without load,
	// since the voltage drop applies to both the full and empty state
	const float voltage_range = (_param_v_full.get() - _param_v_empty.get());

	// remaining battery capacity based on voltage
	const float rvoltage = (voltage_v - (_param_n_cells.get() * bat_v_empty_dynamic))
			       / (_param_n_cells.get() * voltage_range);
	const float rvoltage_filt = _remaining_voltage * 0.99f + rvoltage * 0.01f;

	if (PX4_ISFINITE(rvoltage_filt)) {
		_remaining_voltage = rvoltage_filt;
	}

	// remaining battery capacity based on used current integrated time
	const float rcap = 1.0f - _discharged_mah / _param_capacity.get();
	const float rcap_filt = _remaining_capacity * 0.99f + rcap * 0.01f;

	if (PX4_ISFINITE(rcap_filt)) {
		_remaining_capacity = rcap_filt;
	}

	// limit to sane values
	_remaining_voltage = (_remaining_voltage < 0.0f) ? 0.0f : _remaining_voltage;
	_remaining_voltage = (_remaining_voltage > 1.0f) ? 1.0f : _remaining_voltage;

	_remaining_capacity = (_remaining_capacity < 0.0f) ? 0.0f : _remaining_capacity;
	_remaining_capacity = (_remaining_capacity > 1.0f) ? 1.0f : _remaining_capacity;

	// choose which quantity we're using for final reporting
	if (_param_capacity.get() > 0.0f) {
		// if battery capacity is known, use discharged current for estimate,
		// but don't show more than voltage estimate
		_remaining = fminf(_remaining_voltage, _remaining_capacity);

	} else {
		// else use voltage
		_remaining = _remaining_voltage;
	}
}

void
Battery::determineWarning()
{
	// Smallest values must come first
	if (_remaining < _param_crit_thr.get()) {
		_warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (_remaining < _param_low_thr.get()) {
		_warning = battery_status_s::BATTERY_WARNING_LOW;
	}
}

void
Battery::computeScale()
{
	const float voltage_range = (_param_v_full.get() - _param_v_empty.get());

	// reusing capacity calculation to get single cell voltage before drop
	const float bat_v = _param_v_empty.get() + (voltage_range * _remaining_voltage);

	_scale = _param_v_full.get() / bat_v;

	if (_scale > 1.3f) { // Allow at most 30% compensation
		_scale = 1.3f;

	} else if (!PX4_ISFINITE(_scale) || _scale < 1.0f) { // Shouldn't ever be more than the power at full battery
		_scale = 1.0f;
	}
}
