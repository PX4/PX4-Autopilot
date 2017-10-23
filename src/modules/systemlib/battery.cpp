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

Battery::Battery() :
	SuperBlock(nullptr, "BAT"),
	_v_empty(this, "V_EMPTY"),
	_v_charged(this, "V_CHARGED"),
	_n_cells(this, "N_CELLS"),
	_capacity(this, "CAPACITY"),
	_v_load_drop(this, "V_LOAD_DROP"),
	_r_internal(this, "R_INTERNAL"),
	_low_thr(this, "LOW_THR"),
	_crit_thr(this, "CRIT_THR"),
	_emergency_thr(this, "EMERGEN_THR"),
	_voltage_filtered_v(-1.f),
	_current_filtered_a(-1.f),
	_discharged_mah(0.f),
	_remaining_voltage(1.f),
	_remaining_capacity(1.f),
	_remaining(1.f),
	_scale(1.f),
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
	battery_status->current_a = -1.f;
	battery_status->remaining = 1.f;
	battery_status->scale = 1.f;
	battery_status->cell_count = _n_cells.get();
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
	filterCurrent(current_a);
	sumDischarged(timestamp, current_a);
	estimateRemaining(_voltage_filtered_v, _current_filtered_a, throttle_normalized, armed);
	determineWarning(connected);
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
		battery_status->connected = connected;
		battery_status->system_source = selected_source;
		battery_status->priority = priority;
	}
}

void
Battery::filterVoltage(float voltage_v)
{
	if (_voltage_filtered_v < 0.f) {
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
	if (_current_filtered_a < 0.f) {
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
	if (current_a < 0.f) {
		// Because the measurement was invalid we need to stop integration
		// and re-initialize with the next valid measurement
		_last_timestamp = 0;
		return;
	}

	// Ignore first update because we don't know dT.
	if (_last_timestamp != 0) {
		_discharged_mah += current_a * ((float)(timestamp - _last_timestamp)) / 1e3f / 3600.f;
	}

	_last_timestamp = timestamp;
}

void
Battery::estimateRemaining(float voltage_v, float current_a, float throttle_normalized, bool armed)
{
	// correct battery voltage locally for load drop to avoid estimation fluctuations
	const float bat_r = _r_internal.get();

	if (bat_r >= 0.f) {
		voltage_v += bat_r * current_a;

	} else {
		// assume quadratic relation between throttle and current
		// good assumption if throttle represents RPM
		voltage_v += throttle_normalized * throttle_normalized * _v_load_drop.get();
	}

	// remaining battery capacity based on voltage
	const float cell_voltage = voltage_v / _n_cells.get();
	_remaining_voltage = math::gradual(cell_voltage, _v_empty.get(), _v_charged.get(), 0.f, 1.f);

	// choose which quantity we're using for final reporting
	if (_capacity.get() > 0.f) {
		// remaining battery capacity based on used current integrated time
		_remaining_capacity = math::max(1.f - _discharged_mah / _capacity.get(), 0.f);

		// if battery capacity is known, use discharged current for estimate,
		// but don't show more than voltage estimate
		_remaining = fminf(_remaining_voltage, _remaining_capacity);

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
		if (_remaining < _emergency_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_EMERGENCY)) {
			_warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

		} else if (_remaining < _crit_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_CRITICAL)) {
			_warning = battery_status_s::BATTERY_WARNING_CRITICAL;

		} else if (_remaining < _low_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_LOW)) {
			_warning = battery_status_s::BATTERY_WARNING_LOW;
		}
	}
}

void
Battery::computeScale()
{
	const float voltage_range = (_v_charged.get() - _v_empty.get());

	// reusing capacity calculation to get single cell voltage before drop
	const float bat_v = _v_empty.get() + (voltage_range * _remaining_voltage);

	_scale = _v_charged.get() / bat_v;

	if (_scale > 1.3f) { // Allow at most 30% compensation
		_scale = 1.3f;

	} else if (!PX4_ISFINITE(_scale) || _scale < 1.f) { // Shouldn't ever be more than the power at full battery
		_scale = 1.f;
	}
}
