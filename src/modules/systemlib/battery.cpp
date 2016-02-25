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
	SuperBlock(NULL, ""),
	_param_v_empty(this, "BAT_V_EMPTY", false),
	_param_v_full(this, "BAT_V_CHARGED", false),
	_param_n_cells(this, "BAT_N_CELLS", false),
	_param_capacity(this, "BAT_CAPACITY", false),
	_param_v_load_drop(this, "BAT_V_LOAD_DROP", false),
	_voltage_filtered_v(0.0f),
	_throttle_filtered(0.0f),
	_discharged_mah(0.0f),
	_remaining(1.0f),
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
	battery_status->voltage_v = 0.0f;
	battery_status->voltage_filtered_v = 0.0f;
	battery_status->current_a = -1.0f;
	battery_status->discharged_mah = -1.0f;
	battery_status->cell_count = _param_n_cells.get();
	// TODO: check if it is sane to reset warning to NONE
	battery_status->warning = battery_status_s::BATTERY_WARNING_NONE;
}

void
Battery::updateBatteryStatus(hrt_abstime timestamp, float voltage_v, float current_a, float throttle_normalized,
			     battery_status_s *battery_status)
{
	filterVoltage(voltage_v);
	sumDischarged(timestamp, current_a);
	estimateRemaining(voltage_v, throttle_normalized);
	determineWarning();

	if (_voltage_filtered_v > 2.1f) {
		battery_status->voltage_v = voltage_v;
		battery_status->voltage_filtered_v = _voltage_filtered_v;
		battery_status->current_a = current_a;
		battery_status->discharged_mah = _discharged_mah;
		battery_status->cell_count = _param_n_cells.get();
		battery_status->warning = _warning;

	} else {
		reset(battery_status);
	}
}

void
Battery::filterVoltage(float voltage_v)
{
	// TODO: inspect that filter performance
	const float filtered_next = _voltage_filtered_v * 0.999f + voltage_v * 0.001f;

	if (PX4_ISFINITE(filtered_next)) {
		_voltage_filtered_v = filtered_next;
	}
}

void
Battery::sumDischarged(hrt_abstime timestamp, float current_a)
{
	// Ignore first update because we don't know dT.
	if (_last_timestamp != 0) {
		_discharged_mah = current_a * (timestamp - _last_timestamp) * 1.0e-3f / 3600.0f;
	}

	_last_timestamp = timestamp;
}

void
Battery::estimateRemaining(float voltage_v, float throttle_normalized)
{
	// XXX this time constant needs to become tunable but really, the right fix are smart batteries.
	const float filtered_next = _throttle_filtered * 0.97f + throttle_normalized * 0.03f;

	if (PX4_ISFINITE(filtered_next)) {
		_throttle_filtered = filtered_next;
	}

	/* remaining charge estimate based on voltage and internal resistance (drop under load) */
	const float bat_v_empty_dynamic = _param_v_empty.get() - (_param_v_load_drop.get() * _throttle_filtered);
	/* the range from full to empty is the same for batteries under load and without load,
	 * since the voltage drop applies to both the full and empty state
	 */
	const float voltage_range = (_param_v_full.get() - _param_v_empty.get());
	const float remaining_voltage = (voltage_v - (_param_n_cells.get() * bat_v_empty_dynamic))
					/ (_param_n_cells.get() * voltage_range);

	if (_param_capacity.get() > 0.0f) {
		/* if battery capacity is known, use discharged current for estimate, but don't show more than voltage estimate */
		_remaining = fminf(remaining_voltage, 1.0f - _discharged_mah / _param_capacity.get());

	} else {
		/* else use voltage */
		_remaining = remaining_voltage;
	}

	/* limit to sane values */
	_remaining = (_remaining < 0.0f) ? 0.0f : _remaining;
	_remaining = (_remaining > 1.0f) ? 1.0f : _remaining;
}

void
Battery::determineWarning()
{
	// TODO: Determine threshold or make params.
	if (_remaining < 0.18f) {
		_warning = battery_status_s::BATTERY_WARNING_LOW;

	} else if (_remaining < 0.09f) {
		_warning = battery_status_s::BATTERY_WARNING_CRITICAL;
	}
}
