/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file GnssSelector.cpp
 */

#include "GnssSelector.hpp"

#include <lib/mathlib/mathlib.h>

GnssSelector::GnssSelector()
{
	for (AlphaFilter<float> &availability : _availability) {
		availability = AlphaFilter<float>(AVAILABILITY_TIME_CONSTANT_US);
	}
}

void GnssSelector::update(uint64_t hrt_now_us)
{
	_is_new_output_data_available = false;

	updateReceiverTimeouts(hrt_now_us);
	updateAvailability(hrt_now_us);

	const int selected = selectReceiver(hrt_now_us);

	_selected_instance = selected;
	_output_antenna_offset = _antenna_offset[selected];
	_is_new_output_data_available =  _updated[selected];

	if (_is_new_output_data_available) {
		if ((_output_instance >= 0) && (_output_instance != _selected_instance)) {
			_selection_count++;
		}

		_output_instance = _selected_instance;
	}

	for (uint8_t i = 0; i < GNSS_MAX_RECEIVERS; i++) {
		// clear updated flags
		_updated[i] = false;
		_time_prev_us[i] = _gnss_state[i].timestamp;
	}
}

uint8_t GnssSelector::getSelectionReason() const
{
	bool other_has_published = false;

	for (int i = 0; i < GNSS_MAX_RECEIVERS; i++) {
		if ((i != _selected_instance) && _has_published[i]) {
			other_has_published = true;
		}
	}

	if (!other_has_published) {
		return vehicle_gnss_s::SELECTION_ONLY;
	}

	if (hasPreferred() && (_selected_instance == _preferred_instance)) {
		return vehicle_gnss_s::SELECTION_PREFERRED;
	}

	// A return to a receiver that is no longer preferred explains nothing, as after the first selection
	if ((_selection_reason != REASON_INITIAL) && (_selection_reason != vehicle_gnss_s::SELECTION_PREFERRED)) {
		return _selection_reason;
	}

	if (hasPreferred()) {
		return isSilent(_preferred_instance) ? vehicle_gnss_s::SELECTION_TIMEOUT : vehicle_gnss_s::SELECTION_UNHEALTHY;
	}

	return vehicle_gnss_s::SELECTION_RANKED;
}

void GnssSelector::updateReceiverTimeouts(uint64_t hrt_now_us)
{
	for (uint8_t i = 0; i < GNSS_MAX_RECEIVERS; i++) {
		const uint64_t timestamp = _gnss_state[i].timestamp;

		if (timestamp > _time_prev_us[i]) {
			// the first sample of a receiver has no previous one to compute an interval from
			if ((_time_prev_us[i] > 0) && (timestamp - _time_prev_us[i] < GNSS_TIMEOUT_US)) {
				const uint64_t interval_us = timestamp - _time_prev_us[i];
				_sample_late[i] = interval_us > lateIntervalUs(i);

				const float dt = 1e-6f * interval_us;
				_interval_s[i] = (_interval_s[i] > 0.f)
						 ? _interval_s[i] + dt / (INTERVAL_TIME_CONSTANT_S + dt) * (dt - _interval_s[i])
						 : dt;

				if (_interval_samples[i] < INTERVAL_SETTLE_SAMPLES) {
					_interval_samples[i]++;

				} else {
					_usual_interval_s[i] = (_usual_interval_s[i] > 0.f) ? math::min(_usual_interval_s[i], _interval_s[i])
							       : _interval_s[i];
				}
			}

		} else if ((timestamp > 0) && (hrt_now_us >= timestamp + GNSS_TIMEOUT_US)) {
			// Timed out - kill the stored fix for this receiver. It may come back at another rate.
			_gnss_state[i].timestamp = 0;
			_gnss_state[i].fix_type = 0;
			_gnss_state[i].satellites_used = 0;
			_gnss_state[i].vel_ned_valid = 0;
			_checks_passed[i] = false;
			_interval_s[i] = 0.f;
			_interval_samples[i] = 0;
			_usual_interval_s[i] = 0.f;
			_sample_late[i] = false;
		}
	}
}

void GnssSelector::updateAvailability(uint64_t hrt_now_us)
{
	// Updates run whenever a receiver publishes, so a longer gap means that none did. It is weighted as one second,
	// so that the first sample after it doesn't overwrite the history.
	uint64_t dt_us = 0;

	if ((_time_last_update_us > 0) && (hrt_now_us > _time_last_update_us)) {
		dt_us = math::min(hrt_now_us - _time_last_update_us, static_cast<uint64_t>(1_s));
	}

	_time_last_update_us = hrt_now_us;

	for (int i = 0; i < GNSS_MAX_RECEIVERS; i++) {
		const bool usable = isUsable(i, hrt_now_us);

		if (usable) {
			_time_last_usable_us[i] = hrt_now_us;

			if (_time_usable_since_us[i] == 0) {
				_time_usable_since_us[i] = hrt_now_us;
			}

		} else {
			_time_usable_since_us[i] = 0;
		}

		if (usable && !_has_passed[i]) {
			// A receiver first passes after the pre-flight health time of its checks, which already proves it
			_availability[i].reset(1.f);
			_has_passed[i] = true;

		} else {
			_availability[i].update(usable ? 1.f : 0.f, dt_us);
		}
	}
}

hrt_abstime GnssSelector::lateIntervalUs(int instance) const
{
	if (_usual_interval_s[instance] > 0.f) {
		return math::constrain(static_cast<hrt_abstime>(LATE_INTERVAL_RATIO * _usual_interval_s[instance] * 1e6f),
				       LATE_MIN_US, GNSS_TIMEOUT_US);
	}

	return GNSS_TIMEOUT_US;
}

bool GnssSelector::isUsable(int instance, uint64_t hrt_now_us) const
{
	if (isSilent(instance) || !_checks_passed[instance] || _sample_late[instance]) {
		return false;
	}

	return hrt_now_us <= _gnss_state[instance].timestamp + lateIntervalUs(instance);
}

bool GnssSelector::isMoreAccurate(int instance, int than) const
{
	const sensor_gnss_s &a = _gnss_state[instance];
	const sensor_gnss_s &b = _gnss_state[than];

	// A receiver that doesn't report its accuracy isn't compared
	if (!(a.eph > 0.f) || !(a.epv > 0.f) || !(b.eph > 0.f) || !(b.epv > 0.f)) {
		return false;
	}

	return (math::max(a.eph, ACCURACY_FLOOR) <= ACCURACY_RATIO * math::max(b.eph, ACCURACY_FLOOR))
	       && (math::max(a.epv, ACCURACY_FLOOR) <= math::max(b.epv, ACCURACY_FLOOR));
}

int GnssSelector::switchTo(int instance, uint8_t reason)
{
	_selection_reason = reason;
	_switch_candidate = -1;
	return instance;
}

int GnssSelector::selectReceiver(uint64_t hrt_now_us)
{
	const int current = _selected_instance;
	const int preferred = hasPreferred() ? _preferred_instance : -1;

	// A replacement is the preferred receiver, otherwise the most available one
	const auto is_better_replacement = [&](int instance, int best) {
		return (best < 0) || (instance == preferred)
		       || ((best != preferred) && (_availability[instance].getState() > _availability[best].getState()));
	};

	if (isSilent(current)) {
		int best = -1;

		for (int i = 0; i < GNSS_MAX_RECEIVERS; i++) {
			if ((i != current) && !isSilent(i) && is_better_replacement(i, best)) {
				best = i;
			}
		}

		if (best < 0) {
			_switch_candidate = -1;
			return current;
		}

		// Before the first output no selected receiver stopped publishing
		return switchTo(best, (_output_instance >= 0) ? vehicle_gnss_s::SELECTION_TIMEOUT : REASON_INITIAL);
	}

	// A failed receiver is replaced by any usable one, however available that one was recently
	if (hasFailed(current, hrt_now_us)) {
		int best = -1;

		for (int i = 0; i < GNSS_MAX_RECEIVERS; i++) {
			if ((i != current) && isUsable(i, hrt_now_us) && is_better_replacement(i, best)) {
				best = i;
			}
		}

		if (best >= 0) {
			return switchTo(best, vehicle_gnss_s::SELECTION_UNHEALTHY);
		}
	}

	// Intermittent failures never add up to FAIL_TIME_US, but cost availability. The replacement must have been usable
	// for FAIL_TIME_US, or a receiver that just failed would come straight back on its first usable sample.
	const float current_availability = _availability[current].getState();
	int healthier = -1;

	for (int i = 0; i < GNSS_MAX_RECEIVERS; i++) {
		if ((i != current) && isUsable(i, hrt_now_us)
		    && (hrt_now_us >= _time_usable_since_us[i] + FAIL_TIME_US)
		    && (_availability[i].getState() > current_availability + AVAILABILITY_MARGIN)
		    && ((healthier < 0) || (_availability[i].getState() > _availability[healthier].getState()))) {
			healthier = i;
		}
	}

	if (healthier >= 0) {
		return switchTo(healthier, vehicle_gnss_s::SELECTION_UNHEALTHY);
	}

	const float comparable_availability = current_availability - 0.5f * AVAILABILITY_MARGIN;

	if (preferred >= 0) {
		_switch_candidate = -1;

		const hrt_abstime return_hold_us = _armed ? RETURN_HOLD_ARMED_US : 0;

		if ((current != preferred) && isUsable(preferred, hrt_now_us)
		    && (hrt_now_us >= _time_usable_since_us[preferred] + return_hold_us)
		    && (_availability[preferred].getState() >= comparable_availability)) {
			return switchTo(preferred, vehicle_gnss_s::SELECTION_PREFERRED);
		}

		return current;
	}

	int candidate = -1;

	for (int i = 0; i < GNSS_MAX_RECEIVERS; i++) {
		if ((i != current) && isUsable(i, hrt_now_us) && isMoreAccurate(i, current)
		    && (_availability[i].getState() >= comparable_availability)
		    && ((candidate < 0) || isMoreAccurate(i, candidate))) {
			candidate = i;
		}
	}

	if (candidate < 0) {
		_switch_candidate = -1;
		return current;
	}

	if (candidate != _switch_candidate) {
		_switch_candidate = candidate;
		_switch_candidate_since_us = hrt_now_us;
		return current;
	}

	if (hrt_now_us >= _switch_candidate_since_us + ACCURACY_HOLD_US) {
		return switchTo(candidate, vehicle_gnss_s::SELECTION_ACCURACY);
	}

	return current;
}
