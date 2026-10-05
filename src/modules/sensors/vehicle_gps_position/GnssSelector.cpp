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
	updateReceiverTimeouts(hrt_now_us);
	updateAvailability(hrt_now_us);

	_selected_instance = selectReceiver(hrt_now_us);
	_selected_has_new_sample = _updated[_selected_instance];

	if (_selected_has_new_sample) {
		if ((_output_instance >= 0) && (_output_instance != _selected_instance)) {
			_selection_count++;
		}

		_output_instance = _selected_instance;
	}

	for (uint8_t i = 0; i < GNSS_MAX_RECEIVERS; i++) {
		_updated[i] = false;
		_time_prev_us[i] = _sample[i].timestamp;
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
		const uint64_t timestamp = _sample[i].timestamp;
		UpdateInterval &interval = _interval[i];

		if (timestamp > _time_prev_us[i]) {
			// the first sample of a receiver has no previous one to compute an interval from
			if ((_time_prev_us[i] > 0) && (timestamp - _time_prev_us[i] < GNSS_TIMEOUT_US)) {
				const uint64_t interval_us = timestamp - _time_prev_us[i];
				interval.late = interval_us > lateIntervalUs(i);

				const float dt = 1e-6f * interval_us;
				interval.filtered_s = (interval.filtered_s > 0.f)
						      ? interval.filtered_s + dt / (INTERVAL_TIME_CONSTANT_S + dt) * (dt - interval.filtered_s)
						      : dt;

				if (interval.samples < INTERVAL_SETTLE_SAMPLES) {
					interval.samples++;

				} else {
					interval.usual_s = (interval.usual_s > 0.f) ? math::min(interval.usual_s, interval.filtered_s)
							   : interval.filtered_s;
				}
			}

		} else if ((timestamp > 0) && (hrt_now_us >= timestamp + GNSS_TIMEOUT_US)) {
			resetReceiver(i);
		}
	}
}

void GnssSelector::updateAvailability(uint64_t hrt_now_us)
{
	// A gap longer than a second means that updates stalled. It is weighted as one second, so that the update after it
	// doesn't overwrite the history.
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
	if (_interval[instance].usual_s > 0.f) {
		return math::constrain(static_cast<hrt_abstime>(LATE_INTERVAL_RATIO * _interval[instance].usual_s * 1e6f),
				       LATE_MIN_US, GNSS_TIMEOUT_US);
	}

	return GNSS_TIMEOUT_US;
}

bool GnssSelector::isUsable(int instance, uint64_t hrt_now_us) const
{
	if (isSilent(instance) || !_sample[instance].checks_passed || _interval[instance].late) {
		return false;
	}

	return hrt_now_us <= _sample[instance].timestamp + lateIntervalUs(instance);
}

GnssSelector::Rank GnssSelector::rank(int instance, uint64_t hrt_now_us) const
{
	if (!isUsable(instance, hrt_now_us)) {
		return RANK_UNUSABLE;
	}

	if (!_sample[instance].meets_requirements) {
		return RANK_USABLE;
	}

	return _sample[instance].rtk_fixed ? RANK_RTK_FIXED : RANK_REQUIREMENTS;
}

int GnssSelector::switchTo(int instance, uint8_t reason)
{
	_selection_reason = reason;
	return instance;
}

int GnssSelector::failOver(int instance, uint8_t reason)
{
	if (_armed) {
		_failed_while_armed[_selected_instance] = true;
	}

	return switchTo(instance, reason);
}

int GnssSelector::selectReceiver(uint64_t hrt_now_us)
{
	const int current = _selected_instance;
	const int preferred = hasPreferred() ? _preferred_instance : -1;

	// While disarmed the preferred receiver is used even when it fails its checks, so that the vehicle doesn't take off
	// on the other one
	if (!_armed && (preferred >= 0)) {
		return (current == preferred) ? current : switchTo(preferred, vehicle_gnss_s::SELECTION_PREFERRED);
	}

	// When the current is failing, a switch needs to happen (if there is a valid alternative available)
	const bool current_failing = isSilent(current) || hasFailed(current, hrt_now_us);

	// Find if there is a healthier receiver:
	// Intermittent failures never add up to FAIL_TIME_US, but cost availability. The replacement must have been usable
	// for FAIL_TIME_US, or a receiver that just failed would come straight back on its first usable sample.
	const float current_availability = _availability[current].getState();
	int healthiest_instance = -1;

	for (int i = 0; i < GNSS_MAX_RECEIVERS; i++) {
		if ((i != current) &&  isUsable(i, hrt_now_us)
		    && (current_failing || !_failed_while_armed[i]) // reconsider previously failed only if current is failing
		    && (hrt_now_us >= _time_usable_since_us[i] + FAIL_TIME_US)
		    && (_availability[i].getState() > current_availability + AVAILABILITY_MARGIN)
		    && ((healthiest_instance < 0) || (_availability[i].getState() > _availability[healthiest_instance].getState()))) {
			healthiest_instance = i;
		}
	}

	// Find if there is a receiver with a better rank:
	// A higher ranked receiver must keep its rank through the hold time. A receiver that failed while armed doesn't
	// count, so that the selection doesn't move back to it until the selected one fails.
	const Rank current_rank = rank(current, hrt_now_us);
	int best_rank_instance = -1;
	int best_rank = current_rank;

	for (int i = 0; i < GNSS_MAX_RECEIVERS; i++) {
		const Rank rank_i = rank(i, hrt_now_us);

		if ((i != current) && isComparablyAvailable(i, current)
		    && (current_failing || !_failed_while_armed[i]) // reconsider previously failed only if current is failing
		    && (rank_i > math::max((Rank)best_rank, RANK_USABLE))) {
			best_rank_instance = i;
			best_rank = rank_i;
		}
	}

	// There is a better option available: Find the best.
	if (current_failing || healthiest_instance >= 0 || best_rank_instance >= 0) {

		if (preferred >= 0) {
			// always take the preferred if it passes the strict checks
			if (_sample[preferred].meets_requirements) {
				return current_failing ? failOver(preferred, vehicle_gnss_s::SELECTION_UNHEALTHY) : switchTo(preferred,
						vehicle_gnss_s::SELECTION_PREFERRED);
			}

			// take preferred if best rank and comparable availability as the best receiver
			const Rank preferred_rank = rank(preferred, hrt_now_us);

			if (preferred_rank >= best_rank && isComparablyAvailable(preferred, healthiest_instance)) {
				return current_failing ? failOver(preferred, vehicle_gnss_s::SELECTION_UNHEALTHY) : switchTo(preferred,
						vehicle_gnss_s::SELECTION_PREFERRED);
			}

		}


		// no preference, prioritize the most available instance over the best ranked instance:
		// it is more important to have reliable lower-quality data than intermittent higher-quality data
		if (healthiest_instance >= 0) {
			return failOver(healthiest_instance, vehicle_gnss_s::SELECTION_UNHEALTHY);
		}

		if (best_rank_instance >= 0) {
			return current_failing ? failOver(preferred, vehicle_gnss_s::SELECTION_UNHEALTHY) : switchTo(best_rank_instance,
					vehicle_gnss_s::SELECTION_RANKED);
		}

		// none is better than the current
		return current;
	}

	return current;
}
