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
	_selection_reason = selectionReason();
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

uint8_t GnssSelector::selectionReason() const
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

	if (_switch_reason != REASON_INITIAL) {
		return _switch_reason;
	}

	if (hasPreferred() && (_selected_instance == _preferred_instance)) {
		return vehicle_gnss_s::SELECTION_PREFERRED;
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
	if (_interval[instance].samples >= INTERVAL_SETTLE_SAMPLES) {
		return math::constrain(static_cast<hrt_abstime>(LATE_INTERVAL_RATIO * _interval[instance].filtered_s * 1e6f),
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

	return (_sample[instance].rtk_fixed && !hasPreferred()) ? RANK_RTK_FIXED : RANK_REQUIREMENTS;
}

int GnssSelector::bestReplacement(int current, bool usable_only, uint64_t hrt_now_us) const
{
	int best = -1;

	for (int i = 0; i < GNSS_MAX_RECEIVERS; i++) {
		if ((i == current) || isSilent(i) || (usable_only && !isUsable(i, hrt_now_us))) {
			continue;
		}

		if ((best < 0) || isBetterReceiver(i, best, hrt_now_us)) {
			best = i;
		}
	}

	return best;
}

int GnssSelector::bestCandidate(uint64_t hrt_now_us) const
{
	int best = -1;

	for (int i = 0; i < GNSS_MAX_RECEIVERS; i++) {
		if (!isUsable(i, hrt_now_us)) {
			continue;
		}

		if ((best < 0) || isBetterReceiver(i, best, hrt_now_us, AVAILABILITY_MARGIN)) {
			best = i;
		}
	}

	return best;
}

bool GnssSelector::isBetterReceiver(int instance, int other, uint64_t hrt_now_us, float tiebreaker_availability_margin) const
{
	if (instance < 0) {
		return false;
	}

	if (other < 0) {
		return true;
	}

	const Rank instance_rank = rank(instance, hrt_now_us);
	const Rank other_rank = rank(other, hrt_now_us);

	// A receiver that failed while armed is never better than a usable one that didn't
	if (other_rank >= RANK_USABLE && !_failed_while_armed[other] && _failed_while_armed[instance]) {
		return false;
	}

	// One that is clearly more available is better whatever its rank, as something is wrong with the other one
	if (instance_rank >= RANK_USABLE
	    && _availability[instance].getState() > _availability[other].getState() + AVAILABILITY_MARGIN) {
		return true;
	}

	if (instance_rank != other_rank) {
		return instance_rank > other_rank;
	}

	// The preferred receiver is better only while it meets the requirements: when neither does, moving to it is a
	// reset for nothing
	if (hasPreferred() && (instance_rank >= RANK_REQUIREMENTS)
	    && ((instance == _preferred_instance) || (other == _preferred_instance))) {
		return instance == _preferred_instance;
	}

	return _availability[instance].getState() > _availability[other].getState() + tiebreaker_availability_margin;
}

hrt_abstime GnssSelector::switchHoldUs() const
{
	return _armed ? (SWITCH_HOLD_ARMED_US << math::min(_rank_switches_while_armed, SWITCH_HOLD_MAX_DOUBLINGS))
	       : SWITCH_HOLD_DISARMED_US;
}

int GnssSelector::switchTo(int instance, uint8_t reason)
{
	_switch_reason = reason;
	_switch_candidate = -1;
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

	// While disarmed the preferred receiver is used whenever it publishes, even while it fails its checks, so that the
	// vehicle doesn't take off on the other one
	if (!_armed && (preferred >= 0) && !isSilent(preferred)) {
		_switch_candidate = -1;
		return (current == preferred) ? current : switchTo(preferred, vehicle_gnss_s::SELECTION_PREFERRED);
	}

	// A receiver that stopped publishing is replaced by any one that publishes, so that samples keep coming
	if (isSilent(current)) {
		const int best = bestReplacement(current, false, hrt_now_us);

		if (best < 0) {
			_switch_candidate = -1;
			return current;
		}

		// Before the first output no selected receiver stopped publishing
		return (_output_instance >= 0) ? failOver(best, vehicle_gnss_s::SELECTION_TIMEOUT) : switchTo(best, REASON_INITIAL);
	}

	// A failed receiver is replaced by any usable one, however available that one was recently
	if (hasFailed(current, hrt_now_us)) {
		const int best = bestReplacement(current, true, hrt_now_us);

		if (best >= 0) {
			return failOver(best, vehicle_gnss_s::SELECTION_UNHEALTHY);
		}
	}

	// The best receiver replaces the selected one once it has been clearly better through the hold time, so that
	// shorter changes in either one ride through
	const int best_candidate = bestCandidate(hrt_now_us);

	if (best_candidate < 0 || best_candidate == current) {
		_switch_candidate = -1;
		return current;
	}

	// Only switch if the best candidate is significantly better
	if (!isBetterReceiver(best_candidate, current, hrt_now_us, AVAILABILITY_MARGIN)) {
		_switch_candidate = -1;
		return current;
	}

	if (best_candidate != _switch_candidate) {
		_switch_candidate = best_candidate;
		_switch_candidate_since_us = hrt_now_us;
		return current;
	}

	if (hrt_now_us < _switch_candidate_since_us + switchHoldUs()) {
		return current;
	}

	if (_armed) {
		_rank_switches_while_armed++;
	}

	if (best_candidate == preferred) {
		return switchTo(best_candidate, vehicle_gnss_s::SELECTION_PREFERRED);
	}

	uint8_t switch_reason = vehicle_gnss_s::SELECTION_RANKED;
	Rank best_rank = rank(best_candidate, hrt_now_us);

	if (_availability[best_candidate].getState() > _availability[current].getState() + AVAILABILITY_MARGIN) {
		switch_reason = vehicle_gnss_s::SELECTION_UNHEALTHY;

	} else if (best_rank == RANK_RTK_FIXED) {
		switch_reason = vehicle_gnss_s::SELECTION_RTK_FIXED;

	} else if (best_rank == RANK_REQUIREMENTS) {
		switch_reason = vehicle_gnss_s::SELECTION_REQUIREMENTS;
	}

	return switchTo(best_candidate, switch_reason);
}
