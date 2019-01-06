/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file data_validator_group.cpp
 *
 * A data validation group to identify anomalies in data streams
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "data_validator_group.h"
#include <ecl.h>
#include <float.h>

DataValidatorGroup::DataValidatorGroup(unsigned siblings)
{
	DataValidator *next = nullptr;
	DataValidator *prev = nullptr;

	for (unsigned i = 0; i < siblings; i++) {
		next = new DataValidator();

		if (i == 0) {
			_first = next;

		} else {
			prev->setSibling(next);
		}

		prev = next;
	}

	_last = next;

	if (_first) {
		_timeout_interval_us = _first->get_timeout();
	}
}

DataValidatorGroup::~DataValidatorGroup()
{
	while (_first) {
		DataValidator *next = _first->sibling();
		delete (_first);
		_first = next;
	}
}

DataValidator *DataValidatorGroup::add_new_validator()
{
	DataValidator *validator = new DataValidator();

	if (!validator) {
		return nullptr;
	}

	_last->setSibling(validator);
	_last = validator;
	_last->set_timeout(_timeout_interval_us);
	return _last;
}

void
DataValidatorGroup::set_timeout(uint32_t timeout_interval_us)
{
	DataValidator *next = _first;

	while (next != nullptr) {
		next->set_timeout(timeout_interval_us);
		next = next->sibling();
	}

	_timeout_interval_us = timeout_interval_us;
}

void
DataValidatorGroup::set_equal_value_threshold(uint32_t threshold)
{
	DataValidator *next = _first;

	while (next != nullptr) {
		next->set_equal_value_threshold(threshold);
		next = next->sibling();
	}
}


void
DataValidatorGroup::put(unsigned index, uint64_t timestamp, const float val[3], uint64_t error_count, int priority)
{
	DataValidator *next = _first;
	unsigned i = 0;

	while (next != nullptr) {
		if (i == index) {
			next->put(timestamp, val, error_count, priority);
			break;
		}

		next = next->sibling();
		i++;
	}
}

float *
DataValidatorGroup::get_best(uint64_t timestamp, int *index)
{
	DataValidator *next = _first;

	// XXX This should eventually also include voting
	int pre_check_best = _curr_best;
	float pre_check_confidence = 1.0f;
	int pre_check_prio = -1;
	float max_confidence = -1.0f;
	int max_priority = -1000;
	int max_index = -1;
	DataValidator *best = nullptr;

	int i = 0;

	while (next != nullptr) {
		float confidence = next->confidence(timestamp);

		if (i == pre_check_best) {
			pre_check_prio = next->priority();
			pre_check_confidence = confidence;
		}

		/*
		 * Switch if:
		 * 1) the confidence is higher and priority is equal or higher
		 * 2) the confidence is no less than 1% different and the priority is higher
		 */
		if ((((max_confidence < MIN_REGULAR_CONFIDENCE) && (confidence >= MIN_REGULAR_CONFIDENCE)) ||
		     (confidence > max_confidence && (next->priority() >= max_priority)) ||
		     (fabsf(confidence - max_confidence) < 0.01f && (next->priority() > max_priority))
		    ) && (confidence > 0.0f)) {

			max_index = i;
			max_confidence = confidence;
			max_priority = next->priority();
			best = next;
		}

		next = next->sibling();
		i++;
	}

	/* the current best sensor is not matching the previous best sensor,
	 * or the only sensor went bad */
	if (max_index != _curr_best || ((max_confidence < FLT_EPSILON) && (_curr_best >= 0))) {

		bool true_failsafe = true;

		/* check whether the switch was a failsafe or preferring a higher priority sensor */
		if (pre_check_prio != -1 && pre_check_prio < max_priority &&
		    fabsf(pre_check_confidence - max_confidence) < 0.1f) {

			/* this is not a failover */
			true_failsafe = false;

			/* reset error flags, this is likely a hotplug sensor coming online late */
			if (best != nullptr) {
				best->reset_state();
			}
		}

		/* if we're no initialized, initialize the bookkeeping but do not count a failsafe */
		if (_curr_best < 0) {
			_prev_best = max_index;

		} else {
			/* we were initialized before, this is a real failsafe */
			_prev_best = pre_check_best;

			if (true_failsafe) {
				_toggle_count++;

				/* if this is the first time, log when we failed */
				if (_first_failover_time == 0) {
					_first_failover_time = timestamp;
				}
			}
		}

		/* for all cases we want to keep a record of the best index */
		_curr_best = max_index;
	}

	*index = max_index;
	return (best) ? best->value() : nullptr;
}

float
DataValidatorGroup::get_vibration_factor(uint64_t timestamp)
{
	DataValidator *next = _first;

	float vibe = 0.0f;

	/* find the best RMS value of a non-timed out sensor */
	while (next != nullptr) {

		if (next->confidence(timestamp) > 0.5f) {
			float *rms = next->rms();

			for (unsigned j = 0; j < 3; j++) {
				if (rms[j] > vibe) {
					vibe = rms[j];
				}
			}
		}

		next = next->sibling();
	}

	return vibe;
}

float
DataValidatorGroup::get_vibration_offset(uint64_t timestamp, int axis)
{
	DataValidator *next = _first;

	float vibe = -1.0f;

	/* find the best vibration value of a non-timed out sensor */
	while (next != nullptr) {

		if (next->confidence(timestamp) > 0.5f) {
			float *vibration_offset = next->vibration_offset();

			if (vibe < 0.0f || vibration_offset[axis] < vibe) {
				vibe = vibration_offset[axis];
			}
		}

		next = next->sibling();
	}

	return vibe;
}

void
DataValidatorGroup::print()
{
	/* print the group's state */
	ECL_INFO("validator: best: %d, prev best: %d, failsafe: %s (%u events)",
		 _curr_best, _prev_best, (_toggle_count > 0) ? "YES" : "NO",
		 _toggle_count);

	DataValidator *next = _first;
	unsigned i = 0;

	while (next != nullptr) {
		if (next->used()) {
			uint32_t flags = next->state();

			ECL_INFO("sensor #%u, prio: %d, state:%s%s%s%s%s%s", i, next->priority(),
				 ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
				 ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
				 ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TOUT" : ""),
				 ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ECNT" : ""),
				 ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " EDNST" : ""),
				 ((flags == DataValidator::ERROR_FLAG_NO_ERROR) ? " OK" : ""));

			next->print();
		}

		next = next->sibling();
		i++;
	}
}

int
DataValidatorGroup::failover_index()
{
	DataValidator *next = _first;
	unsigned i = 0;

	while (next != nullptr) {
		if (next->used() && (next->state() != DataValidator::ERROR_FLAG_NO_ERROR) && (i == (unsigned)_prev_best)) {
			return i;
		}

		next = next->sibling();
		i++;
	}

	return -1;
}

uint32_t
DataValidatorGroup::failover_state()
{
	DataValidator *next = _first;
	unsigned i = 0;

	while (next != nullptr) {
		if (next->used() && (next->state() != DataValidator::ERROR_FLAG_NO_ERROR) && (i == (unsigned)_prev_best)) {
			return next->state();
		}

		next = next->sibling();
		i++;
	}

	return DataValidator::ERROR_FLAG_NO_ERROR;
}
