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
 * @file data_validator.c
 *
 * A data validation class to identify anomalies in data streams
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "data_validator.h"
#include <ecl/ecl.h>

DataValidator::DataValidator(DataValidator *prev_sibling) :
	_error_mask(ERROR_FLAG_NO_ERROR),
	_time_last(0),
	_timeout_interval(20000),
	_event_count(0),
	_error_count(0),
	_error_density(0),
	_priority(0),
	_mean{0.0f},
	_lp{0.0f},
	_M2{0.0f},
	_rms{0.0f},
	_value{0.0f},
	_value_equal_count(0),
	_sibling(prev_sibling)
{

}

DataValidator::~DataValidator()
{

}

void
DataValidator::put(uint64_t timestamp, float val, uint64_t error_count_in, int priority_in)
{
	float data[3];

	data[0] = val;
	data[1] = 0.0f;
	data[2] = 0.0f;

	put(timestamp, data, error_count_in, priority_in);
}

void
DataValidator::put(uint64_t timestamp, float val[3], uint64_t error_count_in, int priority_in)
{
	_event_count++;

	if (error_count_in > _error_count) {
		_error_density += (error_count_in - _error_count);
	} else if (_error_density > 0) {
		_error_density--;
	}

	_error_count = error_count_in;
	_priority = priority_in;

	for (unsigned i = 0; i < _dimensions; i++) {
		if (_time_last == 0) {
			_mean[i] = 0;
			_lp[i] = val[i];
			_M2[i] = 0;
		} else {
			float lp_val = val[i] - _lp[i];

			float delta_val = lp_val - _mean[i];
			_mean[i] += delta_val / _event_count;
			_M2[i] += delta_val * (lp_val - _mean[i]);
			_rms[i] = sqrtf(_M2[i] / (_event_count - 1));

			if (fabsf(_value[i] - val[i]) < 0.000001f) {
				_value_equal_count++;
			} else {
				_value_equal_count = 0;
			}
		}

		// XXX replace with better filter, make it auto-tune to update rate
		_lp[i] = _lp[i] * 0.5f + val[i] * 0.5f;

		_value[i] = val[i];
	}

	_time_last = timestamp;
}

float
DataValidator::confidence(uint64_t timestamp)
{
	float ret = 1.0f;
	
	/* check if we have any data */
	if (_time_last == 0) {
		_error_mask |= ERROR_FLAG_NO_DATA;
		ret = 0.0f;

	/* timed out - that's it */
	} else if (timestamp - _time_last > _timeout_interval) {
		_error_mask |= ERROR_FLAG_TIMEOUT;
		ret = 0.0f;

	/* we got the exact same sensor value N times in a row */
	} else if (_value_equal_count > VALUE_EQUAL_COUNT_MAX) {
		_error_mask |= ERROR_FLAG_STALE_DATA;
		ret = 0.0f;

	/* check error count limit */
	} else if (_error_count > NORETURN_ERRCOUNT) {
		_error_mask |= ERROR_FLAG_HIGH_ERRCOUNT;
		ret = 0.0f;

	/* cap error density counter at window size */
	} else if (_error_density > ERROR_DENSITY_WINDOW) {
		_error_mask |= ERROR_FLAG_HIGH_ERRDENSITY;
		_error_density = ERROR_DENSITY_WINDOW;

	/* no error */
	} else {
		_error_mask = ERROR_FLAG_NO_ERROR;
	}
	
	/* no critical errors */
	if (ret > 0.0f) {
		/* return local error density for last N measurements */
		ret = 1.0f - (_error_density / ERROR_DENSITY_WINDOW);
	}
	
	return ret;
}

void
DataValidator::print()
{
	if (_time_last == 0) {
		ECL_INFO("\tno data");
		return;
	}

	for (unsigned i = 0; i < _dimensions; i++) {
		ECL_INFO("\tval: %8.4f, lp: %8.4f mean dev: %8.4f RMS: %8.4f conf: %8.4f",
			(double) _value[i], (double)_lp[i], (double)_mean[i],
			(double)_rms[i], (double)confidence(hrt_absolute_time()));
	}
}
