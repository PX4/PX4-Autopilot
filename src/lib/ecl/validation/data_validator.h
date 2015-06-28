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
 * @file data_validator.h
 *
 * A data validation class to identify anomalies in data streams
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

class DataValidator {
public:
	DataValidator(DataValidator *prev_sibling = nullptr);
	virtual ~DataValidator();

	/**
	 * Put an item into the validator.
	 *
	 * @param val		Item to put
	 */
	void			put(uint64_t timestamp, float val[3], uint64_t error_count);

	/**
	 * Get the next sibling in the group
	 *
	 * @return		the next sibling
	 */
	DataValidator*		sibling() { return _sibling; }

	/**
	 * Get the confidence of this validator
	 * @return		the confidence between 0 and 1
	 */
	float			confidence(uint64_t timestamp);

	/**
	 * Get the error count of this validator
	 * @return		the error count
	 */
	uint64_t		error_count() { return _error_count; }

	/**
	 * Get the values of this validator
	 * @return		the stored value
	 */
	float*			value() { return _value; }

	/**
	 * Get the RMS values of this validator
	 * @return		the stored RMS
	 */
	float*			rms() { return _rms; }

	/**
	 * Print the validator value
	 *
	 */
	void			print();

private:
	static const unsigned _dimensions = 3;
	uint64_t _time_last;			/**< last timestamp */
	uint64_t _timeout_interval;		/**< interval in which the datastream times out in us */
	uint64_t _event_count;			/**< total data counter */
	uint64_t _error_count;			/**< error count */
	float _mean[_dimensions];				/**< mean of value */
	float _lp[3];				/**< low pass value */
	float _M2[3];				/**< RMS component value */
	float _rms[3];				/**< root mean square error */
	float _value[3];			/**< last value */
	float _value_equal_count;		/**< equal values in a row */
	DataValidator *_sibling;		/**< sibling in the group */
	const unsigned NORETURN_ERRCOUNT = 1000;	/**< if the error count reaches this value, return sensor as invalid */
	const unsigned VALUE_EQUAL_COUNT_MAX = 100;	/**< if the sensor value is the same (accumulated also between axes) this many times, flag it */

	/* we don't want this class to be copied */
	DataValidator(const DataValidator&);
	DataValidator operator=(const DataValidator&);
};

DataValidator::DataValidator(DataValidator *prev_sibling) :
	_time_last(0),
	_timeout_interval(50000),
	_event_count(0),
	_error_count(0),
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
DataValidator::put(uint64_t timestamp, float val[3], uint64_t error_count_in)
{
	_event_count++;
	_error_count = error_count_in;

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
	/* check if we have any data */
	if (_time_last == 0) {
		return 0.0f;
	}

	/* check error count limit */
	if (_error_count > NORETURN_ERRCOUNT) {
		return 0.0f;
	}

	/* we got the exact same sensor value N times in a row */
	if (_value_equal_count > VALUE_EQUAL_COUNT_MAX) {
		return 0.0f;
	}

	/* timed out - that's it */
	if (timestamp - _time_last > _timeout_interval) {
		return 0.0f;
	}

	return 1.0f;
}

void
DataValidator::print()
{
	if (_time_last == 0) {
		printf("\tno data\n");
		return;
	}

	for (unsigned i = 0; i < _dimensions; i++) {
		printf("\tval: %8.4f, lp: %8.4f mean dev: %8.4f RMS: %8.4f\n",
			(double) _value[i], (double)_lp[i], (double)_mean[i], (double)_rms[i]);
	}
}
