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

class DataValidator;

class DataValidator {
public:
	DataValidator(DataValidator *sibling);
	virtual ~DataValidator();

	/**
	 * Put an item into the validator.
	 *
	 * @param val		Item to put
	 */
	void			put(uint64_t timestamp, float val);

	/**
	 * Get the next sibling in the group
	 *
	 * @return		the next sibling
	 */
	float			sibling() { return _sibling; }

	/**
	 * Get the confidence of this validator
	 * @return		the confidence between 0 and 1
	 */
	float			confidence();

	/**
	 * Get the value of this validator
	 * @return		the stored value
	 */
	float			value() { return _value; }

	/**
	 * Print the validator value
	 *
	 */
	float			print(bool auto_reset);

private:
	uint64_t _time_last;			/**< last timestamp */
	uint64_t _timeout_interval;		/**< interval in which the datastream times out in us */
	uint64_t _event_count;			/**< total data counter */
	float _mean;				/**< mean of value */
	float _lp;				/**< low pass value */
	float _M2;				/**< RMS component value */
	float _rms;				/**< root mean square error */
	float _value;				/**< last value */
	DataValidator *_sibling;		/**< sibling in the group */

	/* we don't want this class to be copied */
	DataValidator(const DataValidator&);
	DataValidator operator=(const DataValidator&);
};

DataValidator(sibling = nullptr) :
	_time_last(0),
	_timeout_interval(50000),
	_event_count(0),
	_mean(0.0f),
	_lp(0.0f),
	_M2(0.0f),
	_rms(0.0f),
	_value(0.0f),
	_sibling(sibling)
{

}

~DataValidator()
{

}

void
DataValidator::put(uint64_t timestamp, float val)
{
	_event_count++;

	if (_time_last == 0) {
		_mean = val;
		_lp = val;
		_M2 = 0;
	} else {
		float lp_val = val - _lp;

		float delta_val = lp_val - _mean;
		_mean += delta_val / _event_count;
		_M2 += delta_val * (lp_val - _mean);
		_rms = sqrtf(_M2 / (_event_count - 1));
	}

	// XXX replace with better filter, make it auto-tune to update rate
	_lp = _lp * 0.95f + val * 0.05f;

	_time_last = timestamp;
	_value = val;
}

void
DataValidator::confidence(uint64_t timestamp)
{
	/* timed out - that's it */
	if (timestamp - _time_last > _timeout_interval) {
		return 0.0f;
	}

	// XXX work out scaling between confidence and RMS
	return _rms / _mean;
}

float
DataValidator::print()
{
	printf("\tmean: %8.4f lp: %8.4f RMS: %8.4f\n", _lp + _mean, _lp, _rms);
}
