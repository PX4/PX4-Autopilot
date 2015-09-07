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
 * @file data_validator_group.h
 *
 * A data validation group to identify anomalies in data streams
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include "data_validator.h"

class __EXPORT DataValidatorGroup {
public:
	DataValidatorGroup(unsigned siblings);
	virtual ~DataValidatorGroup();

	/**
	 * Put an item into the validator group.
	 *
	 * @param index		Sensor index
	 * @param timestamp	The timestamp of the measurement
	 * @param val		The 3D vector
	 * @param error_count	The current error count of the sensor
	 * @param priority	The priority of the sensor
	 */
	void			put(unsigned index, uint64_t timestamp,
					float val[3], uint64_t error_count, int priority);

	/**
	 * Get the best data triplet of the group
	 *
	 * @return		pointer to the array of best values
	 */
	float*			get_best(uint64_t timestamp, int *index);

	/**
	 * Get the RMS / vibration factor
	 *
	 * @return		float value representing the RMS, which a valid indicator for vibration
	 */
	float			get_vibration_factor(uint64_t timestamp);

	/**
	 * Get the number of failover events
	 *
	 * @return		the number of failovers
	 */
	unsigned		failover_count();

	/**
	 * Print the validator value
	 *
	 */
	void			print();

	/**
	 * Set the timeout value
	 *
	 * @param timeout_interval_us The timeout interval in microseconds
	 */
	void			set_timeout(uint64_t timeout_interval_us);

private:
	DataValidator *_first;		/**< sibling in the group */
	int _curr_best;		/**< currently best index */
	int _prev_best;		/**< the previous best index */
	uint64_t _first_failover_time;	/**< timestamp where the first failover occured or zero if none occured */
	unsigned _toggle_count;		/**< number of back and forth switches between two sensors */

	/* we don't want this class to be copied */
	DataValidatorGroup(const DataValidatorGroup&);
	DataValidatorGroup operator=(const DataValidatorGroup&);
};
