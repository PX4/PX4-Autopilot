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

class DataValidatorGroup {
public:
	DataValidatorGroup(unsigned siblings);
	virtual ~DataValidatorGroup();

	/**
	 * Put an item into the validator group.
	 *
	 * @param x		X Item to put
	 * @param y		Y Item to put
	 * @param z		Z Item to put
	 */
	void			put(unsigned index, uint64_t timestamp,
					float val[3], uint64_t error_count);

	/**
	 * Get the best data triplet of the group
	 *
	 * @return		pointer to the array of best values
	 */
	float*			get_best(uint64_t timestamp, int *index);

	/**
	 * Print the validator value
	 *
	 */
	void			print();

private:
	DataValidator *_first;		/**< sibling in the group */

	/* we don't want this class to be copied */
	DataValidatorGroup(const DataValidatorGroup&);
	DataValidatorGroup operator=(const DataValidatorGroup&);
};

DataValidatorGroup::DataValidatorGroup(unsigned siblings) :
	_first(nullptr)
{
	DataValidator *next = _first;

	for (unsigned i = 0; i < siblings; i++) {
		next = new DataValidator(next);
	}
}

DataValidatorGroup::~DataValidatorGroup()
{

}

void
DataValidatorGroup::put(unsigned index, uint64_t timestamp, float val[3], uint64_t error_count)
{
	DataValidator *next = _first;
	unsigned i = 0;

	while (next != nullptr) {
		if (i == index) {
			next->put(timestamp, val, error_count);
			break;
		}
		next = next->sibling();
		i++;
	}
}

float*
DataValidatorGroup::get_best(uint64_t timestamp, int *index)
{
	DataValidator *next = _first;

	// XXX This should eventually also include voting
	float max_confidence = 0.0f;
	int max_index = -1;
	uint64_t min_error_count = 30000;
	DataValidator *best = nullptr;

	unsigned i = 0;

	while (next != nullptr) {
		float confidence = next->confidence(timestamp);
		if (confidence > max_confidence &&
			next->error_count() < min_error_count) {
			max_index = i;
			max_confidence = confidence;
			min_error_count = next->error_count();
			best = next;
		}
		next = next->sibling();
		i++;
	}

	*index = max_index;
	return (best) ? best->value() : nullptr;
}

void
DataValidatorGroup::print()
{
	DataValidator *next = _first;

	while (next != nullptr) {
		next->print();
		next = next->sibling();
	}
}
