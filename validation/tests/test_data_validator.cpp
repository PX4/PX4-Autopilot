/****************************************************************************
 *
 *   Copyright (c) 2019 Todd Stellanova. All rights reserved.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be  used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 * @file test_data_validator.cpp
 * Testing the DataValidator class
 *
 * @author Todd Stellanova
 */

#include <stdint.h>
#include <cassert>
//#include <stdio.h>
#include <math.h>
#include "../data_validator.h"

//void dump_validator_state(DataValidator* validator)
//{
//  uint32_t state = validator->state();
//  printf("state: 0x%x no_data: %d stale: %d timeout:%d\n",
//         validator->state(),
//         DataValidator::ERROR_FLAG_NO_DATA & state,
//         DataValidator::ERROR_FLAG_STALE_DATA & state,
//         DataValidator::ERROR_FLAG_TIMEOUT & state
//  );
//  validator->print();
//}


/**
 * Insert a series of samples around a mean value
 * @param validator The validator under test
 * @param mean The mean value
 * @param count The number of samples to insert in the validator
 * @param rms_err (out) calculated rms error of the inserted samples
 */
void insert_values_around_mean(DataValidator *validator, const float mean, uint32_t count, float *rms_err)
{
	uint64_t timestamp = 500;
	uint64_t timestamp_incr = 5;
	const uint64_t error_count = 0;
	const int priority = 50;
	const float swing = 1E-2f;
	double sum_dev_squares = 0.0f;

	//insert a series of values that swing around the mean
	for (uint32_t i = 0; i < count;  i++) {
		float iter_swing = (0 == (i % 2)) ? swing : -swing;
		float iter_val = mean + iter_swing;
		float iter_dev = iter_val - mean;
		sum_dev_squares += (iter_dev * iter_dev);
		timestamp += timestamp_incr;
		validator->put(timestamp, iter_val, error_count, priority);
	}

	double rms = sqrt(sum_dev_squares / (double)count);
	//note: this should be approximately equal to "swing"
	*rms_err = (float)rms;
}

void test_init()
{
	uint64_t fake_timestamp = 666;

	DataValidator *validator = new DataValidator;
	// initially there should be no siblings
	assert(nullptr == validator->sibling());
	// initially we should have zero confidence
	assert(0.0f == validator->confidence(fake_timestamp));
	// initially the error count should be zero
	assert(0 == validator->error_count());
	// initially unused
	assert(!validator->used());
	// initially no priority
	assert(0 == validator->priority());

	DataValidator *sibling_validator = new DataValidator;
	validator->setSibling(sibling_validator);
	assert(sibling_validator == validator->sibling());

	//verify that with no data, confidence is zero and error mask is set
	assert(0.0f == validator->confidence(fake_timestamp + 1));
	uint32_t state = validator->state();
	assert(DataValidator::ERROR_FLAG_NO_DATA == (DataValidator::ERROR_FLAG_NO_DATA & state));

}

void test_put()
{
	uint64_t timestamp = 500;
	uint64_t timestamp_incr = 5;
	const uint32_t timeout_usec = 2000;//from original private value
	float val = 3.14159f;
	uint64_t error_count = 0;
	int priority = 50;
	//from private value: this is min change needed to avoid stale detection
	const float sufficient_incr_value = (1.1f * 1E-6f);
	const int equal_value_count = 100; //default is private VALUE_EQUAL_COUNT_DEFAULT

	DataValidator *validator = new DataValidator;
	validator->set_timeout(timeout_usec);
	validator->set_equal_value_threshold(equal_value_count);

	//put a bunch of values that are all different
	for (int i = 0; i < equal_value_count;  i++, val +=  sufficient_incr_value) {
		timestamp += timestamp_incr;
		validator->put(timestamp, val, error_count, priority);
	}

	assert(validator->used());

	// we've just provided a bunch of valid data: should be fully confident
	float conf = validator->confidence(timestamp);
//  if (1.0f != conf) {
//    printf("conf: %f\n",(double)conf);
//    dump_validator_state(validator);
//  }
	assert(1.0f == conf);
	// should be no errors
	assert(0 == validator->state());

	//now check confidence much beyond the timeout window-- should timeout
	conf = validator->confidence(timestamp + (1.1 * timeout_usec));
//  if (0.0f != conf) {
//    printf("conf: %f\n",(double)conf);
//    dump_validator_state(validator);
//  }
	assert(0.0f == conf);
	assert(DataValidator::ERROR_FLAG_TIMEOUT == (DataValidator::ERROR_FLAG_TIMEOUT & validator->state()));

}

/**
 * Verify that the DataValidator detects sensor data that does not vary sufficiently
 */
void test_stale_detector()
{
	uint64_t timestamp = 500;
	uint64_t timestamp_incr = 5;
	float val = 3.14159f;
	uint64_t error_count = 0;
	int priority = 50;
	const float insufficient_incr_value = (0.99 * 1E-6f);//insufficient to avoid stale detection
	const int equal_value_count = 100; //default is private VALUE_EQUAL_COUNT_DEFAULT

	DataValidator *validator = new DataValidator;
	validator->set_equal_value_threshold(equal_value_count);

	//put a bunch of values that are all different
	for (int i = 0; i < equal_value_count; i++, val += insufficient_incr_value) {
		timestamp += timestamp_incr;
		validator->put(timestamp, val, error_count, priority);
	}

	// data is stale: should have no confidence
	assert(0.0f == validator->confidence(timestamp));

	// should be a stale error
	uint32_t state = validator->state();
//  if (DataValidator::ERROR_FLAG_STALE_DATA != state) {
//    dump_validator_state(validator);
//  }
	assert(DataValidator::ERROR_FLAG_STALE_DATA == (DataValidator::ERROR_FLAG_STALE_DATA & state));

}

/**
 * Verify the RMS error calculated by the DataValidator for a series of samples
 */
void test_rms_calculation()
{
	const int equal_value_count = 100; //default is private VALUE_EQUAL_COUNT_DEFAULT
	const float mean_value = 3.14159f;
	const uint32_t sample_count = 1000;
	float expected_rms_err = 0.0f;

	DataValidator *validator = new DataValidator;
	validator->set_equal_value_threshold(equal_value_count);

	insert_values_around_mean(validator, mean_value, sample_count, &expected_rms_err);
	float *rms = validator->rms();
	assert(nullptr != rms);
	float calc_rms_err = rms[0];
	float diff = fabsf(calc_rms_err - expected_rms_err);
	float diff_frac = (diff / expected_rms_err);
//  printf("rms: %f expect: %f diff: %f frac: %f\n", (double)calc_rms_err, (double)expected_rms_err,
//      (double)diff, (double)diff_frac);
	assert(diff_frac < 0.03f);
}

int main(int argc, char *argv[])
{
	(void)argc; // unused
	(void)argv; // unused

	test_init();
	test_put();
	test_stale_detector();
	test_rms_calculation();

	return 0; //passed
}
