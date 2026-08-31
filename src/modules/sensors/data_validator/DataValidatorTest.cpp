/****************************************************************************
 *
 *   Copyright (c) 2019-2026 PX4 Development Team. All rights reserved.
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

#include "DataValidator.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{

// derived from the class-private VALUE_EQUAL_COUNT_DEFAULT / timeout defaults
constexpr uint32_t kTimeoutUsec = 2000;
constexpr uint64_t kTimestampIncrUsec = 5;
constexpr uint8_t kPriority = 50;

// smallest per-sample change that still avoids stale detection
constexpr float kSufficientIncrement = 1.1e-6f;

// deterministic stand-in for rand(), so a failure is always reproducible
class Lcg
{
public:
	float next_unit() { _state = _state * 1103515245u + 12345u; return (float)((_state >> 16) & 0x7fff) / 32767.f; }
private:
	uint32_t _state{666};
};

/**
 * Insert a run of samples that all differ, so the stale detector stays quiet.
 * value_io / timestamp_io carry the running value and timestamp in and out.
 */
void fill_with_samples(DataValidator &validator, float increment, float *value_io, uint64_t *timestamp_io)
{
	constexpr int equal_value_count = 100;

	validator.set_equal_value_threshold(equal_value_count);
	validator.set_timeout(kTimeoutUsec);

	for (int i = 0; i < equal_value_count; i++, *value_io += increment) {
		*timestamp_io += kTimestampIncrUsec;
		validator.put(*timestamp_io, *value_io, 0, kPriority);
	}
}

/**
 * Insert count samples alternating +/- swing around mean, and return the rms error they imply.
 */
float insert_values_around_mean(DataValidator &validator, float mean, uint32_t count, uint64_t *timestamp_io)
{
	constexpr float swing = 1e-2f;
	double sum_dev_squares = 0.0;

	for (uint32_t i = 0; i < count; i++) {
		const float value = mean + ((i % 2) == 0 ? swing : -swing);
		const double deviation = static_cast<double>(value) - static_cast<double>(mean);
		sum_dev_squares += deviation * deviation;
		*timestamp_io += kTimestampIncrUsec;
		validator.put(*timestamp_io, value, 0, kPriority);
	}

	return (float)sqrt(sum_dev_squares / (double)count);
}

} // namespace

TEST(DataValidator, initialisesEmpty)
{
	const uint64_t timestamp = 666;
	DataValidator validator;

	EXPECT_EQ(nullptr, validator.sibling());
	EXPECT_FLOAT_EQ(0.f, validator.confidence(timestamp));
	EXPECT_EQ(0u, validator.error_count());
	EXPECT_FALSE(validator.used());
	EXPECT_EQ(0, validator.priority());

	validator.set_timeout(kTimeoutUsec);
	EXPECT_EQ(kTimeoutUsec, validator.get_timeout());

	DataValidator sibling;
	validator.setSibling(&sibling);
	EXPECT_EQ(&sibling, validator.sibling());

	// with no data at all the validator has no confidence and says why
	EXPECT_FLOAT_EQ(0.f, validator.confidence(timestamp + 1));
	EXPECT_TRUE(validator.state() & DataValidator::ERROR_FLAG_NO_DATA);

	validator.print();
}

TEST(DataValidator, fullyConfidentAfterValidData)
{
	uint64_t timestamp = 500;
	float value = 3.14159f;

	DataValidator validator;
	fill_with_samples(validator, kSufficientIncrement, &value, &timestamp);

	EXPECT_TRUE(validator.used());
	EXPECT_FLOAT_EQ(value - kSufficientIncrement, validator.value()[0]);
	EXPECT_FLOAT_EQ(1.f, validator.confidence(timestamp));
	EXPECT_EQ(DataValidator::ERROR_FLAG_NO_ERROR, validator.state());

	// past the timeout window the same data is worthless
	EXPECT_FLOAT_EQ(0.f, validator.confidence(timestamp + (uint64_t)(1.1f * kTimeoutUsec)));
	EXPECT_TRUE(validator.state() & DataValidator::ERROR_FLAG_TIMEOUT);
}

TEST(DataValidator, detectsStaleData)
{
	uint64_t timestamp = 500;
	float value = 3.14159f;

	// too small a change per sample to count as new data
	DataValidator validator;
	fill_with_samples(validator, 0.99e-6f, &value, &timestamp);

	EXPECT_FLOAT_EQ(0.f, validator.confidence(timestamp));
	EXPECT_TRUE(validator.state() & DataValidator::ERROR_FLAG_STALE_DATA);
}

TEST(DataValidator, tracksRmsError)
{
	uint64_t timestamp = 500;

	DataValidator validator;
	validator.set_equal_value_threshold(100);

	const float expected_rms = insert_values_around_mean(validator, 3.14159f, 1000, &timestamp);

	ASSERT_NE(nullptr, validator.rms());
	EXPECT_NEAR(expected_rms, validator.rms()[0], 0.03f * expected_rms);
}

TEST(DataValidator, errorDensityReducesConfidence)
{
	// stay under the private NORETURN_ERRCOUNT for this stage
	constexpr int total_iterations = 1000;
	constexpr float error_density_window = 100.f;

	uint64_t timestamp = 500;
	float value = 3.14159f;
	uint32_t error_count = 0;
	int expected_error_density = 0;
	Lcg rng;

	DataValidator validator;
	validator.set_timeout(kTimeoutUsec);
	validator.set_equal_value_threshold(50000);

	for (int i = 0; i < total_iterations; i++, value += kSufficientIncrement) {
		timestamp += kTimestampIncrUsec;

		// up to a 50% error rate still passes the density filter
		if (rng.next_unit() < 0.5f) {
			error_count++;
			expected_error_density++;

		} else if (expected_error_density > 0) {
			expected_error_density--;
		}

		validator.put(timestamp, value, error_count, kPriority);
	}

	EXPECT_TRUE(validator.used());
	EXPECT_EQ(error_count, validator.error_count());

	// degraded but not failed: confidence tracks the error density exactly
	const float confidence = validator.confidence(timestamp);
	EXPECT_GT(confidence, 0.f);
	EXPECT_LT(confidence, 1.f);
	EXPECT_NEAR(1.f - (expected_error_density / error_density_window), confidence, 1e-6f);
	EXPECT_EQ(DataValidator::ERROR_FLAG_NO_ERROR, validator.state());
}

TEST(DataValidator, flagsHighErrorDensityThenHighErrorCount)
{
	uint64_t timestamp = 500;
	float value = 3.14159f;
	uint32_t error_count = 0;

	DataValidator validator;
	validator.set_timeout(kTimeoutUsec);
	validator.set_equal_value_threshold(50000);

	for (int i = 0; i < 250; i++, value += kSufficientIncrement) {
		timestamp += kTimestampIncrUsec;
		validator.put(timestamp, value, ++error_count, kPriority);
	}

	EXPECT_FLOAT_EQ(0.f, validator.confidence(timestamp));
	EXPECT_TRUE(validator.state() & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY);

	validator.reset_state();

	// enough errors to exceed the private NORETURN_ERRCOUNT
	for (int i = 0; i < 10000; i++, value += kSufficientIncrement) {
		timestamp += kTimestampIncrUsec;
		validator.put(timestamp, value, ++error_count, kPriority);
	}

	EXPECT_FLOAT_EQ(0.f, validator.confidence(timestamp));
	EXPECT_TRUE(validator.state() & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT);
}
