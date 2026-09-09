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

#include "DataValidatorGroup.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{

// derived from the class-private defaults
constexpr uint32_t kTimeoutUsec = 2000;
constexpr int kEqualValueCount = 1000;
constexpr uint64_t kBaseTimestamp = 666;
constexpr unsigned kBaseSiblings = 4;

// deterministic stand-in for rand(), so a failure is always reproducible
class Lcg
{
public:
	float next_unit() { _state = _state * 1103515245u + 12345u; return (float)((_state >> 16) & 0x7fff) / 32767.f; }
private:
	uint32_t _state{666};
};

class DataValidatorGroupTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		_group = new DataValidatorGroup(kBaseSiblings);
		ASSERT_NE(nullptr, _group);
		_group->set_timeout(kTimeoutUsec);
		_group->set_equal_value_threshold(kEqualValueCount);
		_siblings = kBaseSiblings;
	}

	void TearDown() override { delete _group; }

	/** Add a validator to the group and return its index. */
	int add_validator(DataValidator **handle = nullptr)
	{
		DataValidator *validator = _group->add_new_validator();

		// the group's settings must reach members added after the fact
		EXPECT_EQ(kTimeoutUsec, validator->get_timeout());
		EXPECT_EQ(kEqualValueCount, validator->get_equal_value_threshold());

		if (handle != nullptr) {
			*handle = validator;
		}

		return (int)_siblings++;
	}

	void put(int index, uint64_t timestamp, float value, uint8_t priority, uint32_t error_count = 0)
	{
		const float data[DataValidator::dimensions] {value, 0.f, 0.f};
		_group->put(index, timestamp, data, error_count, priority);
	}

	DataValidatorGroup *_group{nullptr};
	unsigned _siblings{0};
};

} // namespace

TEST_F(DataValidatorGroupTest, initialisesWithNothingSelected)
{
	// verify that calling print doesn't crash
	_group->print();

	EXPECT_EQ(0u, _group->failover_count());
	EXPECT_EQ(DataValidator::ERROR_FLAG_NO_ERROR, _group->failover_state());
	EXPECT_EQ(-1, _group->failover_index());

	int best_index = -1;
	EXPECT_EQ(nullptr, _group->get_best(kBaseTimestamp, &best_index));
}

TEST_F(DataValidatorGroupTest, selectsTheHighestPrioritySensor)
{
	DataValidator *validator1 = nullptr;
	DataValidator *validator2 = nullptr;
	const int idx1 = add_validator(&validator1);
	const int idx2 = add_validator(&validator2);

	Lcg rng;
	float last_value = 0.f;

	for (int i = 0; i < 500; i++) {
		last_value = rng.next_unit();
		put(idx1, kBaseTimestamp, last_value, 100);
		put(idx2, kBaseTimestamp, last_value, 10);
	}

	int best_index = -1;
	float *best = _group->get_best(kBaseTimestamp, &best_index);
	ASSERT_NE(nullptr, best);
	EXPECT_EQ(idx1, best_index);
	EXPECT_FLOAT_EQ(last_value, best[0]);
	EXPECT_FLOAT_EQ(last_value, validator1->value()[0]);
	EXPECT_FLOAT_EQ(last_value, validator2->value()[0]);
}

TEST_F(DataValidatorGroupTest, raisingPriorityIsNotAFailover)
{
	const int idx1 = add_validator();
	const int idx2 = add_validator();

	Lcg rng;

	for (int i = 0; i < 100; i++) {
		const float value = rng.next_unit();
		put(idx1, kBaseTimestamp, value, 100);
		put(idx2, kBaseTimestamp, value, 10);
	}

	int established = -1;
	_group->get_best(kBaseTimestamp, &established);
	ASSERT_EQ(idx1, established);

	// a single sample at the swapped priorities is enough to move the selection
	const float new_best = 3.14159f;
	put(idx1, kBaseTimestamp, new_best, 1);
	put(idx2, kBaseTimestamp, new_best, 100);

	int best_index = -1;
	float *best = _group->get_best(kBaseTimestamp, &best_index);
	ASSERT_NE(nullptr, best);
	EXPECT_FLOAT_EQ(new_best, best[0]);
	EXPECT_EQ(idx2, best_index);
	EXPECT_EQ(0u, _group->failover_count());
}

TEST_F(DataValidatorGroupTest, failsOverToTheSensorWithoutErrors)
{
	DataValidator *validator1 = nullptr;
	const int idx1 = add_validator(&validator1);
	const int idx2 = add_validator();

	Lcg rng;

	for (int i = 0; i < 100; i++) {
		const float value = rng.next_unit();
		put(idx1, kBaseTimestamp, value, 100);
		put(idx2, kBaseTimestamp, value, 10);
	}

	int established = -1;
	_group->get_best(kBaseTimestamp, &established);
	ASSERT_EQ(idx1, established);

	// pile errors onto the selected sensor only
	const float new_best = 3.14159f;
	uint32_t errors = 0;

	for (int i = 0; i < 25; i++) {
		put(idx1, kBaseTimestamp, new_best, 100, ++errors);
		put(idx2, kBaseTimestamp, new_best, 10, 0);
	}

	EXPECT_EQ(errors, validator1->error_count());

	int best_index = -1;
	float *best = _group->get_best(kBaseTimestamp + 1, &best_index);
	ASSERT_NE(nullptr, best);
	EXPECT_FLOAT_EQ(new_best, best[0]);
	EXPECT_EQ(idx2, best_index);
	EXPECT_EQ(1u, _group->failover_count());

	// error density is up but the sensor has not hard-failed, so the group reports no failure
	EXPECT_EQ(DataValidator::ERROR_FLAG_NO_ERROR, validator1->state());
	EXPECT_EQ(-1, _group->failover_index());
	EXPECT_EQ(DataValidator::ERROR_FLAG_NO_ERROR, _group->failover_state());
}

TEST_F(DataValidatorGroupTest, reportsTheIndexOfAFailedSensor)
{
	DataValidator *validator = nullptr;
	const int idx = add_validator(&validator);

	uint64_t timestamp = kBaseTimestamp;
	float value = 3.14159f;

	for (int i = 0; i < 100; i++, value += 1.1e-6f) {
		timestamp += 5;
		put(idx, timestamp, value, 50);
	}

	int best_index = -1;
	ASSERT_NE(nullptr, _group->get_best(timestamp, &best_index));
	EXPECT_EQ(idx, best_index);

	// let it time out
	_group->get_best(timestamp + (uint64_t)(1.1f * kTimeoutUsec), &best_index);
	EXPECT_TRUE(validator->state() & DataValidator::ERROR_FLAG_TIMEOUT);
	EXPECT_EQ(idx, _group->failover_index());
}

/*
 * A priority of 0 disables a sensor: it must never be selected, but it stays tracked so that
 * its data is still logged and its health still reported.
 */

TEST_F(DataValidatorGroupTest, neverSelectsADisabledSensor)
{
	const int idx_disabled = add_validator();
	const int idx_enabled = add_validator();

	uint64_t timestamp = kBaseTimestamp;

	for (int i = 0; i < 50; i++) {
		timestamp += 5;
		put(idx_disabled, timestamp, 1.f + i * 1e-3f, 0);
		put(idx_enabled, timestamp, 2.f + i * 1e-3f, 50);
	}

	int best_index = -1;
	ASSERT_NE(nullptr, _group->get_best(timestamp, &best_index));
	EXPECT_EQ(idx_enabled, best_index);
}

TEST_F(DataValidatorGroupTest, disabledSensorIsNotSelectedEvenWhenItIsTheOnlyOneWithData)
{
	const int idx_disabled = add_validator();
	add_validator();

	uint64_t timestamp = kBaseTimestamp;

	for (int i = 0; i < 50; i++) {
		timestamp += 5;
		put(idx_disabled, timestamp, 1.f + i * 1e-3f, 0);
	}

	int best_index = -1;
	EXPECT_EQ(nullptr, _group->get_best(timestamp, &best_index));
	EXPECT_EQ(-1, best_index);
}

TEST_F(DataValidatorGroupTest, disabledSensorStillReportsItsErrorState)
{
	const int idx_disabled = add_validator();
	const int idx_enabled = add_validator();

	uint64_t timestamp = kBaseTimestamp;

	for (int i = 0; i < 50; i++) {
		timestamp += 5;
		put(idx_disabled, timestamp, 1.f + i * 1e-3f, 0);
		put(idx_enabled, timestamp, 2.f + i * 1e-3f, 50);
	}

	int best_index = -1;
	_group->get_best(timestamp, &best_index);
	EXPECT_EQ(DataValidator::ERROR_FLAG_NO_ERROR, _group->get_sensor_state(idx_disabled));

	// stop feeding it, and let it age past the timeout: being excluded from selection must
	// not freeze its health
	for (int i = 0; i < 50; i++) {
		timestamp += kTimeoutUsec / 10;
		put(idx_enabled, timestamp, 3.f + i * 1e-3f, 50);
	}

	_group->get_best(timestamp, &best_index);
	EXPECT_EQ(idx_enabled, best_index);
	EXPECT_TRUE(_group->get_sensor_state(idx_disabled) & DataValidator::ERROR_FLAG_TIMEOUT);
}

TEST_F(DataValidatorGroupTest, disablingTheSelectedSensorIsNotAFailover)
{
	const int idx0 = add_validator();
	const int idx1 = add_validator();

	uint64_t timestamp = kBaseTimestamp;
	int best_index = -1;

	for (int i = 0; i < 50; i++) {
		timestamp += 5;
		put(idx0, timestamp, 1.f + i * 1e-3f, 50);
		put(idx1, timestamp, 2.f + i * 1e-3f, 75);
		_group->get_best(timestamp, &best_index);
	}

	ASSERT_EQ(idx1, best_index);
	ASSERT_EQ(0u, _group->failover_count());

	// the operator disables the selected sensor: handing over is not a failure
	for (int i = 0; i < 5; i++) {
		timestamp += 5;
		put(idx0, timestamp, 1.1f + i * 1e-3f, 50);
		put(idx1, timestamp, 2.1f + i * 1e-3f, 0);
		_group->get_best(timestamp, &best_index);
	}

	EXPECT_EQ(idx0, best_index);
	EXPECT_EQ(0u, _group->failover_count());

	// disabling the last one clears the selection, still without a failover
	for (int i = 0; i < 5; i++) {
		timestamp += 5;
		put(idx0, timestamp, 1.2f + i * 1e-3f, 0);
		_group->get_best(timestamp, &best_index);
	}

	EXPECT_EQ(-1, best_index);
	EXPECT_EQ(0u, _group->failover_count());
}
