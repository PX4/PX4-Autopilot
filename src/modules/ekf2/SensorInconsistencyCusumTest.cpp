/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * to run: make tests TESTFILTER=SensorInconsistencyCusum
 */

#include <gtest/gtest.h>

#include "SensorInconsistencyCusum.hpp"

class SensorInconsistencyCusumTest : public ::testing::Test
{
public:
	static constexpr uint8_t kSize = 4;
	static constexpr float kDeadband = 2.f;
	static constexpr float kLimit = 5.f;
	static constexpr float kDt = 0.01f;

	void run(float duration_s)
	{
		const int steps = static_cast<int>(roundf(duration_s / kDt));

		for (int i = 0; i < steps; i++) {
			_cusum.update(_device_ids, _inconsistency, kDeadband, kLimit, kDt);
		}
	}

	void setSensorCount(uint8_t count)
	{
		for (uint8_t i = 0; i < kSize; i++) {
			_device_ids[i] = (i < count) ? (101 + i) : 0;
		}
	}

	// deviation of each sensor from the mean of all sensors when one of them is off by sensor_error
	void setSensorError(uint8_t faulty_index, float sensor_error)
	{
		uint8_t sensor_count = 0;

		for (uint8_t i = 0; i < kSize; i++) {
			sensor_count += (_device_ids[i] != 0) ? 1 : 0;
		}

		const float mean = sensor_error / sensor_count;

		for (uint8_t i = 0; i < kSize; i++) {
			_inconsistency[i] = fabsf(((i == faulty_index) ? sensor_error : 0.f) - mean);
		}
	}

	SensorInconsistencyCusum<kSize> _cusum;
	uint32_t _device_ids[kSize] {101, 102, 103, 0};
	float _inconsistency[kSize] {};
};

TEST_F(SensorInconsistencyCusumTest, staysAtZeroBelowDeadband)
{
	setSensorError(0, 0.9f * kDeadband);
	run(100.f);

	EXPECT_FALSE(_cusum.faultDetected());
	EXPECT_EQ(_cusum.faultyDeviceId(), 0u);
	EXPECT_FLOAT_EQ(_cusum.accumulatedError(0), 0.f);
	EXPECT_FLOAT_EQ(_cusum.accumulatedError(1), 0.f);
	EXPECT_FLOAT_EQ(_cusum.accumulatedError(2), 0.f);
}

TEST_F(SensorInconsistencyCusumTest, detectsOnceExcessIntegratesToLimit)
{
	const float excess = 4.f;
	const float time_to_limit_s = kLimit / excess;
	setSensorError(0, kDeadband + excess);

	run(0.9f * time_to_limit_s);
	EXPECT_FALSE(_cusum.faultDetected());
	EXPECT_NEAR(_cusum.accumulatedError(0), 0.9f * kLimit, 0.01f * kLimit);

	run(0.2f * time_to_limit_s);
	EXPECT_TRUE(_cusum.faultDetected());
	EXPECT_EQ(_cusum.faultyDeviceId(), _device_ids[0]);
}

TEST_F(SensorInconsistencyCusumTest, detectionTimeIndependentOfSensorCount)
{
	const float excess = 4.f;
	const float time_to_limit_s = kLimit / excess;

	for (uint8_t sensor_count = 2; sensor_count <= kSize; sensor_count++) {
		_cusum = {};
		setSensorCount(sensor_count);
		setSensorError(0, kDeadband + excess);

		run(0.9f * time_to_limit_s);
		EXPECT_FALSE(_cusum.faultDetected()) << "sensor count " << static_cast<int>(sensor_count);

		run(0.2f * time_to_limit_s);
		EXPECT_TRUE(_cusum.faultDetected()) << "sensor count " << static_cast<int>(sensor_count);
	}
}

TEST_F(SensorInconsistencyCusumTest, healthyPeriodEarnsNoCredit)
{
	const float excess = 4.f;
	const float time_to_limit_s = kLimit / excess;

	setSensorError(0, 0.f);
	run(100.f);
	EXPECT_FLOAT_EQ(_cusum.accumulatedError(0), 0.f);

	setSensorError(0, kDeadband + excess);
	run(1.1f * time_to_limit_s);
	EXPECT_TRUE(_cusum.faultDetected());
}

TEST_F(SensorInconsistencyCusumTest, clearsAfterDrainingBelowLimit)
{
	setSensorError(0, kDeadband + 4.f);
	run(2.f);
	ASSERT_TRUE(_cusum.faultDetected());
	const float error_at_fault_end = _cusum.accumulatedError(0);

	setSensorError(0, 0.f);
	const float time_to_drain_to_limit_s = (error_at_fault_end - kLimit) / kDeadband;

	run(0.9f * time_to_drain_to_limit_s);
	EXPECT_TRUE(_cusum.faultDetected());

	run(0.2f * time_to_drain_to_limit_s);
	EXPECT_FALSE(_cusum.faultDetected());
	EXPECT_EQ(_cusum.faultyDeviceId(), 0u);
}

TEST_F(SensorInconsistencyCusumTest, isolatesFaultySensorWithThreeSensors)
{
	// large enough for the shifted mean to push the two healthy sensors over the limit as well
	setSensorError(1, 10.f * kDeadband);
	run(10.f);

	EXPECT_TRUE(_cusum.faultDetected());
	EXPECT_GT(_cusum.accumulatedError(0), kLimit);
	EXPECT_GT(_cusum.accumulatedError(2), kLimit);
	EXPECT_EQ(_cusum.faultyDeviceId(), _device_ids[1]);
}

TEST_F(SensorInconsistencyCusumTest, cannotIsolateWithTwoSensors)
{
	setSensorCount(2);
	setSensorError(0, 10.f * kDeadband);
	run(10.f);

	EXPECT_TRUE(_cusum.faultDetected());
	EXPECT_EQ(_cusum.faultyDeviceId(), 0u);
}

TEST_F(SensorInconsistencyCusumTest, singleSensorNeverFaulty)
{
	setSensorCount(1);
	const float excess = 8.f;
	_inconsistency[0] = kDeadband + excess;
	run(10.f);

	EXPECT_NEAR(_cusum.accumulatedError(0), 10.f * excess, 0.01f * kLimit);
	EXPECT_FALSE(_cusum.faultDetected());
	EXPECT_EQ(_cusum.faultyDeviceId(), 0u);
}

TEST_F(SensorInconsistencyCusumTest, absentSensorIsIgnored)
{
	_inconsistency[3] = 100.f;
	run(10.f);

	EXPECT_TRUE(isnan(_cusum.accumulatedError(3)));
	EXPECT_FALSE(_cusum.faultDetected());
}

TEST_F(SensorInconsistencyCusumTest, sensorAppearingLaterStartsFromZero)
{
	run(1.f);
	ASSERT_TRUE(isnan(_cusum.accumulatedError(3)));

	setSensorCount(4);
	const float excess = 4.f;
	setSensorError(3, kDeadband + excess);
	run(0.5f);

	EXPECT_NEAR(_cusum.accumulatedError(3), 0.5f * excess, 0.01f * kLimit);
}
