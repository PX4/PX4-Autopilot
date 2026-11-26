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

	void setInconsistency(float sensor_0, float sensor_1, float sensor_2)
	{
		_inconsistency[0] = sensor_0;
		_inconsistency[1] = sensor_1;
		_inconsistency[2] = sensor_2;
	}

	SensorInconsistencyCusum<kSize> _cusum;
	uint32_t _device_ids[kSize] {101, 102, 103, 0};
	float _inconsistency[kSize] {};
};

TEST_F(SensorInconsistencyCusumTest, staysAtZeroBelowDeadband)
{
	setInconsistency(0.9f * kDeadband, 0.5f * kDeadband, 0.f);
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
	setInconsistency(kDeadband + excess, 0.f, 0.f);

	run(0.9f * time_to_limit_s);
	EXPECT_FALSE(_cusum.faultDetected());
	EXPECT_NEAR(_cusum.accumulatedError(0), 0.9f * kLimit, 0.01f * kLimit);

	run(0.2f * time_to_limit_s);
	EXPECT_TRUE(_cusum.faultDetected());
	EXPECT_EQ(_cusum.faultyDeviceId(), _device_ids[0]);
}

TEST_F(SensorInconsistencyCusumTest, healthyPeriodEarnsNoCredit)
{
	const float excess = 4.f;
	const float time_to_limit_s = kLimit / excess;

	setInconsistency(0.f, 0.f, 0.f);
	run(100.f);
	EXPECT_FLOAT_EQ(_cusum.accumulatedError(0), 0.f);

	setInconsistency(kDeadband + excess, 0.f, 0.f);
	run(1.1f * time_to_limit_s);
	EXPECT_TRUE(_cusum.faultDetected());
}

TEST_F(SensorInconsistencyCusumTest, clearsAfterDrainingBelowLimit)
{
	setInconsistency(kDeadband + 4.f, 0.f, 0.f);
	run(2.f);
	ASSERT_TRUE(_cusum.faultDetected());
	const float error_at_fault_end = _cusum.accumulatedError(0);

	setInconsistency(0.f, 0.f, 0.f);
	const float time_to_drain_to_limit_s = (error_at_fault_end - kLimit) / kDeadband;

	run(0.9f * time_to_drain_to_limit_s);
	EXPECT_TRUE(_cusum.faultDetected());

	run(0.2f * time_to_drain_to_limit_s);
	EXPECT_FALSE(_cusum.faultDetected());
	EXPECT_EQ(_cusum.faultyDeviceId(), 0u);
}

TEST_F(SensorInconsistencyCusumTest, isolatesLargestErrorWithThreeSensors)
{
	// a fault on sensor 1 also shifts the mean the two healthy sensors are compared with
	setInconsistency(5.f, 10.f, 5.f);
	run(10.f);

	EXPECT_TRUE(_cusum.faultDetected());
	EXPECT_GT(_cusum.accumulatedError(0), kLimit);
	EXPECT_GT(_cusum.accumulatedError(2), kLimit);
	EXPECT_EQ(_cusum.faultyDeviceId(), _device_ids[1]);
}

TEST_F(SensorInconsistencyCusumTest, cannotIsolateWithTwoSensors)
{
	_device_ids[2] = 0;
	setInconsistency(6.f, 6.f, 0.f);
	run(10.f);

	EXPECT_TRUE(_cusum.faultDetected());
	EXPECT_EQ(_cusum.faultyDeviceId(), 0u);
}

TEST_F(SensorInconsistencyCusumTest, singleSensorNeverFaulty)
{
	_device_ids[1] = 0;
	_device_ids[2] = 0;
	setInconsistency(10.f, 0.f, 0.f);
	run(10.f);

	EXPECT_GT(_cusum.accumulatedError(0), kLimit);
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

	_device_ids[3] = 104;
	const float excess = 4.f;
	_inconsistency[3] = kDeadband + excess;
	run(0.5f);

	EXPECT_NEAR(_cusum.accumulatedError(3), 0.5f * excess, 0.01f * kLimit);
}
