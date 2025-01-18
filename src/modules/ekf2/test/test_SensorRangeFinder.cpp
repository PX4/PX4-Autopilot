/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <math.h>
#include "EKF/common.h"
#include "EKF/aid_sources/range_finder/sensor_range_finder.hpp"
#include <matrix/math.hpp>

using estimator::sensor::rangeSample;
using matrix::Dcmf;
using matrix::Eulerf;
using namespace estimator::sensor;

class SensorRangeFinderTest : public ::testing::Test
{
public:
	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		_range_finder.setPitchOffset(0.f);
		_range_finder.setCosMaxTilt(0.707f);
		_range_finder.setLimits(_min_range, _max_range);
		_range_finder.setMaxFogDistance(2.f);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}

protected:
	SensorRangeFinder _range_finder{};
	const rangeSample _good_sample{(uint64_t)2e6, 5.f, 100}; // {time_us, range, quality}
	const float _min_range{0.5f};
	const float _max_range{10.f};

	void updateSensorAtRate(rangeSample sample, uint64_t duration_us, uint64_t dt_update_us, uint64_t dt_sensor_us);
	void testTilt(const Eulerf &euler, bool should_pass);
};

void SensorRangeFinderTest::updateSensorAtRate(rangeSample sample, uint64_t duration_us, uint64_t dt_update_us,
		uint64_t dt_sensor_us)
{
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};

	rangeSample new_sample = sample;
	uint64_t t_now_us = sample.time_us;

	for (int i = 0; i < int(duration_us / dt_update_us); i++) {
		t_now_us += dt_update_us;

		if ((i % int(dt_sensor_us / dt_update_us)) == 0) {
			new_sample.rng += 0.2f; // update the range to not trigger the stuck detection

			if (new_sample.rng > _max_range) {
				new_sample.rng = _min_range;
			}

			new_sample.time_us = t_now_us;
			_range_finder.setSample(new_sample);
		}

		_range_finder.runChecks(t_now_us, attitude);
	}
}

void SensorRangeFinderTest::testTilt(const Eulerf &euler, bool should_pass)
{
	const Dcmf attitude{euler};
	_range_finder.setSample(_good_sample);
	_range_finder.runChecks(_good_sample.time_us, attitude);

	if (should_pass) {
		EXPECT_TRUE(_range_finder.isDataHealthy());
		EXPECT_TRUE(_range_finder.isHealthy());

	} else {
		EXPECT_FALSE(_range_finder.isDataHealthy());
		EXPECT_FALSE(_range_finder.isHealthy());
	}
}

TEST_F(SensorRangeFinderTest, setRange)
{
	rangeSample sample{};
	sample.rng = 1.f;
	sample.time_us = 1e6;
	sample.quality = 9;

	_range_finder.setRange(sample.rng);
	_range_finder.setDataReadiness(true);
	_range_finder.setValidity(true);
	EXPECT_TRUE(_range_finder.isDataHealthy());
	EXPECT_TRUE(_range_finder.isHealthy());
}

TEST_F(SensorRangeFinderTest, goodData)
{
	// WHEN: the drone is leveled and the data is good
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};
	_range_finder.setSample(_good_sample);
	_range_finder.runChecks(_good_sample.time_us, attitude);

	// THEN: the data can be used for aiding
	EXPECT_TRUE(_range_finder.isDataHealthy());
	EXPECT_TRUE(_range_finder.isHealthy());
}

TEST_F(SensorRangeFinderTest, tiltExceeded)
{
	const Eulerf zero(0.f, 0.f, 0.f);
	const Eulerf pitch_46(0.f, 0.8f, 0.f);
	const Eulerf pitch_minus46(0.f, -0.8f, 0.f);
	const Eulerf pitch_40(0.f, 0.7f, 1.f);
	const Eulerf pitch_minus40(0.f, -0.7f, 0.f);
	const Eulerf roll_46(0.8f, 0.f, 0.f);
	const Eulerf roll_minus46(-0.8f, 0.f, 0.f);
	const Eulerf roll_40(0.7f, 0.f, 2.f);
	const Eulerf roll_minus40(-0.7f, 0.f, 3.f);
	const Eulerf roll_28_pitch_minus28(0.5f, -0.5f, 4.f);
	const Eulerf roll_46_pitch_minus46(0.8f, -0.8f, 4.f);

	testTilt(zero, true);
	testTilt(pitch_46, false);
	testTilt(pitch_minus46, false);
	testTilt(pitch_40, true);
	testTilt(pitch_minus40, true);
	testTilt(roll_46, false);
	testTilt(roll_minus46, false);
	testTilt(roll_40, true);
	testTilt(roll_minus40, true);
	testTilt(roll_28_pitch_minus28, true);
	testTilt(roll_46_pitch_minus46, false);

}

TEST_F(SensorRangeFinderTest, rangeMaxExceeded)
{
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};

	// WHEN: the measured range is larger than the maximum
	rangeSample bad_sample = _good_sample;
	bad_sample.rng = _max_range + 0.01f;
	_range_finder.setSample(bad_sample);
	_range_finder.runChecks(bad_sample.time_us, attitude);

	// THEN: the data should be marked as unhealthy
	EXPECT_FALSE(_range_finder.isDataHealthy());
	EXPECT_FALSE(_range_finder.isHealthy());
}

TEST_F(SensorRangeFinderTest, rangeMinExceeded)
{
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};

	// WHEN: the measured range is shorter than the minimum
	rangeSample bad_sample = _good_sample;
	bad_sample.rng = _min_range - 0.01f;
	_range_finder.setSample(bad_sample);
	_range_finder.runChecks(bad_sample.time_us, attitude);

	// THEN: the data should be marked as unhealthy
	EXPECT_FALSE(_range_finder.isDataHealthy());
	EXPECT_FALSE(_range_finder.isHealthy());
}

TEST_F(SensorRangeFinderTest, outOfDate)
{
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};

	// WHEN: the data is outdated
	rangeSample outdated_sample = _good_sample;
	outdated_sample.time_us = 0;
	uint64_t t_now = _good_sample.time_us;
	_range_finder.setSample(outdated_sample);
	_range_finder.runChecks(t_now, attitude);

	// THEN: the data should be marked as unhealthy
	EXPECT_FALSE(_range_finder.isDataHealthy());
	EXPECT_FALSE(_range_finder.isHealthy());
}

TEST_F(SensorRangeFinderTest, rangeStuck)
{
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};

	// WHEN: the data is first not valid and then
	// constantly the same
	rangeSample new_sample = _good_sample;
	const uint64_t dt = 3e5;
	const uint64_t stuck_timeout = 11e6;
	new_sample.quality = 0;

	for (int i = 0; i < int(stuck_timeout / dt); i++) {
		_range_finder.setSample(new_sample);
		_range_finder.runChecks(new_sample.time_us, attitude);
		new_sample.time_us += dt;
	}

	EXPECT_FALSE(_range_finder.isDataHealthy());
	EXPECT_FALSE(_range_finder.isHealthy());

	new_sample.quality = 100;

	// we need a few sample to pass the hysteresis check
	for (int i = 0; i < int(2e6 / dt); i++) {
		_range_finder.setSample(new_sample);
		_range_finder.runChecks(new_sample.time_us, attitude);
		new_sample.time_us += dt;
	}

	// THEN: the data should be marked as unhealthy
	// because the sensor is "stuck"
	if (_range_finder.isStuckDetectorEnabled()) {
		EXPECT_FALSE(_range_finder.isDataHealthy());
		EXPECT_FALSE(_range_finder.isHealthy());

	} else {
		// If stuck detector is disabled then the
		// data should instantly be marked as healthy
		EXPECT_TRUE(_range_finder.isDataHealthy());
		EXPECT_TRUE(_range_finder.isHealthy());
	}

	// BUT WHEN: we continue to send samples but with changing distance
	for (int i = 0; i < 2; i++) {
		new_sample.rng += 1.f;
		_range_finder.setSample(new_sample);
		_range_finder.runChecks(new_sample.time_us, attitude);
		new_sample.time_us += dt;
	}

	// THEN: the data should be marked as healthy
	// because the sensor is not "stuck" anymore
	EXPECT_TRUE(_range_finder.isDataHealthy());
	EXPECT_TRUE(_range_finder.isHealthy());
}

TEST_F(SensorRangeFinderTest, qualityHysteresis)
{
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};

	// WHEN: the data is first bad and then good
	rangeSample new_sample = _good_sample;

	new_sample.quality = 0;
	_range_finder.setSample(new_sample);
	_range_finder.runChecks(new_sample.time_us, attitude);
	EXPECT_FALSE(_range_finder.isDataHealthy());
	EXPECT_FALSE(_range_finder.isHealthy());

	new_sample.quality = _good_sample.quality;
	_range_finder.setSample(new_sample);
	_range_finder.runChecks(new_sample.time_us, attitude);
	EXPECT_FALSE(_range_finder.isDataHealthy());
	EXPECT_FALSE(_range_finder.isHealthy());

	// AND: we need to put enough good data to pass the hysteresis
	const uint64_t dt = 3e5;
	const uint64_t hyst_time = 1e6;

	for (int i = 0; i < int(hyst_time / dt) + 2; i++) {
		_range_finder.setSample(new_sample);
		_range_finder.runChecks(new_sample.time_us, attitude);
		new_sample.time_us += dt;
	}

	// THEN: the data is again declared healthy
	EXPECT_TRUE(_range_finder.isDataHealthy());
	EXPECT_TRUE(_range_finder.isHealthy());
}

TEST_F(SensorRangeFinderTest, continuity)
{
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};

	// WHEN: the data rate is too slow
	const uint64_t dt_update_us = 10e3;
	uint64_t dt_sensor_us = 4e6;
	uint64_t duration_us = 8e6;
	updateSensorAtRate(_good_sample, duration_us, dt_update_us, dt_sensor_us);

	// THEN: the data should be marked as unhealthy
	// Note that it also fails the out-of-date test here
	EXPECT_FALSE(_range_finder.isDataHealthy());
	EXPECT_FALSE(_range_finder.isHealthy());

	// AND WHEN: the data rate is acceptable
	dt_sensor_us = 3e5;
	duration_us = 5e5;
	updateSensorAtRate(_good_sample, duration_us, dt_update_us, dt_sensor_us);

	// THEN: it should still fail until the filter converge
	// to the new datarate
	EXPECT_FALSE(_range_finder.isDataHealthy());
	EXPECT_FALSE(_range_finder.isHealthy());

	updateSensorAtRate(_good_sample, duration_us, dt_update_us, dt_sensor_us);
	EXPECT_TRUE(_range_finder.isDataHealthy());
	EXPECT_TRUE(_range_finder.isHealthy());
}

TEST_F(SensorRangeFinderTest, distBottom)
{
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};
	rangeSample sample{};
	sample.rng = 1.f;
	sample.time_us = 1e6;
	sample.quality = 9;

	_range_finder.setSample(sample);
	_range_finder.runChecks(sample.time_us, attitude);
	EXPECT_FLOAT_EQ(_range_finder.getDistBottom(), sample.rng);

	const Dcmf attitude20{Eulerf(-0.35f, 0.f, 0.f)};
	_range_finder.runChecks(sample.time_us, attitude20);
	EXPECT_FLOAT_EQ(_range_finder.getDistBottom(), sample.rng * cosf(-0.35));
}

TEST_F(SensorRangeFinderTest, blockedByFog)
{
	// WHEN: sensor is not blocked by fog
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};
	const uint64_t dt_update_us = 10e3;
	uint64_t dt_sensor_us = 3e5;
	uint64_t duration_us = 5e5;

	updateSensorAtRate(_good_sample, duration_us, dt_update_us, dt_sensor_us);
	// THEN: the data should be marked as healthy
	EXPECT_TRUE(_range_finder.isDataHealthy());
	EXPECT_TRUE(_range_finder.isHealthy());


	// WHEN: sensor is then blocked by fog
	// range jumps to value below 2m
	uint64_t t_now_us = _range_finder.getSampleAddress()->time_us;
	rangeSample sample{t_now_us, 1.f, 100};
	updateSensorAtRate(sample, duration_us, dt_update_us, dt_sensor_us);

	// THEN: the data should be marked as unhealthy
	EXPECT_FALSE(_range_finder.isDataHealthy());
	EXPECT_FALSE(_range_finder.isHealthy());

	// WHEN: the sensor is not blocked by fog anymore
	sample.rng = 5.f;
	updateSensorAtRate(sample, duration_us, dt_update_us, dt_sensor_us);

	// THEN: the data should be marked as healthy again
	EXPECT_TRUE(_range_finder.isDataHealthy());
	EXPECT_TRUE(_range_finder.isHealthy());

	// WHEN: the sensor is is not jumping to a value below 2m
	while (sample.rng > _min_range) {
		sample.time_us += dt_update_us;
		_range_finder.setSample(sample);
		_range_finder.runChecks(sample.time_us, attitude);
		sample.rng -= 0.5f;
	}

	// THEN: the data should still be marked as healthy
	EXPECT_TRUE(_range_finder.isDataHealthy());
	EXPECT_TRUE(_range_finder.isHealthy());

}
