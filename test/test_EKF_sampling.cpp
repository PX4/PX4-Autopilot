/****************************************************************************
 *
 *   Copyright (c) 2019 ECL Development Team. All rights reserved.
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
#include "EKF/ekf.h"

class EkfImuSamplingTestParametrized : public ::testing::TestWithParam<std::tuple<float,float>>
{
 public:

	Ekf _ekf{};

	uint32_t _t_us{0};

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		_ekf.init(0);

	}

	void TearDown() override
	{

	}
};

TEST_P(EkfImuSamplingTestParametrized, imuSamplingAtMultipleRates)
{
	// WHEN: adding imu samples at a higher rate than the update loop
	// THEN: imu sample should be down sampled
	// WHEN: adding imu samples at a same or lower rate than the update loop
	// THEN: imu sample should reach buffer unchanged

	uint32_t dt_us = std::get<0>(GetParam()) * (_ekf.FILTER_UPDATE_PERIOD_MS * 1000);
	uint32_t expected_dt_us = std::get<1>(GetParam()) * (_ekf.FILTER_UPDATE_PERIOD_MS * 1000);

	Vector3f ang_vel{0.0f,0.0f,0.0f};
	Vector3f accel{-0.46f,0.87f,0.0f};
	imuSample imu_sample;
	imu_sample.delta_ang_dt = dt_us * 1.e-6f;
	imu_sample.delta_ang = ang_vel * imu_sample.delta_ang_dt;
	imu_sample.delta_vel_dt = dt_us * 1.e-6f;
	imu_sample.delta_vel = accel * imu_sample.delta_vel_dt;

	// The higher the imu rate is the more measurements we have to set before reaching the FILTER_UPDATE_PERIOD
	for(int i = 0; i<(int)20/std::get<0>(GetParam()); ++i)
	{
		imu_sample.time_us = _t_us;
		_ekf.setIMUData(imu_sample);
		_t_us += dt_us;
	}

	// Get the imu sample that was put into the buffer
	imuSample imu_sample_buffered = _ekf.get_imu_sample_delayed();
	EXPECT_NEAR(expected_dt_us / 1e6f, imu_sample_buffered.delta_ang_dt, 1e-5f);
	EXPECT_NEAR(expected_dt_us / 1e6f, imu_sample_buffered.delta_vel_dt, 1e-5f);

	// WHEN: downsampling the imu measurement
	// THEN: the delta vel should be accumulated correctly
	EXPECT_NEAR(ang_vel(0), imu_sample_buffered.delta_ang(0)/imu_sample_buffered.delta_ang_dt, 1e-3f);
	EXPECT_NEAR(ang_vel(1), imu_sample_buffered.delta_ang(1)/imu_sample_buffered.delta_ang_dt, 1e-3f);
	EXPECT_NEAR(ang_vel(2), imu_sample_buffered.delta_ang(2)/imu_sample_buffered.delta_ang_dt, 1e-3f);
	EXPECT_NEAR(accel(0), imu_sample_buffered.delta_vel(0)/imu_sample_buffered.delta_vel_dt, 1e-3f);
	EXPECT_NEAR(accel(1), imu_sample_buffered.delta_vel(1)/imu_sample_buffered.delta_vel_dt, 1e-3f);
	EXPECT_NEAR(accel(2), imu_sample_buffered.delta_vel(2)/imu_sample_buffered.delta_vel_dt, 1e-3f);

}

INSTANTIATE_TEST_CASE_P(imuSamplingAtMultipleRates,
			EkfImuSamplingTestParametrized,
			::testing::Values(
				std::make_tuple<float,float>(1.0f,1.0f),
				std::make_tuple<float,float>(1.6f,1.6f),
				std::make_tuple<float,float>(0.333f,1.0f)
			));
