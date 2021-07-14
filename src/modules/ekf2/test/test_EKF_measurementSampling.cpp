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

class EkfMeasurementSamplingTest : public ::testing::Test
{
public:
	EkfMeasurementSamplingTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()} {};

	std::shared_ptr<Ekf> _ekf;

	void SetUp() override
	{
		_ekf->init(0);
	}
};

TEST_F(EkfMeasurementSamplingTest, baroDownSampling)
{
	// Need to send imu samples to initialise
	imuSample imu_sample;

	// WHEN: baro data = {1, -1 , 1, -1, ...} being set at high rate
	int baro_rate_Hz = 10000;
	uint64_t time = 0;
	float baro_data = 1.0f;

	for (int i = 0; i < 2 * baro_rate_Hz; i++) {
		if (i % 100 == 0) {
			// send imu data at a 100 times lower rate
			imu_sample.time_us = time;
			_ekf->setIMUData(imu_sample);
		}

		_ekf->setBaroData(baroSample{time, baro_data});
		baro_data *= -1.0f;
		time += 1000000 / baro_rate_Hz;
	}

	_ekf->update();

	// THEN: average and buffered baro dato should be close to zero
	baroSample baro_sample_from_buffer = _ekf->get_baro_sample_delayed();
	EXPECT_NEAR(baro_sample_from_buffer.hgt, 0.0f, 0.01f);
}
