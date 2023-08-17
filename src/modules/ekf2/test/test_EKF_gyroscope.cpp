/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * Gyro-related tests (bias convergence, ...)
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"


class EkfGyroscopeTest : public ::testing::Test
{
public:

	EkfGyroscopeTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// Init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}

	void testBias(const Vector3f &bias, float duration, const Vector3f &tolerance);
};

void EkfGyroscopeTest::testBias(const Vector3f &bias, float duration, const Vector3f &tolerance)
{
	_sensor_simulator._imu.setGyroData(bias);
	_sensor_simulator.runSeconds(duration);
	EXPECT_TRUE(_ekf->control_status_flags().vehicle_at_rest);

	const Vector3f estimated_bias = _ekf->getGyroBias();

	for (int i = 0; i < 3; i++) {
		EXPECT_NEAR(estimated_bias(i), bias(i), tolerance(i)) << "index " << i;
	}
}

TEST_F(EkfGyroscopeTest, biasEstimateZero)
{
	testBias(Vector3f(), 10, Vector3f());
}

TEST_F(EkfGyroscopeTest, biasEstimatePositive)
{
	// The estimate should track a slowly changing bias
	const float biases[4] = {0.001f, 0.002f, 0.0035f, 0.005f};
	Vector3f bias;

	for (int i = 0; i < 4; i ++) {
		bias.setAll(biases[i]);
		// The Z gyro bias takes more time to converge as the Z rotation variance is higher
		testBias(bias, 30, Vector3f(0.0008f, 0.0008f, 0.004f));
	}
}

TEST_F(EkfGyroscopeTest, biasEstimateNegative)
{
	const float biases[4] = {-0.001f, -0.002f, -0.0035, -0.005f};
	Vector3f bias;

	for (int i = 0; i < 4; i ++) {
		bias.setAll(biases[i]);
		testBias(bias, 30, Vector3f(0.0008f, 0.0008f, 0.004f));
	}
}
