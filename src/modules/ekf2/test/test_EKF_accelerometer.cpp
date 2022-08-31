/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * Accelerometer failure handling (bias, clipping, ...)
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"


class EkfAccelerometerTest : public ::testing::Test
{
public:

	EkfAccelerometerTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// run briefly to init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_sensor_simulator.runSeconds(7);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}

	void testBias(float bias, float duration, float tolerance);
};

void EkfAccelerometerTest::testBias(float bias, float duration, float tolerance)
{
	_sensor_simulator._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G + bias));
	_sensor_simulator.runSeconds(duration);
	Vector3f estimated_bias = _ekf->getAccelBias();
	EXPECT_TRUE(matrix::isEqual(estimated_bias, Vector3f(0.f, 0.f, bias),
				    tolerance)) << "bias = " << bias << ", estimated = " << estimated_bias(2);
}

TEST_F(EkfAccelerometerTest, biasEstimateZero)
{
	testBias(0.f, 10, 0.f);
}

TEST_F(EkfAccelerometerTest, biasEstimatePositive)
{
	// The estimate should track a slowly changing bias
	const float biases[4] = {0.1f, 0.2f, 0.3f, 0.38f};

	for (int i = 0; i < 4; i ++) {
		testBias(biases[i], 10, 0.03f);
	}
}

TEST_F(EkfAccelerometerTest, biasEstimateNegative)
{
	const float biases[4] = {-0.12f, -0.22f, -0.31, -0.4f};

	for (int i = 0; i < 4; i ++) {
		testBias(biases[i], 10, 0.03f);
	}
}

TEST_F(EkfAccelerometerTest, imuFallingDetectionBaroOnly)
{
	// GIVEN: an accelerometer with a really large Z bias
	const float bias = CONSTANTS_ONE_G;
	_sensor_simulator._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G + bias));
	_sensor_simulator.runSeconds(2);

	// WHEN: there is only one source of vertical aiding
	// THEN: the estimator cannot know which one is wrong
	EXPECT_FALSE(_ekf->fault_status_flags().bad_acc_vertical);

	// BUT WHEN: the accelerometer also reports clipping on the Z axis
	_sensor_simulator._imu.setAccelClipping(false, false, true);
	_sensor_simulator.runSeconds(2);

	// THEN: a single source is enough to detect a bad acceleration
	EXPECT_TRUE(_ekf->fault_status_flags().bad_acc_vertical);
}

TEST_F(EkfAccelerometerTest, imuFallingDetectionBaroGnssVel)
{
	// GIVEN: Baro and GNSS velocity fusion
	_sensor_simulator.startGps();
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.runSeconds(15);

	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());

	// AND: an accelerometer with a really large Z bias
	const float bias = CONSTANTS_ONE_G;
	_sensor_simulator._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G + bias));
	_sensor_simulator.runSeconds(2);

	// THEN: the bad vertical acceleration is detected
	EXPECT_TRUE(_ekf->fault_status_flags().bad_acc_vertical);

	// AND WHEN: the accelerometer also reports clipping on the Z axis
	_sensor_simulator._imu.setAccelClipping(false, false, true);
	_sensor_simulator.runSeconds(2);

	// THEN: the bad vertical acceleration is still detected
	EXPECT_TRUE(_ekf->fault_status_flags().bad_acc_vertical);
}

TEST_F(EkfAccelerometerTest, imuFallingDetectionGnssOnly)
{
	// GIVEN: GNSS height and velocity fusion
	_sensor_simulator.startGps();
	_ekf_wrapper.enableGpsFusion();
	_ekf_wrapper.enableGpsHeightFusion();
	_ekf_wrapper.disableBaroHeightFusion();
	_sensor_simulator.runSeconds(15);

	EXPECT_FALSE(_ekf_wrapper.isIntendingBaroHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeightFusion());

	// AND: an accelerometer with a really large Z bias
	const float bias = CONSTANTS_ONE_G;
	_sensor_simulator._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G + bias));
	_sensor_simulator.runSeconds(2);

	// THEN: the bad vertical acceleration is not detected because both sources are of the same type
	EXPECT_FALSE(_ekf->fault_status_flags().bad_acc_vertical);

	// BUT WHEN: the accelerometer also reports clipping on the Z axis
	_sensor_simulator._imu.setAccelClipping(false, false, true);
	_sensor_simulator.runSeconds(2);

	// THEN: a single source is enough to detect a bad acceleration
	EXPECT_TRUE(_ekf->fault_status_flags().bad_acc_vertical);
}

TEST_F(EkfAccelerometerTest, imuFallingDetectionBaroRange)
{
	// GIVEN: baro and range height fusion
	_sensor_simulator._rng.setData(1.f, 100);
	_sensor_simulator._rng.setLimits(0.1f, 9.f);
	_sensor_simulator.startRangeFinder();
	_ekf_wrapper.enableRangeHeightFusion();
	_sensor_simulator.runSeconds(5);

	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingRangeHeightFusion());

	// AND: an accelerometer with a really large Z bias
	const float bias = CONSTANTS_ONE_G;
	_sensor_simulator._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G + bias));
	_sensor_simulator.runSeconds(2);

	// THEN: the bad vertical is detected because both sources agree
	EXPECT_TRUE(_ekf->fault_status_flags().bad_acc_vertical);
}

TEST_F(EkfAccelerometerTest, imuFallingDetectionBaroEvVel)
{
	// GIVEN: baro and EV vel fusion
	_ekf_wrapper.enableExternalVisionVelocityFusion();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(1);

	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());

	// AND: an accelerometer with a really large Z bias
	const float bias = CONSTANTS_ONE_G;
	_sensor_simulator._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G + bias));
	_sensor_simulator.runSeconds(2);

	// THEN: the bad vertical is detected because both sources agree
	EXPECT_TRUE(_ekf->fault_status_flags().bad_acc_vertical);
}

TEST_F(EkfAccelerometerTest, imuFallingDetectionEvVelHgt)
{
	// GIVEN: EV height and vel fusion
	_ekf_wrapper.enableExternalVisionVelocityFusion();
	_ekf_wrapper.enableExternalVisionHeightFusion();
	_sensor_simulator.startExternalVision();
	_ekf_wrapper.disableBaroHeightFusion();
	_sensor_simulator.runSeconds(1);

	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionHeightFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingBaroHeightFusion());

	// AND: an accelerometer with a really large Z bias
	const float bias = CONSTANTS_ONE_G;
	_sensor_simulator._imu.setAccelData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G + bias));
	_sensor_simulator.runSeconds(2);

	// THEN: the bad vertical acceleration is not detected because both sources are of the same type
	EXPECT_FALSE(_ekf->fault_status_flags().bad_acc_vertical);

	// BUT WHEN: the accelerometer also reports clipping on the Z axis
	_sensor_simulator._imu.setAccelClipping(false, false, true);
	_sensor_simulator.runSeconds(2);

	// THEN: a single source is enough to detect a bad acceleration
	EXPECT_TRUE(_ekf->fault_status_flags().bad_acc_vertical);
}
