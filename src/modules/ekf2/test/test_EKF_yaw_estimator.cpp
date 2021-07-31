/****************************************************************************
 *
 *   Copyright (c) 2021 ECL Development Team. All rights reserved.
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
 * Test the yaw estimator
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"

class EKFYawEstimatorTest : public ::testing::Test
{
public:

	EKFYawEstimatorTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	// Setup the Ekf with mag aiding disabled
	void SetUp() override
	{
		_ekf->init(0);
		_ekf_wrapper.setMagFuseTypeNone();
		_sensor_simulator.runSeconds(2);
		_ekf_wrapper.enableGpsFusion();
		_sensor_simulator.startGps();
		_sensor_simulator.runSeconds(11);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}
};

TEST_F(EKFYawEstimatorTest, inAirYawAlignment)
{
	// GIVEN: an accelerating vehicle with unknown heading
	EXPECT_EQ(1, (int) _ekf->control_status_flags().tilt_align);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().yaw_align);

	const Vector3f accel_frd{1.0, -1.f, 0.f};
	_sensor_simulator._imu.setAccelData(accel_frd + Vector3f(0.f, 0.f, -CONSTANTS_ONE_G));
	const float dt = 0.5f;
	const float yaw = math::radians(-30.f);
	const Dcmf R_to_earth{Eulerf(0.f, 0.f, yaw)};

	_ekf->set_in_air_status(true);
	ResetLoggingChecker reset_logging_checker(_ekf);
	reset_logging_checker.capturePreResetState();

	// WHEN: GNSS velocity is available
	Vector3f simulated_velocity{};

	for (int i = 0; i < 10; i++) {
		_sensor_simulator.runSeconds(dt);
		simulated_velocity += R_to_earth * accel_frd * dt;
		_sensor_simulator._gps.setVelocity(simulated_velocity);
	}

	// THEN: the heading can be estimated and then used to fuse GNSS vel and pos to the main EKF
	float yaw_est{};
	float yaw_est_var{};
	float dummy[5];
	_ekf->getDataEKFGSF(&yaw_est, &yaw_est_var, dummy, dummy, dummy, dummy);

	const float tolerance_rad = math::radians(5.f);
	EXPECT_NEAR(yaw_est, yaw, tolerance_rad);
	EXPECT_LT(yaw_est_var, tolerance_rad);

	// 2 resets: 1 after IMU+GNSS yaw alignment and 1 when starting GNSS aiding
	reset_logging_checker.capturePostResetState();
	EXPECT_TRUE(reset_logging_checker.isHorizontalVelocityResetCounterIncreasedBy(2));
	EXPECT_TRUE(reset_logging_checker.isHorizontalPositionResetCounterIncreasedBy(2));

	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_TRUE(_ekf->global_position_is_valid());
}
