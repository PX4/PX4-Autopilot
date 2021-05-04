/****************************************************************************
 *
 *   Copyright (c) 2020 ECL Development Team. All rights reserved.
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
 * Test the gps yaw fusion
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"

class EkfGpsHeadingTest : public ::testing::Test {
 public:

	EkfGpsHeadingTest(): ::testing::Test(),
	_ekf{std::make_shared<Ekf>()},
	_sensor_simulator(_ekf),
	_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	void runConvergenceScenario(float yaw_offset_rad = 0.f, float antenna_offset_rad = 0.f);
	void checkConvergence(float truth, float tolerance = FLT_EPSILON);

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		_ekf->init(0);
		_sensor_simulator.runSeconds(_init_duration_s);
		_sensor_simulator._gps.setYaw(NAN);
		_sensor_simulator.runSeconds(2);
		_ekf_wrapper.enableGpsFusion();
		_ekf_wrapper.enableGpsHeadingFusion();
		_sensor_simulator.startGps();
		_sensor_simulator.runSeconds(11);
	}

	const uint32_t _init_duration_s{4};
};

void EkfGpsHeadingTest::runConvergenceScenario(float yaw_offset_rad, float antenna_offset_rad)
{
	// GIVEN: an initial GPS yaw, not aligned with the current one
	float gps_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle() + yaw_offset_rad);

	_sensor_simulator._gps.setYaw(gps_heading);
	_sensor_simulator._gps.setYawOffset(antenna_offset_rad);

	// WHEN: the GPS yaw fusion is activated
	_ekf_wrapper.enableGpsHeadingFusion();
	_sensor_simulator.runSeconds(5);

	// THEN: the estimate is reset and stays close to the measurement
	checkConvergence(gps_heading, 0.05f);
}

void EkfGpsHeadingTest::checkConvergence(float truth, float tolerance)
{
	const float yaw_est = _ekf_wrapper.getYawAngle();
	EXPECT_NEAR(yaw_est, truth, math::radians(tolerance))
		<< "yaw est: " << math::degrees(yaw_est) << "gps yaw: " << math::degrees(truth);
}

TEST_F(EkfGpsHeadingTest, fusionStartWithReset)
{
	// GIVEN:EKF that fuses GPS

	// WHEN: enabling GPS heading fusion and heading difference is bigger than 15 degrees
	const float gps_heading = _ekf_wrapper.getYawAngle() + math::radians(20.f);
	_sensor_simulator._gps.setYaw(gps_heading);
	_ekf_wrapper.enableGpsHeadingFusion();
	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.runSeconds(0.4);

	// THEN: GPS heading fusion should have started;
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeadingFusion());

	// AND: a reset to GPS heading is performed
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), gps_heading, 0.001);

	// WHEN: GPS heading is disabled
	_sensor_simulator._gps.stop();
	_sensor_simulator.runSeconds(11);

	// THEN: after a while the fusion should be stopped
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsHeadingFusion());
}

TEST_F(EkfGpsHeadingTest, yawConvergence)
{
	// GIVEN: an initial GPS yaw, not aligned with the current one
	const float initial_yaw = math::radians(10.f);
	float gps_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle() + initial_yaw);

	_sensor_simulator._gps.setYaw(gps_heading);

	// WHEN: the GPS yaw fusion is activated
	_ekf_wrapper.enableGpsHeadingFusion();
	_sensor_simulator.runSeconds(5);

	// THEN: the estimate is reset and stays close to the measurement
	checkConvergence(gps_heading, 0.05f);

	// AND WHEN: the the measurement changes
	gps_heading += math::radians(2.f);
	_sensor_simulator._gps.setYaw(gps_heading);
	_sensor_simulator.runSeconds(10);

	// THEN: the estimate slowly converges to the new measurement
	// Note that the process is slow, because the gyro did not detect any motion
	checkConvergence(gps_heading, 0.5f);
}

TEST_F(EkfGpsHeadingTest, yaw0)
{
	runConvergenceScenario();
}

TEST_F(EkfGpsHeadingTest, yaw60)
{
	const float yaw_offset_rad = math::radians(60.f);
	const float antenna_offset_rad = math::radians(80.f);
	runConvergenceScenario(yaw_offset_rad, antenna_offset_rad);
}

TEST_F(EkfGpsHeadingTest, yaw180)
{
	const float yaw_offset_rad = math::radians(180.f);
	const float antenna_offset_rad = math::radians(-20.f);
	runConvergenceScenario(yaw_offset_rad, antenna_offset_rad);
}

TEST_F(EkfGpsHeadingTest, yawMinus120)
{
	const float yaw_offset_rad = math::radians(120.f);
	const float antenna_offset_rad = math::radians(-42.f);
	runConvergenceScenario(yaw_offset_rad, antenna_offset_rad);
}

TEST_F(EkfGpsHeadingTest, yawMinus30)
{
	const float yaw_offset_rad = math::radians(-30.f);
	const float antenna_offset_rad = math::radians(10.f);
	runConvergenceScenario(yaw_offset_rad, antenna_offset_rad);
}

TEST_F(EkfGpsHeadingTest, fallBackToMag)
{
	// GIVEN: an initial GPS yaw, not aligned with the current one
	// GPS yaw is expected to arrive a bit later, first feed some NANs
	// to the filter
	_sensor_simulator.runSeconds(6);
	float gps_heading = _ekf_wrapper.getYawAngle() + math::radians(10.f);
	_sensor_simulator._gps.setYaw(gps_heading);

	// WHEN: the GPS yaw fusion is activated
	_sensor_simulator.runSeconds(1);

	// THEN: GPS heading fusion should have started, and mag
	// fusion should be disabled
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());

	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();

	// BUT WHEN: the GPS yaw is suddenly invalid
	gps_heading = NAN;
	_sensor_simulator._gps.setYaw(gps_heading);
	_sensor_simulator.runSeconds(7.5);

	// THEN: after a few seconds, the fusion should stop and
	// the estimator should fall back to mag fusion
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsHeadingFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
}
