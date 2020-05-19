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

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		_ekf->init(0);
		_sensor_simulator.runSeconds(2);
		_ekf_wrapper.enableGpsFusion();
		_sensor_simulator.startGps();
		_sensor_simulator.runSeconds(11);
	}
};

TEST_F(EkfGpsHeadingTest, fusionStartWithReset)
{
	// GIVEN:EKF that fuses GPS

	// WHEN: enabling GPS heading fusion and heading difference is bigger than 15 degrees
	const float gps_heading = _ekf_wrapper.getYawAngle() + math::radians(20.f);
	_sensor_simulator._gps.setYaw(gps_heading);
	_ekf_wrapper.enableGpsHeadingFusion();
	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.runSeconds(0.2);

	// THEN: GPS heading fusion should have started;
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeadingFusion());

	// AND: a reset to GPS heading is performed
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), gps_heading, 0.001);

	// WHEN: GPS heading is disabled
	_sensor_simulator._gps.stop();
	_sensor_simulator.runSeconds(11);

	// THEN: after a while the fusion should be stopped
	// TODO: THIS IS NOT HAPPENING
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsHeadingFusion());
}

TEST_F(EkfGpsHeadingTest, fusionStartWithoutReset)
{
	// GIVEN:EKF that fuses GPS

	// WHEN: enabling GPS heading fusion and heading difference is smaller than 15 degrees
	const float gps_heading = _ekf_wrapper.getYawAngle() + math::radians(10.f);
	_sensor_simulator._gps.setYaw(gps_heading);
	_ekf_wrapper.enableGpsHeadingFusion();
	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.runSeconds(0.2);

	// THEN: GPS heading fusion should have started;
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeadingFusion());

	// AND: no reset to GPS heading should be performed
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter);

	// TODO: investigate why heading is not converging exactly to GPS heading
}
