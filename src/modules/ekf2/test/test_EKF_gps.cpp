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

/**
 * Test the gps fusion
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"

class EkfGpsTest : public ::testing::Test
{
public:

	EkfGpsTest(): ::testing::Test(),
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

TEST_F(EkfGpsTest, gpsTimeout)
{
	// GIVEN:EKF that fuses GPS

	// WHEN: setting the PDOP to high
	_sensor_simulator._gps.setNumberOfSatellites(3);

	// THEN: EKF should stop fusing GPS
	_sensor_simulator.runSeconds(20);

	// TODO: this is not happening as expected
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
}

TEST_F(EkfGpsTest, resetToGpsVelocity)
{
	ResetLoggingChecker reset_logging_checker(_ekf);
	// GIVEN:EKF that fuses GPS
	// and has gps checks already passed

	// WHEN: stopping GPS fusion
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11);

	reset_logging_checker.capturePreResetState();

	// AND: simulate constant velocity gps samples for short time
	_sensor_simulator.startGps();
	const Vector3f simulated_velocity(0.5f, 1.0f, -0.3f);
	_sensor_simulator._gps.setVelocity(simulated_velocity);
	const uint64_t dt_us = 1e5;
	_sensor_simulator._gps.stepHorizontalPositionByMeters(Vector2f(simulated_velocity) * dt_us * 1e-6);
	_sensor_simulator._gps.stepHeightByMeters(simulated_velocity(2) * dt_us * 1e-6f);

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.runMicroseconds(dt_us);

	// THEN: a reset to GPS velocity should be done
	const Vector3f estimated_velocity = _ekf->getVelocity();
	EXPECT_NEAR(estimated_velocity(0), simulated_velocity(0), 1e-3f);
	EXPECT_NEAR(estimated_velocity(1), simulated_velocity(1), 1e-3f);
	EXPECT_NEAR(estimated_velocity(2), simulated_velocity(2), 1e-3f);

	// AND: the reset in velocity should be saved correctly
	reset_logging_checker.capturePostResetState();
	EXPECT_TRUE(reset_logging_checker.isHorizontalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isVerticalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isVelocityDeltaLoggedCorrectly(1e-2f));
}

TEST_F(EkfGpsTest, resetToGpsPosition)
{
	// GIVEN:EKF that fuses GPS
	// and has gps checks already passed
	const Vector3f previous_position = _ekf->getPosition();

	// WHEN: stopping GPS fusion
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11);

	// AND: simulate jump in position
	_sensor_simulator.startGps();
	const Vector3f simulated_position_change(2.0f, -1.0f, 0.f);
	_sensor_simulator._gps.stepHorizontalPositionByMeters(
		Vector2f(simulated_position_change));
	_sensor_simulator.runMicroseconds(1e5);

	// THEN: a reset to the new GPS position should be done
	const Vector3f estimated_position = _ekf->getPosition();
	EXPECT_TRUE(isEqual(estimated_position,
			    previous_position + simulated_position_change, 1e-2f));
}

TEST_F(EkfGpsTest, gpsHgtToBaroFallback)
{
	// GIVEN: EKF that fuses GPS and flow, and in GPS height mode
	_sensor_simulator._flow.setData(_sensor_simulator._flow.dataAtRest());
	_ekf_wrapper.enableFlowFusion();
	_sensor_simulator.startFlow();

	_ekf_wrapper.enableGpsHeightFusion();

	_sensor_simulator.runSeconds(1);
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingFlowFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());

	// WHEN: stopping GPS fusion
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11);

	// THEN: the height source should automatically change to baro
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
}

TEST_F(EkfGpsTest, altitudeDrift)
{
	// GIVEN: a drifting GNSS altitude
	const float dt = 0.2f;
	const float height_rate = 0.15f;
	const float duration = 80.f;

	// WHEN: running on ground
	for (int i = 0; i < (duration / dt); i++) {
		_sensor_simulator._gps.stepHeightByMeters(height_rate * dt);
		_sensor_simulator.runSeconds(dt);
	}

	float baro_innov;
	_ekf->getBaroHgtInnov(baro_innov);
	BiasEstimator::status status = _ekf->getBaroBiasEstimatorStatus();

	printf("baro innov = %f\n", (double)baro_innov);
	printf("bias: %f, innov bias = %f\n", (double)status.bias, (double)status.innov);

	// THEN: the baro and local position should follow it
	EXPECT_LT(fabsf(baro_innov), 0.1f);
}
