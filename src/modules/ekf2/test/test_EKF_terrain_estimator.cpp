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

/**
 * Test the terrain estimator
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"

class EkfTerrainTest : public ::testing::Test
{
public:

	EkfTerrainTest(): ::testing::Test(),
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
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}

	void runFlowAndRngScenario(const float rng_height, const float flow_height)
	{
		_sensor_simulator.startGps();

		_ekf->set_min_required_gps_health_time(1e6);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_ekf_wrapper.enableGpsFusion();
		_sensor_simulator.runSeconds(1.5); // Run to pass the GPS checks
		EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());

		const Vector3f simulated_velocity(0.5f, -1.0f, 0.f);

		// Configure GPS simulator data
		_sensor_simulator._gps.setVelocity(simulated_velocity);
		_sensor_simulator._gps.setPositionRateNED(simulated_velocity);

		// Configure range finder simulator data
		_sensor_simulator._rng.setData(rng_height, 100);
		_sensor_simulator._rng.setLimits(0.1f, 20.f);
		_sensor_simulator.startRangeFinder();

		// Configure optical flow simulator data
		flowSample flow_sample = _sensor_simulator._flow.dataAtRest();
		flow_sample.flow_rate =
			Vector2f(simulated_velocity(1) / flow_height,
				 -simulated_velocity(0) / flow_height);
		_sensor_simulator._flow.setData(flow_sample);
		const float max_flow_rate = 5.f;
		const float min_ground_distance = 0.f;
		const float max_ground_distance = 50.f;
		_ekf->set_optical_flow_limits(max_flow_rate, min_ground_distance, max_ground_distance);
		_sensor_simulator.startFlow();

		_ekf->set_in_air_status(true);
		_ekf->set_vehicle_at_rest(false);

		_sensor_simulator.runSeconds(10);
	}
};

TEST_F(EkfTerrainTest, setFlowAndRangeTerrainFusion)
{
	// WHEN: simulate being 5m above ground
	const float simulated_distance_to_ground = 1.f;
	runFlowAndRngScenario(simulated_distance_to_ground, simulated_distance_to_ground);

	// THEN: By default, both rng and flow aiding are active
	EXPECT_TRUE(_ekf_wrapper.isIntendingTerrainRngFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingTerrainFlowFusion());
	const float estimated_distance_to_ground = _ekf->getTerrainVertPos();
	EXPECT_NEAR(estimated_distance_to_ground, simulated_distance_to_ground, 0.01);

	// WHEN: rng fusion is disabled
	_ekf_wrapper.disableTerrainRngFusion();
	_sensor_simulator.runSeconds(5.1);

	// THEN: rng fusion should be disabled and flow fusion should take over
	EXPECT_FALSE(_ekf_wrapper.isIntendingTerrainRngFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingTerrainFlowFusion());

	// WHEN: flow is now diabled
	_ekf_wrapper.disableTerrainFlowFusion();
	_sensor_simulator.runSeconds(0.2);

	// THEN: flow is now also disabled
	EXPECT_FALSE(_ekf_wrapper.isIntendingTerrainRngFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingTerrainFlowFusion());
}

TEST_F(EkfTerrainTest, testFlowForTerrainFusion)
{
	// GIVEN: flow for terrain enabled but not range finder
	_ekf_wrapper.enableTerrainFlowFusion();
	_ekf_wrapper.disableTerrainRngFusion();

	// WHEN: the sensors do not agree
	const float rng_height = 1.f;
	const float flow_height = 5.f;
	runFlowAndRngScenario(rng_height, flow_height);

	// THEN: the estimator should use flow for terrain and the estimated terrain height
	// should converge to the simulated height
	EXPECT_FALSE(_ekf_wrapper.isIntendingTerrainRngFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingTerrainFlowFusion());

	const float estimated_distance_to_ground = _ekf->getTerrainVertPos();
	EXPECT_NEAR(estimated_distance_to_ground, flow_height, 0.5f);
}

TEST_F(EkfTerrainTest, testRngForTerrainFusion)
{
	// GIVEN: rng for terrain but not flow
	_ekf_wrapper.disableTerrainFlowFusion();
	_ekf_wrapper.enableTerrainRngFusion();

	// WHEN: the sensors do not agree
	const float rng_height = 1.f;
	const float flow_height = 5.f;
	runFlowAndRngScenario(rng_height, flow_height);

	// THEN: the estimator should use rng for terrain and the estimated terrain height
	// should converge to the simulated height
	EXPECT_TRUE(_ekf_wrapper.isIntendingTerrainRngFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingTerrainFlowFusion());

	const float estimated_distance_to_ground = _ekf->getTerrainVertPos();
	EXPECT_NEAR(estimated_distance_to_ground, rng_height, 0.01f);
}

TEST_F(EkfTerrainTest, testHeightReset)
{
	// GIVEN: rng for terrain but not flow
	_ekf_wrapper.disableTerrainFlowFusion();
	_ekf_wrapper.enableTerrainRngFusion();

	const float rng_height = 1.f;
	const float flow_height = 1.f;
	runFlowAndRngScenario(rng_height, flow_height);

	const float estimated_distance_to_ground = _ekf->getTerrainVertPos() - _ekf->getPosition()(2);

	ResetLoggingChecker reset_logging_checker(_ekf);
	reset_logging_checker.capturePreResetState();

	// WHEN: the baro height is suddenly changed to trigger a height reset
	const float new_baro_height = _sensor_simulator._baro.getData() + 50.f;
	_sensor_simulator._baro.setData(new_baro_height);
	_sensor_simulator.stopGps(); // prevent from switching to GNSS height
	_sensor_simulator.runSeconds(6);

	// THEN: a height reset occured and the estimated distance to the ground remains constant
	reset_logging_checker.capturePostResetState();
	EXPECT_TRUE(reset_logging_checker.isVerticalPositionResetCounterIncreasedBy(1));
	EXPECT_NEAR(estimated_distance_to_ground, _ekf->getTerrainVertPos() - _ekf->getPosition()(2), 1e-3f);
}
