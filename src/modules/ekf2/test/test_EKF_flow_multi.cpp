/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * Test the fusion of two optical flow sensors
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"


class EkfFlowMultiTest : public ::testing::Test
{
public:
	using Flow = sensor_simulator::sensor::Flow;

	EkfFlowMultiTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	static constexpr float kDistanceToGround = 5.f;

	void SetUp() override
	{
		for (uint8_t slot = 0; slot < 2; slot++) {
			_ekf->flowSource(slot).setLimits(5.f, 0.f, 50.f);
		}

		// run briefly to init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_sensor_simulator.runSeconds(7);
	}

	// hover above the ground with the range finder fusing, flow is the only horizontal aiding source
	void startHoverWithRangeFinder()
	{
		_sensor_simulator._trajectory[2].setCurrentPosition(-kDistanceToGround);
		_sensor_simulator._rng.setData(kDistanceToGround, 100);
		_sensor_simulator._rng.setLimits(0.1f, 9.f);
		_sensor_simulator.startRangeFinder();

		_ekf->set_in_air_status(true);
		_ekf->set_vehicle_at_rest(false);

		_sensor_simulator.runSeconds(5.f);
	}

	void startFlow(uint8_t slot)
	{
		Flow &flow = (slot == 0) ? _sensor_simulator._flow : _sensor_simulator._flow1;
		flow.setData(flow.dataAtRest());
		_ekf_wrapper.enableFlowFusion(slot);
		flow.start();
	}

	// flow measured by a sensor that disagrees with the vehicle motion (e.g. wrong scale or rotation)
	void setInconsistentFlow(uint8_t slot)
	{
		Flow &flow = (slot == 0) ? _sensor_simulator._flow : _sensor_simulator._flow1;
		flowSample flow_sample = flow.dataAtRest();
		flow_sample.flow_rate = Vector2f(1.f, 0.f);
		flow.setData(flow_sample);
	}
};

TEST_F(EkfFlowMultiTest, inconsistentSensorDoesNotResetVelocity)
{
	// GIVEN: two flow sensors fused while hovering
	startHoverWithRangeFinder();
	startFlow(0);
	startFlow(1);
	_sensor_simulator.runSeconds(3.f);

	ASSERT_TRUE(_ekf->control_status_flags().opt_flow);
	ASSERT_TRUE(_ekf->aid_src_optical_flow(0).fused);
	ASSERT_TRUE(_ekf->aid_src_optical_flow(1).fused);

	const uint8_t vel_reset_count = _ekf->get_velNE_reset_count();

	// WHEN: the second sensor starts to disagree with the vehicle motion
	setInconsistentFlow(1);
	_sensor_simulator.runSeconds(5.f);

	// THEN: the second sensor is rejected and never resets the velocity
	// while the first one keeps constraining the drift
	EXPECT_EQ(_ekf->get_velNE_reset_count(), vel_reset_count);
	EXPECT_LT(Vector2f(_ekf->getVelocity()).norm(), 0.1f);

	EXPECT_TRUE(_ekf->control_status_flags().opt_flow);
	EXPECT_TRUE(_ekf->aid_src_optical_flow(0).fused);
	EXPECT_FALSE(_ekf->aid_src_optical_flow(1).fused);
}

TEST_F(EkfFlowMultiTest, simultaneousStartResetsVelocityOnce)
{
	// GIVEN: no horizontal aiding, both sensors deliver their samples in the same updates
	startHoverWithRangeFinder();
	const uint8_t vel_reset_count = _ekf->get_velNE_reset_count();

	// WHEN: both sensors start
	startFlow(0);
	startFlow(1);
	_sensor_simulator.runSeconds(2.f);

	// THEN: one sensor resets the velocity, the other one starts fusing on top of it
	EXPECT_TRUE(_ekf->aid_src_optical_flow(0).fused);
	EXPECT_TRUE(_ekf->aid_src_optical_flow(1).fused);
	EXPECT_EQ(_ekf->get_velNE_reset_count(), vel_reset_count + 1);
}

TEST_F(EkfFlowMultiTest, sensorStartDoesNotResetTerrain)
{
	// GIVEN: one flow sensor estimating the terrain after the range finder stopped
	startHoverWithRangeFinder();
	startFlow(0);
	_sensor_simulator.runSeconds(3.f);

	_sensor_simulator.stopRangeFinder();
	_sensor_simulator.runSeconds(5.f);

	ASSERT_TRUE(_ekf->control_status_flags().opt_flow_terrain);
	ASSERT_FALSE(_ekf->control_status_flags().rng_terrain);
	ASSERT_NEAR(_ekf->getHagl(), kDistanceToGround, 0.2f);

	const uint8_t hagl_reset_count = _ekf->get_hagl_reset_count();

	// WHEN: a second sensor whose first measurements are rejected becomes available
	startFlow(1);
	setInconsistentFlow(1);
	_sensor_simulator.runSeconds(3.f);

	// THEN: the terrain estimate of the first sensor is kept
	EXPECT_EQ(_ekf->get_hagl_reset_count(), hagl_reset_count);
	EXPECT_NEAR(_ekf->getHagl(), kDistanceToGround, 0.2f);
	EXPECT_TRUE(_ekf->aid_src_optical_flow(0).fused);
	EXPECT_FALSE(_ekf->aid_src_optical_flow(1).fused);
}

TEST_F(EkfFlowMultiTest, sensorEnabledAtRuntimeIsFused)
{
	// GIVEN: two flow sensors publishing data, the second one disabled
	startHoverWithRangeFinder();
	startFlow(0);
	_ekf_wrapper.disableFlowFusion(1);
	_sensor_simulator._flow1.setData(_sensor_simulator._flow1.dataAtRest());
	_sensor_simulator.startFlow1();
	_sensor_simulator.runSeconds(3.f);

	EXPECT_TRUE(_ekf->aid_src_optical_flow(0).fused);
	EXPECT_EQ(_ekf->aid_src_optical_flow(1).timestamp_sample, 0u);

	// WHEN: the second sensor gets enabled in flight
	_ekf_wrapper.enableFlowFusion(1);
	_sensor_simulator.runSeconds(3.f);

	// THEN: it is fused without a reboot
	EXPECT_TRUE(_ekf->aid_src_optical_flow(1).fused);
	EXPECT_TRUE(_ekf->aid_src_optical_flow(0).fused);
}

TEST_F(EkfFlowMultiTest, bothSensorsFusedInMotion)
{
	// GIVEN: two flow sensors while hovering
	startHoverWithRangeFinder();
	startFlow(0);
	startFlow(1);
	_sensor_simulator.runSeconds(3.f);

	// WHEN: the vehicle moves horizontally
	const Vector3f simulated_velocity(0.8f, -0.5f, 0.f);
	_sensor_simulator.setTrajectoryTargetVelocity(simulated_velocity);
	_sensor_simulator.runTrajectorySeconds(_sensor_simulator._trajectory[0].getTotalTime() + 2.f);

	// THEN: both sensors keep being fused and the velocity estimate follows
	EXPECT_TRUE(_ekf->aid_src_optical_flow(0).fused);
	EXPECT_TRUE(_ekf->aid_src_optical_flow(1).fused);
	EXPECT_NEAR(_ekf->getVelocity()(0), simulated_velocity(0), 0.05f);
	EXPECT_NEAR(_ekf->getVelocity()(1), simulated_velocity(1), 0.05f);
}

TEST_F(EkfFlowMultiTest, handoverBetweenSensorRanges)
{
	// GIVEN: a short range (up to 3 m) and a long range sensor while hovering at 5 m
	_ekf->flowSource(0).setLimits(5.f, 0.f, 3.f);
	startHoverWithRangeFinder();
	startFlow(0);
	startFlow(1);
	_sensor_simulator.runSeconds(3.f);

	// THEN: only the long range sensor is fused
	EXPECT_FALSE(_ekf->aid_src_optical_flow(0).fused);
	EXPECT_TRUE(_ekf->aid_src_optical_flow(1).fused);
	EXPECT_TRUE(_ekf->control_status_flags().opt_flow);

	// WHEN: descending into the range of the short range sensor
	_sensor_simulator.setTrajectoryTargetVelocity(Vector3f(0.f, 0.f, 1.f));
	_sensor_simulator.runTrajectorySeconds(3.f);
	_sensor_simulator.setTrajectoryTargetVelocity(Vector3f(0.f, 0.f, 0.f));
	_sensor_simulator.runTrajectorySeconds(3.f);

	// THEN: it starts being fused as well
	ASSERT_LT(_ekf->getHagl(), 2.5f);
	EXPECT_TRUE(_ekf->aid_src_optical_flow(0).fused);
	EXPECT_TRUE(_ekf->aid_src_optical_flow(1).fused);
}

TEST_F(EkfFlowMultiTest, disablingOneSensorKeepsTheOther)
{
	// GIVEN: two flow sensors fused while hovering
	startHoverWithRangeFinder();
	startFlow(0);
	startFlow(1);
	_sensor_simulator.runSeconds(3.f);

	// WHEN: the second sensor is disabled
	_ekf_wrapper.disableFlowFusion(1);
	_sensor_simulator.runSeconds(3.f);

	// THEN: only the first one is still fused
	EXPECT_TRUE(_ekf->control_status_flags().opt_flow);
	EXPECT_TRUE(_ekf->aid_src_optical_flow(0).fused);
	EXPECT_GT(_ekf->time_delayed_us() - _ekf->aid_src_optical_flow(1).time_last_fuse, 2'000'000u);
}

TEST_F(EkfFlowMultiTest, fusionControlStopsAllSensors)
{
	// GIVEN: two flow sensors fused while hovering
	startHoverWithRangeFinder();
	startFlow(0);
	startFlow(1);
	_sensor_simulator.runSeconds(3.f);
	ASSERT_TRUE(_ekf->control_status_flags().opt_flow);

	// WHEN: optical flow fusion is disabled at runtime (EKF2_SENS_EN / FUSION_SOURCE_OF)
	_ekf->getFusionControlHandle()->of.enabled = false;
	_sensor_simulator.runSeconds(1.f);

	// THEN: no flow sensor is fused anymore
	EXPECT_FALSE(_ekf->control_status_flags().opt_flow);
	EXPECT_FALSE(_ekf->control_status_flags().opt_flow_terrain);

	// AND WHEN: enabled again
	_ekf->getFusionControlHandle()->of.enabled = true;
	_sensor_simulator.runSeconds(3.f);

	// THEN: flow fusion resumes
	EXPECT_TRUE(_ekf->control_status_flags().opt_flow);
}
