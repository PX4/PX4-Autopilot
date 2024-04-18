/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * Test the flow fusion
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"


class EkfFlowTest : public ::testing::Test
{
public:

	EkfFlowTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		const float max_flow_rate = 5.f;
		const float min_ground_distance = 0.f;
		const float max_ground_distance = 50.f;
		_ekf->set_optical_flow_limits(max_flow_rate, min_ground_distance, max_ground_distance);

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

	void startRangeFinderFusion(float distance);
	void startZeroFlowFusion();
	void setFlowFromHorizontalVelocityAndDistance(flowSample &flow_sample, const Vector2f &simulated_horz_velocity,
			float estimated_distance_to_ground);
};

void EkfFlowTest::startRangeFinderFusion(float distance)
{
	_sensor_simulator._rng.setData(distance, 100);
	_sensor_simulator._rng.setLimits(0.1f, 9.f);
	_sensor_simulator.startRangeFinder();
}

void EkfFlowTest::startZeroFlowFusion()
{
	// Start fusing zero flow data
	_sensor_simulator._flow.setData(_sensor_simulator._flow.dataAtRest());
	_ekf_wrapper.enableFlowFusion();
	_sensor_simulator.startFlow();
}

void EkfFlowTest::setFlowFromHorizontalVelocityAndDistance(flowSample &flow_sample,
		const Vector2f &simulated_horz_velocity, float estimated_distance_to_ground)
{
	flow_sample.flow_rate =
		Vector2f(simulated_horz_velocity(1) / estimated_distance_to_ground,
			 -simulated_horz_velocity(0) / estimated_distance_to_ground);
}

TEST_F(EkfFlowTest, resetToFlowVelocityInAir)
{
	ResetLoggingChecker reset_logging_checker(_ekf);

	// WHEN: simulate being 5m above ground
	const float simulated_distance_to_ground = 5.f;
	_sensor_simulator._trajectory[2].setCurrentPosition(-simulated_distance_to_ground);
	startRangeFinderFusion(simulated_distance_to_ground);
	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);

	_sensor_simulator.runSeconds(5.f);

	const float estimated_distance_to_ground = _ekf->getTerrainVertPos();
	EXPECT_FLOAT_EQ(estimated_distance_to_ground, simulated_distance_to_ground);

	reset_logging_checker.capturePreResetState();

	// WHEN: start fusing flow data
	const Vector3f simulated_velocity(0.5f, -0.2f, 0.f);
	_sensor_simulator._trajectory[0].setCurrentVelocity(simulated_velocity(0));
	_sensor_simulator._trajectory[1].setCurrentVelocity(simulated_velocity(1));
	_sensor_simulator.setTrajectoryTargetVelocity(simulated_velocity);
	_ekf_wrapper.enableFlowFusion();
	_sensor_simulator.startFlow();

	// Let it reset but not fuse more measurements. We actually need to send 2
	// samples to get a reset because the first one cannot be used as the gyro
	// compensation needs to be accumulated between two samples.
	_sensor_simulator.runTrajectorySeconds(0.14);

	// THEN: estimated velocity should match simulated velocity
	const Vector3f estimated_velocity = _ekf->getVelocity();
	estimated_velocity.print();
	simulated_velocity.print();
	EXPECT_TRUE(isEqual(estimated_velocity, simulated_velocity))
			<< "estimated vel = " << estimated_velocity(0) << ", "
			<< estimated_velocity(1);

	// AND: the reset in velocity should be saved correctly
	reset_logging_checker.capturePostResetState();
	EXPECT_TRUE(reset_logging_checker.isHorizontalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isVerticalVelocityResetCounterIncreasedBy(0));
	EXPECT_TRUE(reset_logging_checker.isVelocityDeltaLoggedCorrectly(1e-9f));
}

TEST_F(EkfFlowTest, resetToFlowVelocityOnGround)
{
	ResetLoggingChecker reset_logging_checker(_ekf);

	// WHEN: being on ground
	const float estimated_distance_to_ground = _ekf->getTerrainVertPos();
	EXPECT_LT(estimated_distance_to_ground, 0.3f);

	reset_logging_checker.capturePreResetState();

	// WHEN: start fusing flow data
	flowSample flow_sample = _sensor_simulator._flow.dataAtRest();
	flow_sample.quality = 0;
	_sensor_simulator._flow.setData(flow_sample);
	_ekf_wrapper.enableFlowFusion();
	_sensor_simulator.startFlow();
	_sensor_simulator.startRangeFinder();
	_sensor_simulator.runSeconds(1.0);

	// THEN: estimated velocity should match simulated velocity
	const Vector2f estimated_horz_velocity = Vector2f(_ekf->getVelocity());
	EXPECT_TRUE(isEqual(estimated_horz_velocity, Vector2f(0.f, 0.f)))
			<< estimated_horz_velocity(0) << ", " << estimated_horz_velocity(1);

	// AND: the reset in velocity should be saved correctly
	reset_logging_checker.capturePostResetState();
	EXPECT_TRUE(reset_logging_checker.isHorizontalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isVerticalVelocityResetCounterIncreasedBy(0));
	EXPECT_TRUE(reset_logging_checker.isVelocityDeltaLoggedCorrectly(1e-9f));
}

TEST_F(EkfFlowTest, inAirConvergence)
{
	// WHEN: simulate being 5m above ground
	const float simulated_distance_to_ground = 5.f;
	_sensor_simulator._trajectory[2].setCurrentPosition(-simulated_distance_to_ground);
	startRangeFinderFusion(simulated_distance_to_ground);

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);

	_sensor_simulator.runSeconds(5.f);

	// WHEN: start fusing flow data
	Vector3f simulated_velocity(0.5f, -0.2f, 0.f);
	_sensor_simulator._trajectory[0].setCurrentVelocity(simulated_velocity(0));
	_sensor_simulator._trajectory[1].setCurrentVelocity(simulated_velocity(1));
	_sensor_simulator.setTrajectoryTargetVelocity(simulated_velocity);
	_ekf_wrapper.enableFlowFusion();
	_sensor_simulator.startFlow();
	// Let it reset but not fuse more measurements. We actually need to send 2
	// samples to get a reset because the first one cannot be used as the gyro
	// compensation needs to be accumulated between two samples.
	_sensor_simulator.runTrajectorySeconds(0.14);

	// THEN: estimated velocity should match simulated velocity
	Vector3f estimated_velocity = _ekf->getVelocity();
	EXPECT_TRUE(isEqual(estimated_velocity, simulated_velocity))
			<< "estimated vel = " << estimated_velocity(0) << ", "
			<< estimated_velocity(1);

	// AND: when the velocity changes
	simulated_velocity = Vector3f(1.8f, -1.5f, -0.5f);
	_sensor_simulator.setTrajectoryTargetVelocity(simulated_velocity);
	_sensor_simulator.runTrajectorySeconds(_sensor_simulator._trajectory[0].getTotalTime());

	// THEN: estimated velocity should converge to the simulated velocity
	// This takes a bit of time because the data is inconsistent with IMU measurements
	estimated_velocity = _ekf->getVelocity();
	EXPECT_NEAR(estimated_velocity(0), simulated_velocity(0), 0.05f)
			<< "estimated vel = " << estimated_velocity(0);
	EXPECT_NEAR(estimated_velocity(1), simulated_velocity(1), 0.05f)
			<< estimated_velocity(1);
}

TEST_F(EkfFlowTest, yawMotionCorrectionWithAutopilotGyroData)
{
	// WHEN: fusing range finder and optical flow data in air
	const float simulated_distance_to_ground = 5.f;
	startRangeFinderFusion(simulated_distance_to_ground);
	startZeroFlowFusion();

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);

	_sensor_simulator.runSeconds(5.f);

	// AND WHEN: there is a pure yaw rotation
	const Vector3f body_rate(0.f, 0.f, 2.9f);
	const Vector3f flow_offset(0.15, -0.05f, 0.2f);
	_ekf_wrapper.setFlowOffset(flow_offset);

	const Vector2f simulated_horz_velocity(body_rate % flow_offset);
	flowSample flow_sample = _sensor_simulator._flow.dataAtRest();
	setFlowFromHorizontalVelocityAndDistance(flow_sample, simulated_horz_velocity, simulated_distance_to_ground);

	// use autopilot gyro data
	flow_sample.gyro_rate.setAll(NAN);

	_sensor_simulator._flow.setData(flow_sample);
	_sensor_simulator._imu.setGyroData(body_rate);
	_sensor_simulator.runSeconds(10.f);

	// THEN: the flow due to the yaw rotation and the offsets is canceled
	// and the velocity estimate stays 0
	// FIXME: the estimate isn't perfect 0 mainly because the mag simulated measurement isn't rotating
	const Vector2f estimated_horz_velocity = Vector2f(_ekf->getVelocity());
	EXPECT_NEAR(estimated_horz_velocity(0), 0.f, 0.02f)
			<< "estimated vel = " << estimated_horz_velocity(0);
	EXPECT_NEAR(estimated_horz_velocity(1), 0.f, 0.02f)
			<< "estimated vel = " << estimated_horz_velocity(1);
}

TEST_F(EkfFlowTest, yawMotionCorrectionWithFlowGyroData)
{
	// WHEN: fusing range finder and optical flow data in air
	const float simulated_distance_to_ground = 5.f;
	startRangeFinderFusion(simulated_distance_to_ground);
	startZeroFlowFusion();

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);

	_sensor_simulator.runSeconds(5.f);

	// AND WHEN: there is a pure yaw rotation
	const Vector3f body_rate(0.f, 0.f, 2.9f);
	const Vector3f flow_offset(-0.15, 0.05f, 0.2f);
	_ekf_wrapper.setFlowOffset(flow_offset);

	const Vector2f simulated_horz_velocity(body_rate % flow_offset);
	flowSample flow_sample = _sensor_simulator._flow.dataAtRest();
	setFlowFromHorizontalVelocityAndDistance(flow_sample, simulated_horz_velocity, simulated_distance_to_ground);

	// use flow sensor gyro data
	// for clarification of the sign, see definition of flowSample
	flow_sample.gyro_rate = -body_rate;

	_sensor_simulator._flow.setData(flow_sample);
	_sensor_simulator._imu.setGyroData(body_rate);
	_sensor_simulator.runSeconds(10.f);

	// THEN: the flow due to the yaw rotation and the offsets is canceled
	// and the velocity estimate stays 0
	// FIXME: the estimate isn't perfect 0 mainly because the mag simulated measurement isn't rotating
	const Vector2f estimated_horz_velocity = Vector2f(_ekf->getVelocity());
	EXPECT_NEAR(estimated_horz_velocity(0), 0.f, 0.02f)
			<< "estimated vel = " << estimated_horz_velocity(0);
	EXPECT_NEAR(estimated_horz_velocity(1), 0.f, 0.02f)
			<< "estimated vel = " << estimated_horz_velocity(1);
	_ekf->state().vector().print();
	_ekf->covariances().print();
}

TEST_F(EkfFlowTest, yawMotionNoMagFusion)
{
	// WHEN: fusing range finder and optical flow data in air
	const float simulated_distance_to_ground = 5.f;
	startRangeFinderFusion(simulated_distance_to_ground);
	startZeroFlowFusion();
	_ekf_wrapper.setMagFuseTypeNone();

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);

	_sensor_simulator.runSeconds(5.f);

	// AND WHEN: there is a pure yaw rotation
	const Vector3f body_rate(0.f, 0.f, 3.14159f);
	const Vector3f flow_offset(-0.15, 0.05f, 0.2f);
	_ekf_wrapper.setFlowOffset(flow_offset);

	const Vector2f simulated_horz_velocity(body_rate % flow_offset);
	flowSample flow_sample = _sensor_simulator._flow.dataAtRest();
	setFlowFromHorizontalVelocityAndDistance(flow_sample, simulated_horz_velocity, simulated_distance_to_ground);

	// use flow sensor gyro data
	// for clarification of the sign, see definition of flowSample
	flow_sample.gyro_rate = -body_rate;

	_sensor_simulator._flow.setData(flow_sample);
	_sensor_simulator._imu.setGyroData(body_rate);
	_sensor_simulator.runSeconds(10.f);

	// THEN: the flow due to the yaw rotation and the offsets is canceled
	// and the velocity estimate stays 0
	const Vector2f estimated_horz_velocity = Vector2f(_ekf->getVelocity());
	EXPECT_NEAR(estimated_horz_velocity(0), 0.f, 0.01f)
			<< "estimated vel = " << estimated_horz_velocity(0);
	EXPECT_NEAR(estimated_horz_velocity(1), 0.f, 0.01f)
			<< "estimated vel = " << estimated_horz_velocity(1);
	_ekf->state().vector().print();
	_ekf->covariances().print();
}
