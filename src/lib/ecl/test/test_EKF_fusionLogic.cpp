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
 * Test the fusion start and stop logic
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"


class EkfFusionLogicTest : public ::testing::Test {
 public:

	EkfFusionLogicTest(): ::testing::Test(),
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
		_sensor_simulator.runSeconds(7);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}
};


TEST_F(EkfFusionLogicTest, doNoFusion)
{
	// GIVEN: a tilt and heading aligned filter
	// WHEN: having no aiding source
	// THEN: EKF should not have a valid position estimate
	EXPECT_FALSE(_ekf->local_position_is_valid());

	_sensor_simulator.runSeconds(4);

	// THEN: Local and global position should not be valid
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());
}

TEST_F(EkfFusionLogicTest, doGpsFusion)
{
	// GIVEN: a tilt and heading aligned filter
	// WHEN: we enable GPS fusion and we send good quality gps data for 11s
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(11);

	// THEN: EKF should intend to fuse GPS
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	// THEN: Local and global position should be valid
	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_TRUE(_ekf->global_position_is_valid());

	// WHEN: GPS data is not send for 11s
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11);

	// THEN: EKF should stop to intend to fuse GPS
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	// WHEN: GPS data is send again for 11s
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(11);

	// THEN: EKF should to intend to fuse GPS
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_TRUE(_ekf->global_position_is_valid());

	// // WHEN: clients decides to stop GPS fusion
	// _ekf_wrapper.disableGpsFusion();
	// // THEN: EKF should stop to intend to fuse GPS immediately
	// _sensor_simulator.runMicroseconds(1000);
	// EXPECT_FALSE(_ekf_wrapper.isIntendingGpsFusion());
	// THIS is not happening at the moment
}

TEST_F(EkfFusionLogicTest, rejectGpsSignalJump)
{
	// GIVEN: a tilt and heading aligned filter
	// WHEN: we enable GPS fusion and we send good quality gps data for 11s
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(15);

	// THEN: EKF should intend to fuse GPS
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());

	// WHEN: Having a big horizontal position Gps jump coming from the Gps Receiver
	const Vector3f pos_old = _ekf->getPosition();
	const Vector3f vel_old = _ekf->getVelocity();
	const Vector3f accel_bias_old = _ekf->getAccelBias();
	_sensor_simulator._gps.stepHorizontalPositionByMeters(Vector2f{20.0f, 0.0f});
	_sensor_simulator.runSeconds(2);

	// THEN: The estimate should not change much in the short run
	//       and GPS fusion should be stopped after a while.
	Vector3f pos_new = _ekf->getPosition();
	Vector3f vel_new = _ekf->getVelocity();
	Vector3f accel_bias_new = _ekf->getAccelBias();
	EXPECT_TRUE(matrix::isEqual(pos_new, pos_old, 0.01f));
	EXPECT_TRUE(matrix::isEqual(vel_new, vel_old, 0.01f));
	EXPECT_TRUE(matrix::isEqual(accel_bias_new, accel_bias_old, 0.01f));

	_sensor_simulator.runSeconds(10);
	pos_new = _ekf->getPosition();
	vel_new = _ekf->getVelocity();
	accel_bias_new = _ekf->getAccelBias();
	EXPECT_TRUE(matrix::isEqual(pos_new, pos_old, 0.01f));
	EXPECT_TRUE(matrix::isEqual(vel_new, vel_old, 0.01f));
	EXPECT_TRUE(matrix::isEqual(accel_bias_new, accel_bias_old, 0.01f));
	// EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion()); // What do we expect here?
}

TEST_F(EkfFusionLogicTest, doFlowFusion)
{
	// GIVEN: a tilt and heading aligned filter
	// WHEN: sending flow data without having the flow fusion enabled
	//       flow measurement fusion should not be intended.
	const float max_flow_rate = 5.f;
	const float min_ground_distance = 0.f;
	const float max_ground_distance = 50.f;
	_ekf->set_optical_flow_limits(max_flow_rate, min_ground_distance, max_ground_distance);
	_sensor_simulator.startFlow();
	_sensor_simulator.runSeconds(4);

	// THEN: EKF should not intend to fuse flow measurements
	EXPECT_FALSE(_ekf_wrapper.isIntendingFlowFusion());
	// THEN: Local and global position should not be valid
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	// WHEN: Flow data is not send and we enable flow fusion
	_sensor_simulator.stopFlow();
	_sensor_simulator.runSeconds(1); // empty buffer
	_ekf_wrapper.enableFlowFusion();
	_sensor_simulator.runSeconds(3);

	// THEN: EKF should not intend to fuse flow
	EXPECT_FALSE(_ekf_wrapper.isIntendingFlowFusion());
	// THEN: Local and global position should not be valid
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	// WHEN: Flow data is sent and we enable flow fusion
	_sensor_simulator.startFlow();
	_ekf_wrapper.enableFlowFusion();
	_sensor_simulator.runSeconds(10);

	// THEN: EKF should intend to fuse flow
	EXPECT_TRUE(_ekf_wrapper.isIntendingFlowFusion());
	// THEN: Local and global position should be valid
	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	// WHEN: Stop sending flow data
	_sensor_simulator.stopFlow();
	_sensor_simulator.runSeconds(11);

	// THEN: EKF should not intend to fuse flow measurements
	EXPECT_FALSE(_ekf_wrapper.isIntendingFlowFusion());
	// THEN: Local and global position should not be valid
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());
}

TEST_F(EkfFusionLogicTest, doVisionPositionFusion)
{
	// WHEN: allow vision position to be fused and we send vision data
	_ekf_wrapper.enableExternalVisionPositionFusion();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(4);

	// THEN: EKF should intend to fuse vision position estimate
	//       and we have a valid local position estimate
	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());
	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	// WHEN: stop sending vision data
	_sensor_simulator.stopExternalVision();
	_sensor_simulator.runSeconds(7);

	// THEN: EKF should stop to intend to fuse vision position estimate
	//       and EKF should not have a valid local position estimate anymore
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());
}

TEST_F(EkfFusionLogicTest, doVisionVelocityFusion)
{
	// WHEN: allow vision position to be fused and we send vision data
	_ekf_wrapper.enableExternalVisionVelocityFusion();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(4);

	// THEN: EKF should intend to fuse vision position estimate
	//       and we have a valid local position estimate
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());
	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	// WHEN: stop sending vision data
	_sensor_simulator.stopExternalVision();
	_sensor_simulator.runSeconds(7);

	// THEN: EKF should stop to intend to fuse vision position estimate
	//       and EKF should not have a valid local position estimate anymore
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());
}

TEST_F(EkfFusionLogicTest, doVisionHeadingFusion)
{
	// WHEN: allow vision position to be fused and we send vision data
	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_ekf_wrapper.enableExternalVisionHeadingFusion();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(4);

	// THEN: EKF should intend to fuse vision heading estimates
	//       and we should not have a valid local position estimate
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());
	// THEN: Yaw state should be reset to vision
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);

	// WHEN: stop sending vision data
	_sensor_simulator.stopExternalVision();
	_sensor_simulator.runSeconds(7.1);

	// THEN: EKF should stop to intend to fuse vision position estimate
	//       and EKF should not have a valid local position estimate anymore
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());
	// THEN: Yaw state shoud be reset to mag 
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 2);
}

TEST_F(EkfFusionLogicTest, doBaroHeightFusion)
{
	// GIVEN: EKF that receives baro data

	// THEN: EKF should intend to fuse baro by default
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());

	// WHEN: stop sending baro data
	_sensor_simulator.stopBaro();
	_sensor_simulator.runSeconds(6);

	// THEN: EKF should stop to intend to use baro hgt
	// TODO: We have no fall back in balce
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion()); // TODO: Needs to change
}


TEST_F(EkfFusionLogicTest, doGpsHeightFusion)
{
	// WHEN: commanding GPS height and sending GPS data
	_ekf_wrapper.setGpsHeight();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(11);

	// THEN: EKF should intend to fuse gps height
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeightFusion());

	// WHEN: stop sending gps data
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11); // TODO: We have to wait way too long

	// THEN: EKF should stop to intend to use gps height
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsHeightFusion());
}


TEST_F(EkfFusionLogicTest, doRangeHeightFusion)
{
	// WHEN: commanding range height and sending range data
	_ekf_wrapper.setRangeHeight();
	_sensor_simulator.startRangeFinder();
	_sensor_simulator.runSeconds(2.5f);
	// THEN: EKF should intend to fuse range height
	EXPECT_TRUE(_ekf_wrapper.isIntendingRangeHeightFusion());

	const float dt = 8e-3f;
	for (int i = 0; i < 5; i++) {
		_sensor_simulator.runSeconds(dt);
		// THEN: EKF should intend to fuse range height, even if
		// there is no new data at each EKF iteration (EKF rate > sensor rate)
		EXPECT_TRUE(_ekf_wrapper.isIntendingRangeHeightFusion());
	}

	// WHEN: stop sending range data
	_sensor_simulator.stopRangeFinder();
	_sensor_simulator.runSeconds(5.1);

	// THEN: EKF should stop to intend to use range height
	// and fall back to baro height
	EXPECT_FALSE(_ekf_wrapper.isIntendingRangeHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
}

TEST_F(EkfFusionLogicTest, doVisionHeightFusion)
{
	// WHEN: commanding vision height and sending vision data
	_ekf_wrapper.setVisionHeight();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(2);

	// THEN: EKF should intend to fuse vision height
	EXPECT_TRUE(_ekf_wrapper.isIntendingVisionHeightFusion());

	// WHEN: stop sending vision data
	_sensor_simulator.stopExternalVision();
	_sensor_simulator.runSeconds(12);

	// THEN: EKF should stop to intend to use vision height

	EXPECT_FALSE(_ekf_wrapper.isIntendingVisionHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
}
