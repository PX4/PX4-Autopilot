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
		_sensor_simulator.runSeconds(2);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}
};


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
	Vector3f pos_old = _ekf_wrapper.getPosition();
	Vector3f vel_old = _ekf_wrapper.getVelocity();
	Vector3f accel_bias_old = _ekf_wrapper.getAccelBias();
	_sensor_simulator._gps.stepHorizontalPositionByMeters(Vector2f{10.0f, 0.0f});
	_sensor_simulator.runSeconds(2);

	// THEN: The estimate should not change much in the short run
	//       and GPS fusion should be stopped after a while.
	Vector3f pos_new = _ekf_wrapper.getPosition();
	Vector3f vel_new = _ekf_wrapper.getVelocity();
	Vector3f accel_bias_new = _ekf_wrapper.getAccelBias();
	EXPECT_TRUE(matrix::isEqual(pos_new, pos_old, 0.01f));
	EXPECT_TRUE(matrix::isEqual(vel_new, vel_old, 0.01f));
	EXPECT_TRUE(matrix::isEqual(accel_bias_new, accel_bias_old, 0.01f));

	_sensor_simulator.runSeconds(10);
	pos_new = _ekf_wrapper.getPosition();
	vel_new = _ekf_wrapper.getVelocity();
	accel_bias_new = _ekf_wrapper.getAccelBias();
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
	_sensor_simulator.startFlow();
	_sensor_simulator.runSeconds(4);

	// THEN: EKF should intend to fuse flow measurements
	EXPECT_FALSE(_ekf_wrapper.isIntendingFlowFusion());
	// THEN: Local and global position should not be valid
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	// WHEN: Flow data is not send and we enable flow fusion
	_sensor_simulator.stopFlow();
	_sensor_simulator.runSeconds(3); // TODO: without this line tests fail
	// probably there are still values in the buffer.
	_ekf_wrapper.enableFlowFusion();
	_sensor_simulator.runSeconds(3);

	// THEN: EKF should not intend to fuse flow
	EXPECT_FALSE(_ekf_wrapper.isIntendingFlowFusion());
	// THEN: Local and global position should not be valid
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	// WHEN: Flow data is  send and we enable flow fusion
	_sensor_simulator.startFlow();
	_ekf_wrapper.enableFlowFusion();
	_sensor_simulator.runSeconds(10);

	// THEN: EKF should intend to fuse flow
	EXPECT_TRUE(_ekf_wrapper.isIntendingFlowFusion());
	// THEN: Local and global position should be valid
	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());
}
