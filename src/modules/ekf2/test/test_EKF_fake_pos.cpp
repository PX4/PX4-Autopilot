/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"


class EkfFakePosTest : public ::testing::Test
{
public:

	EkfFakePosTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf),
		_quat_sim(Eulerf(0.0f, 0.0f, math::radians(45.0f))) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;
	const Quatf _quat_sim;

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// run briefly to init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_sensor_simulator.simulateOrientation(_quat_sim);
		_sensor_simulator.runSeconds(7);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}
};

TEST_F(EkfFakePosTest, testValidFakePos)
{
	_ekf->set_vehicle_at_rest(false);
	_ekf->set_constant_pos(true);
	_sensor_simulator.runSeconds(1);

	EXPECT_EQ(1, (int) _ekf->control_status_flags().constant_pos);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().fake_pos);
	EXPECT_EQ(1, (int) _ekf->control_status_flags().valid_fake_pos);
}

TEST_F(EkfFakePosTest, testFakePosStopGnss)
{
	_ekf->set_vehicle_at_rest(false);
	_ekf->set_constant_pos(true);
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(12);

	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_EQ(1, (int) _ekf->control_status_flags().constant_pos);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().fake_pos);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().valid_fake_pos);
}

TEST_F(EkfFakePosTest, testValidFakePosValidLocalPos)
{
	_ekf->set_is_fixed_wing(true);
	_sensor_simulator.startAirspeedSensor();
	_sensor_simulator._airspeed.setData(-0.01f, 0.f); // airspeed close to 0
	_ekf_wrapper.enableBetaFusion();

	// WHEN: the vehicle is not as rest but is known to be at a constant position
	_ekf->set_vehicle_at_rest(false);
	_ekf->set_constant_pos(true);
	_sensor_simulator.runSeconds(1);

	// THEN: the valid fake position is fused
	EXPECT_EQ(1, (int) _ekf->control_status_flags().constant_pos);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().fake_pos);
	EXPECT_EQ(1, (int) _ekf->control_status_flags().valid_fake_pos);

	// AND: since airspeed is expected to provide wind-relative dead-reckoning after takeoff
	// the local position is considered valid
	_sensor_simulator.runSeconds(60);

	EXPECT_EQ(1, (int) _ekf->control_status_flags().valid_fake_pos);
	EXPECT_TRUE(_ekf->isLocalHorizontalPositionValid());
}
